/** @file

  Detect the board variant and call LibPcdSetSku() to select the
  correct PCD SKU before any SKU-sensitive driver runs.

  Identity source priority:
    1. TLV EEPROM product_name via raw MMIO I2C (authoritative)
    2. DTB root "model" property from HOB (fallback)

  SKU mapping (substring match on product_name or model):
    contains "com260"           ->  SKU 1 (COM260)
    contains "fml13v05"         ->  SKU 2 (FML13V05)
    anything else               ->  SKU 0 (DEFAULT)

  Copyright (c) 2025-2026, Spacemit Corporation

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/FdtLib.h>
#include <Library/HobLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>

#include <Guid/Fdt.h>
#include <Guid/FdtHob.h>

#define SKU_ID_DEFAULT  0
#define SKU_ID_COM260   1
#define SKU_ID_FML13V05 2

//
// SpacemiT I2C controller registers (DesignWare-compatible).
//
#define ICR_OFFSET   0x00
#define ISR_OFFSET   0x04
#define ISAR_OFFSET  0x08
#define IDBR_OFFSET  0x0C

#define ICR_START  0x01
#define ICR_STOP   0x02
#define ICR_ACKNAK 0x04
#define ICR_TB     0x08
#define ICR_SCLE   BIT13
#define ICR_IUE    BIT14
#define ICR_GCD    BIT21
#define ICR_BEIE   BIT22
#define ICR_IRFIE  BIT20
#define ICR_ITEIE  BIT19

#define ISR_ITE    BIT19
#define ISR_IRF    BIT20
#define ISR_BED    BIT22

#define I2C2_BASE       0xD4012000
#define EEPROM_ADDR     0x50
#define I2C_TIMEOUT_US  50000

//
// TLV EEPROM format.
//
#define TLV_HEADER_SIZE         11
#define TLV_SIGNATURE           "TlvInfo"
#define TLV_CODE_PRODUCT_NAME   0x21

/**
  Wait for an I2C status bit with timeout.

  @param[in]  Base     I2C controller MMIO base.
  @param[in]  BitMask  ISR bit(s) to wait for.

  @retval TRUE   Bit set within timeout.
  @retval FALSE  Timeout.
**/
STATIC
BOOLEAN
I2cWaitStatus (
  IN UINTN   Base,
  IN UINT32  BitMask
  )
{
  UINTN  Elapsed;

  for (Elapsed = 0; Elapsed < I2C_TIMEOUT_US; Elapsed += 10) {
    if ((MmioRead32 (Base + ISR_OFFSET) & BitMask) != 0) {
      return TRUE;
    }

    MicroSecondDelay (10);
  }

  return FALSE;
}

/**
  Read a single byte from the EEPROM at a given register address.

  Uses the standard I2C random-read sequence:
    START -> slave+W -> regaddr -> RESTART -> slave+R -> read -> STOP

  @param[in]  Base       I2C controller MMIO base.
  @param[in]  SlaveAddr  7-bit I2C slave address.
  @param[in]  RegAddr    EEPROM byte address to read.
  @param[out] Data       Byte read from EEPROM.

  @retval EFI_SUCCESS    Byte read successfully.
  @retval EFI_TIMEOUT    I2C bus did not respond.
**/
STATIC
EFI_STATUS
I2cReadByte (
  IN  UINTN   Base,
  IN  UINT8   SlaveAddr,
  IN  UINT8   RegAddr,
  OUT UINT8   *Data
  )
{
  UINT32  Icr;

  Icr = ICR_IUE | ICR_SCLE | ICR_GCD | ICR_BEIE | ICR_ITEIE | ICR_IRFIE;

  MmioWrite32 (Base + ISR_OFFSET, 0xFFFFFFFF);
  MmioWrite32 (Base + ICR_OFFSET, Icr);

  //
  // Write phase: send slave address + register address.
  //
  MmioWrite32 (Base + IDBR_OFFSET, (UINT32)(SlaveAddr << 1));
  MmioWrite32 (Base + ICR_OFFSET, Icr | ICR_START | ICR_TB);

  if (!I2cWaitStatus (Base, ISR_ITE)) {
    return EFI_TIMEOUT;
  }

  MmioWrite32 (Base + ISR_OFFSET, ISR_ITE);

  MmioWrite32 (Base + IDBR_OFFSET, (UINT32)RegAddr);
  MmioWrite32 (Base + ICR_OFFSET, Icr | ICR_TB);

  if (!I2cWaitStatus (Base, ISR_ITE)) {
    return EFI_TIMEOUT;
  }

  MmioWrite32 (Base + ISR_OFFSET, ISR_ITE);

  //
  // Read phase: restart, send slave address with read bit, read one byte.
  //
  MmioWrite32 (Base + IDBR_OFFSET, (UINT32)((SlaveAddr << 1) | 1));
  MmioWrite32 (Base + ICR_OFFSET, Icr | ICR_START | ICR_TB);

  if (!I2cWaitStatus (Base, ISR_ITE)) {
    return EFI_TIMEOUT;
  }

  MmioWrite32 (Base + ISR_OFFSET, ISR_ITE);

  MmioWrite32 (Base + ICR_OFFSET, Icr | ICR_STOP | ICR_ACKNAK | ICR_TB);

  if (!I2cWaitStatus (Base, ISR_IRF)) {
    return EFI_TIMEOUT;
  }

  *Data = (UINT8)MmioRead32 (Base + IDBR_OFFSET);
  MmioWrite32 (Base + ISR_OFFSET, ISR_IRF);

  return EFI_SUCCESS;
}

/**
  Read multiple bytes from the EEPROM.

  @param[in]  Base       I2C controller MMIO base.
  @param[in]  SlaveAddr  7-bit I2C slave address.
  @param[in]  RegAddr    Starting EEPROM byte address.
  @param[out] Buffer     Output buffer.
  @param[in]  Length     Number of bytes to read.

  @retval EFI_SUCCESS    All bytes read.
  @retval EFI_TIMEOUT    I2C bus did not respond.
**/
STATIC
EFI_STATUS
I2cReadBytes (
  IN  UINTN   Base,
  IN  UINT8   SlaveAddr,
  IN  UINT8   RegAddr,
  OUT UINT8   *Buffer,
  IN  UINTN   Length
  )
{
  EFI_STATUS  Status;
  UINTN       Index;

  for (Index = 0; Index < Length; Index++) {
    Status = I2cReadByte (Base, SlaveAddr, (UINT8)(RegAddr + Index), &Buffer[Index]);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  return EFI_SUCCESS;
}

/**
  Read the product_name from the TLV EEPROM via raw I2C.

  @param[out] ProductName  Null-terminated product name string.
  @param[in]  BufferSize   Size of ProductName buffer.

  @retval EFI_SUCCESS      Product name read successfully.
  @retval EFI_NOT_FOUND    TLV header invalid or product_name not found.
  @retval EFI_TIMEOUT      I2C communication failed.
**/
STATIC
EFI_STATUS
ReadProductNameFromEeprom (
  OUT CHAR8  *ProductName,
  IN  UINTN  BufferSize
  )
{
  EFI_STATUS  Status;
  UINT8       Header[TLV_HEADER_SIZE];
  UINT16      TotalLen;
  UINT8       Tid;
  UINT8       Tlen;
  UINT8       Offset;
  UINT8       EndOffset;

  //
  // Ensure the I2C MMIO region is mapped — the MMU is active but
  // the GCD memory map may not include this range yet.
  //
  gDS->AddMemorySpace (
         EfiGcdMemoryTypeMemoryMappedIo,
         I2C2_BASE,
         SIZE_4KB,
         EFI_MEMORY_UC
         );
  gDS->SetMemorySpaceAttributes (I2C2_BASE, SIZE_4KB, EFI_MEMORY_UC);

  Status = I2cReadBytes (I2C2_BASE, EEPROM_ADDR, 0, Header, TLV_HEADER_SIZE);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_WARN, "%a: EEPROM read failed: %r\n", __func__, Status));
    return Status;
  }

  if (AsciiStrnCmp ((CHAR8 *)Header, TLV_SIGNATURE, 7) != 0) {
    DEBUG ((DEBUG_WARN, "%a: TLV signature mismatch\n", __func__));
    return EFI_NOT_FOUND;
  }

  TotalLen = (UINT16)((Header[9] << 8) | Header[10]);
  if ((TotalLen == 0) || (TotalLen > 245)) {
    DEBUG ((DEBUG_WARN, "%a: TLV length invalid (%u)\n", __func__, TotalLen));
    return EFI_NOT_FOUND;
  }

  Offset    = TLV_HEADER_SIZE;
  EndOffset = (UINT8)(TLV_HEADER_SIZE + (UINT8)TotalLen);

  while ((UINTN)(Offset + 2) <= EndOffset) {
    Status = I2cReadByte (I2C2_BASE, EEPROM_ADDR, Offset, &Tid);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    Status = I2cReadByte (I2C2_BASE, EEPROM_ADDR, (UINT8)(Offset + 1), &Tlen);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    if (Tid == TLV_CODE_PRODUCT_NAME) {
      if ((Tlen == 0) || ((UINTN)Tlen >= BufferSize)) {
        return EFI_NOT_FOUND;
      }

      Status = I2cReadBytes (
                 I2C2_BASE,
                 EEPROM_ADDR,
                 (UINT8)(Offset + 2),
                 (UINT8 *)ProductName,
                 Tlen
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }

      ProductName[Tlen] = '\0';
      return EFI_SUCCESS;
    }

    Offset = (UINT8)(Offset + 2 + Tlen);
  }

  DEBUG ((DEBUG_WARN, "%a: product_name TLV entry not found\n", __func__));
  return EFI_NOT_FOUND;
}

/**
  Get the model string from the HOB DTB as a fallback.

  @param[out] Model     Pointer to the model string (NOT a copy).
  @param[out] ModelLen  Length of the model string.

  @retval EFI_SUCCESS     Model found.
  @retval EFI_NOT_FOUND   DTB or model property not available.
**/
STATIC
EFI_STATUS
GetModelFromHobDtb (
  OUT CONST CHAR8  **Model,
  OUT INT32        *ModelLen
  )
{
  VOID    *Hob;
  VOID    *FdtBase;
  UINT64  FdtAddress;
  INTN    RootOffset;

  Hob = GetFirstGuidHob (&gFdtHobGuid);
  if ((Hob == NULL) || (GET_GUID_HOB_DATA_SIZE (Hob) != sizeof (UINT64))) {
    return EFI_NOT_FOUND;
  }

  FdtAddress = *((UINT64 *)GET_GUID_HOB_DATA (Hob));
  FdtBase    = (VOID *)(UINTN)FdtAddress;
  if (FdtCheckHeader (FdtBase) != 0) {
    return EFI_NOT_FOUND;
  }

  RootOffset = FdtPathOffset (FdtBase, "/");
  if (RootOffset < 0) {
    return EFI_NOT_FOUND;
  }

  *Model = FdtGetProp (FdtBase, RootOffset, "model", ModelLen);
  if ((*Model == NULL) || (*ModelLen <= 0)) {
    return EFI_NOT_FOUND;
  }

  return EFI_SUCCESS;
}

/**
  Match a model/product name string to a SKU ID.

  @param[in]  Name  Null-terminated board identity string.

  @return SKU ID (DEFAULT, COM260, or FML13V05).
**/
STATIC
UINTN
MatchSku (
  IN CONST CHAR8  *Name
  )
{
  if (AsciiStrStr (Name, "com260") != NULL) {
    return SKU_ID_COM260;
  }

  if (AsciiStrStr (Name, "fml13v05") != NULL) {
    return SKU_ID_FML13V05;
  }

  return SKU_ID_DEFAULT;
}

EFI_STATUS
EFIAPI
SkuSelectDxeEntryPoint (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS   Status;
  CHAR8        ProductName[64];
  CONST CHAR8  *Model;
  INT32        ModelLen;
  CONST CHAR8  *Source;
  CONST CHAR8  *Identity;
  UINTN        SkuId;

  //
  // Priority 1: TLV EEPROM product_name via raw I2C.
  //
  Status = ReadProductNameFromEeprom (ProductName, sizeof (ProductName));
  if (!EFI_ERROR (Status)) {
    Identity = ProductName;
    Source   = "EEPROM";
  } else {
    //
    // Priority 2: HOB DTB model property.
    //
    Status = GetModelFromHobDtb (&Model, &ModelLen);
    if (!EFI_ERROR (Status)) {
      Identity = Model;
      Source   = "DTB";
    } else {
      DEBUG ((DEBUG_WARN, "%a: no board identity found, using DEFAULT SKU\n",
              __func__));
      LibPcdSetSku (SKU_ID_DEFAULT);
      return EFI_SUCCESS;
    }
  }

  SkuId = MatchSku (Identity);

  DEBUG ((DEBUG_INFO, "%a: %a product_name = \"%a\"\n", __func__, Source, Identity));
  DEBUG ((DEBUG_INFO, "%a: SKU set to %a (%u)\n", __func__,
          SkuId == SKU_ID_COM260 ? "COM260" :
          SkuId == SKU_ID_FML13V05 ? "FML13V05" : "DEFAULT",
          SkuId));

  LibPcdSetSku (SkuId);
  return EFI_SUCCESS;
}
