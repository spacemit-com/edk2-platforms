/** @file
  Load the vendor Linux DTB from the firmware volume for the
  SpacemiT K3 platform.

  The DTB is embedded at build time (like ACPI tables on x86) and
  extracted here via GetSectionFromAnyFv.  Boot-hartid is set for
  RISC-V EFISTUB, and EFI_DT_FIXUP_PROTOCOL applies MAC address,
  serial number, and memory node fixups.

  Copyright (c) 2026, Spacemit Corporation. All rights reserved.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <PiDxe.h>

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/FdtLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include <Protocol/SimpleFileSystem.h>
#include <Guid/FileInfo.h>

//
// EFI_DT_FIXUP_PROTOCOL (from Silicon/Spacemit FdtFixupDxe).
//
#define EFI_DT_APPLY_FIXUPS  0x00000001

typedef struct _EFI_DT_FIXUP_PROTOCOL EFI_DT_FIXUP_PROTOCOL;

typedef
EFI_STATUS
(EFIAPI *EFI_FDT_FIXUP_PROTOCOL_FIXUP)(
  IN     EFI_DT_FIXUP_PROTOCOL  *This,
  IN OUT VOID                   *Dtb,
  IN OUT UINTN                  *BufferSize,
  IN     UINT32                 Flags
  );

struct _EFI_DT_FIXUP_PROTOCOL {
  UINT64                        Revision;
  EFI_FDT_FIXUP_PROTOCOL_FIXUP Fixup;
};

extern EFI_GUID  gEfiFdtFixupProtocolGuid;

//
// GUID matching the FILE FREEFORM entry in MUSE-Pico.fdf.
//
STATIC CONST EFI_GUID  mVendorDtbFvGuid = {
  0x7901d61e, 0xf64d, 0x4989,
  { 0x83, 0x5f, 0x3e, 0xbe, 0x50, 0xc8, 0x3c, 0x99 }
};

#define VENDOR_DTB_PATH     L"\\dtb\\vendor.dtb"
#define DTB_FIXUP_PADDING   0x8000
#define DTB_MAX_SIZE        (4 * 1024 * 1024)

STATIC VOID   *mDeferredFixupDtb;
STATIC UINTN  mDeferredFixupSize;

/**
  Set boot-hartid in /chosen (required by RISC-V EFISTUB).

  @param[in,out]  Fdt  Device tree blob with sufficient padding.
**/
STATIC
VOID
SetBootHartId (
  IN OUT VOID  *Fdt
  )
{
  INT32   ChosenOffset;
  UINT32  HartId;
  UINT32  HartIdBe;

  HartId       = PcdGet32 (PcdBootHartId);
  ChosenOffset = FdtPathOffset (Fdt, "/chosen");

  if (ChosenOffset < 0) {
    ChosenOffset = FdtAddSubnode (Fdt, 0, "chosen");
    if (ChosenOffset < 0) {
      DEBUG ((DEBUG_WARN, "%a: cannot create /chosen\n", __func__));
      return;
    }
  }

  HartIdBe = CpuToFdt32 (HartId);
  FdtSetProp (Fdt, ChosenOffset, "boot-hartid", &HartIdBe, sizeof (HartIdBe));
  DEBUG ((DEBUG_INFO, "%a: boot-hartid = %u\n", __func__, HartId));
}

/**
  ReadyToBoot callback — apply platform fixups (MAC, serial, memory)
  to the vendor DTB installed earlier by DtPlatformDxe.

  FdtFixupDxe DEPEXes on PlatformInfoDxe and dispatches during BDS
  ConnectAll, after initial DXE dispatch.  This callback runs at
  ReadyToBoot when the fixup protocol is available.  The DTB is
  modified in-place; the config table pointer remains valid.

  @param[in]  Event    ReadyToBoot event.
  @param[in]  Context  Unused.
**/
STATIC
VOID
EFIAPI
OnReadyToBootApplyFixups (
  IN EFI_EVENT  Event,
  IN VOID       *Context
  )
{
  EFI_STATUS              Status;
  EFI_DT_FIXUP_PROTOCOL  *FdtFixup;
  UINTN                  Size;

  gBS->CloseEvent (Event);

  if (mDeferredFixupDtb == NULL) {
    return;
  }

  Status = gBS->LocateProtocol (
                  &gEfiFdtFixupProtocolGuid,
                  NULL,
                  (VOID **)&FdtFixup
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_WARN, "%a: fixup protocol still not found at ReadyToBoot\n",
            __func__));
    return;
  }

  Size   = mDeferredFixupSize;
  Status = FdtFixup->Fixup (FdtFixup, mDeferredFixupDtb, &Size, EFI_DT_APPLY_FIXUPS);

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_WARN, "%a: fixup returned %r\n", __func__, Status));
  } else {
    DEBUG ((DEBUG_INFO, "%a: fixups applied, DTB size = %u\n", __func__,
            FdtTotalSize (mDeferredFixupDtb)));
  }
}

/**
  Try to load the vendor DTB from a filesystem volume.

  @param[in]   Fs          SimpleFileSystem protocol.
  @param[out]  Dtb         Allocated buffer with DTB.
  @param[out]  DtbSize     Actual DTB size (without padding).

  @retval EFI_SUCCESS      DTB loaded and validated.
  @retval EFI_NOT_FOUND    File not present on this volume.
**/
STATIC
EFI_STATUS
LoadDtbFromFilesystem (
  IN  EFI_SIMPLE_FILE_SYSTEM_PROTOCOL  *Fs,
  OUT VOID                             **Dtb,
  OUT UINTN                            *DtbSize
  )
{
  EFI_STATUS         Status;
  EFI_FILE_PROTOCOL  *Root = NULL;
  EFI_FILE_PROTOCOL  *File = NULL;
  EFI_FILE_INFO      *Info = NULL;
  UINTN              InfoSize;
  UINTN              FileSize;
  UINTN              AllocSize;
  VOID               *Buffer = NULL;

  Status = Fs->OpenVolume (Fs, &Root);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = Root->Open (Root, &File, VENDOR_DTB_PATH, EFI_FILE_MODE_READ, 0);
  if (EFI_ERROR (Status)) {
    Root->Close (Root);
    return EFI_NOT_FOUND;
  }

  InfoSize = sizeof (EFI_FILE_INFO) + 256;
  Info     = AllocatePool (InfoSize);
  if (Info == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Done;
  }

  Status = File->GetInfo (File, &gEfiFileInfoGuid, &InfoSize, Info);
  if (EFI_ERROR (Status)) {
    goto Done;
  }

  FileSize = (UINTN)Info->FileSize;
  if ((FileSize == 0) || (FileSize > DTB_MAX_SIZE)) {
    Status = EFI_INVALID_PARAMETER;
    goto Done;
  }

  AllocSize = FileSize + DTB_FIXUP_PADDING;
  Buffer    = AllocatePool (AllocSize);
  if (Buffer == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Done;
  }

  Status = File->Read (File, &FileSize, Buffer);
  if (EFI_ERROR (Status)) {
    FreePool (Buffer);
    Buffer = NULL;
    goto Done;
  }

  if (FdtCheckHeader (Buffer) != 0) {
    FreePool (Buffer);
    Buffer = NULL;
    Status = EFI_INVALID_PARAMETER;
    goto Done;
  }

  *Dtb     = Buffer;
  *DtbSize = AllocSize;

Done:
  if (Info != NULL) {
    FreePool (Info);
  }

  if (File != NULL) {
    File->Close (File);
  }

  if (Root != NULL) {
    Root->Close (Root);
  }

  return Status;
}

/**
  Return a pool-allocated copy of the vendor DTB for the K3 platform.

  Search order:
    1. Filesystem -- \dtb\vendor.dtb on any volume (kernel-matched)
    2. Firmware volume -- factory DTB embedded in firmware

  Boot-hartid and platform fixups are applied before returning.

  @param[out]  Dtb      Pointer to the DTB copy.
  @param[out]  DtbSize  Size of the DTB copy.

  @retval EFI_SUCCESS           DTB loaded and ready.
  @retval EFI_NOT_FOUND         No vendor DTB in FV or filesystem.
  @retval EFI_OUT_OF_RESOURCES  Allocation failure.
**/
EFI_STATUS
EFIAPI
DtPlatformLoadDtb (
  OUT VOID   **Dtb,
  OUT UINTN  *DtbSize
  )
{
  EFI_STATUS  Status;
  EFI_HANDLE  *Handles     = NULL;
  UINTN       HandleCount  = 0;
  UINTN       Index;
  VOID        *DtbBuffer   = NULL;
  UINTN       BufferSize   = 0;
  VOID        *FvDtb;
  UINTN       FvDtbSize;
  UINTN       AllocSize;

  EFI_SIMPLE_FILE_SYSTEM_PROTOCOL  *Fs;

  //
  // Priority 1: filesystem (kernel-matched DTB from ESP).
  //
  Status = gBS->LocateHandleBuffer (
                  ByProtocol,
                  &gEfiSimpleFileSystemProtocolGuid,
                  NULL,
                  &HandleCount,
                  &Handles
                  );
  if (!EFI_ERROR (Status)) {
    for (Index = 0; Index < HandleCount; Index++) {
      Status = gBS->HandleProtocol (
                      Handles[Index],
                      &gEfiSimpleFileSystemProtocolGuid,
                      (VOID **)&Fs
                      );
      if (EFI_ERROR (Status)) {
        continue;
      }

      Status = LoadDtbFromFilesystem (Fs, &DtbBuffer, &BufferSize);
      if (!EFI_ERROR (Status)) {
        DEBUG ((DEBUG_INFO, "%a: loaded vendor DTB from filesystem (%u bytes)\n",
                __func__, FdtTotalSize (DtbBuffer)));
        break;
      }
    }

    FreePool (Handles);
  }

  //
  // Priority 2: firmware volume (factory DTB).
  //
  if (DtbBuffer == NULL) {
    Status = GetSectionFromAnyFv (
               &mVendorDtbFvGuid,
               EFI_SECTION_RAW,
               0,
               &FvDtb,
               &FvDtbSize
               );
    if (!EFI_ERROR (Status)) {
      if (FdtCheckHeader (FvDtb) != 0) {
        DEBUG ((DEBUG_ERROR, "%a: FV DTB has invalid header\n", __func__));
        FreePool (FvDtb);
        return EFI_NOT_FOUND;
      }

      AllocSize = FvDtbSize + DTB_FIXUP_PADDING;
      DtbBuffer = AllocatePool (AllocSize);
      if (DtbBuffer == NULL) {
        FreePool (FvDtb);
        return EFI_OUT_OF_RESOURCES;
      }

      CopyMem (DtbBuffer, FvDtb, FvDtbSize);
      FreePool (FvDtb);
      BufferSize = AllocSize;

      DEBUG ((DEBUG_INFO, "%a: loaded vendor DTB from firmware volume (%u bytes)\n",
              __func__, FdtTotalSize (DtbBuffer)));
    }
  }

  if (DtbBuffer == NULL) {
    DEBUG ((DEBUG_WARN, "%a: no vendor DTB found\n", __func__));
    return EFI_NOT_FOUND;
  }

  //
  // Expand and apply boot-hartid + platform fixups.
  //
  if (FdtOpenInto (DtbBuffer, DtbBuffer, (INT32)BufferSize) != 0) {
    DEBUG ((DEBUG_ERROR, "%a: FdtOpenInto failed\n", __func__));
    FreePool (DtbBuffer);
    return EFI_OUT_OF_RESOURCES;
  }

  SetBootHartId (DtbBuffer);

  //
  // Defer fixups to ReadyToBoot — FdtFixupDxe is not yet dispatched
  // at DXE time (it DEPEXes on PlatformInfoDxe which loads during
  // BDS ConnectAll).
  //
  mDeferredFixupDtb  = DtbBuffer;
  mDeferredFixupSize = BufferSize;

  {
    EFI_EVENT  ReadyToBootEvent;

    Status = EfiCreateEventReadyToBootEx (
               TPL_CALLBACK,
               OnReadyToBootApplyFixups,
               NULL,
               &ReadyToBootEvent
               );
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_WARN, "%a: failed to register fixup callback: %r\n",
              __func__, Status));
    }
  }

  *Dtb     = DtbBuffer;
  *DtbSize = FdtTotalSize (DtbBuffer);

  return EFI_SUCCESS;
}
