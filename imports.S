    .set noreorder

#include "pspimport.s"

    IMPORT_START    "InterruptManagerForKernel",0x00090011
    IMPORT_FUNC     "InterruptManagerForKernel",0x092968F4,sceKernelCpuSuspendIntr
    IMPORT_FUNC     "InterruptManagerForKernel",0x5F10D406,sceKernelCpuResumeIntr
    IMPORT_FUNC     "InterruptManagerForKernel",0x3B84732D,sceKernelCpuResumeIntrWithSync
    IMPORT_FUNC     "InterruptManagerForKernel",0xFE28C6D9,sceKernelIsIntrContext
    IMPORT_FUNC     "InterruptManagerForKernel",0x58DD8978,sceKernelRegisterIntrHandler
    IMPORT_FUNC     "InterruptManagerForKernel",0xF987B1F0,sceKernelReleaseIntrHandler
    IMPORT_FUNC     "InterruptManagerForKernel",0x4D6E7305,sceKernelEnableIntr
    IMPORT_FUNC     "InterruptManagerForKernel",0xD774BA45,sceKernelDisableIntr

    IMPORT_START    "KDebugForKernel",0x00010011
    IMPORT_FUNC     "KDebugForKernel",0x84F370BC,Kprintf

    IMPORT_START    "SysclibForKernel",0x00010011
    IMPORT_FUNC     "SysclibForKernel",0x7AB35214,strncmp
    IMPORT_FUNC     "SysclibForKernel",0xB49A7697,strncpy
    IMPORT_FUNC     "SysclibForKernel",0x52DF196C,strlen
    IMPORT_FUNC     "SysclibForKernel",0xAB7592FF,memcpy

    IMPORT_START    "sceSysEventForKernel",0x00010011
    IMPORT_FUNC     "sceSysEventForKernel",0xCD9E4BB5,sceKernelRegisterSysEventHandler
    IMPORT_FUNC     "sceSysEventForKernel",0xD7D3FDCD,sceKernelUnregisterSysEventHandler

    IMPORT_START    "sceSuspendForKernel",0x00010011
    IMPORT_FUNC     "sceSuspendForKernel",0xEADB1BD7,sceKernelPowerLock
    IMPORT_FUNC     "sceSuspendForKernel",0x3AEE7261,sceKernelPowerUnlock

    IMPORT_START    "ThreadManForKernel",0x00010011
    IMPORT_FUNC     "ThreadManForKernel",0xD6DA4BA1,sceKernelCreateSema
    IMPORT_FUNC     "ThreadManForKernel",0x28B6489C,sceKernelDeleteSema
    IMPORT_FUNC     "ThreadManForKernel",0x3F53E640,sceKernelSignalSema
    IMPORT_FUNC     "ThreadManForKernel",0x4E3A1105,sceKernelWaitSema
    IMPORT_FUNC     "ThreadManForKernel",0x55C20A00,sceKernelCreateEventFlag
    IMPORT_FUNC     "ThreadManForKernel",0xEF9E4C70,sceKernelDeleteEventFlag
    IMPORT_FUNC     "ThreadManForKernel",0x1FB15A32,sceKernelSetEventFlag
    IMPORT_FUNC     "ThreadManForKernel",0x812346E4,sceKernelClearEventFlag
    IMPORT_FUNC     "ThreadManForKernel",0x402FCF22,sceKernelWaitEventFlag
    IMPORT_FUNC     "ThreadManForKernel",0xCD203292,sceKernelCancelEventFlag
    IMPORT_FUNC     "ThreadManForKernel",0xC07BB470,sceKernelCreateFpl
    IMPORT_FUNC     "ThreadManForKernel",0xED1410E0,sceKernelDeleteFpl
    IMPORT_FUNC     "ThreadManForKernel",0x623AE665,sceKernelTryAllocateFpl
    IMPORT_FUNC     "ThreadManForKernel",0x6652B8CA,sceKernelSetAlarm
    IMPORT_FUNC     "ThreadManForKernel",0x7E65B999,sceKernelCancelAlarm

    IMPORT_START    "UtilsForKernel",0x00090011
    IMPORT_FUNC     "UtilsForKernel",0x3EE30821,sceKernelDcacheWritebackRange
    IMPORT_FUNC     "UtilsForKernel",0xBFA98062,sceKernelDcacheInvalidateRange

    IMPORT_START    "sceIdStorage_driver",0x00010011
    IMPORT_FUNC     "sceIdStorage_driver",0xEB830733,sceIdStorageGetLeafSize
    IMPORT_FUNC     "sceIdStorage_driver",0xFEFA40C2,sceIdStorageIsFormatted
    IMPORT_FUNC     "sceIdStorage_driver",0x6FE062D1,sceIdStorageLookup

#ifndef __USE_SYSREG

    IMPORT_START    "sceSysreg_driver",0x00010011
    IMPORT_FUNC     "sceSysreg_driver",0xCD4FB614,sceSysregUsbResetEnable
    IMPORT_FUNC     "sceSysreg_driver",0x69EECBE5,sceSysregUsbResetDisable
    IMPORT_FUNC     "sceSysreg_driver",0x3E216017,sceSysregUsbBusClockEnable
    IMPORT_FUNC     "sceSysreg_driver",0xBFBABB63,sceSysregUsbBusClockDisable
    IMPORT_FUNC     "sceSysreg_driver",0x7234EA80,sceSysregUsbClkEnable
    IMPORT_FUNC     "sceSysreg_driver",0x38EC3281,sceSysregUsbClkDisable
    IMPORT_FUNC     "sceSysreg_driver",0xBC68D9B6,sceSysregUsbIoEnable
    IMPORT_FUNC     "sceSysreg_driver",0xA3C8E075,sceSysregUsbIoDisable
    IMPORT_FUNC     "sceSysreg_driver",0x8D0FED1E,sceSysreg_driver_8D0FED1E
    IMPORT_FUNC     "sceSysreg_driver",0xA46E9CA8,sceSysreg_driver_A46E9CA8
    IMPORT_FUNC     "sceSysreg_driver",0x633595F2,sceSysregUsbGetConnectStatus
    IMPORT_FUNC     "sceSysreg_driver",0x32471457,sceSysregUsbQueryIntr
    IMPORT_FUNC     "sceSysreg_driver",0x692F31FF,sceSysregUsbAcquireIntr

#endif /* __USE_SYSREG */
