
#include <pspkernel.h>

#define SYSREG_RESET_ENABLE_REG     (*(volatile u32 *)(0xBC10004C))
#define SYSREG_BUSCLK_ENABLE_REG    (*(volatile u32 *)(0xBC100050))
#define SYSREG_CLK1_ENABLE_REG      (*(volatile u32 *)(0xBC100054))
#define SYSREG_UNK_IO_ENABLE_REG    (*(volatile u32 *)(0xBC100074))
#define SYSREG_IO_ENABLE_REG        (*(volatile u32 *)(0xBC100078))

#define SET_MASK(_dev)  (1<<(_dev))
#define SYSREG_ENABLE   (1)
#define SYSREG_DISABLE  (0)

enum SysregResetDevices
{
    SYSREG_RESET_TOP = 0,
    SYSREG_RESET_SC,
    SYSREG_RESET_ME,
    SYSREG_RESET_AW,
    SYSREG_RESET_VME,
    SYSREG_RESET_AVC,
    SYSREG_RESET_USB,
    SYSREG_RESET_ATA,
    SYSREG_RESET_MSIF0,
    SYSREG_RESET_MSIF1,
    SYSREG_RESET_KIRK
};

enum SysregBusClkDevices
{
    SYSREG_BUSCLK_ME = 0,
    SYSREG_BUSCLK_AWA,
    SYSREG_BUSCLK_AWB,
    SYSREG_BUSCLK_EDRAM,
    SYSREG_BUSCLK_DMACPLUS,
    SYSREG_BUSCLK_DMAC0,
    SYSREG_BUSCLK_DMAC1,
    SYSREG_BUSCLK_KIRK,
    SYSREG_BUSCLK_ATA,
    SYSREG_BUSCLK_USB,
    SYSREG_BUSCLK_MSIF0,
    SYSREG_BUSCLK_MSIF1,
    SYSREG_BUSCLK_EMCDDR,
    SYSREG_BUSCLK_EMCSM,
    SYSREG_BUSCLK_APB,
    SYSREG_BUSCLK_AUDIO0,
    SYSREG_BUSCLK_AUDIO1
};

enum SysregClk1Devices
{
    SYSREG_CLK1_ATA = 0,
    SYSREG_CLK1_USB = 4,
    SYSREG_CLK1_MSIF0 = 8,
    SYSREG_CLK1_MSIF1
};

enum SysregIoDevices
{
    SYSREG_IO_EMCSM = 1,
    SYSREG_IO_USB,
    SYSREG_IO_ATA,
    SYSREG_IO_MSIF0,
    SYSREG_IO_MSIF1,
    SYSREG_IO_LCDC,
    SYSREG_IO_AUDIO0,
    SYSREG_IO_AUDIO1,
    SYSREG_IO_IIC,
    SYSREG_IO_SIRCS,
    SYSREG_IO_UNK,
    SYSREG_IO_KEY,
    SYSREG_IO_PWM,
    SYSREG_IO_UART0 = 16,
    SYSREG_IO_UART1,
    SYSREG_IO_UART2,
    SYSREG_IO_UART3,
    SYSREG_IO_UART4,
    SYSREG_IO_UART5,
    SYSREG_IO_SPI0 = 24,
    SYSREG_IO_SPI1,
    SYSREG_IO_SPI2,
    SYSREG_IO_SPI3,
    SYSREG_IO_SPI4,
    SYSREG_IO_SPI5
};


static
int reset_enable (u32 devmask, int enable)
{
  u32 intr = sceKernelCpuSuspendIntr();

  int prev = ((SYSREG_RESET_ENABLE_REG & devmask) > 0) ? SYSREG_ENABLE : SYSREG_DISABLE;

  if (enable)
    SYSREG_RESET_ENABLE_REG |=  devmask;
  else
    SYSREG_RESET_ENABLE_REG &= ~devmask;

  sceKernelCpuResumeIntr(intr);

  return(prev);
}

static
int bus_clk_enable (u32 devmask, int enable)
{
  u32 intr = sceKernelCpuSuspendIntr ();

  int prev = ((SYSREG_BUSCLK_ENABLE_REG & devmask) > 0) ? SYSREG_ENABLE : SYSREG_DISABLE;

  if (enable)
    SYSREG_BUSCLK_ENABLE_REG |=  devmask;
  else
    SYSREG_BUSCLK_ENABLE_REG &= ~devmask;

  sceKernelCpuResumeIntr(intr);

  return(prev);
}

static
int clk_enable_1(u32 devmask, int enable)
{
  u32 intr = sceKernelCpuSuspendIntr();

  int prev = ((SYSREG_CLK1_ENABLE_REG & devmask) > 0) ? SYSREG_ENABLE : SYSREG_DISABLE;

  if (enable)
    SYSREG_CLK1_ENABLE_REG |=  devmask;
  else
    SYSREG_CLK1_ENABLE_REG &= ~devmask;

  sceKernelCpuResumeIntr(intr);

  return(prev);
}

static
int io_enable(u32 devmask, int enable)
{
  u32 intr = sceKernelCpuSuspendIntr();

  int prev = ((SYSREG_IO_ENABLE_REG & devmask) > 0) ? SYSREG_ENABLE : SYSREG_DISABLE;

  if (enable)
    SYSREG_IO_ENABLE_REG |=  devmask;
  else
    SYSREG_IO_ENABLE_REG &= ~devmask;

  sceKernelCpuResumeIntr(intr);

  return(prev);
}

static
int unk_io_enable(u32 devmask, int enable)
{
  u32 intr = sceKernelCpuSuspendIntr();

  int prev = ((SYSREG_UNK_IO_ENABLE_REG & devmask) > 0) ? SYSREG_ENABLE : SYSREG_DISABLE;

  if (enable)
    SYSREG_UNK_IO_ENABLE_REG |=  devmask;
  else
    SYSREG_UNK_IO_ENABLE_REG &= ~devmask;

  sceKernelCpuResumeIntr(intr);

  return(prev);
}





int sceSysregUsbResetEnable (void)
{
  return(reset_enable(SET_MASK(SYSREG_RESET_USB), SYSREG_ENABLE));
}

int sceSysregUsbResetDisable (void)
{
  return(reset_enable(SET_MASK(SYSREG_RESET_USB), SYSREG_DISABLE));
}

int sceSysregUsbBusClockEnable (void)
{
  return(bus_clk_enable(SET_MASK(SYSREG_BUSCLK_USB), SYSREG_ENABLE));
}

int sceSysregUsbBusClockDisable (void)
{
  return(bus_clk_enable(SET_MASK(SYSREG_BUSCLK_USB), SYSREG_DISABLE));
}


int sceSysregUsbClkEnable (int no)
{
  return(clk_enable_1((no&0xF)<<SYSREG_CLK1_USB, SYSREG_ENABLE));
}

int sceSysregUsbClkDisable (int no)
{
  return(clk_enable_1((no&0xF)<<SYSREG_CLK1_USB, SYSREG_DISABLE));
}

int sceSysregUsbIoEnable (void)
{
  return(io_enable(SET_MASK(SYSREG_IO_USB), SYSREG_ENABLE));
}

int sceSysregUsbIoDisable (void)
{
  return(io_enable(SET_MASK(SYSREG_IO_USB), SYSREG_DISABLE));
}

int sceSysregUsbGetConnectStatus (void)
{
  return(*(volatile u32*)(0xBC100080) & 1);
}

int sceSysregUsbQueryIntr (void)
{
  return((*(volatile u32*)(0xBC100080)>>1) & 0xF);
}

int sceSysregUsbAcquireIntr (int mask)
{
  u32 intr = sceKernelCpuSuspendIntr ();

  int prev = (*(volatile u32*)(0xBC100080)>>1) & 0xF;
  *(volatile u32*)(0xBC100080) = (mask & prev) << 1;

  sceKernelCpuResumeIntrWithSync (intr);

  return(prev);
}

int sceSysreg_driver_8D0FED1E (void)
{
  return(unk_io_enable(0x100, SYSREG_ENABLE));
}

int sceSysreg_driver_A46E9CA8 (void)
{
  return(unk_io_enable(0x100, SYSREG_DISABLE));
}




