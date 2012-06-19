TARGET = usb
OBJS = usb.o usbconfig.o usbdrv.o usbsysevt.o imports.o sysreg.o

INCDIR = 
CFLAGS = -O2 -G0 -Wall -fno-builtin-printf -fno-builtin-log -D__USE_SYSREG
ASFLAGS = $(CFLAGS)
LDFLAGS = -mno-crt0 -nostartfiles
LIBDIR =

# Use the kernel's small inbuilt libc
#USE_KERNEL_LIBC = 1
# Use only kernel libraries
#USE_KERNEL_LIBS = 1

PRX_EXPORTS=exports.exp
BUILD_PRX=1

PSPSDK=$(shell psp-config --pspsdk-path)
include $(PSPSDK)/lib/build.mak
LIBS = 
