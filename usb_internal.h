#ifndef __USB_INTERNAL_H
#define __USB_INTERNAL_H

#include "usb.h"
#include <pspsdk.h>
#include <stddef.h>

/* Constants and macros */
#define DEFAULT_USB_TIMEOUT 1000000
#define MAIN_SEMAPHORE_NAME "SceUsb"

#define PSP_USB_IDSTORAGE_KEY  0x41

#define PSP_USB_IRQ_MAIN       26
#define PSP_USB_IRQ_RESUME     56
#define PSP_USB_IRQ_READY      57
#define PSP_USB_IRQ_CONNECT    58
#define PSP_USB_IRQ_DISCONNECT 59

#define PSP_USB_INTERRUPT_CONNECT    1
#define PSP_USB_INTERRUPT_DISCONNECT 2
#define PSP_USB_INTERRUPT_READY      4
#define PSP_USB_INTERRUPT_RESUME     8

#define USB_PORT_0000                             0xbd800000
#define USB_PORT_0004                             0xbd800004
#define USB_PORT_ENDPOINT_PACKET_SIZEW_BASE       0xbd800008
#define USB_PORT_PACKET_SIZE_BASE                 0xbd80000C 
#define USB_PORT_TXDMA_BASE                       0xbd800014
#define USB_PORT_0200                             0xbd800200
#define USB_PORT_0204                             0xbd800204
#define USB_PORT_020C                             0xbd80020C
#define USB_PORT_0210                             0xbd800210
#define USB_PORT_RXDMA_BASE                       0xbd800214
#define USB_PORT_0400                             0xbd800400
#define USB_PORT_0404                             0xbd800404
#define USB_PORT_0408                             0xbd800408
#define USB_PORT_040C                             0xbd80040C
#define USB_PORT_0410                             0xbd800410
#define USB_PORT_0414                             0xbd800414
#define USB_PORT_0418                             0xbd800418
#define USB_PORT_MAINALARM_SETZERO                0xbd80041C
#define USB_PORT_ENDPOINT_CONFIG_BASE4            0xbd800504

#define IO_PORT_BC000050                          0xbc000050
#define IO_PORT_DDRFLUSH                          0xbd000004

#define PSP_USB_MAX_TXFIFO_SIZE    3265
#define PSP_USB_FIFO_ELEMENTS      16

#define PSP_USB_ENDPOINT_STATUS_TRANSMITTING     0x01
#define PSP_USB_ENDPOINT_STATUS_REM0             0x02
#define PSP_USB_ENDPOINT_STATUS_EP0_USE_FIFO     0x04
#define PSP_USB_ENDPOINT_STATUS_UNKNOWN2         0x08
#define PSP_USB_ENDPOINT_STATUS_HASDATA          0x10
#define PSP_USB_ENDPOINT_STATUS_COPYING          0x20
#define PSP_USB_ENDPOINT_STATUS_CLEARED          0x40
#define PSP_USB_ENDPOINT_STATUS_STALLED          0x80
#define PSP_USB_ENDPOINT_STATUS_CANCEL          0x100
#define PSP_USB_ENDPOINT_STATUS_CANCELALL       0x200
#define PSP_USB_ENDPOINT_STATUS_DMACONFIGURED   0x400

#define PSP_USB_MAIN_ALARM_TIME              3000
#define PSP_USB_ALARM_TIME                    500

#define SYNC() __builtin_allegrex_sync ()


#define SAVE_K1() \
  __asm__ __volatile__ (       \
      "move %0, $k1\n"         \
      "srl  $k1, $k1, 16\n"    \
      : "=r"(k1)               \
      :                        \
      : "$27"                  \
  )

#define RESTORE_K1() \
  __asm__ __volatile__ (       \
      "move $k1, %0\n"         \
      :                        \
      : "r"(k1)                \
      : "$27"                  \
  )

#define IS_USER_MODE()  (((k1 >> 16) & 0x18) != 0)
#define CHECK_RANGE(ptr, size) \
  ((((unsigned int)(ptr))|((unsigned int)(size))|(((unsigned int)(ptr))+((unsigned int)(size))))&0x80000000)


#define CHECK_DDR_ADDRESS(addr) \
  (((((addr) >> 27) & 3) == 1) && ((0x35 >> (((addr) >> 29) & 0x07)) & 0x01))

#define CHECK_CACHED_ADDRESS(addr) \
  ((0xD3 >> ((((unsigned int) (addr)) >> 29) & 0x07)) & 1)

#define PHYSICAL_ADDRESS(addr) \
  (((((addr) >> 31) ^ 1) << 30) | ((addr) & 0x1FFFFFFF))

#define ALIGN_UP(addr, bits) \
  ((((addr) - 1) | (bits)) + 1)

#define ALIGN_DOWN(addr, bits) \
  ((addr) & (~(bits)))


/* Structures */
struct usb_rxfifo {
  char *buffer;
  int fifo_size;
  int end;
  int start;
  struct rxfifo_element *elements;
};

struct rxfifo_element {
  char *ptr;
  int  size;
};

struct usb_queue {
  struct UsbEndpoint *endpoint;
  int endpoint_type; /* CONTROL, ISOCHRONOUS, BULK, INTERRUPT */
  int endpoint_dir;
  int packet_size;
  struct UsbbdDeviceRequest *first;
  struct UsbbdDeviceRequest *last;
};

struct usb_endpoint {
  struct usb_queue *queue;
  int packet_size;
  int status;         /* status */
  int unk4;
  volatile struct usb_port *ports[2];
  int idx;            /* some index */
  int interval;       /* 1 << bInterval */
  SceUID alarm;
};

struct usb_port {
  volatile unsigned int unk1;
  volatile unsigned int unk2;
  volatile unsigned int unk3;
  volatile unsigned int unk4;
}; 



/* Variables */
extern struct UsbDriver *g_started_drivers;
extern struct UsbDriver *g_registered_drivers;

extern SceUID g_mainsema;
extern int g_busdrv_started;

extern int g_usb_version;
extern int g_prev_cable_connected;
extern int g_usb_activated;
extern int g_cable_connected;
extern int g_connection_status;
extern int g_total_interfaces;
extern int g_total_endpoints;

extern struct DeviceDescriptor *g_devdesc_hi;
extern struct UsbConfiguration *g_conf_hi;
extern struct DeviceDescriptor *g_devdesc;
extern struct UsbConfiguration *g_conf;
extern struct StringDescriptor *g_descriptors;
extern struct DeviceQualifierDescriptor *g_qual_hi;
extern struct DeviceQualifierDescriptor *g_qual;

extern struct StringDescriptor *g_string;
extern struct StringDescriptor *g_vendor_desc;
extern struct StringDescriptor *g_product_desc;

extern struct DeviceRequest g_devreq;
extern struct UsbEndpoint g_ep0;
extern struct UsbbdDeviceRequest g_request;

extern SceUID g_devdescfpl;
extern SceUID g_fsconffpl;
extern SceUID g_hsconffpl;
extern SceUID g_ep0fpl;

extern struct usb_queue g_queues[PSP_USB_MAX_ENDPOINTS];
extern struct usb_endpoint g_endpoints[PSP_USB_MAX_ENDPOINTS + 1];
extern struct usb_rxfifo g_rxfifos[PSP_USB_MAX_ENDPOINTS];


/* External functions */
extern int sceKernelPowerLock (int arg);
extern int sceKernelPowerUnlock (int arg);

extern void sceSysregUsbIoEnable (void);
extern void sceSysregUsbIoDisable (void);
extern void sceSysregUsbBusClockEnable (void);
extern void sceSysregUsbBusClockDisable (void);
extern void sceSysregUsbAcquireIntr (int intrno);
extern int sceSysregUsbQueryIntr (void);
extern void sceSysregUsbResetDisable (void);
extern int sceSysreg_driver_A46E9CA8 (void);
extern int sceSysreg_driver_8D0FED1E (void);
extern void sceSysregUsbResetEnable (void);
extern int sceSysregUsbClkEnable (int no);
extern int sceSysregUsbClkDisable (int no);
extern int sceSysregUsbGetConnectStatus (void);

extern int sceKernelCancelEventFlag (int evid, unsigned int bits);

extern int sceIdStorageIsFormatted (void);
extern int sceIdStorageGetLeafSize (void);

/* Functions */

void usb_memcpy (void *dest, void *src, int size);
int refresh_usb_event_flag (void);
int check_device_class (void);
int assign_interfaces (void);
void assign_endpoints (void);
int load_device_configuration (unsigned int productId);
int make_configpacket (void);
int process_request (void);
void process_request_default (void);
int change_setting (int interface_num, int alternate_setting, struct ConfigDescriptor *conf);

int register_sysevent_handler (void);
int unregister_sysevent_handler (void);

int usb_clear_fifo (int endpoint);

int usb_bus_driver_start (int size, void *args);
int usb_bus_driver_stop (int size, void *args);

int clear_registered_protocol_drivers (void);
int check_registered_protocol_drivers (void);


int usb_cancel_all (int endpoint);
void usb_resume (void);
void usb_disconnect (void);
void usb_stop (void);
#endif /* __USB_INTERNAL_H */
