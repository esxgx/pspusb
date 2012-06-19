#include <pspkernel.h>
#include <pspsysevent.h>

#include "usb.h"
#include "usb_internal.h"

/* Variables */
static int usb_sysevent_handler (int ev_id, char* ev_name, void* param, int* result);
static PspSysEventHandler g_sysevent_handler = {    /* 0xAA4C */
    64,
    "SceUsb",
    0x00FFFF00,
    &usb_sysevent_handler,
    0,
    0,
    NULL
};

/* Function Prototypes */
static int usb_sysevent_resume_handler (void);
static int usb_sysevent_suspend_handler (void);


/* Subroutine at 0x7514 */
int register_sysevent_handler (void)
{
  int ret;
  ret = sceKernelRegisterSysEventHandler (&g_sysevent_handler);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot register system event handler : 0x%08x\n", ret);
  }
  return ret;
}


/* Subroutine at 0x7564 */
int unregister_sysevent_handler (void)
{
  int ret;
  ret = sceKernelUnregisterSysEventHandler (&g_sysevent_handler);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot unregister system event handler : 0x%08x\n", ret);
  }
  return ret;
}

/* Subroutine at 0x75B4 */
static
int usb_sysevent_handler (int ev_id, char* ev_name, void* param, int* result)
{
  switch (ev_id) {
  case 0x400E: /* Suspend (power down) */
    usb_sysevent_suspend_handler ();
    break;
  case 0x1000E: /* Resume */
    usb_sysevent_resume_handler ();
    break;
  }
  return 0;
}

/* Subroutine at 0x7604 */
static
int usb_sysevent_suspend_handler (void)
{
  if (g_prev_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED) return 0;
  g_prev_cable_connected = PSP_USB_STATUS_CABLE_CONNECTED;

  if (g_usb_activated == PSP_USB_STATUS_ACTIVATED
      && g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED) {
    usb_disconnect ();
  }
  if (g_connection_status == PSP_USB_STATUS_CONNECTION_ESTABLISHED) {
    if (g_started_drivers) {
      struct UsbDriver *current;
      for (current = g_started_drivers; current; current = current->link) {
        if (current->detach) {
          current->detach ();
        }
      }
    }
    if (g_total_endpoints > 0) {
      int i;
      for (i = 0; i < g_total_endpoints; i++) {
        usb_cancel_all (i);
      }
    }
    g_connection_status = PSP_USB_STATUS_CONNECTION_NEW;
  }
  if (g_devdesc_hi) g_usb_version = 2;
  sceKernelDisableIntr (PSP_USB_IRQ_DISCONNECT);
  sceKernelDisableIntr (PSP_USB_IRQ_CONNECT);
  sceSysregUsbIoDisable ();
  sceSysregUsbBusClockDisable ();
  sceSysregUsbClkDisable (9);
  g_cable_connected = PSP_USB_STATUS_CABLE_DISCONNECTED;
  refresh_usb_event_flag ();
  return 0;
}

/* Subroutine at 0x7740 */
static
int usb_sysevent_resume_handler (void)
{
  int ret;

  if (g_prev_cable_connected == PSP_USB_STATUS_CABLE_DISCONNECTED) return 0;
  _sw (_lw (IO_PORT_BC000050) | 0x20000, IO_PORT_BC000050);
  ret = sceSysregUsbQueryIntr ();
  sceSysregUsbAcquireIntr (ret);
  SYNC ();
  sceSysregUsbIoEnable ();
  ret = sceSysregUsbGetConnectStatus ();
  g_prev_cable_connected = PSP_USB_STATUS_CABLE_DISCONNECTED;
  if (ret) g_cable_connected = PSP_USB_STATUS_CABLE_CONNECTED;
  refresh_usb_event_flag ();
  sceKernelEnableIntr (PSP_USB_IRQ_CONNECT);
  sceKernelEnableIntr (PSP_USB_IRQ_DISCONNECT);
  if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
    sceSysregUsbClkEnable (9);
    sceSysregUsbBusClockEnable ();
    if (g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED) {
      usb_resume ();
    }
  }
  return 0;
}

