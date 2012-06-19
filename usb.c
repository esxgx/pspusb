#include <pspkernel.h>
#include <string.h>

#include "usb.h"
#include "usb_internal.h"

PSP_MODULE_INFO ("sceUSB_Driver", 0x1007, 1, 2);





/* Global variables */
struct usb_queue g_queues[PSP_USB_MAX_ENDPOINTS];     /* 0xA884 */

int g_usb_version;                            /* 0xA960 */
int g_prev_cable_connected = PSP_USB_STATUS_CABLE_CONNECTED; /* 0xA964 */
int g_usb_activated = PSP_USB_STATUS_DEACTIVATED;            /* 0xA968 */
int g_cable_connected = PSP_USB_STATUS_CABLE_DISCONNECTED;   /* 0xA96C */
int g_connection_status = PSP_USB_STATUS_CONNECTION_NEW;     /* 0xA970 */
int g_total_interfaces;                       /* 0xA974 */
int g_total_endpoints;                        /* 0xA978 */

SceUID g_mainsema;                            /* 0xA9EC */
static SceUID g_usbeventflag;                 /* 0xA9F0 */

struct usb_endpoint g_endpoints[PSP_USB_MAX_ENDPOINTS + 1];   /* 0xAAC0 */
struct usb_rxfifo g_rxfifos[PSP_USB_MAX_ENDPOINTS];           /* 0xAC28 */

static short g_clearedendpoints;              /* 0xACDC */
static short g_stalledendpoints;              /* 0xACDE */
static short g_ace0;                          /* 0xACE0 */
static short g_isochronoustransfers;          /* 0xACE2 */
static SceUID g_swfifofpl;                    /* 0xACE4 */
static SceUID g_mainalarm;                    /* 0xACE8 */
static volatile struct usb_port *g_defaultport;        /* 0xACEC */

static struct usb_port g_ports[21]; /* 0xAD00 */



/* Static functions */

static int check_fifos (void);
static int check_request (struct UsbbdDeviceRequest *req);

static int usb_interrupt_handler (void);
static int usb_connect_interrupt_handler (void);
static int usb_disconnect_interrupt_handler (void);
static int usb_ready_interrupt_handler (void);
static int usb_resume_interrupt_handler (void);

static int check_fifo (struct UsbDriver *driver, struct DeviceDescriptor *devp, struct UsbConfiguration *confp);

static int on_module_start (void);
static int on_module_stop (void);

static int release_bus_driver (void);
static int start_bus_driver (void);


static int queue_pop (struct usb_queue *queue, int retcode);
static void prepare_send (int endpnum);

static void cancel_recv_all (void);
static void configure_rxdma_fifo (struct usb_endpoint *t, int usefirst);

static int get_devrequest (void);
static void check_stall_or_clear (void);
static int sub_66F8 (struct usb_endpoint *t);
static void setup_recv_all (void);


static int on_send_packet (int endpnum);
static int on_receive_packet (int endpnum);


static int flush_tx_data (struct usb_endpoint *t);


static void cancel_send (int endpnum);
static void prepare_recv (int endpnum);

static void cancel_recv (int endpnum);
static int flush_fifo_data (int endpnum);

static int cancel_current_transaction (int endpoint, int retcode);
static int configure_rxdma (struct usb_endpoint *t);

static SceUInt endpoint_in_alarm_handler (void *common);
static SceUInt endpoint_out_alarm_handler (void *common);

static void resume_handler (void);

static void suspend_usb (void);

static int prepare_fifos (void);

static void set_usb_version (int usb_version);

static int configure_txdma (struct usb_endpoint *endp);


/* Subroutine at 0x0000 */
int sceUsbActivate (unsigned int productId)
{
  SceUInt timeout = DEFAULT_USB_TIMEOUT;
  register int k1;
  int ret;

  SAVE_K1 ();
  if (sceKernelIsIntrContext ()) {
    Kprintf ("usbbd ERROR : illegal context\n");
    ret = SCE_KERNEL_ERROR_ILLEGAL_CONTEXT;
  } else {
    ret = sceKernelWaitSema (g_mainsema, 1, &timeout);
    if (ret < 0) {
      Kprintf ("usbbd ERROR : Activate() wait semaphore : 0x%08x\n", ret);
    } else {
      if (!g_busdrv_started) {
        Kprintf ("usbbd ERROR : bus driver isn't started\n");
        ret = PSP_USB_ERROR_BUS_DRIVER_NOT_STARTED;
      } else {
        if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
          Kprintf ("usbbd ERROR : already activated\n");
          ret = PSP_USB_ERROR_ALREADY_DONE;
        } else {
          if (!g_started_drivers) {
            Kprintf ("usbbd ERROR : no registered protocol driver\n");
            ret = PSP_USB_ERROR_DRIVER_NOT_FOUND;
          } else {
            if (check_device_class () < 0) {
              ret = PSP_USB_ERROR_INVALID_ARGUMENT;
            } else {
              if (g_total_interfaces > PSP_USB_MAX_INTERFACES || g_total_endpoints > PSP_USB_MAX_ENDPOINTS) {
                Kprintf ("usbbd ERROR : number of interfaces %d(max = %d) or\n number of endpoints %d(max = %d) is over limitation\n",
                    g_total_interfaces, PSP_USB_MAX_INTERFACES, g_total_endpoints, PSP_USB_MAX_ENDPOINTS);
                ret = PSP_USB_ERROR_ARGUMENT_EXCEEDED_LIMIT;
              } else {
                if (assign_interfaces () < 0) {
                  ret = PSP_USB_ERROR_INVALID_ARGUMENT;
                } else {
                  assign_endpoints ();
                  if (check_fifos () < 0) {
                    ret = PSP_USB_ERROR_MEMORY_EXHAUSTED;
                  } else {
                    if (load_device_configuration (productId) < 0) {
                      ret = PSP_USB_ERROR_MEMORY_EXHAUSTED;
                    } else {
                      if (make_configpacket () >= 0) {
                        g_request.endpoint = &g_ep0;
                        g_request.isControlRequest = 1;
                        g_request.returnCode = PSP_USB_RETCODE_SUCCESS;
                        g_request.data = NULL;
                        g_request.size = 0;
                        g_request.onComplete = NULL;
                        g_request.next = NULL;
                        g_request.unused = NULL;
                        g_request.transmitted = 0;

                        ret = prepare_fifos ();
                        if (ret < 0) {
                          if (sceKernelDeleteFpl (g_ep0fpl) < 0)
                            Kprintf ("usbbd ERROR : cannot delete FPL for EP0 buffer\n");
                        } else {
                          int intr;
                          g_usb_activated = PSP_USB_STATUS_ACTIVATED;
                          refresh_usb_event_flag ();

                          intr = sceKernelCpuSuspendIntr ();
                          if (g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED) {
                            if (g_prev_cable_connected == PSP_USB_STATUS_CABLE_DISCONNECTED) {
                              usb_resume ();
                            }
                          }
                          sceKernelCpuResumeIntr (intr);
                          ret = 0;
                        }
                      } else {
                        ret = PSP_USB_ERROR_MEMORY_EXHAUSTED;
                      }

                      if (ret < 0) {
                        if (sceKernelDeleteFpl (g_devdescfpl) < 0)
                          Kprintf ("usbbd ERROR : cannot delete FPL for device desc\n");
                        if (sceKernelDeleteFpl (g_fsconffpl) < 0)
                          Kprintf ("usbbd ERROR : cannot delete FPL for fs config\n");
                        if (g_conf_hi) {
                          if (sceKernelDeleteFpl (g_hsconffpl) < 0)
                            Kprintf ("usbbd ERROR : cannot delete FPL for hs config\n");
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
      sceKernelSignalSema (g_mainsema, 1);
    }
  }

  RESTORE_K1 ();
  return ret;
}

/* Subroutine at 0x0318 */
int sceUsbDeactivate (void)
{
  SceUInt timeout = DEFAULT_USB_TIMEOUT;
  register int k1;
  int ret;

  SAVE_K1 ();
  if (sceKernelIsIntrContext ()) {
    Kprintf ("usbbd ERROR : illegal context\n");
    ret = SCE_KERNEL_ERROR_ILLEGAL_CONTEXT;
  } else {
    ret = sceKernelWaitSema (g_mainsema, 1, &timeout);
    if (ret < 0) {
      Kprintf ("usbbd ERROR : Deactivate() wait semaphore : 0x%08x\n", ret);
    } else {
      if (!g_busdrv_started) {
        Kprintf ("usbbd ERROR : bus driver isn't started\n");
        ret = PSP_USB_ERROR_BUS_DRIVER_NOT_STARTED;
      } else {
        if (g_usb_activated == PSP_USB_STATUS_DEACTIVATED) {
          Kprintf ("usbbd ERROR : already deactivated\n");
          ret = PSP_USB_ERROR_ALREADY_DONE;
        } else {
          int intr;

          g_usb_activated = PSP_USB_STATUS_DEACTIVATED;
          intr = sceKernelCpuSuspendIntr ();
          if (g_prev_cable_connected == PSP_USB_STATUS_CABLE_DISCONNECTED) {
            if (g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED) {
              usb_disconnect ();
            }
          }
          if (g_connection_status == PSP_USB_STATUS_CONNECTION_ESTABLISHED) {
            struct UsbDriver *driver;

            for (driver = g_started_drivers; driver; driver = driver->link) {
              if (driver->detach) driver->detach ();
            }

            if (g_total_endpoints > 0) {
              int i;
              for (i = 0; i < g_total_endpoints; i++)
                usb_cancel_all (i);
            }
          }
          sceKernelCpuResumeIntr (intr);

          usb_stop ();

          if (sceKernelDeleteFpl (g_ep0fpl) < 0)
            Kprintf ("usbbd ERROR : cannot delete FPL for EP0 buffer\n");

          if (sceKernelDeleteFpl (g_devdescfpl) < 0)
            Kprintf ("usbbd ERROR : cannot delete FPL for device desc\n");

          if (sceKernelDeleteFpl (g_fsconffpl) < 0)
            Kprintf ("usbbd ERROR : cannot delete FPL for fs config\n");

          if (g_conf_hi) {
            if (sceKernelDeleteFpl (g_hsconffpl) < 0)
              Kprintf ("usbbd ERROR : cannot delete FPL for hs config\n");
          }

          g_connection_status = PSP_USB_STATUS_CONNECTION_NEW;
          g_string = NULL;
          g_vendor_desc = NULL;
          g_product_desc = NULL;

          refresh_usb_event_flag ();

          ret = 0;
        }
      }
      sceKernelSignalSema (g_mainsema, 1);
    }
  }

  RESTORE_K1 ();
  return ret;
}


/* Subroutine at 0x0598 */
int sceUsbWaitState (unsigned int state, unsigned int waitMode, SceUInt *timeout)
{
  register int k1;
  int ret;

  if (!g_busdrv_started) {
    Kprintf ("usbbd ERROR : bus driver isn't started\n");
    ret = PSP_USB_ERROR_BUS_DRIVER_NOT_STARTED;
  } else {
    SAVE_K1 ();
    if (sceKernelIsIntrContext ()) {
      Kprintf ("usbbd ERROR : illegal context\n");
      ret = SCE_KERNEL_ERROR_ILLEGAL_CONTEXT;
    } else {
      if (IS_USER_MODE ()) {
        if (CHECK_RANGE (timeout, 4)) {
          RESTORE_K1 ();
          return SCE_KERNEL_ERROR_ILLEGAL_ADDR;
        }
      }
      if (waitMode == PSP_EVENT_WAITAND) {
        int group = 0;
        int temp = state;

        while (group < 4) {
          int i, count = 0;

          for (i = 4; i > 0; i--) {
            count += (temp & 1);
            temp >>= 1;
          }
          group++;
          if (count >= 2) {
            Kprintf ("usbbd ERROR : sceUsbWaitState() : Impossible combination state=0x%04x, waitMode=AND\n", state);
            RESTORE_K1 ();
            return PSP_USB_ERROR_INVALID_ARGUMENT;
          }
        }
        if ((state & PSP_USB_STATUS_DEACTIVATED) != 0 &&
            (state & (PSP_USB_STATUS_CONNECTION_ESTABLISHED | PSP_USB_STATUS_CONNECTION_SUSPENDED)) != 0) {
          Kprintf ("usbbd ERROR : sceUsbWaitState() : Impossible combination state=0x%04x, waitMode=AND\n", state);
          RESTORE_K1 ();
          return PSP_USB_ERROR_INVALID_ARGUMENT;
        }
      }

      ret = sceKernelWaitEventFlag (g_usbeventflag, state, waitMode, NULL, timeout);
      if (ret < 0) {
        if (ret == SCE_KERNEL_ERROR_WAIT_TIMEOUT) {
          ret = PSP_USB_ERROR_WAIT_TIMEOUT;
        } else if (ret == SCE_KERNEL_ERROR_WAIT_CANCEL) {
          ret = PSP_USB_ERROR_WAIT_CANCEL;
        } else {
          Kprintf ("usbbd ERROR : sceUsbWaitState() : error at waiting event 0x%08x\n", ret);
        }
      } else {
        ret = g_cable_connected | g_usb_activated | g_connection_status;
      }

    }
    RESTORE_K1 ();
  }
  return ret;
}


/* Subroutine at 0x0804 */
int refresh_usb_event_flag (void)
{
  sceKernelClearEventFlag (g_usbeventflag, 0);
  return sceKernelSetEventFlag (g_usbeventflag, g_usb_activated | g_cable_connected | g_connection_status);
}



/* Subroutine at 0x23CC */
int sceUsbbdReqSend (struct UsbbdDeviceRequest *req)
{
  struct UsbEndpoint *endp;
  struct usb_queue *queue;
  unsigned int addr;
  int ret, intr;

  ret = check_request (req);
  if (ret < 0) return ret;
  endp = req->endpoint;
  queue = &g_queues[endp->endpointNumber];

  if (queue->endpoint_type == USB_ENDPOINT_TYPE_CONTROL) {
    req->isControlRequest = 1;
  }
  req->returnCode = PSP_USB_RETCODE_SEND;
  addr = (unsigned int) req->data;
  req->physicalAddress = (void *) PHYSICAL_ADDRESS (addr);
  req->transmitted = 0;

  intr = sceKernelCpuSuspendIntr ();
  if (queue->last) {
    queue->last->next = req;
  } else {
    queue->first = req;
  }
  while (req->next) {
    req = req->next;
    if (queue->endpoint_type == USB_ENDPOINT_TYPE_CONTROL) {
      req->isControlRequest = 1;
    }
    req->transmitted = 0;
    addr = (unsigned int) req->data;
    req->physicalAddress = (void *) PHYSICAL_ADDRESS (addr);
    req->returnCode = PSP_USB_RETCODE_SEND;
  }
  queue->last = req;
  prepare_send (endp->endpointNumber);
  sceKernelCpuResumeIntr (intr);

  return 0;
}

/* Subroutine at 0x24EC */
int sceUsbbdReqRecv (struct UsbbdDeviceRequest *req)
{
  struct UsbEndpoint *endp;
  struct usb_queue *queue;
  unsigned int addr;
  int ret, intr;

  ret = check_request (req);
  if (ret < 0) {
    return ret;
  }
  endp = req->endpoint;
  queue = &g_queues[endp->endpointNumber];
  if (queue->endpoint_type == USB_ENDPOINT_TYPE_CONTROL) {
    req->isControlRequest = 1;
  }
  req->returnCode = PSP_USB_RETCODE_RECV;
  addr = (unsigned int) req->data;
  req->physicalAddress = (void *) PHYSICAL_ADDRESS (addr);
  req->transmitted = 0;
  intr = sceKernelCpuSuspendIntr ();
  if (queue->last) {
    queue->last->next = req;
  } else {
    queue->first = req;
  }
  while (req->next) {
    req = req->next;
    if (queue->endpoint_type == USB_ENDPOINT_TYPE_CONTROL) {
      req->isControlRequest = 1;
    }
    req->transmitted = 0;
    addr = (unsigned int) req->data;
    req->physicalAddress = (void *) PHYSICAL_ADDRESS (addr);
    req->returnCode = PSP_USB_RETCODE_RECV;
  }
  queue->last = req;
  prepare_recv (endp->endpointNumber);
  sceKernelCpuResumeIntr (intr);

  return 0;
}

/* Subroutine at 0x2610 */
int sceUsbbdReqCancel (struct UsbbdDeviceRequest *req)
{
  if (req) {
    if (!req->endpoint) {
      Kprintf ("usbbd ERROR : Null pointer\n");
      return PSP_USB_ERROR_INVALID_ARGUMENT;
    } else {
      struct usb_queue *queue = &g_queues[req->endpoint->endpointNumber];
      struct UsbbdDeviceRequest *current, *previous;
      int intr;

      intr = sceKernelCpuSuspendIntr ();
      current = previous = queue->first;
      if (queue->first) {
        while (current) {
          if (current == req) {
            if (!current) break;
            if (queue->first == current) {
              if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
                if (g_cable_connected != PSP_USB_STATUS_CABLE_CONNECTED ||
                    g_connection_status == PSP_USB_STATUS_CONNECTION_SUSPENDED ||
                    g_cable_connected == g_prev_cable_connected) {
                  if (queue->first) queue_pop (queue, PSP_USB_RETCODE_CANCEL);
                  break;
                } else {
                  if (cancel_current_transaction (req->endpoint->endpointNumber, PSP_USB_RETCODE_CANCEL) == 0) {
                    queue_pop (queue, PSP_USB_RETCODE_CANCEL);
                    break;
                  } else {
                    sceKernelCpuResumeIntr (intr);
                    return PSP_USB_ERROR_DRIVER_IN_PROGRESS;
                  }
                }
              } else {
                if (queue->first) queue_pop (queue, PSP_USB_RETCODE_CANCEL);
                break;
              }
            } else {
              current->returnCode = PSP_USB_RETCODE_CANCEL;
              previous->next = current->next;
              current->endpoint->transmittedBytes += current->transmitted;
              current->next = NULL;
              if (current->onComplete) {
                current->onComplete (current);
              }
              break;
            }
          } else {
            previous = current;
            current = current->next;
          }
        }
      }

      sceKernelCpuResumeIntr (intr);
    }
  }
  return 0;
}

/* Subroutine at 0x2790 */
int usb_cancel_all (int endpoint)
{
  unsigned int intr;
  struct usb_queue *queue;

  queue = &g_queues[endpoint];
  intr = sceKernelCpuSuspendIntr ();
  if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
    if (g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED) {
      if (g_connection_status != PSP_USB_STATUS_CONNECTION_SUSPENDED) {
        if (g_prev_cable_connected != g_cable_connected) {
          if (queue->first) {
            if (cancel_current_transaction (endpoint, PSP_USB_RETCODE_CANCEL_ALL)) {
              sceKernelCpuResumeIntr (intr);
              return PSP_USB_ERROR_DRIVER_IN_PROGRESS;
            }
          }
        }
      }
    }
  }
  while (queue->first) {
    queue_pop (queue, PSP_USB_RETCODE_CANCEL_ALL);
  }

  sceKernelCpuResumeIntr (intr);
  return 0;
}


/* Subroutine at 0x28B0 */
static
int queue_pop (struct usb_queue *queue, int retcode)
{
  struct UsbbdDeviceRequest *req, *next;


  req = queue->first;
  if (req) {
    next = req->next;
    if (next) {
      queue->first = next;
      if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
        if (g_prev_cable_connected != PSP_USB_STATUS_CABLE_CONNECTED) {
          if (retcode != PSP_USB_RETCODE_CANCEL_ALL) {
            if (next->returnCode == PSP_USB_RETCODE_RECV) {
              prepare_recv (req->endpoint->endpointNumber);
            } else {
              prepare_send (req->endpoint->endpointNumber);
            }
          }
        }
      }
      req->returnCode = retcode;
    } else {
      queue->first = queue->last = NULL;
      if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
        if (g_prev_cable_connected != PSP_USB_STATUS_CABLE_CONNECTED) {
          if (queue->endpoint_dir != 0) {
            cancel_send (req->endpoint->endpointNumber);
          } else {
            cancel_recv (req->endpoint->endpointNumber);
          }
        }
      }
      if (retcode == PSP_USB_RETCODE_CANCEL_ALL)
        retcode = PSP_USB_RETCODE_CANCELTRANSMISSION;
      req->returnCode = retcode;
    }
    req->next = NULL;
    req->endpoint->transmittedBytes += req->transmitted;
    if (req->onComplete) {
      req->onComplete (req);
    }
  }
  return 0;
}

/* Subroutine at 0x29E4 */
int usb_bus_driver_start (int size, void *args)
{
  int ret;

  g_started_drivers = NULL;
  g_usb_version = -1;
  g_usb_activated = PSP_USB_STATUS_DEACTIVATED;
  g_cable_connected = PSP_USB_STATUS_CABLE_DISCONNECTED;
  g_connection_status = PSP_USB_STATUS_CONNECTION_NEW;
  g_queues[0].endpoint = &g_ep0;
  g_queues[0].endpoint_dir = 255;
  g_total_interfaces = 0;
  g_total_endpoints = 1;
  g_string = NULL;
  g_vendor_desc = NULL;
  g_product_desc = NULL;
  g_ep0.endpointNumber = 0;
  g_ep0.driverEndpointNumber = 0;
  g_ep0.transmittedBytes = 0;
  g_queues[0].endpoint_type = USB_ENDPOINT_TYPE_CONTROL;
  g_queues[0].last = NULL;
  g_queues[0].first = NULL;

  ret = sceKernelCreateEventFlag ("SceUsb", PSP_EVENT_WAITMULTIPLE | 1, 0, 0);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot delete event flag\n");
    return ret;
  }
  g_usbeventflag = ret;

  refresh_usb_event_flag ();
  ret = start_bus_driver ();
  if (ret < 0) {
    if (sceKernelDeleteEventFlag (g_usbeventflag) < 0) {
      Kprintf ("usbbd ERROR : cannot delete event flag\n");
    }
    return ret;
  }

  return 0;
}

/* Subroutine at 0x2AF4 */
int usb_bus_driver_stop (int size, void *args)
{
  if (g_started_drivers) {
    Kprintf ("usbbd ERROR : bus driver is used by protocol drivers\n");
    return PSP_USB_ERROR_DRIVER_IN_PROGRESS;
  }
  release_bus_driver ();
  if (sceKernelDeleteEventFlag (g_usbeventflag) < 0) {
    Kprintf ("usbbd ERROR : cannot delete event flag\n");
  }
  g_connection_status = PSP_USB_STATUS_CONNECTION_NEW;
  g_prev_cable_connected = PSP_USB_STATUS_CABLE_CONNECTED;
  g_usb_activated = PSP_USB_STATUS_DEACTIVATED;
  g_cable_connected = PSP_USB_STATUS_CABLE_DISCONNECTED;
  return 0;
}


/* Subroutine at 0x2B90 */
int module_stop (SceSize args, void *argp)
{
  SceUInt timeout = DEFAULT_USB_TIMEOUT;
  int ret;

  ret = sceKernelWaitSema (g_mainsema, 1, &timeout);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : Exit() wait semaphore : 0x%08x\n", ret);
    return 1;
  }
  if (g_busdrv_started == 1) {
    Kprintf ("usbbd ERROR : bus driver is not stopped\n");
    sceKernelSignalSema (g_mainsema, 1);
    return 1;
  }
  check_registered_protocol_drivers ();
  on_module_stop ();

  ret = sceKernelDeleteSema (g_mainsema);
  if (ret < 0) Kprintf ("usbbd ERROR : Cannot delete semaphore : 0x%08x\n", ret);
  g_mainsema = 0;
  return 0;
}

/* Subroutine at 0x2C60 */
int module_reboot_before (void)
{
  struct UsbDriverName list;
  int ret;

  if (g_busdrv_started == 1) {
    if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
      sceUsbDeactivate ();
    }
    list.size = sizeof (struct UsbDriverName);
    while (1) {
      ret = sceUsbGetDrvList (1, &list, 1);
      if (ret < 0) break;
      ret = sceUsbStop (list.name, 0, NULL);
      if (ret < 0) {
        Kprintf ("usbbd ERROR : Cannot stop %s : 0x%08x\n", list.name, ret);
      }
    }
    ret = sceUsbStop (PSP_USB_BUS_DRIVERNAME, 0, NULL);
    if (ret < 0) Kprintf ("usbbd ERROR : Cannot stop %s : 0x%08x\n", PSP_USB_BUS_DRIVERNAME, ret);
  }
  ret = sceKernelDeleteSema (g_mainsema);
  if (ret < 0) Kprintf ("usbbd ERROR : Cannot delete semaphore : 0x%08x\n", ret);
  g_mainsema = 0;
  return 0;
}

/* Subroutine at 0x2D74 */
int sceUsbGetState (void)
{
  return (g_usb_activated | g_cable_connected | g_connection_status);
}

/* Subroutine at 0x2D94 */
int sceUsbWaitCancel (void)
{
  register int k1;
  int ret;

  if (!g_busdrv_started) {
    Kprintf ("usbbd ERROR : bus driver isn't started\n");
    ret = PSP_USB_ERROR_BUS_DRIVER_NOT_STARTED;
  } else {
    SAVE_K1 ();
    sceKernelCancelEventFlag (g_usbeventflag, g_usb_activated | g_cable_connected | g_connection_status);
    ret = 0;
    RESTORE_K1 ();
  }
  return ret;
}

/* Subroutine at 0x2E0C */
static
int change_all_settings (struct ConfigDescriptor *conf)
{
  int interf_num;
  for (interf_num = 0; interf_num < conf->bNumInterfaces; interf_num++) {
    if (change_setting (interf_num, -1, conf) < 0)
      return -1;
  }
  return 0;
}


/* Subroutine at 0x2E74 */
int sceUsbbdReqCancelAll (struct UsbEndpoint *endp)
{
  if (!endp) {
    Kprintf ("usbbd ERROR : Null pointer\n");
    return PSP_USB_ERROR_INVALID_ARGUMENT;
  }
  if (endp->endpointNumber == 0) {
    Kprintf ("usbbd ERROR : Cannot cancel request of EP0\n");
    return PSP_USB_ERROR_INVALID_ARGUMENT;
  }

  usb_cancel_all (endp->endpointNumber);
  return 0;
}

/* Subroutine at 0x2EC4 */
static
void usb_cancel_all_skip_first (int endpoint)
{
  struct usb_queue *queue = &g_queues[endpoint];
  int intr;
  intr = sceKernelCpuSuspendIntr ();
  while (queue->first) {
    queue_pop (queue, PSP_USB_RETCODE_CANCEL_ALL);
  }
  sceKernelCpuResumeIntr (intr);
}

/* Subroutine at 0x2F3C */
int module_start (SceSize args, void *argp)
{
  SceUID ret;
  Kprintf ("USB Open Edition driver\nCopyright (c) hnaves (hsnaves@gmail.com)\n");
  ret = sceKernelCreateSema (MAIN_SEMAPHORE_NAME, 1, 1, 1, 0);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : Cannot create semaphore : 0x%08x\n", ret);
    return 1;
  }
  g_mainsema = ret;
  on_module_start ();
  clear_registered_protocol_drivers ();
  return 0;
}

/* Subroutine at 0x2FA8 */
static
int check_request (struct UsbbdDeviceRequest *req)
{
  if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
    if (g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED) {
      if (g_connection_status != PSP_USB_STATUS_CONNECTION_SUSPENDED) {
        if (g_prev_cable_connected != PSP_USB_STATUS_CABLE_CONNECTED) {
          if (req) {
            unsigned int addr = (unsigned int) req->data;
            if (CHECK_DDR_ADDRESS (addr)) {
              int endpoint = req->endpoint->endpointNumber;
              while (1) {
                if (endpoint != req->endpoint->endpointNumber)
                  return PSP_USB_ERROR_INVALID_ARGUMENT;
                if (req->returnCode > 0)
                  return PSP_USB_ERROR_ALREADY_DONE;
                if (req->size == 0) {
                  if (!req->isControlRequest)
                    return PSP_USB_ERROR_INVALID_ARGUMENT;
                }

                req = req->next;
                if (!req) return 0;
              }
            }
            Kprintf ("usbbd ERROR : Not DDR address 0x%p\n", req->data);
          }
          return PSP_USB_ERROR_INVALID_ARGUMENT;
        }
      }
    }
  }
  return PSP_USB_ERROR_BUS_DRIVER_NOT_STARTED;
}

/* Subroutine at 0x30A8 */
static
int prepare_fifos (void)
{
  int i, total_size, num_elements = 1;
  int ret;
  char *ptr;

  total_size = ALIGN_UP (g_rxfifos[0].fifo_size, 63);
  for (i = 1; i <= 8; i++) {
    if (g_rxfifos[i].fifo_size > 0) {
      int size = PSP_USB_FIFO_ELEMENTS * g_rxfifos[i].fifo_size;
      if (size > PSP_USB_MAX_TXFIFO_SIZE)
        size = PSP_USB_MAX_TXFIFO_SIZE;
      g_rxfifos[i].fifo_size = ALIGN_UP (size, 63);
      total_size += g_rxfifos[i].fifo_size;
      num_elements += PSP_USB_FIFO_ELEMENTS;
    }
  }

  g_swfifofpl = sceKernelCreateFpl ("SceUsbSoftwareFIFO", PSP_MEMORY_PARTITION_KERNEL, 0x00000100, (total_size + num_elements * sizeof (struct rxfifo_element)), 1, NULL);
  if (g_swfifofpl < 0) {
    Kprintf ("usbbd ERROR : cannot create FPL to SW FIFO\n");
    return PSP_USB_ERROR_MEMORY_EXHAUSTED;
  }

  ret = sceKernelTryAllocateFpl (g_swfifofpl, (void **) &ptr);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot allocate memory to SW FIFO\n");
    ret = sceKernelDeleteFpl (g_swfifofpl);
    if (ret < 0) {
      Kprintf ("usbbd ERROR : cannot delete FPL for SW FIFO\\n");
    }
    return PSP_USB_ERROR_MEMORY_EXHAUSTED;
  }

  for (i = 0; i <= 8; i++) {
    if (g_rxfifos[i].fifo_size > 0) {
      g_rxfifos[i].buffer = ptr;
      ptr += g_rxfifos[i].fifo_size;
    }
  }

  g_rxfifos[0].elements = (struct rxfifo_element *) ptr;
  ptr += sizeof (struct rxfifo_element);

  for (i = 1; i <= 8; i++) {
    if (g_rxfifos[i].fifo_size > 0) {
      g_rxfifos[i].elements = (struct rxfifo_element *) ptr;
      ptr += PSP_USB_FIFO_ELEMENTS * sizeof (struct rxfifo_element);
    }
  }

  sceSysregUsbClkEnable (9);
  sceSysregUsbBusClockEnable ();
  return 0;
}

/* Subroutine at 0x3228 */
static
int check_fifos (void)
{
  int i;

  for (i = 0; i <= 8; i++) {
    g_rxfifos[i].fifo_size = 0;
  }

  if (g_started_drivers) {
    struct UsbDriver *driver;
    for (driver = g_started_drivers; driver; driver = driver->link) {
      struct DeviceDescriptor *devp_hi = driver->descriptor_hi;
      if (devp_hi) {
        if (check_fifo (driver, devp_hi, driver->configuration_hi) < 0)
          return -1;
      }
      if (check_fifo (driver, driver->descriptor, driver->configuration) < 0)
        return -1;
    }
  }

  return 0;
}

/* Subroutine at 0x32C0 */
static
int check_fifo (struct UsbDriver *driver, struct DeviceDescriptor *devp, struct UsbConfiguration *confp)
{
  struct ConfigDescriptor *cd;
  struct InterfaceSettings *settings;
  int in_size, max_in_size, total_in_size;
  int out_size, max_out_size, total_out_size;

  if (devp->bMaxPacketSize0 > g_rxfifos[0].fifo_size) {
    g_rxfifos[0].fifo_size = devp->bMaxPacketSize0;
  }

  total_in_size = total_out_size = devp->bMaxPacketSize0;

  cd = confp->configDescriptors;
  settings = cd->settings;
  if (cd->bNumInterfaces > 0) {
    int i, j, k, l;
    for (i = 0; i < cd->bNumInterfaces; i++) {
      struct InterfaceDescriptor *id = settings[i].descriptors;

      in_size = 0;
      out_size = 0;
      max_in_size = 0;
      max_out_size = 0;

      if (settings[i].numDescriptors > 0) {
        for (j = 0; j < settings[i].numDescriptors; j++) {
          struct EndpointDescriptor *ed = id[j].endpoints;
          if (id[j].bNumEndpoints > 0) {
            for (k = 0; k < id[j].bNumEndpoints; k++) {
              int size;

              if (ed[k].bEndpointAddress & USB_ENDPOINT_DIR_MASK) {
                if ((ed[k].bmAttributes & USB_ENDPOINT_TYPE_MASK) == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
                  size = ed[k].wMaxPacketSize << 1;
                } else {
                  size = ed[k].wMaxPacketSize;
                }
                in_size += size;
              } else {
                int driver_endpoint = (ed[k].bEndpointAddress & USB_ENDPOINT_ADDRESS_MASK);
                int endpoint = driver_endpoint;
                out_size += ed[k].wMaxPacketSize;
                if (driver->numEndpoints > 0) {
                  struct UsbEndpoint *endp = driver->endpoints;
                  for (l = 0; l < driver->numEndpoints; l++) {
                    if (endp[l].driverEndpointNumber == driver_endpoint) {
                      endpoint = endp[l].endpointNumber;
                      break;
                    }
                  }
                }
                if (g_rxfifos[endpoint].fifo_size < ed[k].wMaxPacketSize) {
                  g_rxfifos[endpoint].fifo_size = ed[k].wMaxPacketSize;
                }

              }

            }
          }

          if (max_in_size < in_size)
            max_in_size = in_size;
          if (max_out_size < out_size)
            max_out_size = out_size;
        }
      }

      total_in_size += max_in_size;
      total_out_size += max_out_size;
    }
  }
  if (total_in_size >= PSP_USB_MAX_TXFIFO_SIZE) {  /* EP IN (host point-of-view) */
    Kprintf ("usbbd ERROR : needed TxFIFO size is too large\n");
    return -1;
  }
  if (total_out_size >= PSP_USB_MAX_TXFIFO_SIZE) { /* EP OUT (host point-of-view) */
    Kprintf ("usbbd ERROR : needed RxFIFO size is too large\n");
    return -1;
  }
  return 0;
}


/* Subroutine at 0x34D8 */
void usb_disconnect (void)
{
  if (g_mainalarm >= 0) {
    sceKernelCancelAlarm (g_mainalarm);
    g_mainalarm = -1;
  }
  sceKernelDisableIntr (PSP_USB_IRQ_MAIN);
  sceKernelDisableIntr (PSP_USB_IRQ_READY);
  sceKernelDisableIntr (PSP_USB_IRQ_RESUME);
  if (g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED) {
    _sw (_lw (USB_PORT_0404) | 0x400, USB_PORT_0404);;
  }
  sceSysreg_driver_A46E9CA8 ();
  sceSysregUsbResetEnable ();
}



/* Subroutine at 0x382C */
int usb_clear_fifo (int endpoint)
{
  struct usb_endpoint *t;
  t = &g_endpoints[endpoint];
  if (g_usb_activated == PSP_USB_STATUS_ACTIVATED &&
     g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED &&
     g_connection_status != PSP_USB_STATUS_CONNECTION_SUSPENDED) {
    if (!((g_clearedendpoints >> endpoint) & 1)) {
      int intr;
      t->status |= PSP_USB_ENDPOINT_STATUS_CLEARED;
      if (endpoint == 0) {
        g_endpoints[9].status |= PSP_USB_ENDPOINT_STATUS_CLEARED;
      }

      intr = sceKernelCpuSuspendIntr ();
      if (sceUsbbdReqCancel (t->queue->first) >= 0) {
        if (t->queue->endpoint_dir == 0x80) {
          int address = USB_PORT_0200 + 32 * endpoint;
          t->status &= ~PSP_USB_ENDPOINT_STATUS_CLEARED;
          _sw (_lw (address) | 0x02, address);
          if (t->queue->first) {
            prepare_send (endpoint);
          }
        } else {
          struct usb_rxfifo *fifo = &g_rxfifos[endpoint];
          t->status &= ~PSP_USB_ENDPOINT_STATUS_HASDATA;
          fifo->start = fifo->end = 0;
          if (!(_lw (USB_PORT_0408) & 0x00008000)) {
            struct rxfifo_element *element = fifo->elements;
            g_clearedendpoints |= 1 << endpoint;
            t->unk4 = 16;
            if (element) {
              configure_rxdma_fifo (t, TRUE);
            }
          } else {
            if (endpoint == 0) {
              t->status &= ~PSP_USB_ENDPOINT_STATUS_CLEARED;
              t = &g_endpoints[9];
            }

            if (t->queue->endpoint_dir == 0xFF) {
              int address = USB_PORT_0000 + 32 * endpoint;
              _sw (_lw (address) | 0x02, address);
            }

            t->status &= ~PSP_USB_ENDPOINT_STATUS_CLEARED;
            if (t->queue->first) {
              if (t->queue->first->returnCode == PSP_USB_RETCODE_RECV) {
                prepare_recv (endpoint);
              } else {
                prepare_send (endpoint);
              }
            }
          }
        }
      }
      sceKernelCpuResumeIntr (intr);
    }
  } else {
    return PSP_USB_ERROR_BUS_DRIVER_NOT_STARTED;
  }
  return 0;
}

/* Subroutine at 0x3A60 */
static
int usb_stall (int endpoint)
{
  struct usb_endpoint *t;

  t = &g_endpoints[endpoint];
  if (g_usb_activated == PSP_USB_STATUS_ACTIVATED &&
     g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED &&
     g_connection_status != PSP_USB_STATUS_CONNECTION_SUSPENDED) {
    if (!((g_stalledendpoints >> endpoint) & 1)) {
      int intr;
      t->status |= PSP_USB_ENDPOINT_STATUS_STALLED;
      if (endpoint == 0) {
        g_endpoints[9].status |= PSP_USB_ENDPOINT_STATUS_STALLED;
      }

      intr = sceKernelCpuSuspendIntr ();
      if (sceUsbbdReqCancel (t->queue->first) >= 0) {
        cancel_recv_all ();
        if (t->queue->endpoint_dir != 0x80) {
          t->status &= ~PSP_USB_ENDPOINT_STATUS_HASDATA;
          g_rxfifos[endpoint].end = g_rxfifos[endpoint].start = 0;
        }

        if (!(_lw (USB_PORT_0408) & 0x00008000)) {
          g_stalledendpoints |= 1 << endpoint;
          if (g_rxfifos[endpoint].elements) {
            configure_rxdma_fifo (t, TRUE);
          }
        } else {

          if (t->queue->endpoint_dir != 0x80) {
            int address = USB_PORT_0200 + 32 * endpoint;
            t->status &= ~PSP_USB_ENDPOINT_STATUS_STALLED;
            _sw (_lw (address) | 0x01, address);
          }

          if (endpoint == 0) {
            t = &g_endpoints[9];
          }

          if (t->queue->endpoint_dir != 0x00) {
            int address = USB_PORT_0200 + 32 * endpoint;
            t->status &= ~PSP_USB_ENDPOINT_STATUS_STALLED;
            _sw (_lw (address) | 0x02, address);
            _sw (_lw (address) | 0x01, address);
          }

          if (t->queue->first) {
            if (t->queue->first->returnCode == PSP_USB_RETCODE_RECV) {
              prepare_recv (endpoint);
            } else {
              prepare_send (endpoint);
            }
          }
        }
      }
      sceKernelCpuResumeIntr (intr);
    }
  } else {
    return PSP_USB_ERROR_BUS_DRIVER_NOT_STARTED;
  }
  return 0;
}

/* Subroutine at 0x3C90 */
static
void cancel_recv_all (void)
{
  if (g_total_endpoints > 0) {
    int i, address = USB_PORT_0200;
    for (i = 0; i < g_total_endpoints; i++) {
      struct usb_queue *queue = &g_queues[i];
      if (queue->endpoint_dir != 0x80) {
        _sw (_lw (address) | 0x80, address);
      }
      address += 0x20;
    }
  }
}

/* Subroutine at 0x3CEC */
static
void prepare_send (int endpnum)
{
  struct usb_endpoint *t;
  if (endpnum == 0) {
    t = &g_endpoints[PSP_USB_MAX_ENDPOINTS];
  } else {
    t = &g_endpoints[endpnum];
  }

  if (t->status & (PSP_USB_ENDPOINT_STATUS_STALLED | PSP_USB_ENDPOINT_STATUS_CLEARED)) {
    cancel_send (endpnum);
    if (t->queue->endpoint_dir == 255) {
      cancel_recv (endpnum);
    }
  } else {
    int mask, bit;

    if (t->status & PSP_USB_ENDPOINT_STATUS_TRANSMITTING) return;
    if (endpnum == 0) {
      t->status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
      if (_lw (USB_PORT_0000) & 0x40) {
        _sw (_lw (USB_PORT_0000) | 0x100, USB_PORT_0000);
      }
      if (_lw (USB_PORT_0000) & 0x40) {
        cancel_recv_all ();
        g_ace0 = 1;
      }
    }

    bit = 1 << endpnum;
    _sw (bit, USB_PORT_0414);
    SYNC ();
    mask = ~bit;

    _sw (_lw (USB_PORT_0418) & mask, USB_PORT_0418);

    if (t->queue->endpoint_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
      _sw (0x20, USB_PORT_040C);
      SYNC ();
      g_isochronoustransfers |= 1 << endpnum;
      _sw (_lw (USB_PORT_0410) & 0xFFFFFFDF, USB_PORT_0410);

    }

    t->status |= PSP_USB_ENDPOINT_STATUS_TRANSMITTING;
  }
}

/* Subroutine at 0x3E80 */
static
void cancel_send (int endpnum)
{
  struct usb_endpoint *t;
  int bit = 1 << endpnum;

  if (endpnum == 0) {
    t = &g_endpoints[PSP_USB_MAX_ENDPOINTS];
  } else {
    t = &g_endpoints[endpnum];
  }

  if (t->status & PSP_USB_ENDPOINT_STATUS_TRANSMITTING) {
    t->status &= ~PSP_USB_ENDPOINT_STATUS_TRANSMITTING;
    _sw (_lw (USB_PORT_0418) | bit, USB_PORT_0418);
    if (endpnum == 0) {
      g_ace0 = 0;
    }
    if (t->queue->endpoint_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
      g_isochronoustransfers &= ~bit;
      if (g_isochronoustransfers == 0) {
        _sw (_lw (USB_PORT_0410) | 0x20, USB_PORT_0410);
      }
    }
  }
}

/* Subroutine at 0x3F34 */
static
void prepare_recv (int endpnum)
{
  struct usb_endpoint *t;
  t = &g_endpoints[endpnum];
  if (!(t->status & PSP_USB_ENDPOINT_STATUS_COPYING)) {
    if (t->status & (PSP_USB_ENDPOINT_STATUS_STALLED | PSP_USB_ENDPOINT_STATUS_CLEARED)) {
      cancel_recv (endpnum);
    } else {
      if (!(t->status & PSP_USB_ENDPOINT_STATUS_DMACONFIGURED)) {
        if (t->status & PSP_USB_ENDPOINT_STATUS_HASDATA) {
          if (flush_fifo_data (endpnum) == 0) return;
        }
        if (configure_rxdma (t) < 0) return;
        if (!(t->status & PSP_USB_ENDPOINT_STATUS_TRANSMITTING)) {
          t->status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
          if (g_stalledendpoints == 0) {
            if (g_ace0 == 0) {
              int address = USB_PORT_0200 + endpnum * 32;
              _sw (_lw (address) | 0x100, address);
            }
          }
        }
        t->status |= PSP_USB_ENDPOINT_STATUS_TRANSMITTING;
      }
    }

  }
}

/* Subroutine at 0x403c */
static
void cancel_recv (int endpnum)
{
  unsigned int address = (USB_PORT_0200 + 32 * endpnum);
  struct usb_endpoint *t;

  t = &g_endpoints[endpnum];
  if (t->status & PSP_USB_ENDPOINT_STATUS_TRANSMITTING) {
    t->status &= ~PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
    _sw (_lw (address) | 0x80, address);
    t->status &= ~PSP_USB_ENDPOINT_STATUS_TRANSMITTING;
  }
}

/* Subroutine at 0x40A0 */
static
int flush_fifo_data (int endpnum)
{
  struct usb_endpoint *t;  /* var5  */
  struct usb_rxfifo *fifo; /* var4  */
  int ret = 0;             /* var47 */
  int mask;                /* var48 */

  t = &g_endpoints[endpnum];
  fifo = &g_rxfifos[endpnum];
  if (fifo->end == fifo->start) {
    Kprintf ("usbbd ERROR : no data in SW FIFO\n");
    fifo->end = fifo->start = 0;
    ret = -1;
    mask = ~PSP_USB_ENDPOINT_STATUS_HASDATA;
  } else {
    struct usb_queue *queue = t->queue;            /* var6 */
    t->status |= PSP_USB_ENDPOINT_STATUS_COPYING;

    while (queue->first) {
      struct UsbbdDeviceRequest *req;              /* var9 */
      int pos, rem;                                /* var11, var21 */
      struct rxfifo_element *element;              /* var14 */
      int total_size, size;                        /* var20, var16 */
      char *ptr;                                   /* var17 */

      req = queue->first;
      pos = fifo->start;
      rem = req->size - req->transmitted;
      element = &fifo->elements[pos];
      size = total_size = element->size;
      ptr = req->data;
      ptr += req->transmitted;

      if (size != 0) {
        unsigned int mem_begin, mem_end;

        while (queue->packet_size == size && total_size < rem) {
          if ((pos + 1) == fifo->end) break;
          fifo->start = pos = pos + 1;
          size = fifo->elements[pos].size;
          total_size += size;
        }

        mem_begin = (unsigned int) element->ptr;
        mem_end = ALIGN_UP (mem_begin + total_size, 63);
        mem_begin = ALIGN_DOWN (mem_begin, 63);

        sceKernelDcacheInvalidateRange ((void *) mem_begin, mem_end - mem_begin);

        usb_memcpy (ptr, element->ptr, total_size);
        if (CHECK_CACHED_ADDRESS (ptr)) {
          sceKernelDcacheWritebackRange (ptr, total_size);
        }

        req->transmitted += total_size;
        rem -= total_size;
      }

      if (rem <= 0 || queue->packet_size <= size) {
        queue_pop (queue, PSP_USB_RETCODE_SUCCESS);
        if (endpnum == 0) {
          t->status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
          _sw (_lw (USB_PORT_0000) | 0x100, USB_PORT_0000);
          if (_lw (USB_PORT_0000) & 0x40) {
            cancel_recv_all ();
            g_ace0 = 2;
          }
        }
      }
      fifo->start++;
      if (fifo->start == fifo->end) {
        fifo->start = fifo->end = 0;
        t->status &= ~PSP_USB_ENDPOINT_STATUS_HASDATA;
        if (queue->first) ret = 1;
        break;
      }
    }
    mask = ~PSP_USB_ENDPOINT_STATUS_COPYING;
  }
  t->status &= mask;
  return ret;
}

/* Subroutine at 0x4360 */
static
int cancel_current_transaction (int endpoint, int retcode)
{
  struct usb_endpoint *t;
  int ret1 = 0, ret2 = 0;

  t = &g_endpoints[endpoint];
  if (t->status & (PSP_USB_ENDPOINT_STATUS_CANCEL | PSP_USB_ENDPOINT_STATUS_CANCELALL)) {
    if (retcode != PSP_USB_RETCODE_CANCEL) {
      t->status |= PSP_USB_ENDPOINT_STATUS_CANCELALL;
    } else {
      t->status |= PSP_USB_ENDPOINT_STATUS_CANCEL;
    }
    return -1;
  }

  if (t->queue->endpoint_dir != 0x80) {
    int intr;
    volatile struct usb_port *v = t->ports[t->idx];

    intr = sceKernelCpuSuspendIntr ();
    if (((v->unk1) >> 30) == 0) {
      int address = USB_PORT_RXDMA_BASE + 32 * endpoint;
      v->unk1 = v->unk1 | 0xC0000000;
      t->status &= ~(PSP_USB_ENDPOINT_STATUS_DMACONFIGURED);
      t->idx = 1 - t->idx;
      _sw (((unsigned int) t->ports[t->idx]) & 0x1FFFFFFF , address);
    } else {
      if (((v->unk1) >> 30) != 3)
        ret1 = -1;
    }

    sceKernelCpuResumeIntrWithSync (intr);

    if (ret1 == 0) {
      t->status &= ~(PSP_USB_ENDPOINT_STATUS_CANCEL | PSP_USB_ENDPOINT_STATUS_CANCELALL);
    } else {
      if (retcode == PSP_USB_RETCODE_CANCEL) {
        t->status |= PSP_USB_ENDPOINT_STATUS_CANCEL;
      } else {
        t->status |= PSP_USB_ENDPOINT_STATUS_CANCELALL;
      }
    }
  }

  if (endpoint == 0) {
    t = &g_endpoints[9];
  }

  if (t->queue->endpoint_dir != 0) {
    int address = USB_PORT_0000 + 32 * endpoint;
    _sw (_lw (address) & 0xFFFFFFF7, address);
    if ((t->ports[0]->unk1 >> 30) == 0) {
      t->ports[0]->unk1 |= 0xC0000000;
      t->status &= ~PSP_USB_ENDPOINT_STATUS_DMACONFIGURED;

      t->status &= ~(PSP_USB_ENDPOINT_STATUS_CANCEL | PSP_USB_ENDPOINT_STATUS_CANCELALL);
    } else if ((t->ports[0]->unk1 >> 30) == 3) {
      t->status &= ~(PSP_USB_ENDPOINT_STATUS_CANCEL | PSP_USB_ENDPOINT_STATUS_CANCELALL);
    } else {
      ret2 = -1;
      if (retcode == PSP_USB_RETCODE_CANCEL) {
        t->status |= PSP_USB_ENDPOINT_STATUS_CANCEL;
      } else {
        t->status |= PSP_USB_ENDPOINT_STATUS_CANCELALL;
      }
    }
  }

  return ret1 | ret2;
}

/* Subroutine at 0x458C */
static
int configure_rxdma (struct usb_endpoint *t)
{
  int endpnum;
  volatile struct usb_port *v;
  int ret = -1;

  endpnum = t->queue->endpoint->endpointNumber;
  v = t->ports[t->idx];
  if (endpnum != 0 || !(t->status & PSP_USB_ENDPOINT_STATUS_EP0_USE_FIFO)) {
    ret = -1;
    if (!(t->status & PSP_USB_ENDPOINT_STATUS_DMACONFIGURED)) {
      if (t->queue->first) {
        struct UsbbdDeviceRequest *req = t->queue->first;
        int rem = req->size - req->transmitted;
        if (rem > t->queue->packet_size) {
          rem = t->queue->packet_size;
        }
        if (rem != 0 || req->isControlRequest) {
          v->unk1 = ((v->unk1 & 0x3FFFFFFF) & 0xCFFFFFFF);
          v->unk1 = (v->unk1 & 0xFFFF0000);
          v->unk3 = ((unsigned int) req->physicalAddress) + req->transmitted;
          v->unk1 = v->unk1 | 0x08000000;
          _sw (((unsigned int) v) & 0x1FFFFFFF, USB_PORT_RXDMA_BASE + 32 * endpnum);
          t->status |= PSP_USB_ENDPOINT_STATUS_DMACONFIGURED;
          _lw ((unsigned int) &v->unk3);
          /* read unk3 ?? */
        } else {
          Kprintf ("usbbd ERROR : usbbd_bi.c:%s: this error cannot be exist!!!!\n", "KickDMAReceive");
        }
        ret = rem;
      } else {
        t->status &= ~PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
        _sw (_lw (USB_PORT_0200 + 32 * endpnum) | 0x80, USB_PORT_0200 + 32 * endpnum);
      }
    }
  } else {
    t->status |= PSP_USB_ENDPOINT_STATUS_DMACONFIGURED;
    v->unk1 = ((v->unk1 & 0x3FFFFFFF) & 0xCFFFFFFF);
    v->unk1 = (v->unk1 & 0xFFFF0000);
    v->unk1 |= 0x08000000;
    v->unk3 = ((unsigned int) g_rxfifos[0].buffer) & 0x1FFFFFFF;
    _lw ((unsigned int) &v->unk3);
    /* read unk3  ?? */
    _sw (((unsigned int) v) & 0x1FFFFFFF, USB_PORT_RXDMA_BASE);
    ret = 0;
  }
  return ret;
}

/* Subroutine at 0x474C */
static
void configure_rxdma_fifo (struct usb_endpoint *t, int usefirst)
{
  int endpnum;
  struct usb_rxfifo *fifo;
  volatile struct usb_port *v;
  struct rxfifo_element *el;


  endpnum = t->queue->endpoint->endpointNumber;
  fifo = &g_rxfifos[endpnum];
  v = t->ports[t->idx];

  if (!usefirst) {
    if (endpnum == 0) {
      Kprintf ("usbbd ERROR : EP0 doesn't have SW FIFO\n");
      fifo->start = fifo->end = 0;
    }
    if (fifo->end >= PSP_USB_FIFO_ELEMENTS) {
      Kprintf ("usbbd ERROR : EP%d : overflow SW FIFO\n", endpnum);
      fifo->start = fifo->end = 0;
    }
    el = &fifo->elements[fifo->end];
  } else {
    el = fifo->elements;
  }
  v->unk1 = ((v->unk1 & 0x3FFFFFFF) & 0xCFFFFFFF);
  v->unk1 = (v->unk1 & 0xFFFF0000);
  v->unk1 |= 0x08000000;
  v->unk3 = ((unsigned int) el->ptr) & 0x1FFFFFFF;
  _lw ((unsigned int) &v->unk3);
  /* read unk3 */
  _sw (((unsigned int) v) & 0x1FFFFFFF, USB_PORT_RXDMA_BASE + 32 * endpnum);
  t->status |= PSP_USB_ENDPOINT_STATUS_DMACONFIGURED;
}

/* Subroutine at 0x488C */
static
void usb_interrupt (void)
{
  int val1, val2;
  val1 = _lw (USB_PORT_0414);
  val2 = _lw (USB_PORT_0418);
  if (val1 != 0) {
    int intr = sceKernelCpuSuspendIntr ();
    int endpoint;

    _sw (val1, USB_PORT_0414);

    for (endpoint = 0; endpoint < g_total_endpoints; endpoint++) {
      struct usb_endpoint *t;
      int val3 = _lw (USB_PORT_0004 + 32 * endpoint);
      int bit = 1 << endpoint;
      if (endpoint == 0) {
        t = &g_endpoints[9];
      } else {
        t = &g_endpoints[endpoint];
      }

      if (!(val1 & bit)) {
        if (!(val2 & bit)) {
          int val4 = _lw (USB_PORT_0000 + 32 * endpoint);
          if (val3 & 0x680) {
            if ( ((t->ports[0]->unk1 >> 30) == 2) &&
                 (!(val4 & 0x08)) ) {
              if (!((_lw (USB_PORT_0414) >> endpoint) & 0x01)) {
                on_send_packet (endpoint);
              }

            } else {
              t->alarm = sceKernelSetAlarm (PSP_USB_ALARM_TIME, &endpoint_in_alarm_handler, t);
              if (t->alarm < 0) {
                Kprintf ("usbbd ERROR : EP%d IN cannot set alarm handler : 0x%08x\n", endpoint, t->alarm);
              }
            }
          }
        }
      } else {
        if (val3 & 0x40) {
          on_send_packet (endpoint);
        } else {
          if (val3 & 0x680) {
            if ((t->ports[0]->unk1 >> 30) == 2) {
              on_send_packet (endpoint);
            } else {
              int val4 = _lw (USB_PORT_0000 + 32 * endpoint);
              if (!(val4 & 0x08)) {
                if ((t->ports[0]->unk1 >> 30) == 1) {
                  /* Flush DDR */
                  _sw (_lw (IO_PORT_DDRFLUSH) | 1, IO_PORT_DDRFLUSH);
                  while (_lw (IO_PORT_DDRFLUSH) & 1);
                  if ((t->ports[0]->unk1 >> 30) == 2) {
                    on_send_packet (endpoint);
                  }
                }
              }
            }
          }
        }
      }

    }

    if (!(val1 & 0xFFFF0000)) {
      for (endpoint = 0; endpoint < g_total_endpoints; endpoint++) {
        if (!(val2 & (1 << (16 + endpoint))))  {
          int val3 = _lw (USB_PORT_0204 + 32 * endpoint);
          int val4 = _lw (USB_PORT_0404);
          if (val3 & 0x2B0) {
            struct usb_endpoint *t = &g_endpoints[endpoint];

            /* TODO check this */
            if ( (((t->ports[t->idx]->unk1 >> 30) == 2) ||
                 ((t->ports[1 - t->idx]->unk1 >> 30) == 2) ||
                 ((endpoint == 0) && ((g_defaultport->unk1 >> 30) == 2))) &&
                 (!(val4 & 0x04)) ) {
              int val5 = _lw (USB_PORT_0414);
              if (!((val5 >> (16 + endpoint)) & 1)) {
                on_receive_packet (endpoint);
                val1 |= (1 << (16 + endpoint));
              }
            } else {
              t->alarm = sceKernelSetAlarm (PSP_USB_ALARM_TIME, &endpoint_out_alarm_handler, t);
              if (t->alarm < 0) {
                Kprintf ("usbbd ERROR : EP%d OUT cannot set alarm handler : 0x%08x\n", endpoint, t->alarm);
              }
            }
          }
        }
      }
    } else {
      for (endpoint = 0; endpoint < g_total_endpoints; endpoint++) {
        if (val1 & (1 << (16 + endpoint))) {
          struct usb_endpoint *t = &g_endpoints[endpoint];
          if ( ((t->ports[t->idx]->unk1 >> 30) == 2) ||
              ((endpoint == 0) && ((g_defaultport->unk1 >> 30) == 2)) ||
              ((t->ports[1 - t->idx]->unk1 >> 30) == 2)) {
            on_receive_packet (endpoint);
          } else {
            int val4 = _lw (USB_PORT_0404);
            if (!(val4 & 0x04)) {
              /* Flush DDR */
              _sw (_lw (IO_PORT_DDRFLUSH) | 1, IO_PORT_DDRFLUSH);
              while (_lw (IO_PORT_DDRFLUSH) & 1);
              if ( ((t->ports[0]->unk1 >> 30) == 2) ||
                  ((endpoint == 0) && ((g_defaultport->unk1 >> 30) == 2)) ||
                  ((t->ports[1]->unk1 >> 30) == 2)) {
                on_receive_packet (endpoint);
              }
            }
          }
        }
      }
    }

    sceKernelCpuResumeIntr (intr);

    if (val1 & 0xFFFF0000) {
      _sw (_lw (USB_PORT_0404) | 4, USB_PORT_0404);
      SYNC ();
      setup_recv_all ();
    }
  }
}

/* Subroutine at 0x4D7C */
static
SceUInt endpoint_in_alarm_handler (void *common)
{
  struct usb_endpoint *t;
  int endpnum;
  int val1, val2;

  t = common;
  endpnum = t->queue->endpoint->endpointNumber;

  val1 = _lw (USB_PORT_0004 + 32 * endpnum);
  val2 = _lw (USB_PORT_0000 + 32 * endpnum);
  if ((val1 & 0x680) == 0) {
    Kprintf ("usbbd ERROR : EP%d IN field is not set\n", endpnum);
    t->alarm = 0;
    return 0;
  } else {
    if ((t->ports[0]->unk1 >> 30) != 2) {
      return PSP_USB_ALARM_TIME;
    }
    if (!(val2 & 0x08)) {
      if ((_lw (USB_PORT_0414) >> endpnum) & 0x01) {
        return 0;
      }
      t->alarm = 0;
      on_send_packet (endpnum);
      return 0;
    }
  }
  return PSP_USB_ALARM_TIME;
}

/* Subroutine at 0x4E40 */
static
SceUInt endpoint_out_alarm_handler (void *common)
{
  struct usb_endpoint *t;
  int endpnum;
  int val1, val2;

  t = common;
  endpnum = t->queue->endpoint->endpointNumber;
  val1 = _lw (USB_PORT_0204 + 32 * endpnum);
  val2 = _lw (USB_PORT_0404);
  if ((val1 & 0x2B0) == 0) {
    Kprintf ("usbbd ERROR : EP%d OUT field is not set\n", endpnum);
    t->alarm = 0;
    return 0;
  } else {
    if ( ((t->ports[t->idx]->unk1 >> 30) == 2) ||
         ( (endpnum == 0) && ((g_defaultport->unk1 >> 30) == 2)) ||
         ((t->ports[1 - t->idx]->unk1 >> 30) == 2) ) {
      if (!(val2 & 0x04)) {
        int val3 = _lw (USB_PORT_0414);
        if (!((val3 >> (16 + endpnum)) & 0x01)) {
          t->alarm = 0;
          on_receive_packet (endpnum);
          _sw (_lw (USB_PORT_0404) | 4, USB_PORT_0404);
          SYNC ();
          setup_recv_all ();
        }
        return 0;
      }
    }
  }
  return PSP_USB_ALARM_TIME;
}

/* Subroutine at 0x4F70 */
static
SceUInt main_alarm_handler (void *common)
{
  if (g_cable_connected != PSP_USB_STATUS_CABLE_DISCONNECTED) {
    int endpoint, intr;

    g_usb_version = -1;
    g_isochronoustransfers = 0;
    g_clearedendpoints = 0;
    g_stalledendpoints = 0;
    g_ace0 = 0;

    for (endpoint = 0; endpoint <= 9; endpoint++) {
      struct usb_endpoint *t = &g_endpoints[endpoint];
      int j;
      for (j = 0; j <= 1; j++) {
        t->ports[j]->unk1 |= 0xC0000000;
        t->ports[j]->unk1 |= 0x08000000;
      }

      t->status = 0;
      t->unk4 = 0;
      t->queue->endpoint_dir = 255;
      t->idx = 0;
      if (t->alarm > 0) {
        sceKernelCancelAlarm (t->alarm);
        t->alarm = 0;
      }
    }

    if (g_devdesc_hi) {
      _sw (0xA8, USB_PORT_0400);
    } else {
      _sw (0xA9, USB_PORT_0400);
    }
    _sw (0x210, USB_PORT_0404);
    _sw (0x0, USB_PORT_MAINALARM_SETZERO);
    _sw (_lw (USB_PORT_040C), USB_PORT_040C);
    _sw (_lw (USB_PORT_0414), USB_PORT_0414);
    _sw (0xFFFFFFB7, USB_PORT_0410);
    _sw (0xFFFFFFFF, USB_PORT_0418);
    SYNC ();

    sceKernelEnableIntr (PSP_USB_IRQ_MAIN);

    intr = sceSysregUsbQueryIntr ();
    if (intr & PSP_USB_INTERRUPT_RESUME) {
      sceSysregUsbAcquireIntr (PSP_USB_INTERRUPT_RESUME);
    }

    SYNC ();

    sceKernelEnableIntr (PSP_USB_IRQ_RESUME);
    sceSysreg_driver_8D0FED1E ();

    if (!g_devdesc_hi) {
      set_usb_version (1);
    }
    g_mainalarm = -1;
  }
  return 0;
}

/* Subroutine at 0x5140 */
static
int on_send_packet (int endpnum)
{
  int val1, val5;
  struct usb_endpoint *t;

  t = &g_endpoints[9];
  val1 = _lw (USB_PORT_0004 + 32 * endpnum);
  val5 = (val1 & ~(0x6C0));

  if (endpnum != 0) {
    t = &g_endpoints[endpnum];
  }
  if (val1 & 0x6C0) {
    if (val1 & 0x200) {
      struct UsbbdDeviceRequest *req = t->queue->first;
      _sw (val5 | 0x00000200, USB_PORT_0004 + 32 * endpnum);

      Kprintf ("usbbd ERROR : EP%d OUT : host bus error : 0x%p, len=%d, count=%d\n",
          endpnum, req->data, req->size, req->transmitted);

    }
    if (val1 & 0x00000080) {
      _sw (val5 | 0x0000080, USB_PORT_0004 + 32 * endpnum);
      Kprintf ("usbbd ERROR : EP%d IN : BS is not Host Ready\n", endpnum);
    }
    if (val1 & 0x00000400) {
      _sw (val5 | 0x0000400, USB_PORT_0004 + 32 * endpnum);
      flush_tx_data (t);
      _sw (val5 | 0x0000040, USB_PORT_0004 + 32 * endpnum);
      val1 = 0;
    }

    if (val1 & 0x40) {
      if (t->queue->endpoint_type != USB_ENDPOINT_TYPE_ISOCHRONOUS) {
        configure_txdma (t);
      }
      _sw (val5 | 0x0000040, USB_PORT_0004 + 32 * endpnum);
    }
  }
  if (t->alarm > 0) {
    sceKernelCancelAlarm (t->alarm);
    t->alarm = 0;
  }
  return 0;
}

/* Subroutine at 0x52D4 */
static
int on_receive_packet (int endpnum)
{
  struct usb_endpoint *t;
  int val1;
  int val6;

  t = &g_endpoints[endpnum];
  val1 = _lw (USB_PORT_0204 + 32 * endpnum);
  val6 = (val1 & 0xFFFFFD4F);

  if (val1 & 0x2B0) {
    if (val1 & 0x200) {
      struct UsbbdDeviceRequest *req = t->queue->first;
      _sw (val6 | 0x0000200, USB_PORT_0204 + 32 * endpnum);

      Kprintf ("usbbd ERROR : EP%d OUT : host bus error : 0x%p, len=%d, count=%d\n",
          endpnum, req->data, req->size, req->transmitted);

    }
    if (val1 & 0x80) {
      _sw (val6 | 0x0000080, USB_PORT_0204 + 32 * endpnum);
    }
    if (val1 & 0x10) {
      _sw (val6 | 0x0000010, USB_PORT_0204 + 32 * endpnum);
      label12:
      get_packet (t);
      if (g_clearedendpoints | g_stalledendpoints | g_ace0) {
        check_stall_or_clear ();
      }
    }
    if (val1 & 0x20) {
      int temp;

      if ((g_defaultport->unk1 >> 30) != 2) {
        val1 &= 0xFFFFFFDF;
        if ((t->ports[t->idx]->unk1 >> 30) == 2) {
          goto label12;
        }
      }

      _sw (val6 | 0x0000020, USB_PORT_0204 + 32 * endpnum);
      usb_cancel_all (endpnum);
      _sw (_lw (USB_PORT_0000 + 32 * endpnum) | 0x02, USB_PORT_0000 + 32 * endpnum);
      g_endpoints[0].status &= ~(PSP_USB_ENDPOINT_STATUS_TRANSMITTING | PSP_USB_ENDPOINT_STATUS_EP0_USE_FIFO);
      g_endpoints[9].status &= ~PSP_USB_ENDPOINT_STATUS_TRANSMITTING;
      g_rxfifos[endpnum].start = g_rxfifos[endpnum].end = 0;
      if ((g_endpoints[0].status & (PSP_USB_ENDPOINT_STATUS_STALLED | PSP_USB_ENDPOINT_STATUS_CLEARED)) == PSP_USB_ENDPOINT_STATUS_STALLED) {
        g_stalledendpoints &= ~(1 << endpnum);
        g_endpoints[0].status &= ~PSP_USB_ENDPOINT_STATUS_STALLED;
        g_endpoints[9].status &= ~PSP_USB_ENDPOINT_STATUS_STALLED;
      }
      temp = get_devrequest ();
      if (temp >= 0) {
        if (process_request () < 0) {
          usb_stall (endpnum);
        } else {
          if (g_devreq.wLength != 0) {
            if (g_devreq.bmRequestType & 0x80) {
              g_endpoints[0].status |= PSP_USB_ENDPOINT_STATUS_EP0_USE_FIFO;
              configure_rxdma (g_endpoints);
            }
          } else {
            g_endpoints[9].status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
            _sw (_lw (USB_PORT_0000 + 32 * endpnum) | 0x100, USB_PORT_0000 + 32 * endpnum);
            if (_lw (USB_PORT_0000 + 32 * endpnum) & 0x40) {
              cancel_recv_all ();
              g_ace0 = 2;
            }
          }
        }
      }

      if (g_stalledendpoints | g_clearedendpoints) {
        check_stall_or_clear ();
      }
      g_endpoints[0].status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
    }
  }

  if (t->alarm > 0) {
    sceKernelCancelAlarm (t->alarm);
    t->alarm = 0;
  }
  return 0;
}

/* Subroutine at 0x562C */
static
int start_bus_driver (void)
{
  int i, j, ret;
  struct usb_endpoint *t;

  t = g_endpoints;
  g_defaultport = (struct usb_port *) ((((unsigned int) g_ports) & 0x1FFFFFFF) | 0xA0000000);

  for (i = 0; i < 10; i++) {
    struct usb_port *v = &g_ports[i * 2 + 1];

    for (j = 0; j <= 1; j++) {
      /* TODO: check this */
      struct usb_port *ptr;
      ptr = (struct usb_port *) ((((unsigned int) v) & 0x1FFFFFFF) | 0xA0000000);
      t->ports[j] = ptr;

      ptr->unk1 |= 0xC0000000;
      ptr->unk1 |= 0x08000000;
      ptr->unk4 = ((unsigned int) ptr) & 0x1FFFFFFF;

      v++;
    }

    t->idx = 0;
    t++;
  }

  for (i = 0; i < PSP_USB_MAX_ENDPOINTS; i++) {
    g_endpoints[i].queue = &g_queues[i];
  }

  g_mainalarm = -1;
  g_endpoints[PSP_USB_MAX_ENDPOINTS].queue = g_queues;

  ret = sceKernelRegisterIntrHandler (PSP_USB_IRQ_MAIN, 1, &usb_interrupt_handler, &g_endpoints, NULL);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot register USB interrupt handler : 0x%08x\n", ret);
    return ret;
  }

  ret = sceKernelRegisterIntrHandler (PSP_USB_IRQ_CONNECT, 1, &usb_connect_interrupt_handler, &g_endpoints, NULL);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot register USBCON interrupt handler : 0x%08x\n", ret);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_MAIN);
    return ret;
  }

  ret = sceKernelRegisterIntrHandler (PSP_USB_IRQ_DISCONNECT, 1, &usb_disconnect_interrupt_handler, &g_endpoints, NULL);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot register USBDIS interrupt handler : 0x%08x\n", ret);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_CONNECT);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_MAIN);
    return ret;
  }

  ret = sceKernelRegisterIntrHandler (PSP_USB_IRQ_READY, 1, &usb_ready_interrupt_handler, &g_endpoints, NULL);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot register USBREADY interrupt handler : 0x%08x\n", ret);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_DISCONNECT);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_CONNECT);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_MAIN);
    return ret;
  }

  ret = sceKernelRegisterIntrHandler (PSP_USB_IRQ_RESUME, 1, &usb_resume_interrupt_handler, &g_endpoints, NULL);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot register USBRESUME interrupt handler : 0x%08x\n", ret);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_READY);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_DISCONNECT);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_CONNECT);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_MAIN);
    return ret;
  }

  ret = sceSysregUsbQueryIntr ();
  if (ret & PSP_USB_INTERRUPT_CONNECT) {
    sceSysregUsbAcquireIntr (PSP_USB_INTERRUPT_CONNECT);
  }

  if (ret & PSP_USB_INTERRUPT_DISCONNECT) {
    sceSysregUsbAcquireIntr (PSP_USB_INTERRUPT_DISCONNECT);
  }

  SYNC ();
  sceKernelPowerLock (0);


  ret = register_sysevent_handler ();
  if (ret < 0) {
    sceKernelPowerUnlock (0);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_RESUME);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_READY);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_DISCONNECT);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_CONNECT);
    sceKernelReleaseIntrHandler (PSP_USB_IRQ_MAIN);
    return ret;
  }

  ret = sceKernelCpuSuspendIntr ();
  _sw (_lw (IO_PORT_BC000050) | 0x00020000, IO_PORT_BC000050);
  sceKernelCpuResumeIntr (ret);

  sceSysregUsbIoEnable ();

  ret = sceSysregUsbQueryIntr ();
  sceSysregUsbAcquireIntr (ret);

  SYNC ();

  g_prev_cable_connected = PSP_USB_STATUS_CABLE_DISCONNECTED;
  ret = sceSysregUsbGetConnectStatus ();
  if (ret)
    g_cable_connected = PSP_USB_STATUS_CABLE_CONNECTED;


  refresh_usb_event_flag ();
  sceKernelEnableIntr (PSP_USB_IRQ_CONNECT);
  sceKernelEnableIntr (PSP_USB_IRQ_DISCONNECT);

  sceKernelPowerUnlock (0);

  return 0;
}

/* Subroutine at 0x5980 */
void usb_stop (void)
{
  int ret;
  ret = sceKernelDeleteFpl (g_swfifofpl);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot delete FPL for SW FIFO\n");
  }
  sceSysregUsbBusClockDisable ();
  sceSysregUsbClkDisable (9);
}

/* Subroutine at 0x59D0 */
void usb_resume (void)
{
  int ret;
  sceSysregUsbResetDisable ();
  ret = sceSysregUsbQueryIntr ();
  if (ret & PSP_USB_INTERRUPT_READY) {
    sceSysregUsbAcquireIntr (PSP_USB_INTERRUPT_READY);
  }
  SYNC ();
  sceKernelEnableIntr (PSP_USB_IRQ_READY);
}

/* Subroutine at 0x5A4C */
int sceUsbbdClearFIFO (struct UsbEndpoint *endp)
{
  if (!endp) {
    Kprintf ("usbbd ERROR : Null pointer\n");
    return PSP_USB_ERROR_INVALID_ARGUMENT;
  }

  if (endp->endpointNumber == 0) {
    Kprintf ("usbbd ERROR : Cannot clear FIFO of EP0\n");
    return PSP_USB_ERROR_INVALID_ARGUMENT;
  }

  return usb_clear_fifo (endp->endpointNumber);
}

/* Subroutine at 0x5A9C */
int sceUsbbdStall (struct UsbEndpoint *endp)
{
  if (!endp) {
    Kprintf ("usbbd ERROR : Null pointer\n");
    return PSP_USB_ERROR_INVALID_ARGUMENT;
  }

  if (endp->endpointNumber == 0) {
    Kprintf ("usbbd ERROR : Cannot stall EP0\n");
    return PSP_USB_ERROR_INVALID_ARGUMENT;
  }

  return usb_stall (endp->endpointNumber);
}

/* Subroutine at 0x5AEC */
static
int release_bus_driver (void)
{
  sceKernelPowerLock (0);
  sceKernelDisableIntr (PSP_USB_IRQ_DISCONNECT);
  sceKernelDisableIntr (PSP_USB_IRQ_CONNECT);
  sceSysregUsbIoDisable ();
  sceSysregUsbBusClockDisable ();
  sceSysregUsbClkDisable (9);

  unregister_sysevent_handler ();
  sceKernelPowerUnlock (0);

  sceKernelReleaseIntrHandler (PSP_USB_IRQ_RESUME);
  sceKernelReleaseIntrHandler (PSP_USB_IRQ_READY);
  sceKernelReleaseIntrHandler (PSP_USB_IRQ_DISCONNECT);
  sceKernelReleaseIntrHandler (PSP_USB_IRQ_CONNECT);
  sceKernelReleaseIntrHandler (PSP_USB_IRQ_MAIN);
  return 0;
}

/* Subroutine at 0x5B6C */
static
int on_module_start (void)
{
  return 0;
}

/* Subroutine at 0x5B74 */
static
int on_module_stop (void)
{
  return 0;
}

/* Subroutine at 0x5B7C */
static
void set_usb_version (int usb_version)
{
  struct DeviceDescriptor *desc;  /* var2 */
  struct UsbConfiguration *conf;  /* var1 */
  struct InterfaceDescriptor *id;
  struct EndpointDescriptor *endp;
  struct InterfaceSettings *settings;
  int packet_size;
  int i, j, k, endpnum;

  g_usb_version = usb_version;
  if (usb_version == 2) {
    desc = g_devdesc_hi;
    conf = g_conf_hi;
  } else {
    desc = g_devdesc;
    conf = g_conf;
  }

  if (g_rxfifos->elements) {
    g_rxfifos->elements->ptr = g_rxfifos->buffer;
  }

  packet_size = desc->bMaxPacketSize0;
  _sw (packet_size << 19, USB_PORT_ENDPOINT_CONFIG_BASE4);
  g_queues->endpoint->transmittedBytes = 0;
  _sw (2, USB_PORT_0000);
  g_queues->packet_size = packet_size;
  _sw (packet_size >> 2, USB_PORT_ENDPOINT_PACKET_SIZEW_BASE);
  g_endpoints->packet_size = packet_size;
  _sw (packet_size, USB_PORT_PACKET_SIZE_BASE);
  g_endpoints[9].packet_size = packet_size;
  _sw (0, USB_PORT_0200);
  _sw (packet_size, USB_PORT_020C);

  endpnum = 1;

  settings = conf->settings;
  id = conf->interfaceDescriptors;
  endp = conf->endpointDescriptors;
  for (i = 0; i < conf->configDescriptors->bNumInterfaces; i++) {
    for (j = 0; j < settings->numDescriptors; j++) {
      for (k = 0; k < id->bNumEndpoints; k++) {
        unsigned int address = USB_PORT_ENDPOINT_CONFIG_BASE4 + 4 * endpnum;
        unsigned int val = 0;
        val = endp->bEndpointAddress & USB_ENDPOINT_ADDRESS_MASK;
        if (endp->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
          val |= 1 << 4;
        val |= (endp->bmAttributes & USB_ENDPOINT_TYPE_MASK) << 5;
        val |= 0x80;
        val |= i << 11;
        val |= j << 15;
        val |= endp->wMaxPacketSize << 19;
        if ((endp->bmAttributes & USB_ENDPOINT_TYPE_MASK) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
          val |= 1 << 30;
        _sw (val, address);
        endpnum++;
        endp++;
      }
      id++;
    }
    settings++;
  }

  _sw (_lw (USB_PORT_0004), USB_PORT_0004);
  _sw (_lw (USB_PORT_0204), USB_PORT_0204);
  SYNC ();
  _sw (0xFFFFFFA4, USB_PORT_0410);
  _sw (0xFFFEFFFF, USB_PORT_0418);
  g_defaultport->unk1 &= 0x3FFFFFFF;
  g_defaultport->unk1 &= 0xCFFFFFFF;
  _sw (((unsigned int) g_defaultport) & 0x1FFFFFFF, USB_PORT_0210);
  _sw (_lw (USB_PORT_0404) | 0x0C, USB_PORT_0404);
  g_endpoints->status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
  g_endpoints[9].status &= ~PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
  _sw (_lw (USB_PORT_0200) | 0x100, USB_PORT_0200);
}

/* Subroutine at 0x5E14 */
static
int get_devrequest (void)
{
  int ret = 0;

  if ((g_defaultport->unk1 & 0xF0000000) == 0x80000000) {
    if (g_clearedendpoints & 1) {
      ret = -1;
    } else {
      /* Bizarre! */
      usb_memcpy (&g_devreq, (void *) &g_defaultport->unk3, sizeof (struct DeviceRequest));
    }
  } else {
    Kprintf ("usbbd ERROR : invalid descriptor's status bs=%d, stat=%d\n",
        g_defaultport->unk1 >> 30, (g_defaultport->unk1 >> 28) & 0x03);
    ret = -1;
  }
  g_defaultport->unk1 &= 0x3FFFFFFF;
  g_defaultport->unk1 &= 0xCFFFFFFF;
  return ret;
}

/* Subroutine at 0x5EE0 */
static
void check_stall_or_clear (void)
{
  int val1;
  int endpoint;

  val1 = _lw (USB_PORT_0408);
  if (!(val1 & 0x8000)) {
    for (endpoint = 0; endpoint < g_total_endpoints; endpoint++) {
      struct usb_endpoint *t = &g_endpoints[endpoint];
      int address = USB_PORT_0000 + 32 * endpoint;
      if (t->status & PSP_USB_ENDPOINT_STATUS_CLEARED) {
        t->unk4--;
        if (t->unk4 == 0) {
          volatile struct usb_port *v = t->ports[t->idx];
          g_clearedendpoints &= ~(1 << endpoint);
          v->unk1 |= 0xC0000000;
          t->status &= ~(PSP_USB_ENDPOINT_STATUS_DMACONFIGURED);
          if (endpoint == 0) {
            t->status &= ~PSP_USB_ENDPOINT_STATUS_CLEARED;
            t = &g_endpoints[9];
          }
          if (t->queue->endpoint_dir == 255) {
            _sw (_lw (address) | 0x02, address);
          }

          t->status &= ~PSP_USB_ENDPOINT_STATUS_CLEARED;
          if (t->queue->first) {
            struct UsbbdDeviceRequest *req = t->queue->first;
            if (req->returnCode == PSP_USB_RETCODE_RECV) {
              prepare_recv (endpoint);
            } else {
              prepare_send (endpoint);
            }
          }
        }
      }
    }
  } else {
    g_clearedendpoints = 0;
    g_stalledendpoints = 0;
    if (g_ace0 != 0) {
      _sw (_lw (USB_PORT_0000) | 0x100, USB_PORT_0000);
      if (g_ace0 == 1) {
        _sw (1, USB_PORT_0414);
        SYNC ();
        _sw (_lw (USB_PORT_0418) & 0xFFFFFFFE, USB_PORT_0418);
      }
      g_ace0 = 0;
      g_endpoints[9].status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
    }

    for (endpoint = 0; endpoint < g_total_endpoints; endpoint++) {
      struct usb_endpoint *t = &g_endpoints[endpoint];
      int address1 = USB_PORT_0000 + 32 * endpoint;
      int address2 = USB_PORT_0200 + 32 * endpoint;
      if (t->status & (PSP_USB_ENDPOINT_STATUS_CLEARED | PSP_USB_ENDPOINT_STATUS_STALLED)) {
        if (t->queue->endpoint_dir != 0x80) {
          volatile struct usb_port *v = t->ports[t->idx];
          if (t->status & PSP_USB_ENDPOINT_STATUS_STALLED) {
            _sw (_lw (address2) | 0x01, address2);
          }
          v->unk1 |= 0xC0000000;
          t->status &= ~PSP_USB_ENDPOINT_STATUS_DMACONFIGURED;
        }

        if (endpoint == 0) {
          t->status &= ~(PSP_USB_ENDPOINT_STATUS_CLEARED | PSP_USB_ENDPOINT_STATUS_STALLED);
          t = &g_endpoints[9];
        }
        if (t->queue->endpoint_dir != 0) {
          _sw (_lw (address1) | 0x02, address1);
          if (t->status & PSP_USB_ENDPOINT_STATUS_STALLED) {
            _sw (_lw (address1) | 0x01, address1);
          }
        }
        t->status &= ~(PSP_USB_ENDPOINT_STATUS_CLEARED | PSP_USB_ENDPOINT_STATUS_STALLED);
        if (t->queue->first) {
          struct UsbbdDeviceRequest *req = t->queue->first;
          if (req->returnCode == PSP_USB_RETCODE_RECV) {
            prepare_recv (endpoint);
          } else {
            prepare_send (endpoint);
          }
        }
      }
    }
  }
}

/* Subroutine at 0x6250 */
static
void setup_recv_all (void)
{
  int endpoint;
  int bits;

  bits = g_ace0 | g_stalledendpoints;
  if (!bits) {
    if (g_endpoints->status & PSP_USB_ENDPOINT_STATUS_UNKNOWN2) {
      _sw (_lw (USB_PORT_0200) | 0x100, USB_PORT_0200);
    }
  }
  for (endpoint = 1; endpoint < g_total_endpoints; endpoint++) {
    struct usb_endpoint *t = &g_endpoints[endpoint];
    struct usb_queue *queue = &g_queues[endpoint];
    int address = USB_PORT_0200 + 32 * endpoint;

    if (t->status & PSP_USB_ENDPOINT_STATUS_UNKNOWN2) {
      if (queue->endpoint_dir != 0x80) {
        if (!bits) {
          _sw (_lw (address) | 0x100, address);
        }
      }
    }
  }
}

/* Subroutine at 0x6320 */
static
int configure_txdma (struct usb_endpoint *endp)
{
  struct UsbbdDeviceRequest *req;
  int ret = -1;

  req = endp->queue->first;

  if (req) {
    if (!(endp->status & PSP_USB_ENDPOINT_STATUS_DMACONFIGURED)) {
      int rem;
      endp->status = ((endp->status | PSP_USB_ENDPOINT_STATUS_DMACONFIGURED) & (~PSP_USB_ENDPOINT_STATUS_REM0));
      rem = req->size - req->transmitted;
      if (rem > endp->queue->packet_size) {
        rem = endp->queue->packet_size;
      }

      ret = rem;
      if (rem != 0 || req->isControlRequest) {
        volatile struct usb_port *v = endp->ports[0];
        int endpnum = endp->queue->endpoint->endpointNumber;

        v->unk1 = (((v->unk1 & 0x3FFFFFFF) & 0xCFFFFFFF) | 0x08000000);
        v->unk1 = (v->unk1 & 0xFFFF0000) | (rem & 0xFFFF);
        if (endp->queue->endpoint_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
          unsigned int val = _lw (USB_PORT_0408);
          v->unk1 = (v->unk1 & 0xF800FFFF) | ((((((val << 3) >> 19) + 1) | (endp->interval - 1)) & 0x000007FF) << 16);
        }
        v->unk3 = ((unsigned int) req->physicalAddress) + req->transmitted;
        if (rem == 0) {
          endp->status |= PSP_USB_ENDPOINT_STATUS_REM0;
        }
        _sw (((unsigned int) v) & 0x1FFFFFFF, USB_PORT_TXDMA_BASE + 32 * endpnum);
        SYNC ();
        _sw (_lw (USB_PORT_0000 + 32 * endpnum) | 0x08, USB_PORT_0000 + 32 * endpnum);
      }
    }
  }
  return ret;
}

/* Subroutine at 0x6478 */
static
int flush_tx_data (struct usb_endpoint *t)
{
  struct UsbbdDeviceRequest *req;
  int endpnum;

  req = t->queue->first;
  endpnum = t->queue->endpoint->endpointNumber;
  t->status &= ~PSP_USB_ENDPOINT_STATUS_DMACONFIGURED;

  if (!req) {
    if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
      if (g_cable_connected == PSP_USB_STATUS_CABLE_CONNECTED) {
        if (g_connection_status != PSP_USB_STATUS_CONNECTION_SUSPENDED) {
          Kprintf ("usbbd ERROR : EP%d no request\n", endpnum);
        }
      }
    }
    return -1;
  } else {
    volatile struct usb_port *v = t->ports[0];
    if ((v->unk1 & 0xF0000000) == 0x40000000) {
      Kprintf ("usbbd ERROR : IN : invalid Tx DMA COMP interrupt. DMA is busy\n");
      return 0;
    }
    if ((v->unk1 & 0xF0000000) == 0x80000000) {
      if (!(t->status & (PSP_USB_ENDPOINT_STATUS_CANCEL | PSP_USB_ENDPOINT_STATUS_CANCELALL))) {
        req->transmitted += v->unk1 & 0xFFFF;
      }
    } else {
      Kprintf ("usbbd ERROR : IN : invalid descriptor's status bs=%d, stat=%d\n",
          v->unk1 >> 30, (v->unk1 >> 28) & 0x03);
    }

    v->unk1 |= 0xC0000000;
    if (!(t->status & (PSP_USB_ENDPOINT_STATUS_CLEARED | PSP_USB_ENDPOINT_STATUS_STALLED))) {
      if (t->status & PSP_USB_ENDPOINT_STATUS_CANCELALL) {
        usb_cancel_all (endpnum);
        return 0;
      }
      if (t->status & PSP_USB_ENDPOINT_STATUS_CANCEL) {
        sceUsbbdReqCancel (req);
        return 0;
      }
      if (req->transmitted <= req->size) {
        if (req->transmitted == req->size) {
          if ((req->size % t->queue->packet_size) == 0) {
            if (req->isControlRequest) {
              if (t->status & PSP_USB_ENDPOINT_STATUS_REM0) {
                configure_txdma (t);
                return 0;
              }
            }
          }
        } else {
          configure_txdma (t);
          return 0;
        }
      }

      if (endpnum != 0) {
        queue_pop (t->queue, PSP_USB_RETCODE_SUCCESS);
        return 0;
      }

      if (g_stalledendpoints != 0)
        return 0;

      if (g_ace0 == 0) {
        g_endpoints->status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
        _sw (_lw (USB_PORT_0200) | 0x100, USB_PORT_0200);
      }
      return 0;
    } else {

      if (t->status & PSP_USB_ENDPOINT_STATUS_CANCELALL) {
        usb_cancel_all (endpnum);
      }
      if (t->status & PSP_USB_ENDPOINT_STATUS_CLEARED) {
        usb_clear_fifo (endpnum);
      } else {
        usb_stall (endpnum);
      }
      return 0;
    }
  }
}

/* Subroutine at 0x66F8 */
static
int get_packet (struct usb_endpoint *t)
{
  volatile struct usb_port *v;
  struct UsbbdDeviceRequest *req;
  int endpnum;
  int size;
  int ret;


  v = t->ports[t->idx];
  endpnum = t->queue->endpoint->endpointNumber;
  req = t->queue->first;
  t->status &= ~PSP_USB_ENDPOINT_STATUS_DMACONFIGURED;
  if ((v->unk1 >> 30) == 2) {
    if (((v->unk1 >> 28) & 3) != 0) {
      if (((v->unk1 >> 28) & 3) == 1) {
        if (!(t->status & (PSP_USB_ENDPOINT_STATUS_CLEARED | PSP_USB_ENDPOINT_STATUS_STALLED))) {
          if (!req) {
            int usefirst;

            t->status &= ~PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
            _sw (_lw (USB_PORT_0200 + 32 * endpnum) | 0x80, USB_PORT_0200 + 32 * endpnum);

            if (t->queue->endpoint_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
              usefirst = TRUE;
            } else {
              usefirst = FALSE;
              t->status |= PSP_USB_ENDPOINT_STATUS_HASDATA;
            }

            configure_rxdma_fifo (t, usefirst);
            ret = 0;
          } else {
            label13:
            Kprintf ("usbbd ERROR : EP%d buffer error\n", endpnum);
            v->unk1 |= 0xC0000000;
            size = t->queue->packet_size;

            label36:
            if (t->status & PSP_USB_ENDPOINT_STATUS_CANCELALL) {
              usb_cancel_all (endpnum);
              ret = 0;
            } else {
              if (t->status & PSP_USB_ENDPOINT_STATUS_CANCEL) {
                sceUsbbdReqCancel (req);
                ret = 0;
              } else {
                if (size < t->queue->packet_size ||
                    req->size < req->transmitted ||
                    (req->size == req->transmitted && !req->isControlRequest)) {

                  queue_pop (t->queue, PSP_USB_RETCODE_SUCCESS);
                  ret = 0;
                  if (endpnum == 0) {
                    t->status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
                    _sw (_lw (USB_PORT_0000) | 0x100, USB_PORT_0000);
                    if (_lw (USB_PORT_0000) & 0x40) {
                      cancel_recv_all ();
                      g_ace0 = 2;
                    }
                  }
                } else {
                  configure_rxdma (t);
                  ret = 0;
                }
              }
            }
          }
        } else {
          v->unk1 |= 0xC0000000 | 0x08000000;
          ret = 0;
        }
      } else {
        goto label13;
      }
    } else {
      int bits = g_clearedendpoints | g_stalledendpoints;
      v->unk1 |= 0xC0000000;
      if ((bits >> endpnum) & 1) {
        configure_rxdma_fifo (t, TRUE);
        ret = 0;
      } else {
        if (!(t->status & (PSP_USB_ENDPOINT_STATUS_CLEARED | PSP_USB_ENDPOINT_STATUS_STALLED))) {
          if (!(t->status & PSP_USB_ENDPOINT_STATUS_HASDATA)) {
            if (endpnum == 0) {
              if (t->status & PSP_USB_ENDPOINT_STATUS_EP0_USE_FIFO) {
                queue_pop (t->queue, PSP_USB_RETCODE_SUCCESS);
                t->status &= ~PSP_USB_ENDPOINT_STATUS_EP0_USE_FIFO;
                return 0;
              }
            }
            size = v->unk1 & 0xFFFF;
            if (t->status & (PSP_USB_ENDPOINT_STATUS_CANCEL | PSP_USB_ENDPOINT_STATUS_CANCELALL))
              goto label36;
            ret = -1;
            if (req) {
              if (size > t->queue->packet_size) {
                Kprintf ("usbbd ERROR : Caution!!! received data size is over packet size\n");
              }
              req->transmitted += size;
              goto label36;
            }
          } else {
            struct usb_rxfifo *fifo = &g_rxfifos[endpnum];
            struct rxfifo_element *el = &fifo->elements[fifo->end];
            el->size = v->unk1 & 0xFFFF;
            fifo->end++;
            if (fifo->end < PSP_USB_FIFO_ELEMENTS) {
              el[1].ptr = el->ptr;
              el[1].ptr += ALIGN_UP (el->size, 3);
            }
            ret = 0;
            if (t->status & PSP_USB_ENDPOINT_STATUS_CANCELALL) {
              usb_cancel_all (endpnum);
            } else {
              if (t->status & PSP_USB_ENDPOINT_STATUS_CANCEL) {
                sceUsbbdReqCancel (req);
              } else {
                if (flush_fifo_data (endpnum) != 0) {
                  prepare_recv (endpnum);
                }
              }
            }
          }
        } else {
          if (t->status & PSP_USB_ENDPOINT_STATUS_CANCELALL) {
            usb_cancel_all (endpnum);
          }
          if (!(t->status & PSP_USB_ENDPOINT_STATUS_CLEARED)) {
            usb_stall (endpnum);
            ret = 0;
          } else {
            usb_clear_fifo (endpnum);
            ret = 0;
          }
        }
      }
    }
  } else {
    ret = -1;
    if ((t->ports[1 - t->idx]->unk1 >> 30) == 2) {
      ret = 0;
      if ((v->unk1 >> 30) == 0) {
        v->unk1 &= 0x3FFFFFFF & 0xCFFFFFFF;
        v->unk1 = (v->unk1 & 0xFFFF0000);
        v->unk1 |= 0x08000000;
        _sw (((unsigned int) v) & 0x1FFFFFFF, USB_PORT_RXDMA_BASE + 32 * endpnum);
        t->status |= PSP_USB_ENDPOINT_STATUS_DMACONFIGURED;
      }
    } else {
      if ((v->unk1 & 0xF0000000) == 0x40000000) {
        Kprintf ("usbbd ERROR : EP %d Invalid Rx DMA COMP interrupt. DMA is Busy\n", endpnum);
      } else {
        Kprintf ("usbbd ERROR : EP%d Rx DMA is not done : bs=%d, s=%d\n",
            endpnum, v->unk1 >> 30, (v->unk1 >> 28) & 3);
      }
    }
  }
  return ret;
}

/* Subroutine at 0x6BB0 */
static
int usb_interrupt_handler (void)
{
  int val;
  val = _lw (USB_PORT_040C);
  if (val != 0) {
    int intr = sceKernelCpuSuspendIntr ();
    struct UsbDriver *driver;
    int endpoint;

    if (val & 0x08) {

      if (g_connection_status == PSP_USB_STATUS_CONNECTION_ESTABLISHED) {

        for (driver = g_started_drivers; driver; driver = driver->link) {
          if (driver->detach)
            driver->detach ();
        }
        if (g_total_interfaces > 0) {
          for (endpoint = 0; endpoint < g_total_endpoints; endpoint++) {
            usb_cancel_all_skip_first (endpoint);
          }
        }
        g_connection_status = PSP_USB_STATUS_CONNECTION_NEW;
        refresh_usb_event_flag ();
      } else {
        usb_cancel_all_skip_first (0);
      }

      for (endpoint = 0; endpoint <= PSP_USB_MAX_ENDPOINTS; endpoint++) {
        g_endpoints[endpoint].queue->endpoint_dir = 255;
        g_endpoints[endpoint].status = 0;
        g_endpoints[endpoint].unk4 = 0;
      }

      g_ace0 = 0;
      g_isochronoustransfers = 0;
      g_clearedendpoints = 0;
      g_stalledendpoints = 0;

      if (g_total_interfaces > 0) {
        for (endpoint = 0; endpoint < g_total_endpoints; endpoint++) {
          usb_clear_fifo (endpoint);
        }
      }

      if (g_devdesc_hi) {
        _sw (0xFFFFFFB7, USB_PORT_0410);
      } else {
        set_usb_version (1);
      }

      _sw (0x08, USB_PORT_040C);
    }

    if (val & 0x40) {
      int new_version;
      _sw (0x40, USB_PORT_040C);
      if (_lw (USB_PORT_0408) & 0x6000) {
        new_version = 1;
      } else {
        new_version = 2;
      }
      if (new_version == g_usb_version) {
        _sw (0xFFFFFFA4, USB_PORT_0410);
      } else {
        set_usb_version (new_version);
      }
    }

    if (val & 0x20) {
      if (g_isochronoustransfers != 0) {
        int mask = g_isochronoustransfers >> 1;
        for (endpoint = 1; endpoint < g_total_endpoints; endpoint++) {
          mask >>= 1;
          if (mask & 1) {
            if ((g_endpoints[endpoint].status & PSP_USB_ENDPOINT_STATUS_DMACONFIGURED) == 0) {
              configure_txdma (&g_endpoints[endpoint]);
              _sw (_lw (USB_PORT_0410) | 0x20, USB_PORT_0410);
            }
          }
        }
      }
      _sw (0x20, USB_PORT_040C);
    }

    if (val & 0x04) {
      suspend_usb ();
      _sw (0x04, USB_PORT_040C);
    }

    if (val & 0x10) {
      suspend_usb ();
      _sw (0x10, USB_PORT_040C);
    }

    if (val & 0x2) {
      struct UsbConfiguration *conf;

      int val1 = _lw (USB_PORT_0408);
      int val2 = _lw (USB_PORT_0408);
      int interface_num = (val1 >> 4) & 0x0F;
      int alternate_setting = (val2 >> 8) & 0x0F;

      if (g_usb_version == 2) {
        conf = g_conf_hi;
      } else {
        conf = g_conf;
      }

      change_setting (interface_num, alternate_setting, conf->configDescriptors);

      for (driver = g_started_drivers; driver; driver = driver->link) {
        if (interface_num < (driver->interface->interfaceNumber + driver->interface->numInterfaces)) {
          if (driver->chageSetting) {
            driver->chageSetting (interface_num - driver->interface->interfaceNumber, alternate_setting);
          }
        }
      }

      process_request_default ();
      _sw (0x02, USB_PORT_040C);
    }

    if (val & 0x01) {
      int val1;
      val1 = _lw (USB_PORT_0408);
      if ((val1 & 0x0F) == 0) {
        if (g_connection_status == PSP_USB_STATUS_CONNECTION_ESTABLISHED) {
          for (driver = g_started_drivers; driver; driver = driver->link) {
            if (driver->detach)
              driver->detach ();
          }

          g_connection_status = PSP_USB_STATUS_CONNECTION_NEW;
          refresh_usb_event_flag ();

          for (endpoint = 0; endpoint <= PSP_USB_MAX_ENDPOINTS; endpoint++) {
            g_endpoints[endpoint].queue->endpoint_dir = 255;
            g_endpoints[endpoint].status = 0;
            g_endpoints[endpoint].unk4 = 0;
          }

          g_ace0 = 0;
          g_isochronoustransfers = 0;
          g_clearedendpoints = 0;
          g_stalledendpoints = 0;

          if (g_total_interfaces > 0) {
            for (endpoint = 0; endpoint < g_total_endpoints; endpoint++) {
              usb_cancel_all (endpoint);
              usb_clear_fifo (endpoint);
            }
          }

        }
      } else {
        struct UsbConfiguration *conf;

        if (g_usb_version == 2) {
          conf = g_conf_hi;
        } else {
          conf = g_conf;
        }

        change_all_settings (conf->configDescriptors);
        g_connection_status = PSP_USB_STATUS_CONNECTION_ESTABLISHED;
        refresh_usb_event_flag ();

        for (driver = g_started_drivers; driver; driver = driver->link) {
          if (driver->attach)
            driver->attach (g_usb_version);
        }
      }

      _sw (0x01, USB_PORT_040C);
    }

    sceKernelCpuResumeIntr (intr);
  }



  usb_interrupt ();
  return -1;

}

/* Subroutine at 0x70E0 */
static
int usb_connect_interrupt_handler (void)
{
  int ret;

  sceSysregUsbAcquireIntr (PSP_USB_INTERRUPT_CONNECT);
  g_cable_connected = PSP_USB_STATUS_CABLE_CONNECTED;
  refresh_usb_event_flag ();
  if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
    if (g_prev_cable_connected == PSP_USB_STATUS_CABLE_DISCONNECTED) {
      sceSysregUsbResetDisable ();
      ret = sceSysregUsbQueryIntr ();
      if (ret & PSP_USB_INTERRUPT_READY) {
        sceSysregUsbAcquireIntr (PSP_USB_INTERRUPT_READY);
      }
      SYNC ();
      sceKernelEnableIntr (PSP_USB_IRQ_READY);
    }
  }
  return -1;
}

/* Subroutine at 0x7178 */
static
int usb_disconnect_interrupt_handler (void)
{
  int ret;

  sceSysregUsbAcquireIntr (PSP_USB_INTERRUPT_DISCONNECT);
  g_cable_connected = PSP_USB_STATUS_CABLE_DISCONNECTED;
  if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
    if (g_prev_cable_connected == PSP_USB_STATUS_CABLE_DISCONNECTED) {
       usb_disconnect ();
       if (g_connection_status == PSP_USB_STATUS_CONNECTION_ESTABLISHED) {
         struct UsbDriver *driver;
         struct usb_endpoint *t;
         int i;

         ret = sceKernelCpuSuspendIntr ();
         driver = g_started_drivers;
         while (driver) {
           if (driver->detach) driver->detach ();
           driver = driver->link;
         }

         t = g_endpoints;
         for (i = 0; i <= PSP_USB_MAX_ENDPOINTS; i++) {
           struct usb_queue *queue = t->queue;
           t->status = 0;
           t->unk4 = 0;
           queue->endpoint_dir = 255;
           t++;
         }

         if (g_total_endpoints > 0) {
           i = 0;
           do {
             usb_cancel_all (i);
           } while (++i < g_total_endpoints);
         }

         sceKernelCpuResumeIntr (ret);
       }
    }
  }

  g_connection_status = PSP_USB_STATUS_CONNECTION_NEW;
  refresh_usb_event_flag ();
  return -1;
}

/* Subroutine at 0x72AC */
static
int usb_ready_interrupt_handler (void)
{
  sceSysregUsbAcquireIntr (PSP_USB_INTERRUPT_READY);
  if (g_cable_connected != PSP_USB_STATUS_CABLE_DISCONNECTED) {
    g_mainalarm = sceKernelSetAlarm (PSP_USB_MAIN_ALARM_TIME, &main_alarm_handler, g_endpoints);
    if (g_mainalarm < 0) {
      Kprintf ("usbbd ERROR : cannot set alarm handler 0x%08X\n", g_mainalarm);
    }
  }
  return -1;
}

/* Subroutine at 0x7324 */
static
int usb_resume_interrupt_handler (void)
{
  sceSysregUsbAcquireIntr (PSP_USB_INTERRUPT_RESUME);
  resume_handler ();
  return -1;
}

/* Subroutine at 0x734C */
static
void suspend_usb (void)
{
  struct UsbDriver *driver;
  int i;

  if (g_connection_status != PSP_USB_STATUS_CONNECTION_SUSPENDED) {
    if (g_connection_status == PSP_USB_STATUS_CONNECTION_ESTABLISHED) {
      for (driver = g_started_drivers; driver; driver = driver->link) {
        if (driver->detach)
          driver->detach ();
      }
    }
    g_connection_status = PSP_USB_STATUS_CONNECTION_SUSPENDED;
    refresh_usb_event_flag ();
    g_clearedendpoints = 0;
    g_stalledendpoints = 0;
    g_ace0 = 0;
    if (g_total_endpoints > 0) {
      for (i = 0; i < g_total_endpoints; i++) {
        g_endpoints[i].status = 0;
        g_endpoints[i].unk4 = 0;
        usb_cancel_all (i);
      }
    }
  }
}

/* Subroutine at 0x7438 */
static
void resume_handler (void)
{
  if (g_connection_status == PSP_USB_STATUS_CONNECTION_SUSPENDED) {
    int val = _lw (USB_PORT_0408);
    int endpoint = 0;

    if ((val & 0x0F) == 0) {
      g_connection_status = PSP_USB_STATUS_CONNECTION_NEW;
    } else {
      struct UsbDriver *driver;
      g_connection_status = PSP_USB_STATUS_CONNECTION_ESTABLISHED;
      for (driver = g_started_drivers; driver; driver = driver->link) {
        if (driver->attach) {
          driver->attach (g_usb_version);
        }
      }
    }

    refresh_usb_event_flag ();
    if (g_total_endpoints > 0) {
      for (endpoint = 0; endpoint < g_total_endpoints; endpoint++) {
        usb_clear_fifo (endpoint);
      }
    }
  }
}

