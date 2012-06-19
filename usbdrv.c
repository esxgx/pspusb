#include <pspkernel.h>
#include <string.h>

#include "usb.h"
#include "usb_internal.h"

/* Variables */
struct UsbDriver *g_started_drivers;        /* 0xA880 */
struct UsbDriver *g_registered_drivers;     /* 0xAE80 */

int g_busdrv_started;                       /* 0xA95C */

static int (*g_busdrv_start) (int size, void *args) = &usb_bus_driver_start; /* 0xA880 + 388 = 0xAA04 */
static int (*g_busdrv_stop) (int size, void *args) = &usb_bus_driver_stop;   /* 0xA880 + 392 = 0xAA08 */

/* Function Prototypes */
static struct UsbDriver *find_driver_by_name (struct UsbDriver *list, const char *name);
static struct UsbDriver *find_driver_by_name2 (struct UsbDriver *list, struct UsbDriver **previous, const char *name);
static struct UsbDriver *add_driver (struct UsbDriver *list, struct UsbDriver *drv);
static int check_driver_configuration (struct UsbDriver *drv);
static int check_max_packet_size (unsigned int endpoint_address, unsigned int transfer_type, int max_length, int usb_version);

/* Subroutine at 0x7818 */
int sceUsbStart (const char* driverName, int size, void *args)
{
  SceUInt timeout = DEFAULT_USB_TIMEOUT;
  register int k1;
  int ret;


  if (driverName) {
    SAVE_K1 ();
    if (sceKernelIsIntrContext ()) {
      Kprintf ("usbbd ERROR : illegal context\n");
      ret = PSP_USB_ERROR_ILLEGAL_CONTEXT;
    } else {
      int len = strlen (driverName);
      if (IS_USER_MODE ()) {
        if (CHECK_RANGE (driverName, len) ||
            CHECK_RANGE (args, size)) {
          Kprintf ("usbbd ERROR : invalid pointer name=0x%p, arg=0x%p\n", driverName, args);
          RESTORE_K1 ();
          return PSP_USB_ERROR_INVALID_POINTER;
        }
      }
      ret = sceKernelWaitSema (g_mainsema, 1, &timeout);
      if (ret < 0) {
        Kprintf ("usbbd ERROR : Start() wait semaphore : 0x%08x\n", ret);
      } else {
        if (strncmp (driverName, PSP_USB_BUS_DRIVERNAME, 12) == 0) {
          if (g_busdrv_started == 1) {
            Kprintf ("usbbd ERROR : bus driver is already started\n");
            ret = PSP_USB_ERROR_ALREADY_DONE;
          } else {
            ret = g_busdrv_start (size, args);
            if (ret >= 0) {
              ret = 0;
              g_busdrv_started = 1;
            }
          }
        } else {
          if (!g_busdrv_started) {
            Kprintf ("usbbd ERROR : bus driver is not started\n");
            ret = PSP_USB_ERROR_BUS_DRIVER_NOT_STARTED;
          } else {
            if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
              Kprintf ("usbbd ERROR : device is in progress\n");
              ret = PSP_USB_ERROR_DRIVER_IN_PROGRESS;
            } else {
              struct UsbDriver *previous;
              struct UsbDriver *found;

              found = find_driver_by_name2 (g_registered_drivers, &previous, driverName);
              if (!found) {
                if (find_driver_by_name (g_started_drivers, driverName)) {
                  Kprintf ("usbbd ERROR : already started %s\n", driverName);
                  ret = PSP_USB_ERROR_ALREADY_DONE;
                } else {
                  Kprintf ("usbbd ERROR : cannot find specified driver\n");
                  ret = PSP_USB_ERROR_DRIVER_NOT_FOUND;
                }
              } else {
                ret = found->start (size, args);
                if (ret >= 0) {
                  if (check_driver_configuration (found)) {
                    ret = PSP_USB_ERROR_INVALID_ARGUMENT;
                  } else {
                    if (!previous) {
                      g_registered_drivers = found->link;
                    } else {
                      previous->link = found->link;
                    }
                    if (!add_driver (g_started_drivers, found)) {
                      g_started_drivers = found;
                    }
                    if (found->interface->expectNumber < 0) {
                      found->interface->expectNumber = -1;
                    }
                    g_total_interfaces += found->interface->numInterfaces;
                    g_total_endpoints += found->numEndpoints - 1;
                    ret = 0;
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
  } else {
    Kprintf ("usbbd ERROR : Null pointer\n");
    return PSP_USB_ERROR_INVALID_POINTER;
  }
}

/* Subroutine at 0x7B28 */
int sceUsbStop (const char* driverName, int size, void *args)
{
  SceUInt timeout = DEFAULT_USB_TIMEOUT;
  register int k1;
  int ret;


  if (driverName) {
    SAVE_K1 ();
    if (sceKernelIsIntrContext ()) {
      Kprintf ("usbbd ERROR : illegal context\n");
      ret = PSP_USB_ERROR_ILLEGAL_CONTEXT;
    } else {
      int len = strlen (driverName);
      if (IS_USER_MODE ()) {
        if (CHECK_RANGE (driverName, len) ||
            CHECK_RANGE (args, size)) {
          Kprintf ("usbbd ERROR : invalid pointer name=0x%p, arg=0x%p\n", driverName, args);
          RESTORE_K1 ();
          return PSP_USB_ERROR_INVALID_POINTER;
        }
      }
      ret = sceKernelWaitSema (g_mainsema, 1, &timeout);
      if (ret < 0) {
        Kprintf ("usbbd ERROR : Stop() wait semaphore : 0x%08x\n", ret);
      } else {
        if (g_usb_activated == PSP_USB_STATUS_ACTIVATED) {
          Kprintf ("usbbd ERROR : device is in progress\n");
          ret = PSP_USB_ERROR_DRIVER_IN_PROGRESS;
        } else {
          if (strncmp (driverName, PSP_USB_BUS_DRIVERNAME, 12) == 0) {
            if (!g_busdrv_started) {
              Kprintf ("usbbd ERROR : bus driver is already stopped\n");
              ret = PSP_USB_ERROR_ALREADY_DONE;
            } else {
              ret = g_busdrv_stop (size, args);
              if (ret >= 0) {
                g_busdrv_started = 0;
                ret = 0;
              }
            }
          } else {
            if (!g_busdrv_started) {
              Kprintf ("usbbd ERROR : bus driver is not started\n");
              ret = PSP_USB_ERROR_BUS_DRIVER_NOT_STARTED;
            } else {
              struct UsbDriver *previous;
              struct UsbDriver *found;

              found = find_driver_by_name2 (g_started_drivers, &previous, driverName);
              if (!found) {
                if (find_driver_by_name (g_registered_drivers, driverName)) {
                  Kprintf ("usbbd ERROR : already stopped %s\n", driverName);
                  ret = PSP_USB_ERROR_ALREADY_DONE;
                } else {
                  Kprintf ("usbbd ERROR : cannot find specified driver\n");
                  ret = PSP_USB_ERROR_DRIVER_NOT_FOUND;
                }
              } else {
                ret = found->stop (size, args);
                if (ret >= 0) {
                  if (!previous) {
                    g_started_drivers = found->link;
                  } else {
                    previous->link = found->link;
                  }
                  if (!add_driver (g_registered_drivers, found)) {
                    g_registered_drivers = found;
                  }
                  g_total_endpoints -= found->numEndpoints - 1;
                  g_total_interfaces -= found->interface->numInterfaces;
                  ret = 0;
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
  } else {
    Kprintf ("usbbd ERROR : Null pointer\n");
    return PSP_USB_ERROR_INVALID_POINTER;
  }
}

/* Subroutine at 0x7E08 */
int sceUsbGetDrvList (unsigned int flags, struct UsbDriverName *list, int size)
{
  SceUInt timeout = DEFAULT_USB_TIMEOUT;
  register int k1;
  int ret;


  if (size && !list) {
    Kprintf ("usbbd ERROR : Null pointer\n");
    return PSP_USB_ERROR_INVALID_POINTER;
  }

  if ((flags & (PSP_USB_STATUS_DRIVER_REGISTERED | PSP_USB_STATUS_DRIVER_STARTED)) != 0) {
    SAVE_K1 ();
    if (sceKernelIsIntrContext ()) {
      Kprintf ("usbbd ERROR : illegal context\n");
      ret = PSP_USB_ERROR_ILLEGAL_CONTEXT;
    } else {
      if (IS_USER_MODE ()) {
        if (CHECK_RANGE (list, size * sizeof (struct UsbDriverName))) {
          Kprintf ("usbbd ERROR : invalid pointer\n");
          RESTORE_K1 ();
          return PSP_USB_ERROR_INVALID_POINTER;
        }
      }
      if (size > 0) {
        int i;
        for (i = 0; i < size; i++) {
          if (list[i].size != sizeof (struct UsbDriverName)) {
            Kprintf ("usbbd ERROR : %dth : size of structure is invalid (%d -> %d)\n", i, sizeof (struct UsbDriverName), list[i].size);
            RESTORE_K1 ();
            return PSP_USB_ERROR_INVALID_VALUE;
          }
        }
      }
      ret = sceKernelWaitSema (g_mainsema, 1, &timeout);
      if (ret < 0) {
        Kprintf ("usbbd ERROR : DrvList() wait semaphore : 0x%08x\n", ret);
      } else {
        int count = 0;
        struct UsbDriver *drv;
        if (flags & PSP_USB_STATUS_DRIVER_STARTED) {
          if (g_started_drivers) {
            for (drv = g_started_drivers; drv; drv = drv->link) {
              if (count < size) {
                int len = strlen (drv->driverName);
                if (len >= 32) {
                  len = 31;
                }
                strncpy (list[count].name, drv->driverName, len);
                list[count].name[len] = '\0';
                list[count].flags = flags;
              }
              count++;
            }
          }
        }
        if (flags & PSP_USB_STATUS_DRIVER_REGISTERED) {
          if (g_registered_drivers) {
            flags = PSP_USB_STATUS_DRIVER_REGISTERED;
            for (drv = g_registered_drivers; drv; drv = drv->link) {
              if (count < size) {
                int len = strlen (drv->driverName);
                if (len >= 32) {
                  len = 31;
                }
                strncpy (list[count].name, drv->driverName, len);
                list[count].name[len] = '\0';
                list[count].flags = flags;
              }
              count++;
            }
          }
        }
        ret = count;
        while (count < size) {
          list[count].flags = 0;
          list[count].name[0] = '\0';
          count++;
        }
        sceKernelSignalSema (g_mainsema, 1);
      }
    }
    RESTORE_K1 ();
    return ret;
  } else {
    Kprintf ("usbbd ERROR : invalid flag\n");
    return PSP_USB_ERROR_INVALID_FLAG;
  }
}

/* Subroutine at 0x80D4 */
int sceUsbGetDrvState (const char* driverName)
{
  SceUInt timeout = DEFAULT_USB_TIMEOUT;
  register int k1;
  int ret;


  if (driverName) {
    SAVE_K1 ();
    if (sceKernelIsIntrContext ()) {
      Kprintf ("usbbd ERROR : illegal context\n");
      ret = PSP_USB_ERROR_ILLEGAL_CONTEXT;
    } else {
      int len = strlen (driverName);
      if (IS_USER_MODE ()) {
        if (CHECK_RANGE (driverName, len)) { /* Is this correct? */
          Kprintf ("usbbd ERROR : invalid pointer\n");
          RESTORE_K1 ();
          return PSP_USB_ERROR_INVALID_POINTER;
        }
      }
      ret = sceKernelWaitSema (g_mainsema, 1, &timeout);
      if (ret < 0) {
        Kprintf ("usbbd ERROR : DrvState() wait semaphore : 0x%08x\n", ret);
      } else {
        if (strncmp (driverName, PSP_USB_BUS_DRIVERNAME, 12) == 0) {
          if (g_busdrv_started) {
            ret = PSP_USB_STATUS_DRIVER_STARTED;
          } else {
            ret = PSP_USB_STATUS_DRIVER_REGISTERED;
          }
        } else {
          if (find_driver_by_name (g_started_drivers, driverName)) {
            ret = PSP_USB_STATUS_DRIVER_STARTED;
          } else if (find_driver_by_name (g_registered_drivers, driverName)) {
            ret = PSP_USB_STATUS_DRIVER_REGISTERED;
          } else {
            Kprintf ("usbbd ERROR : cannot find specified driver : %s\n", driverName);
            ret = PSP_USB_ERROR_DRIVER_NOT_FOUND;
          }
        }
        sceKernelSignalSema (g_mainsema, 1);
      }
    }
    RESTORE_K1 ();
    return ret;
  } else {
    Kprintf ("usbbd ERROR : Null pointer\n");
    return PSP_USB_ERROR_INVALID_POINTER;
  }
}


/* Subroutine at 0x826C */
static
struct UsbDriver *find_driver_by_name (struct UsbDriver *list, const char *name)
{
  if (list) {
    int r, len;
    do {
      len = strlen (list->driverName);
      r = strncmp (name, list->driverName, len);
      if (r == 0) break;
      list = list->link;
    } while (list);
  }
  return list;
}

/* Subroutine at 0x82CC */
static
struct UsbDriver *find_driver_by_name2 (struct UsbDriver *list, struct UsbDriver **previous, const char *name)
{
  *previous = NULL;
  if (list) {
    int r, len;

    do {
      len = strlen (list->driverName);
      r = strncmp (name, list->driverName, len);
      if (r == 0) break;
      *previous = list;
      list = list->link;
    } while (list);
  }
  return list;
}


/* Subroutine at 0x8340 */
static
struct UsbDriver *add_driver (struct UsbDriver *list, struct UsbDriver *drv)
{
  if (list) {
    if (!list->link) {
      list->link = drv;
    }
    while (list->link) list = list->link;
    list->link = drv;
  }
  drv->link = NULL;
  return list;
}

/* Subroutine at 0x8374 */
int sceUsbbdRegister (struct UsbDriver *drv)
{
  SceUInt timeout = DEFAULT_USB_TIMEOUT;
  int ret;


  if (drv) {
    if (!drv->driverName) {
      Kprintf ("usbbd ERROR : need driver name\n");
      return PSP_USB_ERROR_INVALID_ARGUMENT;
    }
    if (!drv->start || !drv->stop) {
      Kprintf ("usbbd ERROR : need start/stop entry\n");
      return PSP_USB_ERROR_INVALID_ARGUMENT;
    }
    if (drv->interface->expectNumber < 0) {
      if (drv->interface->numInterfaces > PSP_USB_MAX_INTERFACES) {
        Kprintf ("usbbd ERROR : number of interfaces %d is over limitation %d\n", drv->interface->numInterfaces, PSP_USB_MAX_INTERFACES);
        return PSP_USB_ERROR_INVALID_ARGUMENT;
      }
    } else {
      if (drv->interface->expectNumber + drv->interface->numInterfaces > PSP_USB_MAX_INTERFACES) {
        Kprintf ("usbbd ERROR : interface number is over limitation\n");
        Kprintf ("usbbd ERROR :      expect=%d, num=%d, max=%d\n", drv->interface->expectNumber, drv->interface->numInterfaces, PSP_USB_MAX_INTERFACES);
        return PSP_USB_ERROR_INVALID_ARGUMENT;
      }
    }
    ret = sceKernelWaitSema (g_mainsema, 1, &timeout);
    if (ret < 0) {
      Kprintf ("usbbd ERROR : Register() wait semaphore : 0x%08x\n", ret);
      return ret;
    }
    if (find_driver_by_name (g_started_drivers, drv->driverName) ||
        find_driver_by_name (g_registered_drivers, drv->driverName)) {
      Kprintf ("usbbd ERROR : already registered %s\n", drv->driverName);
      ret = PSP_USB_ERROR_ALREADY_DONE;
    } else {
      if (!add_driver (g_registered_drivers, drv)) {
        g_registered_drivers = drv;
      }
      ret = 0;
    }

    sceKernelSignalSema (g_mainsema, 1);
    return ret;
  } else {
    Kprintf ("usbbd ERROR : invalid argument\n");
    return PSP_USB_ERROR_INVALID_ARGUMENT;
  }
}


/* Subroutine at 0x8518 */
int sceUsbbdUnregister (struct UsbDriver *drv)
{
  SceUInt timeout = DEFAULT_USB_TIMEOUT;
  int ret;


  if (drv) {
    struct UsbDriver *previous;
    struct UsbDriver *found;

    ret = sceKernelWaitSema (g_mainsema, 1, &timeout);
    if (ret < 0) {
      Kprintf ("usbbd ERROR : Register() wait semaphore : 0x%08x\n", ret);
      return ret;
    }
    ret = 0;
    found = find_driver_by_name2 (g_registered_drivers, &previous, drv->driverName);
    if (!found) {
      found = find_driver_by_name (g_started_drivers, drv->driverName);
      if (!found) {
        Kprintf ("usbbd ERROR : cannot find specified driver\n");
        ret = PSP_USB_ERROR_DRIVER_NOT_FOUND;
      } else {
        Kprintf ("usbbd ERROR : driver is in progress\n");
        ret = PSP_USB_ERROR_DRIVER_IN_PROGRESS;
      }
    } else {
      if (!previous) {
        g_registered_drivers = found->link;
      } else {
        previous->link = found->link;
      }
    }
    sceKernelSignalSema (g_mainsema, 1);
    return ret;
  } else {
    Kprintf ("usbbd ERROR : invalid argument\n");
    return PSP_USB_ERROR_INVALID_ARGUMENT;
  }
}

/* Subroutine at 0x8634 */
static
int check_driver_configuration (struct UsbDriver *drv)
{
  if (drv->descriptor == NULL) {
    Kprintf ("usbbd ERROR : no device descriptor.\n");
    return -1;
  }
  if (drv->configuration == NULL) {
    Kprintf ("usbbd ERROR : no configuration.\n");
    return -1;
  }
  if (check_max_packet_size (0, USB_ENDPOINT_TYPE_CONTROL, drv->descriptor->bMaxPacketSize0, 1)) {
    return -1;
  }
  if (drv->configuration->configDescriptors == NULL) {
    Kprintf ("usbbd ERROR : no configuration descriptor\n");
    return -1;
  }
  if (drv->configuration->settings == NULL) {
    Kprintf ("usbbd ERROR : no interface\n");
    return -1;
  }
  if (drv->configuration->configDescriptors->bNumInterfaces != 0) {
    struct InterfaceSettings *setting = drv->configuration->settings;
    int count = 0;
    do {
      count++;
      if (setting->numDescriptors > PSP_USB_MAX_ALTERNATE) {
        Kprintf ("usbbd ERROR : number of alternate settings %d is too many\n", setting->numDescriptors);
        return -1;
      }
      setting++;
    } while (count < drv->configuration->configDescriptors->bNumInterfaces);
  }
  if (drv->configuration->interfaceDescriptors == NULL) {
    Kprintf ("usbbd ERROR : no interface descriptor\n");
    return -1;
  }

  if (drv->configuration->endpointDescriptors) {
    struct EndpointDescriptor *endp = drv->configuration->endpointDescriptors;
    while (endp->bLength) {
      if (check_max_packet_size (endp->bEndpointAddress & USB_ENDPOINT_ADDRESS_MASK,
                                 endp->bmAttributes & USB_ENDPOINT_TYPE_MASK,
                                 endp->wMaxPacketSize, 1)) {
        return -1;
      }
      endp++;
    }
  }
  if (drv->descriptor_hi == NULL) {
    if (drv->configuration_hi == NULL) {
      return 0;
    } else {
      Kprintf ("usbbd ERROR : there is no device desc or config for high speed\n");
      return -1;
    }
  } else {
    if (drv->configuration_hi == NULL) {
      Kprintf ("usbbd ERROR : there is no device desc or config for high speed\n");
      return -1;
    } else {
      if (drv->descriptor->bcdUSB < 0x200) {
        return 0;
      }
      if (check_max_packet_size (0, USB_ENDPOINT_TYPE_CONTROL, drv->descriptor_hi->bMaxPacketSize0, 2)) {
        return -1;
      }
      if (drv->configuration_hi->configDescriptors == NULL) {
        Kprintf ("usbbd ERROR : no configuration descriptor for high speed\n");
        return -1;
      }

      if (drv->configuration_hi->settings == NULL) {
        Kprintf ("usbbd ERROR : no interface for high speed\n");
        return -1;
      }
      if (drv->configuration_hi->configDescriptors->bNumInterfaces != 0) {
        struct InterfaceSettings *setting = drv->configuration_hi->settings;
        int count = 0;
        do {
          count++;
          if (setting->numDescriptors > PSP_USB_MAX_ALTERNATE) {
            Kprintf ("usbbd ERROR : number of alternate settings for high speed %d is too many\n", setting->numDescriptors);
            return -1;
          }
          setting++;
        } while (count < drv->configuration_hi->configDescriptors->bNumInterfaces);
      }

      if (drv->configuration_hi->interfaceDescriptors == NULL) {
        Kprintf ("usbbd ERROR : no interface descriptor for high speed\n");
        return -1;
      }

      if (drv->configuration_hi->endpointDescriptors) {
        struct EndpointDescriptor *endp = drv->configuration_hi->endpointDescriptors;
        while (endp->bLength) {
          if (check_max_packet_size (endp->bEndpointAddress & USB_ENDPOINT_ADDRESS_MASK,
                                     endp->bmAttributes & USB_ENDPOINT_TYPE_MASK,
                                     endp->wMaxPacketSize, 2)) {
            return -1;
          }
          endp++;
        }
      }

      return 0;
    }
  }
}

/* Subroutine at 0x88B0 */
static
int check_max_packet_size (unsigned int endpoint_address, unsigned int transfer_type, int max_length, int usb_version)
{
  if (transfer_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
    if (usb_version == 2) {
      if (max_length >= 1025) {
        Kprintf ("usbbd ERROR : EP%d High Speed Isochronous : max packet size %d is invalid\n", endpoint_address, max_length);
        return -1;
      }
    } else {
      if (max_length >= 1024) {
        Kprintf ("usbbd ERROR : EP%d Full Speed Isochronous : max packet size %d is invalid\n", endpoint_address, max_length);
        return -1;
      }
    }
  } else if (transfer_type == USB_ENDPOINT_TYPE_BULK) {
    if (usb_version == 2) {
      if (max_length != 512) {
        Kprintf ("usbbd ERROR : EP%d High Speed Bulk : max packet size %d is invalid\n", endpoint_address, max_length);
        return -1;
      }
    } else {
      if (max_length != 8 &&
          max_length != 16 &&
          max_length != 32 &&
          max_length != 64) {
        Kprintf ("usbbd ERROR : EP%d Full Speed Bulk : max packet size %d is invalid\n", endpoint_address, max_length);
        return -1;
      }
    }
  } else if (transfer_type == USB_ENDPOINT_TYPE_INTERRUPT) {
    if (usb_version == 2) {
      if (max_length >= 1025) {
        Kprintf ("usbbd ERROR : EP%d High Speed Interrupt : max packet size %d is invalid\n", endpoint_address, max_length);
        return -1;
      }
    } else {
      if (max_length >= 65) {
        Kprintf ("usbbd ERROR : EP%d Full Speed Interrupt : max packet size %d is invalid\n", endpoint_address, max_length);
        return -1;
      }
    }
  } else if (transfer_type == USB_ENDPOINT_TYPE_CONTROL) {
    if (usb_version == 2) {
      if (max_length != 64) {
        Kprintf ("usbbd ERROR : EP%d High Speed Control : max packet size %d is invalid\n", endpoint_address, max_length);
        return -1;
      }
    } else {
      if (max_length != 8 &&
          max_length != 16 &&
          max_length != 32 &&
          max_length != 64) {
        Kprintf ("usbbd ERROR : EP%d Full Speed Control : max packet size %d is invalid\n", endpoint_address, max_length);
        return -1;
      }
    }
  } else {
    Kprintf ("usbbd ERROR : EP%d : transfer type 0x%0x is invalid\n", endpoint_address, transfer_type);
    return -1;
  }
  return 0;
}

/* Subroutine at 0x8A30 */
int clear_registered_protocol_drivers (void)
{
  g_registered_drivers = NULL;
  return 0;
}

/* Subroutine at 0x8A40 */
int check_registered_protocol_drivers (void)
{
  if (g_registered_drivers) {
    Kprintf ("usbbd ERROR : registered protocol driver exist\n");
    return -1;
  }
  return 0;
}


