#include <pspkernel.h>
#include <pspidstorage.h>
#include <string.h>

#include "usb.h"
#include "usb_internal.h"

/* Structures */
struct StorageStringDescriptor {
  unsigned int id;
  struct StringDescriptor str;
};

/* Variables */
struct DeviceDescriptor *g_devdesc_hi;        /* 0xA97C */
struct UsbConfiguration *g_conf_hi;           /* 0xA980 */
struct DeviceDescriptor *g_devdesc;           /* 0xA984 */
struct UsbConfiguration *g_conf;              /* 0xA988 */
struct StringDescriptor *g_descriptors;       /* 0xA98C */
struct DeviceQualifierDescriptor *g_qual_hi;  /* 0xA990 */
struct DeviceQualifierDescriptor *g_qual;     /* 0xA994 */
static void *g_configpacket_hi;               /* 0xA998 */
static void *g_configpacket;                  /* 0xA99C */
struct StringDescriptor *g_string;            /* 0xA9A0 */
struct StringDescriptor *g_vendor_desc;       /* 0xA9A4 */
struct StringDescriptor *g_product_desc;      /* 0xA9A8 */
static int g_desccount;                       /* 0xA9AC */

struct DeviceRequest g_devreq;                /* 0xA9B0 */
struct UsbEndpoint g_ep0;                     /* 0xA9B8 */
struct UsbbdDeviceRequest g_request;          /* 0xA9C4 */

SceUID g_devdescfpl;                          /* 0xA9F4 */
SceUID g_fsconffpl;                           /* 0xA9F8 */
SceUID g_hsconffpl;                           /* 0xA9FC */

SceUID g_ep0fpl;                              /* 0xAA00 */

static struct StringDescriptor g_default_desc = {
    4,
    3,
    { 1033  }
};                                            /* 0xAA0C */


/* Function Prototypes */
static int load_device_strings (unsigned int productId, int strcount[9]);
static struct UsbConfiguration *load_configuration (int usb_version, int strcount[9]);

static int read_idstorage (int pid, struct StorageStringDescriptor *vendor_desc, struct StorageStringDescriptor *product_desc);

static void make_device_qualifier (struct DeviceDescriptor *device, struct DeviceQualifierDescriptor *qualifier);
static void make_packet (struct ConfigDescriptor *from, char *buffer);

static int process_class_request (void);
static int process_standard_request (struct DeviceRequest *req);

static int configure_endpoint (struct EndpointDescriptor *endp);
static void clear_endpoint_status (int endpoint);


/* Subroutine at 0x0760 */
void usb_memcpy (void *dest, void *src, int size)
{
  char *destination = dest;
  char *source = src;

  if (((((unsigned int) dest) | ((unsigned int) src)) & 0x3) == 0) {
    int rem = size % 0x03;
    size -= rem;
    memcpy (dest, src, size);
    if (rem == 0) return;

    destination += size; source += size;
    size = rem;
  }

  if (size <= 0) return;
  while (size--) {
    *destination = *source;
    destination++; source++;
  }
}

/* Subroutine at 0x0850 */
int check_device_class (void)
{
  struct UsbDriver *driver;
  struct UsbDriver *next;

  driver = g_started_drivers;
  if (!driver) return 0;

  next = driver->link;
  if (next) {
    struct DeviceDescriptor *devp;
    devp = driver->descriptor;
    while (1) {
      if (devp->bDeviceClass != USB_CLASS_PER_INTERFACE) {
        Kprintf ("usbbd ERROR : %s has Device Class : 0x%02x.\nSo cannot activate with other protocol drivers\n",
            driver->driverName, devp->bDeviceClass);
        return -1;
      }
      driver = driver->link;
      if (!driver) break;
      devp = driver->descriptor;
    }
  }
  return 0;
}

/* Subroutine at 0x08B8 */
int assign_interfaces (void)
{
  int mark[8], interfaceNumber;

  if (g_started_drivers) {
    struct UsbDriver *driver = g_started_drivers;
    while (driver) {
      driver->interface->interfaceNumber = -1;
      driver = driver->link;
    }
  }

  for (interfaceNumber = 0; interfaceNumber < 8; interfaceNumber++)
    mark[interfaceNumber] = -2;

  if (g_started_drivers) {
    struct UsbDriver *driver;
    for (driver = g_started_drivers; driver; driver = driver->link) {
      struct UsbInterface *interface = driver->interface;
      int interfnum;

      if (interface->expectNumber == -1)
        continue;

      interfnum = interface->expectNumber;
      if (interface->expectNumber + interface->numInterfaces > interface->expectNumber) {
        while (1) {
          if (mark[interfnum] != -2) {
            Kprintf ("usbbd ERROR : overlap interface #%d\n", interfnum);
            return -1;
          }
          mark[interfnum++] = interface->expectNumber;
          if (interfnum >= interface->expectNumber + interface->numInterfaces)
            break;
        }
      }

      interface->interfaceNumber = interface->expectNumber;
    }
  }

  if (g_total_interfaces > 0) {
    int interfnum;
    for (interfnum = 0; interfnum < g_total_interfaces; interfnum++) {
      if (mark[interfnum] == -2) {
        struct UsbDriver *driver = g_started_drivers;
        if (driver) {
          for (; driver; driver = driver->link) {
            struct UsbInterface *interface = driver->interface;
            if (interface->expectNumber == -1) {
              if (interface->interfaceNumber != -1)
                continue;

              if (interface->numInterfaces > 0) {
                int *ptr = &mark[interfnum];
                int count = 0;
                while (count < interface->numInterfaces) {
                  if (*ptr++ != -2) break;
                  count++;
                }
                if (count < interface->numInterfaces)
                  continue;

                count = 0;
                ptr = &mark[interfnum];
                while (count < interface->numInterfaces) {
                  *ptr++ = interfnum;
                  count++;
                }
                interface->interfaceNumber = interfnum;
              }

            }
          }
        }
        if (mark[interfnum] == -2) {
          Kprintf ("usbbd ERROR : interface number is not continuous\n");
          return -1;
        }
      }
    }
  }
  return 0;
}

/* Subroutine at 0x0B28 */
void assign_endpoints (void)
{
  int count = 0;
  if (g_started_drivers) {
    struct UsbDriver *driver;
    for (driver = g_started_drivers; driver; driver = driver->link) {
      struct UsbEndpoint *endpoint = driver->endpoints;
      int i = 0;
      endpoint->endpointNumber = 0;
      if (driver->numEndpoints > 1) {
        for (i = 1; i < driver->numEndpoints; i++) {
          int endpointNumber;
          endpointNumber = driver->endpoints[i].endpointNumber = count + driver->endpoints[i].driverEndpointNumber;
          g_queues[endpointNumber].endpoint = &driver->endpoints[i];
        }
      }
      count = count + driver->numEndpoints - 1;
    }
  }
}

/* Subroutine at 0x0BB4 */
int load_device_configuration (unsigned int productId)
{
  int strcount[9];
  int ret;

  ret = load_device_strings (productId, strcount);
  if (ret < 0) return -1;

  g_conf = load_configuration (1, strcount);
  if (!g_conf) {
    if (sceKernelDeleteFpl (g_devdescfpl) < 0)
      Kprintf ("usbbd ERROR : cannot delete FPL for device desc\n");
  }

  if (g_devdesc_hi) {
    g_conf_hi = load_configuration (2, strcount);

    if (!g_conf_hi) {
      if (sceKernelDeleteFpl (g_fsconffpl) < 0)
        Kprintf ("usbbd ERROR : cannot delete FPL for fs config\n");
      if (sceKernelDeleteFpl (g_devdescfpl) < 0)
        Kprintf ("usbbd ERROR : cannot delete FPL for device desc\n");
      return -1;
    }
  }

  return 0;
}

/* Subroutine at 0x0C90 */
static
int load_device_strings (unsigned int productId, int strcount[9])
{
  int driver_num, usb_version, ret, *num_desc;
  unsigned short min_usb;
  unsigned char  min_packet0;
  struct StorageStringDescriptor vendor_desc, product_desc;
  int vendorId;
  int size;
  char *ptr;

  strcount[8] = 0;
  driver_num = 0;
  usb_version = 2;
  min_packet0 = 0;
  min_usb = 0;

  num_desc = &strcount[0];
  if (g_started_drivers) {
    struct UsbDriver *driver; /* var2 */
    for (driver = g_started_drivers; driver; driver = driver->link) {
      if (driver->descriptor_hi == NULL) {
        usb_version = 1;
      } else {
        if (driver->configuration_hi == NULL) {
          usb_version = 1;
        }
      }
      if (driver_num != 0) {
        if (min_usb > driver->descriptor->bcdUSB)
          min_usb = driver->descriptor->bcdUSB;
        if (min_packet0 > driver->descriptor->bMaxPacketSize0)
          min_packet0 = driver->descriptor->bMaxPacketSize0;
      } else {
        min_usb = driver->descriptor->bcdUSB;
        min_packet0 = driver->descriptor->bMaxPacketSize0;
      }

      *num_desc = 0;

      if (driver->stringDescriptors) {
        struct StringDescriptor *sd;
        for (sd = driver->stringDescriptors; sd->bLength; sd++) {
          (*num_desc)++;
        }
      }

      driver_num++;
      strcount[8] += *num_desc;
      num_desc++;
    }
  } else {
    usb_version = 1;
  }

  if (min_usb < 0x200)
    usb_version = 1;

  ret = read_idstorage (productId, &vendor_desc, &product_desc);
  if (ret < 2 * sizeof (struct StringDescriptor)) {
    g_product_desc = NULL;
    product_desc.id = -1;
  }

  if (ret < sizeof (struct StringDescriptor)) {
    vendorId = 1356;
    vendor_desc.id = -1;
    g_vendor_desc = NULL;
  } else {
    vendorId = vendor_desc.id;
  }

  if (usb_version == 2) {
    size = ret + 2 * sizeof (struct DeviceDescriptor);
  } else {
    size = ret + sizeof (struct DeviceDescriptor);
    g_conf_hi = NULL;
    g_devdesc_hi = NULL;
  }

  size += (strcount[8] + 1) * sizeof (struct StringDescriptor) /* 64 */;
  g_devdescfpl = sceKernelCreateFpl ("SceUsbDevDesc", PSP_MEMORY_PARTITION_KERNEL, 0x00000100, size, 1, NULL);
  if (g_devdescfpl < 0) {
    Kprintf ("usbbd ERROR : cannot create FPL to device desc\n");
    return -1;
  }

  ret = sceKernelTryAllocateFpl (g_devdescfpl, (void **) &ptr);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot allocate memory to device desc\n");
    ret = sceKernelDeleteFpl (g_devdescfpl);
    if (ret < 0) {
      Kprintf ("usbbd ERROR : cannot delete FPL for device desc\n");
      return ret;
    }
    return -1;
  }

  g_devdesc = (struct DeviceDescriptor *) ptr;
  usb_memcpy (g_devdesc, g_started_drivers->descriptor, sizeof (struct DeviceDescriptor));
  ptr = &ptr[sizeof (struct DeviceDescriptor)];

  g_devdesc->bMaxPacketSize0 = min_packet0;
  g_devdesc->bcdUSB = min_usb;

  if (usb_version == 2) {
    struct UsbDriver *driver;
    g_devdesc_hi = (struct DeviceDescriptor *) ptr;
    usb_memcpy (g_devdesc_hi, g_started_drivers->descriptor_hi, sizeof (struct DeviceDescriptor));
    ptr += sizeof (struct DeviceDescriptor);

    for (driver = g_started_drivers; driver; driver = driver->link) {
      if (g_devdesc_hi->bcdUSB > driver->descriptor_hi->bcdUSB) {
        g_devdesc_hi->bcdUSB = driver->descriptor_hi->bcdUSB;
      }
    }
  }

  g_desccount = 0;
  g_devdesc->idVendor = vendorId;
  g_devdesc->idProduct = productId;

  if (vendor_desc.id < 0) {
    g_devdesc->iManufacturer = 0;
  } else {
    g_desccount = 1;
    g_devdesc->iManufacturer = 1;
    g_vendor_desc = (struct StringDescriptor *) ptr;
    usb_memcpy (g_vendor_desc, &vendor_desc.str, sizeof (struct StringDescriptor));
    ptr += sizeof (struct StringDescriptor);
  }

  if (product_desc.id < 0) {
    g_devdesc->iProduct = 0;
  } else {
    g_desccount += 1;
    g_devdesc->iProduct = g_desccount;
    g_product_desc = (struct StringDescriptor *) ptr;
    usb_memcpy (g_product_desc, &product_desc.str, sizeof (struct StringDescriptor));
    ptr += sizeof (struct StringDescriptor);
  }

  g_devdesc->iSerialNumber = 0;
  if (usb_version == 2) {
    g_devdesc_hi->idProduct = productId;
    g_devdesc_hi->idVendor = vendorId;
    g_devdesc_hi->iManufacturer = g_devdesc->iManufacturer;
    g_devdesc_hi->iProduct = g_devdesc->iProduct;
    g_devdesc_hi->iSerialNumber = 0;
  }

  if (strcount[8]) {
    struct UsbDriver *driver;
    struct StringDescriptor *desc;
    num_desc = &strcount[0];
    g_descriptors = desc = (struct StringDescriptor *) ptr;
    for (driver = g_started_drivers; driver; driver = driver->link) {
      if (*num_desc) {
        usb_memcpy (desc, driver->stringDescriptors, (*num_desc) * sizeof (struct StringDescriptor));
        desc += *num_desc;
      }
      num_desc++;
    }
    desc->bLength = 0;
  }

  if (g_vendor_desc) {
    g_string = &g_default_desc;
  } else {
    if (g_product_desc) {
      g_string = &g_default_desc;
    } else {
      if (g_descriptors) {
        g_string = &g_default_desc;
      } else {
        g_string = NULL;
      }
    }
  }

  sceKernelDcacheWritebackRange (g_devdesc, size);
  return 0;
}

/* Subroutine at 0x11DC */
static
struct UsbConfiguration *load_configuration (int usb_version /* sp[37] */, int strcount[9] /* sp[38] */)
{
  int total_length = 0;                    /* var2 */
  int total_endpoints = 0;                 /* sp[32] */
  int num_endpoints[8];                    /* sp[24] */
  int total_interface_descriptors = 0;     /* sp[20] */
  int num_interface_descriptors[8];        /* sp[12] */
  int num_interfaces[8];                   /* sp[0]  */
  int total_interfaces = 0;                /* sp[8]; */
  int driver_num = 0;                      /* var1; sp[45]  */
  char *data;                              /* sp[36] */
  struct UsbConfiguration *conf;           /* sp[39] */
  struct UsbConfiguration *from;           /* sp[40] */
  struct UsbDriver *driver;                /* sp[41] */
  int interf_count;                        /* sp[42] */
  int endp_count;                          /* sp[43] */
  int desc_count;                          /* sp[44] */
  struct InterfaceSettings *settings;      /* var53  */
  struct InterfaceDescriptor *id;          /* var52  */
  struct EndpointDescriptor *endp;         /* var57  */
  SceUID fpl;
  int ret, offset;

  for (driver = g_started_drivers; driver; driver = driver->link) {
    struct UsbConfiguration *conf;

    if (usb_version == 1) {
      conf = driver->configuration;
    } else {
      conf = driver->configuration_hi;
    }
    total_length += conf->configDescriptors->wTotalLength;
    num_interfaces[driver_num] = conf->configDescriptors->bNumInterfaces;

    num_interface_descriptors[driver_num] = 0;

    if (conf->configDescriptors->bNumInterfaces > 0) {
      struct InterfaceSettings *settings = conf->settings;
      int num = conf->configDescriptors->bNumInterfaces;
      while (num--) {
        num_interface_descriptors[driver_num] += settings->numDescriptors;
        settings++;
      }
    }
    num_endpoints[driver_num] = driver->numEndpoints - 1;
    total_interfaces += num_interfaces[driver_num];
    total_interface_descriptors += num_interface_descriptors[driver_num];
    total_endpoints += num_endpoints[driver_num];
    driver_num++;
  }

  fpl = sceKernelCreateFpl ("SceUsbConfig", PSP_MEMORY_PARTITION_KERNEL, 0x100,
      (total_length - USB_DT_CONFIG_SIZE * driver_num + sizeof (struct UsbConfiguration) + sizeof (struct ConfigDescriptor)) +
      total_interfaces * sizeof (struct InterfaceSettings) +
      total_interface_descriptors * (sizeof (struct InterfaceDescriptor) - USB_DT_INTERFACE_SIZE) +
      total_endpoints * (sizeof (struct EndpointDescriptor) - USB_DT_ENDPOINT_SIZE) +
      sizeof (struct InterfaceDescriptor) + sizeof (struct EndpointDescriptor), 1, NULL);
  if (usb_version == 1) g_fsconffpl = fpl;
  else g_hsconffpl = fpl;

  if (fpl < 0) {
    Kprintf ("usbbd ERROR : cannot create FPL to EP0 buffer\n");
    return NULL;
  }

  ret = sceKernelTryAllocateFpl (fpl, (void **) &data);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot allocate memory to EP0 buffer\n");
    if (sceKernelDeleteFpl (fpl) < 0)
      Kprintf ("usbbd ERROR : cannot delete FPL for config\n");
    return NULL;
  }

  conf = (struct UsbConfiguration *) data;
  offset = sizeof (struct UsbConfiguration);
  conf->configDescriptors = (struct ConfigDescriptor *) &data[offset];
  offset += sizeof (struct ConfigDescriptor);
  conf->settings = (struct InterfaceSettings *) &data[offset];
  offset += sizeof (struct InterfaceSettings) * total_interfaces;
  conf->interfaceDescriptors = (struct InterfaceDescriptor *) &data[offset];
  offset += sizeof (struct InterfaceDescriptor) * (total_interface_descriptors + 1);
  if (total_endpoints == 0) {
    conf->endpointDescriptors = NULL;
    data = &data[offset];
  } else {
    conf->endpointDescriptors = (struct EndpointDescriptor *) &data[offset];
    offset += sizeof (struct EndpointDescriptor) * (total_endpoints + 1);
    data = &data[offset];
  }

  if (usb_version == 1) {
    from = g_started_drivers->configuration;
  } else {
    from = g_started_drivers->configuration_hi;
  }

  usb_memcpy (conf->configDescriptors, from->configDescriptors, sizeof (struct ConfigDescriptor));
  conf->configDescriptors->iConfiguration = 0;
  conf->configDescriptors->bmAttributes = 0xC0; /* Self-powered */
  conf->configDescriptors->bNumInterfaces = total_interfaces;
  conf->configDescriptors->bMaxPower = 1; /* 1 mA */

  conf->configDescriptors->settings = conf->settings;
  conf->configDescriptors->wTotalLength = total_length - USB_DT_CONFIG_SIZE * driver_num + USB_DT_CONFIG_SIZE;

  driver_num = 0;
  settings = conf->settings;
  id = conf->interfaceDescriptors;
  endp = conf->endpointDescriptors;

  interf_count = 0;
  endp_count = 0;
  desc_count = g_desccount;
  for (driver = g_started_drivers; driver; driver = driver->link) {
    struct InterfaceSettings *configarg = settings;

    if (usb_version == 1) {
      from = driver->configuration;
    } else {
      from = driver->configuration_hi;
    }

    usb_memcpy (settings, from->settings, num_interfaces[driver_num] * sizeof (struct InterfaceSettings));

    usb_memcpy (id, from->interfaceDescriptors, num_interface_descriptors[driver_num] * sizeof (struct InterfaceDescriptor));

    if (num_endpoints[driver_num] != 0) {
      usb_memcpy (endp, from->endpointDescriptors, num_endpoints[driver_num] * sizeof (struct EndpointDescriptor));
    }

    if (num_interfaces[driver_num] > 0) {
      int i, j, k;

      for (i = 0; i < num_interfaces[driver_num]; i++) {
        settings->descriptors = id;

        for (j = 0; j < settings->numDescriptors; j++) {
          id->bInterfaceNumber += interf_count;

          if (id->iInterface != 0) {
            id->iInterface += desc_count;
          }

          if (id->extraLength != 0) {
            id->extra = (unsigned char *) data;
            usb_memcpy (id->extra, from->settings[i].descriptors[j].extra, id->extraLength);
            data += id->extraLength;
          }

          if (id->bNumEndpoints != 0) {
            id->endpoints = endp;
            for (k = 0; k < id->bNumEndpoints; k++) {

              endp->bEndpointAddress = (endp->bEndpointAddress & USB_ENDPOINT_DIR_MASK) +
                ((endp->bEndpointAddress & USB_ENDPOINT_ADDRESS_MASK) + endp_count);

              if (endp->extraLength != 0) {
                endp->extra = (unsigned char *) data;
                usb_memcpy (endp->extra, from->settings[i].descriptors[j].endpoints[k].extra, endp->extraLength);
                data += endp->extraLength;
              }
              endp++;
            }
          }
          id++;
        }
        settings++;
      }
    }

    if (driver->configure) {
      driver->configure (usb_version, desc_count, configarg);
    }

    endp_count += num_endpoints[driver_num];
    interf_count += num_interfaces[driver_num];
    desc_count += strcount[driver_num];
    driver_num++;
  }
  if (total_endpoints != 0)
    endp->bLength = 0;
  id->bLength = 0;

  return conf;
}

/* Subroutine at 0x17B8 */
static
int read_idstorage (int id, struct StorageStringDescriptor *vendor_desc, struct StorageStringDescriptor *product_desc)
{
  int leaf_size, ret;
  int i = 0, num_descriptors;
  int offset;
  u16 key;

  ret = sceIdStorageIsFormatted ();
  if (ret < 0) return 0;

  leaf_size = sceIdStorageGetLeafSize();

  ret = sceIdStorageLookup (PSP_USB_IDSTORAGE_KEY, 0, vendor_desc, sizeof (struct StorageStringDescriptor));
  if (ret < 0) return 0;
  offset = sizeof (struct StorageStringDescriptor);

  ret = sceIdStorageLookup (PSP_USB_IDSTORAGE_KEY, offset, &num_descriptors, sizeof (int));
  if (ret < 0) return sizeof (struct StringDescriptor);

  offset += sizeof (int);
  if (num_descriptors > 0) {
    int rem = leaf_size - offset;
    key = PSP_USB_IDSTORAGE_KEY;
    for (; i < num_descriptors; i++) {
      if (rem > sizeof (struct StorageStringDescriptor))
        rem = sizeof (struct StorageStringDescriptor);
      ret = sceIdStorageLookup (key, offset, product_desc, rem);
      offset += rem;

      if (ret < 0) return sizeof (struct StringDescriptor);

      if (rem < sizeof (struct StorageStringDescriptor)) {
        char *to = (char *) product_desc;

        if (++key > PSP_USB_IDSTORAGE_KEY + 1)
          return sizeof (struct StringDescriptor);

        ret = sceIdStorageLookup (key, 0x00000000, &to[rem], sizeof (struct StorageStringDescriptor) - rem);
        if (ret < 0) return sizeof (struct StringDescriptor);
        offset += sizeof (struct StorageStringDescriptor) - rem;
      }
      if (id == product_desc->id) break;

      rem = leaf_size - offset;
    }
  }

  return (i < num_descriptors) ? 2 * sizeof (struct StringDescriptor) : sizeof (struct StringDescriptor);
}

/* Subroutine at 0x1928 */
int make_configpacket (void)
{
  int size, ret;
  char *data;

  size = ALIGN_UP (g_conf->configDescriptors->wTotalLength, 63) + sizeof (struct DeviceQualifierDescriptor);
  if (g_conf_hi) {
    size += ALIGN_UP (g_conf_hi->configDescriptors->wTotalLength, 63) + sizeof (struct DeviceQualifierDescriptor);
  }

  g_ep0fpl = sceKernelCreateFpl ("SceUsbGetDescData", PSP_MEMORY_PARTITION_KERNEL, 0x00000100, size, 1, NULL);
  if (g_ep0fpl < 0) {
    Kprintf ("usbbd ERROR : cannot create FPL for get descriptor data\n");
    return -1;
  }

  ret = sceKernelTryAllocateFpl (g_ep0fpl, (void **) &data);
  if (ret < 0) {
    Kprintf ("usbbd ERROR : cannot allocate memory for get descriptor data\n");

    ret = sceKernelDeleteFpl (g_ep0fpl);
    if (ret < 0) {
      Kprintf ("usbbd ERROR : cannot delete FPL for get descriptor data\n");
      return ret;
    }
    return -1;
  }
  g_configpacket = data;
  data += ALIGN_UP (g_conf->configDescriptors->wTotalLength, 63);
  if (g_conf_hi) {
    g_configpacket_hi = data;
    data += ALIGN_UP (g_conf_hi->configDescriptors->wTotalLength, 63);
  }
  g_qual = (struct DeviceQualifierDescriptor *) data;
  data += sizeof (struct DeviceQualifierDescriptor);
  if (g_conf_hi) {
    g_qual_hi = (struct DeviceQualifierDescriptor *) data;
    data += sizeof (struct DeviceQualifierDescriptor);
  }
  make_device_qualifier (g_devdesc, g_qual);
  make_packet (g_conf->configDescriptors, g_configpacket);

  if (g_conf_hi) {
    make_device_qualifier (g_devdesc_hi, g_qual_hi);
    make_packet (g_conf_hi->configDescriptors, g_configpacket_hi);
  }

  sceKernelDcacheWritebackRange (g_configpacket, size);
  return 0;
}

/* Subroutine at 0x1ACC */
static
void make_device_qualifier (struct DeviceDescriptor *device, struct DeviceQualifierDescriptor *qualifier)
{
  usb_memcpy (qualifier, device, 8);
  qualifier->bLength = USB_DT_DEVQUAL_SIZE;
  qualifier->bDescriptorType = USB_DT_DEVQUAL;
  qualifier->bReserved = 0;
  qualifier->bNumConfigurations = device->bNumConfigurations;
}

/* Subroutine at 0x1B24 */
static
void make_packet (struct ConfigDescriptor *from, char *buffer)
{
  usb_memcpy (buffer, from, USB_DT_CONFIG_SIZE);
  buffer += USB_DT_CONFIG_SIZE;

  if (from->bNumInterfaces > 0) {
    struct InterfaceSettings *settings;
    struct InterfaceDescriptor *id;
    struct EndpointDescriptor *endp;
    int i, j, k;

    settings = from->settings;
    for (i = 0; i < from->bNumInterfaces; i++) {
      if (settings->numDescriptors > 0) {
        id = settings->descriptors;
        for (j = 0; j < settings->numDescriptors; j++) {
          usb_memcpy (buffer, id, USB_DT_INTERFACE_SIZE);
          buffer += USB_DT_INTERFACE_SIZE;

          if (id->extraLength != 0) {
            usb_memcpy (buffer, id->extra, id->extraLength);
            buffer += id->extraLength;
          }

          if (id->bNumEndpoints > 0) {
            endp = id->endpoints;
            for (k = 0; k < id->bNumEndpoints; k++) {
              usb_memcpy (buffer, endp, USB_DT_ENDPOINT_SIZE);
              buffer += USB_DT_ENDPOINT_SIZE;

              if (endp->extraLength != 0) {
                usb_memcpy (buffer, endp->extra, endp->extraLength);
                buffer += endp->extraLength;
              }

              endp++;
            }
          }

          id++;
        }
      }
      settings++;
    }
  }
}

/* Subroutine at 0x1C70 */
int process_request (void)
{
  int req_type;
  req_type = g_devreq.bmRequestType & 0x60;
  if (req_type == 0x20) { /* Class */
    if (process_class_request () < 0) {
      return -1;
    }
  } else if (req_type > 0x20) {
    if (req_type != 0x40) return -1; /* Vendor */
  } else { /* Standard */
    return process_standard_request (&g_devreq);
  }
  return 0;
}

/* Subroutine at 0x1CF4 */
static
int process_class_request (void)
{
  struct UsbDriver *driver;
  int recipient;
  int ret = -1;

  recipient = g_devreq.bmRequestType & 0x1F;
  if (recipient == 1) { /* Interface */
    if (g_started_drivers) {
      int interf_num = (g_devreq.wIndex & 0xFF);
      for (driver = g_started_drivers; driver; driver = driver->link) {
        if (driver->interface->interfaceNumber + driver->interface->numInterfaces > interf_num &&
            driver->interface->interfaceNumber <= interf_num) {
          if (driver->processRequest) {
            ret = driver->processRequest (recipient, interf_num - driver->interface->interfaceNumber, &g_devreq);
          }
        } else {
          if (driver->processRequest) {
            driver->processRequest (recipient, -1, &g_devreq);
          }
        }
      }
    }
  } else if (recipient != 2) { /* Not Endpoint */
    for (driver = g_started_drivers; driver; driver = driver->link) {
      if (driver->processRequest) {
        int temp = driver->processRequest (recipient, 0, &g_devreq);
        if (temp >= 0) ret = temp;
      }
    }
  } else {
    int endp_num = (g_devreq.wIndex & 0xFF);
    if (g_started_drivers) {
      for (driver = g_started_drivers; driver; driver = driver->link) {
        int i, found = 0;
        for (i = 0; i < driver->numEndpoints; i++) {
          struct UsbEndpoint *endp = &driver->endpoints[i];
          if (endp->endpointNumber == (endp_num & USB_ENDPOINT_ADDRESS_MASK)) {
            found = 1;
            if (driver->processRequest) {
              ret = driver->processRequest (recipient, endp->driverEndpointNumber, &g_devreq);
            }
          }
        }
        if (!found) {
          if (driver->processRequest)
            driver->processRequest (recipient, -1, &g_devreq);
        }

      }
    }
  }
  return ret;
}

/* Subroutine at 0x1F38 */
void process_request_default (void)
{
  if (g_connection_status == PSP_USB_STATUS_CONNECTION_ESTABLISHED) {
    struct UsbDriver *driver;
    int recipient = g_devreq.bmRequestType & 0x1F;
    for (driver = g_started_drivers; driver; driver = driver->link) {
      if (driver->processRequest) {
        driver->processRequest (recipient, -1, &g_devreq);
      }
    }
  }
}

/* Subroutine at 0x2168 */
static
int process_get_descriptor (void)
{
  int descriptor_type;
  int descriptor_index;
  int length;
  int ret = -1;


  descriptor_type = g_devreq.wValue >> 8;
  descriptor_index = g_devreq.wValue & 0xFF;
  length = g_devreq.wLength;

  switch (descriptor_type) {
  case USB_DT_DEVICE :
    if (g_usb_version == 2) {
      g_request.data = g_devdesc_hi;
    } else {
      g_request.data = g_devdesc;
    }
    if (length > USB_DT_DEVICE_SIZE) length = USB_DT_DEVICE_SIZE ;
    g_request.size = length;
    break;

  case USB_DT_OTHERSPEED:
    if (g_conf_hi) {
      struct ConfigDescriptor *desc;
      if (g_usb_version == 2) {
        desc = (struct ConfigDescriptor *) g_configpacket;
      } else {
        desc = (struct ConfigDescriptor *) g_configpacket_hi;
      }
      g_request.data = desc;
      desc->bDescriptorType = USB_DT_OTHERSPEED;
      if (descriptor_index < 2) {
        if (length > desc->wTotalLength)
          length = desc->wTotalLength;
        sceKernelDcacheWritebackRange (g_request.data, 64);
        g_request.size = length;
        break;
      }
    }
    process_request_default ();
    return ret;

  case USB_DT_CONFIG:
    {
      struct ConfigDescriptor *desc;
      if (g_usb_version == 2) {
        desc = (struct ConfigDescriptor *) g_configpacket_hi;
      } else {
        desc = (struct ConfigDescriptor *) g_configpacket;
      }
      g_request.data = desc;
      desc->bDescriptorType = USB_DT_CONFIG;
      if (descriptor_index < 2) {
        if (length > desc->wTotalLength)
          length = desc->wTotalLength;
        sceKernelDcacheWritebackRange (g_request.data, 64);
        g_request.size = length;
        break;
      } else {
        process_request_default ();
        return ret;
      }
    }
  case USB_DT_STRING:
    if (g_string) {
      struct StringDescriptor *str = g_string;

      if (descriptor_index != 0) {
        if (descriptor_index > g_desccount) {
          int i, index = descriptor_index - g_desccount -1;
          str = g_descriptors;
          for (i = 0; i < index; i++) {
            if (!str->bLength) break;
            str++;
          }
        } else {
          str = g_vendor_desc;
          if (!str || descriptor_index == 2) {
            str = g_product_desc;
          }
        }

      }

      if (str->bLength) {
        g_request.data = str;
        if (length > str->bLength)
          length = str->bLength;
        g_request.size = length;
        break;
      }
    }
    process_request_default ();
    return ret;

  case USB_DT_DEVQUAL:
    if (g_devdesc_hi) {
      struct DeviceQualifierDescriptor *qual;
      if (g_usb_version == 2)
        qual = g_qual;
      else
        qual = g_qual_hi;
      g_request.data = qual;
      if (length > USB_DT_DEVQUAL_SIZE)
        length = USB_DT_DEVQUAL_SIZE;
      g_request.size = length;
      break;
    }
    process_request_default ();
    return ret;

  default:
    return process_class_request ();
  }

  ret = sceUsbbdReqSend (&g_request);
  if (ret < 0)
    Kprintf ("usbbd ERROR : cannot issue send request 0x%08x\n", ret);

  process_request_default ();
  return ret;
}

/* Subroutine at 0x356C */
static
int process_standard_request (struct DeviceRequest *req)
{
  switch (req->bRequest) {
  case 6: /* GET_DESCRIPTOR */
    return process_get_descriptor ();
  case 0: /* GET_STATUS */
  case 1: /* CLEAR_FEATURE */
  case 3: /* SET_FEATURE */
  case 5: /* SET_ADDRESS */
  case 8: /* GET_CONFIGURATION */
  case 9: /* SET_CONFIGURATION */
  case 10: /* GET_INTERFACE */
  case 11: /* SET_INTERFACE */
  case 12: /* SYNCH_FRAME */
    Kprintf ("usbbd ERROR : standard request 0x%x is handled by hardware only\n", req->bRequest);
  default:
    process_request_default ();
    return -1;
  }
  return 0;
}

/* Subroutine at 0x1FC4 */
int change_setting (int interface_num, int alternate_setting, struct ConfigDescriptor *conf)
{
  struct InterfaceSettings *settings;
  struct InterfaceDescriptor *id;
  struct EndpointDescriptor *endp;
  int i, j, k;

  if (conf->bNumInterfaces == 0) return -1;
  for (i = 0; i < conf->bNumInterfaces; i++) {
    settings = &conf->settings[i];

    if (settings->descriptors->bInterfaceNumber == interface_num) {
      if (alternate_setting < 0) {
        alternate_setting = 0;
      } else {
        if (settings->numDescriptors > 0) {
          for (j = 0; j < settings->numDescriptors; j++) {
            id = &settings->descriptors[j];
            if (id->bAlternateSetting == settings->alternateSetting) {
              if (id->bNumEndpoints != 0) {
                for (k = 0; k < id->bNumEndpoints; k++) {
                  int endpnum;
                  endp = &id->endpoints[k];

                  endpnum = endp->bEndpointAddress & USB_ENDPOINT_ADDRESS_MASK;
                  clear_endpoint_status (endpnum);
                  usb_cancel_all (endpnum);
                  usb_clear_fifo (endpnum);
                }
              }
              break;
            }
          }
        }
      }

      if (settings->numDescriptors > 0) {
        for (j = 0; j < settings->numDescriptors; j++) {
          id = &settings->descriptors[j];
          if (id->bAlternateSetting == alternate_setting) {
            settings->alternateSetting = alternate_setting;
            if (id->bNumEndpoints != 0) {
              for (k = 0; k < id->bNumEndpoints; k++) {
                endp = &id->endpoints[k];
                if (configure_endpoint (endp) < 0)
                  return -1;
              }
            }
          }
        }
      }

      return 0;
    }
  }
  return -1;
}

/* Subroutine at 0x35DC */
static
int configure_endpoint (struct EndpointDescriptor *endp)
{
  int endpnum;
  struct usb_queue *queue;
  struct usb_rxfifo *fifo;


  endpnum = endp->bEndpointAddress & USB_ENDPOINT_ADDRESS_MASK;
  queue = &g_queues[endpnum];
  fifo = &g_rxfifos[endpnum];

  usb_cancel_all (endpnum);
  queue->endpoint_type = endp->bmAttributes & USB_ENDPOINT_TYPE_MASK;
  queue->endpoint_dir = endp->bEndpointAddress & USB_ENDPOINT_DIR_MASK;
  queue->packet_size = endp->wMaxPacketSize;
  queue->first = queue->last = NULL;

  if (fifo->elements) {
    fifo->elements->ptr = fifo->buffer;
  }

  if (queue->endpoint_dir == USB_ENDPOINT_DIR_MASK) {
    int size;
    if (queue->endpoint_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
      size = queue->packet_size * 2;
    } else {
      size = queue->packet_size;
    }
    g_endpoints[endpnum].packet_size = size;
  } else {
    g_endpoints[endpnum].packet_size = queue->packet_size;
  }

  if (queue->endpoint_dir == USB_ENDPOINT_DIR_MASK) {
    if (queue->endpoint_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
      g_endpoints[endpnum].interval = 1 << (endp->bInterval - 1);
    }
    _sw (queue->endpoint_type << 4 | 0x102, USB_PORT_0000 + 32 * endpnum);
    g_endpoints[endpnum].status |= PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
    _sw (g_endpoints[endpnum].packet_size >> 2, USB_PORT_ENDPOINT_PACKET_SIZEW_BASE + 32 * endpnum);
    _sw (queue->packet_size, USB_PORT_PACKET_SIZE_BASE + 32 * endpnum);
    _sw (_lw (USB_PORT_0004 + 32 * endpnum), USB_PORT_0004 + 32 * endpnum);
  } else {
    _sw (queue->endpoint_type << 4 | 0x80, USB_PORT_0200 + 32 * endpnum);
    g_endpoints[endpnum].status &= ~PSP_USB_ENDPOINT_STATUS_UNKNOWN2;
    _sw (queue->packet_size, USB_PORT_020C + 32 * endpnum);
    _sw (_lw (USB_PORT_0204 + 32 * endpnum), USB_PORT_0204 + 32 * endpnum);
    SYNC ();
    _sw (_lw (USB_PORT_0418) & ~(1 << (endpnum + 16)), USB_PORT_0418);
  }

  return 0;
}

/* Subroutine at 0x5A1C */
static
void clear_endpoint_status (int endpoint)
{
  g_endpoints[endpoint].queue->endpoint_dir = 255;
  g_endpoints[endpoint].status = 0;
  g_endpoints[endpoint].unk4 = 0;
}





