/*
 * Qemu UDC Framework.
 *
 * Copyright (C) 2016 Subbaraya Sundeep <sundeep.lkml@gmail.com>.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "sysemu/sysemu.h"
#include <linux/types.h>
#include <linux/usb/ch9.h>
#include "udc-core.h"
#include "gadgetfs.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <sys/eventfd.h>
#include <aio.h>

//#define fprintf 
//#define printf 

#define GADGETFS_INOUT		0xE

static struct UDC the_udc;

//static int verbose = 2;
pthread_t control_t;

static inline int min(unsigned a, unsigned b)
{
	return (a < b) ? a : b;
}

/* -------------------------------------------------- */

#define	STRINGID_MFGR		1
#define	STRINGID_PRODUCT	2
#define	STRINGID_SERIAL		3
#define	STRINGID_CONFIG		4
#define	STRINGID_INTERFACE	5

#define USB_DT_DFU_FUNCTION	0x21

#define DFU_BIT_DETACH					(0x1 << 3)
#define DFU_BIT_MANIFESTATION_TOLERANT  (0x1 << 2)
#define DFU_BIT_CAN_UPLOAD              (0x1 << 1)
#define DFU_BIT_CAN_DNLOAD              0x1

struct usb_dfu_func_descriptor {
	uint8_t	bLength;
	uint8_t	bDescriptorType;
	uint8_t	bmAttributes;
	uint16_t	wDetachTimeOut;
	uint16_t	wTransferSize;
	uint16_t	bcdDFUVersion;
} __attribute__ ((packed));

struct string_desc {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint16_t wLANGID[1];
};

struct dfu_config {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor dfu_intf;
	struct usb_dfu_func_descriptor dfu_func;
} __attribute__ ((packed));

static struct usb_device_descriptor
device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		__constant_cpu_to_le16 (0x0200),
	.bDeviceClass =		0,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	.bMaxPacketSize0 =      64, 
	.idVendor =		__constant_cpu_to_le16 (0x03fd),
	.idProduct =		__constant_cpu_to_le16 (0x0100),
	.bcdDevice =		__constant_cpu_to_le16 (0x0100),
	.iManufacturer =	STRINGID_MFGR,
	.iProduct =		STRINGID_PRODUCT,
	.iSerialNumber =	STRINGID_SERIAL,
	.bNumConfigurations =	1,
};

#define	MAX_USB_POWER		1

#define	CONFIG_VALUE		1

static struct dfu_config
the_config = {
	.config = {
		.bLength		= sizeof the_config.config,
		.bDescriptorType	= USB_DT_CONFIG,
		.wTotalLength		= sizeof the_config,	
		.bNumInterfaces		= 1,
		.bConfigurationValue	= CONFIG_VALUE,
		.iConfiguration		= STRINGID_CONFIG,
		.bmAttributes		= USB_CONFIG_ATT_ONE
						| USB_CONFIG_ATT_SELFPOWER,
		.bMaxPower		= (MAX_USB_POWER + 1) / 2,
	},

	.dfu_intf = {
		.bLength                = sizeof the_config.dfu_intf,
		.bDescriptorType        = USB_DT_INTERFACE,
		.bInterfaceClass        = 0xFE,
		.bInterfaceSubClass     = 0x01,
		.bInterfaceProtocol     = 0x02,
		.iInterface             = STRINGID_INTERFACE,
	},

	.dfu_func = {
		.bLength		= sizeof the_config.dfu_func,
		.bDescriptorType	= USB_DT_DFU_FUNCTION,
		.bmAttributes		= DFU_BIT_DETACH
					    | DFU_BIT_MANIFESTATION_TOLERANT,
		.wDetachTimeOut		= 0,
		.wTransferSize		= 0x1000,
		.bcdDFUVersion		= 0x0110,
	},
};

static const struct usb_config_descriptor
config = {
	.bLength =		sizeof config,
	.bDescriptorType =	USB_DT_CONFIG,

	/* must compute wTotalLength ... */
	.bNumInterfaces =	1,
	.bConfigurationValue =	CONFIG_VALUE,
	.iConfiguration =	STRINGID_CONFIG,
	.bmAttributes =		USB_CONFIG_ATT_ONE
					| USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower =		(MAX_USB_POWER + 1) / 2,
};

static struct usb_interface_descriptor
dfu_intf = {
	.bLength		= sizeof dfu_intf,
	.bDescriptorType	= USB_DT_INTERFACE,

	.bInterfaceClass	= 0xFE,
	.bInterfaceSubClass	= 0x01,
	.bInterfaceProtocol	= 0x02,
	.iInterface		= STRINGID_INTERFACE,
};

static struct usb_dfu_func_descriptor
dfu_func = {
	.bLength		= sizeof dfu_func,
	.bDescriptorType	= USB_DT_DFU_FUNCTION,
	.bmAttributes		= DFU_BIT_DETACH
					| DFU_BIT_MANIFESTATION_TOLERANT,
	.wDetachTimeOut		= 0,
	.wTransferSize		= 0x1000,
	.bcdDFUVersion		= 0x0110,
};

static struct usb_endpoint_descriptor
ep_desc = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
 
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= __constant_cpu_to_le16 (512),
};

/* ---------------------------------------------------------- */

static int	HIGHSPEED = 1;
static const char	*DEVNAME = "/dev/gadget/dummy_udc";

static char *
build_config (char *cp)
{
	struct usb_config_descriptor *c;

	c = (struct usb_config_descriptor *) cp;

	memcpy (cp, &config, config.bLength);
	cp += config.bLength;
	memcpy (cp, &dfu_intf, dfu_intf.bLength);
	cp += dfu_intf.bLength;
	memcpy (cp, &dfu_func, dfu_func.bLength);
	cp += dfu_func.bLength;

	c->wTotalLength = __cpu_to_le16 (cp - (char *) c);
	return cp;
}

static int init_device (void)
{
	char	buf [1024], *cp = &buf [0];
	int		status;

	the_udc.ep0fd = open(DEVNAME, O_RDWR);
	if (the_udc.ep0fd < 0) {
		perror (DEVNAME);
		return -errno;
	}

	*(__u32 *)cp = 0;	/* tag for this format */
	cp += 4;

	/* write full then high speed configs */
	cp = build_config (cp);
	if (HIGHSPEED)
		cp = build_config (cp);

	/* and device descriptor at the end */
	memcpy (cp, &device_desc, sizeof device_desc);
	cp += sizeof device_desc;

	status = write (the_udc.ep0fd, &buf [0], cp - &buf [0]);
	if (status < 0) {
		perror ("write dev descriptors");
		close (the_udc.ep0fd);
		return status;
	} else if (status != (cp - buf)) {
		fprintf (stderr, "dev init, wrote %d expected %ld\n",
				status, cp - buf);
		close (the_udc.ep0fd);
		return -EIO;
	}

	return the_udc.ep0fd;
}

/* ------------------------------------------------------------------------ */

#define EVT_RESET       1
#define EVT_CONNECT     2
#define EVT_SETUP       3
#define EVT_IN          4
#define EVT_OUT         5
#define EVT_SENT        6
#define EVT_RECEIVED	7

typedef uint64_t	evt_t;
/*
static const char *udc_event (enum usb_gadgetfs_event_type t)
{
	switch (t) {
	case GADGETFS_NOP:			return "udc-NOP";
	case GADGETFS_CONNECT:		return "udc-CONNECT";
	case GADGETFS_DISCONNECT:	return "udc-DISCONNECT";
	case GADGETFS_SETUP:		return "udc-SETUP";
	case GADGETFS_IN:			return "udc-IN";
	case GADGETFS_OUT:			return "udc-OUT";
	case GADGETFS_SUSPEND:		return "udc-SUSPEND";
	default:					return "udc-UNKNOWN";
	}
}
*/
/*
static const char *speed (enum usb_device_speed s)
{
	switch (s) {
	case USB_SPEED_LOW:		return "low speed";
	case USB_SPEED_FULL:	return "full speed";
	case USB_SPEED_HIGH:	return "high speed";
	default:				return "UNKNOWN speed";
	}
}
*/
int udc_send_status(int ep, int dir)
{
	return 0;
}

struct EP *find_ep(uint8_t usbnum, uint8_t usbdir);
struct EP *find_ep(uint8_t usbnum, uint8_t usbdir)
{
	struct EP *ep;
	int i;

	for (i = 0; i < 16; i++) {
		ep = &the_udc.eps[i];
		if ((ep->epnumber == usbnum) && (ep->direction == usbdir))
			return ep;
	}
	return NULL;
}

int udc_ep_send(int ep, void *data, uint32_t size)
{
	int status;
	int fd = 0;
	struct EP *endp;
//	unsigned char *mybuf = data;
//	int i;

	if (!data)
		return -EINVAL;

	if (ep == 0) {
		fd = the_udc.ep0fd;
		if (the_udc.three_stage) {
			status = write(fd, data, size);	
			if (status < 0) {
				if (errno == EIDRM)
					fprintf(stderr, "dev_desc timeout\n");
				else
					perror("write dev desc data");
			} else if (status != size) {
				fprintf(stderr, "short dev_desc write, %d\n",
								status);
			}
			if (status >= 0) {
				the_udc.state = DATA;
				the_udc.xfer_len = (uint32_t)status;
			}
		} else {
			status = read(fd, &status, 0);
			if (status)
				fprintf(stderr, "status phase read error\n");
			the_udc.state = STATUS;
			the_udc.xfer_len = 0;
		}
	} else {
		endp = find_ep((uint8_t)ep, 1);
//		printf("ep_send name:%s and fd:%d\n", endp->name, endp->fd);
		status = write(endp->fd, data, size);
		if (status < 0)
			printf("errno:%d, %s\n", errno, strerror(errno));
		the_udc.ops->xfer_done(the_udc.opaque, ep, TO_HOST,
				status);
	}

	return 0;
}

int udc_ep_receive(int ep, void *data, uint32_t size)
{
	struct EP *endp;
	int len;
	uint8_t *buffer = data;

#ifdef AIO
	int err, ret;
	struct aiocb aiocb;

	memset(&aiocb, 0, sizeof(struct aiocb));
#endif

	if (ep == 0) {
		the_udc.state = STATUS;
		the_udc.xfer_len = 0;
	} else {
		endp = find_ep((uint8_t)ep, 0);
	//	printf("ep_receive name:%s and fd:%d\n", endp->name, endp->fd);

#ifdef AIO
		aiocb.aio_fildes = endp->fd;
		aiocb.aio_buf = &buffer;
		aiocb.aio_nbytes = size;

		if (aio_read(&aiocb) == -1)
			printf("Error at aio_read():%s\n", strerror(errno));

		while ((err = aio_error (&aiocb)) == EINPROGRESS);

  		err = aio_error(&aiocb);
  		ret = aio_return(&aiocb);

  		if (err != 0)
    		printf("Error at aio_error() : %s\n", strerror (err));
	
  		if (ret != size)
    		printf("Error at aio_return()\n");

#endif
		len = read(endp->fd, buffer, size);
		if (len < 0)
			printf("errno:%d, %s\n", errno, strerror(errno));
		the_udc.ops->xfer_done(the_udc.opaque, ep, FROM_HOST,
				len);
	}

	return 0;	
}

int evtfd;
struct usb_ctrlrequest *ctrl;
int current_ep;

static void event_read(void *arg)
{
	evt_t event;

	if (sizeof(uint64_t) != read(evtfd, &event, sizeof(uint64_t))) {
        printf("read error\n");
    }

	switch(event) {
    case EVT_RESET:
//        printf("RESET\n");
		the_udc.ops->usb_reset(the_udc.opaque);
        break;
    case EVT_CONNECT:
//        printf("CONNECT\n");
		the_udc.ops->set_speed(the_udc.opaque, USB_SPEED_HIGH);
        break;
    case EVT_SETUP:
//        printf("SETUP\n");
		the_udc.ops->setup_request(the_udc.opaque, ctrl);
        break;
    case EVT_IN:
//        printf("IN\n");
		the_udc.ops->in_token(the_udc.opaque, current_ep);
        break;
    case EVT_OUT:
//        printf("OUT\n");
		the_udc.ops->out_token(the_udc.opaque, current_ep);
        break;
    case EVT_SENT:
//        printf("SENT\n");
		the_udc.ops->xfer_done(the_udc.opaque, 0, TO_HOST,
				the_udc.xfer_len);
        break;
    case EVT_RECEIVED:
//        printf("RECEIVED\n");
		the_udc.ops->xfer_done(the_udc.opaque, 0, FROM_HOST,
				the_udc.xfer_len);
        break;
    default:
        printf("unknown\n");
        break;
    }
}

#define INTERVAL	50000

static void write_evt(evt_t event)
{
 	fd_set fds;
 	struct timeval time = { .tv_sec = 0, .tv_usec = INTERVAL };
    struct timeval *tv;
	int activity;

again:
	FD_ZERO(&fds);
	FD_SET(evtfd, &fds);
	tv = &time;
	activity = select(evtfd + 1, &fds, NULL, NULL, tv);
    if (activity < 0) {
        printf("select error %d %s\n", errno, strerror(errno));
	} else if (activity == 0) {
		/* timedout i.e, event is read and given time for tcg so write now */
		if(write(evtfd, &event, sizeof(evt_t)) < 0)
			printf("error event write\n");
	}
	else if (FD_ISSET(evtfd, &fds)) { /* readable - event is not read yet */
		goto again;	
	}
}

static void *ep0_thread (void *arg)
{
    int ret;
	struct usb_gadgetfs_event event;


	for (;;) {
	ret = read(the_udc.ep0fd, &event, sizeof(event));
    if (ret < 0 && errno != EAGAIN)
        fprintf(stderr, "%s: event error: %i\n", __func__, errno);
    if (ret < (int) sizeof(event))
        fprintf(stderr, "%s: event error2: %i\n", __func__, errno);

//	printf("gadgetfs_event:%s\n", udc_event(event.type));

	switch (event.type) {
	case GADGETFS_NOP:
	case GADGETFS_SUSPEND:
		break;

	case GADGETFS_CONNECT:
//		printf("%s\n", speed(event.u.speed));
		the_udc.speed = event.u.speed;
		write_evt(EVT_RESET);
		write_evt(EVT_CONNECT);
		break;

	case GADGETFS_SETUP:

		the_udc.state = SETUP;
		ctrl = &event.u.setup;

		if (ctrl->wLength)
			the_udc.three_stage = true;
		else
			the_udc.three_stage = false;

		current_ep = 0;
		write_evt(EVT_SETUP);

		/* DATA phase */
		if (the_udc.three_stage) {
			if ((ctrl->bRequestType & USB_DIR_IN))
				write_evt(EVT_IN);
			else
				write_evt(EVT_OUT);

			while (the_udc.state != DATA);

			if ((ctrl->bRequestType & USB_DIR_IN))
				write_evt(EVT_SENT);
			else
				write_evt(EVT_RECEIVED);
		}

		/* STATUS phase */
		if ((ctrl->bRequestType & USB_DIR_IN))
			write_evt(EVT_OUT);
		else
			write_evt(EVT_IN);

		while (the_udc.state != STATUS);
	
		if ((ctrl->bRequestType & USB_DIR_IN))
			write_evt(EVT_RECEIVED);
		else
			write_evt(EVT_SENT);

		break;

	case GADGETFS_IN:
		ctrl = &event.u.setup;
		//	printf("%s ep:%d\n",
		//		(ctrl->bRequestType & USB_DIR_IN) ? "IN" : "OUT",
		//		(int)(ctrl->bRequestType & 0xf));
		current_ep = (int)(ctrl->bRequestType & 0xf);
		if (ctrl->bRequestType & USB_DIR_IN) {
			write_evt(EVT_IN);
		} else
			write_evt(EVT_OUT);
		break;

	case GADGETFS_DISCONNECT:
//		the_udc.state = NOT_ATTACHED;
		the_udc.speed = USB_SPEED_UNKNOWN;
		break;

	default:
		printf("* unhandled event %d\n", event.type);
		break;
	}
	}

	pthread_exit(NULL);
}

int udc_disconnect (void)
{
	return 0;
}

int udc_connect (void)
{
	evtfd = eventfd(0,0);
	qemu_set_fd_handler(evtfd, event_read, NULL, &the_udc);

	init_device();

	if (the_udc.ep0fd < 0) {
		printf("init_device error\n");
		return -EINVAL;
	}

	if ( -1 == pthread_create(&control_t, NULL, ep0_thread, NULL) )
	{
		printf("pthread_create\n");
		return -1;
	}
	return 0;
}

void register_udc(void *data, GadgetOps *ops)
{
	the_udc.opaque = data;
	the_udc.ops = ops;
	the_udc.state = NOT_ATTACHED;
}

int udc_ep_enable(uint8_t usbnum, uint8_t usbdir, uint8_t type,
					uint32_t maxpktsize)
{
	int fd, status;
	char buf[128];
	char name[20];
	struct EP *ep;
	int i;

	if (usbnum == 0)
		return 0;

	ep_desc.bEndpointAddress = (usbdir << 7) | usbnum;
	ep_desc.bmAttributes = type,
	ep_desc.wMaxPacketSize = cpu_to_le16(maxpktsize),

	/* open and initialize with endpoint descriptor(s) */
	sprintf(name, "/dev/gadget/ep%d", (int)usbnum);
//	printf("epname:%s\n", name);

	fd = open (name, O_RDWR);
	if (fd < 0) {
		status = -errno;
		fprintf (stderr, "open %s error %d (%s)\n",
			name, errno, strerror (errno));
		return status;
	}
 
	/* one (fs or ls) or two (fs + hs) sets of config descriptors */
	*(__u32 *)buf = 1;	/* tag for this format */
	memcpy (buf + 4, &ep_desc, USB_DT_ENDPOINT_SIZE);
	if (HIGHSPEED)
		memcpy (buf + 4 + USB_DT_ENDPOINT_SIZE,
			&ep_desc, USB_DT_ENDPOINT_SIZE);
	status = write (fd, buf, 4 + USB_DT_ENDPOINT_SIZE
			+ (HIGHSPEED ? USB_DT_ENDPOINT_SIZE : 0));
	if (status < 0) {
		status = -errno;
		fprintf (stderr, "config %s error %d (%s)\n",
			name, errno, strerror (errno));
		close (fd);
		return status;
	}

	for (i = 0; i < 16; i++) {
		ep = &the_udc.eps[i];
		if (!ep->enabled)
			goto found;
	}

	printf("No free ep\n");
	return -1;
found:
	ep->epnumber = usbnum;
	ep->direction = usbdir;
	ep->type = type;
	ep->fd = fd;
	strcpy(ep->name, name);
//	printf("name:%s and fd:%d\n", ep->name, fd);
	ep->enabled = true;

	return 0;
}

int udc_ep_disable(uint8_t usbnum, uint8_t usbdir)
{
	return 0;
}
