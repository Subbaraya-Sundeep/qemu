/*
 * QEMU model of the Xilinx AXI USB device Controller
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
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "qemu/log.h"
#include "udc-core.h"
#include "qemu/thread.h"

//#define XILINX_USB_ERR_DEBUG
#ifdef XILINX_USB_ERR_DEBUG
#define DB_PRINT(...) do { \
    fprintf(stderr,  ": %s: ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0);
#else
    #define DB_PRINT(...)
#endif

#define R_EP0_CONFIG_STATUS		(0x0000 / 4)  /* EP Config and Status */
#define R_EP0_BUF0_COUNT		(0x0008 / 4)  /* Buffer 0 count */
#define R_EP0_BUF1_COUNT		(0x000C / 4)  /* Buffer 1 count */
#define R_SETUP_PKT_ADDR1		(0x0080 / 4) /* Setup Packet Address */
#define R_SETUP_PKT_ADDR2		(0x0084 / 4) /* Setup Packet Address */
#define R_EP0_RAM				(0x0088 / 4) /* RAM for EP0 buffer */
#define R_ADDRESS				(0x0100 / 4)	
#define R_CONTROL				(0x0104 / 4)  /* Control Register */
#define R_ISR					(0x0108 / 4)  /* Status Register */
#define R_IER					(0x0110 / 4) /* Interrupt Enable Register */
#define R_FRAMENUM				(0x010C / 4) /* Frame Number Register */
#define R_BUFFREADY				(0x0114 / 4)  /* Buffer Ready Register */
#define R_TESTMODE				(0x0118 / 4)  /* Test Mode Register */
#define R_DMA_RESET				(0x0200 / 4)  /* DMA Soft Reset Register */
#define R_DMA_CONTROL			(0x0204 / 4)  /* DMA Control Register */
#define R_DMA_DSAR_ADDR			(0x0208 / 4)  /* DMA source Address Reg */
#define R_DMA_DDAR_ADDR			(0x020C / 4) /* DMA destination Addr Reg */
#define R_DMA_LENGTH			(0x0210 / 4) /* DMA Length Register */
#define R_DMA_STATUS			(0x0214 / 4) /* DMA Status Register */
#define R_EPS_RAM				(0x4000 / 4) /* DMA Status Register */
#define R_MAX					(0x5800 / 4) /* DMA Status Register */

#define IER_MASTER_EN     		0x80000000 /* USB Master Enable Mask */
#define ISR_RESET_MASK      	0x00800000 /* USB Reset Mask */
#define ISR_HIGH_SPEED_MASK 	0x00010000 /* USB Speed Mask */
#define ISR_SETUP_PACKET_MASK   0x00040000 /* Setup packet received */
#define ISR_EP0_BUFF_COMP_MASK 	0x00000001 /* EP 0 Buff 1 Processed */
#define ISR_BUFF_RDY_MASK  		0x00100000 /* FIFO Buff Ready Mask */
#define ISR_BUFF_FREE_MASK 		0x00080000 /* FIFO Buff Free Mask */

#define ECS_EP_VALID			(1 << 31)
#define ECS_EP_STALL			(1 << 30)
#define ECS_EP_DIR				(1 << 29)
#define ECS_EP_ISO				(1 << 28)
#define ECS_EP_MAXPACKET		0x03FF8000
#define ECS_EP_BASE				0x00001FFF

#define BRR_BUF2_SHIFT			8

#define TYPE_XILINX_USB		"xlnx,udc"
#define XILINX_USB(obj)		OBJECT_CHECK(XilinxUSB, (obj), TYPE_XILINX_USB)

typedef struct XilinxUSB {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    uint8_t has_dma;
	uint8_t curr_buf[8];
	bool buf_owner[32];
    uint32_t regs[R_MAX];
} XilinxUSB;

static void xlx_udc_update_irq(XilinxUSB *xudc)
{
	uint32_t pending;

	if ((xudc->regs[R_IER] & IER_MASTER_EN) == 0)
		return;

	pending = xudc->regs[R_ISR] & xudc->regs[R_IER];
	pending = !!pending;

	qemu_set_irq(xudc->irq, pending);
}

static void xlx_udc_reset(DeviceState *d)
{
    XilinxUSB *xudc = XILINX_USB(d);

    memset(xudc->regs, 0, sizeof xudc->regs);

    xlx_udc_update_irq(xudc);
}

static uint64_t
udc_read(void *opaque, hwaddr addr, unsigned int size)
{
    XilinxUSB *xudc = opaque;
    uint32_t r = 0;
	unsigned char *val = (unsigned char *)&r;
	unsigned char *src;
	unsigned int rem;

	rem = addr % 4;
    addr >>= 2;

	if (size != 4) {
		src = (unsigned char *)&xudc->regs[addr] + rem;
		memcpy(val, src, size);
		goto done;
	}

	switch(addr) {
	case R_ISR:
		r = xudc->regs[R_ISR];
		xudc->regs[R_ISR] = 0; 
		break;
	default:
		if (addr < ARRAY_SIZE(xudc->regs)) {
			r = xudc->regs[addr];
		}
		break;
	}

done:
	xlx_udc_update_irq(xudc);

	return r;
}

static void xudc_ep_enable(XilinxUSB *xudc, hwaddr addr)
{
	uint8_t epnum;
	uint8_t dir;
	uint8_t type;
	uint32_t maxpacket;

	epnum = addr / 4;
	dir = !!(xudc->regs[addr] & ECS_EP_DIR);
	type = !!(xudc->regs[addr] & ECS_EP_ISO);
	maxpacket = (xudc->regs[addr] & ECS_EP_MAXPACKET) >> 15;

	if (type)
			type = USB_ENDPOINT_XFER_ISOC;

	if (!type && epnum) {
		if (maxpacket == 64)
			type = USB_ENDPOINT_XFER_INT;
		else	
			type = USB_ENDPOINT_XFER_BULK;
	}

	udc_ep_enable(epnum, dir, type, maxpacket);
}

static void xudc_ep_disable(XilinxUSB *xudc, hwaddr addr)
{
	uint8_t epnum;
	uint8_t dir;

	epnum = addr / 4;
	dir = !!(xudc->regs[addr] & ECS_EP_DIR);

	udc_ep_disable(epnum, dir);
}

static void update_buffers(XilinxUSB *xudc, uint32_t value)
{
	int i;

	for (i = 0; value; value >>= 1, i++) {
		if (value & 1)
			xudc->buf_owner[i] = true;
	}
}

static void
udc_write(void *opaque, hwaddr addr,
            uint64_t val64, unsigned int size)
{
    XilinxUSB *xudc = opaque;
    uint32_t value = val64;
	unsigned char *val = (unsigned char *)&value;
	unsigned char *dest;
	unsigned int rem;

    DB_PRINT("addr=" TARGET_FMT_plx " = %x size=%d\n", addr, value, size);
 
	rem = addr % 4;
	addr >>= 2;

	if (size != 4) {
		dest = (unsigned char *)&xudc->regs[addr] + rem;
		memcpy(dest, val, size);
		goto done;
	}

	switch(addr) {
	case R_CONTROL:
		xudc->regs[R_CONTROL] = value;
		if (value & IER_MASTER_EN) {
			udc_connect();
		} else
			udc_disconnect();
		break;
	case R_BUFFREADY:
		xudc->regs[R_BUFFREADY] = value;
		update_buffers(xudc, value);
		break;
	default:
		if (addr < ARRAY_SIZE(xudc->regs)) {
			xudc->regs[addr] = value;
		}
		if (addr < R_SETUP_PKT_ADDR1) {
			rem = addr % 4;
			if (rem == 0) {
				if (value & ECS_EP_VALID)
					xudc_ep_enable(xudc, addr);
				else
					xudc_ep_disable(xudc, addr);
			}
		}
		break;
	}
done:
   xlx_udc_update_irq(xudc);
}

static void xudc_usb_reset(void *udc)
{
	XilinxUSB *xudc = udc;

	xudc->regs[R_ISR] |= ISR_RESET_MASK;
    xlx_udc_update_irq(xudc);
}

static void xudc_set_speed(void *udc, enum usb_device_speed speed)
{
	XilinxUSB *xudc = udc;

	xudc->regs[R_ISR] |= ISR_HIGH_SPEED_MASK;
    xlx_udc_update_irq(xudc);
}

static int xudc_setup(void *udc, struct usb_ctrlrequest *req)
{
	XilinxUSB *xudc = udc;

	memcpy(&xudc->regs[R_SETUP_PKT_ADDR1], req, sizeof(*req));
	xudc->regs[R_ISR] |= ISR_SETUP_PACKET_MASK | ISR_EP0_BUFF_COMP_MASK;
    xlx_udc_update_irq(xudc);

	return 0;
}

#define is_buf0_ready(ep)	(xudc->buf_owner[(ep)])
#define is_buf1_ready(ep)	(xudc->buf_owner[(ep) + 8])

#define clear_buf0(ep)								\
do {												\
	xudc->regs[R_BUFFREADY] &= ~(1 << (ep));		\
	xudc->buf_owner[(ep)] = false;					\
} while(0)

#define clear_buf1(ep)								\
do {												\
	xudc->regs[R_BUFFREADY] &= ~(1 << ((ep) + 8));	\
	xudc->buf_owner[(ep) + 8] = false;				\
} while(0)

static bool is_buffer_ready(XilinxUSB *xudc, int epnumber)
{
	if (epnumber == 0) {
		return ((xudc->regs[R_BUFFREADY] & 1) == 1);
	} else {
		return (is_buf0_ready(epnumber) || is_buf1_ready(epnumber));
	}
	return false;
}

static void drain_buffer(XilinxUSB *xudc, int epnumber)
{
	uint32_t length = 0;
	uint32_t *buffer = NULL;
	uint32_t index;
	uint32_t maxpkt;

	if (epnumber == 0) {
		length = xudc->regs[R_EP0_BUF0_COUNT];
		buffer = &xudc->regs[R_EP0_RAM];
	} else {
		if (is_buf0_ready(epnumber) && !xudc->curr_buf[epnumber]) {
			index = xudc->regs[R_EP0_CONFIG_STATUS + (epnumber * 4)] &
						ECS_EP_BASE;
			buffer = &xudc->regs[index];
			length = xudc->regs[R_EP0_BUF0_COUNT + (epnumber * 4)];
		} else if (is_buf1_ready(epnumber) && xudc->curr_buf[epnumber]) {
			maxpkt = (xudc->regs[R_EP0_CONFIG_STATUS + (epnumber * 4)]
						& ECS_EP_MAXPACKET) >> 15;
			index = xudc->regs[R_EP0_CONFIG_STATUS + (epnumber * 4)] &
						ECS_EP_BASE;
			index = index + (maxpkt / 4);
			buffer = &xudc->regs[index];
			length = xudc->regs[R_EP0_BUF1_COUNT + (epnumber * 4)];
		}
	}

	udc_ep_send(epnumber, (void *)buffer, length);
}

static int xudc_in_token(void *udc, int epnumber)
{
	XilinxUSB *xudc = udc;

	if (is_buffer_ready(xudc, epnumber))
		drain_buffer(xudc, epnumber);
	else {}
	/* if buffer not ready then NAK - do stuff if required */
	return 0;
}

static void fill_buffer(XilinxUSB *xudc, int epnumber)
{
	uint32_t length = 0;
	uint32_t *buffer = NULL;
	uint32_t index;
	uint32_t maxpkt;

	if (epnumber == 0) {
		buffer = &xudc->regs[R_EP0_RAM];
		length = xudc->regs[R_EP0_BUF0_COUNT];
	} else {
		if (is_buf0_ready(epnumber) && !xudc->curr_buf[epnumber]) {
			maxpkt = (xudc->regs[R_EP0_CONFIG_STATUS + (epnumber * 4)]
						& ECS_EP_MAXPACKET) >> 15;
			index = xudc->regs[R_EP0_CONFIG_STATUS + (epnumber * 4)] &
						ECS_EP_BASE;
			buffer = &xudc->regs[index];
			length = xudc->regs[R_EP0_BUF0_COUNT + (epnumber * 4)];
		} else if (is_buf1_ready(epnumber) & xudc->curr_buf[epnumber]) {
			maxpkt = (xudc->regs[R_EP0_CONFIG_STATUS + (epnumber * 4)]
						& ECS_EP_MAXPACKET) >> 15;
			index = xudc->regs[R_EP0_CONFIG_STATUS + (epnumber * 4)] &
						ECS_EP_BASE;
			index = index + (maxpkt / 4);
			buffer = &xudc->regs[index];
			length = xudc->regs[R_EP0_BUF1_COUNT + (epnumber * 4)];
		}
	}

	length = 512;
	udc_ep_receive(epnumber, (void *)buffer, length);
}

static int xudc_out_token(void *udc, int epnumber)
{
	XilinxUSB *xudc = udc;

	if (is_buffer_ready(xudc, epnumber))
		fill_buffer(xudc, epnumber);
	else {}

	return 0;
}

static int xudc_xfer_done(void *udc, int epnumber, int dir, uint32_t len)
{
	XilinxUSB *xudc = udc;

	if (epnumber == 0) {
		if (dir == TO_HOST) {
			xudc->regs[R_ISR] |= ISR_BUFF_FREE_MASK;
			xudc->regs[R_EP0_BUF0_COUNT] -= len;
		} else {
			xudc->regs[R_ISR] |= ISR_BUFF_RDY_MASK;
			xudc->regs[R_EP0_BUF0_COUNT] = len;
		}
		xudc->regs[R_ISR] |= ISR_EP0_BUFF_COMP_MASK;
		clear_buf0(epnumber);
	} else {
		if (is_buf0_ready(epnumber) && !xudc->curr_buf[epnumber]) {
			clear_buf0(epnumber);
			xudc->regs[R_ISR] |= (1 << epnumber);
			xudc->curr_buf[epnumber] = 1;
			if (dir == TO_HOST)
				xudc->regs[R_EP0_BUF0_COUNT + (epnumber * 4)] -= len;
			else
				xudc->regs[R_EP0_BUF0_COUNT + (epnumber * 4)] = len;
		} else if (is_buf1_ready(epnumber) && xudc->curr_buf[epnumber]) {
			clear_buf1(epnumber);
			xudc->regs[R_ISR] |= (1 << (epnumber + 8));
			xudc->curr_buf[epnumber] = 0;
			if (dir == TO_HOST)
				xudc->regs[R_EP0_BUF1_COUNT + (epnumber * 4)] -= len;
			else
				xudc->regs[R_EP0_BUF1_COUNT + (epnumber * 4)] = len;
		}
	}

    xlx_udc_update_irq(xudc);

	return 0;
}

static GadgetOps xudc_gadget_ops = {
	.usb_reset			= xudc_usb_reset,
	.set_speed			= xudc_set_speed,
	.setup_request		= xudc_setup,
	.in_token			= xudc_in_token,
	.out_token			= xudc_out_token,
	.xfer_done			= xudc_xfer_done,
};

static const MemoryRegionOps udc_ops = {
    .read = udc_read,
    .write = udc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void xilinx_udc_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    XilinxUSB *s = XILINX_USB(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    DB_PRINT("\n");

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->mmio, OBJECT(s), &udc_ops, s,
                          "xilinx-udc", R_MAX * 4);
    sysbus_init_mmio(sbd, &s->mmio);

	register_udc(s, &xudc_gadget_ops);
}

static Property xilinx_udc_properties[] = {
    DEFINE_PROP_UINT8("xlnx,has-builtin-dma", XilinxUSB, has_dma, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void xilinx_udc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = xlx_udc_reset;
    dc->props = xilinx_udc_properties;
}

static const TypeInfo xilinx_udc_info = {
    .name           = TYPE_XILINX_USB,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(XilinxUSB),
    .class_init     = xilinx_udc_class_init,
    .instance_init  = xilinx_udc_init,
};

static void xilinx_udc_register_types(void)
{
    type_register_static(&xilinx_udc_info);
}

type_init(xilinx_udc_register_types)
