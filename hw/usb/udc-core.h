#ifndef UDC_CORE_H
#define UDC_CORE_H

#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <memory.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#include <asm/byteorder.h>

#include <linux/types.h>
//#include "gadgetfs.h"
#include <linux/usb/ch9.h>

#include "usbstring.h"

#define MY_MAGIC        	'U'
#define USER_MODE       	_IO(MY_MAGIC, 1)
#define USER_MODE_ADDRESS	_IOW(MY_MAGIC, 2, unsigned char)

struct UDC;

struct EP {
	int fd;
	uint8_t epnumber;
	uint8_t direction;
	uint8_t type;
	char name[20];
	uint32_t xfer_len;
	bool enabled;
};

#define TO_HOST		1
#define FROM_HOST	0

#define BULK 		0
#define ISOC 		1
#define INTERRUPT 	2

enum usb_state {
	NOT_ATTACHED = 0,
	SETUP,
	DATA,
	COMPLETE,
	STATUS,	
};


struct GadgetOps {
	void (*usb_reset) (void *udc);
	void (*set_speed) (void *udc, enum usb_device_speed speed);

	int (*setup_request) (void *udc, struct usb_ctrlrequest *req);		
	int (*data_nak) (void *udc, int ep, int dir);		
	int (*data_complete) (void *udc, int ep, int dir);		
	int (*status_nak) (void *udc, int ep, int dir);		
	int (*status_complete) (void *udc, int ep, int dir);		
	int (*setup_abort) (void *udc, int ep, int dir);		
	int (*out_token)(void *udc, int epnumber);
	int (*in_token)(void *udc, int epnumber);
	int (*xfer_done) (void *udc, int epnumber, int dir, uint32_t len);
};

typedef struct GadgetOps GadgetOps; 

struct UDC {
	void *opaque;
	int ep0fd;
	GadgetOps *ops;
	struct EP eps[16];
	enum usb_device_speed speed;
	volatile enum usb_state	state;
	bool three_stage;
	uint32_t xfer_len;
};

void register_udc(void *data, GadgetOps *ops);

int udc_connect(void);
int udc_disconnect(void);

int udc_ep_enable(uint8_t usbnum, uint8_t usbdir, uint8_t type,
					uint32_t maxpktsize);
int udc_ep_disable(uint8_t usbnum, uint8_t usbdir);

int udc_ep_send(int epnumber, void *buffer, uint32_t length);
int udc_ep_receive(int epnumber, void *buffer, uint32_t length);
int udc_send_status(int ep, int dir);

#endif
