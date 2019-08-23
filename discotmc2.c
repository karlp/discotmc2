/*
 * discotmc2, usb controlled dac, for stm32 discovery boards.
 * Consider to be BSD2 Clause, Apache 2.0, MIT, or ISC licensed, at your
 * pleasure.
 * karl Palsson <karlp@tweak.net.au>
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/usart.h>

#include "hw.h"
#include "trace.h"

#include "api-control-reqs.h"
#include "funcgen.h"

#define ER_DEBUG
#ifdef ER_DEBUG
#define ER_DPRINTF(fmt, ...) \
	do { printf(fmt, ## __VA_ARGS__); } while (0)
#else
#define ER_DPRINTF(fmt, ...) \
	do { } while (0)
#endif

TaskHandle_t taskHandleUSBD;

struct hw_detail hw_details = {
	.led_rcc = RCC_GPIOB,
	.led_port = GPIOB,
	.led_pin = GPIO1,
};

/* provided in board files please*/
/**
 * Setup any gpios or anything hardware specific.
 * Should _only_ be things that can't be done in shared init()
 */
static void hw_init(void)
{
	rcc_periph_clock_enable(hw_details.led_rcc);
	gpio_mode_setup(hw_details.led_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, hw_details.led_pin);
}


static void prvTaskGreenBlink1(void *pvParameters)
{
	(void) pvParameters;
	int i = 0;
	while (1) {
		printf("B");
		vTaskDelay(portTICK_PERIOD_MS * 500);
		gpio_toggle(hw_details.led_port, hw_details.led_pin);
	}
	vTaskDelete(NULL);
}

static void prvTaskFuncGen(void *pvParameters)
{
	(void)pvParameters;
	int i = 0;
	funcgen_plat_init();
	while (1) {
		i++;
		printf("dac thread still alive %d\n", i);
		vTaskDelay(portTICK_PERIOD_MS * 5000);
	}
}


#define BULK_EP_MAXPACKET	64
#define MY_USB_CFG 2

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_VENDOR,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = BULK_EP_MAXPACKET,
	.idVendor = 0xcafe, /* TODO get our own PID as an OS project */
	.idProduct = 0xcaff,
	.bcdDevice = 0x0001,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor endp_bulk[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x01,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = BULK_EP_MAXPACKET,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x81,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = BULK_EP_MAXPACKET,
		.bInterval = 1,
	},
};

static const struct usb_interface_descriptor iface_sourcesink[] = {
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_VENDOR,
		.iInterface = 0,
		.endpoint = endp_bulk,
	}
};

static const struct usb_interface ifaces_sourcesink[] = {
	{
		.num_altsetting = 1,
		.altsetting = iface_sourcesink,
	}
};

static const struct usb_config_descriptor config[] = {
	{
		.bLength = USB_DT_CONFIGURATION_SIZE,
		.bDescriptorType = USB_DT_CONFIGURATION,
		.wTotalLength = 0,
		.bNumInterfaces = 1,
		.bConfigurationValue = MY_USB_CFG,
		.iConfiguration = 4, /* string index */
		.bmAttributes = 0x80,
		.bMaxPower = 0x32,
		.interface = ifaces_sourcesink,
	},
};

static char serial[] = "0123456789.0123456789.0123456789";
static const char *usb_strings[] = {
	"karlp-cafe",
	"discotmc2",
	serial,
	"source and sink data",
};

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[3*BULK_EP_MAXPACKET];

/* We don't actually do anything with the bulk endpoints (yet?)*/
static void usb_funcgen_out_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void) usbd_dev;
	(void) ep;
	//uint16_t x;
	//x = usbd_ep_read_packet(usbd_dev, ep, dest, BULK_EP_MAXPACKET);
}

static void usb_funcgen_in_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void) usbd_dev;
	(void) ep;
	//uint16_t x = usbd_ep_write_packet(usbd_dev, ep, src, BULK_EP_MAXPACKET);
}


static enum usbd_request_return_codes usb_funcgen_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req,
	uint8_t **buf,
	uint16_t *len,
	usbd_control_complete_callback *complete)
{
	(void)usbd_dev;
	(void)complete;
	ER_DPRINTF("ctrl breq: %x, bmRT: %x, wval: %x, windex :%x, wlen: %x\n",
		req->bRequest, req->bmRequestType,
		req->wValue, req->wIndex, req->wLength);

	uint8_t *real = *buf;
	/* TODO - what do the return values mean again? */
	/* remember, we're only taking vendor interface reqs here, no need to recheck! */
	float freq;
	float amp;
	float offset;
	uint32_t val;
	switch (req->bRequest) {
	case UCR_SETUP_SINE:
		val = *(uint32_t *)&real[0];
		freq = val / 1000.0f;
		val = *(uint32_t *)&real[4];
		amp = val / 1000.0f;
		val = *(uint32_t *)&real[8];
		offset = val / 1000.0f;
		ER_DPRINTF("do sine: freq: %f, amp: %f, offset: %f\n", freq, amp, offset);
		funcgen_sin(req->wValue, freq, amp, offset);
		*len = 0;
		return USBD_REQ_HANDLED;
	case UCR_SET_LED:
		if (req->wValue) {
			gpio_set(hw_details.led_port, hw_details.led_pin);
		} else {
			gpio_clear(hw_details.led_port, hw_details.led_pin);
		}
		*len = 0;
		return USBD_REQ_HANDLED;
	case UCR_SET_OUTPUT:
		// TODO - could handle these in one req? would it save anything meaningful?
		funcgen_output(req->wValue, req->wIndex == 1);
		*len = 0;
		return USBD_REQ_HANDLED;
	case UCR_SET_BUFFER:
		// Buffer enable
		funcgen_buffer(req->wValue, req->wIndex == 1);
		*len = 0;
		return USBD_REQ_HANDLED;
	case UCR_SETUP_TRIANGLE:
		val = *(uint32_t *)&real[0];
		freq = val / 1000.0f;
		val = *(uint32_t *)&real[4];
		amp = val / 1000.0f;
		val = *(uint32_t *)&real[8];
		offset = val / 1000.0f;
		ER_DPRINTF("do triangle: freq: %f, amp: %f, offset: %f\n", freq, amp, offset);
		funcgen_triangle(req->wValue, freq, amp, offset);
		*len = 0;
		return USBD_REQ_HANDLED;

	default:
		ER_DPRINTF("unexpected control breq: %x deferring?!\n", req->bRequest);
		break;
	}
	return USBD_REQ_NEXT_CALLBACK;
}

static void usb_funcgen_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	ER_DPRINTF("set cfg %d\n", wValue);
	switch (wValue) {
	case MY_USB_CFG:
		usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			usb_funcgen_out_cb);
		usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			usb_funcgen_in_cb);
		usbd_register_control_callback(
			usbd_dev,
			USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
			USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
			usb_funcgen_control_request);
		break;
	default:
		ER_DPRINTF("set configuration unknown: %d\n", wValue);
	}
}


/* See libopencm3-tests/tests/freertos-gadget-zero as a standalone project */
static void prvTaskUSBD(void *pvParameters)
{
	(void)pvParameters;

	/* Enable built in USB pullup on L1.  Some boards have a hard pullup,
	 * as the L1 errata says the onboard pullup is out of spec, but
	 * this doesn't hurt those boards */
        rcc_periph_clock_enable(RCC_SYSCFG);
        SYSCFG_PMC |= SYSCFG_PMC_USB_PU;

#ifdef ER_DEBUG
	setbuf(stdout, NULL);
#endif
	// TODO - load the DFU serial or something.
	static const char *userserial = "myserial";
	if (userserial) {
		usb_strings[2] = userserial;
	}
	usbd_device *our_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, config,
		usb_strings, 5,
		usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(our_dev, usb_funcgen_set_config);

	/* numerically greater than free rtos kernel split (lower priority) */
	nvic_set_priority(NVIC_USB_LP_IRQ, 6<<4);
	nvic_enable_irq(NVIC_USB_LP_IRQ);

	const TickType_t xBlockTime = pdMS_TO_TICKS( 500 );
	uint32_t ulNotifiedValue;

	ER_DPRINTF("USBD: loop start\n");
	while (1) {

		ulNotifiedValue = ulTaskNotifyTake(pdTRUE, xBlockTime);
		trace_send8(1, ulNotifiedValue);

		if (ulNotifiedValue == 0) {
			/* No big deal, just no usb traffic. just gives us an idle blip */
			ER_DPRINTF(".");
		} else {
			/* ulNotifiedValue holds a count of the number of outstanding
			interrupts.  Process each in turn. */
			while (ulNotifiedValue--) {
				usbd_poll(our_dev);
			}
			nvic_enable_irq(NVIC_USB_LP_IRQ);
		}
	}
}

/* In case you're on hardware that doesn't have trace connected!
 * TODO - this (obviously) needs to get split out to board layers */
static void setup_usart(void)
{
        uint32_t udev = USART1;
        rcc_periph_clock_enable(RCC_USART1);
        rcc_periph_clock_enable(RCC_GPIOB);
        gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
        gpio_set_af(GPIOB, GPIO_AF7, GPIO6|GPIO7);

        usart_set_baudrate(udev, 115200);
        usart_set_databits(udev, 8);
        usart_set_parity(udev, USART_PARITY_NONE);
        usart_set_stopbits(udev, USART_STOPBITS_1);
        usart_set_mode(udev, USART_MODE_TX_RX);
        usart_set_flow_control(udev, USART_FLOWCONTROL_NONE);

        /* Finally enable the USART. */
        usart_enable(udev);
}

int main(void)
{
        const struct rcc_clock_scale myclock_purple_faker = {
                .pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
                .pll_mul = RCC_CFGR_PLLMUL_MUL12,
                .pll_div = RCC_CFGR_PLLDIV_DIV3,
                .hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
                .ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
                .ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
                .voltage_scale = PWR_SCALE1,
                .flash_waitstates = 1,
                .ahb_frequency = 32e6,
                .apb1_frequency = 32e6,
                .apb2_frequency = 32e6,
        };

	const struct rcc_clock_scale myclock_locm3_hw1 = {
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.pll_mul = RCC_CFGR_PLLMUL_MUL6,
		.pll_div = RCC_CFGR_PLLDIV_DIV3,
		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_waitstates = 1,
		.ahb_frequency = 32e6,
		.apb1_frequency = 32e6,
		.apb2_frequency = 32e6,
	};

	// TODO - properly use board/hw support! duh
	//rcc_clock_setup_pll(&myclock_locm3_hw1);
	rcc_clock_setup_pll(&myclock_purple_faker);
	hw_init();
	setup_usart();
	printf("starting freertos...\n");

	xTaskCreate(prvTaskGreenBlink1, "green.blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(prvTaskFuncGen, "funcgen", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(prvTaskUSBD, "USBD", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, &taskHandleUSBD);

	vTaskStartScheduler();

	return 0;
}

void usb_lp_isr(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	nvic_disable_irq(NVIC_USB_LP_IRQ);
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(taskHandleUSBD, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vAssertCalled(const char * const pcFileName, unsigned long ulLine)
{
	volatile unsigned long ulSetToNonZeroInDebuggerToContinue = 0;

	/* Parameters are not used. */
	(void) ulLine;
	(void) pcFileName;

	taskENTER_CRITICAL();
	{
		while (ulSetToNonZeroInDebuggerToContinue == 0) {
			/* Use the debugger to set ulSetToNonZeroInDebuggerToContinue to a
			non zero value to step out of this function to the point that raised
			this assert(). */
			__asm volatile( "NOP");
			__asm volatile( "NOP");
		}
	}
	taskEXIT_CRITICAL();
}

/* Thanks for not providing prototypes freertos. */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName );
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	volatile unsigned long ulSetToNonZeroInDebuggerToContinue = 0;
	(void) xTask;

	ER_DPRINTF("!!!OVERFLOW in %s\n", pcTaskName);
	taskENTER_CRITICAL();
	{
		while (ulSetToNonZeroInDebuggerToContinue == 0) {
			/* Use the debugger to set ulSetToNonZeroInDebuggerToContinue to a
			non zero value to step out of this function to the point that raised
			this assert(). */
			__asm volatile( "NOP");
			__asm volatile( "NOP");
		}
	}
	taskEXIT_CRITICAL();
}