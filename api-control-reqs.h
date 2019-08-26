#pragma once

/**
 * USB control requests.
 */
enum discotmc2_usb_ctrl_reqs {
	UCR_SETUP_SINE = 1,
	UCR_SET_LED = 3,
	UCR_SET_OUTPUT = 4,
	UCR_SET_BUFFER = 5,
	UCR_SETUP_TRIANGLE = 6,
	UCR_SYNC = 7,
};