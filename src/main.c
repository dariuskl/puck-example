/*!
 * \file	main.c
 * \date	2014-02-17
 * \ingroup	puck2bt
 *
 * \author	Maximilian Stolze <maximilian.stolze@smail.fh-koeln.de>,
 * 		Darius Kellermann <darius.kellermann@smail.fh-koeln.de>,
 * 		Martin Schulze <martin.schulze@smail.fh-koeln.de>
 *
 * \copyright	This file is part of a project that was conducted in the context
 * 		of the master course "Special Aspects of Autonomous Mobile
 * 		Systems" at the Cologne University of Applied Sciences.
 *
 * Contained is the program that runs on the e-puck robot. It is built in a
 * modular fashion and the modules can be configured via the \f configuration.h
 * header file.
 *
 * Timer usage:
 *	- Timer 1: used by proximity sensors,
 *	- Timer 2: UNUSED,
 *	- Timer 3: used by motor control,
 *	- Timer 4: used by camera,
 *	- Timer 5: used by camera.
 */

/*!
 * \addtogroup puck2bt
 * @{
 */

#include <p30F6014A.h>

#include <stdint.h>
#include <stdlib.h>

#include <motor_led/e_led.h>
#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <motor_led/e_motors.h>
#include <a_d/e_prox.h>
#include <a_d/e_ad_conv.h>
#include <uart/e_uart_char.h>
#include <camera/fast_2_timer/e_poxxxx.h>

#include <pucom.h>

#include "configuration.h"
#include "utility.h"

/*!
 * Size of one image in bytes.
 *
 * The bytes per pixel depend on the camera configuration and are currently set
 * to one (grayscale).
 */
#define IMG_DATA_SIZE		(IMG_H * IMG_W)

struct img_buf {
	/*!
	 * 0 - the message has been transmitted,
	 * 1 - the message has NOT yet been transmitted.
	 */
	uint8_t data_is_new;	/*!< Indicates the state of the buffer */
	char data[IMG_DATA_SIZE];	/*!< Image data array */
};

/*!
 * These are the bit mask values for the puck's program selector.
 */
enum PROGRAM_SELECTIONS {
	SEL_MOTION = 1,		/*!< The motion FSM is enabled */
	SEL_SENSING = 2		/*!< The camera and transmission FSMs are enabled */
};

/*!
 * The states of the transmission FSM.
 */
enum TRANSMISSION_STATES {
	TX_INIT,	/*!< Initialization */
	TX_CONFIG,	/*!< Send PMT_CONFIG message */
	TX_CONFIG_ACK,	/*!< Wait for server acknowledgment */
	TX_VISUAL,	/*!< Check for available buffer and send PMT_VISUAL */
	TX_VISUAL_ACK,	/*!< Wait for server acknowledgment */
	TX_VISUAL_SENT	/*!< Wait until the data was sent */
};

/*!
 * The states of the camera FSM.
 */
enum CAMERA_STATES {
	CAM_INACTIVE, /*!< Camera inactive */
	CAM_IDLE,	/*!< Image capture not started */
	CAM_CAPTURING	/*!< Image capture in progress */
};

/*!
 * The states of the motion FSM.
 */
enum MOTION_STATES {
	MOT_INIT,	/*!< Initialization, move forward */
	MOT_FORWARDS,	/*!< Move forward */
	MOT_TURN_LEFT,	/*!< Turn left */
	MOT_TURN_RIGHT,	/*!< Turn right */
	MOT_STOP	/*!< Motors are stopped */
};

int main(void)
{
	int prox_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int closest, closest_index, i, sel = 0;
	unsigned char act_tx_buf = 0, act_img_buf = 0, ack = 0;
	int tx_state, img_state, move_state;
	struct img_buf img_bufs[2];
	struct puck_msg_hdr msg_hdr;
	struct puck_msg_config msg_config;

	img_bufs[0].data_is_new = 0;
	img_bufs[1].data_is_new = 0;

	tx_state = TX_INIT;
	img_state = CAM_INACTIVE;
	move_state = MOT_INIT;

	/* Initialization */
	e_init_port();
#if DBG_INCLUDE_PROXIMITY == 1
	e_init_prox();
#endif	/* DBG_INCLUDE_PROXIMITY */
#if DBG_INCLUDE_MOTION == 1
	e_init_motors();
#endif	/* DBG_INCLUDE_MOTION */
#if DBG_INCLUDE_TRANSMISSION == 1
	e_init_uart1();
#endif	/* DBG_INCLUDE_TRANSMISSION */
#if DBG_INCLUDE_CAM == 1
	e_poxxxx_init_cam();
	e_poxxxx_config_cam((ARRAY_WIDTH - (IMG_W * IMG_SS))/2,
			(ARRAY_HEIGHT - (IMG_H * IMG_SS))/2, IMG_W * IMG_SS,
			IMG_H * IMG_SS, IMG_SS, IMG_SS, CAM_MODE);
	e_poxxxx_write_cam_registers();
#endif	/* DBG_INCLUDE_CAM */

	/* Safety wait period to prevent UART clogging */
	myWait(500);

	/* Selector in position 0 lets the e-Puck wait at the start. */
	do {
		sel = get_selector();
		myWait(500);
	} while (sel == 0);

	while (1) {
		sel = get_selector();

#if DBG_INCLUDE_PROXIMITY == 1
		/*
		 * Determine which sensor sees the closest object.
		 */
		closest = 0;
		for (i = 0; i < 8; i++) {
			prox_values[i] = e_get_prox(i);
			if (prox_values[i] > closest) {
				closest = prox_values[i];
				closest_index = i;
			}
		}

		/*
		 * Indicate the direction of the closest object by
		 * turning on the LED facing it.
		 */
		allRedLEDsOff();
		setLED(closest_index, 1);
#endif	/* DBG_INCLUDE_PROXIMITY */

#if DBG_INCLUDE_MOTION == 1
		/*
		 * The motion state machine. This implements the puck's
		 * motion and obstacle avoidance.
		 */
		switch (move_state) {
		case MOT_INIT:
			if ((sel & SEL_MOTION) == 0) {
				setSpeeds(0, 0);
				move_state = MOT_STOP;
			}
			else {
				setSpeeds(MOVING_SPEED, MOVING_SPEED);
				move_state = MOT_FORWARDS;
			}
			break;
		case MOT_FORWARDS:
			if ((sel & SEL_MOTION) == 0) {
				setSpeeds(0, 0);
				move_state = MOT_STOP;
			}
			else if (PATH_OBSTRUCTED_RIGHT()) {
				setSpeeds(-TURNING_SPEED, TURNING_SPEED);
				move_state = MOT_TURN_LEFT;
			}
			else if (PATH_OBSTRUCTED_LEFT()) {
				setSpeeds(TURNING_SPEED, -TURNING_SPEED);
				move_state = MOT_TURN_RIGHT;
			}
			break;
		case MOT_TURN_LEFT:
			if ((sel & SEL_MOTION) == 0) {
				setSpeeds(0, 0);
				move_state = MOT_STOP;
			}
			else if (!PATH_OBSTRUCTED_RIGHT()) {
				setSpeeds(MOVING_SPEED, MOVING_SPEED);
				move_state = MOT_FORWARDS;
			}
			break;
		case MOT_TURN_RIGHT:
			if ((sel & SEL_MOTION) == 0) {
				setSpeeds(0, 0);
				move_state = MOT_STOP;
			}
			else if (!PATH_OBSTRUCTED_LEFT()) {
				setSpeeds(MOVING_SPEED, MOVING_SPEED);
				move_state = MOT_FORWARDS;
			}
			break;
		case MOT_STOP:
			if (sel & SEL_MOTION) {
				move_state = MOT_INIT;
			}
			break;
		}
#endif	/* DBG_INCLUDE_MOTION */

#if DBG_INCLUDE_TRANSMISSION == 1
		switch (tx_state) {
		case TX_INIT:
			/*
			 * TODO Empty the UART buffers.
			 */
			if ((sel & SEL_SENSING)) {
				tx_state = TX_CONFIG;
			}
			break;
		case TX_CONFIG:
			/*
			 * Send the configuration message to allow the server to
			 * configure itself.
			 */
			if ((sel & SEL_SENSING) == 0) {
				tx_state = TX_INIT;
			}
			else if ((sel & SEL_SENSING) && !e_uart1_sending()) {
				msg_hdr.type = PMT_CONFIG;
				msg_hdr.len = sizeof(msg_config);
				msg_config.cols = IMG_W;
				msg_config.rows = IMG_H;
				e_send_uart1_char((char *)&msg_hdr,
						sizeof(msg_hdr));
				tx_state = TX_CONFIG_ACK;
			}
			break;
		case TX_CONFIG_ACK:
			/*
			 * TODO Timeout the wait and reinitialize when timed
			 * out.
			 */
			if ((sel & SEL_SENSING) == 0) {
				tx_state = TX_INIT;
			}
			else if (e_ischar_uart1() && (ack != PMT_ACK)) {
				e_getchar_uart1((char *)&ack);
			}
			else if (!e_uart1_sending() && (ack == PMT_ACK)) {
				e_send_uart1_char((char *)&msg_config,
						sizeof(msg_config));
				tx_state = TX_VISUAL;
			}
			break;
		case TX_VISUAL:
			/*
			 * Check if a buffer has a new image which has not been
			 * transmitted yet and start transmission by sending
			 * the header.
			 */
			if ((sel & SEL_SENSING) == 0) {
				tx_state = TX_INIT;
			}
			else if ((img_bufs[0].data_is_new == 1) && !e_uart1_sending()) {
				msg_hdr.type = PMT_VISUAL;
				msg_hdr.len = IMG_DATA_SIZE;
				e_send_uart1_char((char *)&msg_hdr,
						sizeof(msg_hdr));
				act_tx_buf = 0;
				tx_state = TX_VISUAL_ACK;
			}
			else if ((img_bufs[1].data_is_new == 1) && !e_uart1_sending()) {
				msg_hdr.type = PMT_VISUAL;
				msg_hdr.len = IMG_DATA_SIZE;
				e_send_uart1_char((char *)&msg_hdr,
						sizeof(msg_hdr));
				act_tx_buf = 1;
				tx_state = TX_VISUAL_ACK;
			}
			break;
		case TX_VISUAL_ACK:
			/*
			 * Receive from UART.
			 */
			if ((sel & SEL_SENSING) == 0) {
				tx_state = TX_INIT;
			}
			else if (e_ischar_uart1() && (ack != PMT_ACK)) {
				e_getchar_uart1((char *)&ack);
			}
			else if (!e_uart1_sending() && (ack == PMT_ACK)) {
				e_send_uart1_char(img_bufs[act_tx_buf].data,
						IMG_DATA_SIZE);
				ack = 0;
				tx_state = TX_VISUAL_SENT;
			}
			break;
		case TX_VISUAL_SENT:
			/*
			 * Wait until the buffer was sent.
			 */
			if (!e_uart1_sending()) {
				img_bufs[act_tx_buf].data_is_new = 0;
				tx_state = TX_VISUAL;
			}
			break;
		}
#endif	/* DBG_INCLUDE_TRANSMISSION */

#if DBG_INCLUDE_CAM == 1
		switch (img_state) {
		case CAM_INACTIVE:
			if (sel & SEL_SENSING) {
				img_state = CAM_IDLE;
			}
			break;
		case CAM_IDLE:
			/* Check if a buffer is free to use and
			 * start capturing if this is the case */
			if ((sel & SEL_SENSING) == 0) {
				img_state = CAM_INACTIVE;
			}
			else if (img_bufs[0].data_is_new == 0) {
				e_poxxxx_launch_capture(img_bufs[0].data);
				act_img_buf = 0;
				img_state = CAM_CAPTURING;
			}
			else if (img_bufs[1].data_is_new == 0) {
				e_poxxxx_launch_capture(img_bufs[1].data);
				act_img_buf = 1;
				img_state = CAM_CAPTURING;
			}

			break;
		case CAM_CAPTURING:
			/* Wait until the image is captured and
			 * mark the buffer as used */
			if ((sel & SEL_SENSING) == 0) {
				img_state = CAM_INACTIVE;
			}
			else if (e_poxxxx_is_img_ready()) {
				img_bufs[act_img_buf].data_is_new = 1;
				img_state = CAM_IDLE;
			}
			break;
		}
#else	/* Real CAM is disabled, so here is a dummy */
		/*
		 * Camera is not included, so generate a fixed image.
		 */
		switch (img_state) {
		case CAM_INACTIVE:
			if (sel & SEL_SENSING) {
				img_state = CAM_IDLE;
			}
			break;
		case CAM_IDLE:
			/* Check if a buffer is free to use and
			 * start capturing if this is the case */
			if ((sel & SEL_SENSING) == 0) {
				img_state = CAM_INACTIVE;
			}
			else if (buf[0].data_is_new == 0) {
				for (i = 0; i < IMG_DATA_SIZE; i++)
					buf[0].data[i] = i;
				act_img_buf = 0;
				img_state = CAM_CAPTURING;
			}
			else if (buf[1].data_is_new == 0) {
				for (i = 0; i < IMG_DATA_SIZE; i++)
					buf[1].data[i] = i;
				act_img_buf = 1;
				img_state = CAM_CAPTURING;
			}
			break;
		case CAM_CAPTURING:
			/* Wait until the image is captured and
			 * mark the buffer as used */
			if ((sel & SEL_SENSING) == 0) {
				img_state = CAM_INACTIVE;
			}
			else {
				buf[act_img_buf].data_is_new = 1;
				img_state = CAM_IDLE;
			}
			break;
		}
#endif	/* DBG_INCLUDE_CAM */


		/*
		 * Short waiting period to give the motors and sensors time to
		 * react.
		 */
		myWait(100);
	}

	return 0;
}

/*!
 * @}
 */

