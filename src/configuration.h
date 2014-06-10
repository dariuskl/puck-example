/*!
 * \file	configuration.h
 * \date	2014-02-21
 * \ingroup	puck2bt
 *
 * \author	Darius Kellermann <darius.kellermann@smail.fh-koeln.de>
 *
 * \copyright	This file is part of a project that was conducted in the context
 * 		of the master course "Special Aspects of Autonomous Mobile
 * 		Systems" at the Cologne University of Applied Sciences.
 */

/*!
 * \defgroup puck2bt e-puck Software
 * \addtogroup puck2bt
 * @{
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

/*!
 * Whether the proximity sensors should be initialized and used for navigation.
 *
 * \note	There is not guarantee that the program works, when disabling
 * 		arbitrary modules.
 */
#define DBG_INCLUDE_PROXIMITY		1


/*!
 * Whether the motion module is enabled. If this is set to 0, the robot will not
 * move.
 *
 * \note	There is not guarantee that the program works, when disabling
 * 		arbitrary modules.
 */
#define DBG_INCLUDE_MOTION		1
#if DBG_INCLUDE_MOTION == 1
/*
 * Motion configuration parameters.
 */
#define MOVING_SPEED 200	/*!< The speed at which the puck moves */
#define TURNING_SPEED 50	/*!< The speed at which the puck turns */
#endif	/* DBG_INCLUDE_MOTION */


#define DBG_INCLUDE_TRANSMISSION	1


#define DBG_INCLUDE_CAM			1
#if DBG_INCLUDE_CAM == 1
/*
 * Camera configuration parameters.
 */
#define IMG_H		25	/*!< Area of interest height */
#define IMG_W		64	/*!< Area of interest width */
#define IMG_SS		8	/*!< Sub-sampling ratio */
/*!
 * Defines the color mode that should be use when configuring the camera.
 */
#define CAM_MODE 	GREY_SCALE_MODE
#endif	/* DBG_INCLUDE_CAM */

#endif /* CONFIGURATION_H_ */

/*!
 * @}
 */

