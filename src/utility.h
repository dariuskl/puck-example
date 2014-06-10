/*!
 * \file	utility.h
 * \date	2014-02-17
 * \ingroup	puck2bt
 *
 * \author	Maximilian Stolze <maximilian.stolze@smail.fh-koeln.de>
 *
 * \copyright	This file is part of a project that was conducted in the context
 * 		of the master course "Special Aspects of Autonomous Mobile
 * 		Systems" at the Cologne University of Applied Sciences.
 *
 * Contained are utility functions that simplify development for the puck.
 */

/*!
 * \addtogroup puck2bt
 * @{
 */

/*!
 * Checks the proximity sensors for obstacles on the right (seen as the robot).
 */
#define PATH_OBSTRUCTED_RIGHT()	((prox_values[0] > 300) ||	\
				 (prox_values[1] > 300) ||	\
				 (prox_values[2] > 1500))

/*!
 * Checks the proximity sensors for obstacles on the left (seen as the robot).
 */
#define PATH_OBSTRUCTED_LEFT()	((prox_values[7] > 300) ||	\
				 (prox_values[6] > 300) ||	\
				 (prox_values[5] > 1500))

void wait(long num);
void myWait(long milli);
int get_selector();
void setLED(int LEDnum, int state);
void allRedLEDsOff();
void allRedLEDsOn();

void setErrorPercent(int input);
void setSpeeds(int leftSpeed, int rightSpeed);
void moveForward(int dist, int speed);
void move(int dist, int speed);
void turn(double degrees, int speed);



char btcomGetCharacter();

/*!
 * @}
 */

