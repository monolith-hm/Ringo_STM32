/*
 * MadgwickAHRS.h
 *
 *  Created on: May 23, 2022
 *      Author: heemu.lee
 */

#ifndef INC_MADGWICKAHRS_H_
#define INC_MADGWICKAHRS_H_

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void ConvertQuaterToEuler(float q0, float q1, float q2, float q3);

#endif /* INC_MADGWICKAHRS_H_ */
