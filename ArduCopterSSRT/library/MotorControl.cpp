/*
 * MotorControl.c
 * 
 * Copyright 2012 Scott Thomson <scotty3785@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
 
#include "axis.cpp"
#include "AP_Math.h"

#define N_TO_U_SCALE_F    250.0F

class MotorControl
{
	public:
	
	float previousheight = 0;

		MotorControl()
		{
			firsttime = true;
		}

		/* Gains:
		*  1st dim:
		*  0: Pitch
		*  1: Roll
		*  2: Yaw
		*
		*  2nd dim
		*  0 = angle
		*  1 = rate
		*
		*  3rd dim:
		*  0 = p
		*  1 = i
		*  2 = d
		*/
		void calculate(uint16_t output[4],
				int16_t inputs[4],
				float gyros[3],
				float accels[3],
				float time,
				float gains[4][2][3],
				float heightVal)
		{
			float dt = ((float)(time - lasttime) / 1000000);			
			lasttime = time;
			
			if( firsttime == false )
			{
				float throttle = (float) inputs[0]-1000;
				float pitchcmd = (float) inputs[2]/1000;
				float rollcmd = (float) inputs[1]/1000;
				float yawcmd = (float) inputs[3]/1000;

				float pitch_ang = accels[1];
				float roll_ang = accels[0];
				float yaw_ang = accels[2];

				float pitch_rate = gyros[1];
				float roll_rate = gyros[0];
				float yaw_rate = gyros[2];
				float heightVel = findHeightRate(dt, heightVal);

				float md_pitch = pitchCtrl.calc(pitchcmd, pitch_ang, pitch_rate, dt, gains[1]);
				float md_roll  = rollCtrl.calc(rollcmd, roll_ang, roll_rate, dt, gains[0]);
				float md_yaw = yawCtrl.calc(yawcmd, yaw_ang, yaw_rate, dt, gains[2]);
				//float md_height = heightCtrl.calc(throttle, heightVal, heightVel, dt, gains[3]);
				float md_height = heightCtrl.calc(100, heightVal, heightVel, dt, gains[3]);

				float motor1diff = (md_pitch / -2.0) + (md_yaw / 4.0);
				float motor2diff = (md_pitch / 2.0) + (md_yaw / 4.0);
				float motor3diff =  (md_roll / 2.0)  - (md_yaw / 4.0);
				float motor4diff = (md_roll / -2.0)  - (md_yaw / 4.0);
				
				float throttleMod = 1000 + ((md_height));
				
				output[0] = (uint16_t) (throttleMod + (motor1diff * N_TO_U_SCALE_F));
				output[1] = (uint16_t) (throttleMod + (motor2diff * N_TO_U_SCALE_F));
				output[2] = (uint16_t) (throttleMod + (motor3diff * N_TO_U_SCALE_F));
				output[3] = (uint16_t) (throttleMod + (motor4diff * N_TO_U_SCALE_F));
				
				for(int n=0; n<4; n++){
					if(output[n] < 1000){
						output[n] = 1000;
					}
					
					if(output[n] > 2000){
						output[n] = 2000;
					}
				}
			}

			firsttime = false;
		}
		
		void reset(void)
		{
			pitchCtrl.reset();
			rollCtrl.reset();
			yawCtrl.reset();
		}
		
		float findHeightRate(float time, float height)
		{
			float heightRate  = ((height-previousheight)/time);
			previousheight = height;
			return heightRate;
		}

	private:

		float lasttime;
		bool firsttime;
		Axis pitchCtrl;
		Axis rollCtrl;
		Axis yawCtrl;
		Axis heightCtrl;
};
