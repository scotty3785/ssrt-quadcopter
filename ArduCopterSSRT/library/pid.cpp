/*
 * Pid.c
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

class Pid
{
	private:
	
		float yn1;

		float xn1;
		float xn2;

	public:

		Pid()
		{
			yn1 = 0.0F;
			xn1 = 0.0F;
			xn2 = 0.0F;
		}

		float vel_pid(float kp, float ki, float kd, float t, float xn)
		{
			float output;
			
			float a = kp + (kd / t) + (ki * t);
			float b = -kp - (2 * kd / t);
			float c = kd / t;
			
			output = yn1 + (a * xn) + (b * xn1) + (c * xn2);
			
			yn1 = output;
			xn2 = xn1;
			xn1 = xn;
			
			return output;
		}
};
