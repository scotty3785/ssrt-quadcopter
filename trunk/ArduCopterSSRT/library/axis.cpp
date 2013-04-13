/*
 * Axis.c
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
 
 #include "pid.cpp"

class Axis
{
	public:
	
		Axis()
		{
		}
		
		float calc(float acmd, float angle, float rate, float dt, float gains[2][3])
		{
			float motordiff;
			
			float kpa = gains[0][0];
			float kia = gains[0][1];
			float kda = gains[0][2];

			float kpr = gains[1][0];
			float kir = gains[1][1];
			float kdr = gains[1][2];

			float angle_error = acmd - angle;
			float rate_error;
			
			float rcmd = anglePID.vel_pid( kpa, kia, kda, dt, angle_error);
			
			rate_error = rcmd - rate;
			
			motordiff = ratePID.vel_pid( kpr, kir, kdr, dt, rate_error );
			
			return motordiff;
		}
	
	private:
	
		Pid anglePID;
		Pid ratePID;
};
