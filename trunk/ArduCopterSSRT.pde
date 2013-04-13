// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_AHRS interface
//

#include <AP_HAL.h>
//#include <AP_HAL_PX4.h>
#include <AP_HAL_AVR.h>
//#include <AP_HAL_AVR_SITL.h>
//#include <AP_HAL_Empty.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>
#include "/home/scott/sketchbook/libraries/mavlink/include/common/mavlink.h"
#include "/home/scott/sketchbook/libraries/RTW_Control/MotorControl.cpp"

#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM1

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins( &adc );
#else
AP_InertialSensor_Stub ins;
#endif

AP_Compass_HMC5843 compass;
MotorControl motor;

GPS *g_gps;

AP_GPS_Auto g_gps_driver(&g_gps);

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(&ins, g_gps);
Vector3f drift;
//AP_AHRS_MPU6000  ahrs(&ins, g_gps);		// only works with APM2

AP_Baro_BMP085_HIL barometer;

#define HIGH 1
#define LOW 0

 # define SLIDE_SWITCH_PIN 40

 # define A_LED_PIN        37
 # define B_LED_PIN        36
 # define C_LED_PIN        35
 # define LED_ON           HIGH
 # define LED_OFF          LOW
 # define SLIDE_SWITCH_PIN 40
 # define MAG_ORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD

//#######################MAVLINK Parameters ############################
#define ONBOARD_PARAM_COUNT 5

uint16_t m_parameter_i = 0;

struct global_struct
{
	float param[ONBOARD_PARAM_COUNT];
	char param_name[ONBOARD_PARAM_COUNT][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
};
 
struct global_struct global_data;

//int system_type = MAV_QUADROTOR;
uint8_t system_id = 101;
uint8_t component_id =0;
int system_type = 2;
int autopilot_type = MAV_AUTOPILOT_GENERIC;

// Initialize the required buffers 
mavlink_message_t msg;
mavlink_status_t status; 
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//######################################################################
//#######################Remote Control In  ############################
//uint16_t channels[8];

/* Modes used during the main loop */
int mode = 0;

#define WAIT -1 //Don't start yet.
#define INIT 0  //Initialise unit
#define MAIN 1  //Normal Operation

#define THROTTLE 0
#define ROLL 1
#define PITCH 2
#define YAW 3
#define SWITCH 5

//######################################################################

float pitchAngleGain[3] = {1,0.25,0.1};
float pitchRateGain[3] = {1,0,0};
float rollAngleGain[3] = {1,0.25,0.1};
float rollRateGain[3] = {1,0,0};
float yawAngleGain[3] = {1,0,0};
float yawRateGain[3] = {1,0,0};

	float gains[3][2][3]= {	
	{{rollAngleGain[0],rollAngleGain[1],rollAngleGain[2]},{rollRateGain[0],rollRateGain[1],rollRateGain[2]}},
	{{pitchAngleGain[0],pitchAngleGain[1],pitchAngleGain[2]},{pitchRateGain[0],pitchRateGain[1],pitchRateGain[2]}},
	{{yawAngleGain[0],yawAngleGain[1],yawAngleGain[2]},{yawRateGain[0],yawRateGain[1],yawRateGain[2]}}
	};

long roll_trim = 0;
long pitch_trim = 0;
long yaw_trim = 0;

static void flash_leds(bool on)
{
    hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
    hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}

void multiread(AP_HAL::RCInput* in, uint16_t* channels) {
    /* Multi-channel read method: */
    uint8_t valid;
    valid = in->read(channels, 8);
    /*hal.console->printf_P(
            PSTR("multi      read %d: %d %d %d %d %d %d %d %d\r\n"),
            (int) valid, 
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7]);*/
}

void multiwrite(AP_HAL::RCOutput* out, uint16_t* channels) {
    out->write(0, channels, 8);
    /* Upper channels duplicate lower channels*/
    out->write(8, channels, 8);
}

void setup(void)
{
	
	//Initialise the UART
	hal.uartA->begin(115200, 128, 256);

    // Configure the Slide Switch to be an input
    hal.gpio->pinMode(SLIDE_SWITCH_PIN, GPIO_INPUT);


    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
			 flash_leds);
    ins.init_accel(flash_leds);

    compass.set_orientation(MAG_ORIENTATION);
    ahrs.init();

    if( compass.init() ) 
    {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } 
    else 
    {
        hal.console->printf("No compass detected\n");
    }
    g_gps = &g_gps_driver;
	#if WITH_GPS
	g_gps->init(hal.uartB);
	#endif

	//Set the unit in to initialise mode
	mode = WAIT;
	
	
}

void loop(void)
{
    static uint16_t counter;
    static uint32_t last_t, last_print, last_compass;
	uint16_t channels[8];
	uint16_t rcout[4];
	float gyro[3];
	float accel[3];
    uint32_t now = hal.scheduler->micros();
    float heading = 0;
	uint32_t init_time = 0;



	switch(mode)
	{
		case WAIT:
			roll_trim = hal.rcin->read(PITCH);
			pitch_trim = hal.rcin->read(ROLL);
			yaw_trim = hal.rcin->read(YAW);
			if (hal.gpio->read(SLIDE_SWITCH_PIN) == 1)
			{
				mode = INIT;
				init_time = hal.scheduler->millis() + 5000;
				hal.console->printf("Exiting Wait Mode\n");
			}
			else
			{
				hal.rcout->disable_ch(0);
				hal.rcout->disable_ch(1);
				hal.rcout->disable_ch(2);
				hal.rcout->disable_ch(3);
			}
		break;
		case INIT:
			hal.console->printf("Enabling Motors\n");
			hal.rcout->enable_ch(0);
			hal.rcout->enable_ch(1);
			hal.rcout->enable_ch(2);
			hal.rcout->enable_ch(3);
			if (hal.scheduler->millis() > init_time)
			{
				mode = MAIN;
			}
			else
			{
		/*roll_trim and pitch_trim are the values of the the value of the
		 * controls when the levers are centred*/
				//roll_trim = hal.rcin->read(PITCH);
				//pitch_trim = hal.rcin->read(ROLL);
				//yaw_trim = hal.rcin->read(YAW);
			}
		break;
		case MAIN:
			if (last_t == 0) 
			{
		        last_t = now;
		        return;
		    }
		    last_t = now;

		    if (now - last_compass > 100*1000UL && compass.read()) 
			{
		        heading = compass.calculate_heading(ahrs.get_dcm_matrix());
		        // read compass at 10Hz
		        last_compass = now;
		    }

		    ahrs.update();
		    counter++;

		    if (now - last_print >= 100000 /* 100ms : 10hz */) 
			{
				int16_t trimmedChannels[4];
				multiread(hal.rcin, channels);
				if (channels[0] < 1090) channels[0] = 1080;
				trimmedChannels[THROTTLE] = channels[THROTTLE];
				trimmedChannels[ROLL] = channels[ROLL] - roll_trim;
				trimmedChannels[PITCH] = channels[PITCH] - pitch_trim;
				trimmedChannels[YAW] = channels[YAW] - yaw_trim;
				
	
		        drift  = ahrs.get_gyro();
        		/*hal.console->printf_P(
                PSTR("r:%4.1f  p:%4.1f y:%4.1f "
                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f rate=%.1f\n"),
                        ahrs.roll,
                        ahrs.pitch,
                        ahrs.yaw,
                        drift.x,
                        drift.y,
                        drift.z,
                        compass.use_for_yaw() ? ToDeg(heading) : 0.0,
                        (1.0e6*counter)/(now-last_print));*/
                accel[1] = ahrs.pitch;
                accel[0] = ahrs.roll;
                accel[2] = ahrs.yaw;
                gyro[0] = 0;
                gyro[1] = 0;
                gyro[2] = 0;

                motor.calculate(rcout,trimmedChannels,gyro,accel,hal.scheduler->micros(),gains);
                //hal.console->printf_P(PSTR("1:%i,2:%i,3:%i,4:%i\n"),channels[0],channels[1],channels[2],channels[3]);
                hal.console->printf_P(PSTR("roll:%4.1f,pitch:%4.1f,%u,%u,%u,%u\n"),ToDeg(ahrs.roll),ToDeg(ahrs.pitch),rcout[0],rcout[1],rcout[2],rcout[3]);
                hal.rcout->write(0,rcout[0]);
				hal.rcout->write(1,rcout[1]);
				hal.rcout->write(2,rcout[2]);
				hal.rcout->write(3,rcout[3]);
				
				
				last_print = now;
        		counter = 0;
    		}
    		
    		if (hal.gpio->read(SLIDE_SWITCH_PIN) == 0)
    		{
					mode = WAIT;
					hal.console->printf("Disabling Motors\n");
			}
			break;

		}

    
}

AP_HAL_MAIN();








