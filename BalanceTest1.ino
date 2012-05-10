// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_DCM library
//

#include <FastSerial.h>
#include <SPI.h>
#include <I2C.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_IMU.h>
#include <AP_DCM.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <DataFlash.h>
#include <APM_RC.h>
#include <GCS_MAVLink.h>
#include <AP_GPS.h>
#include <AP_Baro.h>

/* Modes used during the main loop */

#define INIT 0  //Initialise unit
#define MAIN 1  //Normal Operation

/* Input Channels from Reciever */
#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3
#define SWITCH 4

/* Default scale values */
#define SF 0.02        //Scale Factor to generate delta value (/100)

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  scheduler;
APM_RC_APM1 APM_RC;


AP_ADC_ADS7844          adc;
AP_InertialSensor_Oilpan ins( &adc );

static GPS         *g_gps;

AP_IMU_INS imu( &ins);
AP_DCM  dcm(&imu, g_gps);

# define A_LED_PIN        37
# define C_LED_PIN        35
# define LED_ON           HIGH
# define LED_OFF          LOW
# define MAG_ORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD

int mode; 				      //used to select mode of operation.
int pitch_trim = 0;		//the centred value of pitch
int delta = 0;			   //the value used to adjust left/right motors.
int rc_data[8];			  //Stores all 8 channels of RC data
int motor1;            //Value for Left Hand motor
int motor2;            //Value for Right Hand motor


static void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
    digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}

void setup(void)
{
	Serial.begin(115200);
	Serial.println("Starting up...");

	isr_registry.init();
	scheduler.init(&isr_registry);
	APM_RC.Init(&isr_registry);

	APM_RC.enable_out(CH_1);
    APM_RC.enable_out(CH_2);
    APM_RC.enable_out(CH_3);
    APM_RC.enable_out(CH_4);
    APM_RC.enable_out(CH_5);
    APM_RC.enable_out(CH_6);
    APM_RC.enable_out(CH_7);
    APM_RC.enable_out(CH_8);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);

	imu.init(IMU::COLD_START, delay, flash_leds, &scheduler);
	imu.init_accel(delay, flash_leds);
}

void loop(void)
{
	static uint16_t counter;
	static uint32_t last_t, last_print;
	uint32_t now = micros();
	float deltat;

	if (last_t == 0) {
		last_t = now;
		return;
	}
	deltat = (now - last_t) * 1.0e-6;
	last_t = now;

	dcm.update_DCM();
	delay(20);
	counter++;

	if (APM_RC.GetState() == 1)
	{
		for (int i = 0; i < 8;i++)
		{
			rc_data[i] = APM_RC.InputCh(i);
		}
		if (rc_data[THROTTLE] < 1090)
		{
			APM_RC.OutputCh(0,1080);
			APM_RC.OutputCh(1,1080);
		}
		else
		{
			//delta = (pitch_trim - rc_data[PITCH]) * SF;
			delta = -1 * dcm.pitch_sensor * SF;
			motor1 = (rc_data[THROTTLE] - delta);
			motor2 = (rc_data[THROTTLE] + delta);
			APM_RC.OutputCh(0,motor1);
			APM_RC.OutputCh(1,motor2);
		}
	}

	if (now - last_print >= 0.5e6) {
		Vector3f accel = imu.get_accel();
		Vector3f gyro  = imu.get_gyro();
		gyro  = imu.get_gyro();
		accel = imu.get_accel();
		Serial.printf_P(PSTR("p:%4d in:%4d d:%4d m1:%4d m2:%4d\n"),(int)dcm.pitch_sensor,rc_data[PITCH],delta,motor1,motor2);
		last_print = now;
		counter = 0;
	}
}
