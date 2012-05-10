/*
Two Prop Power Adjust

Two ESCs are connected to the Ardupilot.
The Throttle channel is used to set the power to each motor
The roll channel is used to slightly offset the power from
the left or right motor.

If the roll level is moved to the left the right motor will
be given more power and the left motor less, tilting the jig
to the left.

by Scott Thomson (10 May 2012)

*/

#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h>
#include <FastSerial.h>

Arduino_Mega_ISR_Registry isr_registry
APM_RC_APM1 APM_RC;

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
#define SF 0.01

int mode 				//used to select mode of operation.
int pitch_trim = 0;		//the centred value of pitch
int delta = 0;			//the value used to adjust left/right motors.
int rc_data[8];			//Stores all 8 channels of RC data
int motor1;
int motor2;

void setup()
{
    isr_registry.init();
    APM_RC.Init(&isr_registry);
    APM_RC.enable_out(CH_1);
    APM_RC.enable_out(CH_2);
    APM_RC.enable_out(CH_3);
    APM_RC.enable_out(CH_4);
    APM_RC.enable_out(CH_5);
    APM_RC.enable_out(CH_6);
    APM_RC.enable_out(CH_7);
    APM_RC.enable_out(CH_8);

    Serial.begin(115200);
    Serial.println("Motor Balance Test");

    mode = INIT;
    Serial.print("Initialising");
    delay(1000);
}

void loop()
{

	switch(mode)
	{
	case INIT:

		if (APM_RC.GetState == 1)
		{
			//If we have done 5 seconds on Init, move to main mode
			if (millis() > 5000)
			{
				mode = MAIN;
				Serial.println("");
			}
			else
			{
				//pitch_trim is the value of the pitch lever when it is centred
				pitch_trim = APM_RC.InputCh(PITCH);
				Serial.print("*");
			}
		}

	break;
	case MAIN:
		if (APM_RC.GetState == 1)
		{
			//Read in the values from all 8 RX channels
			for (int i = 0; i < 8;i++)
			{
				rc_data[i] = APM_RC.InputCh(i);
			}

			delta = (pitch_trim - rc_data[PITCH]) * SF;
			motor1 = (rc_data[THROTTLE] + delta);
			motor2 = (rc_data[THROTTLE] - delta);
			APM_RC.OutputCh(CH1,motor1);
			APM_RC.OutputCh(CH2,motor2);
			Serial.printf_P(PSTR("t:%04d p:%04d m1:%04d m2:%04d\n"),(int)rc_data[THROTTLE],(int)rc_data[PITCH],(int)motor1,(int)motor2);
		}
	break;
	};

}