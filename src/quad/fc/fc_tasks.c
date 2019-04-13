
#include <stdio.h>
#include <math.h>
#include "fc_tasks.h"
#include "scheduler.h"
#include "fc_core.h"
#include "common.h"
#include "rx.h"
#include "led.h"
#include "imu.h"		// including time.h
#include "gyro.h"
#include "acceleration.h"
#include "configMaster.h"
#include "runtime_config.h"
#include "led.h"
#include "pwm_output.h"
#include "maths.h"
#include "button.h"
#include "system.h"
#include "rx_pwm.h"
#include "oled.h"
#include "ultrasound.h"
#include "asyncfatfs.h"
#include "blackbox.h"
#include "pid.h"

//#define TASKS_LEDS_TESTING

/* taskMotorEncoder is updated every 10 ms, inactivity is triggered every 10ms * 1000 = 10000 ms = 10 secs */
#define INACTIVITY_CONDITION			1000

#define TASK_PERIOD_HZ(hz)              (1000000 / (hz))            // units in microseconds (us)
#define TASK_PERIOD_MS(ms)              ((ms) * 1000)
#define TASK_PERIOD_US(us)              (us)

int Encoder1, Encoder2;
int stabilisePwmVal, velocityPwmVal, yawPwmVal;
int motor1Pwm, motor2Pwm;
int yawMagnitude = 95;
//int speedLimit = 10000;
//int speedLimit = 6000;
//float speedConfig = 70.0;
int speedLimit = 7000;
float speedConfig = 65.0;
//float speedConfig = 80.0;
//int yawMagnitude = 45;
uint32_t stationaryFlag = 0;
float velocityUpdatedMovement = 0.0f;
int balanceSetpoint = 0;
//int balanceSetpoint = 0;
float velocitySetpoint = 0.0f;
//float yawSetpoint = 0.0f;

bool isMotorActivated = true;

extern uint8_t driveForward, driveReverse, turnLeft, turnRight;

bool stopFlag = true;
bool isCollisionAvoidanceModeActivated = false;			// collision avoidance is switched off by default

static int32_t ultrasound1DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound2DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound3DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound4DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound5DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound6DistanceData = ULTRASOUND_OUT_OF_RANGE;

//static void taskUpdateRxMain(timeUs_t currentTimeUs);
static void taskUpdateAccelerometer(timeUs_t currentTimeUs);
static void taskMotorEncoder(timeUs_t currentTimeUs);
static void taskUpdateGyro(timeUs_t currentTimeUs);
static void taskOLEDDisplay(timeUs_t currentTimeUs);
static void taskUltrasound1ReadData(timeUs_t currentTimeUs);
static void taskUltrasound2ReadData(timeUs_t currentTimeUs);
static void taskUltrasound3ReadData(timeUs_t currentTimeUs);
static void taskUltrasound4ReadData(timeUs_t currentTimeUs);
static void taskUltrasound5ReadData(timeUs_t currentTimeUs);
static void taskUltrasound6ReadData(timeUs_t currentTimeUs);
static void taskDataLogger(timeUs_t currentTimeUs);
//static void taskBluetoothReceive(timeUs_t currentTimeUs);

/* Tasks initialisation */
cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = TASK_PERIOD_HZ(10),            	// 1000000 / 10 = 100000 us = 100 ms
        .staticPriority = TASK_PRIORITY_MEDIUM_HIGH,		// TASK_PRIORITY_MEDIUM_HIGH = 4
    },
    
	/* desiredPeriod = 4000 us = 4 ms = 250 Hz for F450 quad */
    [TASK_GYRO] = {
        .taskName = "GYRO",
        .taskFunc = taskUpdateGyro,
        .desiredPeriod = TASK_GYROPID_DESIRED_PERIOD,       // desiredPeriod = TASK_GYROPID_DESIRED_PERIOD = 125 us using STM32F4
        .staticPriority = TASK_PRIORITY_HIGH,           	// TASK_PRIORITY_HIGH = 5
    },
	
	[TASK_ACCEL] = {
		.taskName = "ACCEL",
		.taskFunc = taskUpdateAccelerometer,
		.desiredPeriod = TASK_PERIOD_HZ(1000),				// 1000000 / 1000 = 1000 us, every 1ms
		.staticPriority = TASK_PRIORITY_HIGH,				// TASK_PRIORITY_HIGH = 5
	},
	
	[TASK_ATTITUDE] = {
		.taskName = "ATTITUDE",
		.taskFunc = taskIMUUpdateAttitude,
		.desiredPeriod = TASK_PERIOD_HZ(100),				// 1000000 / 100 = 10000 us = 10 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 3
	},

    [TASK_MOTORENCODER] = {
        .taskName = "MOTOR_ENCODER",
        .taskFunc = taskMotorEncoder,
//        .desiredPeriod = TASK_PERIOD_HZ(250),            	// 1000000 / 250 = 4000 us = 4 ms		does not work!!!!!
//       .desiredPeriod = TASK_PERIOD_HZ(200),            	// 1000000 / 200 = 5000 us = 5 ms		does not work!!!!!
        .desiredPeriod = TASK_PERIOD_HZ(100),            	// 1000000 / 100 = 10000 us = 10 ms
        .staticPriority = TASK_PRIORITY_REALTIME,			// TASK_PRIORITY_REALTIME = 6
    },
	
	[TASK_OLEDDISPLAY] = {
		.taskName = "OLED_DISPLAY",
		.taskFunc = taskOLEDDisplay,
		.desiredPeriod = TASK_PERIOD_HZ(20),				// 1000000 / 20 = 50000 us = 50 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 3
	},

#if defined(ULTRASOUND)	
	[TASK_ULTRASOUND1_UPDATE] = {
		.taskName = "ULTRASOUND1_UPDATE",
		.taskFunc = ultrasound1Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	
	[TASK_ULTRASOUND1_READDATA] = {
		.taskName = "ULTRASOUND1_READDATA",
		.taskFunc = taskUltrasound1ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND2_UPDATE] = {
		.taskName = "ULTRASOUND2_UPDATE",
		.taskFunc = ultrasound2Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	
	[TASK_ULTRASOUND2_READDATA] = {
		.taskName = "ULTRASOUND2_READDATA",
		.taskFunc = taskUltrasound2ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	
	[TASK_ULTRASOUND3_UPDATE] = {
		.taskName = "ULTRASOUND3_UPDATE",
		.taskFunc = ultrasound3Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND3_READDATA] = {
		.taskName = "ULTRASOUND3_READDATA",
		.taskFunc = taskUltrasound3ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},

	[TASK_ULTRASOUND4_UPDATE] = {
		.taskName = "ULTRASOUND4_UPDATE",
		.taskFunc = ultrasound4Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND4_READDATA] = {
		.taskName = "ULTRASOUND4_READDATA",
		.taskFunc = taskUltrasound4ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	
	[TASK_ULTRASOUND5_UPDATE] = {
		.taskName = "ULTRASOUND5_UPDATE",
		.taskFunc = ultrasound5Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND5_READDATA] = {
		.taskName = "ULTRASOUND5_READDATA",
		.taskFunc = taskUltrasound5ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},

	[TASK_ULTRASOUND6_UPDATE] = {
		.taskName = "ULTRASOUND6_UPDATE",
		.taskFunc = ultrasound6Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND6_READDATA] = {
		.taskName = "ULTRASOUND6_READDATA",
		.taskFunc = taskUltrasound6ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
#endif
#if defined(USE_SDCARD) && defined(BLACKBOX)
	[TASK_DATALOGGER] = {
		.taskName = "DATA_LOGGER",
		.taskFunc = taskDataLogger,
		.desiredPeriod = TASK_PERIOD_HZ(250),				// 1000000 / 250 = 4000 us = 4 ms = 250 Hz
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
#endif
};

static void taskUpdateGyro(timeUs_t currentTimeUs)
{
	/* gyroUpdate */
	gyroUpdate(currentTimeUs);
}

static void taskUpdateAccelerometer(timeUs_t currentTimeUs)
{
//	UNUSED(currentTimeUs);
	
	accUpdate(currentTimeUs, &AccelerometerConfig()->accelerometerTrims);
}

static uint8_t buttonModeSwitchHandler(void)
{
	static uint16_t buttonFlag, buttonCount, buttonOnceCount, longPressCount;

	IO_t modeSwitchBtnPin = IOGetByTag(ButtonModeSwitchConfig()->btnPin);
	
	if ((IORead(modeSwitchBtnPin) == true)) {
		longPressCount++;
		
		if (buttonFlag == 0) {
			buttonFlag = 1;
		}
		
//		printf("longPressCount: %u\r\n", longPressCount);
		
		/* Check if button is pressed for at least 2 secs */
		if (longPressCount > 200) {
			longPressCount = 0;
			return 2;
		}
	} else {
		buttonFlag = 0;
		buttonCount = 0;
	}
	
//	printf("buttonFlag: %u\r\n", buttonFlag);		// buttonFlag == 1 if button is pressed, otherwise 0
//	printf("buttonCount: %u\r\n", buttonCount);		// buttonCount == 1 if button is pressed, otherwise 0
	
	if (buttonCount == 0) {
		if (buttonFlag == 1) {
			buttonOnceCount++;
			buttonCount = 1;
		}
		
		/* 
		 * buttonTwiceCount is 1 if button is pressed once.
		 * buttonTwiceCount is 2 if button is pressed twice.
		 */
//		printf("buttonOnceCount: %u\r\n", buttonOnceCount);
		
		if (buttonOnceCount == 1) {
			buttonOnceCount = 0;
			return 1;				// handling single click
		}
		
//		if (buttonTwiceCount == 2) {
//			buttonTwiceCount = 0;
//			buttonOnceCount = 0;
////			printf("%s, %d\r\n", __FUNCTION__, __LINE__);			
//			return 2;			// handling double clicks
//		}
	}
	
	return 0;
}

static void buttonModeSwitchPollOps(button_t *buttonModeSwitchConfig)
{
	uint8_t res;
	
	res = buttonModeSwitchHandler();
	
//	printf("button click: %u\r\n", res);

	/* Activate/Deactivate balancing routine by clicking button once */
	if (res == 1) {
		stopFlag = !stopFlag;
//		printf("stopFlag: %d\r\n", stopFlag);
		
		/* Activate the motors */
		if (stopFlag == false) {
#ifdef BLACKBOX
			if (feature(FEATURE_BLACKBOX)) {
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
				startBlackbox();
			}
#endif
		}
	}
	
	/* Case for long pressing */
	if (res == 2) {
//	if ((stopFlag == true) && (res == 2)) {
		isCollisionAvoidanceModeActivated = !isCollisionAvoidanceModeActivated;
	}
}

/* Encoder speed values */
int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;
	switch(TIMX)
	{
		case 2:
			Encoder_TIM = (short)TIM2->CNT;
			TIM2->CNT = 0;						// Clear CNT to zero to retrieve the speed of the motor
			break;
		
		case 3:
			Encoder_TIM = (short)TIM3->CNT;
			TIM3->CNT = 0;
			break;
		
		case 4:
			Encoder_TIM = (short)TIM4->CNT;
			TIM4->CNT = 0;
			break;
		
		default:
			Encoder_TIM = 0;
	}
	
//	printf("Encoder_TIM: %d\r\n", Encoder_TIM);
	
	return Encoder_TIM;
}

void limitMotorPwm(int *motor1, int *motor2)
{
	/* The maximum PWM value is 7200 setup by dcBrushedMotorInit(), 6900 for the upper bound */
	int Amplitude = 6900;
	
    if (*motor1 < -Amplitude)
		*motor1 = -Amplitude;	
	
	if (*motor1 > Amplitude)
		*motor1 = Amplitude;

    if (*motor2 < -Amplitude)
		*motor2 = -Amplitude;	
	
	if (*motor2 > Amplitude)
		*motor2 = Amplitude;

//	printf("motor[LimitPwm]: %d\r\n", motor);
}

/**************************************************************************
Description: incremental PI controller
Parameters: Encoder value, target speed
Returns: Motor updated PWM value

Incremental discrete PID equation
pwm += Kp[e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2e(k-1) + e(k-2)]
e(k) represents the current error
e(k-1) represents the previous error
pwm is the updated incremental PID value.

For speed control, the PI-only controller is utilised
pwm += Kp[e(k) - e(k-1)] + Ki * e(k)
**************************************************************************/
int Incremental_PIController(int Encoder, int Target)
{
	float Kp = 80, Ki = 0.2;
//	float updatedPWM;
	static int error, updatedPWM, prev_error;
//	static int error, prev_error;
//	printf("Encoder: %d\r\n", Encoder);
	error = Encoder - Target;                		// 计算偏差
//	printf("Bias: %d\r\n", Bias);
	updatedPWM += Kp * (error - prev_error) + Ki * error;   	// 增量式PI控制器
//	printf("Pwm: %d\r\n", Pwm);
	prev_error = error;	                   			// Store previous bias
	return updatedPWM;                         			// Return PID PWM value
}


//#define DC_BRUSHED_MOTOR1_AIN1	PB13
//#define DC_BRUSHED_MOTOR1_AIN2	PB12
//#define DC_BRUSHED_MOTOR2_BIN1	PB14
//#define DC_BRUSHED_MOTOR2_BIN2	PB15

bool activateMotors(int pitchAngle)
{
	bool isMotorEnabled = true;
	
//	printf("pitchAn7gle: %d\r\n", pitchAngle);

	/* If external button is not pressed after powered up (i.e. balance mode is not turned on), 
	 * pitchAngle is greater or less than 65 and -65, 
	 * motor is not activated.
	 *
	 * then update the isMotorEnabled to false (i.e. deactivate the motors), otherwise, activate the motors
	 */
	if ((stopFlag == true) || (isMotorActivated == false)) {
//	if ((stopFlag == true) || (pitchAngle < -90) || (pitchAngle > 90) || (isMotorActivated == false)) {
//	if ((stopFlag == true) || (pitchAngle < -65) || (pitchAngle > 65) || (isMotorActivated == false)) {
//		printf("motor stop!\r\n");
		if (stopFlag == true)
			LED3_ON;
		
		if ((pitchAngle < -90) || (pitchAngle > 90))
			LED5_ON;
		
		isMotorEnabled = false;			// deactivate motors
	} else {
		LED3_OFF;
		isMotorEnabled = true;			// activate motors
	}
	
	return isMotorEnabled;
}

void deactivateMotors(void)
{
	IO_t l_AIN1 = IOGetByTag(DCBrushedMotorConfig()->AIN1);		// PE3
	IO_t l_AIN2 = IOGetByTag(DCBrushedMotorConfig()->AIN2);		// PE4
	IO_t l_BIN1 = IOGetByTag(DCBrushedMotorConfig()->BIN1);		// PC13
	IO_t l_BIN2 = IOGetByTag(DCBrushedMotorConfig()->BIN2);		// PC15	

	/* Disable two motors */
	IOLo(l_AIN1);
	IOLo(l_AIN2);
	IOLo(l_BIN1);
	IOLo(l_BIN2);
}

void updateMotorPwm(int *motorPwm1, int *motorPwm2)
{
//	volatile uint32_t *motor2PwmAddr = (volatile uint32_t *)((volatile char *)&TIM1->CCR1 + 0x4);
	IO_t l_AIN1 = IOGetByTag(DCBrushedMotorConfig()->AIN1);		// PE3
	IO_t l_AIN2 = IOGetByTag(DCBrushedMotorConfig()->AIN2);		// PE4
	IO_t l_BIN1 = IOGetByTag(DCBrushedMotorConfig()->BIN1);		// PC13
	IO_t l_BIN2 = IOGetByTag(DCBrushedMotorConfig()->BIN2);		// PC15

//	printf("motor pwm: %d\r\n", motor);
	if (*motorPwm1 > 0) {
//		printf("*motorPwm1: %d, %d\r\n", *motorPwm1, __LINE__);
//		IOWrite(DCBrushedMotorConfig()->AIN1, true);			// clear AIN1 to LOW
//		IOWrite(DCBrushedMotorConfig()->AIN2, false);			// set AIN2 to HIGH
//		GPIOB->BSRR |= 1<<29;				// set AIN1 to LOW
//		GPIOB->BSRR |= 1<<12;				// set AIN2 to HIGH
#if 0
		IOHi(l_AIN2);
		IOLo(l_AIN1);
#else
		IOHi(l_AIN1);
		IOLo(l_AIN2);
#endif
	} else {
//		printf("*motorPwm1: %d, %d\r\n", *motorPwm1, __LINE__);
//		GPIOB->BSRR |= 1<<13;				// set AIN1 to HIGH
//		GPIOB->BSRR |= 1<<28;				// set AIN2 to LOW
//		IOWrite(DCBrushedMotorConfig()->AIN1, false);			// set AIN1 to HIGH
//		IOWrite(DCBrushedMotorConfig()->AIN2, true);			// clear AIN2 to LOW
#if 0
		IOHi(l_AIN1);
		IOLo(l_AIN2);
#else
		IOHi(l_AIN2);
		IOLo(l_AIN1);
#endif
	}
	
	pwmWriteDcBrushedMotor(0, ABS(*motorPwm1));	// 0 represents motor 1, write motor pwm value to motor 1 (PWMA)
	
	if (*motorPwm2 > 0) {
//		IOLo(DCBrushedMotorConfig()->BIN2);
//		IOHi(DCBrushedMotorConfig()->BIN1);
//		printf("*motorPwm2: %d, %d\r\n", *motorPwm2, __LINE__);
		IOHi(l_BIN1);
		IOLo(l_BIN2);
	} else {
//		printf("*motorPwm2: %d, %d\r\n", *motorPwm2, __LINE__);
//		IOLo(DCBrushedMotorConfig()->BIN1);
//		IOHi(DCBrushedMotorConfig()->BIN2);
		IOHi(l_BIN2);
		IOLo(l_BIN1);
	}
		
	pwmWriteDcBrushedMotor(1, ABS(*motorPwm2));	// 1 represents motor 2, write motor pwm value to motor 12 (PWMB)
}

//int Target_velocity = 2;
//int PWMMotor1 = -1000;                 // Motor PWM value
//int PWMMotor2 = -1000;                 // Motor PWM value

static int stabilisationControlSBWMR(int pitchAngle, float gyroY)
{
	int errorAngle;
	
	/* For Dagu Wild Thumper Wheels */
//	int Kp = 195;				// 195 * 0.6 = 117
//	float Kd = 27.0f;			// 55 * 0.6 = 33

	/* For small wheels */
	int Kp = 320;				// 500 * 0.7 = 350
	float Kd = 31.8;			// 55 * 0.6 = 33
	float stabilisePwm;
	
//	errorAngle = pitchAngle - 4;
	errorAngle = pitchAngle - balanceSetpoint;
	
	stabilisePwm = Kp * errorAngle + Kd * gyroY;
	
	return (int)stabilisePwm;
}

static float cutoffFreq = 0.72f;

static int velocityControlSBWMR(int leftEncoder, int rightEncoder)
{
	static float velocityPwm, encoderError, encoder, encoderIntegral;
//	float Kp = 144.75;		// leftEncoder + rightEncoder with dividing by 2 (using TOP quadcopter landing plate)
//	float Kp = 102.75;		// leftEncoder + rightEncoder with dividing by 2 (w/o using TOP quadcopter landing plate)
//	float Kp = 58.0;		// leftEncoder + rightEncoder w/o dividing by 2
//	float Kp = 75.0;		// leftEncoder + rightEncoder w/o dividing by 2
//	float Kp = 80.0;		// leftEncoder + rightEncoder w/o dividing by 2
//	float Kp = 58.75;		// leftEncoder + rightEncoder w/o dividing by 2
//	float Ki = 0.5;
//	float Ki = 0.33;

	/* For Dagu Wild Thumper Wheels */
//	float Kp = 190.0f;
//	float Ki = 0.95f;
//	float Kp = 198.0f;				// 2nd try
//	float Ki = 1.01f;
//	float Kp = 202.0f;				// 1st try
//	float Ki = Kp / 200;
	
	/* For small wheels */
	float Kp = 122.0;		// leftEncoder + rightEncoder with dividing by 2 (w/o using TOP quadcopter landing plate)
	float Ki = Kp / 200;
	
	if (driveForward == 1 && driveReverse == 0 && turnLeft == 0 && turnRight == 0) {
//		printf("forward: %d, %d\r\n", leftEncoder, rightEncoder);
		velocityUpdatedMovement = -speedConfig;			// negative value representing moving forward
//		velocityUpdatedMovement = -50.0;			// negative value representing moving forward
//		stationaryFlag = 0;

#if 0		
		if (ultrasound2DistanceData < 80) {		// 20 cm
//			speedLimit = 3000;
			velocityUpdatedMovement = -60.0;
		}
#endif

#if 0
		if (ultrasound2DistanceData < 60) {
//			speedLimit = 12000;
			velocityUpdatedMovement = -40.0;
		}
#endif

#if 0		
		if (ultrasound2DistanceData < 50) {
//			speedLimit = 12000;
			velocityUpdatedMovement = -20.0;
		}
#endif
		
		if (ultrasound2DistanceData < 60) {
//			speedLimit = 12000;
			velocityUpdatedMovement = -20.0;
		}
		
		if (ultrasound2DistanceData < 40) {
//			speedLimit = 12000;
			velocityUpdatedMovement = 0.0;
		}
		
	} else if (driveForward == 0 && driveReverse == 1 && turnLeft == 0 && turnRight == 0) {
//		printf("reverse: %d, %d\r\n", leftEncoder, rightEncoder);
		velocityUpdatedMovement = speedConfig;			// positive value representing moving backward
//		velocityUpdatedMovement = 50.0;			// positive value representing moving backward
//		stationaryFlag = 0;

		if (ultrasound5DistanceData < 60) {
//			speedLimit = 12000;
			velocityUpdatedMovement = 20.0;
		}
		
		if (ultrasound5DistanceData < 40) {
//			speedLimit = 12000;
			velocityUpdatedMovement = 0.0;
		}
		
	} else if (driveForward == 0 && driveReverse == 0 && turnLeft == 0 && turnRight == 0) {
		velocityUpdatedMovement = 0;
	}
	
//	printf("movement: %f\r\n", movement);
	
//	encoderError = (leftEncoder + rightEncoder) - velocitySetpoint;		// velocitySetpoint is set to 0
	encoderError = 0.5f * (leftEncoder + rightEncoder) - velocitySetpoint;		// velocitySetpoint is set to 0
	
	/* Low pass filter */
	encoder *= cutoffFreq;
	encoder += encoderError * (1.0f - cutoffFreq);

//	encoder = speedLpfFilterApplyFn(speedLpfFilter, encoderError);
	
	encoderIntegral += encoder;					// Get displacement of motor rotation by integrating encoder speed using 10ms period
	
	encoderIntegral = encoderIntegral - velocityUpdatedMovement;
	
//	printf("encoderIntegral: %.4f\r\n", encoderIntegral);
//	printf("%d\t\t%d\t\t%.4f\r\n", leftEncoder, rightEncoder, encoderIntegral);
	
	/* 7000 */
	if (encoderIntegral > speedLimit) {
		encoderIntegral = speedLimit;
	}
	
	if (encoderIntegral < -speedLimit) {
		encoderIntegral = -speedLimit;
	}
	
	velocityPwm = Kp * encoder + Ki * encoderIntegral;
	
	if (!activateMotors(attitude.raw[Y])) {
		encoderIntegral = 0;			// clear integral error when motor is deactivated
	}
	
	return velocityPwm;
}

#if 1
static int yawControlSBWMR(float gyroZ, int leftEncoder, int rightEncoder)
{
	static float yawError, yawPwm, encoderTmp1, yawCnt;
	static float yawAdjust = 0.9f;

	/* For Dagu Wild Thumper Wheels */
//	float Kp = 35.0f;
//	float Kd = 4.0f;
//	float Kd = 8.0f;

	/* For small wheels */
	float Kp = 57.0f;
	float Kd = 8.4f;

	if (turnLeft == 1 || turnRight == 1) {

//		if (turnLeft == 1) {
//			printf("left: %d, %d\r\n", leftEncoder, rightEncoder);
//		}else if (turnRight == 1) {
//			printf("right: %d, %d\r\n", leftEncoder, rightEncoder);
//		}
		
		if(++yawCnt == 1) {
			encoderTmp1 = ABS(leftEncoder + rightEncoder);
//			printf("encoderTmp1: %.4f\r\n", encoderTmp1);
			yawAdjust = 50 / encoderTmp1;
//			yawAdjust = encoderTmp2 / encoderTmp1;
		}
		
//		printf("yawAdjust: %.4f\r\n", yawAdjust);

#if 1
		if (yawAdjust < 0.8f) {
//		if (yawAdjust < 0.6) {
			yawAdjust = 0.8f;
		}
		
		if (yawAdjust > 2) {
//		if (yawAdjust > 3) {
			yawAdjust = 3;
		}
#endif	

		stationaryFlag = 0;
	} else {
//		yawAdjust = 0.0f;
		yawAdjust = 0.0f;
//		yawError = 0.0f;
		yawCnt = 0;
		encoderTmp1 = 0;
	}

	if (1 == turnLeft) {
		yawError -= yawAdjust;
		stationaryFlag = 0;
	} else if (1 == turnRight) {
		yawError += yawAdjust;
		stationaryFlag = 0;
	} else {		
		yawError = 0;
	}

    if (yawError > yawMagnitude) {
		yawError = yawMagnitude;
	}

	if (yawError < -yawMagnitude) {
		yawError = -yawMagnitude;
	}

//	if (driveForward == 1 || driveReverse == 1) {
//		Kd = 0.5;
//	} else {
//		Kd = 0;
//	}

//	yawPwm = yawError * Kp + gyroZ * Kd;
	yawPwm = Kp * yawError + Kd * gyroZ;

	return yawPwm;
}
#else
static int yawControlSBWMR(float gyroZ, int leftEncoder, int rightEncoder)
{
	float yawPwm, yawError;
	
	float Kp = 1.0f;
	
	yawError = gyroZ - 0;
	yawPwm = -yawError * Kp;
	
	return yawPwm;
}
#endif

bool liftUpSBWMR(float accZ, int16_t pitchAngle, int32_t leftEncoder, int32_t rightEncoder)
{
	static uint16_t condition, temp0, temp1, temp2;
	
	/* Condition 0: SBWMR is stationary */
	if (condition == 0) {
		if (ABS(leftEncoder) + ABS(rightEncoder) < 30) {
			temp0++;
		} else {
			temp0 = 0;
		}
		
		if (temp0 > 10) {
//			printf("condition0-1\r\n");
			condition = 1;
			temp0 = 0;
		}
	}

	/* SBWMR is lifted up at approximately ZERO degree of pitch angle */
	if (condition == 1) {
		/* Time out (200 * 10ms = 2000 ms = 2 secs), no action */
		if (++temp1 > 500) {
			condition = 0;
			temp1 = 0;
		}
		
		/* Lift up rapidly and closed to ZERO degree of pitch angle
		 *
		 * 5000 is based on the current setup of accelerometer. the z-axis of acc is roughly 3358,
		 * when lift up rapidly, the value is changed to above 5000
		 */
		if (accZ > 6000 && (pitchAngle > (-20 + balanceSetpoint) && pitchAngle < (20 + balanceSetpoint))) {
//			printf("condition1-2\r\n");
			condition = 2;
		}
	}
	
	/* SBWMR's motors are spinning forever due to the positive feedback */
	if (condition == 2) {
		/* Time out (100 * 10ms = 1000ms = 1 sec), no action */
		if (++temp2 > 500) {
			condition = 0;
			temp2 = 0;
		}
		
		if (ABS(leftEncoder + rightEncoder) > 135) {
//			printf("condition2\r\n");
			condition = 0;
			return true;					// deactivate motors
		}
	}
	
	return false;							// activate motors
}

bool layDownSBWMR(int16_t pitchAngle, int32_t leftEncoder, int32_t rightEncoder)
{

	static uint16_t condition, temp;
	
	if (isMotorActivated) {
		return false;
	}
	
	
	if (condition == 0) {
		/* SBWMR's pitch angle is within -10 to 10 degrees and motors are not spinning (encoders are all ZEROs) */
		if ((pitchAngle > (-10 + balanceSetpoint) && pitchAngle < (10 + balanceSetpoint)) && leftEncoder == 0 && rightEncoder == 0) {
			condition = 1;
		}
	}
	
	if (condition == 1) {
		/* Time out 500 * 10ms = 5000 ms = 5 secs */
		if (++temp > 500) {
			condition = 0;
			temp = 0;
		}
		
		/* SBWMR is pushed to slide */
		if (((leftEncoder > 3 && leftEncoder < 60) && (rightEncoder > 3 && rightEncoder < 60)) || 
			((leftEncoder < -3 && leftEncoder > -60) && (rightEncoder < -3 && rightEncoder > -60))) {
			condition = 0;
			return true;
				
		}
	}
	
	return false;
}

static void taskMotorEncoder(timeUs_t currentTimeUs)
{	
//	printf("currentTimeUs: %u\r\n", currentTimeUs);
	
	Encoder1 = Read_Encoder(2);		// 2: TIM2, left encoder
	Encoder2 = Read_Encoder(4);		// 4: TIM4, right encoder
	
//	printf("%d, %d\r\n", Encoder1, Encoder2);
	
	LED6_ON;
	
	/* gyroUpdate */
//	gyroUpdate(currentTimeUs);			// gyro updated in taskUpdateGyro function

#if 0
    if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_SPI && gyro.dev.calibrationFlag) {
//        printf("%u,%.4f,%.4f,%.4f\r\n", currentTimeUs, gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z]);
//        printf("%.4f\t%.4f\t%.4f\t%d\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z], attitude.raw[Y]);
//        printf("%.4f\t%.4f\t%d\r\n", gyro.gyroADCf[Y], gyro.gyroADCf[Z], attitude.raw[Y]);
//        printf("%d\r\n", attitude.raw[Y]);
//        printf("%d\r\n", acc.accSmooth[Z]);
    }
#endif
	
	/* Check if balance or obstacle avoidance modes are activated via external button */
	buttonModeSwitchPollOps(ButtonModeSwitchConfig());
	
//	printf("forward: %u\t back: %u\t left: %u\t right: %u\r\n", driveForward, driveReverse, turnLeft, turnRight);
	
	if ((gyro.dev.mpuDetectionResult.sensor == MPU_9250_SPI) && gyro.dev.calibrationFlag) {
		
//        printf("%.4f\t%.4f\t%d\r\n", gyro.gyroADCf[Y], gyro.gyroADCf[Z], attitude.raw[Y]);
		
		/* Balance control */
		stabilisePwmVal = stabilisationControlSBWMR(attitude.raw[Y], gyro.gyroADCf[Y]);

		/* Velocity control */
		velocityPwmVal = velocityControlSBWMR(Encoder1, Encoder2);
		
//		printf("velocityPwmVal: %d\r\n", velocityPwmVal);
		
		/* Yaw control */
		yawPwmVal = yawControlSBWMR(gyro.gyroADCf[Z], Encoder1, Encoder2);
		
		motor1Pwm = stabilisePwmVal - velocityPwmVal + yawPwmVal;
		motor2Pwm = stabilisePwmVal - velocityPwmVal - yawPwmVal;
//		motor1Pwm = -1000;
//		motor2Pwm = -1000;
		
//		PWMMotor1 = Incremental_PIController(Encoder1, Target_velocity);		// Motor 2
//		PWMMotor2 = Incremental_PIController(Encoder2, Target_velocity);		// Motor 1
//		printf("Motor: %d\r\n", Motor1);

		limitMotorPwm(&motor1Pwm, &motor2Pwm);

		/* Check if the SBWMR is lifted up or not, if so, update the isMotorActivated to false (deactivate the motors) */
		if (liftUpSBWMR(acc.accSmooth[Z], attitude.raw[Y], Encoder1, Encoder2)) {
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			isMotorActivated = false;				// deactivate motors
		}
		
		/* Check if the SBWMR is put down or not, if so, update the isMotorActivated to true (activate the motors) */
		if (layDownSBWMR(attitude.raw[Y], Encoder1, Encoder2)) {
			isMotorActivated = true;
		}
		
		if (activateMotors(attitude.raw[Y])) {
			updateMotorPwm(&motor1Pwm, &motor2Pwm);
		} else {
			deactivateMotors();
		}
	}
	
//	printf("%d\t\t%d\t\t%d\t\t%d\r\n", Encoder1, motor1Pwm, Encoder2, motor2Pwm);
	
//	printf("(LeftEncoder, LeftMotor_PWM) = (%d, %d)\r\n", Encoder1, motor1Pwm);
//	printf("(RightEncoder, RightMotor_PWM) = (%d, %d)\r\n", Encoder2, motor2Pwm);
}

static void taskUltrasound1ReadData(timeUs_t currentTimeUs)
{	
	ultrasound1DistanceData = ultrasound1Read();
	
//	printf("d1: %d\r\n", ultrasound1DistanceData);
}

static void taskUltrasound2ReadData(timeUs_t currentTimeUs)
{	
	ultrasound2DistanceData = ultrasound2Read();
	
//	printf("d2: %d\r\n", ultrasound2DistanceData);
}

static void taskUltrasound3ReadData(timeUs_t currentTimeUs)
{	
	ultrasound3DistanceData = ultrasound3Read();
	
//	printf("d3: %d\r\n", ultrasound3DistanceData);
}

static void taskUltrasound4ReadData(timeUs_t currentTimeUs)
{	
	ultrasound4DistanceData = ultrasound4Read();
	
//	printf("d4: %d\r\n", ultrasound4DistanceData);
}

static void taskUltrasound5ReadData(timeUs_t currentTimeUs)
{	
	ultrasound5DistanceData = ultrasound5Read();
	
//	printf("d5: %d\r\n", ultrasound5DistanceData);
}

static void taskUltrasound6ReadData(timeUs_t currentTimeUs)
{	
	ultrasound6DistanceData = ultrasound6Read();
	
//	printf("d6: %d\r\n", ultrasound6DistanceData);
}

static char dataLog[500];

static void taskOLEDDisplay(timeUs_t currentTimeUs)
{
	static bool switchFromOADisplayFlag = false;

//	UNUSED(currentTimeUs);
	
	/* +----------- Display SBWMR modes (Normal and Obstacle Avoidance) -------------+ */
//	OLED_ShowString(0, 0, "M: ");
	
	if (isCollisionAvoidanceModeActivated == true) {
		OLED_ShowString(0, 0, "  OA   ");
		switchFromOADisplayFlag = true;
	} else {
//		printf("flag: %d\r\n", switchFromOADisplayFlag);
		if (switchFromOADisplayFlag == true) {
//			OLED_ShowString(30, 0, "      ");
//			OLED_ShowString(80, 0, "     ");
			switchFromOADisplayFlag = false;
		} else {
			OLED_ShowString(00, 0, " Normal");
		}
	}
	
	/* +------------------- Display temperature value -------------------+ */
//	printf("temp: %.4f\r\n", temperatureData);
//	OLED_ShowString(00, 10, "Temp: ");
//	OLED_ShowNumber(46, 10, (int)temperatureData, 2, 12);		// display temperature integer part
//	OLED_ShowString(59, 10, ".");
//	OLED_ShowNumber(68, 10, (int)((round(temperatureData * 100) / 100 - (int)temperatureData) * 100), 2, 12);		// display temperature integer part
//	OLED_ShowString(85, 10, "`C");

//	OLED_ShowString(00, 00, "T: ");
	OLED_ShowNumber(66, 00, (int)temperatureData, 2, 12);		// display temperature integer part
	OLED_ShowString(79, 00, ".");
	OLED_ShowNumber(88, 00, (int)((round(temperatureData * 100) / 100 - (int)temperatureData) * 100), 2, 12);		// display temperature integer part
	OLED_ShowString(105, 00, "`C");

	/* +---------------------------- Display Pitch angle -----------------------------+ */
	OLED_ShowString(0, 10, "P/Y: ");
	
	/* Display Euler Pitch angle */
	if (attitude.raw[Y] < 0) {
		OLED_ShowString(40, 10, "-");
		OLED_ShowNumber(45, 10, -attitude.raw[Y], 3, 12);
//		OLED_ShowString(80, 10, "-");
//		OLED_ShowNumber(95, 10, -attitude.raw[Y], 3, 12);
//		OLED_ShowNumber(45, 10, attitude.raw[Y] + 360, 3, 12);
	} else {
		OLED_ShowString(40, 10, "+");
		OLED_ShowNumber(45, 10, attitude.raw[Y], 3, 12);		
//		OLED_ShowString(80, 10, "+");
//		OLED_ShowNumber(95, 10, attitude.raw[Y], 3, 12);		
//		OLED_ShowNumber(45, 10, attitude.raw[Y], 3, 12);
	}

	/* Display Euler Yaw angle */
	if (attitude.raw[Z] < 0) {
//		OLED_ShowString(80, 10, "-");
		OLED_ShowNumber(85, 10, attitude.raw[Z] + 360, 3, 12);
	} else {
//		OLED_ShowString(80, 10, "+");
		OLED_ShowNumber(85, 10, attitude.raw[Z], 3, 12);		
	}
		
	/* +------------------- Display encoder1 (left encoder) value -------------------+ */
	OLED_ShowString(00, 20, "LeftEnco: ");
	
	if (Encoder1 < 0) {
		OLED_ShowString(80, 20, "-");
		OLED_ShowNumber(95, 20, -Encoder1, 3, 12);
	} else {
		OLED_ShowString(80, 20, "+");
		OLED_ShowNumber(95, 20, Encoder1, 3, 12);
	}
	
	/* +------------------- Display encoder2 (right encoder) value -------------------+ */
	OLED_ShowString(00, 30, "RightEnco: ");
	
	if (Encoder2 < 0) {
		OLED_ShowString(80, 30, "-");
		OLED_ShowNumber(95, 30, -Encoder2, 3, 12);
	} else {
		OLED_ShowString(80, 30, "+");
		OLED_ShowNumber(95, 30, Encoder2, 3, 12);
	}

	/* +---------------------------- Display ultrasound data -----------------------------+ */
//	if (ultrasound1DistanceData < 0) {
//		OLED_ShowString(00, 50, "-");
//	} else {
//		OLED_ShowString(00, 50, "");
//	}
	
	/* Display ultrasound sensor data 1 */
	OLED_ShowNumber(10, 40, ultrasound1DistanceData, 3, 12);

	/* Display ultrasound sensor data 2 */
	OLED_ShowNumber(55, 40, ultrasound2DistanceData, 3, 12);

	/* Display ultrasound sensor data 3 */
	OLED_ShowNumber(95, 40, ultrasound3DistanceData, 3, 12);

	/* Display ultrasound sensor data 4 */
	OLED_ShowNumber(10, 50, ultrasound4DistanceData, 3, 12);

	/* Display ultrasound sensor data 5 */
	OLED_ShowNumber(55, 50, ultrasound5DistanceData, 3, 12);

	/* Display ultrasound sensor data 6 */
	OLED_ShowNumber(95, 50, ultrasound6DistanceData, 3, 12);

#if 1
	sprintf(dataLog, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", (float)ultrasound1DistanceData, (float)ultrasound2DistanceData, (float)ultrasound3DistanceData, (float)ultrasound4DistanceData, (float)ultrasound5DistanceData, (float)ultrasound6DistanceData, (float)Encoder1, (float)motor1Pwm, (float)Encoder2, (float)motor2Pwm, gyro.gyroADCf[Y], gyro.gyroADCf[Z], (float)attitude.raw[Y], (float)attitude.raw[Z]);
	printf("%s\r\n", dataLog);
//	printf("%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%d\r\n", currentTimeUs, ultrasound1DistanceData, ultrasound2DistanceData, ultrasound3DistanceData, 
//					ultrasound4DistanceData, ultrasound5DistanceData, ultrasound6DistanceData, Encoder1, motor1Pwm, Encoder2, motor2Pwm, gyro.gyroADCf[Y], 
//					gyro.gyroADCf[Z], attitude.raw[Y]);
#else
//	printf("%d,%d,%d\r\n", attitude.raw[X], attitude.raw[Y], attitude.raw[Z]);
//	printf("%.4f,%.4f,%.4f,%.4f\r\n", gyro.gyroADCf[Y], gyro.gyroADCf[Z], (float)attitude.raw[Y], (float)attitude.raw[Z]);
//	printf("%.4f,%.4f,%.4f,%.4f\r\n", (float)ultrasound1DistanceData, (float)ultrasound2DistanceData, (float)ultrasound3DistanceData, (float)ultrasound4DistanceData);
//	sprintf(dataLog, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", (float)ultrasound1DistanceData, (float)ultrasound2DistanceData, (float)ultrasound3DistanceData, (float)ultrasound4DistanceData, (float)ultrasound5DistanceData, (float)ultrasound6DistanceData);
//	sprintf(dataLog, "%.4f,%.4f,%.4f,%.4f", (float)ultrasound1DistanceData, (float)ultrasound2DistanceData, (float)ultrasound3DistanceData, (float)ultrasound4DistanceData);
	sprintf(dataLog, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", gyro.gyroADCf[Y], gyro.gyroADCf[Z], (float)attitude.raw[Y], (float)attitude.raw[Z], (float)ultrasound1DistanceData, (float)ultrasound2DistanceData, (float)ultrasound3DistanceData, (float)ultrasound4DistanceData, (float)ultrasound5DistanceData, (float)ultrasound6DistanceData);
	printf("%s\r\n", dataLog);
#endif

	OLED_Refresh_Gram();
}

static void taskDataLogger(timeUs_t currentTimeUs)
{
//	printf("currentTimeUs: %u\r\n", currentTimeUs);
	
#ifdef USE_SDCARD
	afatfs_poll();
#endif
	
#ifdef BLACKBOX
	if (/*!cliMode && */feature(FEATURE_BLACKBOX)) {
//		printf("currentTimeUs: %u, %d\r\n", currentTimeUs, __LINE__);
		handleBlackbox(currentTimeUs);
	}
#endif
}

void fcTasksInit(void)
{
    /* Clear RTOS queue and enable SYSTEM TASK */
    schedulerInit();

	/* Enable GYRO Task */
    setTaskEnabled(TASK_GYRO, true);

//	printf("ACC on: %d\r\n", sensors(SENSOR_ACC));						// sensors(SENSOR_ACC) = 1
//	printf("accSamplingInterval: %u\r\n", acc.accSamplingInterval);		// acc.accSamplingInterval = 1000
	
	/* Enable ACCELEROMETER TASK */
	if (sensors(SENSOR_ACC)) {
		setTaskEnabled(TASK_ACCEL, true);
		rescheduleTask(TASK_ACCEL, acc.accSamplingInterval);
	}
	
	/* Enable MOTOR ENCODER TESTING Task */
	setTaskEnabled(TASK_MOTORENCODER, true);
		
	/* Enable ATTITUDE TASK (Data fusion of Euler angles (Roll, Pitch, and Yaw angles)) */
	setTaskEnabled(TASK_ATTITUDE, sensors(SENSOR_ACC));

	/* Enable OLED display TASK */
	setTaskEnabled(TASK_OLEDDISPLAY, true);

	
	/* +----------------------------------------------------------------------------------------+ */
	/* +------------------------------------- Sonar Tasks --------------------------------------+ */
	/* +----------------------------------------------------------------------------------------+ */
	
	/* Enable ultrasound1 update TASK (Repeated sending startup sequence) */
	setTaskEnabled(TASK_ULTRASOUND1_UPDATE, true);

	/* Enable ultrasound1 data collection TASK */
	setTaskEnabled(TASK_ULTRASOUND1_READDATA, true);
	
	/* Enable ultrasound2 update TASK (Repeated sending startup sequence) */
	setTaskEnabled(TASK_ULTRASOUND2_UPDATE, true);

	/* Enable ultrasound2 data collection TASK */
	setTaskEnabled(TASK_ULTRASOUND2_READDATA, true);

	/* Enable ultrasound3 update TASK (Repeated sending startup sequence) */
	setTaskEnabled(TASK_ULTRASOUND3_UPDATE, true);

	/* Enable ultrasound3 data collection TASK */
	setTaskEnabled(TASK_ULTRASOUND3_READDATA, true);

	/* Enable ultrasound4 update TASK (Repeated sending startup sequence) */
	setTaskEnabled(TASK_ULTRASOUND4_UPDATE, true);

	/* Enable ultrasound4 data collection TASK */
	setTaskEnabled(TASK_ULTRASOUND4_READDATA, true);

	/* Enable ultrasound5 update TASK (Repeated sending startup sequence) */
	setTaskEnabled(TASK_ULTRASOUND5_UPDATE, true);

	/* Enable ultrasound5 data collection TASK */
	setTaskEnabled(TASK_ULTRASOUND5_READDATA, true);

	/* Enable ultrasound6 update TASK (Repeated sending startup sequence) */
	setTaskEnabled(TASK_ULTRASOUND6_UPDATE, true);

	/* Enable ultrasound6 data collection TASK */
	setTaskEnabled(TASK_ULTRASOUND6_READDATA, true);

	/* +----------------------------------------------------------------------------------------+ */
	/* +------------------------------------- Sonar Tasks --------------------------------------+ */
	/* +----------------------------------------------------------------------------------------+ */


	/* +----------------------------------------------------------------------------------------+ */
	/* +-------------------------------- Data logger Task --------------------------------------+ */
	/* +----------------------------------------------------------------------------------------+ */

	/* Enable data logger TASK */
	setTaskEnabled(TASK_DATALOGGER, true);

	/* +----------------------------------------------------------------------------------------+ */
	/* +-------------------------------- Data logger Task --------------------------------------+ */
	/* +----------------------------------------------------------------------------------------+ */
}
