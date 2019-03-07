
#include <stdio.h>
#include <math.h>
#include "fc_tasks.h"
#include "fc_rc.h"
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
#include "rxSerial6Test.h"
#include "system.h"
#include "oled.h"

//#define TASKS_LEDS_TESTING

/* taskMotorEncoder is updated every 10 ms, inactivity is triggered every 10ms * 1000 = 10000 ms = 10 secs */
#define INACTIVITY_CONDITION			1000

#define TASK_PERIOD_HZ(hz)              (1000000 / (hz))            // units in microseconds (us)
#define TASK_PERIOD_MS(ms)              ((ms) * 1000)
#define TASK_PERIOD_US(us)              (us)

int Encoder1, Encoder2;
int stabilisePwmVal, velocityPwmVal, yawPwmVal;
int motor1Pwm, motor2Pwm;
int yawMagnitude = 45;
uint32_t stationaryFlag = 0;
float velocityUpdatedMovement = 0.0f;
float balanceSetpoint = 0.0f;
float velocitySetpoint = 0.0f;
//float yawSetpoint = 0.0f;

bool isMotorActivated = true;

extern uint8_t driveForward, driveReverse, turnLeft, turnRight;

bool stopFlag = true;
bool isCollisionAvoidanceModeActivated = false;			// collision avoidance is switched off by default

//static void taskUpdateRxMain(timeUs_t currentTimeUs);
static void taskUpdateAccelerometer(timeUs_t currentTimeUs);
static void taskMotorEncoder(timeUs_t currentTimeUs);
static void taskUpdateGyro(timeUs_t currentTimeUs);
static void taskOLEDDisplay(timeUs_t currentTimeUs);
//static void taskBluetoothReceive(timeUs_t currentTimeUs);

/* Tasks initialisation */
cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
//        .desiredPeriod = TASK_PERIOD_HZ(0.05),            // 1000000 / 0.05 = 20000000 us = 20000 ms = 20 s
//        .desiredPeriod = TASK_PERIOD_HZ(0.5),            // 1000000 / 0.5 = 2000000 us = 2000 ms = 2 s
//        .desiredPeriod = TASK_PERIOD_HZ(1),            // 1000000 / 1 = 1000000 us = 1000 ms = 1 s
        .desiredPeriod = TASK_PERIOD_HZ(10),            // 1000000 / 10 = 100000 us = 100 ms
        .staticPriority = TASK_PRIORITY_MEDIUM_HIGH,
    },
    
	/* desiredPeriod = 4000 us = 4 ms = 250 Hz for F450 quad */
    [TASK_GYRO] = {
        .taskName = "GYRO",
        .taskFunc = taskUpdateGyro,
//        .desiredPeriod = TASK_PERIOD_HZ(50),                // 1000000 / 50 = 20000 us = 20 ms
        .desiredPeriod = TASK_GYROPID_DESIRED_PERIOD,       // desiredPeriod = TASK_GYROPID_DESIRED_PERIOD = 125 us using STM32F4
        .staticPriority = TASK_PRIORITY_HIGH,           // TASK_PRIORITY_REALTIME = 6
    },
	
	[TASK_ACCEL] = {
		.taskName = "ACCEL",
		.taskFunc = taskUpdateAccelerometer,
		.desiredPeriod = TASK_PERIOD_HZ(1000),				// 1000000 / 1000 = 1000 us, every 1ms
		.staticPriority = TASK_PRIORITY_MEDIUM,				// 3
	},
	
	[TASK_ATTITUDE] = {
		.taskName = "ATTITUDE",
		.taskFunc = taskIMUUpdateAttitude,
		.desiredPeriod = TASK_PERIOD_HZ(100),				// 1000000 / 100 = 10000 us = 10 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,				// 3
	},

    [TASK_MOTORENCODER] = {
        .taskName = "MOTOR_ENCODER",
        .taskFunc = taskMotorEncoder,
        .desiredPeriod = TASK_PERIOD_HZ(100),            // 1000000 / 100 = 10000 us = 10 ms
        .staticPriority = TASK_PRIORITY_REALTIME,
    },
	
	[TASK_OLEDDISPLAY] = {
		.taskName = "OLED_DISPLAY",
		.taskFunc = taskOLEDDisplay,
		.desiredPeriod = TASK_PERIOD_HZ(20),			// 1000000 / 20 = 50000 us = 50 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},
	
//	[TASK_BTRX] = {
//		.taskName = "BLUETOOTH_RX",
//		.taskFunc = taskBluetoothReceive,
//		.desiredPeriod = TASK_PERIOD_HZ(1),			// 1000000 / 20 = 100000 us = 50 ms
//		.staticPriority = TASK_PRIORITY_HIGH,
//	}    
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
	}
	
	/* Case for long pressing */
	if (res == 2) {
		isCollisionAvoidanceModeActivated = !isCollisionAvoidanceModeActivated;
	}
}

//static void taskUpdateRxMain(timeUs_t currentTimeUs)			// TODO: make this function static for rtos tasks assignment
//{
//	/* retrieve RX data */
//	processRx(currentTimeUs);
//	
//	/* new rx data is available */
//	isRXDataNew = true;
//	
//	/* updateRcCommands function sets rcCommand  */
//	updateRcCommands();
//	
//	/* update LEDs */
//	updateLEDs();
//}

/* Encoder speed values */
int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;
	switch(TIMX)
	{
		case 2:
			Encoder_TIM = (short)TIM2->CNT;
			TIM2->CNT = 0;
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
	/* The maximum PWM value is 7200 setup by dcBrushedMotorInit(), we use 6900 for the upper bound */
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
	
//	printf("pitchAngle: %d\r\n", pitchAngle);

	/* If external button is not pressed after powered up (i.e. balance mode is not turned on), 
	 * pitchAngle is greater or less than 50 and -50, 
	 * motor is not activated.
	 *
	 * then update the isMotorEnabled to false (i.e. deactivate the motors), otherwise, activate the motors
	 */
	if ((stopFlag == true) || (pitchAngle < -50) || (pitchAngle > 50) || (isMotorActivated == false)) {
//		printf("motor stop!\r\n");
		isMotorEnabled = false;			// deactivate motors
	} else {
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
	int Kp = 300;				// 500 * 0.6 = 300
	float Kd = 31.8;			// 55 * 0.6 = 33
	float stabilisePwm;
	
//	errorAngle = pitchAngle - 4;
	errorAngle = pitchAngle - balanceSetpoint;
	
	stabilisePwm = Kp * errorAngle + Kd * gyroY;
	
	return (int)stabilisePwm;
}

static int velocityControlSBWMR(int leftEncoder, int rightEncoder)
{
	static float velocityPwm, encoderError, encoder, encoderIntegral;
//	float Kp = 144.75;		// leftEncoder + rightEncoder with dividing by 2 (using TOP quadcopter landing plate)
	float Kp = 116.75;		// leftEncoder + rightEncoder with dividing by 2 (w/o using TOP quadcopter landing plate)
//	float Kp = 85.75;		// leftEncoder + rightEncoder w/o dividing by 2
	float Ki = Kp / 200;
	
	if (driveForward == 1 && driveReverse == 0 && turnLeft == 0 && turnRight == 0) {
//		printf("forward: %d, %d\r\n", leftEncoder, rightEncoder);
		velocityUpdatedMovement = -50.0;			// negative value representing moving forward
		stationaryFlag = 0;
	} else if (driveForward == 0 && driveReverse == 1 && turnLeft == 0 && turnRight == 0) {
//		printf("reverse: %d, %d\r\n", leftEncoder, rightEncoder);
		velocityUpdatedMovement = 50.0;			// positive value representing moving backward
		stationaryFlag = 0;
	} else if (driveForward == 0 && driveReverse == 0 && turnLeft == 0 && turnRight == 0) {
		velocityUpdatedMovement = 0;
	}
	
//	printf("movement: %f\r\n", movement);
	
//	encoderError = (leftEncoder + rightEncoder) - velocitySetpoint;		// velocitySetpoint is set to 0
	encoderError = (leftEncoder + rightEncoder) / 2 - velocitySetpoint;		// velocitySetpoint is set to 0
	
	/* Low pass filter */
	encoder *= 0.7f;
	encoder += encoderError * 0.3f;
	
	encoderIntegral += encoder;					// Get displacement of motor rotation by integrating encoder speed using 10ms period
	
	encoderIntegral = encoderIntegral - velocityUpdatedMovement;
	
	/* 4000 */
	if (encoderIntegral > 5000) {
		encoderIntegral = 5000;
	}
	
	if (encoderIntegral < -5000) {
		encoderIntegral = -5000;
	}
	
	velocityPwm = Kp * encoder + Ki * encoderIntegral;
	
	if (!activateMotors(attitude.raw[Y])) {
		encoderIntegral = 0;			// clear integral error when motor is deactivated
	}
	
	return velocityPwm;
}

static int yawControlSBWMR(float gyroZ, int leftEncoder, int rightEncoder)
{
	static float yawError, yawPwm, encoderTmp1, yawCnt;
	static float yawAdjust = 0.9f;

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
		yawAdjust = 0.9f;
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
	
#if 0
		stationaryFlag++;
//		printf("Inactivate, %d\r\n", stationaryFlag);
		if ((velocityUpdatedMovement == 0) && (stationaryFlag >= INACTIVITY_CONDITION)) {
//			yawAdjust = 18.0f;
//			yawError = yawAdjust;			// counter-clockwise yaw rotation
			yawError = 18;					// counter-clockwise yaw rotation
			yawMagnitude = 20;				// limit the yawing speed
		} else {
			yawError = 0;
			yawMagnitude = 45;
			yawAdjust = 0;
		}
#endif
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

	yawPwm = yawError * Kp + gyroZ * Kd;
//	yawPwm = Kp * yawError + Kd * gyroZ;

	return yawPwm;
}

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
		if (accZ > 5000 && (pitchAngle > (-20 + balanceSetpoint) && pitchAngle < (20 + balanceSetpoint))) {
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

//static void taskBluetoothReceive(timeUs_t currentTimeUs)
//{
//	uint8_t btReceiveData = 0;

//	btReceiveData = btSerial6Read();
////	printf("task_bt_data: %u\r\n", btReceiveData);	
//}

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
	
//	printf("Encoder1: %d\r\n", Encoder1);
//	printf("Encoder2: %d\r\n", Encoder2);
//	printf("Motor1 pwm: %d\r\n", PWMMotor1);
//	printf("Motor2 pwm: %d\r\n", PWMMotor2);
}

static void taskOLEDDisplay(timeUs_t currentTimeUs)
{
	static bool switchFromOADisplayFlag = false;
	
	/* +----------- Display SBWMR modes (Normal and Obstacle Avoidance) -------------+ */
//	OLED_ShowString(0, 0, "M: ");
	
	if (isCollisionAvoidanceModeActivated == true) {
		OLED_ShowString(0, 0, "Colli Avoidance");
		switchFromOADisplayFlag = true;
	} else {
//		printf("flag: %d\r\n", switchFromOADisplayFlag);
		if (switchFromOADisplayFlag == true) {
			OLED_ShowString(0, 0, "     ");
			OLED_ShowString(80, 0, "     ");
			switchFromOADisplayFlag = false;
		} else {
			OLED_ShowString(40, 0, "Normal");
		}
	}
	
	/* +------------------- Display temperature value -------------------+ */
//	printf("temp: %.4f\r\n", temperatureData);
	OLED_ShowString(00, 20, "Temp: ");
	OLED_ShowNumber(46, 20, (int)temperatureData, 2, 12);		// display temperature integer part
	OLED_ShowString(59, 20, ".");
	OLED_ShowNumber(68, 20, (int)((round(temperatureData * 100) / 100 - (int)temperatureData) * 100), 2, 12);		// display temperature integer part
	OLED_ShowString(85, 20, "`C");
	
	/* +------------------- Display encoder1 (left encoder) value -------------------+ */
	OLED_ShowString(00, 30, "LeftEnco: ");
	
	if (Encoder1 < 0) {
		OLED_ShowString(80, 30, "-");
		OLED_ShowNumber(95, 30, -Encoder1, 3, 12);
	} else {
		OLED_ShowString(80, 30, "+");
		OLED_ShowNumber(95, 30, Encoder1, 3, 12);
	}
	
	/* +------------------- Display encoder2 (right encoder) value -------------------+ */
	OLED_ShowString(00, 40, "RightEnco: ");
	
	if (Encoder2 < 0) {
		OLED_ShowString(80, 40, "-");
		OLED_ShowNumber(95, 40, -Encoder2, 3, 12);
	} else {
		OLED_ShowString(80, 40, "+");
		OLED_ShowNumber(95, 40, Encoder2, 3, 12);
	}

	/* +---------------------------- Display Pitch angle -----------------------------+ */
	OLED_ShowString(0, 50, "P/Y: ");
	
	/* Display Euler Pitch angle */
	if (attitude.raw[Y] < 0) {
		OLED_ShowString(40, 50, "-");
		OLED_ShowNumber(45, 50, -attitude.raw[Y], 3, 12);
//		OLED_ShowString(80, 50, "-");
//		OLED_ShowNumber(95, 50, -attitude.raw[Y], 3, 12);
//		OLED_ShowNumber(45, 50, attitude.raw[Y] + 360, 3, 12);
	} else {
		OLED_ShowString(40, 50, "+");
		OLED_ShowNumber(45, 50, attitude.raw[Y], 3, 12);		
//		OLED_ShowString(80, 50, "+");
//		OLED_ShowNumber(95, 50, attitude.raw[Y], 3, 12);		
//		OLED_ShowNumber(45, 50, attitude.raw[Y], 3, 12);
	}

	/* Display Euler Yaw angle */
	if (attitude.raw[Z] < 0) {
//		OLED_ShowString(80, 50, "-");
		OLED_ShowNumber(85, 50, attitude.raw[Z] + 360, 3, 12);
	} else {
//		OLED_ShowString(80, 50, "+");
		OLED_ShowNumber(85, 50, attitude.raw[Z], 3, 12);		
	}
	
	OLED_Refresh_Gram();
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
	
	/* Enable BTRX TASK */
//	setTaskEnabled(TASK_BTRX, true);	    
}
