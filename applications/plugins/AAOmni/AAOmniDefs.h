#pragma once
#ifndef AA_OMNI_DEFS
#define AA_OMNI_DEFS

typedef struct omni_out {
	uint16_t mot1;
	uint16_t mot2;
	uint16_t mot3;
} omni_out;


typedef struct omni_in
{
	uint16_t but;
	int16_t mot1;
	int16_t mot2;
	int16_t mot3;
	int16_t pot1;
	int16_t pot2;
	int16_t pot3;
} omni_in;

//motor torque multiplier 32.44
#define THETA1_MIN_COUNT		-9800
#define THETA1_MAX_COUNT		9580
#define THETA1_MIN_DEGREES		-55.0
#define THETA1_MAX_DEGREES		50.0
#define THETA1_OFFSET_DEGREES	-1.9

//motor torque multiplier 29.43
#define THETA2_MIN_COUNT		-2600
#define THETA2_MAX_COUNT		14980
#define THETA2_MIN_DEGREES		0.0
#define THETA2_MAX_DEGREES		-105.0
#define THETA2_OFFSET_DEGREES	-15.52

//motor torque multiplier 32.8
#define THETA3_MIN_COUNT		-22790
#define THETA3_MAX_COUNT		-1500
#define THETA3_MIN_DEGREES		-105.0
#define THETA3_MAX_DEGREES		9.0
#define THETA3_OFFSET_DEGREES	17.03

//the following values are the maxrange of potentionmeter values to be found out
#define AAOMNI_GIMBAL1_MAXCOUNTS 	16000
#define AAOMNI_GIMBAL2_MAXCOUNTS 	10000
#define AAOMNI_GIMBAL3_MAXCOUNTS 	16300

#define AAOMNI_GIMBAL1_RANGE 		292//degrees measured with thread
#define AAOMNI_GIMBAL2_RANGE 		150//degrees
#define AAOMNI_GIMBAL3_RANGE 		316//degrees measured with thread

#define AAOMNI_ARM_LENGTH1 			134
#define AAOMNI_ARM_LENGTH2 			134
#define AAOMNI_TIP_LENGTH			39 //measured with scale

//calibrating position
#define AAOMNI_THETA1_OFFSET 		0//always remain zero
#define AAOMNI_THETA2_OFFSET 		15
#define AAOMNI_THETA3_OFFSET 		22

#define AAOMNI_GIMBAL1_OFFSET 		0//always remain zero
#define AAOMNI_GIMBAL2_OFFSET 		-42
#define AAOMNI_GIMBAL3_OFFSET 		0 //hopefully you have not twisted the tool in docking position


#define VENDOR_ID 					0x0483
#define DEVICE_ID 					0x5710
#define AAOMNI_IN_ENDPOINT_ADDR 	0x82
#define AAOMNI_OUT_ENDPOINT_ADDR 	0x02
#define AAOMNI_BUTTON_HOME 			0x0001
#define AAOMNI_BUTTON1 				0x0002
#define AAOMNI_BUTTON2 				0x0004
#endif