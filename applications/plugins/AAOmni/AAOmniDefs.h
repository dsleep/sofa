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

#define AAOMNI_THETA1_MAXCOUNTS 	18000//4500*4
#define AAOMNI_THETA2_MAXCOUNTS 	15200//3800*4
#define AAOMNI_THETA3_MAXCOUNTS 	14000//3500*4

#define AAOMNI_THETA1_RANGE 		110//degrees measured with thread
#define AAOMNI_THETA2_RANGE 		90//estimate
#define AAOMNI_THETA3_RANGE 		90//estimate

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