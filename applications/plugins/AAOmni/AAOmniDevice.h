//Author- Abhay Gupta UF grad Summer 2017
//For any queries contact - abhayg271@gmail.com
#ifndef _AAOMNI_DEVICE_H
#define _AAOMNI_DEVICE_H
#include <libusb.h>
#include "AAOmniDefs.h"
#include "Matrix.h"
#include <cstdio>
#include <cmath>
 
#define pi 3.1415926


#define ALPHA_FILTERING		0.4
#define AAOMNI_Y_OFFSET		50.0
#define AAOMNI_Z_OFFSET		-150.0
#define AAOMNI_MM_TO_M		0.001

typedef struct AAOmniDevice
{
	libusb_device_handle* libUsbDeviceHandle;
	omni_in inData;
	omni_out outData;
	double alphaFiltering;
	double baseAngles[3];
	double gimbalAngles[3];
	double gimbalAnglesFiltered[3];
	double forceFeedback[3];
	char deviceName[10];
	double transformationMat[4][4];
	uint8_t newData;
}AAOmniDevice;

typedef struct AAOmniContext
{
	libusb_context *libUsbCtx;
	libusb_device **libUsbDevs;
	int* libUsbDeviceOpen;
	AAOmniDevice* aaOmniDevices;
	int numLibUsbDevices;
	int numAAOmniDevices;
}AAOmniContext;


AAOmniContext* AAOmniCreateContext();
int AAOmniGetDevices(AAOmniContext* ctx,int numDevices);
void AAOmniDeleteContext(AAOmniContext* ctx);
void AAOmniGetTransformationMatrix(AAOmniDevice* dev,double ret[4][4]);
void AAOmniGetNewData(AAOmniDevice* dev);
void AAOmniSetForceFeedback(AAOmniDevice* dev,double* force);
void AAOmniUpdateValues(AAOmniDevice *dev);
void AAOmniRemoveAllForces(AAOmniDevice *dev);

#endif