//Author- Abhay Gupta UF grad Summer 2017
//For any queries contact - abhayg271@gmail.com
#include "AAOmniDevice.h"
void populateRotate(Matrix4x4d* in,double angle,int mode)
{
	in->data[3][0]=0;
	in->data[3][1]=0;
	in->data[3][2]=0;
	in->data[3][3]=1;
	in->data[0][3]=0;
	in->data[1][3]=0;
	in->data[2][3]=0;
	double sAngle=sin(angle);
	double cAngle=cos(angle);
	switch(mode)
	{
		case 0://rot x
		in->data[0][0]=1;
        in->data[0][1]=0;
        in->data[0][2]=0;
        in->data[1][0]=0;
        in->data[1][1]=cAngle;
        in->data[1][2]=-sAngle;
        in->data[2][0]=0;
        in->data[2][1]=sAngle;
        in->data[2][2]=cAngle;
        break;
		case 1://rot y
		in->data[0][0]=cAngle;
        in->data[0][1]=0;
        in->data[0][2]=sAngle;
        in->data[1][0]=0;
        in->data[1][1]=1;
        in->data[1][2]=0;
        in->data[2][0]=-sAngle;
        in->data[2][1]=0;
        in->data[2][2]=cAngle;
        break;
		case 2://rot z
		in->data[0][0]=cAngle;
        in->data[0][1]=-sAngle;
        in->data[0][2]=0;
        in->data[1][0]=sAngle;
        in->data[1][1]=cAngle;
        in->data[1][2]=0;
        in->data[2][0]=0;
        in->data[2][1]=0;
        in->data[2][2]=1;
        break;
		default:
		break;
	}
}
void populateTranslate(Matrix4x4d* in,double tx,double ty,double tz)
{
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			if(i==j)
				in->data[i][j]=1;
			else
				in->data[i][j]=0;
		}
	}
	in->data[0][3]=tx;
	in->data[1][3]=ty;
	in->data[2][3]=tz;
}
void initMat(Matrix4x4d* in)
{
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
			in->data[i][j]=0;
}

int getLibUsbErrorString(const char * str, int no)
{
	if (no<0)
		//cout << str << ":" << libusb_error_name(no) << endl;
		printf("%s : %s\n",str,libusb_error_name(no));
	return no;
}
AAOmniContext* AAOmniCreateContext()
{
	ssize_t cnt;
	AAOmniContext* ctx=(AAOmniContext*)calloc(sizeof(AAOmniContext),1);
	if(getLibUsbErrorString("libusb_init",libusb_init(&ctx->libUsbCtx))) 
		goto deleteContext;
	cnt = getLibUsbErrorString("libusb_get_device_list",libusb_get_device_list(ctx->libUsbCtx, &ctx->libUsbDevs)); //get the list of devices
    if (cnt < 0) 
    	goto deleteContext;
    ctx->libUsbDeviceOpen=(int*)calloc(cnt,sizeof(int));
    ctx->numLibUsbDevices=cnt;
    return ctx;
	deleteContext:
	free(ctx->libUsbDeviceOpen);
	free(ctx);
	return NULL;
}
int AAOmniGetDevices(AAOmniContext* ctx,int numDevices)
{
	ctx->aaOmniDevices=(AAOmniDevice*)calloc(sizeof(AAOmniDevice),numDevices);
	ctx->numAAOmniDevices=numDevices;

	libusb_set_debug(ctx->libUsbCtx, LIBUSB_LOG_LEVEL_WARNING); 
    for(int i=0;i<ctx->numLibUsbDevices;i++)
        ctx->libUsbDeviceOpen[i]=0;
    ssize_t k; //for iterating through the list
    libusb_device_descriptor desc;
    int numDevicesNotFound=numDevices;
    int c = -1;
    int r;
    for(int i=0;i<numDevices;i++)
    {
	    for (k = 0; k < ctx->numLibUsbDevices; k++) 
	    {
	        if(ctx->libUsbDeviceOpen[k]==0)
	        {
	            r=getLibUsbErrorString("libusb_get_device_descriptor",libusb_get_device_descriptor(ctx->libUsbDevs[k], &desc));
	            if (r < 0) {
	                //cout << "failed to get device descriptor" << endl;
	                //return -1;
	                continue;
	            }
	            if (desc.idVendor == VENDOR_ID)
	            {
	                getLibUsbErrorString("libusb_open", libusb_open(ctx->libUsbDevs[k],&ctx->aaOmniDevices[i].libUsbDeviceHandle));
	                if (getLibUsbErrorString("libusb_kernel_driver_active",libusb_kernel_driver_active(ctx->aaOmniDevices[i].libUsbDeviceHandle, 0)) == 1) 
	                { //find out if kernel driver is attached
	                    //cout << "Kernel Driver Active" << endl;
	                    if (getLibUsbErrorString("libusb_detach_kernel_driver",libusb_detach_kernel_driver(ctx->aaOmniDevices[i].libUsbDeviceHandle, 0)) == 0) //detach it
	                       ;// cout << "Kernel Driver Detached!" << endl;
	                }
	                r = getLibUsbErrorString("libusb_claim_interface",libusb_claim_interface(ctx->aaOmniDevices[i].libUsbDeviceHandle, 0)); //claim interface 0 (the first) of device (mine had jsut 1)
	                if (r < 0) {
	                    //cout << "Cannot Claim Interface" << endl;//already open
	                    libusb_close(ctx->aaOmniDevices[i].libUsbDeviceHandle);
	                    ctx->aaOmniDevices[i].libUsbDeviceHandle=NULL;//LIBUSB_INVALID_DEVICE_HANDLE;
	                }
	                else
	                {
	                    numDevicesNotFound--;
	                    ctx->libUsbDeviceOpen[k]=1;
	                    ctx->aaOmniDevices[i].alphaFiltering=ALPHA_FILTERING;
	                    break;//success
	                }
	            }
	        }
	        else
	            continue;
	    }
    }
    if(numDevicesNotFound)
    {
    	for(int i=0;i<numDevices;i++)
    	{
    		if(ctx->aaOmniDevices[i].libUsbDeviceHandle!=NULL)
    		libusb_close(ctx->aaOmniDevices[i].libUsbDeviceHandle);
    	}
    	return numDevices-numDevicesNotFound;
    }
    else
    	return numDevices;
}
void AAOmniDeleteContext(AAOmniContext* ctx)
{
	int r;
	for(auto i=0;i<ctx->numAAOmniDevices;i++)
        if(ctx->aaOmniDevices[i].libUsbDeviceHandle!=NULL)//LIBUSB_INVALID_DEVICE_HANDLE)
        {
            r = getLibUsbErrorString("libusb_release_interface",libusb_release_interface(ctx->aaOmniDevices[i].libUsbDeviceHandle, 0)); //release the claimed interface
               
            libusb_close(ctx->aaOmniDevices[i].libUsbDeviceHandle);
        }
    libusb_free_device_list(ctx->libUsbDevs, 1);
    libusb_exit(ctx->libUsbCtx); //close the session
    free(ctx->libUsbDeviceOpen);
	free(ctx->aaOmniDevices);
	free(ctx);
}
void AAOmniGetTransformationMatrix(AAOmniDevice* dev,double ret[4][4])
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			ret[i][j] = dev->transformationMat[i][j];
}
void AAOmniGetNewData(AAOmniDevice* dev)
{
	int temp;
	getLibUsbErrorString("interrupt transfer", libusb_interrupt_transfer(dev->libUsbDeviceHandle, AAOMNI_IN_ENDPOINT_ADDR, 
		(unsigned char*)&dev->inData, sizeof(omni_in), &temp, 0));
	dev->newData = 1;
}
void printMat4x4d(Matrix4x4d* mat)
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			printf("%f ", mat->data[i][j]);
		printf("\n");
	}
}
/*
It is critical that AAOmniGetNewData and AAOmniUpdateValues be called
before calling this funtion else it may result in undefined behaviour.
Also remember to start from the docking position always to avoid any 
calibration errors
*/
void AAOmniSetForceFeedback(AAOmniDevice* dev,double* currentForce)
{
	Matrix4x4d rotyNTheta1;
	populateRotate(&rotyNTheta1, -dev->baseAngles[0], 1);
	Matrix4x4d forces;
	initMat(&forces);
	forces.data[0][0] = currentForce[0];
	forces.data[1][0] = currentForce[1];
	forces.data[2][0] = currentForce[2]; 
	Matrix4x4d rotatedForce;
	MatrixMultiply(&rotyNTheta1, &forces, &rotatedForce);
	double sinnBA1 = sin(-dev->baseAngles[1]); 
	double cosnBA1 = cos(-dev->baseAngles[1]);
	double sinnBA2 = sin(-dev->baseAngles[2]);
	double cosnBA2 = cos(-dev->baseAngles[2]);
	double sinP2BA2 = sin(PI_AA / 2 + dev->baseAngles[2]);
	double cosP2BA2 = cos(PI_AA / 2 + dev->baseAngles[2]);


	double theta = atan2(AAOMNI_ARM_LENGTH1*sinnBA1 - AAOMNI_ARM_LENGTH2*cosnBA2,
		AAOMNI_ARM_LENGTH1*cosnBA1 + AAOMNI_ARM_LENGTH2*sinnBA2);
	double torque[3] = { 0, 0, 0 };
	double ret[4][4];
	AAOmniGetTransformationMatrix(dev, ret);
	double distPtrFromCenter = sqrt(ret[0][3] * ret[0][3] + (ret[2][3] - AAOMNI_Z_OFFSET)*(ret[2][3] - AAOMNI_Z_OFFSET) + 
		(ret[1][3] - AAOMNI_Y_OFFSET)*(ret[1][3] - AAOMNI_Y_OFFSET)) * AAOMNI_MM_TO_M;
	double ppdDistfromYaxis = sqrt(ret[0][3] * ret[0][3] + (ret[2][3] - AAOMNI_Z_OFFSET)*(ret[2][3] - AAOMNI_Z_OFFSET)) * AAOMNI_MM_TO_M;
	torque[0] = rotatedForce.data[0][0] * ppdDistfromYaxis;
	double denom1 = cos(theta)*sinP2BA2 - cosP2BA2*sin(theta);
	torque[1] = (rotatedForce.data[1][0] * sinP2BA2 - rotatedForce.data[2][0] * cosP2BA2) /
		(denom1)*distPtrFromCenter;//(AAOMNI_ARM_LENGTH1 / 1000.0);
	double denom2 = cosP2BA2*sin(theta) + sinP2BA2*cos(theta);
	torque[2] = (rotatedForce.data[1][0] * sin(theta) + rotatedForce.data[2][0] * cos(theta)) /
		(denom2)*(AAOMNI_ARM_LENGTH2 * AAOMNI_MM_TO_M);
	//torque[2] -= torque[1]; //unsure of this equation
	torque[2] = -torque[2];//The motor configured reverse
	
	torque[0] *= AAOMNI_TORQUE_SCALE_F;
	torque[1] *= AAOMNI_TORQUE_SCALE_F;
	torque[2] *= AAOMNI_TORQUE_SCALE_F;
	if (torque[0] > AAOMNI_MAX_FF_PWM)
		torque[0] = AAOMNI_MAX_FF_PWM;
	else if (torque[0] < -AAOMNI_MAX_FF_PWM)
		torque[0] = -AAOMNI_MAX_FF_PWM;
	if (torque[1] > AAOMNI_MAX_FF_PWM)
		torque[1] = AAOMNI_MAX_FF_PWM;
	else if (torque[1] < -AAOMNI_MAX_FF_PWM)
		torque[1] = -AAOMNI_MAX_FF_PWM;
	if (torque[2] > AAOMNI_MAX_FF_PWM)
		torque[2] = AAOMNI_MAX_FF_PWM;
	else if (torque[2] < -AAOMNI_MAX_FF_PWM)
		torque[2] = -AAOMNI_MAX_FF_PWM;
	
	dev->outData.mot1 = abs(torque[0]);
	dev->outData.mot2 = abs(torque[1]);
	dev->outData.mot3 = abs(torque[2]);
	if (torque[0]<0)
		dev->outData.mot1 |= (1 << AAOMNI_TORQUE_DIR_BIT);
	if (torque[1]<0)
		dev->outData.mot2 |= (1 << AAOMNI_TORQUE_DIR_BIT);
	if (torque[2]<0)
		dev->outData.mot3 |= (1 << AAOMNI_TORQUE_DIR_BIT);
	int temp;
	getLibUsbErrorString("interrupt transfer", libusb_interrupt_transfer(dev->libUsbDeviceHandle, AAOMNI_OUT_ENDPOINT_ADDR, 
		(unsigned char*)&dev->outData, sizeof(omni_out), &temp, 0));
}
void AAOmniUpdateValues(AAOmniDevice* dev)
{
	if (dev->newData == 1)
	{
		double actualAngles[3];
		actualAngles[0] = (double)(dev->inData.mot1)*(THETA1_MAX_DEGREES - THETA1_MIN_DEGREES) / (THETA1_MAX_COUNT - THETA1_MIN_COUNT) + THETA1_OFFSET_DEGREES;
		actualAngles[1] = (double)(dev->inData.mot2)*(THETA2_MAX_DEGREES - THETA2_MIN_DEGREES) / (THETA2_MAX_COUNT - THETA2_MIN_COUNT) + THETA2_OFFSET_DEGREES;
		actualAngles[2] = (double)(dev->inData.mot3)*(THETA3_MAX_DEGREES - THETA3_MIN_DEGREES) / (THETA3_MAX_COUNT - THETA3_MIN_COUNT) + THETA3_OFFSET_DEGREES;

		dev->baseAngles[0] = (double)PI_AA*actualAngles[0] / 180;
		dev->baseAngles[1] = (double)PI_AA*actualAngles[1] / 180;
		dev->baseAngles[2] = (double)PI_AA*actualAngles[2] / 180;

		//Equation of a line with 2 points
		dev->gimbalAngles[0] = (double)PI_AA / 180.0*(((AAOMNI_GIMBAL1_MAX_ANGLE - AAOMNI_GIMBAL1_MIN_ANGLE) / (AAOMNI_GIMBAL1_MAX_ENCODER - AAOMNI_GIMBAL1_MIN_ENCODER))*
			(dev->inData.pot1 - AAOMNI_GIMBAL1_MIN_ENCODER) + AAOMNI_GIMBAL1_MIN_ANGLE);
		dev->gimbalAngles[1] = (double)PI_AA / 180.0*(((AAOMNI_GIMBAL2_MAX_ANGLE - AAOMNI_GIMBAL2_MIN_ANGLE) / (AAOMNI_GIMBAL2_MAX_ENCODER - AAOMNI_GIMBAL2_MIN_ENCODER))*
			(dev->inData.pot2 - AAOMNI_GIMBAL2_MIN_ENCODER) + AAOMNI_GIMBAL2_MIN_ANGLE);
		dev->gimbalAngles[2] = (double)PI_AA / 180.0*(((AAOMNI_GIMBAL3_MAX_ANGLE - AAOMNI_GIMBAL3_MIN_ANGLE) / (AAOMNI_GIMBAL3_MAX_ENCODER - AAOMNI_GIMBAL3_MIN_ENCODER))*
			(dev->inData.pot3 - AAOMNI_GIMBAL3_MIN_ENCODER) + AAOMNI_GIMBAL3_MIN_ANGLE);

		dev->gimbalAnglesFiltered[0] = dev->alphaFiltering*dev->gimbalAnglesFiltered[0] + (1 - dev->alphaFiltering)*dev->gimbalAngles[0];
		dev->gimbalAnglesFiltered[1] = dev->alphaFiltering*dev->gimbalAnglesFiltered[1] + (1 - dev->alphaFiltering)*dev->gimbalAngles[1];
		dev->gimbalAnglesFiltered[2] = dev->alphaFiltering*dev->gimbalAnglesFiltered[2] + (1 - dev->alphaFiltering)*dev->gimbalAngles[2];
		//std::cout<<dev->gimbalAngles[0]<<"\t"<<dev->gimbalAngles[1]<<"\t"<<dev->gimbalAngles[2]<<"\t"<<std::endl;
		/*int consider[3]={0,0,0};
		//consider[0]=abs(m_omni_in.pot1-t_omni_in.pot1)>200;
		//consider[1]=abs(m_omni_in.pot2-t_omni_in.pot2)>200;
		//consider[2]=abs(m_omni_in.pot3-t_omni_in.pot3)>200;
		consider[0]=1;
		consider[1]=1;
		consider[2]=1;
		//std::cout<<consider[0]<<consider[1]<<consider[2]<<std::endl;
		for(auto j=0;j<3;j++)
		gimAAvg[j]=gimA[j];

		for(auto j=0;j<3;j++)
		if(consider[j]==1)
		gimAAvg[j]=(1-alpha)*gimA[j]+alpha*gimAAvg[j];

		*/
		double* gimAAvg = dev->gimbalAnglesFiltered;
		double* baseA = dev->baseAngles;
		//Matrix4x4d mtransTip;//apparently not required
		//populateTranslate(&mtransTip,0,0,-AAOMNI_TIP_LENGTH);
		Matrix4x4d mtrans1;
		populateTranslate(&mtrans1, 0, AAOMNI_Y_OFFSET, AAOMNI_Z_OFFSET);
		Matrix4x4d mrot1;//rot z
		populateRotate(&mrot1, gimAAvg[2], 2);
		Matrix4x4d mrot2;//rot x
		populateRotate(&mrot2, gimAAvg[1], 0);
		Matrix4x4d mrot3;//rot y
		populateRotate(&mrot3, gimAAvg[0] + gimAAvg[2] * AAOMNI_GIM0_GIM2_COUPLING_F, 1);//Ugly hack to compensate for the hardware interference between gimbal 1 and gimbal 3
		Matrix4x4d mtrans2;
		populateTranslate(&mtrans2, 0, -AAOMNI_ARM_LENGTH2, 0);
		Matrix4x4d mrot4;//rot x
		populateRotate(&mrot4, baseA[2] - baseA[1], 0);
		Matrix4x4d mtrans3;
		populateTranslate(&mtrans3, 0, 0, AAOMNI_ARM_LENGTH1);
		Matrix4x4d mrot5;//rot x
		populateRotate(&mrot5, baseA[1], 0);
		Matrix4x4d mrot6;//rot y
		populateRotate(&mrot6, baseA[0], 1);
		//Mat4x4d res=mtrans1*mrot6*mrot5*mtrans3*mrot4*mtrans2*mrot3*mrot2*mrot1*mtransTip;
		Matrix4x4d res1, res2;
		//MatrixMultiply(&mrot1,&mtransTip,&res1);
		MatrixMultiply(&mrot2, &mrot1, &res2);
		MatrixMultiply(&mrot3, &res2, &res1);
		MatrixMultiply(&mtrans2, &res1, &res2);
		MatrixMultiply(&mrot4, &res2, &res1);
		MatrixMultiply(&mtrans3, &res1, &res2);
		MatrixMultiply(&mrot5, &res2, &res1);
		MatrixMultiply(&mrot6, &res1, &res2);
		MatrixMultiply(&mtrans1, &res2, &res1);
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				dev->transformationMat[i][j] = res1.data[i][j];
		dev->newData=0;
	}
}
void AAOmniRemoveAllForces(AAOmniDevice *dev)
{
	int temp;
	dev->outData.mot1 = 0;
	dev->outData.mot2 = 0;
	dev->outData.mot3 = 0;
	getLibUsbErrorString("interrupt transfer", libusb_interrupt_transfer(dev->libUsbDeviceHandle, AAOMNI_OUT_ENDPOINT_ADDR,
		(unsigned char*)&dev->outData, sizeof(omni_out), &temp, 0));
}