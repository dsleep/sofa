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
    	return -1;
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
	double* gimAAvg=dev->gimbalAnglesFiltered;
	double* baseA=dev->baseAngles;
	Matrix4x4d mtransTip;
    populateTranslate(&mtransTip,0,0,-AAOMNI_TIP_LENGTH);
	Matrix4x4d mtrans1;
    populateTranslate(&mtrans1,0,50, -100);
    Matrix4x4d mrot1;//rot z
    populateRotate(&mrot1,gimAAvg[2],2);
    Matrix4x4d mrot2;//rot x
    populateRotate(&mrot2,gimAAvg[1],0);
    Matrix4x4d mrot3;//rot y
    populateRotate(&mrot3,gimAAvg[0],1);
    Matrix4x4d mtrans2;
    populateTranslate(&mtrans2,0,-AAOMNI_ARM_LENGTH2,0);
    Matrix4x4d mrot4;//rot x
    populateRotate(&mrot4,baseA[2]-baseA[1],0);
    Matrix4x4d mtrans3;
    populateTranslate(&mtrans3,0,0,AAOMNI_ARM_LENGTH1);
    Matrix4x4d mrot5;//rot x
    populateRotate(&mrot5,baseA[1],0);
    Matrix4x4d mrot6;//rot y
    populateRotate(&mrot6,baseA[0],1);
    //Mat4x4d res=mtrans1*mrot6*mrot5*mtrans3*mrot4*mtrans2*mrot3*mrot2*mrot1*mtransTip;
    Matrix4x4d res1,res2;
    MatrixMultiply(&mrot1,&mtransTip,&res1);
    MatrixMultiply(&mrot2,&res1,&res2);
    MatrixMultiply(&mrot3,&res2,&res1);
    MatrixMultiply(&mtrans2,&res1,&res2);
    MatrixMultiply(&mrot4,&res2,&res1);
    MatrixMultiply(&mtrans3,&res1,&res2);
    MatrixMultiply(&mrot5,&res2,&res1);
    MatrixMultiply(&mrot6,&res1,&res2);
    MatrixMultiply(&mtrans1,&res2,&res1);
    for(int i=0;i<4;i++)
    	for(int j=0;j<4;j++)
    		ret[i][j]=res1.data[i][j];
}
void AAOmniGetNewData(AAOmniDevice* dev)
{
	int temp;
	getLibUsbErrorString("interrupt transfer", libusb_interrupt_transfer(dev->libUsbDeviceHandle, AAOMNI_IN_ENDPOINT_ADDR, 
		(unsigned char*)&dev->inData, sizeof(omni_in), &temp, 0));
}
void AAOmniSetForceFeedback(AAOmniDevice* dev,double* currentForce)
{
	int forceScale=50;
    dev->outData.mot1=((abs(currentForce[0])>1)?1:abs(currentForce[0]))*forceScale;
    dev->outData.mot2=((abs(currentForce[1])>1)?1:abs(currentForce[1]))*forceScale;
    dev->outData.mot3=((abs(currentForce[2])>1)?1:abs(currentForce[2]))*forceScale;
    if(currentForce[0]<0)
        dev->outData.mot1|=0x8000;
    if(currentForce[1]<0)
        dev->outData.mot2|=0x8000;
    if(currentForce[2]<0)
        dev->outData.mot3|=0x8000;
	int temp;
	//getLibUsbErrorString("interrupt transfer", libusb_interrupt_transfer(dev->libUsbDeviceHandle, AAOMNI_OUT_ENDPOINT_ADDR, 
	//	(unsigned char*)&dev->outData, sizeof(omni_out), &temp, 0));
}
void AAOmniUpdateValues(AAOmniDevice* dev)
{
	double actualAngles[3];
	actualAngles[0] = (double)(dev->inData.mot1*THETA1_CONST + AAOMNI_THETA1_OFFSET);
	actualAngles[1] = (double)(dev->inData.mot2*THETA2_CONST + AAOMNI_THETA2_OFFSET);
	actualAngles[2] = (double)(dev->inData.mot3*THETA3_CONST + AAOMNI_THETA3_OFFSET);
	//double r, h, theta;
	//r = AAOMNI_ARM_LENGTH1*(double)cos(pi / 180.0*actualAngles[1]) + AAOMNI_ARM_LENGTH2*(double)cos(pi / 180.0*actualAngles[2]);
	//h = AAOMNI_ARM_LENGTH1*(double)sin(pi / 180.0*actualAngles[1]) - AAOMNI_ARM_LENGTH2*(double)sin(pi / 180.0*actualAngles[2]);
	//theta = (double)pi / 180.0f*(actualAngles[0]+90);
	//printf("theta1: %f, theta2: %f, theta3: %f\n", actualAngles[0], actualAngles[1], actualAngles[2]);
	//printf("r: %f, h: %f, theta: %f\n", r, h, theta);
	//xyz[0] = -r*cos(theta);
	//xyz[2] = r*sin(theta);
	//xyz[1] = h;
	//x is left right, y is top down and z is in out
            
	dev->baseAngles[0] = (double)pi*actualAngles[0]/180;
	dev->baseAngles[1] = (double)-pi*actualAngles[1]/180;
	dev->baseAngles[2] = (double)pi*actualAngles[2]/180;

	dev->gimbalAngles[0] = (double)-pi*(GIMBAL1_CONST*dev->inData.pot1-AAOMNI_GIMBAL1_RANGE/2+AAOMNI_GIMBAL1_OFFSET)/180;
	dev->gimbalAngles[1] = (double)-pi*(GIMBAL2_CONST*dev->inData.pot2-AAOMNI_GIMBAL2_RANGE/2+AAOMNI_GIMBAL2_OFFSET)/180;
	dev->gimbalAngles[2] = (double) pi*(GIMBAL3_CONST*dev->inData.pot3-AAOMNI_GIMBAL3_RANGE/2+AAOMNI_GIMBAL3_OFFSET)/180;

	dev->gimbalAnglesFiltered[0]=dev->alphaFiltering*dev->gimbalAnglesFiltered[0]+(1-dev->alphaFiltering)*dev->gimbalAngles[0];
	dev->gimbalAnglesFiltered[1]=dev->alphaFiltering*dev->gimbalAnglesFiltered[1]+(1-dev->alphaFiltering)*dev->gimbalAngles[1];
	dev->gimbalAnglesFiltered[2]=dev->alphaFiltering*dev->gimbalAnglesFiltered[2]+(1-dev->alphaFiltering)*dev->gimbalAngles[2];
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
}