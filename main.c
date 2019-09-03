/**
 * main.c
 */
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <math.h>

int MsgBuffer[14], MpuReadCount=0, MpuAvgCounter=0, XPointer=0, YPointer=0, ZPointer=0;
float yaw=0, pitch=0, roll=0, Angle_X, Angle_Y;
float gyroAngleX = 0, gyroAngleY = 0, GtempX=0, GtempY=0, GtempZ=0;
float MPUtemp, XBuffer[5], YBuffer[5], ZBuffer[5], Filter[3];
int sgn(float); //no signum in math.h, writing own function.

void FilterImu(float, float, float);    //Input is X, Y, Z in order.
void I2CInit(void);
void I2C_read_data(int);
void GetImuTemps(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void Xint_reset(void);


int main(void)
{
        //
        // Step 1. Initialize System Control:
        // PLL, WatchDog, enable Peripheral Clocks
        // This example function is found in the DSP2833x_SysCtrl.c file.
        //
        InitSysCtrl();

        //
        // Step 2. Initialize GPIO:
        // This example function is found in the DSP2833x_Gpio.c file and
        // illustrates how to set the GPIO to it's default state.
        //
        // InitGpio();

        //
        // Setup only the GP I/O only for I2C functionality
        //
        InitI2CGpio();

        //
        // Step 3. Clear all interrupts and initialize PIE vector table
        // Disable CPU interrupts
        //
        DINT;

        //
        // Initialize PIE control registers to their default state.
        // The default state is all PIE interrupts disabled and flags
        // are cleared.
        // This function is found in the DSP2833x_PieCtrl.c file.
        //
        InitPieCtrl();

        //
        // Disable CPU interrupts and clear all CPU interrupt flags
        //
        IER = 0x0000;
        IFR = 0x0000;

        //
        // Initialize the PIE vector table with pointers to the shell Interrupt
        // Service Routines (ISR).
        // This will populate the entire table, even if the interrupt
        // is not used in this example.  This is useful for debug purposes.
        // The shell ISR routines are found in DSP2833x_DefaultIsr.c.
        // This function is found in DSP2833x_PieVect.c.
        //
        InitPieVectTable();

        //
        // Interrupts that are used in this example are re-mapped to
        // ISR functions found within this file.
        //

        EALLOW;         // This is needed to write to EALLOW protected registers
        PieVectTable.TINT0 = &cpu_timer0_isr;
        PieVectTable.XINT1 = &Xint_reset;
        EDIS;    // This is needed to disable write to EALLOW protected registers

        //
        // Step 4. Initialize all the Device Peripherals:
        //

        ConfigCpuTimer(&CpuTimer0, 150, 7000);
        CpuTimer0Regs.TCR.all = 0x4000;

        //
        // Step 5. User specific code
        //

        I2CInit();

        // Enable interrupts after start

        IER |= M_INT1;
        PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
        PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
        EINT;           // Enable Global interrupt INTM
        ERTM;           // Enable Global realtime interrupt DBGM




        while (1)
        {

        }

}

void I2CInit(void)
{
   I2caRegs.I2CSAR = 0x0068;  // Slave address
   I2caRegs.I2CPSC.all = 14;

   I2caRegs.I2CCLKL = 10;      // NOTE: must be non zero
   I2caRegs.I2CCLKH = 5;       // NOTE: must be non zero
   I2caRegs.I2CIER.all = 0x24; // Enable SCD & ARDY interrupts

   //
   // Take I2C out of reset
   // Stop I2C when suspended
   //
   I2caRegs.I2CMDR.all = 0x0020;

   I2caRegs.I2CFFTX.all = 0x6000;  // Enable FIFO mode and TXFIFO
   I2caRegs.I2CFFRX.all = 0x2040;  // Enable RXFIFO, clear RXFFINT,

   //Clear I2C Buffer:
   unsigned int i;
   for (i = 0; i < 14; i++)
       {
           MsgBuffer[i] = 0x0000;
       }

   for (i = 0; i < 5; i++)  //Clear buffers for noise filter
          {
              XBuffer[i] = 0;
              YBuffer[i] = 0;
              ZBuffer[i] = 0;
          }

   while(I2caRegs.I2CMDR.bit.STP == 1);
      // Check if bus busy
   while(I2caRegs.I2CSTR.bit.BB == 1);

    // Setup slave address
   I2caRegs.I2CSAR = 0x68;
    // Setup number of bytes to send
    // Msg + Address
   I2caRegs.I2CCNT = 2;
   I2caRegs.I2CMDR.all = 0x2E20;    // Send start as master transmitter
   I2caRegs.I2CDXR = 0x6B;  //Mpu PwrMgmnt
   I2caRegs.I2CDXR = 0x00;
   for(i=0 ; i<700; i++);    //Wait


   I2caRegs.I2CSAR = 0x68;
   I2caRegs.I2CCNT = 2;
   I2caRegs.I2CMDR.all = 0x2E20;    // Send start as master transmitter
   I2caRegs.I2CDXR = 0x1B;          //Gyro_Config, start at 500deg/s
   I2caRegs.I2CDXR = 0x08;
   for(i=0 ; i<700; i++);    //Wait


   I2caRegs.I2CSAR = 0x68;
   I2caRegs.I2CCNT = 2;
   I2caRegs.I2CMDR.all = 0x2E20;    // Send start as master transmitter
   I2caRegs.I2CDXR = 0x1C;          //Acelerometer Start at +-2g
   I2caRegs.I2CDXR = 0x00;
   for(i=0 ; i<700; i++);         //Wait

   return;

}

void I2C_read_data(int ReadOffset)
{
    unsigned int i;
    // Wait until the STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    while(I2caRegs.I2CMDR.bit.STP == 1);
    // Check if bus busy
    while(I2caRegs.I2CSTR.bit.BB == 1);
    I2caRegs.I2CSAR = 0x68;
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CMDR.all = 0x2620;    // Send message, no stop
    I2caRegs.I2CDXR = 0x3B+ReadOffset;
    for(i=0 ; i<700; i++);         //Wait
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CMDR.all = 0x2C20;    // recieve messages
    MsgBuffer[ReadOffset] =I2caRegs.I2CDRR;

}

void GetImuTemps(void)
{
    float temp;

    temp= (MsgBuffer[7] << 8) + MsgBuffer[6];   //Raw read
    //
    // C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
    // Can be found in Register Map p30.
    //
    MPUtemp = (temp/340) + 36.53;
}

__interrupt void cpu_timer0_isr(void)
{
    //
    // IMPORTANT NOTE
    // MPU should be at rest during start. Otherwise, calibration will be wrong.
    //

    float PI=3.141, Ax, Ay, Az, GyrX, GyrY, GyrZ;

    if(MpuReadCount < 14)
    {
        I2C_read_data(MpuReadCount);
        MpuReadCount++;
    }
    else //MpuReadCount == 14
    {
        MpuReadCount=0;
        Ax= (MsgBuffer[1] << 8) + MsgBuffer[0];
        Ay= (MsgBuffer[3] << 8) + MsgBuffer[2];
        Az= (MsgBuffer[5] << 8) + MsgBuffer[4];

        Angle_X = (atan(Ay / sqrt(pow(Ax, 2) + pow(Az, 2))) * 180 / PI) - 0.58;
        Angle_Y = (atan(-1 * Ax / sqrt(pow(Ay, 2) + pow(Az, 2))) * 180 / PI) + 1.58;

        GyrX= (MsgBuffer[9] << 8) + MsgBuffer[8];
        GyrY= (MsgBuffer[11] << 8) + MsgBuffer[10];
        GyrZ= (MsgBuffer[13] << 8) + MsgBuffer[12];

        if (GtempX == 0)
        {
            GtempX = GyrX/65.5;
            GtempY = GyrX/65.5;
            GtempZ = GyrX/65.5;
        }
        else if (MpuAvgCounter < 100)
        {
            GtempX = (GtempX + GyrX/65.5)/2; //Mean Filter
            GtempY = (GtempY + GyrY/65.5)/2;
            GtempZ = (GtempZ + GyrZ/65.5)/2;
            MpuAvgCounter++;
        }

        GyrX = (GyrX/65.5) - GtempX;
        GyrY = (GyrY/65.5) - GtempY;
        GyrZ = (GyrZ/65.5) - GtempZ;

        FilterImu(GyrX, GyrY, GyrZ);
        GyrX = Filter[0];
        GyrY = Filter[1];
        GyrZ = Filter[2];

        if (GyrZ < -5000) GyrZ=0;  //Must be a measurement error. Ignore.

        if (fabs(GyrX) > 4) gyroAngleX = gyroAngleX + GyrX * 0.006; // 2 deg/s * s = deg, time assumed 6ms based on interrupt
        if (fabs(GyrY) > 4) gyroAngleY = gyroAngleY + GyrY * 0.006;

        if (fabs(GyrZ) > 4) yaw =  yaw + GyrZ * 0.006;                   //1.5°/s Deadzone
        // Complementary filter - combine accelerometer and gyro angle values
        if (fabs(GyrX) > 4) roll = 0.9 * gyroAngleX + 0.1 * Angle_X;    //1.5°/s Deadzone
        if (fabs(GyrY) > 4) pitch = 0.9 * gyroAngleY + 0.1 * Angle_Y;   //1.5°/s Deadzone

        //
        // Limit angles to -+360°s
        //

        if (fabs(yaw) > 360.0) yaw = yaw - (360*sgn(yaw));
        if (fabs(roll) > 360.0) roll = roll - (360*sgn(roll));
        if (fabs(pitch) > 360.0) pitch = pitch - (360*sgn(pitch));

        GetImuTemps();


    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void FilterImu(float GyrX,float GyrY,float GyrZ)
{
    //
    // Place values into the queue:
    //

    XBuffer[XPointer] = GyrX;
    XPointer++;
    YBuffer[YPointer] = GyrY;
    YPointer++;
    ZBuffer[ZPointer] = GyrZ;
    ZPointer++;

    //
    // Loop pointer around to prevent overflow:
    //

    if (XPointer > 4) XPointer = 0;
    if (YPointer > 4) YPointer = 0;
    if (ZPointer > 4) ZPointer = 0;

    //
    // Apply mean filter
    //
    int i;

    for(i=0 ; i<5 ; i++)
    {
        Filter[0] = Filter[0] + XBuffer[i];
    }

    Filter[0] = Filter[0]/5;    //X axis

    for(i=0 ; i<5 ; i++)
    {
        Filter[1] = Filter[1] + YBuffer[i];
    }

    Filter[1] = Filter[1]/5;    //Y axis

    for(i=0 ; i<5 ; i++)
    {
        Filter[2] = Filter[2] + ZBuffer[i];
    }

    Filter[2] = Filter[2]/5;    //Z axis
}

__interrupt void Xint_reset(void)
{
    //
    // Do NOT reset during motion. Calibration will be messed up.
    //

    int i;

    // Recalibrate to 0.
    GtempX = GtempX + Filter[0];
    GtempY = GtempY + Filter[1];
    GtempZ = GtempZ + Filter[2];

    //Reset filter results
    for (i=0 ; i<3 ; i++)
    {
        Filter[i] = 0;
    }

    //Reset noise filter buffers:
    for (i=0 ; i<5 ; i++)
    {
        XBuffer[i] = 0;
        YBuffer[i] = 0;
        ZBuffer[i] = 0;
    }

    //Reset FIFO Pointers.
    XPointer = 0;
    YPointer = 0;
    ZPointer = 0;

    //Reset Mpu Variables
    MpuReadCount=0;
    yaw=0;
    pitch=0;
    roll=0;
    Angle_X=0;
    Angle_Y=0;
    gyroAngleX=0;
    gyroAngleY=0;

    //Acknowledge interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

int sgn(float num)
{
    // x<0 => sgn(x)=-1
    // x>0 => sgn(x)=+1
    // x=0 => sgn(x)= 0

    if (num < 0) return -1;
    else if (num > 0) return 1;
    else return 0;


}


