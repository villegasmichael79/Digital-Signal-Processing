#include <F28x_Project.h>
#include "OneToOneI2CDriver.h"
#include <AIC23.h>
#include <InitAIC23.c>
#include <SPILibrary.h>

interrupt void Mcbsp_Rx_ISR(void);

//----------------------------------------------------Global Variables-------------------------------------------------------------

int16 buffer[64] ;
int16 count = 0, stage ;
int16 command, sample_L, sample_R, sample ;
Uint32 address ;
int16 mask = 3 ;

float32 coeff[6][6] = {{0.0249, 0, -0.0249, 1.0000, -1.9410, 0.9503},
                       {0.0315, 0, -0.0315, 1.0000, -1.9345, 0.9411},
                       {0.0199, 0, -0.0199, 1.0000, -1.9878, 0.9905},
                       {0.0227, 0, -0.0227, 1.0000, -1.9692, 0.9804},
                       {0.0378, 0, -0.0378, 1.0000, -1.9680, 0.9711},
                       {0.0351, 0, -0.0351, 1.0000, -1.9478, 0.9521}};

float32 X[6][2] ;
float32 Y[6][2] ;
float32 buffer0[4], buffer1[4], buffer2[4], buffer3[4],buffer4[4], buffer5[4], first_sample ;
float32 buffery0[4], buffery1[4], buffery2[4], buffery3[4],buffery4[4], buffery5[4];

float32 sample_out[6] ;


//-------------------------------------------------End of Global Variables------------------------------------------=--------------

void main(void){

    InitSysCtrl();      // Set SYSCLK to 200 MHz, disables watchdog timer, turns on peripheral clocks

        DINT;               // Disable CPU interrupts on startup

        // Init PIE
        InitPieCtrl();      // Initialize PIE -> disable PIE and clear all PIE registers
        IER = 0x0000;       // Clear CPU interrupt register
        IFR = 0x0000;       // Clear CPU interrupt flag register
        InitPieVectTable(); // Initialize PIE vector table to known state
        EALLOW;             // EALLOW for rest of program (unless later function disables it)


        InitSPIA();
        InitMcBSPb() ;
        InitAIC23() ;

        SPI_Gpio_init();
        Spi_init() ;

        command = nomicaaudpath();      // Turn on DAC
        SpiTransmit(command);
        SmallDelay();

        EALLOW;
        PieCtrlRegs.PIECTRL.bit.ENPIE = 1 ; // Enable external interrupts
        PieVectTable.MCBSPB_RX_INT =  &Mcbsp_Rx_ISR ;
        PieCtrlRegs.PIEIER6.bit.INTx7 = 1 ; ; // Enable interrupt 6.7
        IER |= M_INT6 ;

        EnableInterrupts();

        //address = 0 ;


    while(1)
    {

    }

}

interrupt void Mcbsp_Rx_ISR(void)
{

    sample_L = McbspbRegs.DRR2.all ; // Load High Word
    sample_R = McbspbRegs.DRR1.all ; // Load Low Word

    sample = (int16) (((float32)sample_L + (float32)sample_R) * 0.5) ;

    //Stage 0
    stage = 0 ;
    buffer0[count] = (float32)sample ;
    first_sample = buffer0[count] * coeff[stage][0] ;
    X[stage][0] = buffer0[(count - 1) & mask]  * coeff[stage][1] ;
    X[stage][1] = buffer0[(count - 2) & mask]  * coeff[stage][2] ;
    Y[stage][0] = buffery0[(count - 1) & mask]  * coeff[stage][4] ;
    Y[stage][1] = buffery0[(count - 2) & mask]  * coeff[stage][5] ;

    sample_out[stage] = (first_sample + X[stage][0] + X[stage][1] - Y[stage][0] - Y[stage][1]) ;
    buffery0[count] = sample_out[stage];
    // Stage 1
    stage = 1 ;
    buffer1[count] = sample_out[stage - 1] ;
    first_sample = buffer1[count] * coeff[stage][0] ;
    X[stage][0] = buffer1[(count - 1) & mask] * coeff[stage][1] ;
    X[stage][1] = buffer1[(count - 2) & mask] * coeff[stage][2] ;
    Y[stage][0] = buffery1[(count - 1) & mask] * coeff[stage][4] ;
    Y[stage][1] = buffery1[(count - 2) & mask] * coeff[stage][5] ;

    sample_out[stage] = first_sample + X[stage][0] + X[stage][1] - Y[stage][0] - Y[stage][1] ;
    buffery1[count] = sample_out[stage];
    // Stage 2
    stage = 2 ;
    buffer2[count] = sample_out[stage - 1] ;
    first_sample = buffer2[count] * coeff[stage][0] ;
    X[stage][0] = buffer2[(count - 1) & mask] * coeff[stage][1] ;
    X[stage][1] = buffer2[(count - 2) & mask] * coeff[stage][2] ;
    Y[stage][0] = buffery2[(count - 1) & mask] * coeff[stage][4] ;
    Y[stage][1] = buffery2[(count - 2) & mask] * coeff[stage][5] ;

    sample_out[stage] = first_sample + X[stage][0] + X[stage][1] - Y[stage][0] - Y[stage][1] ;
    buffery2[count] = sample_out[stage];
    //Stage 3
    stage = 3 ;
    buffer3[count] = sample_out[stage - 1] ;
    first_sample = buffer3[count] * coeff[stage][0] ;
    X[stage][0] = buffer3[(count - 1) & mask] * coeff[stage][1] ;
    X[stage][1] = buffer3[(count - 2) & mask] * coeff[stage][2] ;
    Y[stage][0] = buffery3[(count - 1) & mask] * coeff[stage][4] ;
    Y[stage][1] = buffery3[(count - 2) & mask] * coeff[stage][5] ;

    sample_out[stage] = first_sample + X[stage][0] + X[stage][1] - Y[stage][0] - Y[stage][1] ;
    buffery3[count] = sample_out[stage];
    //Stage 4
    stage = 4 ;

    buffer4[count] = sample_out[stage - 1] ;
    first_sample = buffer4[count] * coeff[stage][0] ;
    X[stage][0] = buffer4[(count - 1) & mask] * coeff[stage][1] ;
    X[stage][1] = buffer4[(count - 2) & mask] * coeff[stage][2] ;
    Y[stage][0] = buffery4[(count - 1) & mask] * coeff[stage][4] ;
    Y[stage][1] = buffery4[(count - 2) & mask] * coeff[stage][5] ;

    sample_out[stage] = first_sample + X[stage][0] + X[stage][1] - Y[stage][0] - Y[stage][1] ;
    buffery4[count] = sample_out[stage];
    //Stage 5
    stage = 5 ;

    buffer5[count] = sample_out[stage - 1] ;
    first_sample = buffer5[count] * coeff[stage][0] ;
    X[stage][0] = buffer5[(count - 1) & mask] * coeff[stage][1] ;
    X[stage][1] = buffer5[(count - 2) & mask] * coeff[stage][2] ;
    Y[stage][0] = buffery5[(count - 1) & mask] * coeff[stage][4] ;
    Y[stage][1] = buffery5[(count - 2) & mask] * coeff[stage][5] ;

    sample_out[stage] = first_sample + X[stage][0] + X[stage][1] - Y[stage][0] - Y[stage][1] ;
    buffery5[count] = sample_out[stage];
    count++ ;

    if (count>3){
        count = 0 ;
    }

    McbspbRegs.DXR2.all = (int16)sample_out[5] ; // Output Data
    McbspbRegs.DXR1.all = (int16)sample_out[5] ;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;



}
