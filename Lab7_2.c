#include <F28x_Project.h>
#include "OneToOneI2CDriver.h"
#include <AIC23.h>
#include <InitAIC23.c>
#include <SPILibrary.h>

interrupt void Mcbsp_Rx_ISR(void);

//----------------------------------------------------Global Variables-------------------------------------------------------------

//Taps_HP: HPF, Stop Band (-30 dBs) 0 - 2000 Hz, Pass Band (3 dB ripple) 4000-24000 Hz, Fs = 48 KHz;
float32 Taps_HP[23] = { 0.0059,0.0768,0.0062,0.0033,-0.0113,-0.0303,-0.0528,-0.0763,-0.0983,-0.1163,-0.1281,0.8678,-0.1281,-0.1163,-0.0983,-0.0763,-0.0528,-0.0303,-0.0113,0.0033,0.0062,0.0768,0.0059 } ;
int16 Taps = 23 ;

//float32 Taps_HP[43] = {-0.0068,0.0462,-0.0171,-0.0215,-0.0179,-0.0127,-0.0066,0.0010,0.0095,0.0176, 0.0239,0.0266,0.0238,0.0147,-0.0011,-0.0228,-0.0486,-0.0760,-0.1017,-0.1228, -0.1366,0.8585,-0.1366,-0.1228,-0.1017,-0.0760,-0.0486,-0.0228,-0.0011,0.0147, 0.0238,0.0266,0.0239,0.0176,0.0095,0.0010,-0.0066,-0.0127,-0.0179,-0.0215,-0.0171,0.0462,-0.0068 } ;
//int16 Taps = 43 ;

int16 buffer[64] ;
int16 count = 0 ;
int16 command, sample_L, sample_R, sample, sample_out ;
Uint32 address ;
int16 mask = 63 ;
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

        address = 0 ;


    while(1);



}

interrupt void Mcbsp_Rx_ISR(void)
{

    sample_L = McbspbRegs.DRR2.all ; // Load High Word
    sample_R = McbspbRegs.DRR1.all ; // Load Low Word

    sample = (int16) (((float32)sample_L + (float32)sample_R) * 0.5) ;

    buffer[address] = sample ;
    sample_out = 0 ;

    for (Uint16 i = 0 ; i < Taps ; i++){
        float32 current = (float32)buffer[(address-i) & mask]  * Taps_HP[i]  ;
        sample_out = sample_out + current ;
    }

    sample_out = (int16) ((float32)sample_out * 8.0) ;

    McbspbRegs.DXR2.all = sample_out; // Output Data
    McbspbRegs.DXR1.all = sample_out;

    address++;

    if (address == mask + 1 ){
        address = 0 ;
    }


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
