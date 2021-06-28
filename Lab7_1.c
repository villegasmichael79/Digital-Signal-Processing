#include <F28x_Project.h>
#include "OneToOneI2CDriver.h"
#include <AIC23.h>
#include <InitAIC23.c>
#include <SPILibrary.h>

interrupt void Mcbsp_Rx_ISR(void);

//----------------------------------------------------Global Variables-------------------------------------------------------------

//Taps_LP: LPF, Pass Band (3 dB ripple) 0-1400 Hz, Stop Band (-40 dBs) 3K-24K, Fs = 48 KHz;
float32 Taps_LP[42] = { -0.0068, -0.0044, -0.0053, -0.0058, -0.0058, -0.0050, -0.0032, -0.0003, 0.0037, 0.0091, 0.0156, 0.0231, 0.0315, 0.0404, 0.0495, 0.0584, 0.0666, 0.0737, 0.0795, 0.0834, 0.0855, 0.0855, 0.0834, 0.0795, 0.0737, 0.0666, 0.0584, 0.0495, 0.0404, 0.0315, 0.0231, 0.0156, 0.0091, 0.0037, -0.0003, -0.0032,-0.0050, -0.0058, -0.0058,-0.0053, -0.0044, -0.0068 } ;
int16 Taps = 42 ;
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

    for (Uint16 i = 0 ; i < 42 ; i++){
        float32 current = (float32)buffer[(address-i) & mask]  * Taps_LP[i]  ;
        sample_out = sample_out + current ;
    }

    sample_out = (int16) ((float32)sample_out * 8.0) ;

    McbspbRegs.DXR2.all = sample_out; // Output Data
    McbspbRegs.DXR1.all = 0;

    address++;

    if (address == 64){
        address = 0 ;
    }


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
