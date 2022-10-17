/*******************************************************************************
  Input / Output Port COnfiguration Routine source File

  File Name:
    port_config.c

  Summary:
    This file includes subroutine for initializing GPIO pins as analog/digital,
    input or output etc. Also to PPS functionality to Remap-able input or output 
    pins

  Description:
    Definitions in the file are for dsPIC33CK1024MP710 MC DIM plugged onto
    Motor Control Development board from Microchip
 
*******************************************************************************/
/*******************************************************************************
* Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include "port_config.h"
#include "userparms.h"

// *****************************************************************************
/* Function:
    SetupGPIOPorts()

  Summary:
    Routine to set-up GPIO ports

  Description:
    Function initializes GPIO pins for input or output ports,analog/digital pins,
    remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void SetupGPIOPorts(void)
{
    // Reset all PORTx register (all inputs)
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    #ifdef TRISF
        TRISF = 0xFFFF;
        LATF  = 0x0000;
    #endif
    #ifdef ANSELF
        ANSELF = 0x0000;
    #endif

    MapGPIOHWFunction();

    return;
}
// *****************************************************************************
/* Function:
    Map_GPIO_HW_Function()

  Summary:
    Routine to setup GPIO pin used as input/output analog/digital etc

  Description:
    Function initializes GPIO pins as input or output port pins,analog/digital 
    pins,remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void MapGPIOHWFunction(void)
{
    
    /* ANALOG SIGNALS */

    // Configure Port pins for Motor Current Sensing
    //Ia+
    ANSELAbits.ANSELA2 = 1;
    TRISAbits.TRISA2 = 1;    //Pin 24: OA1IN+/AN9/PMA6/RA2
    
    //Ia-
    ANSELAbits.ANSELA1 = 1;
    TRISAbits.TRISA1 = 1;    //Pin 21: OA1IN-/ANA1/RA1
    
    //Ia_out
    ANSELAbits.ANSELA0 = 1;
    TRISAbits.TRISA0 = 0;    //Pin 18: OA1OUT/AN0/CMP1A/IBIAS0/RA0
    
    //Ia_Ext
    ANSELEbits.ANSELE0 = 1;
    TRISEbits.TRISE0 = 1;    //Pin 2: AN20/ANC0/CMP5C/RE0
    
    //Ib+
    ANSELBbits.ANSELB4 = 1;
    TRISBbits.TRISB4 = 1;    //Pin 57: PGC2/OA2IN+/RP36/RB4
    
    //Ib-
    ANSELBbits.ANSELB3 = 1;
    TRISBbits.TRISB3 = 1;    //Pin 54: PGD2/OA2IN-/AN8/CMP4A/RP35/RB3
    
    //Ib_out
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 0;    //Pin 51: OA2OUT/AN1/AN7/ANA0/ANA2/ANA3/CMP1D/CMP2D/CMP3D/CMP4D/CMP5D/CMP6D/RP34/SCL3/INT0/RB2
    
    //Ib_Ext
    ANSELEbits.ANSELE1 = 1;
    TRISEbits.TRISE1 = 1;    //Pin 4: AN21/ANC1/CMP6B/RE1
    
    //Ibus+
    ANSELCbits.ANSELC2 = 1;
    TRISCbits.TRISC2 = 1;    //Pin 37: OA3IN+/AN14/CMP2B/ISRC1/RP50/PMD13/PMA13/RC2
    
    //Ibus-
    ANSELCbits.ANSELC1 = 1;
    TRISCbits.TRISC1 = 1;    //Pin 36: OA3IN-/AN13/CMP1B/ISRC0/RP49/PMA7/RC1
    
    //Ibus_Out
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 0;    //Pin 30: OA3OUT/AN4/ANB1/ANB2/CMP3B/IBIAS3/RA4
    
    //Ibus_Ext
    ANSELDbits.ANSELD10 = 1;
    TRISDbits.TRISD10 = 1;    //Pin 47: AN18/ANC2/CMP3C/ISRC3/RP74/PMD9/PMA9/RD10
    
#ifdef INTERNAL_OPAMP_CONFIG
    
    //Op-Amp Configuration
    AMPCON1Hbits.NCHDIS1 = 0;    //Wide input range for Op Amp #1
    AMPCON1Lbits.AMPEN1 = 1;     //Enables Op Amp #1
    
    AMPCON1Hbits.NCHDIS2 = 0;    //Wide input range for Op Amp #2
    AMPCON1Lbits.AMPEN2 = 1;     //Enables Op Amp #2
    
    AMPCON1Lbits.AMPON = 1;      //Enables op amp modules if their respective AMPENx bits are also asserted

#endif
    
    // Potentiometer input - used as Speed Reference
    // POT1 : DIM #28
    ANSELCbits.ANSELC6 = 1;
    TRISCbits.TRISC6 = 1;   // PIN38: AN17/ANN1/CMP4B/IBIAS1/RP54/PMD12/PMA12/RC6
    //Vbus  : DIM #39
    ANSELCbits.ANSELC3 = 1;
    TRISCbits.TRISC3 = 1;  //AN15/ANN2/CMP2A/IBIAS2/RP51/PMD11/PMA11/RC3
    //MOSFET Temp  : DIM #24
    ANSELDbits.ANSELD11 = 1;
    TRISDbits.TRISD11 = 1;  //AN19/ANB0/CMP2C/RP75/PMA0/PMALL/PSA0/RD11
    
    /* Digital SIGNALS */   
    // DIGITAL INPUT/OUTPUT PINS

    // Inverter Control - PWM Outputs
    // PWM1L : PIN #3  RP47/PWM1L/PMD6/RB15
    // PWM1H : PIN #1  RP46/PWM1H/PMD5/RB14
    // PWM2L : PIN #80  RP45/PWM2L/PMD4/RB13
    // PWM2H : PIN #78  TDI/RP44/PWM2H/PMD3/RB12
    // PWM3L : PIN #76  TCK/RP43/PWM3L/PMD2/RB11
    // PWM3H : PIN #75  TMS/RP42/PWM3H/PMD1/RB10
    TRISBbits.TRISB14 = 0 ;          
    TRISBbits.TRISB15 = 0 ;         
    TRISBbits.TRISB12 = 0 ;          
    TRISBbits.TRISB13 = 0 ;           
    TRISBbits.TRISB10 = 0 ;          
    TRISBbits.TRISB11 = 0 ;         
    

    // Debug LEDs
    // LED1 : DIM #30
    TRISEbits.TRISE4 = 0;           // PIN:28 - RE4
    
    // LED2 : DIM #32
    TRISFbits.TRISF5 = 0;           // PIN:26 - RP85/RF5

    // Push button Switches
      
    // SW1 : DIM #34
    TRISFbits.TRISF4 = 1;           // PIN:25 - RP84/RF4
    // SW2 : DIM #36
    TRISFbits.TRISF3 = 1;           // PIN:23 - RP83/RF3
  
	
	/** Diagnostic Interface for MCLV-48V-300W.
        Re-map UART Channels to the device pins connected to the following 
        DIM pins on the Motor Control Development Boards .
        UART_RX : DIM #54 (Input)  Pin :95 RP97/APWM1H/RA6
        UART_TX : DIM #52 (Output) Pin :96 RP98/APWM1L/RA7   */
    _U1RXR = 97;
    _RP98R = 0b000001;
}
