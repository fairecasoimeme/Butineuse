/****************************************************************************
 *
 * MODULE:             Customer Module Evaluation Tool
 *
 * COMPONENT:          CustomerModuleEvalTool.c
 *
 * VERSION:            CustomerModuleEvalTool.c
 * DESCRIPTION:
 * Code to run module based lab tests. User interaction is via serial
 * console at 38400 baud.
 */
/****************************************************************************
*
* This software is owned by NXP B.V. and/or its supplier and is protected
* under applicable copyright laws. All rights are reserved. We grant You,
* and any third parties, a license to use this software solely and
* exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139].
* You, and any third parties must reproduce the copyright and warranty notice
* and any other legend of ownership on each copy or partial copy of the
* software.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.

* Copyright NXP B.V. 2012. All rights reserved
*
***************************************************************************/
/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <string.h>
#include <AppHardwareApi.h>
#include <JPT.h>
#include "Printf.h"
#include "UartBuffered.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/



#define TIMEOUT_PER_TEST        10000
#define TIMEOUT_RESULTS         10000
#define TIMEOUT_COMMS_TO_MUT    4000000

#define PIN_SG_TRIGGER          1 << 8              /* Gig Gen Trigger Pin  */
#define PIN_DEBUG               1 << 0

#define UART_TO_PC              E_AHI_UART_0        /* Uart to PC           */
#define UART_TO_OTHER           E_AHI_UART_1        /* Uart to other board  */


#define BAUD_RATE               E_AHI_UART_RATE_38400 /* Baud rate to use   */

/* Duty cycles */
#define DC_100                  1
#define DC_50                   2
#define DC_30                   3
#define DC_1                    4
#define DC_VARIABLE             5

#define MAJOR_VERSION_NO        4
#define MINOR_VERSION_NO        5

#if (defined JENNIC_CHIP_FAMILY_JN516x)

    #define WRITE_REG32(A, B) *(volatile uint32 *)(A) = (B)
    #define WRITE_REG16(A, B) *(volatile uint16 *)(A) = (B)
    #define WRITE_REG8(A, B)  *(volatile uint8 *)(A) =(B)
    #define READ_REG32(A)     *(volatile uint32 *)(A)
    #define READ_REG16(A)     *(volatile uint16 *)(A)
    #define READ_REG8(A)      *(volatile uint8 *)(A)
    #define RMW_REG32(A, B, C) vRMW_REG32(A,B,C)

	#define TXPOWERADJUST

#endif

#if (defined JENNIC_CHIP_JN5169)
#define FULLTXPOWERADJUST
#endif
#define ADDR_PHY_IS           	0x02001E4c
#define ADDR_BBC_TXMBEBT    	0x020015c8
#define ADDR_BBC_ISR        	0x2001440
#define ADDR_BBC_TXSTAT     	0x20015c4

#define E_JPT_MODE_MO5_HIPOWER (25)
#define E_JPT_MODE_MO6_HIPOWER (26)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vDoTriggerPacketTest(void);
PRIVATE void vPerformTrigPacketTest(uint32 u32Repetitions, uint32 u32TriggerDelay,
                                    uint32 *pu32Seen, uint32 *pu32Total,
                                    uint32 *pu32Errors);

PRIVATE void vDoPowerTestSubMenu(uint32 u32Mode);
PRIVATE void vDoPowerTest(uint32 u32Mode, uint8 u8DutyCycle, bool_t bFinePowerAdjust);
PRIVATE void vTimer0ISR(uint32 u32DeviceId, uint32 u32ItemBitmap);
//PRIVATE void vTimer1ISR(uint32 u32DeviceId, uint32 u32ItemBitmap);

PRIVATE void vDoPowerMeter(void);
PRIVATE void vDoReceiveTest(void);

PRIVATE void vDoReceivePacketsTest(bool_t LQIonly);
PRIVATE void vDisplayRxPacketInfo(uint32 u32Timestamp, tsJPT_PT_Packet *psPacket  );

PRIVATE void vDoTransmitPacketsTest(void);

PRIVATE void vDoFrequencyTest(void);
PRIVATE void vDoCurrentTest(void);
PRIVATE void vDoConnectionLessPerTest(void);
PRIVATE void vDoConnectionlessPerTestMaster(void);
PRIVATE void vDoConnectionlessPerTestSlave(void);

PRIVATE void vDoCCATest(void);

PRIVATE uint8 u8ChangeChannel(uint8 u8Key);
PRIVATE uint8 u8ChangePower(uint8 u8Key);
PRIVATE uint8 u8ChangePowerFine(uint8 u8Key);
PRIVATE uint32 u32IncDec32(uint8 u8Key, uint32 u32Value, uint32 u32Modifier,
                            uint32 u32Min, uint32 u32Max, uint8 u8IncKeyA,
                            uint8 u8IncKeyB, uint8 u8DecKeyA, uint8 u8DecKeyB);

PRIVATE void vPutC(uint8 u8Data);
PRIVATE char acGetC(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

uint64 u64MacAddress = 0;
uint32 u32MacAddressHigh = 0;
uint32 u32MacAddressLow = 0;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/



#if 0
    static uint8 au8Uart0TxBuffer[16];
    static uint8 au8Uart0RxBuffer[16];

    static uint8 au8Uart1TxBuffer[16];
    static uint8 au8Uart1RxBuffer[16];

#endif


    uint8 au8UartTxBuffer[100];
    uint8 au8UartRxBuffer[100];


char acFrequencies[][10] = {{"2.405 GHz\0"},
                            {"2.410 GHz\0"},
                            {"2.415 GHz\0"},
                            {"2.420 GHz\0"},
                            {"2.425 GHz\0"},
                            {"2.430 GHz\0"},
                            {"2.435 GHz\0"},
                            {"2.440 GHz\0"},
                            {"2.445 GHz\0"},
                            {"2.450 GHz\0"},
                            {"2.455 GHz\0"},
                            {"2.460 GHz\0"},
                            {"2.465 GHz\0"},
                            {"2.470 GHz\0"},
                            {"2.475 GHz\0"},
                            {"2.480 GHz\0"}};

uint32 u32RadioMode     = E_JPT_MODE_LOPOWER;
uint32 u32ModuleRadioMode     = E_JPT_MODE_LOPOWER;

uint8 u8MaxTxPower      = 5;
uint8 u8MaxTxPowerFine  = 47;

#ifdef TXPOWERADJUST
uint8 u8TxPowerAdj	=0;
uint8 u8Attenuator3dB =0;
uint8 u8PowerAdjustChannel = 11;
#endif

uint16 u16OnTime        = 0xffff;
uint16 u16OffTime       = 0xffff;

uint32 u32PowerTestMode = E_JPT_TXPT_RUN_CW;

uint32 u32ClkMultiply   = 1;

uint32 pin_sg_trigger = PIN_SG_TRIGGER;


PUBLIC void vDebug(char *pcMessage)
{
    while (*pcMessage)
    {
        while ((u8AHI_UartReadLineStatus(0) & 0x20) == 0);
        vAHI_UartWriteData(0, *pcMessage);
        pcMessage++;
    }
}

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void AppColdStart(void)
{

    char   acCommand = 0;
    bool_t bExitLoop = FALSE;
    uint32 u32JPT_Ver = 0;
    uint32 u32JPT_RadioModes = 0;
    uint32 u32Chip_Id = 0;
#if (defined JENNIC_CHIP_JN5169)
    uint32 u32RadioParamVersion;
#endif

    // Menu keystokes
    uint8  u8LPKey = 0;
    uint8  u8LPBKey = 0;
    uint8  u8LP0Key = 0;
    uint8  u8HP5Key = 0;
    uint8  u8HP6Key = 0;
    uint8  u8HPEKey = 0;
    uint8  u8HPE6Key = 0;

#ifndef JENNIC_CHIP_FAMILY_JN516x       // reqd for crystal delay loop
    volatile int n;
#endif

#ifdef JENNIC_CHIP_FAMILY_JN516x

    /* Turn off debugger */
    *(volatile uint32 *)0x020000a0 = 0;
#endif

    /* Disable watchdog if enabled by default */
#ifdef WATCHDOG_ENABLED
    vAHI_WatchdogStop();
#endif


    u32JPT_Ver = u32JPT_Init();                 /* initialise production test API */
#if (defined JENNIC_CHIP_JN5169)
#ifdef SDK4168_COMPATIBILITY
    vJPT_GetRadioConfig(&u32RadioParamVersion, &u8LPKey);
#else
    vJPT_GetRadioConfig(&u32RadioParamVersion);
#endif
#endif

    u32JPT_RadioModes = u32JPT_RadioModesAvailable(); /* Get the Modes supported by this device */
    u32AHI_Init();                              /* initialise hardware API */


#ifndef JENNIC_CHIP_FAMILY_JN516x
    for(n=0;n<100000;n++);      // wait for JN516X to move onto 32MHz Crystal
#endif

    vUartInit(UART_TO_PC, BAUD_RATE, au8UartTxBuffer, sizeof(au8UartTxBuffer), au8UartRxBuffer, sizeof(au8UartRxBuffer));/* uart for user interface */

    vInitPrintf((void*)vPutC);

    vAHI_DioSetPullup(0xffffffff, 0x00000000);  /* turn all pullups on      */

    // Default to DIO2 and DIO3 output low
    vAHI_DioSetDirection(0,0xc);    // set DIO2&3 to output
    vAHI_DioSetOutput(0,0);

    /* read Chip_ID register */
    u32Chip_Id= READ_REG32(0x020000fc);


    /* Get module type, low or high power */
    while(!bExitLoop){
        uint8 u8MenuCtr=97;   // ASCII 'a'
        u8LPKey = 0;
        u8LPBKey = 0;
        u8LP0Key = 0;
        u8HP5Key = 0;
        u8HP6Key = 0;
        u8HPEKey = 0;
        u8HPE6Key = 0;

        vPrintf("\n*********************************************"
                "\n*    Customer Module Evaluation Tool        *"
                "\n*    Version %2d.%02d                          *"
                "\n*    Compiled %s %s          *"
                "\n*    Production Test API Version %08x *"
                "\n*    Chip ID %08x                     *"
                "\n*********************************************\n", MAJOR_VERSION_NO, MINOR_VERSION_NO, __DATE__, __TIME__, u32JPT_Ver, u32Chip_Id);
       // vPrintf("RadioMode:%x",u32JPT_RadioModes);

#if (defined JENNIC_CHIP_JN5169)
        vPrintf("\n*********************************************"
                "\n*    Customer Module Evaluation Tool        *"
                "\n*    Radio Params %08x                *"
                "\n*********************************************\n", u32RadioParamVersion);
#endif


        if (u32JPT_RadioModes & (1<<E_JPT_MODE_LOPOWER)){
            u8LPKey = u8MenuCtr;
            vPrintf("\n%c) Standard Module",u8MenuCtr++);
        }
        if (u32JPT_RadioModes & (1<<E_JPT_MODE_BOOST)){
            u8LPBKey = u8MenuCtr;
            vPrintf("\n%c) Standard Module (Boost Mode)",u8MenuCtr++);
        }
        if (u32JPT_RadioModes & (1<<E_JPT_MODE_0DBM)){
            u8LP0Key = u8MenuCtr;
            vPrintf("\n%c) Standard Module (0dBm  Mode)",u8MenuCtr++);
        }
#if (!defined JENNIC_CHIP_JN5169)
        if (u32JPT_RadioModes & (1<<E_JPT_MODE_HIPOWER)){
            u8HP5Key = u8MenuCtr;
            vPrintf("\n%c) MO5 High Power Module",u8MenuCtr++);
        }
#endif
        if (u32JPT_RadioModes & (1<<E_JPT_MODE_HIPOWER)){
            u8HP6Key = u8MenuCtr;
            vPrintf("\n%c) MO6 High Power Module",u8MenuCtr++);
        }
        if (u32JPT_RadioModes & (1<<E_JPT_MODE_ETSI)){
            u8HPEKey = u8MenuCtr;
            vPrintf("\n%c) High Power Module (ETSI-M04 Mode)",u8MenuCtr++);
        }
        if (u32JPT_RadioModes & (1<<E_JPT_MODE_ETSIM06)){
            u8HPE6Key = u8MenuCtr;
            vPrintf("\n%c) High Power Module (ETSI-M06 Mode)",u8MenuCtr++);
        }
        vPrintf("\n\nPlease choose an option > ");

        acCommand = acGetC();
        if (acCommand != 0){
         vPutC(acCommand);

         if ((acCommand == u8LPKey) || ((acCommand|0x20) == u8LPKey)){
                /* Low Power Module */
                vPrintf(" Standard Module Selected\n");
                u32RadioMode = E_JPT_MODE_LOPOWER;
                u32ModuleRadioMode = E_JPT_MODE_LOPOWER;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8LPBKey) || ((acCommand|0x20) == u8LPBKey)){
                /* Boost mode */
                vPrintf(" Boost Mode Selected\n");
                u32RadioMode = E_JPT_MODE_BOOST;
                u32ModuleRadioMode = E_JPT_MODE_BOOST;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8LP0Key) || ((acCommand|0x20) == u8LP0Key)){
                /* 0dBm mode */
                vPrintf(" 0Db Mode Selected\n");
                u32RadioMode = E_JPT_MODE_0DBM;
                u32ModuleRadioMode = E_JPT_MODE_0DBM;
                u8MaxTxPower = 3;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8HP5Key) || ((acCommand|0x20) == u8HP5Key)){
                /* High Power Module */
                vPrintf(" MO5 HP Mode Selected\n");
                u32RadioMode = E_JPT_MODE_HIPOWER;
                u32ModuleRadioMode = E_JPT_MODE_MO5_HIPOWER;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8HP6Key) || ((acCommand|0x20) == u8HP6Key)){
                /* High Power Module */
                vPrintf(" MO6 HP Mode Selected\n");
                u32RadioMode = E_JPT_MODE_HIPOWER;
                u32ModuleRadioMode = E_JPT_MODE_MO6_HIPOWER;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8HPEKey) || ((acCommand|0x20) == u8HPEKey)){
                /* High Power ETSI (M04) Module */
                vPrintf(" HP-M04 (ETSI) Mode Selected\n");
                u8MaxTxPower = 2;
                u32RadioMode = E_JPT_MODE_ETSI;
                u32ModuleRadioMode = E_JPT_MODE_ETSI;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8HPE6Key) || ((acCommand|0x20) == u8HPE6Key)){
                /* High Power ETSI (M06) Module */
                vPrintf(" HP-M06 (ETSI) Mode Selected\n");
                u32RadioMode = E_JPT_MODE_ETSIM06;
                u32ModuleRadioMode = E_JPT_MODE_ETSIM06;
               // u8MaxTxPower = 1;
                bExitLoop = TRUE;
         }
      }
    }

    bExitLoop = FALSE;

#ifdef TXPOWERADJUST
#ifdef FULLTXPOWERADJUST

    /* Get TX power Adust */
    while(!bExitLoop){
    	uint8 u8MenuCtr=97;   // ASCII 'a'
    	u8LPKey = 0;
    	u8LPBKey = 0;
    	u8LP0Key = 0;
    	u8HP5Key = 0;
    	u8HPEKey = 0;
    	u8HPE6Key = 0;

        vPrintf("\n*********************************************"
                "\n*           Tx Power Adjustement            *"
                "\n*********************************************\n");
       // vPrintf("RadioMode:%x",u32JPT_RadioModes);

        	u8LPKey = u8MenuCtr;
        	vPrintf("\n%c) Default Tx Power",u8MenuCtr++);
        	u8LPBKey = u8MenuCtr;
        	vPrintf("\n%c) Default Tx Power +0,8dB",u8MenuCtr++);
        	u8LP0Key = u8MenuCtr;
        	vPrintf("\n%c) Default Tx Power +1,2dB",u8MenuCtr++);
        	u8HP5Key = u8MenuCtr;
        	vPrintf("\n%c) Default Tx Power +1,6dB",u8MenuCtr++);
        vPrintf("\n\nPlease choose an option > ");

        acCommand = acGetC();
        if (acCommand != 0){
         vPutC(acCommand);

         if ((acCommand == u8LPKey) || ((acCommand|0x20) == u8LPKey)){
                /* Default */
        	    vPrintf(" Default Tx Power Selected\n");
        	    u8TxPowerAdj = 0;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8LPBKey) || ((acCommand|0x20) == u8LPBKey)){
                /* +0,8 */
				vPrintf(" Default Tx Power +0,8dB Selected\n");
        	    u8TxPowerAdj = 1;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8LP0Key) || ((acCommand|0x20) == u8LP0Key)){
                /* +1,2 */
     	        vPrintf(" Default Tx Power +1,2dB Selected\n");
        	    u8TxPowerAdj = 2;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8HP5Key) || ((acCommand|0x20) == u8HP5Key)){
                /* +1,6 */
  	            vPrintf(" Default Tx Power +1,6dB Selected\n");
        	    u8TxPowerAdj = 3;
                bExitLoop = TRUE;
         }
      }
    }

    bExitLoop = FALSE;
#endif // FULLTXPOWERADJUST

    /* Get Attenuator Setting */
    while(!bExitLoop){
    	uint8 u8MenuCtr=97;   // ASCII 'a'
    	u8LPKey = 0;
    	u8LPBKey = 0;
    	u8LP0Key = 0;
    	u8HP5Key = 0;
    	u8HPEKey = 0;
    	u8HPE6Key = 0;

        vPrintf("\n*********************************************"
                "\n*             TX 2.5dB Attenuator           *"
                "\n*********************************************\n");
       // vPrintf("RadioMode:%x",u32JPT_RadioModes);

        	u8LPKey = u8MenuCtr;
        	vPrintf("\n%c) 2.5dB Attenuator Off",u8MenuCtr++);
        	u8LPBKey = u8MenuCtr;
        	vPrintf("\n%c) 2.5dB Attenuator On",u8MenuCtr++);
        vPrintf("\n\nPlease choose an option > ");

        acCommand = acGetC();
        if (acCommand != 0){
         vPutC(acCommand);

         if ((acCommand == u8LPKey) || ((acCommand|0x20) == u8LPKey)){
                /* No Attenuator */
        	    vPrintf(" 2.5dB Attenuator Off Selected\n");
        	    u8Attenuator3dB = 0;
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8LPBKey) || ((acCommand|0x20) == u8LPBKey)){
                /* Attenuator 3dB */
				vPrintf(" 2.5dB Attenuator On Selected\n");
				u8Attenuator3dB = 1;
                bExitLoop = TRUE;
         }
      }
    }

    bExitLoop = FALSE;

#endif // TXPOWERADJUST

#ifdef RXPOWERADJUST_SUPPORT
    /* Get TX power Adust */
    while(!bExitLoop){
    	uint8 u8MenuCtr=97;   // ASCII 'a'
    	u8LPKey = 0;
    	u8HP5Key = 0;

        vPrintf("\n*********************************************"
                "\n*           RX Maximum Input Level         *"
                "\n*********************************************\n");
       // vPrintf("RadioMode:%x",u32JPT_RadioModes);

        	u8LPKey = u8MenuCtr;
        	vPrintf("\n%c) RX Maximum Input Level +10dBm",u8MenuCtr++);
        	u8HP5Key = u8MenuCtr;
        	vPrintf("\n%c) RX Maximum Input Level 0dBm (with reduced power consumption)",u8MenuCtr++);
        vPrintf("\n\nPlease choose an option > ");

        acCommand = acGetC();
        if (acCommand != 0){
         vPutC(acCommand);

         if ((acCommand == u8LPKey) || ((acCommand|0x20) == u8LPKey)){
                /* Power Saver mode Off */
        	    vPrintf(" RX Maximum Input Level +10dBm Selected\n");
        	    vJPT_SetMaxInputLevel(E_MAX_INP_LEVEL_10dB);
                bExitLoop = TRUE;
         }
         else if ((acCommand == u8HP5Key) || ((acCommand|0x20) == u8HP5Key)){
                /* Power Saver Mode On */
				vPrintf(" RX Maximum Input Level 0dBm Selected\n");
				vJPT_SetMaxInputLevel(E_MAX_INP_LEVEL_0dB);
                bExitLoop = TRUE;
         }
      }
    }

    bExitLoop = FALSE;

#endif


#if 0
    /* Get module type, low or high power */
    while(!bExitLoop){

        vPrintf("\n*********************************************"
                "\n*        Select CPU Clock Frequency         *"
                "\n*********************************************\n"
                "\na) 16MHz"
                "\nb) 32MHz (Not JN5121)"
                "\n\nPlease choose an option >");

        acCommand = acGetC();
        vPutC(acCommand);

        switch(acCommand) {

            case 'a':                               /* 16MHz */
            case 'A':
                u32ClkMultiply = 1;
                bExitLoop = TRUE;
                break;


            case 'b':                               /* 32MHz if JN5139R1 */
            case 'B':
#if (defined JN5139R1) || (defined JN5139)
                vAHI_CPUClockDoublerEnable(TRUE);
                u32ClkMultiply = 2;
#else
                u32ClkMultiply = 1;
#endif
                bExitLoop = TRUE;
                break;

        }

    }
#endif

    /* Sit in loop forever */
    while (1) {


        vPrintf("\n****************************************"
                "\n*   Customer Module Evaluation Tool    *"
                "\n****************************************\n"
                "\na) TX Power Test (CW)"
                "\nb) TX Power Test (Modulated)"
                "\nc) Receive Test"
                "\nd) Oscillator Frequency Test"
                "\ne) Current Measurement Test"
                "\nf) RF Power Measurement"
                "\ng) Trigger Packet Test"
                "\nh) Receive Packets Test"
                "\ni) Transmit Packets Test"
                "\nj) Connectionless Packet Error Rate Test"
                "\nk) CCA Test"
                "\nl) LQI Test"
                "\n\nPlease choose an option >");

        /*        "\nd) Packet Error Rate Test" PER Moved to seperate app */


        acCommand = acGetC();
        vPutC(acCommand);

        switch(acCommand) {

            case 'a':                               /* CW Power Test */
            case 'A':
                vDoPowerTestSubMenu(E_JPT_TXPT_RUN_CW);
                break;


            case 'b':                               /* Modulated Power Test */
            case 'B':
                vDoPowerTestSubMenu(E_JPT_TXPT_RUN_PRBS);
                break;


            case 'c':                               /* Receive mode test */
            case 'C':
                vDoReceiveTest();
                break;

/* PER Test moved to seperate app */
/*            case 'd':                          */     /* PER Test */
/*            case 'D': */
                /*vDoPerTest();*/
/*                break; */


            case 'd':                               /* Frequency Test */
            case 'D':
                vDoFrequencyTest();
                break;


            case 'e':                               /* Current Measurement */
            case 'E':
                vDoCurrentTest();
                break;


            case 'f':                               /* RF Power Measurement */
            case 'F':
                vDoPowerMeter();
                break;


            case 'g':                               /* Trigger packet test */
            case 'G':
                vDoTriggerPacketTest();
                break;


            case 'h':                               /* Receive packet test */
            case 'H':
                vDoReceivePacketsTest(FALSE);
                break;


            case 'i':                               /* Transmit packet test */
            case 'I':
                vDoTransmitPacketsTest();
                break;


            case 'j':                               /* Connectionless PER Test */
            case 'J':
                vDoConnectionLessPerTest();
                break;


            case 'k':                               /* CCA Test */
            case 'K':
                vDoCCATest();
                break;

            case 'l':                               /* LQI Test */
            case 'L':
                vDoReceivePacketsTest(TRUE);
                break;


        }

    }

}


PUBLIC void AppWarmStart(void)
{
    AppColdStart();
}


/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME:       vDoTriggerPacketTest
 *
 * DESCRIPTION:
 * Generates pulses on an IO pin to trigger an arbitary waveform generator
 * (ARB) to transmit a stored ideal packet. This function then looks for the
 * packets that were transmitted and displays the results.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoTriggerPacketTest(void)
{
    static uint32 u32TriggerDelay = 1;
    static uint32 u32Repetitions = 100;

    uint32 u32Seen;
    uint32 u32Total;
    uint32 u32Errors;
    uint32 i;
    uint32 u32Throughput;
    uint32 u32Remainder;

    char acKey = 0;
    uint8 u8Channel = 0;
    uint8 DIOnum = 8;
    bool_t bDoTest = TRUE;

    /* enable protocol */
    bJPT_RadioInit(u32RadioMode);

    /* Select Trigger Pin */
    while(bDoTest)
    {
    	vPrintf("\n Enter Trigger DIO in Hexadecimal [0..F] (by default choose 8)");
    	acKey = acGetC();
        if (acKey != 0)
        {
            vPutC(acKey);
            switch (acKey)
            {
            case '0':
            	pin_sg_trigger = (1 << 0);
            	DIOnum = 0;
            	bDoTest = FALSE;
            	break;
            case '1':
            	pin_sg_trigger = (1 << 1);
            	DIOnum = 1;
            	bDoTest = FALSE;
            	break;
            case '2':
            	pin_sg_trigger = (1 << 2);
            	DIOnum = 2;
            	bDoTest = FALSE;
            	break;
            case '3':
            	pin_sg_trigger = (1 << 3);
            	DIOnum = 3;
            	bDoTest = FALSE;
            	break;
            case '4':
            	pin_sg_trigger = (1 << 4);
            	DIOnum = 4;
            	bDoTest = FALSE;
            	break;
            case '5':
            	pin_sg_trigger = (1 << 5);
            	DIOnum = 5;
            	bDoTest = FALSE;
            	break;
            case '6':
            	pin_sg_trigger = (1 << 6);
            	DIOnum = 6;
            	bDoTest = FALSE;
            	break;
            case '7':
            	pin_sg_trigger = (1 << 7);
            	DIOnum = 7;
            	bDoTest = FALSE;
            	break;
            case '8':
            	pin_sg_trigger = (1 << 8);
            	DIOnum = 8;
            	bDoTest = FALSE;
            	break;
            case '9':
            	pin_sg_trigger = (1 << 9);
            	DIOnum = 9;
            	bDoTest = FALSE;
            	break;
            case 'a':
            case 'A':
            	pin_sg_trigger = (1 << 10);
            	DIOnum = 10;
            	bDoTest = FALSE;
            	break;
            case 'b':
            case 'B':
            	pin_sg_trigger = (1 << 11);
            	DIOnum = 11;
            	bDoTest = FALSE;
            	break;
            case 'c':
            case 'C':
            	pin_sg_trigger = (1 << 12);
            	DIOnum = 12;
            	bDoTest = FALSE;
            	break;
            case 'd':
            case 'D':
            	pin_sg_trigger = (1 << 13);
            	DIOnum = 13;
            	bDoTest = FALSE;
            	break;
            case 'e':
            case 'E':
            	pin_sg_trigger = (1 << 14);
            	DIOnum = 14;
            	bDoTest = FALSE;
            	break;
            case 'f':
            case 'F':
            	pin_sg_trigger = (1 << 15);
            	DIOnum = 15;
            	bDoTest = FALSE;
           	break;
            case 'g':
            case 'G':
            	pin_sg_trigger = (1 << 16);
            	DIOnum = 16;
            	bDoTest = FALSE;
           	break;
            case 'h':
            case 'H':
            	pin_sg_trigger = (1 << 17);
            	DIOnum = 17;
            	bDoTest = FALSE;
           	break;
            case 'i':
            case 'I':
            	pin_sg_trigger = (1 << 18);
            	DIOnum = 18;
            	bDoTest = FALSE;
           	break;
            case 'j':
            case 'J':
            	pin_sg_trigger = (1 << 19);
            	DIOnum = 19;
            	bDoTest = FALSE;
           	break;
            case 'k':
            case 'K':
            	pin_sg_trigger = (1 << 20);
            	DIOnum = 20;
            	bDoTest = FALSE;
           	break;
            default:
            	pin_sg_trigger = (1 << 8);
            }

        }
    }

	bDoTest = TRUE;


    while(bDoTest == TRUE){

        u8Channel = u8ChangeChannel(acKey);
        bJPT_RadioSetChannel(u8Channel);

        if(u32TriggerDelay < 10){
            u32TriggerDelay = u32IncDec32(acKey, u32TriggerDelay, 1, 1, 200,'.','>',',','<');
        } else {
            u32TriggerDelay = u32IncDec32(acKey, u32TriggerDelay, 10, 9, 200,'.','>',',','<');
        }

        if ((u32Repetitions == 100) && ((acKey == '[') || (acKey == '{'))) {
        	u32Repetitions = 1;
        }
        else
            if ((u32Repetitions == 1) && ((acKey == ']') || (acKey == '}'))){
            	u32Repetitions = 100;
            }
            else
        if (u32Repetitions < 1000){
            u32Repetitions = u32IncDec32(acKey, u32Repetitions, 100, 100, 100000, ']','}','[','{');
        } else {
            u32Repetitions = u32IncDec32(acKey, u32Repetitions, 1000, 900, 100000, ']','}','[','{');
        }


        vPrintf("\n***************************************"
                "\n*        Trigger Packet Test          *"
                "\n***************************************"
                "\n* Key        Function                 *"
                "\n*                                     *"
                "\n*  +      Increment Channel           *"
                "\n*  -      Decrement Channel           *"
                "\n*  ]      Increment Repetitions       *"
                "\n*  [      Decrement Repetitions       *"
                "\n*  >      Increase Trigger Delay      *"
                "\n*  <      Decrease Trigger Delay      *"
                "\n*  s      Start Test                  *"
                "\n*  x      Return to main menu         *"
                "\n*                                     *"
                "\n* Note:                               *");
        /* manage number of spaces at the end of the line depending of DIOnum (one or two digits) */
        if (DIOnum>=10)
        {
        	vPrintf("\n* Connect pin DIO%d to the trigger    *", DIOnum);
        	vPrintf("\n*  !!!!! Trig on RAISING edge !!!!!   *");

        }
        else
        {
        	vPrintf("\n* Connect pin DIO%d to the trigger     *", DIOnum);
        	vPrintf("\n*  !!!!! Trig on RAISING edge !!!!!   *");

        }

        vPrintf("\n* input on the signal generator       *"
                "\n***************************************\n"
                "\nChannel       %d    (%s)"
                "\nRepetitions   %d"
                "\nTrigger delay %d mS"
                "\n",u8Channel, acFrequencies[u8Channel - 11], u32Repetitions, u32TriggerDelay);


        acKey = acGetC();

        if(acKey == 's' || acKey == 'S'){
            u32Seen = 0;
            u32Total = 0;
            u32Errors = 0;

            vPrintf("\n***************************************"
                    "\n*    Running Trigger Packet Test      *"
                    "\n***************************************");


            vPrintf("\n\nPerforming %d repetitions...",u32Repetitions);

            vPerformTrigPacketTest(u32Repetitions, u32TriggerDelay,  &u32Seen, &u32Total, &u32Errors);

            i = (u32Seen * 1000) / u32Total;

            u32Throughput = i / 10;
            u32Remainder = i % 10;


            vPrintf("Done\n\nTotal Packets = %d"
                          "\nPackets Seen  = %d"
                          "\nChip Errors   = %d"
                          "\nThroughput    = %d.%d%%",u32Total, u32Seen, u32Errors, u32Throughput, u32Remainder);

            vPrintf("\n\nPress any key to continue");

            acGetC();

        }

        if(acKey == 'x' || acKey == 'X'){
            bDoTest = FALSE;
        }

    }

    /* disable protocol */
    vJPT_RadioDeInit();

}


/****************************************************************************
 *
 * NAME:       vPerformTrigPacketTest
 *
 * DESCRIPTION:
 * Sends a trigger to start external packet source, times out and checks if
 * packet received. Repeats for a set number of times or until first packet
 * lost. Can send back a packet containing status information after each
 * packet for logging on the PC.
 *
 * Trigger is either a DIO pin if used on daughter card or one of the LEDs
 * if used on the HDK.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32Repetitions  R   Number of frames to look for, 0 if
 *                                      should stop after first missed packet
 *                  pu32Seen        R   Number of frames seen
 *                  pu32Total       R   Total number of frames during observation
 *                  pu32Errors      R   Total errors seen
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
 PRIVATE void vPerformTrigPacketTest(uint32 u32Repetitions, uint32 u32TriggerDelay,
                                    uint32 *pu32Seen, uint32 *pu32Total, uint32 *pu32Errors)
 {
    uint32 u32Total = 0;
    uint32 u32Seen = 0;
    uint32 u32Errors = 0;
    volatile int i;

    uint32 u32Time;
    uint32 u32TimeoutTime = 300000; /* Initial value, gets reduced adaptively */

    /* Don't claim DIO pins for timer */
#if (defined JN5142J01) || (defined JN5142)
    vAHI_TimerFineGrainDIOControl(0xff);
#else
 #if (defined JENNIC_CHIP_FAMILY_JN514x) || (defined JENNIC_CHIP_FAMILY_JN516x)
    vAHI_TimerDIOControl(E_AHI_TIMER_0, FALSE);
 #endif
#endif

    /* set up a timer, prescale = 1 */
    vAHI_TimerEnable(E_AHI_TIMER_0, 6, FALSE, FALSE, FALSE);

    /* use internal clock, gated on high signal */
    vAHI_TimerClockSelect(E_AHI_TIMER_0, FALSE, TRUE);

     /* Enable DIO for sig gen trigger, if not already enabled */
    vAHI_DioSetDirection(0x00000000, pin_sg_trigger);

     /* Clear bit used for trigger and wait a second for it to take effect
        and any frame in progress to clear */
    vAHI_DioSetOutput(0x00000000, pin_sg_trigger);
    for (i = 0; i < 3000000 * u32ClkMultiply; i++);

    vJPT_TPT_Start();

    do {

//         /* Set and clear trigger */
//        vAHI_DioSetOutput(pin_sg_trigger, 0x00000000); /* set trigger high */
//         for (i = 0; i < 1000 * u32ClkMultiply; i++);
//        vAHI_DioSetOutput(0x00000000, pin_sg_trigger); /* set trigger low */

#if (!defined JENNIC_CHIP_JN5147) && (!defined JENNIC_CHIP_JN5142) && (!defined JENNIC_CHIP_FAMILY_JN516x)
    	/* start single shot timer and wait for timeout */
        vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, u32TriggerDelay * 250);
        while(!(u8AHI_TimerFired(E_AHI_TIMER_0) & E_AHI_TIMER_INT_PERIOD));
        vAHI_TimerStop(E_AHI_TIMER_0);
#endif

        /* Set trigger */
       vAHI_DioSetOutput(pin_sg_trigger, 0x00000000); /* set trigger high */
        u32Time = u32JPT_TPT_WaitPacket(u32TimeoutTime, &u32Total, &u32Seen, &u32Errors);
        /* Clear trigger */
        vAHI_DioSetOutput(0x00000000, pin_sg_trigger); /* set trigger low */

        /* if we didn't time out, update the timeout time to a sensible value */
        if(u32Time){
//            u32TimeoutTime = u32Time + 100;
            u32TimeoutTime -= (u32Time - 1000);
        }

    } while (u32Total != u32Repetitions);

    vJPT_TPT_Stop();

    /* Set return values */
    *pu32Seen = u32Seen;
    *pu32Total = u32Total;
    *pu32Errors = u32Errors;

}


/****************************************************************************
 *
 * NAME:       vDoPowerTestSubMenu
 *
 * DESCRIPTION:
 * Allows the user to choose the RF output duty cycle.
 *
 * PARAMETERS:  Name            RW  Usage
 *              u32Mode         R   Mode to use, E_JPT_TXPT_RUN_CW or
 *                                               E_JPT_TXPT_RUN_PRBS
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoPowerTestSubMenu(uint32 u32Mode)
{


    char   acCommand = 0;
    bool_t bExitLoop = FALSE;


    /* Get module type, low or high power */
    while(!bExitLoop){

        if (u32Mode == E_JPT_TXPT_RUN_CW)
        {

            vPrintf("\n********************************************************"
                    "\n*                    Tx Power Test                     *"
                    "\n********************************************************\n"
                    "\na) Output Continuous"
                    "\nx) Return to main menu"
                    "\n\nPlease choose an option >");
        }
        else    /* assume mode = E_JPT_TXPT_RUN_PRBS */
        {
            vPrintf("\n********************************************************"
                    "\n*                    Tx Power Test                     *"
                    "\n********************************************************\n"
                    "\na) Output Continuous"
                    "\nb) 50%% Duty Cycle"
                    "\nc) 30%% Duty Cycle"
                    "\nd) 1%% Duty Cycle"
                    "\nx) Return to main menu"
                    "\n\nPlease choose an option >");
        }

        acCommand = acGetC();
        vPutC(acCommand);

        switch(acCommand) {

            case 'a':
            case 'A':
                vDoPowerTest(u32Mode, DC_100, FALSE);
                bExitLoop = TRUE;
                break;
/*
            case 'b':
            case 'B':
                vDoPowerTest(u32Mode, DC_100, TRUE);
                bExitLoop = TRUE;
                break;
*/

            case 'b':
            case 'B':
                vDoPowerTest(u32Mode, DC_50, FALSE);
                bExitLoop = TRUE;
                break;


            case 'c':
            case 'C':
                vDoPowerTest(u32Mode, DC_30, FALSE);
                bExitLoop = TRUE;
                break;


            case 'd':
            case 'D':
                vDoPowerTest(u32Mode, DC_1, FALSE);
                bExitLoop = TRUE;
                break;

            case 'e':
            case 'E':
                vDoPowerTest(u32Mode, DC_VARIABLE, FALSE);
                bExitLoop = TRUE;
                break;

            case 'f':
            case 'F':
                vDoPowerTest(u32Mode, DC_VARIABLE, TRUE);
                bExitLoop = TRUE;
                break;

            case 'x':
            case 'X':
                bExitLoop = TRUE;
                break;

        }

    }

}


/****************************************************************************
 *
 * NAME:       vDoPowerTest
 *
 * DESCRIPTION:
 * Places the wireless microcontroller into a continuous transmit mode.
 * The RF signal can be either CW or PRBS modulated.
 *
 * PARAMETERS:  Name            RW  Usage
 *              u32Mode         R   Mode to use, E_JPT_TXPT_RUN_CW or
 *                                               E_JPT_TXPT_RUN_PRBS
 *              u8DutyCycle     R   Duty cycle to use, DC_100, DC_50, DC_30
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoPowerTest(uint32 u32Mode, uint8 u8DutyCycle, bool_t bFinePowerAdjust)
{
    char acKey = 0;
    uint8 u8Channel = 0, u8NewChannel;
    uint8 u8PowerLevel = 0, u8NewPowerLevel;
    uint16 u16Period;
    bool_t bDoTest = TRUE;

    u32PowerTestMode = u32Mode;

    u16Period = 8000;
    u16OnTime = 4000;
    u16OffTime = 4000;

    /* enable protocol */
    bJPT_RadioInit(u32RadioMode);

    /* set up interrupt handler for timer 1 */
    vAHI_Timer0RegisterCallback((void*)vTimer0ISR);

     /* Enable DIO for sig gen trigger, if not already enabled */
    vAHI_DioSetDirection(0x00000000, PIN_DEBUG);
    vAHI_DioSetOutput(PIN_DEBUG, 0x00000000); /* set trigger high */

    while(bDoTest == TRUE){

        /* disable protocol */

        u8NewChannel = u8ChangeChannel(acKey);

        if(!bFinePowerAdjust){
            u8NewPowerLevel = u8ChangePower(acKey);
        } else {
            u8NewPowerLevel = u8ChangePowerFine(acKey);
        }

        if((u8NewChannel != u8Channel) || (u8NewPowerLevel != u8PowerLevel)){

            u8Channel = u8NewChannel;
#ifdef TXPOWERADJUST
            u8PowerAdjustChannel = u8Channel;
#endif
            u8PowerLevel = u8NewPowerLevel;

            /* Request from HW designers: stop TxPowerTest before changing channel */
            vJPT_TxPowerTest(E_JPT_TXPT_STOP);
            /* End of request */

//#ifdef TXPOWERADJUST
//        	vJPT_TxPowerAdjust(u8TxPowerAdj, u8Attenuator3dB, u8PowerAdjustChannel);
//#endif
            bJPT_RadioSetChannel(u8Channel);
        }


        vPrintf("\n***************************************"
                "\n*       Power Test In Progress        *"
                "\n***************************************"
                "\n* Key        Function                 *"
                "\n*                                     *"
                "\n*  +      Increment Channel           *"
                "\n*  -      Decrement Channel           *"
                "\n*  <      Reduce output power         *"
                "\n*  >      Increase output power       *"
                "\n*  x      Return to main menu         *"
                "\n*                                     *"
                "\n***************************************\n"
                "\nChannel       %d    (%s)"
                "\nPower Level   %d",u8Channel, acFrequencies[u8Channel - 11], u8PowerLevel);


        switch(u8DutyCycle){

        case DC_100:
#ifdef TXPOWERADJUST
        	vJPT_TxPowerAdjust(u8TxPowerAdj, u8Attenuator3dB, u8PowerAdjustChannel);
#endif
            vJPT_TxPowerTest(u32Mode);
            break;

        case DC_50:
            u16OnTime  = 4000;
            u16OffTime = 8000 - 4000;

            /* set up a timer, prescale = 1 */
            vAHI_TimerEnable(E_AHI_TIMER_0, 4, FALSE, TRUE, FALSE);

            /* use internal clock, gated on high signal */
            vAHI_TimerClockSelect(E_AHI_TIMER_0, FALSE, TRUE);

            vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 8000);
            break;

        case DC_30:
            u16OnTime  = 2400;
            u16OffTime = 8000 - 2400;

            /* set up a timer, prescale = 1 */
            vAHI_TimerEnable(E_AHI_TIMER_0, 4, FALSE, TRUE, FALSE);

            /* use internal clock, gated on high signal */
            vAHI_TimerClockSelect(E_AHI_TIMER_0, FALSE, TRUE);

            vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 8000);
            break;

        case DC_1:
            if(u32RadioMode == E_JPT_MODE_LOPOWER){
                u16OnTime  = 80;
                u16OffTime = 8000 - 80;
            } else {
                u16OnTime  = 160;
                u16OffTime = 8000 - 160;
            }

            /* set up a timer, prescale = 1 */
            vAHI_TimerEnable(E_AHI_TIMER_0, 4, FALSE, TRUE, FALSE);

            /* use internal clock, gated on high signal */
            vAHI_TimerClockSelect(E_AHI_TIMER_0, FALSE, TRUE);

            vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 8000);
            break;

        case DC_VARIABLE:
            vPrintf("\n***************************************"
                    "\n*  m      Increment Period            *"
                    "\n*  n      Decrement Period            *"
                    "\n*  b      Increment On Time           *"
                    "\n*  v      Decrement On Time           *"
                    "\n***************************************"
                    "\nPeriod  = %d"
                    "\nOn Time = %d",u16Period, u16OnTime);

            switch(acKey){

            case 'm':
            case 'M':
                if(u16Period < 0xfff5) u16Period += 10;
                break;

            case 'n':
            case 'N':
                if(u16Period > 10) u16Period -= 10;
                break;

            case 'b':
            case 'B':
                if(u16OnTime < 0xfff5)  u16OnTime += 10;
                break;

            case 'v':
            case 'V':
                if(u16OnTime > 10) u16OnTime -= 10;
                break;

            }
            /* set up a timer, prescale = 1 */
            vAHI_TimerEnable(E_AHI_TIMER_0, 4, FALSE, TRUE, FALSE);

//          vAHI_TimerStop(E_AHI_TIMER_1);

            /* use internal clock, gated on high signal */
            vAHI_TimerClockSelect(E_AHI_TIMER_0, FALSE, TRUE);

            u16OffTime = u16Period - u16OnTime;

            vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 8000);
            break;


        }

        acKey = acGetC();

        if(u8DutyCycle != DC_100){
            vAHI_TimerStop(E_AHI_TIMER_0);
        }

        if(acKey == 'x' || acKey == 'X'){
            bDoTest = FALSE;
        }

    }

    if(u8DutyCycle != DC_100){
        vAHI_TimerStop(E_AHI_TIMER_0);
    }

    /* switch off test mode */
    vJPT_TxPowerTest(E_JPT_TXPT_STOP);

    /* disable protocol */
    vJPT_RadioDeInit();


    vAHI_DioSetOutput(0x00000000, PIN_DEBUG); /* set trigger low */

}
static bool bOn = FALSE;

PRIVATE void vTimer0ISR(uint32 u32DeviceId, uint32 u32ItemBitmap)
{

    if(u32ItemBitmap & E_AHI_TIMER_INT_PERIOD){
        if(bOn == TRUE){
            vJPT_TxPowerTest(E_JPT_TXPT_STOP);
            vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, u16OffTime);
            vAHI_DioSetOutput(0x00000000, PIN_DEBUG); /* set trigger low */
        } else {
#ifdef TXPOWERADJUST
        	vJPT_TxPowerAdjust(u8TxPowerAdj, u8Attenuator3dB, u8PowerAdjustChannel);
#endif
            vJPT_TxPowerTest(u32PowerTestMode);
            vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, u16OnTime);
            vAHI_DioSetOutput(PIN_DEBUG, 0x00000000); /* set trigger high */
        }

        bOn ^= TRUE;
    }

}

#if 0

// original version used timer1 for all devices..6x forced
// move to timer0 to maintain consistency across devices
PRIVATE void vTimer1ISR(uint32 u32DeviceId, uint32 u32ItemBitmap)
{

    static bool bOn = FALSE;

    uint8 u8Interrupts = u8AHI_TimerFired(E_AHI_TIMER_1);

    if(u8Interrupts & E_AHI_TIMER_INT_PERIOD){

        if(bOn == TRUE){
            vJPT_TxPowerTest(E_JPT_TXPT_STOP);
            vAHI_TimerStartSingleShot(E_AHI_TIMER_1, 0, u16OffTime);
            vAHI_DioSetOutput(0x00000000, PIN_DEBUG); /* set trigger low */
        } else {
            vJPT_TxPowerTest(u32PowerTestMode);
            vAHI_TimerStartSingleShot(E_AHI_TIMER_1, 0, u16OnTime);
            vAHI_DioSetOutput(PIN_DEBUG, 0x00000000); /* set trigger high */
        }

        bOn ^= TRUE;

    }

}
#endif
/****************************************************************************
 *
 * NAME:       vDoReceiveTest
 *
 * DESCRIPTION:
 * Puts the wireless microcontroller into receive mode and takes energy
 * detect measurements on the selected channel. Both the current value,
 * and highest detected value are displayed.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoReceiveTest(void)
{
    char acKey = 0;
    uint8 u8Channel;
    uint8 u8Channelsave = 0;
    uint8 u8Energy;

    int16 i16RSSI, i16PeakRSSI;
    bool_t bDoTest = TRUE;

    /* enable protocol */
    bJPT_RadioInit(u32RadioMode);

    while(bDoTest == TRUE){

        i16RSSI = -98;
        i16PeakRSSI = -98;

        u8Channel = u8ChangeChannel(acKey);

        if (u8Channel != u8Channelsave)
        {
        vPrintf("\n"
        		"\n***************************************"
                "\n*      Receive Test In Progress       *"
                "\n***************************************"
                "\n* Key        Function                 *"
                "\n*                                     *"
                "\n*  +      Increment Channel           *"
                "\n*  -      Decrement Channel           *"
                "\n*  x      Return to main menu         *"
                "\n*                                     *"
                "\n***************************************\n"
                "\nChannel       %d    (%s)"
                "\n",u8Channel, acFrequencies[u8Channel - 11]);
        }

        u8Channelsave = u8Channel;

        while(!bUartRxDataAvailable(UART_TO_PC)){

            /* use a wake timer to wait for 100 milliseconds */
            vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, FALSE);
            vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, 3200);

            u8Energy = u8JPT_EnergyDetect(u8Channel, 100);
            i16RSSI = i16JPT_ConvertEnergyTodBm(u8Energy);

            u8Energy = u8JPT_EnergyDetect(u8Channel, 1);

            // Correct for HP Modules LNA on RX (+12dB LNA)
            if ((u32ModuleRadioMode==E_JPT_MODE_MO6_HIPOWER) || (u32ModuleRadioMode==E_JPT_MODE_ETSI) || (u32ModuleRadioMode==E_JPT_MODE_ETSIM06)){
                i16RSSI -=12;
            }
            // Correct for HP Modules LNA on RX (+12dB LNA) and 7dB att on TX path (also on RX path)
            if (u32ModuleRadioMode==E_JPT_MODE_MO5_HIPOWER)
            {
                i16RSSI -=5;
            }

            if(i16RSSI > i16PeakRSSI) i16PeakRSSI = i16RSSI;

            vPrintf("\rSignal Level    %idBm    Peak %idBm  ED %d   ", i16RSSI, i16PeakRSSI, u8Energy);

            while(!(u8AHI_WakeTimerFiredStatus() & 1 << E_AHI_WAKE_TIMER_0));

        }


        acKey = acGetC();

        if(acKey == 'x' || acKey == 'X'){
            bDoTest = FALSE;
        }
        if ((acKey == '\n') || (acKey == '\r'))
        		vPrintf("\n");

    }

    /* disable protocol */
    vJPT_RadioDeInit();

}


/****************************************************************************
 *
 * NAME:       vDoReceivePacketsTest
 *
 * DESCRIPTION:
 * Receives data packets on the specified channel.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoReceivePacketsTest(bool_t LQIonly)
{

    char acKey = 0;
    uint8 u8Channel = 0;
    uint8 u8Channelsave = 0;
    bool_t bDoTest = TRUE;
    bool_t bPacketSeen = FALSE;
    uint32 u32Timestamp;
    tsJPT_PT_Packet sPacket;


    /* enable protocol */
    bJPT_RadioInit(u32RadioMode);

    /* set up the tick timer, we'll use it for timestamps */
    vAHI_TickTimerInit((void*)NULL);
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_CONT);
    vAHI_TickTimerInterval(0xffffffff);

    /* force channel change in bJPT_PacketRx */
    u8Channel = u8ChangeChannel(26);
    if (u8Channel != 26)
    {
    	bJPT_PacketRx(26, &sPacket);
    }
    else
    {
    	bJPT_PacketRx(11, &sPacket);
    }
    u8Channel = 11;
	bJPT_RadioSetChannel(u8Channel);

    while(bDoTest == TRUE){

        /* disable protocol */

        u8Channel = u8ChangeChannel(acKey);

        if (u8Channel != u8Channelsave)
        {
        vPrintf("\n"
        		"\n***************************************"
                "\n*   Receive Packet Test In Progress   *"
                "\n***************************************"
                "\n* Key        Function                 *"
                "\n*                                     *"
                "\n*  +      Increment Channel           *"
                "\n*  -      Decrement Channel           *"
                "\n*  x      Return to main menu         *"
                "\n*                                     *"
                "\n***************************************\n"
                "\nChannel       %d    (%s)"
                "\n",u8Channel, acFrequencies[u8Channel - 11]);

        		/*clear pending ITs */
         	 	 WRITE_REG32(ADDR_BBC_ISR, 0x3FF);

                 bPacketSeen = FALSE;

       }


        u8Channelsave = u8Channel;

        while(!bUartRxDataAvailable(UART_TO_PC)){

            if(bJPT_PacketRx(u8Channel, &sPacket)){

                /* create a timestamp */
                u32Timestamp = u32AHI_TickTimerRead();
#if (defined JENNIC_CHIP_FAMILY_JN514x) || (defined JENNIC_CHIP_FAMILY_JN516x)
                vAHI_TickTimerWrite(0x00000000);
#else
                vAHI_TickTimerClear();
#endif
                u32Timestamp >>= 4; /* quick divide by 16 to give uS */

                if (LQIonly)
                {
                    /* Show modem receive statistics */
#ifdef SDK4168_COMPATIBILITY
                    vPrintf("\rLQI=%d   ", ((sPacket.u8Energy < 8) ? 0:(sPacket.u8Energy -8)));
#else
                    vPrintf("\rLQI=%d   ",sPacket.u8LQI);
#endif

                }
                else
                {
                	vDisplayRxPacketInfo( u32Timestamp, &sPacket );
                }

                bPacketSeen = TRUE;

            }
            else
            {
                if (LQIonly && !bPacketSeen)
                {
                    vPrintf("\rWaiting...");

                }

        	}
        }


        acKey = acGetC();

        if(acKey == 'x' || acKey == 'X'){
            bDoTest = FALSE;
        }
        if (LQIonly)
        {
			if ((acKey == '\n') || (acKey == '\r'))
					vPrintf("\n");
        }

    }

    /* disable protocol */
    vJPT_RadioDeInit();

    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);

}


/****************************************************************************
 *
 * NAME:       vDisplayRxPacketInfo
 *
 * DESCRIPTION:
 * Displays information about the contents of a packet received.
 *
 * PARAMETERS:  Name            RW  Usage
 *              u32Timestamp    R   Value to be used as a timestamp
 *              psPacket        R   Pointer to a tsJPT_PT_Packet struct
 *                                  containing data about the packet.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDisplayRxPacketInfo(uint32 u32Timestamp, tsJPT_PT_Packet *psPacket )
{
	int HEX_Output 	= 0;
	int FCS			= 0;
	int FCS_Length 	= 0;

    int n;
    bool_t bSrcShortAddr = FALSE;
    bool_t bSrcExtAddr = FALSE;
    bool_t bDstShortAddr = FALSE;
    bool_t bDstExtAddr = FALSE;
    bool_t bIntraPan = FALSE;
    bool_t FrameAck = FALSE;

    //vPrintf("\nPKT %iuS - ",u32Timestamp);
    // uint32 secondSinceLastPacket = 0;
    // uint32 uSecondSinceLastPacket = 0;

    // secondSinceLastPacket = u32Timestamp / 1000 ;
    // uSecondSinceLastPacket = u32Timestamp - secondSinceLastPacket*1000;

    if ( FCS ) FCS_Length = 2; else FCS_Length = 0;



    if (HEX_Output)
    	vPrintf("\n00000 ");
    else {
    	/*
    	// Ce code est transfere dans le code python
    	// if (FirstPacket) {
    	if (0) {

    		// Entete du fichier
    		//typedef struct pcap_hdr_s {
    		//        guint32 magic_number;   / magic number /
    		//        guint16 version_major;  / major version number /
    		//        guint16 version_minor;  / minor version number /
    		//        gint32  thiszone;       / GMT to local correction /
    		//        guint32 sigfigs;        / accuracy of timestamps /
    		//        guint32 snaplen;        / max length of captured packets, in octets /
    		//        guint32 network;        / data link type /
    		//} pcap_hdr_t;
    		vPutC(0xd4); vPutC(0xc3); vPutC(0xb2); vPutC(0xa1);
    		vPutC(0x02); vPutC(0x00);
    		vPutC(0x04); vPutC(0x00);
    		vPutC(0x00); vPutC(0x00); vPutC(0x00); vPutC(0x00);
    		vPutC(0x00); vPutC(0x00); vPutC(0x00); vPutC(0x00);
    		vPutC(0xff); vPutC(0xff); vPutC(0x00); vPutC(0x00);
    		// C3 : LINKTYPE_IEEE802_15_4	    195	DLT_IEEE802_15_4	    IEEE 802.15.4 wireless Personal Area Network, with each packet having the FCS at the end of the frame.
    		// E6 : LINKTYPE_IEEE802_15_4_NOFCS	230	DLT_IEEE802_15_4_NOFCS	IEEE 802.15.4 wireless Personal Area Network, without the FCS at the end of the frame.
    		vPutC(0xc3); vPutC(0x00); vPutC(0x00); vPutC(0x00);

    	}
		*/

    	/*
    	// Packet fictif pour essayer
    	if ( 0 ) {
    		// Entete du packet
    		vPutC(0x3e);
    		vPutC(0xd7);
    		vPutC(0xe8);
    		vPutC(0x59);
    		vPutC(0x92);
    		vPutC(0x27);
    		vPutC(0x0f);
    		vPutC(0x00);

    		vPutC(0x2f);
    		vPutC(0x00);
    		vPutC(0x00);
    		vPutC(0x00);
    		vPutC(0x2f);
    		vPutC(0x00);
    		vPutC(0x00);
    		vPutC(0x00);

    		// Packet lui meme.
    		vPutC(0x41);
    		vPutC(0x88);
    		vPutC(0xb5);
    		vPutC(0x35);
    		vPutC(0xd4);
    		vPutC(0xff);
    		vPutC(0xff);
    		vPutC(0x00);

    		vPutC(0x00);
    		vPutC(0x09);
    		vPutC(0x12);
    		vPutC(0xfc);
    		vPutC(0xff);
    		vPutC(0x00);
    		vPutC(0x00);
    		vPutC(0x01);

    		vPutC(0x50);
    		vPutC(0xff);
    		vPutC(0x81);
    		vPutC(0x63);
    		vPutC(0x01);
    		vPutC(0x00);
    		vPutC(0x8d);
    		vPutC(0x15);

    		vPutC(0x00);
    		vPutC(0x28);
    		vPutC(0x1b);
    		vPutC(0x85);
    		vPutC(0x00);
    		vPutC(0x00);
    		vPutC(0xff);
    		vPutC(0x81);

    		vPutC(0x63);
    		vPutC(0x01);
    		vPutC(0x00);
    		vPutC(0x8d);
    		vPutC(0x15);
    		vPutC(0x00);
    		vPutC(0x01);
    		vPutC(0x92);

    		vPutC(0x26);
    		vPutC(0x14);
    		vPutC(0x1b);
    		vPutC(0xc4);
    		vPutC(0x68);
    		// A mettre si format de packet inclus FCS
    		vPutC(0xc3);
    		vPutC(0x1d);

    	}
		*/





    }

    // Packet by itself
/* ========================== Analyze Received Packet ==============================================*/
    if (psPacket->bPacketGood) {
		if(psPacket->u16FrameControl & 1 << 6){ bIntraPan = TRUE; }

		/* Source addressing mode */
		switch((psPacket->u16FrameControl & 3 << 14) >> 14){

			/* PAN id and address field not present */
			case 0:
				bSrcShortAddr = FALSE;
				bSrcExtAddr = FALSE;
				break;

			/* Reserved */
			case 1:
				// Je mets short mais je ne sais pas ce que cela doit etre
				bSrcShortAddr = FALSE;
				bSrcExtAddr = FALSE;
				break;

			/* Address field contains a 16 bit short address */
			case 2:
				// vPrintf("SSAD=%04x ",psPacket->u16SourceShortAddress);
				bSrcShortAddr = TRUE;
				bSrcExtAddr = FALSE;
				break;

			/* Address field contains a 64 bit extended address */
			case 3:
				// vPrintf("SEAD=%016lx ", psPacket->u64SourceExtendedAddress);
				bSrcShortAddr = FALSE;
				bSrcExtAddr = TRUE;
				break;
		}

		/* Destination addressing mode */
		switch((psPacket->u16FrameControl & 3 << 10) >> 10){

			/* PAN id and address field not present */
			case 0:
				bDstShortAddr = FALSE;
				bDstExtAddr = FALSE;
				break;

			/* Reserved */
			case 1:
				// Je mets short mais je ne sais pas ce que cela doit etre
				bDstShortAddr = FALSE;
				bDstExtAddr = FALSE;
				break;

			/* Address field contains a 16 bit short address */
			case 2:
				// vPrintf("DSAD=%04x ",psPacket->u16DestinationShortAddress);
				bDstShortAddr = TRUE;
				bDstExtAddr = FALSE;
				break;

			/* Address field contains a 64 bit extended address */
			case 3:
				// vPrintf("DEAD=%016lx ", psPacket->u64DestinationExtendedAddress);
				bDstShortAddr = FALSE;
				bDstExtAddr = TRUE;
				break;

			}

		/*
		if ( 0 )
		{
			// show good / bad status of packet
			if(psPacket->bPacketGood == FALSE){
				vPrintf("BAD");
			} else {
				vPrintf("OK ");
			}
		}
    	*/

		/* Show modem receive statistics */
		/*
			#ifdef SDK4168_COMPATIBILITY
			vPrintf("ED=%d SQI=%d  ", psPacket->u8Energy, psPacket->u8SQI);
			#else
			vPrintf("ED=%d SQI=%d LQI=%d  ", psPacket->u8Energy, psPacket->u8SQI, psPacket->u8LQI);
			#endif
		*/
    }

    /* look at frame type */
    switch(psPacket->u16FrameControl & 7){

		/* MAC Beacon Reply -----------------------------------------------------------------------------------------  */
		case 0:
			// vPrintf("BCN ");

			if ( (psPacket->u8PayloadLength!=0) && (psPacket->bPacketGood) ) {
				// Entete du packet


					// guint32 ts_sec;         /* timestamp seconds */
					// Tout à zero pour l instant. Devrait mettre la date plus tard.
					vPutC(0); vPutC(0); vPutC(0); vPutC(0);
					// vPutC( (secondSinceLastPacket>> 0)-((secondSinceLastPacket>> 0)<< 8) );
					// vPutC( (secondSinceLastPacket>> 8)-((secondSinceLastPacket>>16)<<16) );
					// vPutC( (secondSinceLastPacket>>16)-((secondSinceLastPacket>>24)<<24) );
					// vPutC( (secondSinceLastPacket>>24) );

					// guint32 ts_usec;        /* timestamp microseconds */
					// Tout à zero pour l instant. Devrait mettre la date plus tard.
					vPutC(0); vPutC(0); vPutC(0); vPutC(0);
					// vPutC(u32Timestamp-((u32Timestamp>>8)<<8));
					// vPutC(u32Timestamp>>8);
					// vPutC( (uSecondSinceLastPacket>> 0)-((uSecondSinceLastPacket>> 0)<< 8) );
					// vPutC( (uSecondSinceLastPacket>> 8)-((uSecondSinceLastPacket>>16)<<16) );
					// vPutC( (uSecondSinceLastPacket>>16)-((uSecondSinceLastPacket>>24)<<24) );
					// vPutC( (uSecondSinceLastPacket>>24) );

					// guint32 incl_len;       /* number of octets of packet saved in file */
					vPutC( psPacket->u8PayloadLength + 7 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);

					// guint32 orig_len;       /* actual length of packet */
					vPutC( psPacket->u8PayloadLength + 7 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);

				// MAC
					// FrameControl
					vPutC(psPacket->u16FrameControl-((psPacket->u16FrameControl>>8)<<8)); vPutC(psPacket->u16FrameControl>>8);
					// SequenceNumber
					vPutC(psPacket->u8SequenceNumber);
					// Source Pan ID
					vPutC(psPacket->u16SourcePanID-((psPacket->u16SourcePanID>>8)<<8)); vPutC(psPacket->u16SourcePanID>>8);
					// Source Short Address
					vPutC(psPacket->u16SourceShortAddress-((psPacket->u16SourceShortAddress>>8)<<8)); vPutC(psPacket->u16SourceShortAddress>>8);

				// Payload
					for(n = 0; n < psPacket->u8PayloadLength; n++)
					{
						if (HEX_Output) {
							vPrintf("%02x ", psPacket->u8Payload[n]);
						}
						else {
							vPutC( psPacket->u8Payload[n] );
						}
					}

				// FCS
				if (FCS) { vPutC(0xAA); vPutC(0xAA); }
			}
			else {
				// Will need to inform Wireshark that a packet was received but not transmetted.
			}

			break;





		/* MAC Data ----------------------------------------------------------------------------------------- */
		case 1:
			if ( (psPacket->u8PayloadLength!=0) && (psPacket->bPacketGood) ) {
				// Entete du packet
				// guint32 ts_sec;         /* timestamp seconds */
				// Tout à zero pour l instant. Devrait mettre la date plus tard.
				vPutC(0);
				vPutC(0);
				vPutC(0);
				vPutC(0);

				// guint32 ts_usec;        /* timestamp microseconds */
				// Tout à zero pour l instant. Devrait mettre la date plus tard.
				vPutC(0);
				vPutC(0);
				vPutC(0);
				vPutC(0);
				// vPutC(u32Timestamp-((u32Timestamp>>8)<<8));
				// vPutC(u32Timestamp>>8);


				// guint32 incl_len;       /* number of octets of packet saved in file */
				vPutC( psPacket->u8PayloadLength + 9 + FCS_Length );
				// vPutC(53);
				vPutC(0);
				vPutC(0);
				vPutC(0);

				// guint32 orig_len;       /* actual length of packet */
				vPutC( psPacket->u8PayloadLength + 9 + FCS_Length );
				// vPutC(53);
				vPutC(0);
				vPutC(0);
				vPutC(0);

				// vPrintf("DAT ");
				// vPrintf("FCtrl: %04x ",psPacket->u16FrameControl);
				if (HEX_Output) {
					vPrintf("%02x ",psPacket->u16FrameControl-((psPacket->u16FrameControl>>8)<<8));
					vPrintf("%02x ",psPacket->u16FrameControl>>8);
					vPrintf("%02x ", psPacket->u8SequenceNumber);
				}
				else {
					// FrameControl
					vPutC(psPacket->u16FrameControl-((psPacket->u16FrameControl>>8)<<8)); vPutC(psPacket->u16FrameControl>>8);
					// SequenceNumber
					vPutC(psPacket->u8SequenceNumber);
				}

				if(bDstShortAddr){
					// vPrintf("DPAN=%04x ",psPacket->u16DestinationPanID);
					// vPrintf("DSAD=%04x ",psPacket->u16DestinationShortAddress);
					if (HEX_Output) {
						vPrintf("%02x ",psPacket->u16DestinationPanID-((psPacket->u16DestinationPanID>>8)<<8));
						vPrintf("%02x ",psPacket->u16DestinationPanID>>8);


						vPrintf("%02x ",psPacket->u16DestinationShortAddress-((psPacket->u16DestinationShortAddress>>8)<<8));
						vPrintf("%02x ",psPacket->u16DestinationShortAddress>>8);
					}
					else {
						// Destination Pan ID
						vPutC(psPacket->u16DestinationPanID-((psPacket->u16DestinationPanID>>8)<<8)); vPutC(psPacket->u16DestinationPanID>>8);
						// Destination Short Address
						vPutC(psPacket->u16DestinationShortAddress-((psPacket->u16DestinationShortAddress>>8)<<8)); vPutC(psPacket->u16DestinationShortAddress>>8);
					}
				}

				if(bSrcShortAddr){
					// vPrintf("SSAD=%04x ",psPacket->u16SourceShortAddress);
					if (HEX_Output) {
					vPrintf("%02x ",psPacket->u16SourceShortAddress-((psPacket->u16SourceShortAddress>>8)<<8));
					vPrintf("%02x ",psPacket->u16SourceShortAddress>>8);
					}
					else {
						// Source Short Address
						vPutC(psPacket->u16SourceShortAddress-((psPacket->u16SourceShortAddress>>8)<<8)); vPutC(psPacket->u16SourceShortAddress>>8);
					}
				}

				/*
				if (0) {
					if(!bIntraPan && bSrcAddr){
						// vPrintf("SPAN=%04x ",psPacket->u16SourcePanID);
						if (HEX_Output) {
							vPrintf("%02x ",psPacket->u16SourcePanID-((psPacket->u16SourcePanID>>8)<<8));
							vPrintf("%02x ",psPacket->u16SourcePanID>>8);
						}
						else {
							vPutC(psPacket->u16SourcePanID-((psPacket->u16SourcePanID>>8)<<8));
							vPutC(psPacket->u16SourcePanID>>8);
						}
					}
				}
				*/

				//#ifdef JENNIC_CHIP_FAMILY_JN516x
				// for(n = 0; n < psPacket->u8PayloadLength; n++)
				for(n = 0; n < psPacket->u8PayloadLength; n++)
				{
					if (HEX_Output) {
						vPrintf("%02x ", psPacket->u8Payload[n]);
					}
					else {
						vPutC( psPacket->u8Payload[n] );
					}
				}
				//#endif


				// Probably need to put the FCS which is missing
				if (FCS) { vPutC(0xAA); vPutC(0xAA); }
			}
			else {
				// Will need to inform Wireshark that a packet was received but not transmetted.
			}

			break;

		/* Ack  ----------------------------------------------------------------------------------------- */
		case 2:
			// vPrintf("ACK ");
			FrameAck = TRUE;

			if ( (psPacket->u8PayloadLength==0) && (psPacket->bPacketGood) ) {
			// if ( 1 ) {
				// Entete du packet
					// guint32 ts_sec;         /* timestamp seconds */
					// Tout à zero pour l instant. Devrait mettre la date plus tard.
					vPutC(0); vPutC(0); vPutC(0); vPutC(0);

					// guint32 ts_usec;        /* timestamp microseconds */
					// Tout à zero pour l instant. Devrait mettre la date plus tard.
					vPutC(0); vPutC(0); vPutC(0); vPutC(0);
					// vPutC(u32Timestamp-((u32Timestamp>>8)<<8));
					// vPutC(u32Timestamp>>8);

					// guint32 incl_len;       /* number of octets of packet saved in file */
					vPutC( psPacket->u8PayloadLength + 3 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);

					// guint32 orig_len;       /* actual length of packet */
					vPutC( psPacket->u8PayloadLength + 3 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);

					// FrameControl
					vPutC(psPacket->u16FrameControl-((psPacket->u16FrameControl>>8)<<8)); vPutC(psPacket->u16FrameControl>>8);
					// SequenceNumber
					vPutC(psPacket->u8SequenceNumber);

				// FCS
					if (FCS) { vPutC(0xAA); vPutC(0xAA); }
			}
			else {
				// Will need to inform Wireshark that a packet was received but not transmetted.
			}

			break;

		/* MAC Command  ----------------------------------------------------------------------------------------- */
			/* Beacon Request */
			/* Association Request */
			/* Association Response */
		case 3:
			// vPrintf("CMD ");
			// Ubiqua: Le message Data Request fait 12 bytes = Mac Header 9 + Payload 1 + FCS 2
			if ( (psPacket->u8PayloadLength!=0) && (psPacket->bPacketGood) ) {
				// Entete du packet
					// guint32 ts_sec;         /* timestamp seconds */
					// Tout à zero pour l instant. Devrait mettre la date plus tard.
					vPutC(0); vPutC(0); vPutC(0); vPutC(0);

					// guint32 ts_usec;        /* timestamp microseconds */
					// Tout à zero pour l instant. Devrait mettre la date plus tard.
					vPutC(0); vPutC(0); vPutC(0); vPutC(0);
					// vPutC(u32Timestamp-((u32Timestamp>>8)<<8));
					// vPutC(u32Timestamp>>8);

					// guint32 incl_len;       /* number of octets of packet saved in file */
					vPutC( psPacket->u8PayloadLength + 2+1+2 + 2*bSrcShortAddr + 8*bSrcExtAddr + 2*(1-bIntraPan)*bSrcExtAddr + 2*bDstShortAddr + 8*bDstExtAddr + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);

					// guint32 orig_len;       /* actual length of packet */
					vPutC( psPacket->u8PayloadLength + 2+1+2 + 2*bSrcShortAddr + 8*bSrcExtAddr + 2*(1-bIntraPan)*bSrcExtAddr + 2*bDstShortAddr + 8*bDstExtAddr + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);

				// MAC
					// FrameControl
					vPutC(psPacket->u16FrameControl-((psPacket->u16FrameControl>>8)<<8)); vPutC(psPacket->u16FrameControl>>8);

					// SequenceNumber
					vPutC(psPacket->u8SequenceNumber);

					// Destination Pan ID
					vPutC(psPacket->u16DestinationPanID-((psPacket->u16DestinationPanID>>8)<<8)); vPutC(psPacket->u16DestinationPanID>>8);

					// Destination  Address
					if ( bDstShortAddr ) {
						vPutC(psPacket->u16DestinationShortAddress-((psPacket->u16DestinationShortAddress>>8)<<8)); vPutC(psPacket->u16DestinationShortAddress>>8);
					}
					if ( bDstExtAddr ) {
						uint8 *data = &(psPacket->u64DestinationExtendedAddress);
						vPutC(data[7]); vPutC(data[6]); vPutC(data[5]); vPutC(data[4]); vPutC(data[3]); vPutC(data[2]); vPutC(data[1]); vPutC(data[0]);
					}

					// Source Pan ID
					if ( (bIntraPan==0) && (bSrcExtAddr) ) {
						vPutC(psPacket->u16SourcePanID-((psPacket->u16SourcePanID>>8)<<8)); vPutC(psPacket->u16SourcePanID>>8);
					}

					// Source  Address
					if ( bSrcShortAddr ) {
						vPutC(psPacket->u16SourceShortAddress-((psPacket->u16SourceShortAddress>>8)<<8)); vPutC(psPacket->u16SourceShortAddress>>8); }
					if ( bSrcExtAddr ) {
						uint8 *data = &(psPacket->u64SourceExtendedAddress);
						vPutC(data[7]); vPutC(data[6]); vPutC(data[5]); vPutC(data[4]); vPutC(data[3]); vPutC(data[2]); vPutC(data[1]); vPutC(data[0]);
					}

				// Payload
					for(n = 0; n < psPacket->u8PayloadLength; n++)
					{
						if (HEX_Output) {
							vPrintf("%02x ", psPacket->u8Payload[n]);
						}
						else {
							vPutC( psPacket->u8Payload[n] );
						}
					}

				// FCS
					if (FCS) { vPutC(0xAA); vPutC(0xAA); }
			}
			else {
				// Will need to inform Wireshark that a packet was received but not transmetted.
			}

			break;

		/* Reserved  -----------------------------------------------------------------------------------------  */
		default:
			// vPrintf("RES ");
			break;
		}




}


/****************************************************************************
 *
 * NAME:       vDoTransmitPacketsTest
 *
 * DESCRIPTION:
 * Transmits packets at regular intervals on a specified channel.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoTransmitPacketsTest(void)
{

    char acKey = 0;
    uint8 u8Channel = 0, u8NewChannel;
    uint32 u32PacketsSent = 0;
    bool_t bDoTest = TRUE;
    bool_t bFastRate = FALSE;
    tsJPT_PT_Packet sPacket;
    uint32 u32TickTimerTimout = 16000000;

    /* set up the tick timer to count in 1 second periods */
    vAHI_TickTimerInit((void*)NULL);
    vAHI_TickTimerIntEnable(FALSE);
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_CONT);
    vAHI_TickTimerInterval(0xffffffff);

    /* enable protocol */
    bJPT_RadioInit(u32RadioMode);

    /* set default power level */
    vJPT_RadioSetPower(u8MaxTxPower);

    while(bDoTest == TRUE){

        /* disable protocol */

        u8NewChannel = u8ChangeChannel(acKey);

        if((u8NewChannel != u8Channel)){

            u8Channel = u8NewChannel;
            u32PacketsSent = 0;
#ifdef TXPOWERADJUST
            u8PowerAdjustChannel = u8Channel;
        	vJPT_TxPowerAdjust(u8TxPowerAdj, u8Attenuator3dB, u8PowerAdjustChannel);
#endif

        }

        if(acKey == 'f' || acKey == 'F'){               /* fast transmit rate */
            u32TickTimerTimout = 320000;                /* fast as possible while still allowing transmitter to keep up with shortest loop time */
            bFastRate = TRUE;
        }
        else if (acKey == 's' || acKey == 'S'){         /* slow transmit rate (approx. 1 per sec) - default */
            u32TickTimerTimout = 16000000;
            bFastRate = FALSE;
        }

if (bFastRate)
{
            vPrintf("\n***************************************"
                    "\n*  Transmit Packet Test In Progress   *"
                    "\n*           Fast Rate                 *");
}
else
{
            vPrintf("\n***************************************"
                    "\n*  Transmit Packet Test In Progress   *"
                    "\n*           Slow Rate                 *");
}
        vPrintf("\n***************************************"
                "\n* Key        Function                 *"
                "\n*                                     *"
                "\n*  +      Increment Channel           *"
                "\n*  -      Decrement Channel           *"
                "\n*  f      Fast transmit rate          *"
                "\n*  s      Slow transmit rate          *"
                "\n*  x      Return to main menu         *"
                "\n*                                     *"
                "\n***************************************\n"
                "\nChannel       %d    (%s)"
                "\nMAC Address   %016lx"
                "\n\n",u8Channel, acFrequencies[u8Channel - 11],u64JPT_MacAddress);

        while(!bUartRxDataAvailable(UART_TO_PC))
        {

            /* Reset timer */
#if (defined JENNIC_CHIP_FAMILY_JN514x) || (defined JENNIC_CHIP_FAMILY_JN516x)
            vAHI_TickTimerWrite(0);
#else
            vAHI_TickTimerInterval(u32TickTimerTimout);
            vAHI_TickTimerClear();
#endif

            sPacket.u64SourceExtendedAddress = u64JPT_MacAddress;
            sPacket.u16SourcePanID = 0x1234;
            sPacket.u16DestinationPanID = 0x1234;
            sPacket.u64DestinationExtendedAddress = 0LL;
            sPacket.u8PayloadLength = 4;

            memcpy(sPacket.u8Payload, &u32PacketsSent, sizeof(uint32));

            sPacket.u16FrameControl = (3 << 14) | (3 << 10) | (1 << 0);

            vJPT_PacketTx(u8Channel, &sPacket);

            u32PacketsSent++;

            vPrintf("\rPackets Sent %d",u32PacketsSent);

            while(u32AHI_TickTimerRead() < u32TickTimerTimout){
                if(bUartRxDataAvailable(UART_TO_PC))
                {
                    break;
                }
            }

        }


        acKey = acGetC();

        if(acKey == 'x' || acKey == 'X'){
            bDoTest = FALSE;
        }

    }

    /* disable protocol */
    vJPT_RadioDeInit();

}


/****************************************************************************
 *
 * NAME:       vDoPowerMeter
 *
 * DESCRIPTION:
 * Uses the wireless microcontroller as a very simple power meter. This
 * scans across channels 11 to 26, and takes an energy detect measurement
 * on each channel. The highest recorded power level detected across the
 * channels is displayed as a value in dBm (dB's related to a power level
 * of 1mW).
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoPowerMeter(void)
{
    char acKey = 0;
    uint8 u8Channel,u8BestChannel;
    uint8 u8Energy;

    int16 i16RSSI, i16PeakRSSI;
    bool_t bDoTest = TRUE;

    /* enable protocol */
    bJPT_RadioInit(u32RadioMode);

    while(bDoTest == TRUE){

        i16RSSI = -98;
        i16PeakRSSI = -98;

        vPrintf("\n***************************************"
                "\n*           Power Meter               *"
                "\n***************************************"
                "\n* Key        Function                 *"
                "\n*                                     *"
                "\n*  x      Return to main menu         *"
                "\n*                                     *"
                "\n***************************************\n\n");

        while(!bUartRxDataAvailable(UART_TO_PC)){

            /* use a wake timer to wait for 100 milliseconds */
            vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, FALSE);
            vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, 3200);

            i16PeakRSSI = -98;
            u8BestChannel = 0;

            for(u8Channel = 11; u8Channel <= 26; u8Channel++){

                u8Energy = u8JPT_EnergyDetect(u8Channel, 100);

                i16RSSI = i16JPT_ConvertEnergyTodBm(u8Energy);

                // Correct for HP Modules LNA on RX (+12dB LNA)
                if ((u32ModuleRadioMode==E_JPT_MODE_MO6_HIPOWER) || (u32ModuleRadioMode==E_JPT_MODE_ETSI) || (u32ModuleRadioMode==E_JPT_MODE_ETSIM06)){
                    i16RSSI -=12;
                }
                // Correct for HP Modules LNA on RX (+12dB LNA) and 7dB att on TX path (also on RX path)
                if (u32ModuleRadioMode==E_JPT_MODE_MO5_HIPOWER)
                {
                    i16RSSI -=5;
                }

                // Update peak recorded value
                if(i16RSSI > i16PeakRSSI){
                    i16PeakRSSI = i16RSSI;
                    u8BestChannel = u8Channel;
                }
            }

            vPrintf("\rPower = %idBm   Channel = %d", i16PeakRSSI,u8BestChannel);

            while(!(u8AHI_WakeTimerFiredStatus() & 1 << E_AHI_WAKE_TIMER_0));

        }


        acKey = acGetC();

        if(acKey == 'x' || acKey == 'X'){
            bDoTest = FALSE;
        }

    }

    /* disable protocol */
    vJPT_RadioDeInit();

}

#if 0
/****************************************************************************
 *
 * NAME:       vDoPerTest
 *
 * DESCRIPTION:
 * A sub menu allowing the user to choose between operating as a PAR test
 * master or a PER test slave.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoPerTest(void)
{

    uint8  u8Chan;
    bool_t bExit = FALSE;

    //vUartInit(UART_TO_OTHER, BAUD_RATE, &tsFifo_Uart1); /* per test comms on uart 1 */
    vUartInit(UART_TO_PC, BAUD_RATE, au8UartTxBuffer, sizeof(au8UartTxBuffer), au8UartRxBuffer, sizeof(au8UartRxBuffer));/* uart for user interface */

    while(1){

        vPrintf("\n***************************************"
                "\n*       Packet Error Rate Test        *"
                "\n***************************************\n"
                "\nNote, this test requires the following"
                "\nconnections to be made :-\n"
                "\n  Master                        Slave"
                "\n  DIO 19  <------------------>  DIO 20"
                "\n  DIO 20  <------------------>  DIO 19"
                "\n  GND     <------------------>  GND\n"
                "\na) Run as PER Test Master (Controller)"
                "\nb) Run as PER Test Slave  (D.U.T)"
                "\nx) Return to main menu"
                "\n\nPlease choose an option >");

        switch(acGetC()){

        /* act as a PER test master */
        case 'a':
        case 'A':
            /* enable protocol */
            bJPT_RadioInit(u32RadioMode);

            /* set default power level */
            vJPT_RadioSetPower(u8MaxTxPower);

            vPrintf("\fChannel\t\tHeaders Seen\tPackets Seen\tAcks Received\tThroughput(%%)");

            for (u8Chan=11; u8Chan<=26; u8Chan++)
            {
                vPrintf("\n  %d\t\t   ",u8Chan);
                vPerTestMaster(u8Chan);
            }

            /* disable protocol */
            vJPT_RadioDeInit();

            vPrintf("\n\nPress any key to continue..");
            acGetC();
            break;

        /* act as a per test slave */
        case 'b':
        case 'B':
            /* enable protocol */
            bJPT_RadioInit(u32RadioMode);

            /* set default power level */
            vJPT_RadioSetPower(u8MaxTxPower);

            vPrintf("\fRunning as PER test slave (DUT), press x to exit");

            do {
                for (u8Chan=11; u8Chan<=26; u8Chan++) {
                    if(!bPerTestSlave(u8Chan)){
                        bExit = TRUE;
                        break;
                    }
                }

            } while (!bExit);

            /* disable protocol */
            vJPT_RadioDeInit();
            break;


        case 'x':
        case 'X':
            vUartDeInit(UART_TO_OTHER);                         /* turn uart off */
            return;
            break;

        }

    }

}


/****************************************************************************
 *
 * NAME:       vDoPerTestMaster
 *
 * DESCRIPTION:
 * Transmits packets on the specified channel and looks for acknowledges from
 * a device operating as the PER test slave. A count is kept of the number
 * of packets sent and the number of ack's received. After performing a number
 * of repetitions, a command is sent via uart to the PER test slave to request
 * its results (number of headers received and number of frames received), and
 * then the results are displayed.
 *
 * PARAMETERS:      Name        RW  Usage
 *                  u8Channel   R   Channel to transmit on
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vPerTestMaster(uint8 u8Channel)
{
    int i;
    uint32 u32Acks;
    uint16 u16Reps;
    uint32 u32HeadersSeen;
    uint32 u32FramesSeen;
    uint32 u32Timeout;
    uint8  u8Throughput;

    /* select channel */
    bJPT_RadioSetChannel(u8Channel);

    vJPT_MTPT_MasterStart(100);

    u32Acks = 0;
    u16Reps = 500;

    for(i = 0; i < u16Reps; i++){

        vJPT_MTPT_MasterTxPacket(&u32Acks);

    }

    /* request results from MUT */
    vUartWrite(UART_TO_OTHER, 'r');

    /* wait for results to arrive over UART */
    u32Timeout = 0;
    while(!bUartRxDataAvailable(UART_TO_OTHER)){
        u32Timeout++;

        if(u32Timeout > TIMEOUT_RESULTS * u32ClkMultiply){
            vPrintf("No data returned from PER Test Slave, Check connections !");
            vUartFlush(UART_TO_OTHER);
            return;
        }
    }

    vJPT_MTPT_MasterStop();

    /* receive results over UART1 */
    bUartReadBinary(UART_TO_OTHER, (uint8*)&u32HeadersSeen, sizeof(u32HeadersSeen), TIMEOUT_RESULTS);
    bUartReadBinary(UART_TO_OTHER, (uint8*)&u32FramesSeen, sizeof(u32FramesSeen), TIMEOUT_RESULTS);

    /* calculate PER */
    u8Throughput = u8DeriveThroughput(u32HeadersSeen, u32Acks);      /* calculate the percentage throughput */

    /* need to format and display the results here */
    vPrintf("%d\t\t   %d\t\t   %d\t\t     %d",u32HeadersSeen, u32FramesSeen, u32Acks, u8Throughput);

}


/****************************************************************************
 *
 * NAME:       bPerTestSlave
 *
 * DESCRIPTION:
 * Chacks for received packets (both headers and complete frames), and keeps
 * a running total of the number of each received, then waits for a command
 * via UART from a device running as the PER test master and transmits the
 * results back via UART.
 *
 * PARAMETERS:      Name        RW  Usage
 *                  u8Chan      R   Channel to listen for packets on
 *
 * RETURNS:
 * bool_t, FALSE if the function exited due to a keypress
 *         TRUE if the function returned normally
 *
 ****************************************************************************/
PRIVATE bool_t bPerTestSlave(uint8 u8Channel)
{
    uint32 u32HeadersSeen = 0;
    uint32 u32FramesSeen = 0;
    uint32 u32ErrorsSeen = 0;

    uint8  u8CommandChar = '0';
    uint8 u8Cmd;

    bJPT_RadioSetChannel(u8Channel);

    vJPT_MTPT_SlaveStart();

    /* Wait for frames */

    do
    {

        vJPT_MTPT_SlavePoll(&u32HeadersSeen, &u32FramesSeen, &u32ErrorsSeen);

        /* see if any key presses */
        if(bUartRxDataAvailable(UART_TO_PC)){
            u8Cmd = u8UartRead(UART_TO_PC);
            if(u8Cmd == 'x' || u8Cmd == 'X'){
                return(0);
            }
        }

        /* see if a command byte has arrived over the UART */
        if(bUartRxDataAvailable(UART_TO_OTHER)){
            u8CommandChar = u8UartRead(UART_TO_OTHER);
        }
    } while (u8CommandChar != 'r');

    vJPT_MTPT_SlaveStop();

    vUartWriteBinary(UART_TO_OTHER, (uint8*)&u32HeadersSeen, sizeof(u32HeadersSeen));
    vUartWriteBinary(UART_TO_OTHER, (uint8*)&u32FramesSeen, sizeof(u32FramesSeen));

    return(1);

}
#endif

/****************************************************************************
 *
 * NAME:       vDoFrequencyTest
 *
 * DESCRIPTION:
 * Places the wireless microcontroller in a mode where the 16MHz clock signal
 * is fed out to an IO pin as a square wave so that it can be measured
 * accurately.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoFrequencyTest(void)
{

    vPrintf("\n***************************************"
            "\n*          Frequency Test             *"
            "\n***************************************\n"
            "\nClock signal is now available on DIO 10"
            "\nfor the next 30 seconds");

    /* give time for last character to be transmitted */
    while(bUartTxInProgress(UART_TO_PC));

    /* enable protocol */
    bJPT_RadioInit(u32RadioMode);

    vJPT_XtalOscillatorTest(E_JPT_XOT_DIO10);

    /* use a wake timer to wait for 30 seconds */
    vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, FALSE);
    vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, 32000 * 30);

    while(!(u8AHI_WakeTimerFiredStatus() & 1 << E_AHI_WAKE_TIMER_0));

    vJPT_XtalOscillatorTest(E_JPT_XOT_STOP);

    /* disable protocol */
    vJPT_RadioDeInit();
}


/****************************************************************************
 *
 * NAME:       vDoCurrentTest
 *
 * DESCRIPTION:
 * Allows the user to put the device into a number of modes which affect the
 * current consumption of the wireless microcontroller.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoCurrentTest(void)
{

#define CT_SLEEP_TIME   32000 * 60

    char acCommand = 0;

    /* Set up DIO so that if its a high power module, external circuitry is off unless required */

    /* Turn pullups on */
    vAHI_DioSetPullup(0x1fffff, 0);

    /* Set DIO 2 and 3 as outputs and low */
    vAHI_DioSetOutput(0, 0xc);
    vAHI_DioSetDirection(0, 0xc);
    vAHI_DioSetPullup(0, 0x0000000c);

    while(1){

        vPrintf("\n***************************************"
                "\n*    Current Measurement Options      *"
                "\n***************************************\n"
                "\na) Deep sleep mode"
                "\nb) Sleep mode without memory retention"
                "\nc) Sleep mode with memory retention"
                "\nd) Doze mode"
                "\ne) microcontroller running"
                "\nf) microcontroller + protocol"
                "\ng) microcontroller + protocol + TX"
                "\nh) microcontroller + protocol + RX"
                "\nx) Return to main menu\n"
                "\n Please choose an option >");

        acCommand = acGetC();

        switch(acCommand){

        case 'a':       /* put device into deep sleep mode */
        case 'A':
            vPrintf("\nPutting device into deep sleep mode."
                    "\nNote, you must reset the device to"
                    "\nexit this state");
            while(bUartTxInProgress(UART_TO_PC));                   /* wait for uart to finish transmission */
            vUartDeInit(UART_TO_PC);                                /* switch off uart */
#ifdef JN5121
            vAHI_PowerDown(TRUE);                                   /* enter deep sleep mode */
#else
// for non-JN5121 devices use vAHI_Sleep
            vAHI_Sleep(E_AHI_SLEEP_DEEP);
#endif
            break;

        case 'b':       /* put device in sleep mode without memory retention */
        case 'B':
            vPrintf("\nPutting device into sleep mode without memory retention for 60 seconds..");
            while(bUartTxInProgress(UART_TO_PC));                   /* wait for uart to finish transmission */

            vUartDeInit(UART_TO_PC);                                /* switch off uart */

            vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, TRUE);         /* enable a wake timer with interrupt */
            vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, CT_SLEEP_TIME); /* start timer */
#ifdef JN5121
            vAHI_MemoryHold(FALSE);                                 /* don't retain memory while sleeping */
            vAHI_PowerDown(FALSE);                                  /* enter normal sleep mode */
#else
/* for non-JN5121 devices use vAHI_Sleep and put flash into deep power down mode
   Note: for non-ST flash devices an alternate method (than vAHI_FlashPowerDown)
   of putting the flash into a low current mode may be required
*/
//#ifndef JENNIC_CHIP_FAMILY_JN516x
#if (!defined JENNIC_CHIP_FAMILY_JN516x)
            vAHI_FlashPowerDown();
#endif
            vAHI_Sleep(E_AHI_SLEEP_OSCON_RAMOFF);
#endif
            break;

        case 'c':       /* put device in sleep mode with memory retention */
        case 'C':
            vPrintf("\nPutting device into sleep mode with memory retention for 60 seconds..");
            while(bUartTxInProgress(UART_TO_PC));                   /* wait for uart to finish transmission */

            vUartDeInit(UART_TO_PC);                                /* switch off uart */

            //vPrintf("\nAbout to enable Wake Timer");
            //while(bUartTxInProgress(UART_TO_PC));
            vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, TRUE);         /* enable a wake timer with interrupt */
            //vPrintf("\nWake Timer Enabled");
            //while(bUartTxInProgress(UART_TO_PC));

            vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, CT_SLEEP_TIME); /* start timer */
            //vPrintf("\nWake Timer Started");
            //while(bUartTxInProgress(UART_TO_PC));
#ifdef JN5121
            vAHI_MemoryHold(TRUE);                                  /* retain memory while sleeping */
            vAHI_PowerDown(FALSE);                                  /* enter normal sleep mode */
#else
/* for non-JN5121 devices use vAHI_Sleep and put flash into deep power down mode
   Note 1: for non-ST flash devices an alternate method (than vAHI_FlashPowerDown)
   of putting the flash into a low current mode may be required
   Note 2: following sleep with memory held the application may need to wakeup the flash
*/
//#ifndef JENNIC_CHIP_FAMILY_JN516x
#if (!defined JENNIC_CHIP_FAMILY_JN516x)
           // vPrintf("\nAttempting to powerdown flash");
           // while(bUartTxInProgress(UART_TO_PC));
            vAHI_FlashPowerDown();                                  /* retain memory while sleeping */
#endif
           // vPrintf("\nAbout to go to sleep");
           // while(bUartTxInProgress(UART_TO_PC));
            vAHI_Sleep(E_AHI_SLEEP_OSCON_RAMON);
#endif
            break;

        case 'd':       /* put device in doze mode */
        case 'D':
            vPrintf("\nPutting device into doze mode for 60 seconds..\n");
            while(bUartTxInProgress(UART_TO_PC));                   /* wait for uart to finish transmission */
            vUartDeInit(UART_TO_PC);                                /* switch off uart */

            vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, TRUE);         /* enable a wake timer with interrupt */
            vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, CT_SLEEP_TIME); /* start timer */

            vAHI_CpuDoze();                                         /* put cpu in doze mode untill interrupt*/

            vUartInit(UART_TO_PC, BAUD_RATE, au8UartTxBuffer, sizeof(au8UartTxBuffer), au8UartRxBuffer, sizeof(au8UartRxBuffer));/* uart for user interface */

            break;

        case 'e':       /* Micro on, nothing else */
        case 'E':
            vPrintf("\nMicro running, press any key to exit..");
            acGetC();                                               /* wait for a key press */
            break;

        case 'f':       /* Micro + protocol */
        case 'F':
            vPrintf("\nMicro running with protocol, press any key to exit..");
            bJPT_RadioInit(u32RadioMode);                               /* turn protocol on */
            acGetC();                                               /* wait for a key press */
            vJPT_RadioDeInit();                                     /* turn protocol off */
            break;

        case 'g':       /* Micro + Protocol + TX */
        case 'G':
            vPrintf("\nMicro running with protocol + TX, press any key to exit..");
            bJPT_RadioInit(u32RadioMode);                               /* turn protocol on */

            vJPT_RadioSetPower(u8MaxTxPower);                       /* set default power level */

#ifdef TXPOWERADJUST
        	vJPT_TxPowerAdjust(u8TxPowerAdj, u8Attenuator3dB, u8PowerAdjustChannel);
#endif
            vJPT_TxPowerTest(E_JPT_TXPT_RUN_PRBS);                  /* put radio in TX mode */
            acGetC();                                               /* wait for a key press */

            vJPT_TxPowerTest(E_JPT_TXPT_STOP);                      /* end TX mode */
            vJPT_RadioDeInit();                                     /* turn protocol off */
            break;

        case 'h':       /* Micro + Protocol + RX */
        case 'H':
            vPrintf("\nMicro running with protocol + RX, press any key to exit..");
            bJPT_RadioInit(u32RadioMode);                               /* turn protocol on */

            do {
                u8JPT_EnergyDetect(11, 50000);                      /* radio in receive mode */
            } while(!bUartRxDataAvailable(UART_TO_PC));

            acGetC();                                               /* wait for a key press */

            vJPT_RadioDeInit();                                     /* turn protocol off */
            break;

        case 'x':
        case 'X':
            return;
            break;

        }

    }

}

/****************************************************************************
 *
 * NAME:       vDoConnectionLessPerTest
 *
 * DESCRIPTION:
 * A sub menu allowing the user to choose between operating as a PER test
 * master or a PER test slave.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoConnectionLessPerTest(void)
{

    while(1){

        vPrintf("\n*****************************************"
                "\n* Connectionless Packet Error Rate Test *"
                "\n*****************************************\n"
                "\na) Run as PER Test Master"
                "\nb) Run as PER Test Slave"
                "\nx) Return to main menu"
                "\n\nPlease choose an option >");

        switch(acGetC()){

        /* act as a PER test master */
        case 'a':
        case 'A':
            vDoConnectionlessPerTestMaster();
            break;

        /* act as a per test slave */
        case 'b':
        case 'B':
            vDoConnectionlessPerTestSlave();
            break;


        case 'x':
        case 'X':
            return;

        }

    }

}


/****************************************************************************
 *
 * NAME:       vDoConnectionlessPerTestMaster
 *
 * DESCRIPTION:
 * Runs the connectionless PER test master.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoConnectionlessPerTestMaster(void)
{

    bool_t bEndpointOk = TRUE;
    uint8 u8Cmd;
    uint8 u8RunMode;
    uint32 u32Count = 0;
    uint32 u32Seen = 0;
    uint16 u16Per;
    uint16 u16Bad;

    char acMode[5][24] = {"Locating endpoint...   ",
                          "Stopped                ",
                          "Running (With Ack's)   ",
                          "Running (Without Ack's)",
                          "Restarting...          "};

    tsJPT_SSPT_MasterState sMasterData; /* holds master state data */

    vPrintf("\fLocating endpoint...");

    /* Initialise the radio */
    bJPT_RadioInit(u32RadioMode);

    /* Initialise site survey PER test master */
    vJPT_SSPT_MasterInit();

    /* Locate the endpoint */
    sMasterData.u8Mode = E_JPT_SSPT_MODE_LOCATE;
    u8RunMode = sMasterData.u8Mode;
    sMasterData.u8Channel = 14;
    sMasterData.u8Retries = 3;
    bEndpointOk = bJPT_SSPT_MasterSetState(&sMasterData);

    if(!bEndpointOk){
        vPrintf("Failed!");
    }


    /* Start the PER test running in Ack's mode */
    sMasterData.u8Mode = E_JPT_SSPT_MODE_RUNNING_ACKS;
    u8RunMode = sMasterData.u8Mode;
    bEndpointOk = bJPT_SSPT_MasterSetState(&sMasterData);


    while(1){

        vPrintf("\n**********************************************************"
                "\n* Connectionless Packet Error Rate Test - Master         *"
                "\n**********************************************************"
                "\n* Key        Function                                    *"
                "\n*                                                        *"
                "\n*  +      Increment Channel                              *"
                "\n*  -      Decrement Channel                              *"
                "\n*  r      Change Retrys                                  *"
                "\n*  m      Change Mode                                    *"
                "\n*  s      Stop/Start                                     *"
                "\n*  x      Return to main menu                            *"
                "\n*                                                        *"
                "\n**********************************************************"
                "\n*         Channel %d %s             *"
                "\n*   Seen   |   Total  |  PER   | CCA Fail | Retrys | LQI *"
                "\n*",sMasterData.u8Channel, acMode[sMasterData.u8Mode - 1]);

        while(!bUartRxDataAvailable(UART_TO_PC)){

            /* get updated per test data */
            vJPT_SSPT_MasterGetState(&sMasterData);


            if (u32Count++ > 10000) {
                u32Count = 0;

                /* if no more packets received, set LQI to 0 since it contains LQI of last packet received */
                if(u32Seen == sMasterData.u32Seen){
                    sMasterData.u8Lqi = 0;
                } else {
                    u32Seen = sMasterData.u32Seen;
                }

                /* Calculate new PER and CCA Fail values */
                if(sMasterData.u32Total == 0){
                    u16Per = 0;
                    u16Bad = 0;
                } else {
                    u16Per = (uint16)(1000 - ((sMasterData.u32Seen * 1000) / sMasterData.u32Total));
                    u16Bad = (uint16)((sMasterData.u32Errors * 1000) / sMasterData.u32Total);
                }

                switch(u8RunMode){

                case E_JPT_SSPT_MODE_RUNNING_ACKS:
                    /* write the dynamic parts of the display */
                    vPrintf("\r* %8d | %8d | %3d.%1d%% |  %3d.%1d%%  |    %d   |  %2d *",sMasterData.u32Seen,
                                                                   sMasterData.u32Total,
                                                                   u16Per / 10, u16Per % 10,
                                                                   u16Bad / 10, u16Bad % 10,
                                                                   sMasterData.u8Retries,
                                                                   sMasterData.u8Lqi);
                    break;

                case E_JPT_SSPT_MODE_RUNNING_NO_ACKS:
                    /* write the dynamic parts of the display */
                    vPrintf("\r* %8d | %8d | %3d.%d%% |    N/A   |   N/A  |  %2d *",sMasterData.u32Seen,
                                                                   sMasterData.u32Total,
                                                                   u16Per / 10, u16Per % 10,
                                                                   sMasterData.u8Lqi);
                    break;

                }

            }

        }


        u8Cmd = u8UartRead(UART_TO_PC);
        switch(u8Cmd){

        case '+':
        case '=':
            if(sMasterData.u8Channel < 26){
                sMasterData.u8Channel++;
            }
            break;

        case '-':
        case '_':
            if(sMasterData.u8Channel > 11){
                sMasterData.u8Channel--;
            }
            break;

        case 'r':
        case 'R':
            if(++sMasterData.u8Retries > 7){
                sMasterData.u8Retries = 0;
            }
            break;

        case 'm':
        case 'M':
            if(u8RunMode == E_JPT_SSPT_MODE_RUNNING_NO_ACKS){
                u8RunMode = E_JPT_SSPT_MODE_RUNNING_ACKS;
            } else {
                u8RunMode = E_JPT_SSPT_MODE_RUNNING_NO_ACKS;
            }
            if(sMasterData.u8Mode != E_JPT_SSPT_MODE_STOPPED){
                sMasterData.u8Mode = u8RunMode;
            }
            break;

        case 's':
        case 'S':
            if(sMasterData.u8Mode != E_JPT_SSPT_MODE_STOPPED){
                sMasterData.u8Mode = E_JPT_SSPT_MODE_STOPPED;
            } else {
                sMasterData.u8Mode = u8RunMode;
            }
            break;

        case 'x':
        case 'X':
            break;

        }

        bEndpointOk = bJPT_SSPT_MasterSetState(&sMasterData);

        if(u8Cmd == 'x' || u8Cmd == 'X'){
            vJPT_RadioDeInit();                 /* turn protocol off */
            vAHI_SwReset();                     /* reset the device */
            return;
        }


    }

}


/****************************************************************************
 *
 * NAME:       vDoConnectionlessPerTestSlave
 *
 * DESCRIPTION:
 * Runs the connectionless PER test slave.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoConnectionlessPerTestSlave(void)
{

    volatile uint32 i;
    uint8 u8Cmd;
    tsJPT_SSPT_SlaveState sSlaveData;   /* will hold slave state data */

    /* Initialise the radio */
    bJPT_RadioInit(u32RadioMode);

    /* Initialise the site survey PER test slave */
    vJPT_SSPT_SlaveInit();

    vPrintf("\n*************************************************"
            "\n* Connectionless Packet Error Rate Test - Slave *"
            "\n*              Press x to quit                  *"
            "\n*************************************************"
            "\n");

    while(1){

        /* wait here a while */
        for(i = 0; i < 1000000 * u32ClkMultiply; i++);

        /* get current PER test state */
        vJPT_SSPT_SlaveGetState(&sSlaveData);

        /* Display status of the test on the UART console and LED's */
        vPrintf("\r* Channel = %d ",sSlaveData.u8Channel);

        switch(sSlaveData.u8Mode){

        case E_JPT_SSPT_MODE_STOPPED:
            vPrintf("Stopped                          *");
            break;

        case E_JPT_SSPT_MODE_RUNNING_ACKS:
            vPrintf("Running with ack's               *");
            break;

        case E_JPT_SSPT_MODE_RUNNING_NO_ACKS:
            vPrintf("Running without ack's            *");
            break;

        }

        /* see if any key presses */
        if(bUartRxDataAvailable(UART_TO_PC)){
            u8Cmd = u8UartRead(UART_TO_PC);
            if(u8Cmd == 'x' || u8Cmd == 'X'){
                vJPT_RadioDeInit();                 /* turn protocol off */
                vAHI_SwReset();                     /* reset the device */
                return;                             /* shouldn't get here */
            }
        }
    }


}


/****************************************************************************
 *
 * NAME:       vDoCCATest
 *
 * DESCRIPTION:
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDoCCATest(void)
{
    char acKey = 0;
    uint8 u8Channel;
    uint8 u8Threshold = 0x30;
    uint8 u8Mode = E_JPT_CCA_MODE_ENERGY;
    bool_t bDoTest = TRUE;

    char acMode[][41] = {{"Energy Above Threshold"},
                        {"Carrier Sense"},
                        {"Energy Above Threshold Or Carrier Sense"},
                        {"Energy Above Threshold And Carrier Sense"}};

    /* enable protocol */
    bJPT_RadioInit(u32RadioMode);

    /* Get initial channel */
    u8Channel = u8ChangeChannel(0);

    while(bDoTest == TRUE){

        vPrintf("\n***************************************"
                "\n*   Clear Channel Assessment Test     *"
                "\n***************************************"
                "\n* Key        Function                 *"
                "\n*                                     *"
                "\n*  +      Increment Channel           *"
                "\n*  -      Decrement Channel           *"
                "\n*  m      Change Mode                 *"
                "\n*  <      Reduce Energy Threshold     *"
                "\n*  >      Increase Energy Threshold   *"
                "\n*  x      Return to main menu         *"
                "\n*                                     *"
                "\n***************************************\n"
                "\nChannel            %d    (%s)"
                "\nEnergy Threshold   %d"
                "\nMode               %s\n",u8Channel, acFrequencies[u8Channel - 11]
                                           ,u8Threshold
                                           ,acMode[u8Mode - 1]);


        while(!bUartRxDataAvailable(UART_TO_PC)){

            /* use a wake timer to wait for 100 milliseconds */
            vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, FALSE);
            vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, 3200);

            if(bJPT_CCA(u8Channel, u8Mode, u8Threshold)){
                vPrintf("\rCCA                Channel Busy ");
            } else {
                vPrintf("\rCCA                Channel Clear");
            }

            while(!(u8AHI_WakeTimerFiredStatus() & 1 << E_AHI_WAKE_TIMER_0));

        }

        acKey = acGetC();

        switch(acKey){

        case '+':
        case '=':
        case '-':
        case '_':
            u8Channel = u8ChangeChannel(acKey);
            break;

        case '>':
        case '.':

            if(u8Threshold < 0xff) u8Threshold++;
            break;

        case '<':
        case ',':
            if(u8Threshold > 0) u8Threshold--;
            break;

        case 'm':
        case 'M':
            u8Mode++;
            if(u8Mode > E_JPT_CCA_MODE_CARRIER_AND_ENERGY) u8Mode = E_JPT_CCA_MODE_ENERGY;
            break;

        case 'x':
        case 'X':
            bDoTest = FALSE;
            break;

        }


    }

    /* disable protocol */
    vJPT_RadioDeInit();

}


/****************************************************************************
 *
 * NAME:       u8ChangeChannel
 *
 * DESCRIPTION:
 * Increments, decrements, or resets a value to be used to set the operating
 * channel. The "+" or "=" key increments the value and the "-" or "_" key
 * decrements it. A keypress value of 0 resets the value to its default.
 *
 * PARAMETERS:      Name        RW  Usage
 *                  u8Key       R   Keypress value (ascii character)
 *
 * RETURNS:
 * uint8, Channel number, 11 to 26
 *
 ****************************************************************************/
PRIVATE uint8 u8ChangeChannel(uint8 u8Key)
{

    static uint8 u8Chan = 18;

    switch(u8Key){

    case '+':
    case '=':
        if (u8Chan < 26) u8Chan++;
        break;

    case '-':
    case '_':
        if (u8Chan > 11) u8Chan--;
        break;

    case 0:
        u8Chan = u8JPT_RadioGetChannel();
        break;

    }

    return(u8Chan);

}


/****************************************************************************
 *
 * NAME:       u8ChangePower
 *
 * DESCRIPTION:
 * Increments, decrements, or resets a value to be used to set the tx power
 * output. The "<" or "," key decrements the value and the ">" or "." key
 * increments it. A keypress value of 0 resets the value to its default.
 *
 * PARAMETERS:      Name        RW  Usage
 *                  u8Key       R   Keypress value (ascii character)
 *
 * RETURNS:
 * uint8, Power output value (0 - 5)
 *
 ****************************************************************************/
PRIVATE uint8 u8ChangePower(uint8 u8Key)
{

    static uint8 u8Power;

    switch(u8Key){

    case '>':
    case '.':
        if (u8Power < u8MaxTxPower) u8Power++;
        break;

    case '<':
    case ',':
        if (u8Power > 0) u8Power--;
        break;

    case 0:
        u8Power = u8JPT_RadioGetPower();
        break;

    }

    vJPT_RadioSetPower(u8Power);
    u8Power = u8JPT_RadioGetPower();

    return(u8Power);

}

/****************************************************************************
 *
 * NAME:       u8ChangePowerFine
 *
 * DESCRIPTION:
 * Increments, decrements, or resets a value to be used to set the tx power
 * output. The "<" or "," key decrements the value and the ">" or "." key
 * increments it. A keypress value of 0 resets the value to its default.
 *
 * PARAMETERS:      Name        RW  Usage
 *                  u8Key       R   Keypress value (ascii character)
 *
 * RETURNS:
 * uint8, Power output value (0 - 47)
 *
 ****************************************************************************/
PRIVATE uint8 u8ChangePowerFine(uint8 u8Key)
{

    static uint8 u8Power;

    switch(u8Key){

    case '>':
    case '.':
        if (u8Power < u8MaxTxPowerFine) u8Power++;
        break;

    case '<':
    case ',':
        if (u8Power > 0) u8Power--;
        break;

    case 0:
        u8Power = u8JPT_RadioGetPowerFine();
        break;

    }

    vJPT_RadioSetPowerFine(u8Power);
    u8Power = u8JPT_RadioGetPowerFine();

    return(u8Power);

}


/****************************************************************************
 *
 * NAME:       u32IncDec32
 *
 * DESCRIPTION:
 * Increments or decrements a 32 bit value depending on a keypress ascii
 * code The calling function can specify the keys and shifted keys that
 * perform the inc or dec.
 *
 * PARAMETERS:      Name        RW  Usage
 *                  u8Key       R   Keypress value (ascii character)
 *                  u32Value    R   Current value of 32 bit val to inc/dec
 *                  u32Modifier R   Step size of inc/dec
 *                  u32Min      R   Lower limit for decrements
 *                  u32Max      R   Upper limit for increments
 *                  u8IncKeyA   R   ascii code of increment key
 *                  u8IncKeyB   R   ascii code of increment key shifted
 *                  u8DecKeyA   R   ascii code of decrement key
 *                  u8DecKeyB   R   ascii code of decrement key shifted
 *
 * RETURNS:
 * uint8, Power output value (0 - 5)
 *
 ****************************************************************************/
PRIVATE uint32 u32IncDec32(uint8 u8Key, uint32 u32Value, uint32 u32Modifier,
                           uint32 u32Min, uint32 u32Max, uint8 u8IncKeyA,
                           uint8 u8IncKeyB, uint8 u8DecKeyA, uint8 u8DecKeyB)
{

    uint32 u32Val = u32Value;

    if(u8Key == u8IncKeyA || u8Key == u8IncKeyB){
        if (u32Val < u32Max - u32Modifier){
            u32Val += u32Modifier;
        } else {
            u32Val = u32Max;
        }
    }

    if(u8Key == u8DecKeyA || u8Key == u8DecKeyB){
        if (u32Val > u32Min + u32Modifier){
            u32Val -= u32Modifier;
        } else {
            u32Val = u32Min;
        }
    }

    return(u32Val);

}


/****************************************************************************
 *
 * NAME:       vPutC
 *
 * DESCRIPTION:
 * Writes a byte to the UART connected to the PC.
 *
 * PARAMETERS:      Name        RW  Usage
 *                  u8Data      R   Byte to write to the UART
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vPutC(uint8 u8Data)
{
    vUartWrite(UART_TO_PC, u8Data);

    // while ((u8AHI_UartReadLineStatus(0) & 0x20) == 0);
    // vAHI_UartWriteData(0, u8Data);

}


/****************************************************************************
 *
 * NAME:       acGetC
 *
 * DESCRIPTION:
 * Reads a character from the uart connected to the pc. If no character is
 * waiting in the rx buffer, it will wait until there is.
 *
 *
 * RETURNS:
 * char, Character read from the UART
 *
 ****************************************************************************/
PRIVATE char acGetC(void)
{

    return(u8UartRead(UART_TO_PC));
}


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

