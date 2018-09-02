/*************************************************************************
 *
*    Used with ICCARM and AARM.
 *
 *    
 **************************************************************************/
#include <intrinsics.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <nxp/iolpc2478.h>
#include <includes.h>

#include "board.h"
#include "sys.h"
#include "sdram_64M_32bit_drv.h"
#include "drv_glcd.h"
#include "drv_touch_scr.h"


#define NONPROT 0xFFFFFFFF
#define CRP1  	0x12345678
#define CRP2  	0x87654321
#define CRP3  	0x43218765

#ifndef SDRAM_DEBUG
#pragma segment=".crp"
#pragma location=".crp"
__root const unsigned crp = NONPROT;
#endif

#define TIMER0_TICK_PER_SEC   10000
#define TIMER1_TICK_PER_SEC   1

double delta_T = 0.0001; 
double freq = 50.0;

// ALpha for delta T = 0.0001
double alpha = 0.030459; // fc = 50 Hz
//double alpha = 0.006243; // fc = 10 Hz
//double alpha = 0.059117; // fc = 100 Hz

//Alpha for delta T = 0.0002
//double alpha = 0.05911; // fc = 50 Hz


int voltage = 0;
double y_in = 0.0; 
double y_in_old = 0.0; 
double y_in_filter = 0.0;//Filtered Voltage
double pulses = 0.0;
int notFirstTime = 0;
int notFirstTime1 = 0;
int notFirstTime2 = 0;
double time = 0.0;
double timeX = 0.0;
double X = 0.0;
double XOld = 0.0;
double frequency = 0.0;
double freqmax = 50.0;
double freqmin = 50.0;
int counter = 0;
int current = 0.0;
double T = 0.0;
double VSquared = 0.0;
double sumVSquared = 0.0;
double Vrms  = 0.0;
double i_in = 0.0;
double ISquared = 0.0;
double sumISquared = 0.0;
double Irms = 0.0;
double P = 0.0;
double sumP = 0.0;
double Power = 0.0;
int timerSocket = 0;
int socketON = 2; //0 = OFF; 1 = ON; 2 = AUTO; Initially the socket is on AUTO
int bulbON = 2; //0 = OFF; 1 = ON; 2 = AUTO; Initially the bulb is on AUTO
int socket = 0;
int bulb = 0;
int d = 0;
int f = 0;
int n = 0;
int s = 0;
int red = 0x0000FF;
int yellow = 0x000cd4ff;
int white = 0x00ffffff;
int blue = 0xFF0000;
int black = 0x00000000;
int green = 0x00FF00;
int darkRed = 0x0000BA;
int cursor_x = 0;
int cursor_y = 0;
int timer = 0;
int bulbTick = 0;
int socketTick = 0;
int tick = 0;
double bulbP = 0;
double socketP = 0;

extern Int32U SDRAM_BASE_ADDR;
extern FontType_t Terminal_6_8_6;
extern FontType_t Terminal_9_12_6;
extern FontType_t Terminal_18_24_12;

#define LCD_VRAM_BASE_ADDR ((Int32U)&SDRAM_BASE_ADDR)

unsigned char Smb380Id, Smb380Ver;


/*************************************************************************
 * Function Name: Timer0IntrHandler
 * Parameters: none
 *
 * Return: none
 *
 * Description: Timer 0 interrupt handler
 *
 *************************************************************************/
void Timer0IntrHandler (void)
{
  USB_D_LINK_LED_FIO ^= USB_D_LINK_LED_MASK; //heartbeat LED blink
  //clear interrupt
  T0IR_bit.MR0INT = 1;
  
  
  //voltage and current signals stored in variables
  if(ADDR2_bit.DONE == 1){
    voltage =  ADDR2_bit.RESULT; //0-1023
    y_in = (double) ((voltage)* 3.3/1023); //0-3.3
  }
  if(ADDR3_bit.DONE == 1){
    current =  ADDR3_bit.RESULT; //0-1023
    i_in = (double) current*1/1023; //0-1
  }

  //Low-pass filter for voltage signal
    if (notFirstTime1==0){
      y_in_filter = alpha * y_in + (1-alpha)* y_in;
      notFirstTime1 = 1;
    }
    y_in_filter = alpha * y_in + (1-alpha)* y_in_old;
      
    //Zero Crossing Detection and frequency calculation
    if((y_in_filter >= (3.3/2)) && (y_in_old <= (3.3/2))){
      if(notFirstTime == 1){
        time = pulses/10000;
        X = (y_in_filter/(y_in_filter-y_in_old))/10000;
        timeX = time - X + XOld;
        frequency = 1/(timeX);
        XOld = (y_in_filter/(y_in_filter-y_in_old))/10000;
        pulses = 0;
        counter++;
        
        
        // maximum and minimum frequency found
        if(counter>1){
          if(frequency<freqmin){
            freqmin = frequency;
          }
          if(frequency>freqmax){
            freqmax = frequency;
          }
        }
        
        
        //Bulb relay set on or off depending on button selection and frequency measurements
        if(bulbON == 1){
          FIO0SET_bit.P0_11 = 1; //Bulb ON
        }
        else if(bulbON == 0){
          FIO0CLR_bit.P0_11 = 1; //Bulb OFF
        }
        else if(bulbON == 2){
          if((frequency<49.5)){
            FIO0CLR_bit.P0_11 = 1; //Bulb OFF
            bulb = 0;    
          }
          if((frequency>49.9)){
            FIO0SET_bit.P0_11 = 1; //Bulb ON
            bulb = 1;
          } 
        }
        
        //Socket relay set on or off depending on button selection and frequency measurements
        if(socketON == 1){
          FIO0SET_bit.P0_19 = 1; //Socket ON
        }
        else if(socketON == 0){
          FIO0CLR_bit.P0_19 = 1; //Socket OFF
        }
        else if(socketON == 2){
          if((frequency<49.9) && (timerSocket>10000) && (socket == 1)){
            FIO0CLR_bit.P0_19 = 1; //Socket OFF
            socket = 0;
            timerSocket = 0;
          }
          if((frequency>50.1) && (timerSocket>10000) && (socket == 0)){
            FIO0SET_bit.P0_19 = 1; //Socket ON
            socket = 1;
            timerSocket = 0;
          } 
        }
 
      }
      else{
        notFirstTime = 1;
        pulses = 0;
        XOld = (y_in_filter/(y_in_filter-y_in_old))/10000;
      } 
    }
    
    //Power, Voltage and Current calcuations
    if((y_in_filter >= (3.3/2)) && (y_in_old <= (3.3/2))){
      if(notFirstTime2 == 1){
        Vrms = pow(sumVSquared/T,0.5);
        Irms = pow(sumISquared/T,0.5);
        Power = sumP/T;
        T = 0;
        sumVSquared = 0;
        sumISquared = 0;
        sumP = 0;
      }
      else{
        notFirstTime2 = 1;
        T = 0;
        sumVSquared = 0;
        sumISquared = 0;
        sumP = 0;
      }
    }
    
    //Update of variables, when a zero-crossing is not detected
    y_in_old = y_in_filter;
    pulses++;
    timerSocket++;
    T++;
    VSquared = pow((y_in_filter*(240*1.4142/3.3)),2);
    sumVSquared = sumVSquared + VSquared;
    ISquared = pow(i_in*1.4142,2);
    sumISquared = sumISquared + ISquared;
    P = (y_in_filter*(240*1.4142/3.3))*i_in*1.4142;
    sumP = sumP + P;

    
  VICADDRESS = 0;

}


/******************************************
 *Function Name: ADC init
******************************************/

void ADC_Init (void)
{
AD0CR_bit.PDN = 0;
PCONP_bit.PCAD = 1;
PCLKSEL0_bit.PCLK_ADC = 0x1; //CCLK = 72MHz-Chapter4 clocking & power control p.60
AD0CR_bit.CLKDIV = 5; //72MHz/(17+1)= 4MHz<=4.5 MHz
AD0CR_bit.BURST = 1; //0=ADC is set to operate in software controlled mode, 1= continue mode
PINSEL1_bit.P0_25 = 0x1; //AD0[2]
PINMODE1_bit.P0_25 = 0x2;
PINSEL1_bit.P0_26 = 0x1; //AD0[3]
PINMODE1_bit.P0_26 = 0x2;
//ADINTEN_bit.ADGINTEN = 0;
//ADINTEN_bit.ADINTEN0 = 1; //Enable interrupt
//ADINTEN_bit.ADINTEN1 = 1;
//ADINTEN_bit.ADINTEN2 = 1;
//ADINTEN_bit.ADINTEN3 = 1;
//AD0CR_bit.START = 0;
//VIC_SetVectoredIRQ (AD0IntrHandler,2,VIC_AD0);
//VICINTENABLE |= 1UL << VIC_AD0;
AD0CR_bit.SEL = 0x0F; //0x4;
AD0CR_bit.PDN = 1; //The A/D Converter is operational
}

/******************************************
 *Function Name: DAC init
******************************************/

void DAC_Init (void)
{
 /*
PCLKSEL0_bit.PCLK_DAC = 1;
PINSEL1_bit.P0_26 = 2;
PINSEL1_bit.P0_19 = 2;
PINMODE1_bit.P0_26 = 2;
DACR_bit.BIAS = 1;
}
*/
 
PINSEL1_bit.P0_26=2;  //sets pin function to AOUT
DACR_bit.BIAS=1;      //set BIAS mode 1
PCLKSEL0_bit.PCLK_DAC=1;  //enable clock signal --- Peripheral Clock Selection register 0.

//DACR_bit.VALUE = 0X3FF;
}

/******************************************
 *Function Name: Initial screen display
******************************************/

void initDisp (void)
{
  //System Status display
   GLCD_SetFont(&Terminal_18_24_12,white,blue);
   GLCD_SetWindow(10,10,310,33);
   GLCD_TextSetPos(0,0);
   GLCD_print("  System Status                  ");
   
   //Relay display
    GLCD_SetFont(&Terminal_18_24_12,white,blue);
    GLCD_SetWindow(10,85,310,110);
    GLCD_TextSetPos(0,0);
    GLCD_print("  Relay Status                           ");
    
    //Button display
    GLCD_SetFont(&Terminal_18_24_12,white,blue);
    GLCD_SetWindow(10,136,310,161);
    GLCD_TextSetPos(0,0);
    GLCD_print("  Manual Control                              ");
}

/******************************************
 *Function Name: Percentage calculation

Calculates for how much time 
proportionally the relays have been on  
******************************************/

void percentage (void)
{
  if((bulbON == 1)||((bulbON==2)&&(bulb==1))){
    bulbTick++;
  }
  if((socketON == 1)||((socketON==2)&&(socket==1))){
    socketTick++;
  }
  tick++;
  
  bulbP = (int) 100 * bulbTick/tick;
  socketP = (int) 100 * socketTick/tick;
}

/******************************************
 *Function Name: System Status display
******************************************/

void systemStatusDisp (void)
{
  // Frequency display
  char FreqString [10]; // destination string
  d = (int) frequency*1000; // Decimal precision: 3 digits
  f = (frequency * 1000)-d;
  d = (int) d/1000;
  if((f<100) && (f>9)){
    n=sprintf ( FreqString, "%d.0%d", d, f); 
  }
  if (f<10){
    n=sprintf ( FreqString, "%d.00%d", d, f); 
  }
  if (f>=100){
    n=sprintf ( FreqString, "%d.%d", d, f);
  }

  GLCD_SetFont(&Terminal_9_12_6,white,black);
  GLCD_SetWindow(20,38,160,52);
  GLCD_TextSetPos(0,0);
  GLCD_print("Frequency: ");
  GLCD_print("%s",FreqString,n);
  GLCD_print(" Hz   ");
  
  // Frequency Max display
  char FreqmaxString [10]; // destination string
  d = (int) freqmax*1000; // Decimal precision: 3 digits
  f = (freqmax * 1000)-d;
  d = (int) d/1000;
  if((f<100) && (f>9)){
    n=sprintf ( FreqmaxString, "%d.0%d", d, f); 
  }
  if (f<10){
    n=sprintf ( FreqmaxString, "%d.00%d", d, f); 
  }
  if (f>=100){
    n=sprintf ( FreqmaxString, "%d.%d", d, f);
  }

  GLCD_SetFont(&Terminal_9_12_6,red,black);
  GLCD_SetWindow(20,54,160,68);
  GLCD_TextSetPos(0,0);
  GLCD_print("Max. Freq: ");
  GLCD_print("%s",FreqmaxString,n);
  GLCD_print(" Hz   ");
  
  // Frequency Min display
  char FreqminString [10]; // destination string
  d = (int) freqmin*1000; // Decimal precision: 3 digits
  f = (freqmin * 1000)-d;
  d = (int) d/1000;
  if((f<100) && (f>9)){
    n=sprintf ( FreqminString, "%d.0%d", d, f); 
  }
  if (f<10){
    n=sprintf ( FreqminString, "%d.00%d", d, f); 
  }
  if (f>=100){
    n=sprintf ( FreqminString, "%d.%d", d, f);
  }

  GLCD_SetFont(&Terminal_9_12_6,red,black);
  GLCD_SetWindow(20,70,160,84);
  GLCD_TextSetPos(0,0);
  GLCD_print("Min. Freq: ");
  GLCD_print("%s",FreqminString,n);
  GLCD_print(" Hz   ");
  
  // Power display
  char PowString [10]; // destination string
  d = (int) Power*1000; // Decimal precision: 3 digits
  f = (Power * 1000)-d;
  d = (int) d/1000;
  if((f<100) && (f>9)){
    n=sprintf ( PowString, "%d.0%d", d, f); 
  }
  if (f<10){
    n=sprintf ( PowString, "%d.00%d", d, f); 
  }
  if (f>=100){
    n=sprintf ( PowString, "%d.%d", d, f);
  }
  
  
  GLCD_SetFont(&Terminal_9_12_6,white,black);
  GLCD_SetWindow(170,38,310,52);
  GLCD_TextSetPos(0,0);
  GLCD_print("Power: ");
  GLCD_print("%s",PowString,n);
  GLCD_print(" W     ");
  
  // Voltage display
  char VoltString [10]; // destination string
  d = (int) Vrms*1000; // Decimal precision: 3 digits
  f = (Vrms * 1000)-d;
  d = (int) d/1000;
  if((f<100) && (f>9)){
    n=sprintf ( VoltString, "%d.0%d", d, f); 
  }
  if (f<10){
    n=sprintf ( VoltString, "%d.00%d", d, f); 
  }
  if (f>=100){
    n=sprintf ( VoltString, "%d.%d", d, f);
  }

  GLCD_SetFont(&Terminal_9_12_6,white,black);
  GLCD_SetWindow(170,54,310,68);
  GLCD_TextSetPos(0,0);
  GLCD_print("Voltage: ");
  GLCD_print("%s",VoltString,n);
  GLCD_print(" V   ");
  
  // Current display
  char CurString [10]; // destination string
  d = (int) Irms*1000; // Decimal precision: 3 digits
  f = (Irms * 1000)-d;
  d = (int) d/1000;
  if((f<100) && (f>9)){
    n=sprintf ( CurString, "%d.0%d", d, f); 
  }
  if (f<10){
    n=sprintf ( CurString, "%d.00%d", d, f); 
  }
  if (f>=100){
    n=sprintf ( CurString, "%d.%d", d, f);
  }
  
  GLCD_SetFont(&Terminal_9_12_6,white,black);
  GLCD_SetWindow(170,70,310,84);
  GLCD_TextSetPos(0,0);
  GLCD_print("Current: ");
  GLCD_print("%s",CurString,n);
  GLCD_print(" A   ");

}

/******************************************
 *Function Name: Relay Status display

Displays if the bulb and socket relay ar on, off or at auto.
Also displays the percentage calculations for the relays.
******************************************/

void relayStatusDisp (void)
{
  if(bulbON == 1){
    GLCD_SetFont(&Terminal_9_12_6,white,black);
    GLCD_SetWindow(20,115,70,129);
    GLCD_TextSetPos(0,0);
    GLCD_print("Bulb: ON  ");
  }
  if(bulbON == 0){
    GLCD_SetFont(&Terminal_9_12_6,white,black);
    GLCD_SetWindow(20,115,70,129);
    GLCD_TextSetPos(0,0);
    GLCD_print("Bulb: OFF ");
  }
  if(socketON == 1){
    GLCD_SetFont(&Terminal_9_12_6,white,black);
    GLCD_SetWindow(170,115,235,129);
    GLCD_TextSetPos(0,0);
    GLCD_print("Socket: ON  ");
  }
  if(socketON == 0){
    GLCD_SetFont(&Terminal_9_12_6,white,black);
    GLCD_SetWindow(170,115,235,129);
    GLCD_TextSetPos(0,0);
    GLCD_print("Socket: OFF ");
  }
 
  if((bulb == 1) && (bulbON == 2)){
    GLCD_SetFont(&Terminal_9_12_6,white,black);
    GLCD_SetWindow(20,115,70,129);
    GLCD_TextSetPos(0,0);
    GLCD_print("Bulb: ON  ");
  }
  if((bulb == 0) && (bulbON == 2)){
    GLCD_SetFont(&Terminal_9_12_6,white,black);
    GLCD_SetWindow(20,115,70,129);
    GLCD_TextSetPos(0,0);
    GLCD_print("Bulb: OFF ");
  }
  if((socket == 1) && (socketON == 2)){
    GLCD_SetFont(&Terminal_9_12_6,white,black);
    GLCD_SetWindow(170,115,235,129);
    GLCD_TextSetPos(0,0);
    GLCD_print("Socket: ON  ");
  }
  if((socket == 0) && (socketON == 2)){
    GLCD_SetFont(&Terminal_9_12_6,white,black);
    GLCD_SetWindow(170,115,235,129);
    GLCD_TextSetPos(0,0);
    GLCD_print("Socket: OFF ");
  }
  
  
  //Percentage display for bulb relay
  char bulbString [10]; // destination string
  d = bulbP; // Decimal precision: 3 digits
  n=sprintf ( bulbString, "%d", d); 
  
  
  GLCD_SetFont(&Terminal_9_12_6,red,black);
  GLCD_SetWindow(90,115,160,129);
  GLCD_TextSetPos(0,0);
  GLCD_print("(ON: ");
  GLCD_print("%s",bulbString,n);
  GLCD_print("%%) ");
  
  //Percentage display for socket relay
  char socketString [10]; // destination string
  d = socketP; // Decimal precision: 3 digits
  n=sprintf ( socketString, "%d", d); 
  
  GLCD_SetFont(&Terminal_9_12_6,red,black);
  GLCD_SetWindow(245,115,310,129);
  GLCD_TextSetPos(0,0);
  GLCD_print("(ON: ");
  GLCD_print("%s",socketString,n);
  GLCD_print("%%) ");
  
  
}

/******************************************
 *Function Name: Button display
******************************************/

void buttonDisp (void)
{
  //Bulb ON
  if(bulbON == 1){
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(10,170,100,182);
    GLCD_TextSetPos(0,0);
    GLCD_print("      Bulb            ");
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(10,182,100,194);
    GLCD_TextSetPos(0,0);
    GLCD_print("       ON          ");
  }
  if((bulbON == 0) || (bulbON == 2)){
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(10,170,100,182);
    GLCD_TextSetPos(0,0);
    GLCD_print("      Bulb            ");
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(10,182,100,194);
    GLCD_TextSetPos(0,0);
    GLCD_print("       ON          ");
  }
  
  //Bulb OFF
  if(bulbON == 0){
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(110,170,210,182);
    GLCD_TextSetPos(0,0);
    GLCD_print("       Bulb            ");
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(110,182,210,194);
    GLCD_TextSetPos(0,0);
    GLCD_print("       OFF          ");
  }
  if((bulbON == 1) || (bulbON == 2)){
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(110,170,210,182);
    GLCD_TextSetPos(0,0);
    GLCD_print("       Bulb            ");
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(110,182,210,194);
    GLCD_TextSetPos(0,0);
    GLCD_print("       OFF          ");
  }
  
  //Bulb AUTO
  if(bulbON == 2){
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(220,170,310,182);
    GLCD_TextSetPos(0,0);
    GLCD_print("      Bulb            ");
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(220,182,310,194);
    GLCD_TextSetPos(0,0);
    GLCD_print("      AUTO          ");
  }
  if((bulbON == 0) || (bulbON == 1)){
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(220,170,310,182);
    GLCD_TextSetPos(0,0);
    GLCD_print("      Bulb            ");
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(220,182,310,194);
    GLCD_TextSetPos(0,0);
    GLCD_print("      AUTO          ");
  }
  
  //Socket ON
  if(socketON == 1){
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(10,205,100,217);
    GLCD_TextSetPos(0,0);
    GLCD_print("     Socket            ");
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(10,217,100,229);
    GLCD_TextSetPos(0,0);
    GLCD_print("       ON          ");
  }
  if((socketON == 0) || (socketON == 2)){
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(10,205,100,217);
    GLCD_TextSetPos(0,0);
    GLCD_print("     Socket            ");
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(10,217,100,229);
    GLCD_TextSetPos(0,0);
    GLCD_print("       ON          ");
  }
  
  //Socket OFF
  if(socketON == 0){
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(110,205,210,217);
    GLCD_TextSetPos(0,0);
    GLCD_print("      Socket            ");
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(110,217,210,229);
    GLCD_TextSetPos(0,0);
    GLCD_print("       OFF          ");
  }
  if((socketON == 2) || (socketON == 1)){
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(110,205,210,217);
    GLCD_TextSetPos(0,0);
    GLCD_print("      Socket            ");
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(110,217,210,229);
    GLCD_TextSetPos(0,0);
    GLCD_print("       OFF          ");
  }
  
  //Socket AUTO
  if(socketON == 2){
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(220,205,310,217);
    GLCD_TextSetPos(0,0);
    GLCD_print("     Socket            ");
    GLCD_SetFont(&Terminal_9_12_6,white,darkRed);
    GLCD_SetWindow(220,217,310,229);
    GLCD_TextSetPos(0,0);
    GLCD_print("      AUTO          ");
  }
  
  if((socketON == 0) || (socketON == 1)){
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(220,205,310,217);
    GLCD_TextSetPos(0,0);
    GLCD_print("     Socket            ");
    GLCD_SetFont(&Terminal_9_12_6,white,red);
    GLCD_SetWindow(220,217,310,229);
    GLCD_TextSetPos(0,0);
    GLCD_print("      AUTO          ");
  }
}

/******************************************
 *Function Name: Touch Button

Changes the state of bulbON and socketON depending on what button is touched
0 = OFF, 1 = ON, 2 = AUTO
******************************************/

void touchButton (int cursor_x, int cursor_y)
{
  if((cursor_y > 170) && (cursor_y < 194)){
        if(cursor_x < 100){
          bulbON = 1;
        }
        else if((cursor_x > 110 && cursor_x < 210)){
          bulbON = 0;
        }
        else if(cursor_x > 220){
          bulbON = 2;
        }
      }
      if((cursor_y > 205) && (cursor_y < 229)){
        if(cursor_x < 100){
          socketON = 1;
        }
        else if((cursor_x > 110 && cursor_x < 210)){
          socketON = 0;
        }
        else if(cursor_x > 220){
          socketON = 2;
        }
      }
}

/*************************************************************************
 * Function Name: main
 * Parameters: none
 *
 * Return: none
 *
 * Description: main
 *
 *************************************************************************/
 int main(void)
{
typedef Int32U ram_unit;
Int32U cursor_x = (C_GLCD_H_SIZE - CURSOR_H_SIZE)/2, cursor_y = (C_GLCD_V_SIZE - CURSOR_V_SIZE)/2;
ToushRes_t XY_Touch;
Boolean Touch = FALSE;

  GLCD_Ctrl (FALSE);

  // Init GPIO
  GpioInit();
#ifndef SDRAM_DEBUG
  // MAM init
  MAMCR_bit.MODECTRL = 0;
  MAMTIM_bit.CYCLES  = 3;   // FCLK > 40 MHz
  MAMCR_bit.MODECTRL = 2;   // MAM functions fully enabled
  // Init clock
  InitClock();
  // SDRAM Init
  SDRAM_Init();
#endif // SDRAM_DEBUG
  // Init VIC
  VIC_Init();
  
  //Initialize Converters
  ADC_Init();
  //DAC_Init();
  
  //IO initialization
   FIO0DIR_bit.P0_11 = 1; //Relay Bulb
   FIO0DIR_bit.P0_19 = 1; //Relay Socket
  
  // GLCD init
  GLCD_Init (NULL, NULL);
  
  // Init touch screen
  TouchScrInit();
  
  //Heartbeat indication LED
  USB_D_LINK_LED_SEL = 0; // GPIO
  USB_D_LINK_LED_FSET = USB_D_LINK_LED_MASK;
  USB_D_LINK_LED_FDIR = USB_D_LINK_LED_MASK;
  

  // Touched indication LED
  USB_H_LINK_LED_SEL = 0; // GPIO
  USB_H_LINK_LED_FSET |= USB_H_LINK_LED_MASK;
  USB_H_LINK_LED_FDIR |= USB_H_LINK_LED_MASK;

  // Enable TIM0 clocks
  PCONP_bit.PCTIM0 = 1; // enable clock

  // Init Time0
  T0TCR_bit.CE = 0;     // counting  disable
  T0TCR_bit.CR = 1;     // set reset
  T0TCR_bit.CR = 0;     // release reset
  T0CTCR_bit.CTM = 0;   // Timer Mode: every rising PCLK edge
  T0MCR_bit.MR0I = 1;   // Enable Interrupt on MR0
  T0MCR_bit.MR0R = 1;   // Enable reset on MR0
  T0MCR_bit.MR0S = 0;   // Disable stop on MR0
  // set timer 0 period
  T0PR = 0;
  T0MR0 = SYS_GetFpclk(TIMER0_PCLK_OFFSET)/(TIMER0_TICK_PER_SEC);
  // init timer 0 interrupt
  T0IR_bit.MR0INT = 1;  // clear pending interrupt
  VIC_SetVectoredIRQ(Timer0IntrHandler,0,VIC_TIMER0);
  VICINTENABLE |= 1UL << VIC_TIMER0;
  T0TCR_bit.CE = 1;     // counting Enable
  __enable_interrupt();

  //Initial screen display
  initDisp();
  systemStatusDisp();
  relayStatusDisp();
  buttonDisp();
  
  GLCD_Ctrl (TRUE);

  while(1)
  {    
    if(timer>10){
      //Displaying system status and changes
      systemStatusDisp();
      timer = 0;
    }
    timer++;
    
    //display relaystatus and button display
    relayStatusDisp();
    buttonDisp();
 
    if(TouchGet(&XY_Touch))
    {
      
      //Get the x and y coordination of the touch
      cursor_x = XY_Touch.X;
      cursor_y = XY_Touch.Y;
      
      //If a button is touched
      touchButton(cursor_x, cursor_y);
      
      if (FALSE == Touch)
      {
        Touch = TRUE;
        USB_H_LINK_LED_FCLR = USB_H_LINK_LED_MASK;
      }
    }
    else if(Touch)
    {
      USB_H_LINK_LED_FSET = USB_H_LINK_LED_MASK;
      Touch = FALSE;
    }
    // percentage calculation for the relays
    percentage();
  }
}
