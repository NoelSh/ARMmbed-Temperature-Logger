//INTRODUCTION
/*------------------------------------------------------------------------------
Author:     Oluwole O. Oyetoke
Date:       25th November, 2016
SUMMARY:    This code is written to run on the ARMmbed LPC1768 board.
            It is developed to configure the board which is connected to a 
            TMP102 to constantly measure temperature, update the display
            evey minute and also plot these values over time on the LCD
------------------------------------------------------------------------------*/



//HEADER INCLUSION & DEFINITIONS
//------------------------------------------------------------------------------
#include "mbed.h"   //mbed header file inclusion
#include "N5110.h"  //Nokia LCD Screen header file inclusion
#include <math.h>

//Define slave address for different conditions
//ADD0 of TMP102 is connected to ground... address will be Ob1001000 = 0x48
#define slaveAddress       0x48
#define slaveWriteAddress  0x90
#define slaveReadAddress   0x91


/*Define needed register addresses
--------------------------------------------------------------
| BIT FUNCTION | P7 | P6 | P5 | P4 | P3 |  P2 | P1 | p0 |    |
|    BYTE 1    | 0  | 0  | 0  | 1  | 1  |  0  | 0  | 0  |    |
|------------------------------------------------------------| 
P1 P0      REGISTER
0  0   TEMPERATURE REGISTER (Read Only)
0  1   CONFIGURATION REGISTER (Read/Write)
1  0   TLOW REGISTER (Read/Write)
1  1   THIGH REGISTER (Read/Write)
*/
#define tempRegAddress   0x00 //0b00000000
#define configRegAddress 0x01; //0b00000001
#define tlowRegAddress   0x02; //0b00000010
#define thighRegAddress  0x03; //0b00000011
//------------------------------------------------------------------------------
    

//PIN CONNECTION SET-UP
//------------------------------------------------------------------------------
//Set pin connection to LCD
N5110 lcd(p7,p8,p9,p10,p11,p13,p21);

//Set pin connection for the TMP102
I2C tmp102(p28, p27); //p28-SDA, p27-SCL

//Set pins for error display leds
BusOut myleds(LED1, LED2, LED3, LED4);

//Set pins for USB connectivity 
Serial pc(USBTX,USBRX); //Usually (p9,p10) for external devices

//Set logging control pin
DigitalIn loggerSwitch(p18);
DigitalIn plotterModeButton(p16);
DigitalIn standbyButton(p17);

//Set interrupt on pin 16 and 17
InterruptIn plotterInterrupt(p16);
InterruptIn standbyInterrupt(p17);
//------------------------------------------------------------------------------


//DEFINE NEEDED OBJECTS
//------------------------------------------------------------------------------
Ticker tempIntervalReader, timeIntervalReader, fileWriteInterval;
//------------------------------------------------------------------------------


//FUNCTION DEFINITION
//------------------------------------------------------------------------------
void ErrorBuzzer(int code);
void initializeTMP102(); //Function used to initialize the TMP102 & configure
void readTemperature();  //Function used to read the temperature momentarily
void serialISR();        //ISR that is called when serial data is received
void setTime();          //Function to set the UNIX time
void writeDataToFile(int data); //Function for writing to file
void updateLCDTime();     //Function to update LCD Time
void trigerViewChange();       //Function used to trigger view change
void plot();
void standbyFunction();
//------------------------------------------------------------------------------



//DEFINE GLOBAL VARIABLES
//------------------------------------------------------------------------------
char rxString[16];          //Buffer to store received string
int setTimeFlag = 0;        //Flag for ISR
int setLogFlag = 0;         //Flag to control logging
int setPlotFlag = 0;        //Flag to switch in and out of plot mode
int LCDStanbyFlag=0;        //Flag to stanby LCD
float temperatureValue = 0;   //Initial temperature value
char tempValueDispBuffer[14];  // Screen can only take 14 characters/line @ maximum
char timeValueDispBuffer[30];  // Screen can only take 14 characters/line @ maximum
char dateValueDispBuffer[30];  // Screen can only take 14 characters/line @ maximum
char datetimeValueDispBuffer[30];  // Screen can only take 14 characters/line @ maximum
LocalFileSystem local("local"); //Create Local File System
int xAxisCounter=0;
//------------------------------------------------------------------------------



//FUNCTION TO SET RTC TIME THROUGH SERIAL CONNECTION
//------------------------------------------------------------------------------
void setTime() {
 pc.printf("set_time - %s",rxString);// print time for debugging
 int time = atoi(rxString);         // atoi() converts a string to an integer
 set_time(time);                    // update the time
}
//------------------------------------------------------------------------------

//FUNCTION TO UPDATE LCD WITH NEW TIME READING
//------------------------------------------------------------------------------
void updateLCDTime(){
             time_t seconds = time(NULL); // Get current time
   
 // Format time into a string (time and date)
 strftime(timeValueDispBuffer, 30 , "   %X", localtime(&seconds));
strftime(dateValueDispBuffer, 30 , "   %x", localtime(&seconds));
strftime(datetimeValueDispBuffer,30 , "%x %X", localtime(&seconds));

 
 // print time on LCD
 if((setPlotFlag==0)){
            lcd.printString(dateValueDispBuffer,0,2);   // Print on LCD  
            lcd.printString(timeValueDispBuffer,0,3);   // Print on LCD 
            } 
}
//--------------------------------------------------------------------------------


//SERIAL LINK INTERRUPT SERVICE ROUTINE
//------------------------------------------------------------------------------
void serialISR() {
 // when a serial interrupt occurs, read rx string into buffer
 pc.gets(rxString,16);
 // set flag
 setTimeFlag = 1;
}
//------------------------------------------------------------------------------


// FUNCTION USED TO INITIATE LED FLASHING WHEN AN ERROR OCCURS
//------------------------------------------------------------------------------
void ErrorBuzzer(int code)
{
 while(1) {         //hang in infinite loop flashing error code
 myleds = 0;        //LEDs OFF
 wait(0.25);        //Wait for 1/4th of a second
 myleds = code;     //LEDs ON
 wait(0.25);        //Wait for 1/4th of a second
 }
}
//------------------------------------------------------------------------------


//FUNCTIONUSED TO SET-UP TMP102 CONFIGURATION REGISTER AS DESIRED
//------------------------------------------------------------------------------
void initializeTMP102(){    
    
    //Set bus speed to 400kHz
    tmp102.frequency(400000);
    
    int acknowledgement = 1; //Initialized to false

    /*
    Configure the 16 bits register structure of the configuration register
--------------------------------------------------------------
| BIT FUNCTION | OS | R1 | R0 | F1 | F0 | POL | TM | SD |    |
|    BYTE 1    | 0  | 1  | 1  | 1  | 1  |  0  | 0  | 0  |    |
|------------------------------------------------------------|     
| BIT FUNCTION | CR1 | CR0 | AL | EM | 0 | 0 | 0 | 0 |       |
|    BYTE 2    |  0  | 1   | 1  | 1  | 1 | 0 | 0 | 0 |       |                                                        |
--------------------------------------------------------------   
 BIT FUNTION EXPLANTION
 OS (ONE-SHOT): 0            --> not in use 
 R1, R2 (RESOLUTION):  1, 1  --> Sets the temperature register to 12 bits 
 F1, F0 (FALUT QUE): 1,1     --> 6 consecutive faults before alert is triggered
 POL (POLARITY): 0           --> Alert pin will be active low
 TM (THERMOSTAT MODE): 0     --> Comparator mode activated
 SD (SHUTDOWN MODE): 0       --> Continious conversion. No intermitent shutdown
 CR1, CR0 (CONV. RES.): 0, 1 --> 1Hz, 1 conversion per second
 AL (ALERT): 1               --> Read only + only takes the opposite vaue of POL
 EM (EXTENDED MODE): 0       --> Normal Mode. Sets TREG,THIGH,TLOW reg. to 12 bts
    */
    char byteOne = 0x70; //0b01111000
    char byteTwo = 0x60; //0b01100000
    
    
//Initiate contact with slave, specify desired register and the data to write.
//Package contains address and the data to write
    char configPackage[3];
    configPackage[0] = configRegAddress;
    configPackage[1] = byteOne;
    configPackage[2] = byteTwo;
    acknowledgement = tmp102.write(slaveWriteAddress, configPackage, 3); 
    if(acknowledgement==1){ //I2C always returns 0 for successful operation
       ErrorBuzzer(1); //Error Message
        }   
    }
//------------------------------------------------------------------------------





//Temperature Reader Function
//------------------------------------------------------------------------------
void readTemperature(){
    int acknowledgement=1; //Initialized to 1
    char tempRegisterData[2];
    int temperature = 0;
   //Inititate contact with slave by trying to write to its temp register
   //Temp register is a read only register, so it will not happen
   acknowledgement = tmp102.write(slaveWriteAddress,tempRegAddress,1); 
   if(acknowledgement==1){ //I2C always returns 0 for successful operation
       ErrorBuzzer(2); //Error Message
        } 
     //Try to read from temperature register   
    acknowledgement = tmp102.read(slaveReadAddress,tempRegisterData,2);
   if(acknowledgement==1){ //I2C always returns 0 for successful operation
       ErrorBuzzer(3); //Error Message
        } 
        
        temperature = (tempRegisterData[0] << 4) | (tempRegisterData[1] >> 4);
        temperatureValue = temperature*0.0625;
        if(setPlotFlag==1){
              plot();
            }
//return temperature*0.0625;
    //float tmp = (float((tempRegisterData[0]<<8)|tempRegisterData[1]) / 256.0);
    }
//------------------------------------------------------------------------------


//FUNCTION FOR WRITING TO FILE
//------------------------------------------------------------------------------
void writeDataToFile()
{
 if(setLogFlag==1){  //Only record if logging is turned ON
 myleds = 15; // turn on LEDs for feedback
 FILE *fp = fopen("/local/templog.csv", "a"); // open 'templog.txt' for appending
 // if the file doesn't exist it is created, if it exists, data is appended to the end
 fprintf(fp,"%s, %.2f \n",datetimeValueDispBuffer, temperatureValue ); // print string to file
 fclose(fp); // close file
 myleds = 0; // turn off LEDs to signify file access has finished
 }
}
//------------------------------------------------------------------------------


//INTERRUPT FUNCTION FOR DETECTING PLOT vs WRITE MODE
//------------------------------------------------------------------------------
void trigerViewChange() {
    if(setPlotFlag==0){
        setPlotFlag=1;
        lcd.clear();
        for(int i=0; i<=83; i++){
            lcd.setPixel(i,45);
        }
        
        for(int i=0; i<=47; i++){
          
            lcd.setPixel(2,i);
        }
        }
        else if(setPlotFlag==1){
            setPlotFlag = 0;
               lcd.clear();
                lcd.printString(" TEMP. LOGGER!",0,0);        //Print Welcome Message
            }
           
}
//------------------------------------------------------------------------------


//FUNCTION FOR PLOTTING TEMPERATURE VALUE ON LCD
//------------------------------------------------------------------------------
void plot(){
  //Note that 0,0 of the LCD starts from thr top left hand corner
  //Pixels are addressed in the range of 0 to 47 (y) and 0 to 83 (x).
  int tempIntegerValue=8;
  //Plot only tempertures between 0 and 47 degree celcious
     tempIntegerValue = floor(temperatureValue);
 //Note that 0,0 of the LCD starts from thr top left hand corner
 int rightYPosition=0;
   
   
   if (setPlotFlag==1){
     if((tempIntegerValue<47) && (xAxisCounter<=83) ){
       rightYPosition = 47-tempIntegerValue;
       lcd.setPixel(xAxisCounter,  rightYPosition);
        lcd.refresh();
       xAxisCounter++; 
       sprintf(tempValueDispBuffer,"Now %.2f C",temperatureValue); 
            lcd.printString(tempValueDispBuffer,0,0);   // Print on LCD 
       }else if(xAxisCounter>83){ 
           lcd.clear();
           lcd.refresh();
           xAxisCounter=0;
           rightYPosition = 47-tempIntegerValue;
            lcd.setPixel(xAxisCounter,  rightYPosition);
             lcd.refresh();
           }
       else{
           xAxisCounter=0;
           
           }
           }
           
    }
//------------------------------------------------------------------------------


//STANDBY AND WAKE FUNCTION
//------------------------------------------------------------------------------
    void standbyFunction(){
        if(LCDStanbyFlag==0){
   lcd.turnOff(); 
     LCDStanbyFlag = 1;
   }
   else if (LCDStanbyFlag==1){
           lcd.init();                             //Initialize and Turn ON LCD
    if((setPlotFlag==0)){
    lcd.printString(" TEMP. LOGGER!",0,0);        //Print Welcome Message
    }else{
                          // lcd.printString(" PLT MODE,",0,0); 
                           }
     LCDStanbyFlag = 0;
        }
        
        }
//------------------------------------------------------------------------------    


//MAIN FUNCTION
//------------------------------------------------------------------------------
int main() {
  
    //Initialize temperature display lengths
    int tempDispLength=0;
    
    plotterModeButton.mode(PullUp);
    standbyButton.mode(PullUp);
    
    lcd.init();                            //Initialize LCD
    if((setPlotFlag==0)){
    lcd.printString(" TEMP. LOGGER!",0,0); //Print Welcome Message
    }
    
    initializeTMP102();                    //Initialize Temperature Sensor
    
    
    // Attach serialISR function to the ISR and set serial baud rate
    pc.attach(&serialISR); 
    pc.baud(9600);
    
    
    //Attach readTemperature function to the ticker. 
    //Schedule to interrup every 60 seconds
    readTemperature();
    tempIntervalReader.attach(&readTemperature, 2.0);
    
    //Attach write to file operation to  a 60 seconds ticker
    fileWriteInterval.attach(&writeDataToFile, 2.0);
    
    //Attach updateLCDTime function to the ticker. 
    //Schedule to interrupt and update LCDTime every 1 second 
    timeIntervalReader.attach(&updateLCDTime, 1.0);
    
    //Attach interrupt to button on p16 to trigger view change to and from plotter 
     plotterInterrupt.fall(&trigerViewChange);
     
     //Attach standby function to interrupt on button p17
     standbyInterrupt.fall(&standbyFunction); 

    //Value '0' will initialise time to 1st January 1970 
    //set_time(1480958460);  //Uncomment after RTC clock value has been set

   //Continue spining this loop, however, every
   // 60 seconds, the ticker will be called and the time updated ever 1s
    while(1) {
        if((setLogFlag==0) && (setPlotFlag==0)){
            lcd.printString("  LOGGING OFF  ",0,5);
            }
            else if((setLogFlag==1) && (setPlotFlag==0)){
                lcd.printString("  LOGGING ON  ",0, 5);
                }
        
   //Display Current Temperature Reading
   // print formatted data to buffer
    tempDispLength = sprintf(tempValueDispBuffer,"  T = %.2f C",temperatureValue); 
    if ((tempDispLength <= 14) && (setPlotFlag==0))  {
            lcd.printString(tempValueDispBuffer,0,4);   // Print on LCD 
                                }
                                
   //If updated time has been sent through serial
        if (setTimeFlag==1) { 
            setTimeFlag = 0; // clear set time flag
            setTime(); // Call update time function
                            }
                    
    //Check if Logging is ON
    if(loggerSwitch){
           setLogFlag=1;           
                      }
                      else{
                               setLogFlag=0;  
                          } 
                          
     //Switch to plotter Mode through button toggle                                      
                    if(setPlotFlag==0){
                        lcd.printString("   VIEW MODE",0,1);
                        }else{
                         //  lcd.printString(" PLT MODE,",0,0);  
                            }
    }
}
//------------------------------------------------------------------------------