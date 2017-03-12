

#include <LED.h>
#include <Time.h>
#include <TimeLib.h>
#include <TimerObject.h>
#include <SerialCRC.h>
#include <eRCaGuy_analogReadXXbit.h>
#include <EEPROM.h>


// calculate Analog Reference Voltage from built-in 1.1V reference
#include <AnalogVRef.h>
double Vin_ARef;
double VCC_ARef;
double VAREF;


struct EEPROMStore {
  double BatteryAmpSeconds;
  double BatteryAmpHours;
  int writeCount;
  unsigned long CRC;
};

EEPROMStore EpromData = {0.0,0.0,0,0};


const uint8_t PUSHBUTTON_PIN = 3;
const uint8_t LIGHT_PIN = LED_BUILTIN;
const uint8_t RX_PIN = 7;
const uint8_t TX_PIN = 8;  // don't forget resistor divider for 3.3V (2kohm/1kohm)
const uint8_t IB_PIN = A3;
const uint8_t VB_PIN = A1;

//Global constants
const unsigned int num_samples = 10;  //change this to 1 to take only a single reading; leave it >1 to return an avg. of this # of readings
//const uint8_t pin = A0; //analogRead pin
const uint8_t pin = 0; //analogRead pin
//constants required to determine the voltage at the pin
//BE SURE YOU USE THE CORRECT ONE OF THESE WHEN CALCULATING THE VOLTAGE FROM A READING! Take notes of how these constants are used below.
const float MAX_READING_10_bit = 1023.0;
const float MAX_READING_11_bit = 2047.0;
const float MAX_READING_12_bit = 4095.0;

//id strings to send with readings
const char BatteryVoltage_STRING[] = "vb=";
const char BatteryCurrent_STRING[] = "ib=";
const char BatteryUsed_STRING[] = "qbused=";

bool powerFailed = false;

double V; //Voltage calculated on the analog pin
double analog_reading; //the ADC reading
double VBmultiplier = 42/4.2;
double VBattery;
double IBattery;
double IBatteryScale=10.0; // Amps/Volt of Input (100 mV/Amp)
double powerLossThreshold = 10.0; //Assume Power Lost if Battery Voltage below this threshold.
double powerRestoredThreshold = 12.0;  //Assume power restored above this battery voltage.
int IntervalCurrentMeasurement = 100; //milliseconds
double tIntervalCurrent = IntervalCurrentMeasurement / 1000.0; //current measurement interval in seconds

//instantiate an object of this library class; call it "adc"
eRCaGuy_analogReadXXbit adc;

TimerObject *T_UpdateCurrentReadings = new TimerObject(IntervalCurrentMeasurement);
TimerObject *T_UpdateDisplay = new TimerObject(1000);
TimerObject *T_UpdateDisplayFast = new TimerObject(100);

LED statusLED = LED(LIGHT_PIN);
static swSerialCRC swSerial(RX_PIN, TX_PIN); // RX, TX


void updateReadings(){
    //local variables
  uint8_t bits_of_precision; //bits of precision for the ADC (Analog to Digital Converter)
  //12-bit ADC reading
  bits_of_precision = 12; //bits of precision for the ADC (Analog to Digital Converter)
  
  //Read the battery voltage
  analog_reading = adc.analogReadXXbit(VB_PIN,bits_of_precision,num_samples); //get the avg. of [num_samples] 12-bit readings
  V = analog_reading/MAX_READING_12_bit*VAREF; //voltage
  VBattery = V * VBmultiplier;

 
}

void updateCurrentReadings(){
  //local variables
  uint8_t bits_of_precision; //bits of precision for the ADC (Analog to Digital Converter)
  //12-bit ADC reading
  bits_of_precision = 12; //bits of precision for the ADC (Analog to Digital Converter)
  double Vzerocurrent; //the current sensor outputs 1/2 supply voltage at 0 current.
  
    // calculate Analog Reference voltage by measuring internal 1.1V reference
// these are 2 different versions of the same thing.
    Vin_ARef = read_vin_mv()/1000.0;  //this seems a bit flaky
    VCC_ARef = readVcc()/1000.0;      //this seems solid

    // fudge factor varies by board.
    VAREF = VCC_ARef*0.9541;
    //maybe smooth this result?
    
    //Read the battery current
  analog_reading = adc.analogReadXXbit(IB_PIN,bits_of_precision,num_samples); //get the avg. of [num_samples] 12-bit readings
  V = analog_reading/MAX_READING_12_bit*VAREF; //voltage
  Vzerocurrent = VAREF / 2.0;
  IBattery = (V-Vzerocurrent)*IBatteryScale;  //Battery Current in Amps.

  //Calculate the battery charge used.
  EpromData.BatteryAmpSeconds = EpromData.BatteryAmpSeconds + IBattery * tIntervalCurrent;
  if ( EpromData.BatteryAmpSeconds >= 36.0 ) {
    EpromData.BatteryAmpHours = EpromData.BatteryAmpHours + 0.01;
    EpromData.BatteryAmpSeconds = EpromData.BatteryAmpSeconds - 36.0;
    }
  
  
}

void updateDisplay(){
  //send readings to the Display
  //Battery voltage, current, and charge used.
  swSerial.print(BatteryVoltage_STRING);
  swSerial.println(VBattery,1);

  swSerial.print(BatteryUsed_STRING);
  swSerial.println(EpromData.BatteryAmpHours);
  
}

void updateDisplayFast(){
  //send readings to the Display
  //Battery current.
  char stmpBuf[15];
  swSerial.print(BatteryCurrent_STRING);
  swSerial.println(IBattery);
  
}

void readEPROM() {
  //restore values from EEPROM from last power off
  EEPROM.get(0,EpromData);
}


/* new comment */

void updateEPROM() {
  //save values in EEPROM that we want to keep across power cycles.
  EpromData.writeCount++;
  EEPROM.put(0,EpromData);
}

void checkPowerFailure() {
  //If battery voltage falls below threshold value, 
  // then save values to EPROM.
  if ( !powerFailed && (VBattery < powerLossThreshold )) {
    updateEPROM();
    //set a flag so that we only do this once per powerfailure.
    powerFailed = true;
  } 
  else if ( powerFailed && (VBattery > powerRestoredThreshold )){
    powerFailed = false;
  }
  
}

void processCommands(){
  String inCommand;
  //process any commands from serial line
  if (Serial.available() > 0) {
    inCommand = Serial.readStringUntil('\n');
  }
  inCommand.toLowerCase();
  //battery has been recharged, reset usage
  if (inCommand.equalsIgnoreCase("rq:C9F64A58")){
    
  }
  //dump EPROM data
  if (inCommand.equalsIgnoreCase("eprom")) {
    updateEPROM();
    readEPROM();
    swSerial.println(EpromData.BatteryAmpSeconds);
    swSerial.println(EpromData.BatteryAmpHours);
    swSerial.println(EpromData.writeCount);
    swSerial.println(EpromData.CRC);
    
  }
}
void setup() {

  swSerial.begin(4800);
  //initialize the serial port for debugging output
  Serial.begin(9600);
  
  pinMode(LIGHT_PIN, OUTPUT);

  //blink this program number on the status led
  statusLED.off();
  delay(500);
  statusLED.blink(2*200,2);
  delay(500);
  statusLED.blink(5*200,5);

  //set up timers
  T_UpdateCurrentReadings->setOnTimer(&updateCurrentReadings);
  T_UpdateCurrentReadings->Start();

  T_UpdateDisplay->setOnTimer(&updateDisplay);
  T_UpdateDisplay->Start();
  
  T_UpdateDisplayFast->setOnTimer(&updateDisplayFast);
  T_UpdateDisplayFast->Start();
  

  VAREF = 5.0;

  //initialize the EEPROM the first time the program runs.
  //updateEPROM();
  //restore saved values from EEPROM
  readEPROM();
}

void loop() {

  updateReadings();
  checkPowerFailure();
  processCommands();
  
  //update the timers for updating input readings sending information for display
  T_UpdateCurrentReadings->Update();
  T_UpdateDisplay->Update();
  T_UpdateDisplayFast->Update();

}
