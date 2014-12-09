// Fredjke's !Craft Brewery
// Compatible with the DCCduino
/* --- Versions ---
// V1 : deprecated
// V2 : deprecated
// V3 : 2014-10-15
// V4 : 2014-10-15 - Adding potentiometer for T° Consign 
// V5 : 2014-10-16 - Adding Serial Interface + PID
// V6 : 2014-10-17 - Adding Temperature Curve consigns
// V7 : 2014-10-24 - Rewrite complet
// V8  : 2014-12-03 - Only temperature
// V9  : 2014-12-03 - Only LCD
// V10 : 2014-12-03 - Only PID
// V11 : 2014-12-05 - Complet
*/

/* --- Pin Assignment ---
// SCL/SDA : LCD Display
// 6  : Relay
// 9  : Buzzer
// 10 : OneWire Temperature sensors
// 13 : Push button
*/
 
// *** Declare global constants *** //
#define LCD_UPDATE_INTERVAL 3000 // Time in ms for swapping display between Clock/Consigns & Temperatures.
#define PID_UPDATE_INTERVAL 3000 // Time in ms for updating PID
#define HEAT_RELAY_PERIOD 20     // Define maximum number of LCD_UPDATE_INTERVAL for On/Off operation for Heater Relay. Relay protection mechanism.
#define Max_Sensors 2		 // Maximum number of sensors connected to platform.
#define Brew 0			 // Sensor 0 used for Brewing
#define Sparge 1			// Sensor 1 used for Sparge

// *** Declare Multi Rest temperatures ***
#define RestOneT 45 		// Temperature in C° for Proteinic Rest (45°C - 55°C)
#define RestOneD 0 		// Duration in minutes for Proteinic Rest (10min - 30min)
#define RestTwoT 63 		// Temperature in C° for Saccharification Beta-Amylase Rest (55°C - 65°C)
#define RestTwoD 0 		// Duration in minutes for Saccharification Beta-Amylase Rest (30min - 60min)
#define RestThreeT 68 		// Temperature in C° for Saccharification Alpha-Amylase Rest (68°C - 72°C)
#define RestThreeD 60 		// Duration in minutes for Saccharification Alpha-Amylase Rest (30min - 90min)
#define RestFourT 78 		// Temperature in C° for Mash-Out (78°C - 80°)
#define RestFourD 10 		// Duration in minutes for Mash-Out (10min - 15min)
#define BoilD 60 		// Duration in minutes for Boil Phase (default 60min)
#define HopOneD 60 		// Duration in minutes for Hop addition (60 min)
#define HopTwoD 15 		// Duration in minutes for Hop addition (15 min)
#define HopThreeD 10 		// Duration in minutes for Hop addition (10 min)
#define HopFourD 5 		// Duration in minutes for Hop addition (5 min)
#define HopFiveD 0 		// Duration in minutes for Hop addition (0 min)
#define BoilStepI 4		// Number of iterations with push button pressed to step into Boil Step
boolean HopOne = 1;  	// Presence of HopOne for Hop addition at HopOneD minutes
boolean HopTwo = 1;  	// Presence of HopTwo for Hop addition at HopTwoD minutes
boolean HopThree = 1;  	// Presence of HopThree for Hop addition at HopThreeD minutes
boolean HopFour = 0;  	// Presence of HopFour for Hop addition at HopFourD minutes
boolean HopFive = 1;  	// Presence of HopFive for Hop addition at HopFiveD minutes

// --- Declare Variables for Auto Brew Process ---
boolean AutoBrewSwitch = 0;
boolean ManualBrewSwitch = 0;
long iBrewTime = 0; 		// Store start millis of Brew process
byte BoilStep = 0;	// Count 4 iterations (LCD_UPDATE_INTERVAL) with Push button pressed for stepping over into Boil Step.
long StepTime = 0; 	// initialize millis at each rest
byte BrewStep = 0;      // initialize Brew steps to step 0
boolean Buzz = false;	// Buzzer initialization
	
// --- Declare global variables ---
float Celsius[Max_Sensors];		// Matrix of Temperature readings

float TargetTemp;  					//current temperature goal
float heatPower = 0.0; 					// 0 up to HEAT_RELAY_PERIOD*PID_UPDATE_INTERVAL milliseconds (Default : 20 * 3000 = 1 minute)
unsigned long lastPIDTime;  		// most recent PID update time in ms 
unsigned long lastBUZTime;			// most recent BUZ time in ms
// --- End of Declare global variables ---

// --- Declare LCD ---
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Define printByte Function to allow display of degree symbol : °
#if defined(ARDUINO) && ARDUINO >= 100 
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif
#define Degree  223 // ASCII Code for ° symbol (Degree)
byte ShowClockMode = 1; // Swap Message between Desired Temp during brewing and Hop addition during Boil
boolean Toggle; // Toggle between Temperature and clock.

void SetupLCD() {
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  delay(1000);
  lcd.print("Fredjke's !Craft ");
  lcd.setCursor(0, 1);
  lcd.print("Brewery v11 ");
  delay(500);
  lcd.setCursor(0, 1);
  lcd.print("Brewery v11 .");
  delay(500);
  lcd.setCursor(0, 1);
  lcd.print("Brewery v11 ..");
  delay(500);
  lcd.setCursor(0, 1);
  lcd.print("Brewery v11 ...");
  delay(1000);
  lcd.clear();
  // Serial.println("Fredjke's !Craft Brewery v11 ...");
  ShowClockMode = 1;
}
// --- End of Declare LCD ---

// --- Declare Temperature Sensors ---
#include <OneWire.h>
OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary if not using Keyes Module)
// --- End of Declare Temperature Sensors ---

// Variables for the Potentiometer - Led Output
int buzPin = 9;   // select the output pin for the Buzzer
int pusPin = 13;   // select the input pin for the Push Button

void SetupPush() {
//  Serial.println(" *** Setup Push Button *** ");
  pinMode(pusPin, INPUT);  // declare the pusPin as an INPUT
}
void SetupBuzzer() {
//  Serial.println(" *** Setup Buzzer *** ");
  pinMode(buzPin, OUTPUT);  // declare the buzPin as an OUTPUT
}
void updateBUZ(boolean on) { // turn the ledPin on or off depending on LedPot
	digitalWrite(buzPin, on); }
	
// *** Declare PID *** //
#define PGAIN_ADR 0
#define IGAIN_ADR 4
#define DGAIN_ADR 8
#define TEMP_ADR 12
#define WINDUP_GUARD_GAIN 6000.0 // Must be equal to the max amount of cycles allowed by the HEAT_RELAY_PERIOD*PID_UPDATE_INTERVAL

float iState = 0;
float lastTemp = 0;

float pgain, igain, dgain;								// PID Gains
float pTerm, iTerm, dTerm; 											// PID Terms
int pgainAddress, igainAddress, dgainAddress;						// PID addresses where gains are saved in Eeprom

void setupPID(unsigned int padd, int iadd, int dadd) {
  // with this setup, you pass the addresses for the PID algorithm to use to 
  // for storing the gain settings.  This way wastes 6 bytes to store the addresses,
  // but its nice because you can keep all the EEPROM address allocation in once place.

  pgainAddress = padd;
  igainAddress = iadd;
  dgainAddress = dadd;

  pgain = readFloat(pgainAddress);
  igain = readFloat(igainAddress);
  dgain = readFloat(dgainAddress);
}
void setTargetTemp(float t) {
  TargetTemp = t;
  writeFloat(t, TEMP_ADR);
}
float getTargetTemp() {
  return TargetTemp;
}
float getLastTemp() {
  return lastTemp;
}

float getP() { // get the P gain 
  return pgain;}
float getI() { // get the I gain
  return igain;}
float getD() { // get the D gain
  return dgain;}
void setP(float p) { // set the P gain and store it to eeprom
  pgain = p; 
  writeFloat(p, pgainAddress);}
void setI(float i) { // set the I gain and store it to eeprom
  igain = i; 
  writeFloat(i, igainAddress);}
void setD(float d) { // set the D gain and store it to eeprom
  dgain = d; 
  writeFloat(d, dgainAddress);}

float updatePID(float targetTemp, float curTemp) {
  // these local variables can be factored out if memory is an issue, 
  // but they make it more readable
  double result;
  float error;
  float windupGuard;

  // determine how badly we are doing
  error = targetTemp - curTemp;

  // the pTerm is the view from now, the pgain judges 
  // how much we care about error we are this instant.
  pTerm = pgain * error;

  // iState keeps changing over time; it's 
  // overall "performance" over time, or accumulated error
  iState += error;

  // to prevent the iTerm getting huge despite lots of 
  //  error, we use a "windup guard" 
  // (this happens when the machine is first turned on and
  // it cant help be cold despite its best efforts)

  // not necessary, but this makes windup guard values 
  // relative to the current iGain
  windupGuard = WINDUP_GUARD_GAIN / igain;  

  if (iState > windupGuard) { 
	iState = windupGuard; }
  else if (iState < -windupGuard) { 
	iState = -windupGuard; }
  iTerm = igain * iState;
 
  // the dTerm, the difference between the temperature now
  //  and our last reading, indicated the "speed," 
  // how quickly the temp is changing. (aka. Differential)
  dTerm = (dgain * (curTemp - lastTemp));

  // now that we've use lastTemp, put the current temp in
  // our pocket until for the next round
  lastTemp = curTemp;

  // the magic feedback bit
  return  pTerm + iTerm - dTerm;
}
void printPIDDebugString() {    // A  helper function to keep track of the PID algorithm 
  Serial.print("PID formula (P + I - D): ");
  printFloat(pTerm, 2);
  Serial.print(" + ");
  printFloat(iTerm, 2);
  Serial.print(" - ");
  printFloat(dTerm, 2);
  Serial.print(" POWER: ");
  printFloat(getHeatCycles(), 0);
  Serial.print(" ");
}
// END declare PID

// Start Heater Control
#define HEAT_RELAY_PIN 6    // Heater relay pin assignment
float heatcycles;           // the number of millis out of HEAT_RELAY_PERIOD for the current heat amount (percent * 10)
boolean heaterState = 0;    // Heater declaration and initialization
unsigned long heatCurrentTime, heatLastTime; // millis as timing references

void setupHeater() {
  pinMode(HEAT_RELAY_PIN , OUTPUT);}
void updateHeater() {
  heatCurrentTime = millis();
  if(heatCurrentTime - heatLastTime >= HEAT_RELAY_PERIOD or heatLastTime > heatCurrentTime) { //second statement prevents overflow errors
 // begin cycle
    _turnHeatElementOnOff(1);
    heatLastTime = heatCurrentTime;   
  } 
  if (heatCurrentTime - heatLastTime >= heatcycles) {
    _turnHeatElementOnOff(0);
  }
}
void setHeatPowerPercentage(float power) {
  if (power <= 0.0) {
    power = 0.0;
  }	
  if (power >= WINDUP_GUARD_GAIN) {
    power = WINDUP_GUARD_GAIN;
  }
  heatcycles = power;
}
float getHeatCycles() {
  return heatcycles;
}
void _turnHeatElementOnOff(boolean on) {
  digitalWrite(HEAT_RELAY_PIN, on);	//turn pin high
  heaterState = on;
}
// End Heater Control

// --- Simple extension to the EEPROM library ---
#include <avr/EEPROM.h>
float readFloat(int address) {
  float out;
  eeprom_read_block((void *) &out, (unsigned char *) address ,4 );
  return out;
}
void writeFloat(float value, int address) {
  eeprom_write_block((void *) &value, (unsigned char *) address ,4);
}
// --- END EEPROM Float ---

// --- Serial Interface ---
#define AUTO_PRINT_INTERVAL 200  // milliseconds
#define MAX_DELTA  100
#define MIN_DELTA  0.01
#define PRINT_PLACES_AFTER_DECIMAL 2  // set to match MIN_DELTA

int incomingByte = 0;
float delta = 1.0;
boolean autoupdate;
boolean printmode = 0;
unsigned long lastUpdateTime = 0;

void setupSerialInterface()  {
  Serial.begin(9600);
  Serial.println("\nWelcome to the Fredjke's !Craft Brewery for Arduino");
//  Serial.println("Send back one or more characters to setup the controller.");
//  Serial.println("If this is your initial run, please enter 'R' to Reset the EEPROM.");
  Serial.println("Enter '?' for help.  Here's to a great brew!");
}
void printHelp() {
  Serial.println("Send these characters for control:");
  Serial.println("<space> : print status now");
//  Serial.println("u : toggle periodic status update");
//  Serial.println("g : toggle update style between human and graphing mode");
//  Serial.println("R : reset/initialize PID gain values");
//  Serial.println("b : print PID debug values");
//  Serial.println("A : Toggle AutoBrewSwitch");
//  Serial.println("M : Toggle ManualBrewSwitch");
//  Serial.println("? : print help");  
//  Serial.println("+/- : adjust delta by a factor of ten");
//  Serial.println("P/p : up/down adjust p gain by delta");
//  Serial.println("I/i : up/down adjust i gain by delta");
//  Serial.println("D/d : up/down adjust d gain by delta");
//  Serial.println("T/t : up/down adjust set temp by delta"); 
}
void updateSerialInterface() {
  while(Serial.available()){

    incomingByte = Serial.read();
    if (incomingByte == 'R') {
      setP(30.0); // make sure to keep the decimal point on these values
      setI(0.0);  // make sure to keep the decimal point on these values
      setD(0.0);  // make sure to keep the decimal point on these values
      setTargetTemp(20.0); // here too
      Serial.print("Reset Done: ");Serial.println();
    } 
    if (incomingByte == 'P') {
      setP(getP() + delta);
      Serial.print("Set P gain: ");Serial.print(" ");Serial.print(getP());Serial.println();
    } 
    if (incomingByte == 'p') {
      setP(getP() - delta);
      Serial.print("Set P gain: ");Serial.print(" ");Serial.print(getP());Serial.println();
    } 
    if (incomingByte == 'I') {
      setI(getI() + delta);
      Serial.print("Set I gain: ");Serial.print(" ");Serial.print(getI());Serial.println();
    } 
    if (incomingByte == 'i') {
      setI(getI() - delta);
      Serial.print("Set I gain: ");Serial.print(" ");Serial.print(getI());Serial.println();
    } 
    if (incomingByte == 'D') {
      setD(getD() + delta);
      Serial.print("Set D gain: ");Serial.print(" ");Serial.print(getD());Serial.println();
    } 
    if (incomingByte == 'd' ){
      setD(getD() - delta);
      Serial.print("Set D gain: ");Serial.print(" ");Serial.print(getD());Serial.println();
    } 
    if (incomingByte == 'T') {
      setTargetTemp(getTargetTemp() + delta);
      Serial.print("Set target temperature: ");Serial.print(" ");Serial.print(getTargetTemp());Serial.println();
    } 
    if (incomingByte == 't') {
      setTargetTemp(getTargetTemp() - delta);
      Serial.print("Set target temperature: ");Serial.print(" ");Serial.print(getTargetTemp());Serial.println();
    }
    if (incomingByte == '+') {
      delta *= 10.0;
      if (delta > MAX_DELTA) delta = MAX_DELTA;
      Serial.print("Set delta: ");Serial.print(" ");Serial.print(delta);Serial.println();
    } 
    if (incomingByte == '-') {
      delta /= 10.0;
      if (delta < MIN_DELTA) delta = MIN_DELTA;
      Serial.print("Set delta: ");Serial.print(" ");Serial.print(delta);Serial.println();
    }
    if (incomingByte == 'u') {
      // toggle updating
      autoupdate = not autoupdate;
      Serial.print("Autoupdate: ");Serial.print(" ");Serial.print(autoupdate);Serial.println();
    }
    if (incomingByte == 'g') {
      // toggle updating
      printmode = not printmode;
      Serial.print("Print Mode: ");Serial.print(" ");Serial.print(printmode);Serial.println();
    }
    if (incomingByte == ' ') {
      // toggle updating
      printStatus();
    }
    if (incomingByte == 'M') {
      // toggle ManualBrewSwitch
      ManualBrewSwitch = not ManualBrewSwitch;
      if (ManualBrewSwitch){ // stop AutoBrew
         AutoBrewSwitch = 0;
         iBrewTime = 0;
      } else {// reset values
         setTargetTemp(20.0);
      }
      Serial.print("Manual Brew Switch: ");Serial.print(" ");Serial.print(ManualBrewSwitch);Serial.println();
    }
    if (incomingByte == 'A') {
      // toggle AutoBrewSwitch
      AutoBrewSwitch = not AutoBrewSwitch;
      if (AutoBrewSwitch){ // stop AutoBrew
        ManualBrewSwitch = 0;
      } else {// reset values
        iBrewTime = 0;
        setTargetTemp(20.0);
      }
      Serial.print("Auto Brew Switch: ");Serial.print(" ");Serial.print(AutoBrewSwitch);Serial.println();
    }
    if (incomingByte == '?') {
      printHelp(); 
    }
    if (incomingByte == 'b') {
      printPIDDebugString(); 
      Serial.println();
    }
  }

  if (millis() < lastUpdateTime) {
    lastUpdateTime = 0;
  }
  if ((millis() - lastUpdateTime) > AUTO_PRINT_INTERVAL) {
   // this is triggers every slightly more than a second from the delay between these two millis() calls
    lastUpdateTime += AUTO_PRINT_INTERVAL;
    if (autoupdate) {
      if (printmode) {
        printStatusForGraph();
      }
      else {
        printStatus();
      }
    } 
  }
}
void printStatus() {  // A means for getting feedback on the current system status and controllable parameters
	Serial.print(" SET TEMP:");
	printFloat(getTargetTemp(),PRINT_PLACES_AFTER_DECIMAL);
	Serial.print(", CUR TEMP:");
	printFloat(getLastTemp(),PRINT_PLACES_AFTER_DECIMAL);
	Serial.print(", GAINS p:");
	printFloat(getP(),PRINT_PLACES_AFTER_DECIMAL);
        Serial.print(" i:");
	printFloat(getI(),PRINT_PLACES_AFTER_DECIMAL);
	Serial.print(" d:");
	printFloat(getD(),PRINT_PLACES_AFTER_DECIMAL);
	Serial.print(", Delta: ");
	printFloat(delta,PRINT_PLACES_AFTER_DECIMAL);
	Serial.print(", Power: ");
	printFloat((float)getHeatCycles(), 0);
	Serial.print(" \n");	
}
void printStatusForGraph() {
  printFloat(getTargetTemp(),PRINT_PLACES_AFTER_DECIMAL);
  Serial.print(", ");
  printFloat(getLastTemp(),PRINT_PLACES_AFTER_DECIMAL);
  Serial.print(", ");
  printFloat(getP(),PRINT_PLACES_AFTER_DECIMAL);
  Serial.print(", ");
  printFloat(getI(),PRINT_PLACES_AFTER_DECIMAL);
  Serial.print(", ");
  printFloat(getD(),PRINT_PLACES_AFTER_DECIMAL);
  Serial.print(", ");
  printFloat((float)getHeatCycles(), 0);
  Serial.println();
}
void printFloat(float value, int places) { // printFloat prints out the float 'value' rounded to 'places' places after the decimal point
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;

  // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
    d/= 10.0;    
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }


  // write out the negative if needed
  if (value < 0)
    Serial.print('-');

  if (tenscount == 0)
    Serial.print(0, DEC);

  for (i=0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    Serial.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return
  if (places <= 0)
    return;

  // otherwise, write the point and continue on
  Serial.print('.');  

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    Serial.print(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }
}
// --- End of Serial Interface ---

// --- Main Setup ---
void setup() {
	byte i;
	setupSerialInterface();						// Use of Complete Serial Interface
	Serial.println(" *** Begin Setup Procedure *** ");
//	Serial.print(" *** Setup LCD : ");
	SetupLCD(); 
//	Serial.println("Done");
//	Serial.print(" *** Setup Push button and Buzzer : ");
 	SetupPush(); 							// initialize Push button
 	SetupBuzzer(); 							// initialize Buzzer
//	Serial.println("Done");
//	Serial.print(" *** Setup PID : ");
	setupPID(PGAIN_ADR, IGAIN_ADR, DGAIN_ADR ); 	                // Send addresses to the PID module 
//	Serial.println("Done");
//	Serial.print(" *** Setup PID Timer : ");
	lastPIDTime = millis();						// Initialize PID time
//	Serial.println("Done");
//	Serial.print(" *** Setup Temperature consign : ");
	TargetTemp = readFloat(TEMP_ADR);                               // from EEPROM. load the saved value
//	Serial.println("Done");
//	Serial.print(" *** Setup Temperature sensors : ");
	GetTemp(); 							// Initialize Temperature Sensors
//	Serial.println("Done");
//	Serial.print(" *** Setup Heater : ");
        setupHeater();
//	Serial.println("Done");
	Serial.println(" *** End of Setup Procedure *** ");
} // --- End of Main Setup ---

void ShowClock(long Pclock, byte Mode) { // --- Show Clock + Saved Temperature consigns ---
	lcd.home();
	lcd.setCursor(0, 0);
	Pclock -= int(iBrewTime/1000); // display clock from start of device if not in autobrew, clock from start of autobrew if in autobrewmode
	if ( Pclock / 3600 < 10 ) {lcd.print(Pclock / 3600);} else { Pclock -= 36000; lcd.print(Pclock / 3600);} 
	lcd.print(":");
	if ( (Pclock % 3600) / 60 < 10 ) {lcd.print("0"); lcd.print((Pclock % 3600) / 60 );} else { lcd.print((Pclock % 3600) / 60 );}
	lcd.print(":");
	if ( ((Pclock % 3600) % 60) < 10 ) {lcd.print("0"); lcd.print((Pclock % 3600) % 60);} else { lcd.print((Pclock % 3600) % 60);}
    if ( AutoBrewSwitch ) { lcd.print(" Step "); lcd.print(BrewStep);}
	else { lcd.print(" s. start");} // clear rest of line      

	switch (Mode){
		case 1 : // Normal Mode
                     lcd.setCursor(0, 1);
                     lcd.print("Set T"); lcd.printByte(Degree); lcd.print(": "); lcd.print(getTargetTemp());lcd.print(" "); lcd.printByte(Degree); lcd.print("C");
                     break;
		case 2 : // Hops Addition during Boil
		     lcd.setCursor(0, 1);
		     lcd.print("Add hops now !!!");
                     break;
		case 3 : // Ready for Filtering
		     lcd.setCursor(0, 1);
		     lcd.print("Filter now!");
                     break;
		default : // Undefined
		     lcd.setCursor(0, 1);
		     lcd.print("Mode undefined. ");
	} // End of Mode Switch
return;
}
void ShowTemp() {   // Display Temperature on LCD 
  byte i;
  lcd.home();
  for  ( i = 0; i < Max_Sensors; i++) {
    lcd.setCursor(0, i);
    if (i == Brew) { lcd.print("Brew  "); lcd.print(": "); lcd.print(Celsius[i]); lcd.print(" "); lcd.printByte(Degree); lcd.print("C");}
    else if (i == Sparge ) { lcd.print("Sparge"); lcd.print(": "); lcd.print(Celsius[i]); lcd.print(" "); lcd.printByte(Degree); lcd.print("C");}
    else {lcd.print("T"); lcd.print((i+1)); lcd.print(": "); lcd.print(Celsius[i]); lcd.print(" "); lcd.printByte(Degree); lcd.print("C");}
  }
  return;
}

void HopAddition(byte hop) { // Buzz and Display message for Hop Addition
 	 ShowClockMode = 2;
 	 lastBUZTime = millis();
}

void GetTemp() { // --- Get Temperatures on all sensors ---
  byte i = 0, j = 0;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  
  for (j = 0; j <= Max_Sensors ; j++) {

    if ( !ds.search(addr)) { // No more Adresses
//		Serial.println("No more addresses.");
//		Serial.println();
		ds.reset_search();
		delay(250);
		return;
    }
    
//    Serial.print("ROM "); Serial.print(j); Serial.print(" = ");
    for( i = 0; i < 8; i++) { // Print Sensor Address
//      Serial.write(' ');
//      Serial.print(addr[i], HEX);
    }
//    Serial.println();
  
    if (OneWire::crc8(addr, 7) != addr[7]) { // Check CRC of Sensor Address
//        Serial.println("CRC is not valid!");
        return;
    } 
   
    // the first ROM byte indicates which chip
    // Only for Serial debugging
    switch (addr[0]) { // Check type of Sensor
      case 0x10:
//        Serial.println("  Chip = DS18S20");  // or old DS1820
        type_s = 1;
        break;
      case 0x28:
//        Serial.println("  Chip = DS18B20");
        type_s = 0;
        break;
      case 0x22:
//        Serial.println("  Chip = DS1822");
        type_s = 0;
        break;
      default:
//        Serial.println("Device is not a DS18x20 family device.");
        return;
    } // END Switch

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
  
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad

//    Serial.print("  Data = ");
//    Serial.print(present, HEX);
//    Serial.print(" ");
    for ( i = 0; i < 9; i++) {           // Read 9 bytes of Data
      data[i] = ds.read();
//      Serial.print(data[i], HEX);
//      Serial.print(" ");
    }
//    Serial.print(" CRC=");
//    Serial.print(OneWire::crc8(data, 8), HEX);
//    Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
	
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
 // at lower res, the low bits are undefined, so let's zero them
 if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
 else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
 // default is 12 bit resolution, 750 ms conversion time
  }
  Celsius[j] = (float)raw / 16.0;
//  Serial.print("  Temperature "); Serial.print(j); Serial.print(" = ");
//  Serial.print(Celsius[j]);
//  Serial.println(" Celsius");
  } // End of For Loop inside Get Temp to get all sensors
} // End of Get Temp Function

void loop() {	// --- Main Loop ---
    long i;
    long cur_ms = millis();

    updateSerialInterface();

    lcd.clear(); 
    Toggle = not Toggle; // Switch display every LCD_UPDATE_INTERVAL
  	if (Toggle) {ShowTemp();}   // Display Temperatures
	else {ShowClock((cur_ms/1000), ShowClockMode);}     // Display Clock/Consigns

        // --- Temperature Sensors ---	
        GetTemp();

	// Handle Manual Brew section activated by the Switch
	if (ManualBrewSwitch or AutoBrewSwitch){
           // --- PID ---
	   // every second, udpate the current heat control, and print out current status
	   if (cur_ms < lastPIDTime) {   // This checks for rollover with millis()
           lastPIDTime = 0; }
	   if ((cur_ms - lastPIDTime) > PID_UPDATE_INTERVAL) {
              lastPIDTime += PID_UPDATE_INTERVAL;
	      // --- Heater ---
              heatPower = updatePID(getTargetTemp(), Celsius[Brew]);
              setHeatPowerPercentage(heatPower); }  
	   updateHeater();
	} // --- End of ManualBrewSwitch If Section ---

	// Handle Buzzer
	if (digitalRead(pusPin)) { // Switch Off Buzzer if Push Button is pressed 
		lastBUZTime = 0; 
		Buzz = 0;
		ShowClockMode = 1;
	} 	
	if (cur_ms < lastBUZTime + 10000 ) { // Start Buzzer for 10 seconds if HopAddition 
		Buzz = 1;
	}
	else {	// Switch Off Buzzer
		Buzz = 0;
		ShowClockMode = 1;
	}
	updateBUZ(Buzz);
	
	// Handle Automatic Brew section activated by the Switch
	if (AutoBrewSwitch){
		if (iBrewTime == 0){ iBrewTime = cur_ms; BrewStep = 1; // Store initial Brew time millis() value
				Serial.println("Auto Brew Process started: ");}
		switch (BrewStep) {
			case 1: // Proteinic Rest : Start Temperature increase
				if (getTargetTemp() != RestOneT) {setTargetTemp(RestOneT);}
				if (Celsius[0] >= getTargetTemp()) {StepTime = cur_ms; lastBUZTime = cur_ms; BrewStep = 2;
					Serial.println(" - Step 1 : Done.");}
				break;
			case 2: // Proteinic Rest : Start Duration Check
				if (cur_ms > StepTime + 60000 * RestOneD) {lastBUZTime = cur_ms; BrewStep = 3;
					Serial.println(" - Step 2 : Done.");}
				break;
			case 3: // Saccharification Rest Beta-Amylase : Start Temperature increase
				if (getTargetTemp() != RestTwoT) {setTargetTemp(RestTwoT);}
				if (Celsius[0] >= getTargetTemp()) {StepTime = cur_ms; lastBUZTime = cur_ms; BrewStep = 4;
					Serial.println(" - Step 3 : Done.");}
				break;
 			case 4: // Saccharification Rest Beta-Amylase : Start Duration Check
 				if (cur_ms > StepTime + 60000 * RestTwoD) {lastBUZTime = cur_ms; BrewStep = 5;
					Serial.println(" - Step 4 : Done.");}
				break;
 			case 5: // Saccharification Rest Alpha-Amylase : Start Temperature increase
 				if (getTargetTemp() != RestThreeT) {setTargetTemp(RestThreeT);}
 				if (Celsius[0] >= getTargetTemp()) {StepTime = cur_ms; lastBUZTime = cur_ms; BrewStep = 6;
					Serial.println(" - Step 5 : Done.");}
				break;
 			case 6: // Saccharification Rest Alpha-Amylase : Start Duration Check
 				if (cur_ms > StepTime + 60000 * RestThreeD) {lastBUZTime = cur_ms; BrewStep = 7;
					Serial.println(" - Step 6 : Done.");}
				break;
 			case 7: // Mash-Out Rest : Start Temperature increase
 				if (getTargetTemp() != RestFourT) {setTargetTemp(RestFourT);}
 				if (Celsius[0] >= getTargetTemp()) {StepTime = cur_ms; lastBUZTime = cur_ms; BrewStep = 8;
					Serial.println(" - Step 7 : Done.");}
				break;
 			case 8: // Mash-Out Rest : Start Duration Check
 				if (cur_ms > StepTime + 60000 * RestFourD) {lastBUZTime = cur_ms; BrewStep = 9;
					Serial.println(" - Step 8 : Done.");}
				break;
 			case 9: // Here comes filtration step, manual intervention needed.
 				lastBUZTime = cur_ms;
 				setTargetTemp(0); // Deactivate heating while filtering
 				ShowClockMode = 3; // Display Message during Filtering process
 				if (digitalRead(pusPin)) {BoilStep -= 1;}
 				else {BoilStep = BoilStepI;}
 				if (BoilStep == 0){BrewStep = 10; // Step over to Step 10 only after BoilStepI iterations
					Serial.println(" - Step 9 : Done.");}
				break;
 			case 10: // Boil : Start Temperature increase
 				if (getTargetTemp() != 99) {setTargetTemp(99);}
 				if (Celsius[0] >= getTargetTemp()) {StepTime = cur_ms; lastBUZTime = cur_ms; BrewStep = 11;
					Serial.println(" - Step 10 : Done.");}
				break;
 			case 11: // Boil : Start Duration Check
 				if (HopOne){
 					if (cur_ms > StepTime + 60000 * BoilD - 60000 * HopOneD) {HopAddition(1); HopOne = 0;}
 				}
 				if (HopTwo){
 					if (cur_ms > StepTime + 60000 * BoilD - 60000 * HopTwoD) {HopAddition(2); HopTwo = 0;}
 				}
 				if (HopThree){
 					if (cur_ms > StepTime + 60000 * BoilD - 60000 * HopThreeD) {HopAddition(3); HopThree = 0;}
 				}
 				if (HopFour){
 					if (cur_ms > StepTime + 60000 * BoilD - 60000 * HopFourD) {HopAddition(4); HopFour = 0;}
 				}
 				if (HopFive){
 					if (cur_ms > StepTime + 60000 * BoilD - 60000 * HopFiveD) {HopAddition(5); HopFive = 0;}
 				}
 				if (cur_ms > StepTime + 60000 * BoilD) {lastBUZTime = cur_ms; BrewStep = 12;
					Serial.println(" - Step 11 : Done.");}
				break;
 			case 12: // Finalize
 				setTargetTemp(0);
				Serial.println(" - Step 12 : Done.");
                  		Serial.println("Auto Brew Process finished.");
				break;
		} // --- End of Switch BrewStep section ---
	} // --- End of AutoBrewSwitch If Section ---

        // Force cycle duration to last for at least LCD_UPDATE_INTERVAL
        if (LCD_UPDATE_INTERVAL > (millis()-cur_ms)) { delay(LCD_UPDATE_INTERVAL - (millis()-cur_ms));}
	
} // --- End of Main Loop ---



