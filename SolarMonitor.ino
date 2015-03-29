#include <DHT.h>
//#include <Narcoleptic.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

// A solar monitor. This records solar output from a panel using Hall effect sensors
// on the input and load at the controller as well as battery and panel voltage.
// Temperature and humidity are also logged

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  60000 // mills between entries (reduce to take more/faster data)

// Next log millis:
uint32_t next_log_time;

// Number of hours (in milliseconds) the LED lamps should light for at night.
#define NUMBER_LED_LAMP_HOURS 3 
long lamp_off_time = 0;

bool solar_panel_off = false;

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   0 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the signal LEDs
#define redLEDpin 2
#define greenLEDpin 3

#define DHTPIN 4     // what pin we're connected to

#define PUSHBUTTON_PIN 7  // A push button to control the LED lamps
#define LED_CONTROL_PIN 6  // Used to control the LED lamps

#define LOAD_OUT_CURRENT_PIN A3
#define PANEL_VOLTAGE_PIN    A2
#define BATTERY_VOLTAGE_PIN  A1
#define PANEL_IN_CURRENT_PIN A0


#define VOLTAGE_DIVIDER_FACTOR  6.0  // Voltage divider

#define NUM_SAMPLES 10 //Take 10 samples for each analog read:

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);


RTC_DS1307 RTC; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;



int panel_off_count = 0;
const double PANEL_VOLTAGE_THRESHOLD=0.40;
const int PANEL_COUNT_THRESHOLD = 2; // If the panel is off for five samples then 

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  while(1);
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println();

  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);

  // Start with the voltage pins as input:
  pinMode( PANEL_VOLTAGE_PIN , INPUT );
  pinMode(  BATTERY_VOLTAGE_PIN, INPUT );
  pinMode( PANEL_IN_CURRENT_PIN , INPUT );
  pinMode( LOAD_OUT_CURRENT_PIN , INPUT );

  pinMode( LED_CONTROL_PIN, OUTPUT );
  digitalWrite( LED_CONTROL_PIN, LOW );

  pinMode( PUSHBUTTON_PIN, INPUT );
  digitalWrite( PUSHBUTTON_PIN, HIGH );

#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START



  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");

  // create a new file
  char filename[13] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    // Serial.println(filename);
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      //Serial.print("Using file: " );
      //Serial.println(filename);
      break;  // leave the loop!
    }
  }

  if (! logfile) {
    error("couldnt create file");
  } 

  Serial.print("Logging to: ");
  Serial.println(filename);
  delay(100);
  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }
  Serial.println("RTC OK");

  logfile.println("millis,stamp,datetime,temp, humidity,vpanel,vbattery,ipanel,iload");    
#if ECHO_TO_SERIAL
  Serial.println("millis,stamp,datetime,temp, humidity,vpanel,vbattery,ipanel,iload");
#endif //ECHO_TO_SERIAL

  dht.begin();

  next_log_time = millis() + LOG_INTERVAL;
  delay(5000);
}

void loop(void)
{
  DateTime now;
  float panel_voltage;  
  float battery_voltage;
  float panel_current;
  float load_current;

  if( (long)(millis()-next_log_time) >= 0 ) {

    next_log_time = millis() + LOG_INTERVAL;


    digitalWrite(greenLEDpin, HIGH);

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(t) || isnan(h)) {
      Serial.println("Failed to read from DHT");
      digitalWrite(redLEDpin, HIGH);
    } 
    else {



      // log milliseconds since starting
      uint32_t m = millis();
      logfile.print(m);           // milliseconds since start
      logfile.print(", ");    
#if ECHO_TO_SERIAL
      Serial.print(m);         // milliseconds since start
      Serial.print(", ");  
#endif

      // fetch the time
      now = RTC.now();

      // log time
      logfile.print(now.unixtime()); // seconds since 1/1/1970
      logfile.print(", ");
      logfile.print('"');
      logfile.print(now.year(), DEC);
      logfile.print("/");
      logfile.print(now.month(), DEC);
      logfile.print("/");
      logfile.print(now.day(), DEC);
      logfile.print(" ");
      logfile.print(now.hour(), DEC);
      logfile.print(":");
      logfile.print(now.minute(), DEC);
      logfile.print(":");
      logfile.print(now.second(), DEC);
      logfile.print('"');
#if ECHO_TO_SERIAL
      Serial.print(now.unixtime()); // seconds since 1/1/1970
      Serial.print(", ");
      Serial.print('"');
      Serial.print(now.year(), DEC);
      Serial.print("/");
      Serial.print(now.month(), DEC);
      Serial.print("/");
      Serial.print(now.day(), DEC);
      Serial.print(" ");
      Serial.print(now.hour(), DEC);
      Serial.print(":");
      Serial.print(now.minute(), DEC);
      Serial.print(":");
      Serial.print(now.second(), DEC);
      Serial.print('"');
#endif //ECHO_TO_SERIAL




      logfile.print(", ");    
      logfile.print(t);
      logfile.print(", ");    
      logfile.print(h);
#if ECHO_TO_SERIAL
      Serial.print(", ");   
      Serial.print(t);
      Serial.print(", ");    
      Serial.print(h);
#endif //ECHO_TO_SERIAL


      panel_voltage = getPanelVoltage( PANEL_VOLTAGE_PIN );
      logfile.print(", ");
      logfile.print(panel_voltage);
#if ECHO_TO_SERIAL
      Serial.print(", ");   
      Serial.print(panel_voltage, DEC);
#endif // ECHO_TO_SERIAL

      battery_voltage = getBatteryVoltage( BATTERY_VOLTAGE_PIN );
      logfile.print(", ");
      logfile.print(battery_voltage);
#if ECHO_TO_SERIAL
      Serial.print(", ");   
      Serial.print(battery_voltage, DEC);
#endif // ECHO_TO_SERIAL


      panel_current = getCurrent( PANEL_IN_CURRENT_PIN );
      logfile.print(", ");
      logfile.print(panel_current);
#if ECHO_TO_SERIAL
      Serial.print(", ");   
      Serial.print(panel_current, DEC);
#endif // ECHO_TO_SERIAL

      load_current = getCurrent( LOAD_OUT_CURRENT_PIN );
      logfile.print(", ");
      logfile.print(load_current);
#if ECHO_TO_SERIAL
      Serial.print(", ");   
      Serial.print(load_current, DEC);
#endif // ECHO_TO_SERIAL

      logfile.println();
#if ECHO_TO_SERIAL
      Serial.println();
#endif // ECHO_TO_SERIAL

      digitalWrite(greenLEDpin, LOW);

      // blink LED to show we are syncing data to the card & updating FAT!
      digitalWrite(redLEDpin, HIGH);
      logfile.flush();
      digitalWrite(redLEDpin, LOW);
    }

    if( panel_voltage > PANEL_VOLTAGE_THRESHOLD ) {
      if( !solar_panel_off ) {
        panel_off_count+= 1;
#if ECHO_TO_SERIAL
        Serial.print("Incrementing panel off count, value is now: ");
        Serial.print( panel_off_count, DEC );
        Serial.println();
#endif // ECHO_TO_SERIAL
      }
    } 
    else {
      panel_off_count = 0;
#if ECHO_TO_SERIAL
      Serial.print("Resetting panel off count, value is now: ");
      Serial.print( panel_off_count, DEC );
      Serial.println();
#endif // ECHO_TO_SERIAL
    }

    // Only run the LED lamps for a fixed number of hours - check the time if the 
    // solar panel is off:
    if( solar_panel_off ) {
#if ECHO_TO_SERIAL  
      Serial.print("Checking for time out, current hour = " );
      Serial.print( now.hour(), DEC );
      Serial.print(" timeout time = " );
      Serial.println( lamp_off_time, DEC );
#endif
      if( now.hour() > lamp_off_time ) {
#if ECHO_TO_SERIAL      
        Serial.println("LEDS timed out, turning off");
#endif  
        digitalWrite(LED_CONTROL_PIN, LOW ); 
      }
    }

    // The solar panel has been off for the specified number of monitoring periods
    // Turn on the LED lamps and record the time intervale to turn them off
    if( panel_off_count > PANEL_COUNT_THRESHOLD  ) {
#if ECHO_TO_SERIAL      
      Serial.print("Panel count above threshold: ");
      Serial.println(panel_off_count, DEC );
#endif  
      if( ! solar_panel_off ) {
        digitalWrite(LED_CONTROL_PIN, HIGH );
        solar_panel_off = true;
        lamp_off_time = now.hour() + NUMBER_LED_LAMP_HOURS;
        if (lamp_off_time > 24 )
          lamp_off_time = 24;
#if ECHO_TO_SERIAL
        Serial.print("Turning on LEDs, off time =  ");
        Serial.println(lamp_off_time, DEC );
#endif 
      }
    } 
    else {
      digitalWrite(LED_CONTROL_PIN, LOW ); 
      solar_panel_off = false;
      lamp_off_time = 0;
#if ECHO_TO_SERIAL
      Serial.println("Panel on again, turning off LEDs");

#endif 
    }
  }





  else {
    delay(10); 
  }
}

// Return a voltage and correct for the voltage divider
float getBatteryVoltage(int pin) {
  int vin=0;
  int i;
  for( i=0; i< NUM_SAMPLES; i++ ) {
    vin += analogRead(pin); 
    delay(10); 
  }
  vin = vin / NUM_SAMPLES;
  float voltage = (float)5.0*vin/1023.0 * VOLTAGE_DIVIDER_FACTOR;  
  return voltage;
}

// Return panel voltage, indirectly via optoisolater.
// This is not the actual panel voltage but will give some idea
// of whether the panel is active:
float getPanelVoltage(int pin) {
  int vin=0;
  int i;
  for( i=0; i< NUM_SAMPLES; i++ ) {
    vin += analogRead(pin); 
    delay(10); 
  }
  vin = vin / NUM_SAMPLES;
  float voltage = (float)5.0*vin/1023.0;
  return voltage;
}


// Calculate the current flowing from the Hall effect sensor at the attached pin:
float getCurrent( int pin ) {
  int vin=0;
  int i;
  for( i=0; i< NUM_SAMPLES; i++ ) {
    vin += analogRead(pin); 
    delay(10); 
  }
  vin = vin / NUM_SAMPLES;

  // Convert to current. First subtract the midpoint of 2.5, then divide by 0.185 to 
  // convert volts to amps: 
  float current = ((5.0*vin/1023.0)-2.5)/0.185;
  return current;
}





















