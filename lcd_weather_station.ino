/* Video tutorial link: https://youtu.be/4XEe0HY0j6k
 * Video tutorial link: https://youtu.be/yPaODwsLpf4
 */

// Code to use SoftwareSerial
#include <SoftwareSerial.h> //The SoftwareSerial library has been developed to allow serial communication on other digital pins of the Arduino.
SoftwareSerial espSerial =  SoftwareSerial(2,3);      // arduino RX pin=2  arduino TX pin=3    connect the arduino RX pin to esp8266 module TX pin   -  connect the arduino TX pin to esp8266 module RX pin

String apiKey = "TMU3ECVYHTG6BA0V";     // replace with your channel's thingspeak WRITE API key

String ssid="balasundar-EXT";    //my  Wifi network SSID
String password ="";  //my  Wifi network password

boolean DEBUG=true;

#include <Wire.h>
#include <SPI.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define anInput     A0                        //analog feed from MQ135
#define digTrigger   2                        //digital feed from MQ135
#define co2Zero     87                       //calibrated CO2 0 level
#define AOUTpin     A3                           //ANALOG FEED FOR CO SENSOR

int LDR = 1;     //analog pin to which LDR is connected, here we set it to 1 so it means A1
int LDRValue = 0;      //that’s a variable to store LDR values
int light_sensitivity = 500;    //This is the approx value of light surrounding your LDR
double rldr=0;
float Vout;
double lux;

//const int AOUTpin=3;//the AOUT pin of the CO sensor goes into analog pin A0 of the arduino
const int DOUTpin=8;//the DOUT pin of the CO sensor goes into digital pin D8 of the arduino
//const int ledPin=13;//the anode of the LED connects to digital pin D13 of the arduino


#include <LiquidCrystal.h>
int contrast=15;
// initialize the library by providing the nuber of pins to it
LiquidCrystal lcd(12 ,11 ,5 , 4, 10, 8);


float limit;
float value;


const int sampleWindow = 50;                              // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);



//======================================================================== showResponce
void showResponse(int waitTime){
    long t=millis();                           //Returns the number of milliseconds since the Arduino board began running the current program.
    char c;
    while (t+waitTime>millis()){
      if (espSerial.available()){                           //if serial communication between arduino and esp8266 is established
        c=espSerial.read();
        if (DEBUG) Serial.print(c);                        //write esp8266 communication onto the serial monitor
      }
    }
                   
}

//========================================================================
boolean thingSpeakWrite(float value1, float value2, float value3, float value4){
  String cmd = "AT+CIPSTART=\"TCP\",\"";                  // TCP connection
  cmd += "184.106.153.149";                               // api.thingspeak.com
  cmd += "\",80";
  espSerial.println(cmd);
  if (DEBUG) Serial.println(cmd);                         //sending command to the serial monitor
  if(espSerial.find("Error")){                            //prints error if connection has failed
    if (DEBUG) Serial.println("AT+CIPSTART error");
    return false;
  }
  
  
  String getStr = "GET /update?api_key=";   // prepare GET string in the particular format
  getStr += apiKey;
  
  getStr +="&field1=";
  getStr += String(value1);                  //value1 is the data that is to be sent to thinkspeak
 getStr +="&field2=";
   getStr += String(value2);
   getStr +="&field3=";
   getStr += String(value3);
   getStr +="&field4=";
   getStr += String(value4);
   //getStr +="&field5=";
   //getStr += String(value5);
   //getStr +="&field6=";
   //getStr += String(value6);
   //getStr +="&field7=";
   //getStr += String(value7);
  // ...
  getStr += "\r\n\r\n";

  // send data length
  cmd = "AT+CIPSEND=";
  cmd += String(getStr.length());
  espSerial.println(cmd);
  if (DEBUG)  Serial.println(cmd);
  
  delay(100);
  if(espSerial.find(">")){       
  //Get the data from the WiFi module and send it to the debug serial port.
  
    espSerial.print(getStr);
    if (DEBUG)  Serial.print(getStr);
  }
  else{
    espSerial.println("AT+CIPCLOSE");
    // alert user
    if (DEBUG)   Serial.println("AT+CIPCLOSE");
    return false;
  }
  return true;
}
//================================================================================ setup
void setup() {                
  DEBUG=true;           // enable debug serial
  Serial.begin(115200); 

  pinMode(13, OUTPUT);
  pinMode(anInput,INPUT);                     //MQ135 analog feed set for input
  pinMode(digTrigger,INPUT);                  //MQ135 digital feed set for input
  //pinMode(led,OUTPUT);                        //led set for output

pinMode(AOUTpin,INPUT);//input for co

 analogWrite(6,contrast);
lcd.begin(16, 2);


  espSerial.begin(115200);  // enable software serial
                          // Your esp8266 module's speed is probably at 115200. 
                          // For this reason the first time set the speed to 115200 or to your esp8266 configured speed 
                          // and upload. Then change to 9600 and upload again
  
  //espSerial.println("AT+RST");         // Enable this line to reset the module;
  //showResponse(1000);

  espSerial.println("AT+CWMODE=1");   // set esp8266 as client
  showResponse(1000);

  espSerial.println("AT+CWJAP=\""+ssid+"\",\""+password+"\"");  // set your home router SSID and password
  showResponse(5000);

  Serial.println(F("BME280 test"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
 
  


 if (DEBUG)  Serial.println("Setup completed");
}


// ====================================================================== loop
void loop() {

  // Read sensor values

Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
    int co2now[10];                               //int array for co2 readings
    int co2raw = 0;                               //int for raw value of co2  
    int co2comp = 0;                              //int for compensated co2 
    int co2ppm = 0;                               //int for calculated ppm
    int zzz = 0;                                  //int for averaging
    int grafX = 0;                                //int for x value of graph

//BME280
    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();


    //CO2 
    for (int x = 0;x<10;x++){                   //samplpe co2 10x over 2 seconds
    co2now[x]=analogRead(A0);
    delay(200);
  }

for (int x = 0;x<10;x++){                     //add samples together
    zzz=zzz + co2now[x];
    
  }
  co2raw = zzz/10;                            //divide samples by 10
  co2comp = co2raw - co2Zero;                 //get compensated value
  Serial.println(co2raw);
  co2ppm = map(co2comp,0,1023,385,5000);      //map value for atmospheric levels

  
 Serial.print(co2ppm);                      //print co2 ppm
  Serial.println(" PPM");                      //print units

//LDR 
LDRValue = analogRead(LDR); 
    Vout=LDRValue*(0.0048828125);
    rldr=((5-Vout))/Vout;
    lux=10000/pow(rldr*10,(4/3));
    Serial.println("The lux reading is:");
    Serial.println(lux);       //prints the LDR values to serial monitor
   


//CARBON MONOXIDE
value= analogRead(A3);//reads the analaog value from the CO sensor's AOUT pin
limit= digitalRead(DOUTpin);//reads the digital value from the CO sensor's DOUT pin
//Serial.print("op value: ");
//Serial.println(value);//prints the CO value
limit=2.402*value/500;
//limit=map(value,0,12489,0,100);
Serial.print("CO % vol ");
Serial.println(limit);//prints the limit reached as either LOW or HIGH (above or underneath)


//decibel
 unsigned long startMillis= millis();                   // Start of sample window
   float peakToPeak = 0;                                  // peak-to-peak level
 
   unsigned int signalMax = 0;                            //minimum value
   unsigned int signalMin = 1024;                         //maximum value
 
                                                          // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(2);                             //get reading from microphone
      if (sample < 1024)                                  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;                           // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;                           // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;                    // max - min = peak-peak amplitude
   float db = map(peakToPeak,4,690,10,63);             //calibrate for deciBels
   Serial.print("The db reading is:");
   Serial.println(db);
   //Serial.println(peakToPeak);
   //delay(1000);

   thingSpeakWrite(bme.readTemperature(),bme.readPressure() / 100.0, bme.readHumidity(), co2ppm);                                      // Write values to be sent to thingspeak
        delay(20000);  // thingspeak needs 15 sec delay between updates, 

lcd.print(" Temperature = ");
  //delay(3000);
lcd.setCursor(0,1);
  lcd.print(bme.readTemperature());
  delay(3000);
  lcd.clear();
  lcd.print(" Pressure = ");
  //delay(3000);
lcd.setCursor(0,1);
  lcd.print(bme.readPressure() / 100.0F + 100);
   lcd.print("hPa");
  delay(3000);
  lcd.clear();
   lcd.print(" Humidity =  ");
  //delay(3000);
lcd.setCursor(0,1);
  lcd.print(bme.readHumidity());
  lcd.print("% ");
  delay(3000);
  lcd.clear();
   lcd.print(" CO2 ppm =  ");
  //delay(3000);
lcd.setCursor(0,1);
  lcd.print(co2ppm);
      lcd.print("ppm ");
  delay(3000);
  lcd.clear();
   lcd.print(" CO % vol  ");
  //delay(3000);
lcd.setCursor(0,1);
  lcd.print(limit);
    lcd.print("% ");
  delay(3000);
  lcd.clear();
   lcd.print(" sound ");
  //delay(3000);
lcd.setCursor(0,1);
  lcd.print(db);
    lcd.print("db ");
  delay(3000);
  lcd.clear();
   lcd.print(" light intensity ");
  //delay(3000);
lcd.setCursor(0,1);
  lcd.print(lux);
    lcd.print("lx ");
 delay(3000);
  lcd.clear();

     
          
}
  
    
      



 
