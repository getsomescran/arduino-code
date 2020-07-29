

int dhtpin= D3
int sensor_pin = A5; // Soil Sensor input at Analog PIN A0
int output_value ;
const int echopin = 12;
const int trigpin = 13;
int SensorValue =0;
long timespent;
int distance;
int sensorPin = A0
;int sensorValue = 0;
#include <BH1750FVI.h>
 #include <Wire.h> 
 #include <ds3231.h>
#define sensorPower 
#define sensorPin A12
 #define LedPin 5 
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 5

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;
void setup() 
 {
 pinMode(trigpin, OUTPUT);
 pinMode(echopin, INPUT);
 serial.begin(9600);
 
}






{
  
Serial.begin(9600);On .
  LightSensor.begin();
 
  LightSensor.SetAddress(Device_Address_H); 
  LightSensor.SetMode(Continuous_H_resolution_Mode);
  pinMode(9,OUTPUT) 
}
{
   Serial.begin(9600);
  Wire.begin();
  DS3231_init(DS3231_INTCN);
  /*----------------------------------------------------------------------------
  In order to synchronise your clock module, insert timetable values below !
  ----------------------------------------------------------------------------*/
  t.hour=12; 
  t.min=30;
  t.sec=0;
  t.mday=2;
  t.mon=3;
  t.year=2020;
 
  DS3231_set(t); 
}
 
}

{ pinMode(4,OUTPUT);
     Serial.begin(9600);
   Serial.println("Reading From the Sensor ...");
   delay(2000);}
void loop() 

{
  digitalWrite(trigpin, LOW);
  delay(2000);

  digitalWrite(trigPin, HIGH);
  delay(2000)
  timespent = pulseIn(echoPin, HIGH);
  distance= timespent*0.034/2;

  Serial.print("distance: ");
  serial.println (distance);

if distance >== 6.5 
digitalWrite(4, LOW);
}

{

  
  DS3231_get(&t);
  Serial.print("Date : ");
  Serial.print(t.mday);
  Serial.print("/");
  Serial.print(t.mon);
  Serial.print("/");
  Serial.print(t.year);
  Serial.print("\t Hour : ");
  Serial.print(t.hour);
  Serial.print(":");
  Serial.print(t.min);
  Serial.print(".");
  Serial.println(t.sec);
 
  delay(1000);
}
  Light_Intensity=LightSensor.GetLightIntensity();
  delay(2000);
 
  SensorValue=map(Light_Intensity,0,2000,255,0);
  SensorValue=constrain(SensorValue,255,0);
  digitalWrite(LedPin,SensorValue);
  
}
{sensorValue = analogRead(sensorPin); // read the value from the sensor
Serial.println(sensorValue); //prints the values coming from the sensor on the screen

delay(100);



if (sensorValue) <=(1200)
digitalWrite( 5, HIGH)
}
{ // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"))}
    {
   output_value= analogRead(sensor_pin);
 output_value = map(output_value,550,10,0,100);
   Serial.print("Mositure : ");
   Serial.print(output_value);
   Serial.println("%");
   if(output_value<0){
      digitalWrite(4,HIGH);
     }
     else{
            digitalWrite(4,LOW);
     }
   delay(1000);
}

{ 
   float Celcius=0;
  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
  Serial.print(" C  ");
  Serial.print(Celcius);
  delay(1000);
}
