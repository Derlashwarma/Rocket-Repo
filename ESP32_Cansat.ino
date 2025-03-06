#include <Wire.h>
#include <MS5611.h> 
#include <MQ135.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SD.h>

// GPS Setup
#define RXD2 17
#define TXD2 16
#define GPS_BAUD 9600
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// Pressure Sensor (MS5611)
MS5611 MS5611(0x77);

// NTC (Thermistor Setup) Steinhart–Hart Equation
#define THERMISTOR_PIN 13
#define SERIES_RESISTOR 10000  
#define NOMINAL_RESISTANCE 10000
#define NOMINAL_TEMPERATURE 25.0
#define BETA_COEFFICIENT 3950  

// GPS Data
double latitude, longitude, altitude;
int date, day, year, hour, minute, second;

// MQ135
#define PIN_MQ135 34
MQ135 mq135_sensor(PIN_MQ135);

// SD Card module
#define SD_CS 5
File data_file;

// Transmitter/ Sender
#define HC12_BAUD 9600    // HC-12 communication speed

// Apogee Calculation Variables
bool apogee = false;
float maxAltitude = 0;
unsigned long fallStartTime = 0;
bool falling = false;

void setup() {
  // Transmitter Initialization
  // Serial2.begin(HC12_BAUD, SERIAL_8N1, 3, 1);

    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(27, OUTPUT);
    //Buzzer
    pinMode(14, OUTPUT);

  Serial.begin(115200);

  digitalWrite(27,HIGH);
  // GPS Initialization
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Waiting for GPS signal...");
  while (!gps.location.isValid()) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    Serial.print(".");
    delay(2000);
  }
  digitalWrite(25,HIGH);

  // MS5611 Initialization
  while (!Serial);
  Wire.begin();

  while(!MS5611.begin()){
    Serial.println("MS5611 initialization failed!");
    delay(2000);
  }

  // Initialize SD Card Module
  while (!SD.begin(SD_CS)){
    Serial.println("SD Card initialization failed!");
    delay(2000);
  }; 
  Serial.println("SD Card initialized successfully.");
  digitalWrite(26,HIGH);
  digitalWrite(27,LOW);
}

void loop() {
  checkApogee();

  // Read MS5611 component
  MS5611.read();
  float realPressure = readPressure();
  float realTemperature = readTemperature();
  // float thermistorTemperature = getThermistorTemp();

  // Read MQ135 component
  float mq135_data = mq135_sensor.getPPM(); 
  String air_quality = classifyAirQuality(mq135_data);

  String ms5611Data = "P: " + String(realPressure) + "\t" + "T(MS5611): " + String(realTemperature);
  String secondary_mission = ms5611Data + "\t AQ Value: " + String(mq135_data) + "\t AQ: " + air_quality;
  // Get GPS Data
  readGPSData();
  // String CansatGPS = "Latitude: " + String(latitude, 6) + "\t"+ "Longitude: " + String(longitude, 6) + "\t Altitude: " + String(altitude) + "\t"  + String(date) + " " + String(day) + " " + String(year); 
  String primaryMission = String(realPressure) + " | " + String(altitude) + " | " + String(realTemperature) + " | " + String(getThermistorTemp()) + " | " + String(latitude) + " | " + String(longitude) + " | " + String(getSDCardStatus()) + " | " + String(hour) + ":" + String(minute) + ":" + String(second);
  Serial.println(primaryMission); 
  Serial.println(secondary_mission);  
  writeData(primaryMission,secondary_mission);
  Serial2.println(primaryMission);
  Serial2.println(secondary_mission);
  buzz();
  delay(300);
}

// Buzz
void buzz(){
    digitalWrite(14, HIGH);
    delay(200);
    digitalWrite(14, LOW);
}

// SD Card Module Functions
void writeData(String data1,String data2){
  data_file = SD.open("/rocket_data.txt", FILE_APPEND);
  if(data_file){
    data_file.println(data1);
    data_file.println(data2);
  }
  else{
    Serial.println("File Not Found");
  }
}

// Get SDCard Status
bool getSDCardStatus() {
  return SD.begin(SD_CS);
}

// Get Thermistor Data
  float getThermistorTemp() {
     int rawValue = analogRead(THERMISTOR_PIN);
    float voltage = rawValue * (3.3 / 4095.0);

    float resistance = SERIES_RESISTOR / ((3.3 / voltage) - 1);
    
    float steinhart;
    steinhart = resistance / NOMINAL_RESISTANCE;
    steinhart = log(steinhart);
    steinhart /= BETA_COEFFICIENT;
    steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;

    return steinhart;
  }

// Get MQ135 Data
float ppmData(float temperature,float humidity){
  return mq135_sensor.getCorrectedPPM(temperature, humidity);
}

String classifyAirQuality(float ppm) {
  if (ppm <= 300) {
    return "Good Quality";
  } else if (ppm > 300 && ppm <= 1000) {
    return "Moderate Quality";
  } else {
    return "Bad Quality";
  }
}


// Read pressure from MS5611
float readPressure() {
  return MS5611.getPressure();
}

// Read temperature from MS5611
float readTemperature() {
  return MS5611.getTemperature();
}

// Read GPS data
void readGPSData() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        altitude = gps.altitude.meters();
      }
      if (gps.date.isValid()) {
        date = gps.date.month();
        day = gps.date.day();
        year = gps.date.year();
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
      }
    }
  }
}

// Apogee Detection Function
void checkApogee() {
    if (altitude >= 800) {
        if (altitude > maxAltitude) {
            maxAltitude = altitude;
            falling = false;
            fallStartTime = 0;
        } else {
            if (!falling) {
                falling = true;
                fallStartTime = millis();
            } else if (millis() - fallStartTime >= 3000) {
                apogee = true;
            }
        }
    }
}

// Detatch Method goes here