#include <Wire.h>
#include <MS5611.h> 
#include <MQ135.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <DHT.h>



// ------------------------------------------- PRIMARY MISSION SETUP FOUND HERE --------------------------------------------------------------------
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

// SD Card module
#define SD_CS 5
File data_file;

// Transmitter/ Sender
#define HC12_BAUD 9600    // HC-12 communication speed


// ------------------------------------------- SECONDARY MISSION SETUP FOUND HERE --------------------------------------------------------------------
// MQ135
#define PIN_MQ135 34
MQ135 mq135_sensor(PIN_MQ135);

//DHT12
#define DHTPIN 4
#define DHTTYPE DHT12
DHT dht(DHTPIN, DHTTYPE);

// Apogee Calculation Variables
bool apogee = false;
float maxAltitude = 0;
unsigned long fallStartTime = 0;
bool falling = false;


// --------------------------------------------------------- CANSAT SETUP IS HERE --------------------------------------------------------------------------------
void setup() {
  // Transmitter Initialization
  // Serial2.begin(HC12_BAUD, SERIAL_8N1, 3, 1);

    // GREEN
    pinMode(25, OUTPUT);
    
    // YELLOW
    pinMode(26, OUTPUT);
    
    // RED
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
  digitalWrite(26,HIGH);
  digitalWrite(27,LOW);

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
  
  digitalWrite(26,LOW);
  dht.begin();
  digitalWrite(25,HIGH);
  
  buzz();
}


// --------------------------------------------------------- CANSAT LOOP IS HERE --------------------------------------------------------------------------------

void loop() {
  
  // --------------------------------------------------------- PRIMARY MISSIONS ARE FOUND HERE --------------------------------------------------------------------------------
  // Read MS5611 component
  MS5611.read();
  float realPressure = readPressure();
  float MS5611Temperature = readTemperature();
  float thermistorTemp = getThermistorTemp();

  // Get GPS Data
  readGPSData();
  //                       PRESSURE                           ALTITUDE                    TEMPERATURE (MS5611)                TEMPERATURE (THERMISTOR)      LATITUDE                      LONGITUDE                       SDCARD STATUS                    HOUR                 MINUTE                SECOND               
  String primaryMission = String(realPressure) + " | " + String(altitude) + " | " + String(MS5611Temperature,2) + " | " + String(thermistorTemp,2) + " | " + String(latitude) + " | " + String(longitude) + " | " + String(getSDCardStatus()) + " | " + String(hour) + ":" + String(minute) + ":" + String(second);
  Serial.println(primaryMission); 

 
  // --------------------------------------------------------- SECONDARY MISSIONS ARE FOUND HERE --------------------------------------------------------------------------------

  //DHT12 MISSION
  float DHTHumidity = dht.readHumidity();
  
  // Read MQ135 component
  float mq135_data =  analogRead(PIN_MQ135);
  String air_quality = classifyAirQuality(mq135_data);

  String secondary_mission = String(DHTHumidity,2) + " | " + String(mq135_data) + " | " + air_quality;

  Serial.println(secondary_mission);  
  writeData(primaryMission,secondary_mission);
  
  delay(300);
}

// Buzz
void buzz(){
    digitalWrite(14, HIGH);
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

String classifyAirQuality(float ppm) {
    if (ppm <= 100) {
        return "Excellent";
    } else if (ppm <= 200) {
        return "Good";
    } else if (ppm <= 300) {
        return "Moderate";
    } else if (ppm <= 500) {
        return "Poor";
    } else if (ppm <= 1000) {
        return "Unhealthy";
    } else {
        return "Hazardous";
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
