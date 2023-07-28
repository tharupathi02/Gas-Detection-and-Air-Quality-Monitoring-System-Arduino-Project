#include <MQUnifiedsensor.h>
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <DHT.h>

#define FIREBASE_HOST "iot-lec-default-rtdb.firebaseio.com"       // Firebase HOST
#define FIREBASE_AUTH "tux3VWvRMfgybk3MdKojJkVgcnXnMVK5MFc5twl9"  // Firebase AUTH code
#define WIFI_SSID "POCO F3"                                 // Enter your WIFI Name
#define WIFI_PASSWORD "12345678"                                  // Enter your Password

//MQ-135 Definitions
#define board "ESP8266"
#define Voltage_Resolution 3.3
#define pin A0                 //Analog input 0 of your arduino
#define type "MQ-135"            //MQ6
#define ADC_Bit_Resolution 10  // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6    //RS / R0 = 3.6 ppm

float currentCO;
float currentAlcohol;
float currentCO2;
float currentToluene;
float currentNH4;
float currentAcetone;

MQUnifiedsensor MQ135(board, Voltage_Resolution, ADC_Bit_Resolution, pin, type);


void setup() {
  Serial.begin(9600);

  // WIFI Connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  // MQ-135 Init
  MQ135.setRegressionMethod(1);
  MQ135.init(); 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println(" Done!.");

}

void loop() {
  
  // Read MQ135 Sensor Data
  readMQ6Sensor();

  // Push All Values into the Firebase
  pushAllSensorValues();

}

void readMQ6Sensor(){
  MQ135.update();

  MQ135.setA(605.18); MQ135.setB(-3.937);
  currentCO = MQ135.readSensor(); 

  MQ135.setA(77.255); MQ135.setB(-3.18);
  currentAlcohol = MQ135.readSensor();

  MQ135.setA(110.47); MQ135.setB(-2.862);
  currentCO2 = MQ135.readSensor();

  MQ135.setA(44.947); MQ135.setB(-3.445);
  currentToluene = MQ135.readSensor();
  
  MQ135.setA(102.2 ); MQ135.setB(-2.473);
  currentNH4 = MQ135.readSensor();

  MQ135.setA(34.668); MQ135.setB(-3.369);
  currentAcetone = MQ135.readSensor();


  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/CO", currentCO);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/ALCOHOL", currentAlcohol);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/CO2", currentCO2);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/TOLUENE", currentToluene);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/NH4", currentNH4);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/ACETONE", currentAcetone);

}

void pushAllSensorValues() {

  // MQ-135 Values
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_135_SENSOR/CO", currentCO);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_135_SENSOR/ALCOHOL", currentAlcohol);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_135_SENSOR/CO2", currentCO2);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_135_SENSOR/TOLUENE", currentToluene);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_135_SENSOR/NH4", currentNH4);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_135_SENSOR/ACETONE", currentAcetone);
}
