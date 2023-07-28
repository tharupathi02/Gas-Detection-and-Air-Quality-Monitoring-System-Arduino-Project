#include <MQUnifiedsensor.h>
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <DHT.h>

#define FIREBASE_HOST "iot-lec-default-rtdb.firebaseio.com"       // Firebase HOST
#define FIREBASE_AUTH "tux3VWvRMfgybk3MdKojJkVgcnXnMVK5MFc5twl9"  // Firebase AUTH code
#define WIFI_SSID "POCO F3"                                 // Enter your WIFI Name
#define WIFI_PASSWORD "12345678"                                  // Enter your Password

// Pin Declaration
#define RELAY1ELECTRONIC_EQUIP D1  // LOW - Current Flowing | HIGH - Current Not Flowing
#define RELAY2AIRBLOWER D2         // LOW - Current Flowing | HIGH - Current Not Flowing
#define BUZZER D3
#define DHTPIN D4

// MQ-6 Library Definitions
#define board "ESP8266"
#define Voltage_Resolution 3.3
#define pin A0                 //Analog input 0 of your arduino
#define type "MQ-6"            //MQ6
#define ADC_Bit_Resolution 10  // For arduino UNO/MEGA/NANO
#define RatioMQ6CleanAir 10    //RS / R0 = 10 ppm

MQUnifiedsensor MQ6(board, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

// Variable Diclaration
float cureentHumidity;
float cureentTemperature;

// MQ-6 Sensor Reading Data
float currentH2;
float currentLPG;
float currentCH4;
float currentCO;
float currentAlcohol;

float firebaseCO;
float firebaseAlcohol;
float firebaseCO2;
float firebaseToluen;
float firebaseNH4;
float firebaseAceton;

float lpgThresholdLevel1 = 10;
float lpgThresholdLevel2 = 50;
float lpgThresholdLevel3 = 75;

String message = "";
bool messageSent = false;  // Flag to track if the message has been sent.
String mobileNumber = "+94769835600";

// Initialization
#define DHTTYPE DHT11  // Initialize dht type as DHT11
DHT dht(DHTPIN, DHTTYPE);

unsigned long TIMEPREVIOURMILLS_TEMP = 0;
unsigned long TIMEPREVIOURMILLS_MQ = 0;
unsigned long TIMEPREVIOURMILLS_CHECKING = 0;

void setup() {
  Serial.begin(9600);
  dht.begin();  //reads dht sensor data

  // MQ-6 Init
  MQ6.setRegressionMethod(1);
  MQ6.init();
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ6.update();
    calcR0 += MQ6.calibrate(RatioMQ6CleanAir);
    Serial.print(".");
  }
  MQ6.setR0(calcR0 / 10);
  Serial.println("  done!.");

  // PinModes
  pinMode(DHTPIN, INPUT);
  pinMode(RELAY1ELECTRONIC_EQUIP, OUTPUT);
  pinMode(RELAY2AIRBLOWER, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(RELAY1ELECTRONIC_EQUIP, LOW);
  digitalWrite(RELAY2AIRBLOWER, HIGH);

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
  Serial.println("Connecting to Firebase...");
  while (Firebase.failed() == true) {
    Serial.print(F("."));
    delay(300);
  }
  Serial.println();
  Serial.println();
  Serial.println("Firebase Connected.");
  Firebase.setString("/GAS_AIR_PROJECT/CONNECTION/SSID", WIFI_SSID);
  Firebase.setString("/GAS_AIR_PROJECT/DETECTION/GAS/LPG", "NO");

  sendMessageGSM("Gas Detection and Air Quality Monitoring System");

  ringBuzzer(200);
}

void loop() {

  // Check MQ-6 Sensor values and send data into Firebase every 1 seconds
  if (millis() - TIMEPREVIOURMILLS_MQ > 1000 || TIMEPREVIOURMILLS_MQ == 0) {
    TIMEPREVIOURMILLS_MQ = millis();
    readMQ6Data();
    getMQ135FirebaseData();
  }

  // Check Environment condition and sensor data every 2 seconds
  if (millis() - TIMEPREVIOURMILLS_CHECKING > 2000 || TIMEPREVIOURMILLS_CHECKING == 0) {
    TIMEPREVIOURMILLS_CHECKING = millis();
    checkEnvironment();
  }

  // Check DHT11 Sensor values and send data into Firebase every 5 seconds
  if (millis() - TIMEPREVIOURMILLS_TEMP > 5000 || TIMEPREVIOURMILLS_TEMP == 0) {
    TIMEPREVIOURMILLS_TEMP = millis();
    readTemperatureData();
  }

  RecieveMessage();
}

void checkEnvironment() {
  int lpgValue = Firebase.getFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_6_SENSOR/LPG");

  if (lpgValue > lpgThresholdLevel1 && lpgValue < lpgThresholdLevel2) {
    Firebase.setString("/GAS_AIR_PROJECT/DETECTION/GAS/LPG", "YES");
    digitalWrite(RELAY1ELECTRONIC_EQUIP, HIGH);
    digitalWrite(RELAY2AIRBLOWER, LOW);
    Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/ALL_ELECTRONIC_EQUIP", "OFF");
    Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/AIR_BLOWER", "ON");
    ringBuzzer(1000);
    if (!messageSent) {
      sendMessageGSM("Level 1: Gas Detected\nA low level of gas has been detected in the area.\n\nLP Gas Level -> " + lpgValue);
      messageSent = true;
    }
  } else if (lpgValue > lpgThresholdLevel2 && lpgValue < lpgThresholdLevel3) {
    Firebase.setString("/GAS_AIR_PROJECT/DETECTION/GAS/LPG", "YES");
    digitalWrite(RELAY1ELECTRONIC_EQUIP, HIGH);
    digitalWrite(RELAY2AIRBLOWER, LOW);
    Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/ALL_ELECTRONIC_EQUIP", "OFF");
    Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/AIR_BLOWER", "ON");
    ringBuzzer(500);
    if (!messageSent) {
      sendMessageGSM("Level 2: Gas Detected (High)\nA significant amount of gas has been detected.\n\nLP Gas Level -> " + lpgValue);
      messageSent = true;
    }
  } else if (lpgValue > lpgThresholdLevel3) {
    Firebase.setString("/GAS_AIR_PROJECT/DETECTION/GAS/LPG", "YES");
    digitalWrite(RELAY1ELECTRONIC_EQUIP, HIGH);
    digitalWrite(RELAY2AIRBLOWER, LOW);
    Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/ALL_ELECTRONIC_EQUIP", "OFF");
    Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/AIR_BLOWER", "ON");
    ringBuzzer(200);
    if (!messageSent) {
      sendMessageGSM("Level 3: Gas Detected (Critical)\nA critical gas leak has been detected.\n\nLP Gas Level -> " + lpgValue);
      messageSent = true;
    }
  } else if (lpgValue < lpgThresholdLevel3) {
    Firebase.setString("/GAS_AIR_PROJECT/DETECTION/GAS/LPG", "NO");
    digitalWrite(RELAY1ELECTRONIC_EQUIP, LOW);
    digitalWrite(RELAY2AIRBLOWER, HIGH);
    Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/ALL_ELECTRONIC_EQUIP", "ON");
    Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/AIR_BLOWER", "OFF");
    digitalWrite(BUZZER, LOW);
    if (messageSent) {
      sendMessageGSM("Good News!\nGas levels are back to normal. The area is now safe for occupancy and regular activities.\n\nLP Gas Level -> " + lpgValue);
      messageSent = false;
    }
  }
}

void readMQ6Data() {
  // Read MQ6 Sensor Data
  MQ6.update();

  MQ6.setA(1009.2);
  MQ6.setB(-2.35);
  currentLPG = MQ6.readSensor();

  MQ6.setA(88158);
  MQ6.setB(-3.597);
  currentH2 = MQ6.readSensor();

  MQ6.setA(2127.2);
  MQ6.setB(-2.526);
  currentCH4 = MQ6.readSensor();

  MQ6.setA(1000000000000000);
  MQ6.setB(-13.5);
  currentCO = MQ6.readSensor();

  MQ6.setA(50000000);
  MQ6.setB(-6.017);
  currentAlcohol = MQ6.readSensor();

  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_6_SENSOR/LPG", currentLPG);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_6_SENSOR/H2", currentH2);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_6_SENSOR/CH4", currentCH4);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_6_SENSOR/CO", currentCO);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_6_SENSOR/ALCOHOL", currentAlcohol);

  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_6_LPG_GAS/LPG", currentLPG);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_6_LPG_GAS/H2", currentH2);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_6_LPG_GAS/CH4", currentCH4);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_6_LPG_GAS/CO", currentCO);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/MQ_SENSORS/MQ_6_LPG_GAS/ALCOHOL", currentAlcohol);

  Serial.print("\t|\tLPG ");
  Serial.print(currentLPG);
  Serial.print("\tH2 ");
  Serial.print(currentH2);
  Serial.print("\tCH4 ");
  Serial.print(currentCH4);
  Serial.print("\tCO ");
  Serial.print(currentCO);
  Serial.print("\tAlcohol ");
  Serial.println(currentAlcohol);
}

void readTemperatureData() {
  // Read DHT11 Sensor Data
  cureentHumidity = dht.readHumidity();
  cureentTemperature = dht.readTemperature();

  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/DHT11_SENSOR/HUMIDITY", cureentHumidity);
  Firebase.setFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/DHT11_SENSOR/TEMPERATURE", cureentTemperature);

  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/DHT11_SENSOR/HUMIDITY", cureentHumidity);
  Firebase.pushFloat("/GAS_AIR_PROJECT/All_VALUES/DHT11_SENSOR/TEMPERATURE", cureentTemperature);

  Serial.print("Temperature ");
  Serial.print(cureentTemperature);
  Serial.print("\tHumidity ");
  Serial.print(cureentHumidity);
}

void sendMessageGSM(String gsmMessage) {
  Serial.println("AT+CMGF=1");                    //Sets the GSM Module in Text Mode
  delay(1000);                                      // Delay of 1 second
  Serial.println("AT+CMGS=\"+94769835600\"\r");
  delay(1000);
  Serial.println(gsmMessage);  // The SMS text you want to send
  delay(100);
  Serial.println((char)26);  // ASCII code of CTRL+Z for saying the end of sms to  the module
  delay(1000);
}

void callGSM() {
  Serial.println("ATD+94769835600;");  //replace x by your number
  delay(100);
  Serial.println("ATH");
  delay(1000);
  Serial.println("calling.....");
}

void ringBuzzer(int tonedelay) {
  digitalWrite(BUZZER, HIGH);
  delay(tonedelay);
  digitalWrite(BUZZER, LOW);
  delay(tonedelay);
  digitalWrite(BUZZER, HIGH);
  delay(tonedelay);
  digitalWrite(BUZZER, LOW);
  delay(tonedelay);
}

void getMQ135FirebaseData() {
  firebaseCO = Firebase.getFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/CO");
  firebaseAlcohol = Firebase.getFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/ALCOHOL");
  firebaseCO2 = Firebase.getFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/CO2");
  firebaseToluen = Firebase.getFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/TOLUEN");
  firebaseNH4 = Firebase.getFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/NH4");
  firebaseAceton = Firebase.getFloat("/GAS_AIR_PROJECT/CURRENT_VALUES/MQ_135_SENSOR/ACETON");
}

void RecieveMessage() {
  message = "";
  if (Serial.available() > 0) {
    message = Serial.readString();
    message.toUpperCase();

    if (message.indexOf("") >= 0) {
      message = "";
    }
    if (message.indexOf("MQ") >= 0) {
      sendMessageGSM("MQ-6 Gas Sensor Values\nLPG - " + String(currentLPG) + "\nH2 - " + String(currentH2) + "\nCH4 - " + String(currentCH4) + "\nCO - " + String(currentCO) + "\nAlcohol - " + String(currentAlcohol));
      message = "";
    }
    if (message.indexOf("TEMP") >= 0) {
      sendMessageGSM("Temperature and Humidity\nTemperature - " + String(cureentTemperature) + "\Humidity - " + String(cureentHumidity));
      message = "";
    }
    if (message.indexOf("ALL OFF") >= 0) {
      digitalWrite(RELAY1ELECTRONIC_EQUIP, HIGH);
      Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/ALL_ELECTRONIC_EQUIP", "OFF");
      sendMessageGSM("All Electronic Equipments are OFF");
      message = "";
    }
    if (message.indexOf("ALL ON") >= 0) {
      digitalWrite(RELAY1ELECTRONIC_EQUIP, LOW);
      Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/ALL_ELECTRONIC_EQUIP", "ON");
      sendMessageGSM("All Electronic Equipments are ON");
      message = "";
    } else if (message.indexOf("AIR OFF") >= 0) {
      digitalWrite(RELAY2AIRBLOWER, HIGH);
      Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/AIR_BLOWER", "OFF");
      sendMessageGSM("Air Blower is OFF");
      message = "";
    }
    if (message.indexOf("AIR ON") >= 0) {
      digitalWrite(RELAY2AIRBLOWER, LOW);
      Firebase.setString("/GAS_AIR_PROJECT/SWITCH_PANEL/RELAYS/AIR_BLOWER", "ON");
      sendMessageGSM("Air Blower is ON");
      message = "";
    }
    if (message.indexOf("") >= 0) {
      message = "";
    }

  } else {
    message = "";
  }
}
