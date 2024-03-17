#include <Firebase_ESP_Client.h>
#include <WiFiManager.h>
//#include <ESPmDNS.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP085.h>


#include "addons/TokenHelper.h" //Provide the token generation process info.
#include "addons/RTDBHelper.h" //Provide the RTDB payload printing info and other helper functions.

//---------------------------------
//GLOBALNE//
//---------------------------------

ModbusMaster node;
SoftwareSerial mySerial(16, 4);

WiFiManager wm;

#define API_KEY "AIzaSyAgO8bf3n4nXjz8-GzxyJ6WLlbuBSjjyag"  // Insert Firebase project API Key
#define DATABASE_URL "https://esp32-example-383bf-default-rtdb.europe-west1.firebasedatabase.app/"  // Insert RTDB URLefine the RTDB URL

FirebaseData fbdo;   //Define Firebase Data object
FirebaseAuth auth;
FirebaseConfig config;

unsigned int sendDataPrevMillis = 0;
unsigned int sendDataPrevMillis_2 = 0;
unsigned int sendDataPrevMillis_read = 0;

bool signupOK = false;                     //since we are doing an anonymous sign in 

#define WIFI_PIN 27
#define LED_WIFI_OK 26
#define LED_WIFI_BAD 25

#define przekaznik1 33
#define przekaznik2 32

String relay1_state;  //pobieranie stanu ON / OFF z firebase
String relay2_state;  //pobieranie stanu ON / OFF z firebase
bool relay2_automatic;  //pobieranie stanu 1 / 0 przekaznika 2

unsigned int  timeout   = 120; // seconds to run for
unsigned int  startTime = millis();
bool portalRunning      = false;
bool startAP            = false; // start AP and webserver if true, else start only webserver

LiquidCrystal_I2C lcd(0x27, 16, 2); //ZASILANIE 5V!

Adafruit_BMP085 bmp;
//---------------------------------
//SETUP//
//---------------------------------
void setup() {
  Serial.begin(2400);
  mySerial.begin(2400);
  node.begin(001, mySerial);
  
  pinMode(WIFI_PIN, INPUT_PULLUP);
  pinMode(LED_WIFI_OK, OUTPUT);
  pinMode(LED_WIFI_BAD, OUTPUT);
  pinMode(przekaznik1, OUTPUT);
  pinMode(przekaznik2, OUTPUT);
  digitalWrite (przekaznik1, LOW);
  digitalWrite (przekaznik2, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();                     
  lcd.print("START...");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wykrywanie");
  lcd.setCursor(0, 1);
  lcd.print("czujnikow");
  delay(2000);
  lcd.clear();
  if (!bmp.begin(0x77)) {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    lcd.print("Nie wykryto BMP");
    delay(2000);
    lcd.clear();
  }
  lcd.print("OK");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ustaw WIFI przez");
  lcd.setCursor(0, 1);
  lcd.print("> WIFI SETUP");

  
  //Serial.print("Starting...");
  WiFi.mode(WIFI_STA);
  wm.autoConnect("WIFI SETUP");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Polonczono");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(2000);
  lcd.clear();
    
  config.api_key = API_KEY;  // Assign the api key (required)
  config.database_url = DATABASE_URL; // Assign the RTDB URL (required)

  if (Firebase.signUp(&config, &auth, "", "")){   // Sign up
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h   Assign the callback function for the long running token generation task
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(false);

}


//-------------------------------------------------------------
//FUNKCJE//
//-------------------------------------------------------------
float readVoltage() {
 
  uint8_t result;
  uint16_t data[2];
  result = node.readInputRegisters(0, 2);
  delay(70);
  
  if (result == node.ku8MBSuccess) {
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    uint32_t combinedData = (static_cast<uint32_t>(data[0]) << 16) | data[1];
    float voltageValue = (data[0] << 16) | data[1];
    
    memcpy(&voltageValue, &combinedData, sizeof(float));
   
    return voltageValue;
  } else {
    Serial.print("Błąd odczytu napięcia!: ");
    Serial.println(result);
    return 0;
  }
 
}
//-------------------------------------------------------------
float readCurrent() {
  
  uint8_t result;
  uint16_t data[2];
  result = node.readInputRegisters(6, 2);
  delay(70);

  if (result == node.ku8MBSuccess) {
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    uint32_t combinedData = (static_cast<uint32_t>(data[0]) << 16) | data[1];
    float currentValue = (data[0] << 16) | data[1];
    
    memcpy(&currentValue, &combinedData, sizeof(float));
    
    return currentValue;
  } else {
    Serial.print("Błąd odczytu natężenia prądu!: ");
    Serial.println(result);
    return 0;
  }
  
}
//-------------------------------------------------------------
float readPower() {
  
  uint8_t result;
  uint16_t data[2];
  result = node.readInputRegisters(12, 2);
  delay(70);
  
  if (result == node.ku8MBSuccess) {
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    uint32_t combinedData = (static_cast<uint32_t>(data[0]) << 16) | data[1];
    float powerValue = (data[0] << 16) | data[1];
    
    memcpy(&powerValue, &combinedData, sizeof(float));
   
    return powerValue;
  } else {
    Serial.print("Błąd odczytu mocy!: ");
    Serial.println(result);
    return 0;
  }
  
}
//-------------------------------------------------------------
float readImpEnergy() {
  uint8_t result;
  uint16_t data[2];
  result = node.readInputRegisters(72, 2);
  delay(70);
  
  if (result == node.ku8MBSuccess) {
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    uint32_t combinedData = (static_cast<uint32_t>(data[0]) << 16) | data[1];
    float impEnergyValue = (data[0] << 16) | data[1];
    
    memcpy(&impEnergyValue, &combinedData, sizeof(float));
   
    return impEnergyValue;
  } else {
    Serial.print("Błąd odczytu mocy!: ");
    Serial.println(result);
    return 0;
  }
}
//-------------------------------------------------------------
float readExpEnergy() {
  uint8_t result;
  uint16_t data[2];
  result = node.readInputRegisters(74, 2);
  delay (70);
  
  if (result == node.ku8MBSuccess) {
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    uint32_t combinedData = (static_cast<uint32_t>(data[0]) << 16) | data[1];
    float expEnergyValue = (data[0] << 16) | data[1];
    
    memcpy(&expEnergyValue, &combinedData, sizeof(float));
   
    return expEnergyValue;
  } else {
    Serial.print("Błąd odczytu mocy!: ");
    Serial.println(result);
    return 0;
  }
}
//-------------------------------------------------------------
float readTotEnergy() {
  uint8_t result;
  uint16_t data[2];
  result = node.readInputRegisters(342, 2);
  delay (70);

  if (result == node.ku8MBSuccess) {
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    uint32_t combinedData = (static_cast<uint32_t>(data[0]) << 16) | data[1];
    float TotEnergyValue = (data[0] << 16) | data[1];
    
    memcpy(&TotEnergyValue, &combinedData, sizeof(float));
   
    return TotEnergyValue;
  } else {
    Serial.print("Błąd odczytu mocy!: ");
    Serial.println(result);
    return 0;
  }
}
//-------------------------------------------------------------
void doWiFiManager(){
  
  // is auto timeout portal running
  if(portalRunning){
    wm.process(); // do processing

    // check for timeout
    if((millis()-startTime) > (timeout+100000)){
      Serial.println("portaltimeout");
      portalRunning = false;
      if(startAP){
        wm.stopConfigPortal();
      }
      else{
        wm.stopWebPortal();
      } 
    }
  }
//-------------------------------------------------------------

  // is configuration portal requested?
  if(digitalRead(WIFI_PIN) == LOW && (!portalRunning)) {
    if(startAP){
      Serial.println("Button Pressed, Starting Config Portal");
      wm.setConfigPortalBlocking(false);
      wm.startConfigPortal();
    }  
    else{
      Serial.println("Button Pressed, Starting Web Portal");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Usunieto WIFI");
      lcd.setCursor(0, 1);
      lcd.print("> WIFI SETUP");
      wm.resetSettings();

      wm.autoConnect("WIFI SETUP");
      lcd.clear();
    }  
    portalRunning = true;
    startTime = millis();
  }
 
  if(WiFi.isConnected()) {
    digitalWrite(LED_WIFI_OK, HIGH);
    digitalWrite(LED_WIFI_BAD, LOW);
    //Serial.println("POLACZONY");
  } else {
    digitalWrite(LED_WIFI_OK, LOW);
    digitalWrite(LED_WIFI_BAD, HIGH);
    WiFi.reconnect();
  }
}
//-------------------------------------------------------------
void sendToFirebase_siec(float voltageValue, float currentValue, float powerValue){

  String nap = String(voltageValue, 2);
  String pra = String(currentValue, 2);
  String moc = String(powerValue, 2);
  
  
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 3000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();

      delay(5);
    if (Firebase.RTDB.setString(&fbdo, "Siec/Nap", nap)){
     // Serial.println("PASSED PATH: " + fbdo.dataPath()+ " TYPE: " + fbdo.dataType() + " " + nap + " V");  
    }
    else {
      //Serial.println("Send Siec 1 FAILED - REASON: " + fbdo.errorReason());
      
    }
    
    delay(5);
    if (Firebase.RTDB.setString(&fbdo, "Siec/Prad", pra)){
     // Serial.println("PASSED PATH: " + fbdo.dataPath()+ " TYPE: " + fbdo.dataType() +" " + pra + " A"); 
    }
    else {
      //Serial.println("Send Siec 2 FAILED - REASON: " + fbdo.errorReason());
      
    }

    delay(5);
    if (Firebase.RTDB.setString(&fbdo, "Siec/Moc", moc)){
     // Serial.println("PASSED PATH: " + fbdo.dataPath()+ " TYPE: " + fbdo.dataType() +" " + moc + " W");
     
    }
    else {
      //Serial.println("Send Siec 3 FAILED - REASON: " + fbdo.errorReason());
      
    }
  }

}
//-------------------------------------------------------------
void sendToFirebase_energia(float impEnergyValue, float expEnergyValue, float TotEnergyValue){

    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure();
    pressure = 0.01 * pressure;
    
    String impo = String(impEnergyValue, 2);
    String expo = String(expEnergyValue, 2);
    String tota = String(TotEnergyValue, 2);

    String temp = String(temperature,2);
    String pres = String(pressure,2);
  
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis_2 > 14773 || sendDataPrevMillis_2 == 0)){
    sendDataPrevMillis_2 = millis();

    delay(5);
    if (Firebase.RTDB.setString(&fbdo, "Energia/Imp", impo)){
     // Serial.println("PASSED PATH: " + fbdo.dataPath()+ " TYPE: " + fbdo.dataType() + " " + impo + " kWh");
    
    }
      else {
        Serial.println("SEND 1 FAILED - REASON: " + fbdo.errorReason());
      }
      
    delay(5);
    // Write an Float number on the database path test/float
    if (Firebase.RTDB.setString(&fbdo, "Energia/Exp", expo)){
     // Serial.println("PASSED PATH: " + fbdo.dataPath()+ " TYPE: " + fbdo.dataType() +" " + expo + " kWh");
    
    }
      else {
        //Serial.println("SEND 2 FAILED REASON: " + fbdo.errorReason());
      }
      
    delay(5);
    if (Firebase.RTDB.setString(&fbdo, "Energia/Tot", tota)){
    //  Serial.println("PASSED PATH: " + fbdo.dataPath()+ " TYPE: " + fbdo.dataType() +" " + tota + " kWh");
    }
      else {
        //Serial.println("SEND 3 FAILED REASON: " + fbdo.errorReason());
      }

    delay(5);  
    if (Firebase.RTDB.setString(&fbdo, "Pogoda/Temp", temp)){
    //  Serial.println("PASSED PATH: " + fbdo.dataPath()+ " TYPE: " + fbdo.dataType() +" " + tota + " kWh");
    }
      else {
        //Serial.println("SEND 4 FAILED REASON: " + fbdo.errorReason());
      }

    delay(5);
    if (Firebase.RTDB.setString(&fbdo, "Pogoda/Cisn", pres)){
    //  Serial.println("PASSED PATH: " + fbdo.dataPath()+ " TYPE: " + fbdo.dataType() +" " + tota + " kWh");
    }
      else {
        //Serial.println("SEND 5 FAILED REASON: " + fbdo.errorReason());
      }
    
  }
}
//-------------------------------------------------------------
void readFromFirebase(float powerValue) {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis_read > 355 || sendDataPrevMillis_read == 0)){
    sendDataPrevMillis_read = millis();
    
    delay(5);
    if (Firebase.RTDB.getString(&fbdo, "/RELAY/1")) {
      if(fbdo.dataType() == "string")  {
        relay1_state = fbdo.stringData();
       // Serial.println("PASSED: " + relay1_state);
        if (relay1_state == "ON") {
          digitalWrite (przekaznik1, HIGH);
        }
        else if (relay1_state == "OFF") {
          digitalWrite (przekaznik1, LOW);
        }
      }
  
      else {
        //Serial.println("READ 1 FAILED REASON: " + fbdo.errorReason());
        
      }
     }

    delay(5);
    if (Firebase.RTDB.getString(&fbdo, "/RELAY/2")) {
      if(fbdo.dataType() == "string")  {
        relay2_state = fbdo.stringData();
       // Serial.println("PASSED: " + relay2_state);
        if ((relay2_state == "ON") && (relay2_automatic == false)) {
          digitalWrite (przekaznik2, HIGH);
        }
        else if (relay2_state == "OFF") {
          digitalWrite (przekaznik2, LOW);
        }
      }
  
      else {
        //Serial.println("READ 2 FAILED REASON: " + fbdo.errorReason());
        
      }
     }

    delay(5);
    if (Firebase.RTDB.getBool(&fbdo, "/RELAY/2Automatyczny")) {
        if(fbdo.dataType() == "boolean")  {
          relay2_automatic = fbdo.boolData();
          //Serial.println("PASSED: " + relay2_automatic);
         
          if ((relay2_state == "ZERO") && (relay2_automatic == true)) {
              if (powerValue > 39) {
                digitalWrite(przekaznik2, HIGH);
              } else {
                digitalWrite(przekaznik2, LOW);
              }
            } else if ((relay2_state == "ZERO") && (relay2_automatic == false)) {
              digitalWrite(przekaznik2, LOW);
              }
        }
  
        else {
          //Serial.println("READ 3 FAILED REASON: " + fbdo.errorReason());
 
        }
      }
  }
}
//-------------------------------------------------------------
void LCD(float voltageValue, float currentValue, float powerValue) {
  
  lcd.setCursor(0, 0);
  lcd.print(voltageValue);
  lcd.print(" V");
  lcd.setCursor(10, 0);
  lcd.print(currentValue);
  lcd.print(" A");
  lcd.setCursor(0, 1);
  lcd.print(powerValue);
  lcd.setCursor(6, 1);
  lcd.print(" W");
  lcd.setCursor(10, 1);
  if(WiFi.isConnected()) {
    lcd.print("WiFi");
  } else {
    lcd.print("----");
  }
  
}
//-------------------------------------------------------------


//-------------------------------------------------------------
//LOOP//
//-------------------------------------------------------------
void loop() {

    
    sendToFirebase_siec(readVoltage(), readCurrent(), readPower());
    delay(50);
    sendToFirebase_energia(readImpEnergy(), readExpEnergy(), readTotEnergy());
    delay(50);
    readFromFirebase(readPower());
    delay(50);
    lcd.setCursor(15, 1);
    lcd.print("#");

    doWiFiManager();
    delay(50);
    LCD(readVoltage(), readCurrent(), readPower());
    delay(50);
    
    lcd.setCursor(15, 1);
    lcd.print(" ");

    
    // Wyświetlenie danych na Serial Monitorze
//    Serial.print("Napięcie: ");
//    Serial.println(wynikNAPIECIE);
//    Serial.print("Prąd: ");
//    Serial.println(wynikPRAD);
//    Serial.print("Moc: ");
//    Serial.println(wynikMOC);
//    Serial.print("Import: ");
//    Serial.println(wynikIMPORT);
//    Serial.print("Export: ");
//    Serial.println(wynikEXPORT);
//    Serial.print("Total: ");
//    Serial.println(wynikTOTAL);

}
