/*
 *  This sketch fixes the MQ2 measuring issue when using Wifi.h
 *  Wifi.h deactivates pin 25 analogical input on esp32.ino default sketch.
 *  This is because Wifi.h works only with ADC1 for analogical measurement
 *  Please check the esp32-wroom-32d.jpg image on ESP32 folder
 * 

 * ADC1 GPIO
 * ADC1_CH0 (GPIO 36) // only tested on this and it works as expected :)
 * ADC1_CH1 (GPIO 37)
 * ADC1_CH2 (GPIO 38)
 * ADC1_CH3 (GPIO 39)
 * ADC1_CH4 (GPIO 32)
 * ADC1_CH5 (GPIO 33)
 * ADC1_CH6 (GPIO 34)
 * ADC1_CH7 (GPIO 35)
 *
 * ADC2 GPIO
 * ADC2_CH0 (GPIO 4)
 * ADC2_CH1 (GPIO 0)
 * ADC2_CH2 (GPIO 2)
 * ADC2_CH3 (GPIO 15)
 * ADC2_CH4 (GPIO 13)
 * ADC2_CH5 (GPIO 12)
 * ADC2_CH6 (GPIO 14)
 * ADC2_CH7 (GPIO 27)
 * ADC2_CH8 (GPIO 25)
 * ADC2_CH9 (GPIO 26)
 *
 */

#include <WiFi.h>
#include <PubSubClient.h>


// Update these with values suitable for your network.
const char* ssid = "XXXX";
const char* password = "XXXXX";
const char* mqtt_server = "192.168.2.39";
const int BUZZER_PIN = 32; // GIOP32 pin connected to buzzer

#define mqtt_port 1883
#define MQTT_USER "sarfraaz"
#define MQTT_PASSWORD "XXXXX"
#define MQTT_SERIAL_PUBLISH_CH "MyHome/Sensor/ESP32/serialdata/tx"
#define MQTT_RESTART_RECEIVER_CH "MyHome/Sensor/ESP32/restart"


WiFiClient wifiClient;

PubSubClient client(wifiClient);


//Include the library
#include <MQUnifiedsensor.h>

#include <LiquidCrystal_I2C.h>

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

/************************Hardware Related Macros************************************/
#define         Board                   ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.

//https://www.amazon.com/HiLetgo-ESP-WROOM-32-Development-Microcontroller-Integrated/dp/B0718T232Z (Although Amazon shows ESP-WROOM-32 ESP32 ESP-32S, the board is the ESP-WROOM-32D)
#define         Pin                     (36) //check the esp32-wroom-32d.jpg image on ESP32 folder 

/***********************Software Related Macros************************************/
#define         Type                    ("MQ-2") //MQ2 or other MQ Sensor, if change this verify your a and b values.
#define         Voltage_Resolution      (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         ADC_Bit_Resolution      (12) // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         RatioMQ2CleanAir        (9.83) //RS / R0 = 9.83 ppm
/*****************************Globals***********************************************/
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
/*****************************Globals***********************************************/



void setup()
{

  //Init the serial port communication - to debug the library
  Serial.begin(115200); //Init serial port
  delay(10);
  pinMode(BUZZER_PIN, OUTPUT);
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  //Set math model to calculate the PPM concentration and the value of constants
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(987.99); MQ2.setB(-2.162); // Configurate the ecuation values to get H2 concentration
  
/*
    Exponential regression:
    Gas    | a      | b
    H2     | 987.99 | -2.162
    LPG    | 574.25 | -2.222
    CO     | 36974  | -3.109
    Alcohol| 3616.1 | -2.675
    Propane| 658.71 | -2.168
  */

  /*****************************  MQ Init ********************************************/ 
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/ 
  MQ2.init(); 
 
  /* 
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ2.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
  // In this routine the sensor will measure the resistance of the sensor supposing before was pre-heated
  // and now is on clean air (Calibration conditions), and it will setup R0 value.
  // We recomend execute this routine only on setup or on the laboratory and save on the eeprom of your arduino
  // This routine not need to execute to every restart, you can load your R0 if you know the value
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  //MQ2.serialDebug(true); uncomment if you want to print the table on the serial port

    // We start by connecting to a WiFi network

   
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    client.setServer(mqtt_server, mqtt_port);   
    client.setCallback(callback);     
    reconnect();
    digitalWrite(BUZZER_PIN, HIGH);
}



void loop()
{
  client.loop();
  MQ2.update(); // Update data, the arduino will be read the voltage on the analog pin
  //MQ2.serialDebug(); // Will print the table on the serial port
  float ppmValue;
  ppmValue = MQ2.readSensor();


 // set cursor to first column, first row
  lcd.setCursor(0, 0);
  // print message
  lcd.print("PPM");
  delay(1000);
  // clears the display to print new message
  lcd.clear();
  // set cursor to first column, second row
  lcd.setCursor(0,1);
  lcd.print(String(ppmValue).c_str());
  delay(1000);
  lcd.clear(); 

  if(ppmValue > 13) {    
    digitalWrite(BUZZER_PIN, LOW); // turn on
    Serial.println("High");
  }
  else
  {    
    digitalWrite(BUZZER_PIN, HIGH);  // turn off
  }
 
 if(!client.connected()) {    
    reconnect();
 }    
 if(ppmValue > 25)
 {
 delay(1000); //Sampling frequency
 }
 else
 {
 delay(10000);
 }
 client.publish("MyHome/Sensor/ESP32/MQ2",String(ppmValue).c_str());
 Serial.println("PPM  ");
 Serial.println(String(ppmValue).c_str());

 Serial.println(client.connected());

    
}


void callback(char* topic, byte *payload, unsigned int length) {
    Serial.println("-------new message from broker-----");
    Serial.print("channel:");
    Serial.println(topic);
    Serial.print("data:");  
    Serial.write(payload, length);
    ESP.restart();
    Serial.println();
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQTT_USER,MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.setKeepAlive(90);
      client.publish("MyHome/Sensor/presence/ESP32/", "hello world");      
      client.subscribe(MQTT_RESTART_RECEIVER_CH);
      } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
