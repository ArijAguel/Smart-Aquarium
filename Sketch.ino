#include <ESP32Servo.h>
#include <LiquidCrystal.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";

//***Set server***
const char* mqttServer = "broker.hivemq.com"; 
int port = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void mqttConnect() {
  while(!mqttClient.connected()) {
    Serial.println("Attemping MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if(mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
    }
    else {
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

// Function to publish data to MQTT topics
void publishData(float temperature, float pH, float distance, float waterLevel) {
  mqttClient.publish("ESP32Client-/temperature", String(temperature).c_str());
  mqttClient.publish("ESP32Client-/pH", String(pH).c_str());
  mqttClient.publish("ESP32Client-/distance", String(distance).c_str());
  mqttClient.publish("ESP32Client-/water_level", String(waterLevel).c_str());


  if ((temperature < 20 || temperature > 30) ) {
    mqttClient.publish("ESP32Client-/AlertTemperature", "Alert! : the temperature must be between 20 and 28 Celsius");
  }

   
  if ( (distance < 200)) {
       mqttClient.publish("ESP32Client-/AlertDistance", "Alert! the distance must be greater than 200 cm");
  }
}

//callback: appelée lorsqu'un message MQTT est reçu. 
//Elle affiche le sujet du message et le contenu du message dans la console.
void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String strMsg;
  for(int i=0; i<length; i++) {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);


}

int tempPin = 32;
int phPin = 34;
int waterLevelPin = 35;
int trigUltraPin = 23;
int echoUltraPin = 22;
int fishFeederPin = 33;
int turnOnFishFeederBtnPin = 16;
int buzzerPin = 4;
int turnOnFanPin = 15;



Servo fishFeeder; //servomoteur
LiquidCrystal lcd(13, 12, 14, 27, 26, 25); //lcd 

//Led
int LED_COUNT = 16;
int LED_PIN = 19;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB+NEO_KHZ800);
// pixels:bande de LEDs NeoPixel pour afficher des effets lumineux rouge 
//si la valeur du niveau d'eau ou de la temprature sont anormales

//utilise un capteur à ultrasons placé au fond de l'aquarium pour mesurer la distance 
//à la surface de l'eau


long getDistance() {
  //distance par rapport à un obstacle (changement de milieu)
  digitalWrite(trigUltraPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigUltraPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigUltraPin, LOW);
  
  long duration = pulseIn(echoUltraPin, HIGH);
  long disanceCm = duration * 0.034 / 2;
  //distance = vitesse son dans l'eau * durée allez retour / 2
  
  return disanceCm;
}

void setup() {
  pinMode(tempPin, INPUT);
  pinMode(phPin, INPUT);
  pinMode(turnOnFanPin, OUTPUT);
  pinMode(trigUltraPin, OUTPUT);
  pinMode(echoUltraPin, INPUT);
  pinMode(waterLevelPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(turnOnFishFeederBtnPin, INPUT);

  Serial.begin(115200);
  Serial.begin(9600);

  fishFeeder.attach(fishFeederPin);
  fishFeeder.write(0);

  lcd.begin(16, 2);
  pixels.begin();           // INITIALIZE NeoPixel pixels object (REQUIRED)
  pixels.show();            // Turn OFF all pixels ASAP
  pixels.setBrightness(200);

  //Connect to Wifi
  Serial.print("Connecting to WiFi");
  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive( 90 );
  mqttConnect();
  
}

//effectue un balayage de couleur sur la bande de LEDs NeoPixel
void colorWipe(uint32_t color, int wait) {
  for(int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, color);
    pixels.show();
    delay(wait);
  }
}
void turnOnLed() {
}

void turnOffLed() {
  //pixels.clear();
  //pixels.show();
}

//ventilateur
void turnOnFan() {
  digitalWrite(turnOnFanPin, HIGH);
}

void turnOffFan() {
  digitalWrite(turnOnFanPin, LOW);
}

float readTemperature() {
  const float VCC = 3.3;             // NodeMCU on board 3.3v vcc
  const float R2 = 10000;            // 10k ohm series resistor
  const float adc_resolution = 4096; // 12-bit adc

  const float A = 0.001129148;   // thermistor equation parameters
  const float B = 0.000234125;
  const float C = 0.0000000876741; 

  float Vout, Rth, temperature, adc_value; 

  adc_value = adc_resolution-analogRead(tempPin)+0.5; // switch direction
  Vout = (adc_value * VCC) / adc_resolution; 
  Rth = (VCC * R2 / Vout) - R2; // Formula for R2 as Pull-down: Vcc-Rth-R2-GND


  temperature = (1 / (A + (B * log(Rth)) + (C * pow((log(Rth)),3))));   // Temperature in kelvin

  temperature = temperature - 273.15;  // Temperature in degree celsius
  Serial.print(" Temperature = ");
  Serial.print(temperature);
  Serial.println(" degree celsius");
  return temperature;
}







void loop() {  

 
  // Read temperature
  float celcius = readTemperature();
  lcd.print("Temperature: ");
  lcd.print(celcius);
  delay (1000);
  
  
  lcd.setCursor(0, 1);
  // Read distance
  float distanceCm = getDistance();
  lcd.print("Distance: ");
  lcd.print(distanceCm);
  delay (1000);
  // Check temperature conditions for the buzzer alert
  if ((celcius < 20 || celcius > 30) ) {
    // Turn on the buzzer for 1 second
    tone(buzzerPin, 300, 1000);
    Serial.println("Buzzer alert! : the temperature must be between 20 and 28 Celsius");
    colorWipe(pixels.Color(255, 0, 0), 50); // Red
  }
  else  
   // Check  distance conditions for the buzzer alert
  if ( (distanceCm < 200)) {
    // Turn on the buzzer for 1 second
    tone(buzzerPin, 300, 1000);
    Serial.println("Buzzer alert! the distance must be greater than 200 cm");
    colorWipe(pixels.Color(255, 0, 0), 50); // Red
  }
  else {
    colorWipe(pixels.Color(0, 255, 0), 50); }// Green
  
  
  fishFeeder.write(90);
  delay(1000);
  fishFeeder.write(0);
  if (digitalRead(turnOnFishFeederBtnPin) == LOW) {
  fishFeeder.write(150);} // Operate the fish feeder
 
  if ((celcius > 30))  
   { turnOnFan();}
  else if ((celcius = 20)) 
   {turnOffFan();}
  
 
  lcd.clear();
  delay (500);

  
  // Read pH value
  float pH = analogRead(phPin);
  lcd.setCursor(0, 0);
  lcd.print("PH: ");
  lcd.print(pH);
  delay(1000);
  

 

  
  float waterLevel = analogRead(waterLevelPin);
  lcd.setCursor(0, 1);
  lcd.print("Water: ");
  lcd.print(waterLevel);
  delay(1000);
  lcd.clear();
  delay(500);
  //Turn off Led
  turnOffLed();
  // Turn off Fan
  if (celcius > 28)
    turnOnFan();
  delay(1000);
  
  // Publish data to MQTT topics
  mqttClient.loop();
  if (mqttClient.connected()) {
    Serial.print("connected to mqtt");
    publishData(celcius, pH, distanceCm, waterLevel);
  }

}
