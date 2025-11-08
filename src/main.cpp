/*
  Data: 29/08/2023
  Alexandre Nuernberg - alexandreberg@gmail.com
  Programa que envia dados de temperatura e humidade de um sensor DHT11 ou BME280 para um servidor MySQL no Hostinger
  de forma segura usando httpd_post com SSL.
  Adaptado de:
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-mysql-database-php/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

MCU: ESP32 Dev KIT: env:esp32doit-devkit-v1
Ligações do módulo DHT11:
s - io5

Resultado:
  O resultado da comunicação depois da transferência dos dados para o servidor deve ser:
  Leitura do Monitor Serial:
Leitura Nº: 5
Temperatura: 20.20
Humidade: 85.00
 
httpRequestData: api_key=xxxxxxxxxxxxxx&sensor=DHT11-01&location=Sitio&value1=20.20&value2=85.00&value3= 
HTTP Response code: 200

No servidor:
https://ilha3d.com/temp01/esp-data.php
ID	Sensor	Location	Value 1	Value 2	Value 3	Timestamp
39023	DHT11-01	Sitio	20.40	85.00		2023-08-29 18:41:49
39022	DHT11-01	Sitio	20.40	85.00		2023-08-29 18:41:34
39021	DHT11-01	Sitio	20.30	84.00		2023-08-29 18:41:03

TODO:
- OK Corrigido - 02.09.2023 - corrigir o horário está em horário do servidor 2hs a menos que no Brasil
- OK Criado - 02.09.2023 - Criação do arquivo credentials.h para não sincronizar as senhas com o GitHub:
  Para o sketch rodar, deve haver um arquivo na pasta: src/credentials.h com o seguinte conteúdo:
  ---> inicio do arquivo
  const char* ssid     = "nomeDoSSID";
  const char* password = "senhaDoWiFi";
  const char* serverName = "https://link para o aquivo de post no servidor web";
  String apiKeyValue = "ChaveDeAcessoAoArquivoDePost";
  String sensorName = "nomeDoSensor";
  String sensorLocation = "localizacaoDoSensor";
  <---- Final do arquivo

- OK Criado e sincronizado - 02.09.2023 - Sincronizar o projeto com o GitHub: alexandreberg/Esp32_Temerature_Humidity_Monitor_with_DHT11_BMExxx
https://github.com/alexandreberg/Esp32_Temerature_Humidity_Monitor_with_DHT11_BMExxx.git
- OK Corrigido - 02.09.2023 - Alterar api_key no arquivo post-esp-data.php do servidor e do credentials

TODOs PENDENTES:
- 09.09.2023 - Está em testes ... => corrigir o problema de desconexão do WiFi

- corrigir prob de não ler os dado do sensores:
  ID	Sensor	Location	Value 1	Value 2	Value 3	Timestamp
  39100	DHT11-01	Sitio	nan	nan		2023-08-30 01:05:13
  NaN stands for "Not a Number".
  float temperature;
float bTemperature;

temperature = dht.readTemperature();

if(!isnan(temperature)){
  bTemperature = temperature;
  Serial.println(temperature);

}
else{
  Serial.println(bTemperature);
}

- 08 nov 2025: adjusting new apikey

*/

//Choose your sensor type: SensorDHT for DHT11 or SensorBME for BME280
#define SensorDHT
//#define SensorBME // Usa o BME280 ao invés do DHT11

//Definição de bibliotecas:
#include <Arduino.h> //Include Arduino Headers
#include "credentials.h" // Wifi credentials file

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>

#ifdef SensorBME
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#endif

#ifdef SensorDHT
#include "DHT.h"
#endif

#ifdef SensorBME
#include <SPI.h>
#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;  // I2C
Adafruit_BME280 bme(BME_CS);  // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);  // software SPI
#endif

#ifdef SensorDHT
#define DHTTYPE DHT11 // DHT 11
uint8_t DHTPin = 5; 
DHT dht(DHTPin, DHTTYPE); 
float Temperature;
float Humidity;
int i=1;
#endif

void restartESP32() {
  Serial.println("Restarting ESP32...");
  delay(10000);  // Give some time for serial output to be sent

  // Perform the restart using the ESP.restart() function
  ESP.restart();
}

void leituraDHT() { 
  //Leitura do DHT:
  Temperature = dht.readTemperature(); 
  Humidity = dht.readHumidity(); 
  /*if (Humidity < 0 || Humidity > 101) {
    dht.begin(); //start DHT sensor again
    delay(5000);
    Temperature = dht.readTemperature(); 
    Humidity = dht.readHumidity(); 
  }*/
  Serial.print("Leitura Nº: "); Serial.println(i);
  Serial.print("Temperatura: "); Serial.println(Temperature);
  Serial.print("Humidade: "); Serial.println(Humidity);
  Serial.println(" ");
  i ++;
}

void setup() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("");
  Serial.println("Monitor de Temperatura e Humidade com ESP32 e DHT11");
  Serial.println("Versao: 2025110801");
  Serial.println("");
  Serial.println("");
  
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  #ifdef SensorDHT
  pinMode(DHTPin, INPUT);
  dht.begin();
  delay(5000); //try to avoit failure of leitures (nan)
  #endif

  #ifdef SensorBME
  // (you can also pass in a Wire library object like &Wire2)
  bool status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring or change I2C address!");
    while (1);
  }
  #endif
}

void loop() {
  #ifdef SensorDHT
  leituraDHT();
  #endif

  //Check WiFi connection status
  if(WiFi.status()!= WL_CONNECTED) {
    Serial.println("WiFi Disconnected, trying to reconnect...");
    WiFi.begin(ssid, password);
    Serial.println("Reconnecting to WiFi...");
    while(WiFi.status() != WL_CONNECTED) { 
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
    //Testando o comando ESP.restart():
    //restartESP32(); // restard esp32 board    
  }

  else {
   WiFiClientSecure *client = new WiFiClientSecure;
    client->setInsecure(); //don't use SSL certificate
    HTTPClient https;
    
    // Your Domain name with URL path or IP address with path
    https.begin(*client, serverName);
    
    // Specify content-type header
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
    // Prepare your HTTP POST request data
    
    String httpRequestData = "api_key=" + apiKeyValue + "&sensor=" + sensorName
                          + "&location=" + sensorLocation + "&value1=" + String(dht.readTemperature())
                          + "&value2=" + String(dht.readHumidity()) + "&value3=" + String(" ") + "";
                         // + "&location=" + sensorLocation + "&value1=" + String(bme.readTemperature())
                         // + "&value2=" + String(bme.readHumidity()) + "&value3=" + String(bme.readPressure()/100.0F) + "";
                          
    Serial.print("httpRequestData: ");
    Serial.println(httpRequestData);

    // You can comment the httpRequestData variable above
    // then, use the httpRequestData variable below (for testing purposes without the BME280 sensor)
    //Testing the connection with the webserver and MySQL server:
    //String httpRequestData = "api_key=xxxxxxxxxxxxxxxxxxxxx&sensor=DHT11-01&location=TESTE&value1=24.75&value2=49.54&value3=1005.14";
    //String httpRequestData = "api_key=xxxxxxxxxxxxxxxxxxxxx&sensor=Teste001&location=TESTE&value1=999999&value2=999999&value3=999999";
    

    // Send HTTP POST request
    int httpResponseCode = https.POST(httpRequestData);
     
    // If you need an HTTP request with a content type: text/plain
    //https.addHeader("Content-Type", "text/plain");
    //int httpResponseCode = https.POST("Hello, World!");
    
    // If you need an HTTP request with a content type: application/json, use the following:
    //https.addHeader("Content-Type", "application/json");
    //int httpResponseCode = https.POST("{\"value1\":\"19\",\"value2\":\"67\",\"value3\":\"78\"}");
    
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
      if (httpResponseCode == -1) {
        restartESP32(); //If there is error -1 restart esp32
      }
    }
    // Free resources
    https.end();
  }
  //Send an HTTP POST request every 5 minutes
  delay(300000);  //5min

}