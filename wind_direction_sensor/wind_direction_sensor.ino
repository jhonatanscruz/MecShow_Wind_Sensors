/*
 * Code to read Mikroe Environment Click(BME680 Sensor)
 * and Campbell 034B Wind Set
 * All data is sent to Gateway by Xbee
 * 
 * Autor: Jhonatan da Silva Cruz
 * From FTTECH Software Team
 * 
 * Last Update: 2022-06-24
 */
#include "FTTech_SAMD51Clicks.h"
#include "FTTech_Xbee.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define PIN_CH1_4a20 A1 // Wind direction
#define PIN_CH2_4a20 A0 // Wind Speed
#define DEBUG false

int BAUDRATE = 115200;
Adafruit_BME680 bme; // I2C

/* ***************************************************************************
 *            INTERVALO
 * ***************************************************************************
*/
  unsigned long previousMillis = 0; // Stores the last time reading was made
  const long interval = 1000;       // Interval at which to read (milliseconds)
  float dir_temp[4];
  float spd_temp[4];

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Configuração   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup() {
  //4-20 Click uses external feeding, so
  //you doesn't need to power it with the click output.
  digitalWrite(52, HIGH); //Put Click 4 ON
  FTClicks.turnON_5V(); // Enable 5V output
  FTClicks.turnON(2);   // Enable the 3.3V output for Click 2
  Serial.begin(9600);
  Serial4.begin(BAUDRATE);
  digitalWrite(A9, HIGH); //Xbee Reset
  delay(100);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization for BME680
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);

  Serial.println(F("*****************INICIOU*****************"));
  delay(100);
}

void loop() {

  // ===========================================================================================
  //                 ===================== Efetua a Leitura =======================
  // ===========================================================================================
  // the interval at which you want to read and send data.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    // ==================================== Measuring data ====================================
    uint8_t aux = 0;
    float CH1_4a20; // Wind direction
    float CH2_4a20; // Wind speed

    while(aux < 3){
      dir_temp[aux] = analogRead(PIN_CH1_4a20) * (3.3/1023);
      spd_temp[aux] = analogRead(PIN_CH2_4a20) * (3.3/1023);
      aux++;
    }

    //Trata as leituras de Direção do Vento
    if(dir_temp[0] != 0 && dir_temp[1] != 0 && dir_temp[2] != 0) CH1_4a20 = dir_temp[2];
    else CH1_4a20 = 0;

    //Trata as leituras de Velocidade do Vento
    if(spd_temp[0] != 0 && spd_temp[1] != 0 && spd_temp[2] != 0) CH2_4a20 = spd_temp[2];
    else CH2_4a20 = 0;

  // ==========================================================================================

    // save the last time reading
    previousMillis = currentMillis;

    if (!bme.performReading()) {
      Serial.println("Failed to perform reading :(");
    }

    if(DEBUG){
      Serial.println("Canal 1: ");
      Serial.print("\tTensão: "); Serial.println(CH1_4a20);
      Serial.println("DIREÇÃO: " + String(getWindDirection(CH1_4a20),3));
      Serial.println("Canal 2 : ");
      Serial.print("\tTensão: "); Serial.println(CH2_4a20);
      Serial.println("SPEED: " + String(getWindSpeed(CH2_4a20),3));
      
      Serial.print("Temperature = ");
      Serial.print(bme.temperature);
      Serial.println(" *C");

      Serial.print("Pressure = ");
      Serial.print(bme.pressure / 100.0);
      Serial.println(" hPa");

      Serial.print("Humidity = ");
      Serial.print(bme.humidity);
      Serial.println(" %");
    }

    else{
      // ===== Send Wind Direction to Xbee =====
      String sensor_dir = "FTMS,DIR," + String(getWindDirection(CH1_4a20),2);
      for(int i = 0; i < sensor_dir.length(); i++){
        Serial4.write(sensor_dir[i]);
      }
      Serial4.flush();
      Serial.println("Enviado DIR");
      delay(100);

      // ===== Send Wind Speed to Xbee =====
      String sensor_spd = "FTMS,SPD," + String(getWindSpeed(CH2_4a20),2);
      for(int i = 0; i < sensor_spd.length(); i++){
        Serial4.write(sensor_spd[i]);
      }
      Serial4.flush();
      Serial.println("Enviado SPD");
      delay(100);

      // ===== Send BME680 TEMPERATURE to Xbee =====
      String BME_TEMP = "FTMS,TEM," + String(bme.temperature,2);
      for(int i = 0; i < sensor_dir.length(); i++){
        Serial4.write(BME_TEMP[i]);
      }
      Serial4.flush();
      Serial.println("Enviado BME_TEMP");
      delay(100);

      // ===== Send BME680 PRESSURE to Xbee =====
      String BME_PRESS = "FTMS,PRE," + String(bme.pressure / 100.0,2);
      for(int i = 0; i < sensor_dir.length(); i++){
        Serial4.write(BME_PRESS[i]);
      }
      Serial4.flush();
      Serial.println("Enviado BME_PRESS");
      delay(100);

      // ===== Send BME680 HUMIDITY to Xbee =====
      String BME_HUM = "FTMS,HUM," + String(bme.humidity,2);
      for(int i = 0; i < sensor_dir.length(); i++){
        Serial4.write(BME_HUM[i]);
      }
      Serial4.flush();
      Serial.println("Enviado BME_HUM");
    }
  }
}

float getWindDirection(float CH1_4a20){

  float maxVoltage = 2.93;              // VOLTS
  float minVoltage = 0.19;              // VOLTS
  float tensao = CH1_4a20;

  if(tensao > maxVoltage) maxVoltage = tensao;
  if(tensao < minVoltage) minVoltage = tensao;

  float grads = (tensao - minVoltage) * 360 / (maxVoltage - minVoltage); // GRAUS

  float interval = 45;

  String direction[8] = {"Norte","Nordeste","Leste","Sudeste","Sul","Sudoeste","Oeste","Noroeste"};

  String message = "";

  if( grads >= (360 - (interval/2)) || grads < (interval/2)){
    message = "NORTE " + (String)grads + "º";
  }
  else if(grads >= (interval/2) && grads < 3*(interval/2)){
    message = "NORDESTE " + (String)grads + "º";
  }
  else if(grads >= 3*(interval/2) && grads < 5*(interval/2)){
    message = "LESTE " + (String)grads + "º";
  }
  else if(grads >= 5*(interval/2) && grads < 7*(interval/2)){
    message = "SUDESTE " + (String)grads + "º";
  }
  else if(grads >= 7*(interval/2) && grads < 9*(interval/2)){
    message = "SUL " + (String)grads + "º";
  }
  else if(grads >= 9*(interval/2) && grads < 11*(interval/2)){
    message = "SUDOESTE " + (String)grads + "º";
  }
  else if(grads >= 11*(interval/2) && grads < 13*(interval/2)){
    message = "OESTE " + (String)grads + "º";
  }
  else if(grads >= 13*(interval/2) && grads < 15*(interval/2)){
    message = "NOROESTE " + (String)grads + "º";
  }

  return grads;
}

float getWindSpeed(float CH2_4a20){

  float maxVoltage = 2.93;  // VOLTS
  float minVoltage = 0.19;  // VOLTS
  float tensao = CH2_4a20;

  if(tensao > maxVoltage) maxVoltage = tensao;
  if(tensao < minVoltage) minVoltage = tensao;

  float spd = (tensao - minVoltage) * 50 / (maxVoltage - minVoltage); // VELOCIDADE

  if(spd < 0.5) spd = 0;

  return spd;
}
