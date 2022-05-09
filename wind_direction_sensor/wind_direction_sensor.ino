#include "FTTech_SAMD51Clicks.h"
// XBEE 2
#include "FTTech_Xbee.h"

#define PIN_CH1_4a20 A1 // Wind direction
#define PIN_CH2_4a20 A0 // Wind Speed
#define DEBUG false

int BAUDRATE = 115200;

/* ***************************************************************************
 *            INTERVALO
 * ***************************************************************************
*/
  unsigned long previousMillis = 0; // Stores the last time reading was made
  const long interval = 5000;       // Interval at which to read (milliseconds)

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Configuração   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FTTech_Xbee xbee; // Create object with the serial parameter

void setup() {
  //while(!Serial);
  digitalWrite(52, HIGH); //Put Click 4 ON
  //4-20 Click uses external feeding, so
  //you doesn't need to power it with the click output.

  Serial4.begin(BAUDRATE);
  digitalWrite(A9, HIGH); //Xbee Reset
  xbee.begin(BAUDRATE);
  Serial.println(F("*****************INICIOU*****************"));
}

void loop() {

  // ===========================================================================================
  //                 ===================== Efetua a Leitura =======================
  // ===========================================================================================
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    float CH1_4a20 = analogRead(PIN_CH1_4a20) * (3.3/1023);
    float CH2_4a20 = analogRead(PIN_CH2_4a20) * (3.3/1023);

    // save the last time reading
    previousMillis = currentMillis;

    if(DEBUG){
      Serial.println("Canal 1: ");
      Serial.print("\tTensão: "); Serial.println(CH1_4a20);
      Serial.println("DIREÇÃO: " + String(getWindDirection(CH1_4a20),3));
      Serial.println("Canal 2 : ");
      Serial.print("\tTensão: "); Serial.println(CH2_4a20);
      Serial.println("SPEED: " + String(getWindSpeed(CH2_4a20),3));
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

      // save the last time reading
      previousMillis = currentMillis;
      
      // ===== Send Wind Speed to Xbee =====
      String sensor_spd = "FTMS,SPD," + String(getWindSpeed(CH2_4a20),2);
      for(int i = 0; i < sensor_spd.length(); i++){
        Serial4.write(sensor_spd[i]);
      }
      Serial4.flush();
      Serial.println("Enviado SPD");
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
