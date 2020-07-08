
// =============Bibliotecas===========================================================
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>


#include <Arduino.h>
#include <analogWrite.h>

/*
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
*/
// ================================================================================
#define MOT_A1_PIN 5
#define MOT_A2_PIN 18
#define MOT_B1_PIN 19
#define MOT_B2_PIN 21
// ================================================================================

//=============Iniciar el servidor Wi-Fi==========================================
WiFiServer servidor(80);

char ssid[100]     = "INFINITUM2711";
char password[100] = "5KOEcUB87j";
bool modoServidor = true;
long int tiempoInicio = 0;
long int tiempoFinal = 0;

long int tiempoConexionInicio = 0;
long int tiempoComparacion = 0;
int contador = 0;
String currentLine = "";
bool finMensaje = false;

void conectaWiFi(){
  
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  
    WiFi.begin(ssid,password);
         
       

     
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
       
        Serial.print(".");
        
       
        
    }

 //Mensajes al monitor serial
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("MAC address: ");
    Serial.println(WiFi.macAddress());
   
    if (!MDNS.begin("robot")) {
      
    }
   else{
   servidor.begin();
   MDNS.addService("http", "tcp", 80); 
   } 
}
//========================================================================================================================

//*******************************************************Set up***********************************************************
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  conectaWiFi();
  tiempoInicio = millis();

   // Initialize the stepper driver control pins to output drive mode.
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

}
// ================================================================================
/// Set the current on a motor channel using PWM and directional logic.
/// Changing the current will affect the motor speed, but please note this is
/// not a calibrated speed control.  This function will configure the pin output
/// state and return.
///
/// \param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN  pin number xIN1 for the given channel
/// \param IN2_PIN  pin number xIN2 for the given channel

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    analogWrite(IN2_PIN, 0);

  } else { // stop or forward
    analogWrite(IN1_PIN, 0);
    analogWrite(IN2_PIN, pwm);
  }
}

void set_motor_currents(int pwm_A, int pwm_B)
{
  set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);
}



//**************************************************************************************************************************

//******************************Loop****************************************************************************************
void loop() {
  
  
  
  if(modoServidor){
    currentLine = " ";

   WiFiClient clienteServidor = servidor.available();
   finMensaje = false;
   if (clienteServidor) {
   tiempoConexionInicio = xTaskGetTickCount();
    
    while(clienteServidor.connected()){

      if(clienteServidor.available() > 0){
        char c = clienteServidor.read();             // read a byte, then
        

          Serial.print((char)c);                // print it out the serial monitor

          ///Recibe mensaje JSON - Inicio
        if(c == '}'){ finMensaje = true; }
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {

           
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

// --------------------Comenzar a recibir los mensajes JSON------------------------------------------
        if(finMensaje){
          String mensajeJSON = currentLine;
          //Serial.println(mensajeJSON);
            //Decodificacion mensaje JSON - inicio
            
            StaticJsonBuffer<100> bufferJSON;
            JsonObject& objetoJSON = bufferJSON.parseObject(mensajeJSON);
            if(objetoJSON.success()){
              int dir = objetoJSON["motor"];        
              
switch (dir) {
  case 1: //Adelante    I    D

   set_motor_currents(-200,200);
   delay(500);
   set_motor_currents(0,0);

    break;
  case 2: //Atras      I     D   
    set_motor_currents(0,0);
   delay(500);
   set_motor_currents(200,-200);
      delay(500);
   set_motor_currents(0,0);
   
    break;
  case 3://Izquierda   I     D
    set_motor_currents(0,0);
   delay(500);
    set_motor_currents(200,200); 
    break;
  case 4: //Derecha      I    D
   set_motor_currents(0,0);
   delay(500);      
   set_motor_currents(-200,-200);
    break;
  case 5: //Paro         I D
   set_motor_currents(0,0);
    break;
  case 6: //Vuelta Izquierda     I    D

   set_motor_currents(180,180);
   delay(250);
   set_motor_currents(0,0); 
    break;
  case 7: //Vuelta Derecha      I    D

   set_motor_currents(-180,-180);
   delay(250);
   set_motor_currents(0,0); 
    break;    
  case 8: //Especial      I    D
   set_motor_currents(0,0);
   delay(500);
   set_motor_currents(180,-180);
   delay(500);
   set_motor_currents(0,0); 
   delay(500);
   set_motor_currents(-180,180);
   delay(500);
   set_motor_currents(0,0);
   delay(500);
   set_motor_currents(-180,-180);   
   delay(500); 
   set_motor_currents(0,0);
   delay(500);   
   set_motor_currents(180,180);           
    break;        
}
            }

//--------------Decodificacion mensaje JSON - fin-----------------------------------------------
            

          
          
         clienteServidor.println("HTTP/1.1 200 OK");
            clienteServidor.println("Content-type:text/html");
            clienteServidor.println();

            // the content of the HTTP response follows the header:
            clienteServidor.println("Configuracion Recibida");

            // The HTTP response ends with another blank line:
            clienteServidor.println();
          
          //resuestaServidor(); 
                     
            

          break;
        }
        
       
        //resuestaServidor();
         
        //Termina recibir mensaje JSON
        tiempoComparacion =  xTaskGetTickCount();
        if(tiempoComparacion > (tiempoConexionInicio + 3000)){
            //clienteServidor.stop();
            Serial.println("Error timeout");
            break;

        }

      }
              
    }
            clienteServidor.stop();
    delay(500);
            
    
   }
  }
  else{
    tiempoFinal = millis();
  if(tiempoFinal > (tiempoInicio+10000)){
    tiempoInicio = millis();
    
  }
  }

}
