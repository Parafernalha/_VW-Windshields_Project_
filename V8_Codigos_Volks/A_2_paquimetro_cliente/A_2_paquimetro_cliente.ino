
// 2º Paquimetro - Direita Superior
#define CLOCK_PIN 19 //  Laranja
#define DATA_PIN  23 //  Verde

//Paquimetro Superior
#define CLOCK_PIN_SUP 26 //  Laranja
#define DATA_PIN_SUP  25 //  Verde

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "SSD1306Wire.h"

char buf[20];
unsigned long tmpTime;
unsigned long temp;
int j;

// Paquimetro Laterial
int sign;
int inches;
long value;
//Paquimetro Superior
int sign2;
int inches2;
long value2;

float result;
float resultado_superior; //leitura do paquimetro superior
bool mm = true; //define mm to false if you want inces values

//Display
SSD1306Wire  display(0x3c, 5, 4);

//Comunicação

uint8_t broadcastAddress[] = {0x9C, 0x9C, 0x1F, 0xE5, 0x93, 0xF4};

typedef struct struct_message {
    float val1;
    float val2;
    float val3;
    float val4;


} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



void Paquimetro(){
  //temp=millis();
  while(digitalRead(CLOCK_PIN)==LOW) {}
  tmpTime=micros();
  //temp=millis();
  while(digitalRead(CLOCK_PIN)==HIGH) {}
  if((micros()-tmpTime)<500) return;
  readCaliper(); 
  buf[0]=' ';
  dtostrf(result,6,3,buf+1); strcat(buf," in ");  
  dtostrf(result*2.54,6,3,buf+1); strcat(buf," cm "); 
}

// Leitura paquimetro cliente
void readCaliper(){
  sign=1;
  value=0;
  inches=0;
  for(int i=0;i<24;i++) {
    while(digitalRead(CLOCK_PIN)==LOW) {}
    while(digitalRead(CLOCK_PIN)==HIGH) {}
    if(digitalRead(DATA_PIN)==HIGH) {
      if(i<20) value|=(1<<i);
      if(i==20) sign=-1;
      if(i==23) inches=1; 
    }
  }if(mm){
    result=(value*sign)/100.0;
  }else{
  result=(value*sign)/(inches?2000.0:100.0); //We map the values for inches, define mm to false if you want inces values
  }
}



// Saida do Paquimetro Superior
void Paquimetro_Superior(){

  while(digitalRead(CLOCK_PIN_SUP)==LOW) {}
  tmpTime=micros();

  while(digitalRead(CLOCK_PIN_SUP)==HIGH) {}
  if((micros()-tmpTime)<500) return;
  readCaliper2(); 
  buf[0]=' ';
  dtostrf(resultado_superior,6,3,buf+1); strcat(buf," in ");  
  dtostrf(resultado_superior*2.54,6,3,buf+1); strcat(buf," cm "); 
}
// Leitura Paquimetro Superior
void readCaliper2(){
  sign2=1;
  value2=0;
  inches2=0;
  for(int i=0;i<24;i++) {
    while(digitalRead(CLOCK_PIN_SUP)==LOW) {}
    while(digitalRead(CLOCK_PIN_SUP)==HIGH) {}
    if(digitalRead(DATA_PIN_SUP)==HIGH) {
      if(i<20) value2|=(1<<i);
      if(i==20) sign2=-1;
      if(i==23) inches2=1; 
    }
  }
  if(mm){
    resultado_superior=(value2*sign2)/100.0;
  }else{
    resultado_superior=(value2*sign2)/(inches2?2000.0:100.0); //We map the values for inches, define mm to false if you want inces values
  }
}

void Tela(){
  
  String resultStg = "A_2: ";     // empty string
  resultStg.concat(resultado_superior);

  display.clear();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(63, 13, resultStg);

  display.display();
}


void setup() 
{
  Serial.begin(115200);

    
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){   
    Serial.println("Failed to add peer");
    return;
  }

  display.init();
  
  pinMode(CLOCK_PIN, INPUT);
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN_SUP, INPUT);
  pinMode(DATA_PIN_SUP, INPUT);
 
}

void loop(){
  Paquimetro();
  Paquimetro_Superior();
  myData.val1 = result;
  myData.val2 = -1;
  myData.val3 = -1;
  myData.val4 = resultado_superior;
  Serial.println(result);
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.println("Sent with success 1° Paqui");
  }
  else {
      Serial.println("Error sending the data 1º Paqui");
  }
  delay(25);
  
  Tela();
  
}
