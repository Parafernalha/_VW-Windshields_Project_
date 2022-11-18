//Paquimetro Lateral
#define CLOCK_PIN 19 //  Laranja
#define DATA_PIN  23 //  Verde
//Paquimetro Superior
#define CLOCK_PIN_SUP 26 //  Laranja
#define DATA_PIN_SUP  25 //  Verde

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "SSD1306Wire.h"

// leds de sinalizacao
int led_vermelho_inf = 15;
int led_laranja_inf = 2;
int led_amarelo_inf = 0;
int led_verde = 21;
int led_amarelo_sup = 16;
int led_laranja_sup = 17;
int led_vermelho_sup = 18;

// leds de sinalizacao paquimetro superior
int sup_vermelho_acima = 27;
int sup_verde = 32;
int sup_vermelho_abaixo = 33;


char buf[20];
unsigned long tmpTime;

int controle_processo = 14;
int dif_inf_aux = 1;
int dif_sup_aux = 1;

// Paquimetro Laterial
int sign;
int inches;
long value;
//Paquimetro Superior
int sign2;
int inches2;
long value2;

float limite_minimo ;
float limite_maximo;

float result; // leitura A_1_Servidor
float resultado_superior; //leitura do paquimetro superior
float A_1_Servidor; // Paquimeto Servidor Superior Esquerda
float A_2_Cliente; // Paquimetro Cliente Superior Direito
float A_2_ClienteB; // Paquimetro Cliente Superior Direito
float B_1_Cliente; // Paquimetro Cliente Inferior Esquerda
float B_2_Cliente; // Paquimetro Cliente Inferior Direita
float dif_sup; // diferença paquimetros superiores
float dif_inf; // diferença paquimetros inferiores
bool mm = true; //define mm to false if you want inces values

//Display
SSD1306Wire  display(0x3c, 5, 4);

//Comunicação entre Esps

typedef struct struct_message {
    float val1;
    float val2;
    float val3;
    float val4;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  float var1 = myData.val1;
  float var2 = myData.val2;
  float var3 = myData.val3;
  float var4 = myData.val4;
  
  if(var1 != -1){
    A_2_Cliente = myData.val1;
  }
  if(var2 != -1){
    B_1_Cliente = myData.val2;
  }
  if(var3 != -1){
    B_2_Cliente = myData.val3;
  }
  if(var4 != -1){
    A_2_ClienteB = myData.val4;
  }
}

// Saida do Seu Paquimetro (Master)
void Paquimetro(){

  while(digitalRead(CLOCK_PIN)==LOW) {}
  tmpTime=micros();

  while(digitalRead(CLOCK_PIN)==HIGH) {}
  if((micros()-tmpTime)<500) return;
  readCaliper(); 
  buf[0]=' ';
  dtostrf(result,6,3,buf+1); strcat(buf," in ");  
  dtostrf(result*2.54,6,3,buf+1); strcat(buf," cm "); 
  A_1_Servidor = result;
}

// Leitura do Paquimetro do Master
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
  }
  if(mm){
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
// Saidas Leds
void Leds(){
  
  // VALORES LIMITES DOS LEDS
  float pos_amar = 0.20;
  float pos_lar = 1.00;
  float pos_verm = 2.00;

  
  // LED VERDE ACESO
  if((dif_sup > pos_amar * -1) && (dif_inf > pos_amar * -1) and (dif_sup < pos_amar) and (dif_inf < pos_amar)){
    digitalWrite(led_vermelho_inf, LOW);
    digitalWrite(led_laranja_inf, LOW);
    digitalWrite(led_amarelo_inf, LOW);
    digitalWrite(led_verde, HIGH);    // Verde
    digitalWrite(led_amarelo_sup, LOW);
    digitalWrite(led_laranja_sup, LOW);
    digitalWrite(led_vermelho_sup, LOW);
    }

   // LED AMARELO ACESO POR SINAL POSITIVO
   if((dif_sup >= pos_amar and dif_sup < pos_lar) or (dif_inf >= pos_amar and dif_inf < pos_lar)){
    digitalWrite(led_vermelho_inf, LOW);
    digitalWrite(led_laranja_inf, LOW);
    digitalWrite(led_amarelo_inf, LOW);
    digitalWrite(led_verde, LOW);
    digitalWrite(led_amarelo_sup, HIGH); // Amarelo
    digitalWrite(led_laranja_sup, LOW);
    digitalWrite(led_vermelho_sup, LOW);
    }

  // LED AMARELO ACESO POR SINAL NEGATIVO
   if((dif_sup <= pos_amar * -1 and dif_sup > pos_lar * -1) or (dif_inf <= pos_amar * -1 and dif_inf > pos_lar * -1)){
    digitalWrite(led_vermelho_inf, LOW);
    digitalWrite(led_laranja_inf, LOW);
    digitalWrite(led_amarelo_inf, HIGH); // Amarelo
    digitalWrite(led_verde, LOW);
    digitalWrite(led_amarelo_sup, LOW);
    digitalWrite(led_laranja_sup, LOW);
    digitalWrite(led_vermelho_sup, LOW);
    }
    
  // LED LARANJA ACESO POR SINAL POSITIVO
   if((dif_sup >= pos_lar and dif_sup < pos_verm) or (dif_inf >= pos_lar and dif_inf < pos_verm)){
    digitalWrite(led_vermelho_inf, LOW);
    digitalWrite(led_laranja_inf, LOW);
    digitalWrite(led_amarelo_inf, LOW);
    digitalWrite(led_verde, LOW);
    digitalWrite(led_amarelo_sup, LOW);
    digitalWrite(led_laranja_sup, HIGH); // Laranja
    digitalWrite(led_vermelho_sup, LOW);
   }
   
  // LED LARANJA ACESO POR SINAL NEGATIVO
    if((dif_sup <= pos_lar * -1 and dif_sup > pos_verm * -1) or (dif_inf <= pos_lar * -1 and dif_inf > pos_verm * -1)){
    digitalWrite(led_vermelho_inf, LOW);
    digitalWrite(led_laranja_inf, HIGH); // Laranja
    digitalWrite(led_amarelo_inf, LOW);
    digitalWrite(led_verde, LOW);
    digitalWrite(led_amarelo_sup, LOW);
    digitalWrite(led_laranja_sup, LOW);
    digitalWrite(led_vermelho_sup, LOW);
    }

  // LED VERMELHO ACESO POR SINAL POSITIVO
   if(dif_sup >= pos_verm or dif_inf >= pos_verm){
    digitalWrite(led_vermelho_inf, LOW);
    digitalWrite(led_laranja_inf, LOW);
    digitalWrite(led_amarelo_inf, LOW);
    digitalWrite(led_verde, LOW);
    digitalWrite(led_amarelo_sup, LOW);
    digitalWrite(led_laranja_sup, LOW);
    digitalWrite(led_vermelho_sup, HIGH); // Vermelho
   }
   
  // LED VERMELHO ACESO POR SINAL NEGATIVO
    if(dif_sup <= pos_verm * -1 or dif_inf <= pos_verm * -1 ){
    digitalWrite(led_vermelho_inf, HIGH); // Vermelho
    digitalWrite(led_laranja_inf, LOW);
    digitalWrite(led_amarelo_inf, LOW);
    digitalWrite(led_verde, LOW);
    digitalWrite(led_amarelo_sup, LOW);
    digitalWrite(led_laranja_sup, LOW);
    digitalWrite(led_vermelho_sup, LOW);
    }
}
void Leds_superior(){
  
  // VALORES LIMITES DOS LEDS
  float limite_minimo = 2.90;
  float limite_maximo = 3.10;
  
  if(resultado_superior <= limite_minimo or A_2_ClienteB <= limite_minimo){
    digitalWrite(sup_vermelho_acima, LOW);
    digitalWrite(sup_verde, LOW);  
    digitalWrite(sup_vermelho_abaixo, HIGH);
    }
   if(((resultado_superior > limite_minimo) and (resultado_superior < limite_maximo)) and ((A_2_ClienteB > limite_minimo) and (A_2_ClienteB < limite_maximo))){
    digitalWrite(sup_vermelho_acima, LOW);
    digitalWrite(sup_verde, HIGH);  
    digitalWrite(sup_vermelho_abaixo, LOW);
    }
   if(resultado_superior >= limite_maximo or A_2_ClienteB >= limite_maximo){
    digitalWrite(sup_vermelho_acima, HIGH);
    digitalWrite(sup_verde, LOW);  
    digitalWrite(sup_vermelho_abaixo, LOW);
  }
}
void Tela(){
  
  String resultStg = "Dif S: ";     // empty string
  resultStg.concat(dif_sup);

  String resultStg2 = "A 1: ";     // empty string
  resultStg2.concat(A_1_Servidor);
  
  String resultStg3 = "Dif I: ";     // empty string
  resultStg3.concat(A_2_ClienteB);

  display.clear();
  /*
  // Mostra o valor da diferença superior
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(63, 13, resultStg);
  // Mostra o valor da diferença inferior
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(63, 30, resultStg3);
  // Mostra o valor do paquimetro da Esp do Servidor
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(63, 47, resultStg2);
  // Barra que mostra o valor da diferença superior
  display.setFont(ArialMT_Plain_16);

  */
  for(int h=0 ; h<4;h++){
    display.drawLine(64, h, map(A_2_ClienteB*100, 230 , 370 , 0 , 128), h);
  }
  for(int h=60 ; h<64;h++){
    display.drawLine(64, h, map(resultado_superior*100, 230 , 370 , 0 , 128), h);
  }
  for(int h=0 ; h<4;h++){
    display.drawLine(h, 32, h ,map(dif_sup*100, -200 , 200 , 0 , 64));
  }
  for(int h=124 ; h<128;h++){
    display.drawLine(h, 32, h ,map(dif_inf*100, -200 , 200 , 0 , 64));
  }

    display.drawLine(map(A_2_ClienteB*100, 230 , 370 , 0 , 128), 4 ,map(resultado_superior*100, 230 , 370 , 0 , 128),60);
    display.drawLine(4,map(dif_sup*100, -200 , 200 , 0 , 64),124,map(dif_inf*100, -200 , 200 , 0 , 64));
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

  esp_now_register_recv_cb(OnDataRecv);
 
  display.init();

  pinMode(CLOCK_PIN, INPUT);
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN_SUP, INPUT);
  pinMode(DATA_PIN_SUP, INPUT);
  pinMode(controle_processo, INPUT);
  
  pinMode(led_vermelho_inf, OUTPUT);
  pinMode(led_laranja_inf, OUTPUT); 
  pinMode(led_amarelo_inf, OUTPUT); 
  pinMode(led_verde, OUTPUT); 
  pinMode(led_amarelo_sup, OUTPUT);
  pinMode(led_laranja_sup, OUTPUT); 
  pinMode(led_vermelho_sup, OUTPUT);  
  pinMode(sup_vermelho_acima, OUTPUT);
  pinMode(sup_verde, OUTPUT);
  pinMode(sup_vermelho_abaixo, OUTPUT); 
  

  digitalWrite(led_verde, HIGH);
  digitalWrite(sup_vermelho_abaixo, HIGH);
  digitalWrite(sup_vermelho_acima, HIGH);
  delay(1000);
  digitalWrite(led_verde, LOW);
  digitalWrite(sup_vermelho_abaixo, LOW);
  digitalWrite(sup_vermelho_acima, LOW);

}

void loop(){
  
  while(digitalRead(controle_processo) == HIGH) {
    Paquimetro();
    Paquimetro_Superior();
    dif_sup = ((A_1_Servidor - A_2_Cliente)/2)*10;
    dif_inf = ((B_1_Cliente - B_2_Cliente)/2)*10;
    dif_inf = (dif_inf/10); // Dividido por 10 
    dif_sup = (dif_sup/10); // Dividido por 10
    Leds();
    Leds_superior();
    Tela();
    
    
    dif_inf_aux = 0;
    dif_sup_aux = 0;
  }
  while(digitalRead(controle_processo)==LOW) {
    Paquimetro();
    Paquimetro_Superior();
    digitalWrite(led_vermelho_inf, LOW);
    digitalWrite(led_laranja_inf, LOW);
    digitalWrite(led_amarelo_inf, LOW);
    digitalWrite(led_verde, LOW);
    digitalWrite(led_amarelo_sup, LOW);
    digitalWrite(led_laranja_sup, LOW);
    digitalWrite(led_vermelho_sup, LOW);
    digitalWrite(sup_vermelho_acima, LOW);
    digitalWrite(sup_verde, LOW);  
    digitalWrite(sup_vermelho_abaixo, LOW);
    if (dif_inf_aux == 0 and dif_sup_aux == 0){
      Serial.println("------------------");
      Serial.print("Diferença Superior: ");
      Serial.println(dif_sup);
      Serial.print("Diferença Inferior: ");
      Serial.println(dif_inf);
      Serial.println("------------------");
      dif_inf_aux = 1;
      dif_sup_aux = 1;
    }
    Tela();
  }
}
