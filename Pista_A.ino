#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// Motores con el puente H 
// Se pueden cambiar los pines
const int IN1 = 9;  // Motor A
const int IN2 = 8;  // Motor A
const int IN3 = 7;  // Motor B
const int IN4 = 6;  // Motor B
// Son para regular la velocidad en caso de que se necesite ir más rapido o más lento y deben de ir a PWM
const int ENA = 12; // PWM
const int ENB = 11; // PWM

// Sensor ultrasonico enfrente
const int trig_frente = 2;
const int echo_frente = 3; 
// Sensor ultrasonico izquierdo
const int trig_izq = 4;
const int echo_izq = 5; 
// Sensor ultrasonico derecho 
const int trig_der = 1;
const int echo_der = 13; 

// Sensor de color 
// Crear el objeto para el sensor de color
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Crear objeto para la LCD 
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Servomotores para la pala
Servo servo1;  
Servo servo2;

void setup() {
  // Configurar los pines como salida (puente H)
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ultrasonico 
  pinMode(trig_frente, OUTPUT);
  pinMode(echo_frente, INPUT);
  pinMode(trig_izq, OUTPUT);
  pinMode(echo_izq, INPUT);
  pinMode(trig_der, OUTPUT);
  pinMode(echo_der, INPUT);

  // Sensor de color y LCD
  tcs.begin();
  // Iniciar la LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Servomotores
  servo1.attach(16);  
  servo2.attach(17);
}

// Funciones de movimiento 
void avanzar() {
  digitalWrite(IN1, HIGH);  // Motor A hacia adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Motor B hacia adelante
  digitalWrite(IN4, LOW);
  delay(500); 
  detener();
}

void retroceder() {
  digitalWrite(IN1, LOW);   // Motor A hacia atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);   // Motor B hacia atrás
  digitalWrite(IN4, HIGH);
  delay(500); 
  detener();
}

void girarDerecha() {
  digitalWrite(IN1, HIGH);  // Motor A hacia adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Motor B hacia atrás
  digitalWrite(IN4, HIGH);
  delay(500); 
  detener();
}

void girarIzquierda() {
  digitalWrite(IN1, LOW);   // Motor A hacia atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);  // Motor B hacia adelante
  digitalWrite(IN4, LOW);
  delay(500); 
  detener();
}

void detener() {
  digitalWrite(IN1, LOW);   // Detener ambos motores
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void giro180(){
  digitalWrite(IN1, LOW);   // Motor A hacia atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);  // Motor B hacia adelante
  digitalWrite(IN4, LOW);
  delay(1500); // Ajustar para cuando se de la vuelta 180 grados
  detener();
}

bool paredFrente () { // Si hay pared a menos de 15 o a 15 cm regresa true
  int distanciaUmbralFrente = 15;
  long duracion;
  long distancia;
  // Pulso ultrasónico
  digitalWrite(trig_frente, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_frente,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_frente, LOW);

  // Tiempo del echo 
  duracion = pulseIn(echo_frente, HIGH);
  // Calcula distancia 
  distancia = duracion * 0.0343 / 2;
  // Comprobar si la distancia es menor o igual al umbral
  return (distancia <= distanciaUmbralFrente);
}

bool paredIzquierda (){
  int distanciaUmbralIzq = 15;
  long duracion;
  long distancia; 
  // Pulso ultrasónico
  digitalWrite(trig_izq, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_izq,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_izq, LOW);

  // Tiempo del echo 
  duracion = pulseIn(echo_izq, HIGH);
  // Calcula distancia 
  distancia = duracion * 0.0343 / 2;
  // Comprobar si la distancia es menor o igual al umbral
  return (distancia <= distanciaUmbralIzq);
}

bool paredDerecha (){
  long duracion;
  long distancia; 
  int distanciaUmbralDer = 15;
  // Pulso ultrasónico
  digitalWrite(trig_der, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_der,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_der, LOW);

  // Tiempo del echo 
  duracion = pulseIn(echo_der, HIGH);
  // Calcula distancia 
  distancia = duracion * 0.0343 / 2;
  // Comprobar si la distancia es menor o igual al umbral
  return (distancia <= distanciaUmbralDer);
}

bool detectarColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Normalizar valores de color
  float red = r / (float)c * 255;
  float green = g / (float)c * 255;
  float blue = b / (float)c * 255;

  // Mostrar valores de RGB en el Serial Monitor
  Serial.print("R: "); Serial.print(red);
  Serial.print(" G: "); Serial.print(green);
  Serial.print(" B: "); Serial.println(blue);
  if (red > 150 && green < 100 && blue < 100) {
    lcd.clear();
    lcd.print("Color: Rojo");
    return true;
    }
}

void agarrarPelota(){
  servo1.write(180);
  servo2.write(180);
  delay(1000);  
  while(true);  
}

void loopPistaA(){
  bool casillaPelota = false;
  int casillaEntrada = 0;
  while(casillaPelota == false){
    avanzar();
    if(paredFrente == false){
      avanzar();
      casillaEntrada = 1;
      casillaPelota = true;
    }
    else {
      girarIzquierda();
      avanzar();
      girarDerecha();
      avanzar();
      if(paredDerecha == false){
        girarDerecha();
        avanzar(); // programar una funcion que avance media casilla o algo asi 
        casillaEntrada = 2;
        casillaPelota = true;
      }
      else{
        avanzar();
        girarDerecha();
        avanzar();
        if (paredDerecha == false){
          girarDerecha();
          avanzar();
          casillaEntrada = 3;
          casillaPelota = true;
        }
        else{
          avanzar();
          girarDerecha();
          avanzar();
          if (paredDerecha == false){
            girarDerecha();
            avanzar();
            casillaEntrada = 4;
            casillaPelota = true;
          }
        }
      }
    }
  }
  agarrarPelota();
  if(casillaEntrada == 1){
    retroceder();
    girarIzquierda();
    avanzar();
    girarDerecha();
    avanzar(); // hacer funcion de avanzar dos casillas
    avanzar();
    girarDerecha();
    avanzar();
    girarIzquierda();
  }
  else if(casillaEntrada == 2){
    retroceder();
    girarIzquierda();
    avanzar();
    girarDerecha();
    avanzar();
    girarIzquierda();
  }
  else if(casillaEntrada == 3){
    retroceder();
    giro180();
    avanzar();
  }
  else if(casillaEntrada == 4){
    retroceder();
    girarDerecha();
    avanzar();
    girarIzquierda();
    avanzar();
    girarDerecha();
  }

}

void loop() {
  loopPistaA();
}
