#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <LiquidCrystal_I2C.h>

// Se definen todas las constantes y los pines

// Motores con el puente H 
// Se pueden cambiar los pines
const int IN1 = 9;  // Motor A
const int IN2 = 8;  // Motor A
const int IN3 = 7;  // Motor B
const int IN4 = 6;  // Motor B
// Son para regular la velocidad en caso de que se necesite ir más rapido o más lento y deben de ir a PWM
const int ENA = 12; // PWM
const int ENB = 11; // PWM

int movimiento = 0;

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

// Arreglo para almacenar los movimientos
const int MAX_MOVIMIENTOS = 100;  // Se puede ajustar según el tamaño máximo de movimientos esperados
int movimientos[MAX_MOVIMIENTOS];
int indiceMovimientos = 0;

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
}

// Funciones de movimiento 
void avanzar() {
  movimiento = 1;
  registrarMovimiento(movimiento);
  digitalWrite(IN1, HIGH);  // Motor A hacia adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Motor B hacia adelante
  digitalWrite(IN4, LOW);
  delay(500); 
  detener();
}

void retroceder() {
  movimiento = 2;
  registrarMovimiento(movimiento);
  digitalWrite(IN1, LOW);   // Motor A hacia atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);   // Motor B hacia atrás
  digitalWrite(IN4, HIGH);
  delay(500); 
  detener();
}

void girarDerecha() {
  movimiento = 3;
  registrarMovimiento(movimiento);
  digitalWrite(IN1, HIGH);  // Motor A hacia adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Motor B hacia atrás
  digitalWrite(IN4, HIGH);
  delay(500); 
  detener();
}

void girarIzquierda() {
  movimiento = 4;
  registrarMovimiento(movimiento);
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

void bajarRampa() {
  movimiento = 1;
  digitalWrite(IN1, HIGH);  // Motor A hacia adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Motor B hacia adelante
  digitalWrite(IN4, LOW);
  analogWrite (ENA, 100); // Ajustar valores para la velocidad
  analogWrite (ENB, 100);
  delay(1500); // Ajustar valores para en cuanto tiempo baja la rampa
  detener();
}

void registrarMovimiento(int mov) {
  if (indiceMovimientos < MAX_MOVIMIENTOS) {
    movimientos[indiceMovimientos] = mov;
    indiceMovimientos++;
  }
}

void regresarAlInicio() {
  for (int i = indiceMovimientos - 1; i >= 0; i--) {
    switch (movimientos[i]) {
      case 1:
        retroceder(); // Si avanzó, ahora retrocede
        break;
      case 2:
        avanzar(); // Si retrocedió, ahora avanza
        break;
      case 3:
        girarIzquierda(); // Si giró a la derecha, ahora gira a la izquierda
        break;
      case 4:
        girarDerecha(); // Si giró a la izquierda, ahora gira a la derecha
        break;
    }
  }
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

int detectarColor() {
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

  // Detectar el color rangos aproximados de RGB
  if (red > 200 && blue > 150 && green < 100) {
    lcd.clear();
    lcd.print("Rosa");
    return 1;
  } else if (red > 100 && blue > 200 && green < 150) {
    lcd.clear();
    lcd.print("Morado");
    return 2;
  } else if (red > 200 && green > 200 && blue < 100) {
    lcd.clear();
    lcd.print("Amarillo");
    return 3;
  } else if (red < 50 && green < 50 && blue < 50) {
    lcd.clear();
    lcd.print("Negro");
    return 4;
  } else if (red > 150 && green < 100 && blue < 100) {
    lcd.clear();
    lcd.print("Color: Rojo");
    return 5;
    }
}

void loopLaberinto () {
  int contRosa = 0;
  int contMorado = 0;
  int contAmarillo = 0;
  int contNegro = 0;
  bool Final = false; 

  while (Final = false){
    bool casillaFrente = false;
    bool casillaIzquierda = false;
    bool casillaDerecha = false;
    int color = detectarColor();

    if (paredIzquierda == false && casillaIzquierda == false ){
      girarIzquierda();
      avanzar();
      casillaFrente = false;
      casillaIzquierda = false;
      casillaDerecha = false;
      if(color == 4){
        retroceder();
        girarDerecha();
        casillaIzquierda = true;
      }else if(color == 1){
        contRosa++;
      }else if(color == 2){
        contMorado++;
      }else if(color == 3){
        contAmarillo++;
      }else if(color == 5){
        detener();
        Final = true;
      }
    }
    else if (paredFrente == false && casillaFrente == false){
      avanzar();
      casillaFrente = false;
      casillaIzquierda = false;
      casillaDerecha = false;
      if(color == 4){
        retroceder();
        casillaFrente = true;
      }else if(color == 1){
        contRosa++;
      }else if(color == 2){
        contMorado++;
      }else if(color == 3){
        contAmarillo++;
      }else if(color == 5){
        detener();
        Final = true;
      }
    }
    else if(paredDerecha == false && casillaDerecha == false){
      girarDerecha();
      avanzar();
      casillaFrente = false;
      casillaIzquierda = false;
      casillaDerecha = false;
      if(color == 4){
        retroceder();
        girarIzquierda();
        casillaDerecha = true;
      }else if(color == 1){
        contRosa++;
      }else if(color == 2){
        contMorado++;
      }else if(color == 3){
        contAmarillo++;
      }else if(color == 5){
        detener();
        Final = true;
      }
    }
    else if(paredFrente == true && paredIzquierda == true && paredDerecha == true){
      retroceder();
    }
  }
  // Se podria incluir tambien en el while 
  if (contRosa > contMorado && contRosa > contAmarillo){
    lcd.clear();
    lcd.print("Rosa");
  } else if (contMorado > contRosa && contMorado > contAmarillo){
    lcd.clear();
    lcd.print("Morado");
  } else if (contAmarillo > contRosa && contAmarillo > contMorado){
    lcd.clear();
    lcd.print("Amarillo");
  }

  giro180();
  // Falta la funcion y/o algortimo para que regrese a la casilla del color mas mostrado y regrese al checkpoint
  regresarAlInicio();
  // Falta agregar lo de bajar rampa (Que gire a la derecha y vaya a maxima velocidad hacer una funcion de adelante para eso)
  bajarRampa();
}
void loop() {
  loopLaberinto();
}
