#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>

// Se definen todas las constantes y los pines

// Motores con el puente H 
// Se pueden cambiar los pines
const int IN1 = 0;  // Motor A
const int IN2 = 1;  // Motor A
const int IN3 = 2;  // Motor B
const int IN4 = 3;  // Motor B
// Son para regular la velocidad en caso de que se necesite ir más rapido o más lento y deben de ir a PWM
const int ENA = 12; // PWM
const int ENB = 11; // PWM

int movimiento = 0;

// Sensor ultrasonico enfrente
const int trig_frente = 6;
const int echo_frente = 7; 
// Sensor ultrasonico izquierdo
const int trig_izq = 8;
const int echo_izq = 9; 
// Sensor ultrasonico derecho 
const int trig_der = 10;
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

// Giroscopio
MPU6050 mpu;  // Sin argumentos en el constructor
float currentAngle = 0;
float targetAngle = 0;
unsigned long lastTime;


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

  // Giroscopio
  Serial.begin(9600);
  Wire.begin();
  lastTime = millis();
}

// Función para obtener el ángulo actual
float getAngleChange() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);  // Obtiene los datos de rotación en los tres ejes

  // Convierte gz (rotación en el eje Z) de la escala de MPU-6050 a grados/segundo
  float degreesPerSecond = gz / 131.0;  // Escala para +/- 250 grados/segundo

  // Calcula el tiempo transcurrido en segundos
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // Convierte a segundos
  lastTime = currentTime;

  // Calcula el cambio de ángulo
  return degreesPerSecond * deltaTime;
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
  targetAngle = currentAngle + 90;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  while (currentAngle < targetAngle) {
    currentAngle += getAngleChange();
    delay(10);
  }
  detener();
}

void girarIzquierda() {
  movimiento = 4;
  registrarMovimiento(movimiento);
  targetAngle = currentAngle - 90;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  while (currentAngle > targetAngle) {
    currentAngle += getAngleChange();
    delay(10);
  }
  detener();
}

void detener() {
  digitalWrite(IN1, LOW);   // Detener ambos motores
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void giro180(){
  targetAngle = currentAngle + 180;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  while (currentAngle < targetAngle) {
    currentAngle += getAngleChange();
    delay(10);
  }
  detener();
}

//void bajarRampa() {
  //digitalWrite(IN1, HIGH);  // Motor A hacia adelante
  //digitalWrite(IN2, LOW);
  //digitalWrite(IN3, HIGH);  // Motor B hacia adelante
  //digitalWrite(IN4, LOW);
  //analogWrite (ENA, 100); // Ajustar valores para la velocidad
  //analogWrite (ENB, 100);
  //delay(1500); // Ajustar valores para en cuanto tiempo baja la rampa
  //detener();
//}

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
        lcd.print("Rosa");
      }else if(color == 2){
        contMorado++;
        lcd.print("Morado");
      }else if(color == 3){
        contAmarillo++;
        lcd.print("Amarillo");
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
        lcd.print("Rosa");
      }else if(color == 2){
        contMorado++;
        lcd.print("Morado");
      }else if(color == 3){
        contAmarillo++;
        lcd.print("Amarillo");
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
        lcd.print("Rosa");
      }else if(color == 2){
        contMorado++;
        lcd.print("Morado");
      }else if(color == 3){
        contAmarillo++;
        lcd.print("Amarillo");
      }else if(color == 5){
        detener();
        Final = true;
      }
    }
    else if(paredFrente == true && paredIzquierda == true && paredDerecha == true){
      giro180();
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
  regresarAlInicio();
  //bajarRampa();
  detener();
}
void loop() {
  loopLaberinto();
}

// comentario

// Faltante:
// Checar si se imprime los colores cuando se llega a cada casilla, y checar lo de la rampa