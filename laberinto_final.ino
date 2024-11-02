#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <LiquidCrystal_I2C.h>

// Se definen todas las constantes y los pines

// Motores con el puente H 
// Se pueden cambiar los pines
const int IN1 = 22;  // Motor A
const int IN2 = 24;  // Motor A
const int IN3 = 26;  // Motor B
const int IN4 = 28;  // Motor B
const int ENA = 2; // PWM
const int ENB = 3; // PWM

int movimiento = 0;

// Sensor ultrasonico enfrente
const int trig_frente = 18;
const int echo_frente = 19; 
// Sensor ultrasonico izquierdo
const int trig_izq = 16;
const int echo_izq = 17; 
// Sensor ultrasonico derecho 
const int trig_der = 14;
const int echo_der = 15; 

// Sensor de color 
//Crear el objeto para el sensor de color
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Crear objeto para la LCD 
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Arreglo para almacenar los movimientos
const int MAX_MOVIMIENTOS = 100;  // Se puede ajustar según el tamaño máximo de movimientos esperados
int movimientos[MAX_MOVIMIENTOS];
int indiceMovimientos = 0;

MPU6050 sensor;
int ax, ay, az;
int gx, gy, gz;

long tiempo_prev;
float dt;

float ang_z;
float ang_z_prev;
float ang_z_offset = 0.0;
bool calibrado = false;

float ang_y;
float ang_y_prev;
float ang_y_offset = 0.0;
bool calibrado_y = false;

// Variables PID
float Kp = 4.0, Ki = 0.4, Kd = 0.9;
float error, prevError = 0, integral = 0, derivada;
float anguloObjetivo = 0;
int velocidadBaseF = 85;   // Velocidad base para avanzar
int velocidadBaseB = 100;  // Velocidad base para retroceder

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
  //Iniciar la LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  Serial.begin(9600);

  // Giroscopio
  Wire.begin();
  sensor.initialize();    // Iniciando el sensor  
}

void registrarMovimiento(int mov) {
  if (indiceMovimientos < MAX_MOVIMIENTOS) {
    movimientos[indiceMovimientos] = mov;
    indiceMovimientos++;
  }
}

void actualizarAngulo() {
  // Leer las velocidades angulares
  sensor.getRotation(&gx, &gy, &gz);
  
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  
  // Calcular ángulo en Z
  ang_z = ang_z_prev + (gz / 131) * dt;

  // Calibrar la posición inicial en Z
  if (!calibrado) {
    ang_z_offset = ang_z; // Fijar el ángulo inicial
    calibrado = true;
  }

  // Aplicar el offset para que el valor inicial sea cero
  ang_z -= ang_z_offset;
  ang_z_prev = ang_z;
}

void actualizarAngulo_y() {
  sensor.getRotation(&gx, &gy, &gz);
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  
  ang_y = ang_y_prev + (gy / 131) * dt;

  if (!calibrado_y) {
    ang_y_offset = ang_y;
    calibrado_y = true;
  }
  
  ang_y -= ang_y_offset;
  ang_y_prev = ang_y;
}

void detener() {
  digitalWrite(IN1, LOW);   // Detener ambos motores
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Funciones de movimiento 

void avanzar() {
  movimiento = 1;
  registrarMovimiento(movimiento);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 165);  // Ajustar valores para la velocidad
  analogWrite(ENB, 165);
  delay(400);  // Ajustar la duración
  detener();
}

void retroceder() {
  movimiento = 2;
  registrarMovimiento(movimiento);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 120);  // Ajustar valores para la velocidad
  analogWrite(ENB, 120);
  delay(500);  // Ajustar la duración
  detener();
}

void girarDerecha(float grados) {
  movimiento = 3;
  registrarMovimiento(movimiento);
  actualizarAngulo();  // Actualizar el ángulo inicial
  float angulo_inicio = ang_z;

  // Mover el motor hasta alcanzar el ángulo deseado
  while ((angulo_inicio - ang_z) < grados) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 160);
    analogWrite(ENB, 160);
    
    actualizarAngulo();
  }
  Serial.println("Ya termino el giro der");
  detener();
}

void girarIzquierda(float grados) {
  movimiento = 4;
  registrarMovimiento(movimiento);
  actualizarAngulo();  // Actualizar el ángulo inicial
  float angulo_inicio = ang_z;

  // Mover el motor hasta alcanzar el ángulo deseado
  while ((ang_z - angulo_inicio) < grados) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 160);
    analogWrite(ENB, 160);
    actualizarAngulo();
  }
  Serial.println("Ya termino el giro izq");
  detener();
}

void giro180() {
  actualizarAngulo();  // Actualizar el ángulo inicial
  float angulo_inicio = ang_z;

  // Calcular el nuevo ángulo deseado
  float angulo_deseado = angulo_inicio + 170; 

  // Normalizar el ángulo deseado si supera 360 grados
  if (angulo_deseado >= 360) {
    angulo_deseado -= 360; 
  }

  // Girar a la derecha
  while (ang_z < angulo_deseado) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 160);
    analogWrite(ENB, 160);
    
    actualizarAngulo();
  }

  // Asegúrate de que se detenga al alcanzar el ángulo deseado
  detener();
}


void regresarAlInicio() {
  for (int i = indiceMovimientos - 1; i >= 0; i--) {
    switch (movimientos[i]) {
      case 1:
        retroceder();
        delay(1000); // Si avanzó, ahora retrocede
        break;
      case 2:
        avanzar();
        delay(1000); // Si retrocedió, ahora avanza
        break;
      case 3:
        girarIzquierda(73);
        delay(1000); // Si giró a la derecha, ahora gira a la izquierda
        break;
      case 4:
        girarDerecha(71);
        delay(1000); // Si giró a la izquierda, ahora gira a la derecha
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
  delay(20);
}

bool paredIzquierda (){
  int distanciaUmbralIzq = 20;
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
  delay(20);
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
  Serial.print(distancia);
  return (distancia <= distanciaUmbralDer);

  delay(20);

}

int detectarColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Normalizar valores de color
  float red = r / (float)c * 255;
  float green = g / (float)c * 255;
  float blue = b / (float)c * 255;

  //Mostrar valores de RGB en el Serial Monitor
  Serial.print("R: "); Serial.print(red);
  Serial.print(" G: "); Serial.print(green);
  Serial.print(" B: "); Serial.println(blue);


  // Detectar el color rangos aproximados de RGB
  if (red < 157 && green < 143 && blue < 112) {
    lcd.clear();
    lcd.print("Negro");
    return 4;
  } 
  else if (red < 187 && green < 175 && blue < 144) {
   lcd.clear();
   lcd.print("Amarillo");
    return 3;}
  else if (red < 194 && blue < 198 && green < 194) {
    lcd.clear();
    lcd.print("Morado");
    return 2;
  } 
  else if (red < 215 && blue < 193 && green < 185) {
    lcd.clear();
    lcd.print("Rosa");
    return 1;
  } 
  else if (red < 228 && green < 194 && blue < 188) {
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
  Serial.println("Laberinto");
  while (Final == false){
    bool paredIzq = paredIzquierda();
    bool paredDer = paredDerecha();
    bool paredFre = paredFrente();
    // Serial.print("Pared Izquierda: "); Serial.println(paredIzq);
    //     Serial.print("Pared Derecha: "); Serial.println(paredDer);
    //     Serial.print("Pared Frente: "); Serial.println(paredFre);
    bool casillaFrente = false;
    bool casillaIzquierda = false;
    bool casillaDerecha = false;
    int color = detectarColor();
    if ((paredIzq == false) && casillaIzquierda == false){
      Serial.println("Izquierda");
      girarIzquierda(73);
      delay(1000);
      avanzar();
      delay(1000);
      casillaFrente = false;
      casillaIzquierda = false;
      casillaDerecha = false;
      
      if(color == 4){
        retroceder();
        delay(1000);
        girarDerecha(71);
        delay(1000);
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
      }
      else if(color == 5){
        detener();
        delay(1000);
        Final = true;
      }
    }
    else if (paredFre == false && casillaFrente == false){
      Serial.println("Avanzar");
      avanzar();
      delay(1000);
      casillaFrente = false;
      casillaIzquierda = false;
      casillaDerecha = false;
      if(color == 4){
        retroceder();
        delay(1000);
        casillaFrente = true;
      }
      else if(color == 1){
        contRosa++;
        lcd.print("Rosa");
      }
      else if(color == 2){
        contMorado++;
        lcd.print("Morado");
      }
      else if(color == 3){
        contAmarillo++;
        lcd.print("Amarillo");
      }
      else if(color == 5){
        detener();
        delay(1000);
        Final = true;
      }
    }
    else if(paredDer == false && casillaDerecha == false){
      Serial.println("Derecha");
      girarDerecha(71.0);
      delay(1000);
      //Serial.println("ya giro");
      avanzar();
      //Serial.print("ya avanzo");
      delay(1000);
      casillaFrente = false;
      casillaIzquierda = false;
      casillaDerecha = false;
      if(color == 4){
        retroceder();
        delay(1000);
        girarIzquierda(73);
        delay(1000);
        casillaDerecha = true;
      }
      else if(color == 1){
        contRosa++;
        lcd.print("Rosa");
      }
      else if(color == 2){
        contMorado++;
        lcd.print("Morado");
      }
      else if(color == 3){
        contAmarillo++;
        lcd.print("Amarillo");
      }
      else if(color == 5){
        detener();
        delay(1000);
        Final = true;
      }
    }
    else if(paredFre == true && paredIzq == true && paredDer == true){
      Serial.println("180 grados");
      retroceder();
      delay(100);
    }
  } 
  //While end laberinto
  
  
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
  delay(1000);
  regresarAlInicio();
  detener();
}


void loop(){
  loopLaberinto(); 

  int finRampa = 0;
  int estadoAnterior =0;
  unsigned long tiempoEstable = 0; // Para medir estabilidad en terreno plano
  const int tiempoEnPlano = 1000;  // Tiempo requerido en ms para confirmar que dejó la rampa
  bool enRampa = false;


  while (finRampa == 0){
    actualizarAngulo_y();
    if (!enRampa && finRampa == 0) {
      if (ang_y >= -5) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, velocidadBaseF);
        analogWrite(ENB, velocidadBaseF);
        Serial.println("Avanzando hacia la rampa...");
      } else {
        enRampa = true;
        Serial.println("Robot ha llegado a la rampa.");
      }
    } else {
      velocidadBaseF = 60;
      velocidadBaseB = 90;
      error = anguloObjetivo - ang_y;
      integral += error * dt;
      derivada = (error - prevError) / dt;
      prevError = error;

      int ajuste = Kp * error + Ki * integral + Kd * derivada;

      // if (error < -2) {
      //   int velocidadIzquierda = velocidadBaseB - ajuste;
      //   int velocidadDerecha = velocidadBaseB - ajuste;
      //   velocidadIzquierda = constrain(velocidadIzquierda, 0, 220);
      //   velocidadDerecha = constrain(velocidadDerecha, 0, 220);
      //   Serial.println("Retrocediendo");
      //   digitalWrite(IN1, LOW);
      //   digitalWrite(IN2, HIGH);
      //   digitalWrite(IN3, LOW);
      //   digitalWrite(IN4, HIGH);

      //   analogWrite(ENA, velocidadIzquierda);
      //   analogWrite(ENB, velocidadDerecha);
      // } 
      if (error > 2) {
        int velocidadIzquierda = velocidadBaseF + ajuste;
        int velocidadDerecha = velocidadBaseF + ajuste;
        velocidadIzquierda = constrain(velocidadIzquierda, 0, 210);
        velocidadDerecha = constrain(velocidadDerecha, 0, 210);
        Serial.println("Avanzando");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        analogWrite(ENA, velocidadIzquierda);
        analogWrite(ENB, velocidadDerecha);
        enRampa=true;
        
      } else {
        if (estadoAnterior != 0) {
          Serial.println("Estable");
          estadoAnterior = 0;
        }
        detener();
      }
      

      Serial.print("Inclinación en Y: ");
      Serial.print(ang_y);
      Serial.print("\tError: ");
      Serial.print(error);
      Serial.print("\tAjuste: ");
      Serial.println(ajuste);
    }

    // Monitoreo para detectar si ha dejado la rampa
    if (abs(ang_y) <= 5) { // Si `ang_y` está en un rango cercano a nivel
      if (tiempoEstable == 0) tiempoEstable = millis();  // Iniciar temporizador
      else if (millis() - tiempoEstable >= tiempoEnPlano) {  // Si ha estado estable por 1 segundo
        finRampa = 1;
        Serial.println("Robot ha dejado la rampa.");
      }
    } else {
      tiempoEstable = 0;  // Reiniciar si se inclina de nuevo
    }

    delay(20);

  } // end while rampa
 Serial.println("detenido");
  //delay(20);
  //Serial.println("loop2");
  //delay(20);
 // detectarColor();
}