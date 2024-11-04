#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <MPU6050.h>
#include <I2Cdev.h>

// Motores con el puente H 
// Se pueden cambiar los pines
const int IN1 = 22;  // Motor A
const int IN2 = 24;  // Motor A
const int IN3 = 26;  // Motor B
const int IN4 = 28;  // Motor B
const int ENA = 2; // PWM
const int ENB = 3; // PWM

// Sensor ultrasonico enfrente
const int trig_frente = 18;
const int echo_frente = 19; 
// Sensor ultrasonico izquierdo
const int trig_izq = 16;
const int echo_izq = 17; 
// Sensor ultrasonico derecho 
const int trig_der = 14;
const int echo_der = 15; 


// Servomotores para la pala
Servo servo1;  
Servo servo2;

// Giroscopio
MPU6050 sensor;
int ax, ay, az;
int gx, gy, gz;

long tiempo_prev;
float dt;
float ang_z;
float ang_z_prev;
float ang_z_offset = 0.0;
bool calibrado = false;

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

  // Giroscopio
  Serial.begin(9600);
  Wire.begin();
  sensor.initialize(); 
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


// Funciones de movimiento 
void avanzar() {
  digitalWrite(IN1, HIGH);  // Motor A hacia adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Motor B hacia adelante
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);  // Ajustar valores para la velocidad
  analogWrite(ENB, 150);
  delay(450);  // Ajustar la duración
  detener();
}

void avanzar_media() {
  digitalWrite(IN1, HIGH);  // Motor A hacia adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Motor B hacia adelante
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);  // Ajustar valores para la velocidad
  analogWrite(ENB, 150);
  delay(200);  // Ajustar la duración
  detener();
}

void retroceder() {
  digitalWrite(IN1, LOW);   // Motor A hacia atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);   // Motor B hacia atrás
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 120);  // Ajustar valores para la velocidad
  analogWrite(ENB, 120);
  delay(500); 
  detener();
}

void girarDerecha(float grados) {
  actualizarAngulo();  // Actualizar el ángulo inicial
  float angulo_inicio = ang_z;

  // Mover el motor hasta alcanzar el ángulo deseado
  while ((angulo_inicio - ang_z) < grados) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    
    actualizarAngulo();
  }
  Serial.println("Ya termino el giro der");
  detener();
}

void girarIzquierda(float grados) {
  actualizarAngulo();  // Actualizar el ángulo inicial
  float angulo_inicio = ang_z;

  // Mover el motor hasta alcanzar el ángulo deseado
  while ((ang_z - angulo_inicio) < grados) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    actualizarAngulo();
  }
  Serial.println("Ya termino el giro izq");
  detener();
}

void detener() {
  digitalWrite(IN1, LOW);   // Detener ambos motores
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
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
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    
    actualizarAngulo();
    //Serial.print("Giro a la derecha. Angulo Z: ");
    //Serial.println(ang_z);
  }
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

void agarrarPelota(){
  servo1.write(180);
  servo2.write(180);
  delay(1000);  
  while(true);  
}

void loopPistaA(){
  bool casillaPelota = false;
  bool paredFre = paredFrente();
  bool paredIzq = paredIzquierda();
  bool paredDer = paredDerecha();
  int casillaEntrada = 0;
  while(casillaPelota == false){
    avanzar();
    Serial.println("Avanza");
    delay(1500);
    paredFre = paredFrente();
    paredIzq = paredIzquierda();
    paredDer = paredDerecha();
    if(paredFre == false){
      avanzar();
      Serial.println("Avanza");
      delay(1500);
      Serial.println("Meta1");
      casillaEntrada = 1;
      casillaPelota = true;
    }
      else{
        girarDerecha(73);
        delay(1500);
        avanzar();
        Serial.println("Avanzar");
        delay(1500);
        girarIzquierda(73);
        delay(1500);
        avanzar();
        if (paredIzq == false){
          girarIzquierda(73);
          Serial.println("Girar");
          delay(1500);
          avanzar();
          Serial.println("Avanzar");
          delay(1500);
          //agarrarPelota();
          casillaEntrada = 4;
          casillaPelota = true;
        }
        else {
          avanzar();
          delay(1500);
          girarIzquierda(73);
          delay(1500);
          avanzar();
          delay(1500);
          if(paredIzq == false){
            girarIzquierda(73);
            delay(1500);
            avanzar();
            delay(1500);
            //agarrarPelota();
            casillaEntrada = 3;
            casillaPelota = true;
          }
        else{
          avanzar();
          delay(1500);
          girarIzquierda(73);
          delay(1500);
          avanzar();
          Serial.println("Avanzar");
          delay(1500);
          if (paredIzq == false){
            girarIzquierda(73);
            delay(1500);
            avanzar();
            delay(1500);
            //agarrarPelota();
            casillaEntrada = 2;
            casillaPelota = true;
          }
        }
      }
    }
  }

  if(casillaEntrada == 1){
    Serial.println("Buscando meta 1");
    retroceder();
    delay(1500);
    girarIzquierda(73);
    delay(1500);
    avanzar();
    delay(1500);
    girarDerecha(73);
    delay(1500);
    avanzar(); // hacer funcion de avanzar dos casillas
    delay(1500);
    avanzar();
    delay(1500);
    girarDerecha(73);
    delay(1500);
    avanzar();
    delay(1500);
    girarIzquierda(73);
    delay(1500);
  }
  else if(casillaEntrada == 2){
     Serial.println("Buscando meta 2");
    retroceder();
    delay(1500);
    girarIzquierda(73);
    delay(1500);
    avanzar();
    delay(1500);
    girarDerecha(73);
    delay(1500);
    avanzar();
    delay(1500);
    girarIzquierda(73);
    delay(1500);
  }
  else if(casillaEntrada == 3){
    Serial.println("Buscando meta 3");
    retroceder();
    delay(1500);
    retroceder();
    delay(1500);
  }
  else if(casillaEntrada == 4){
    Serial.println("Buscando meta 4");
    retroceder();
    delay(1500);
    girarDerecha(73);
    delay(1500);
    avanzar();
    delay(1500);
    girarIzquierda(73);
    delay(1500);
    avanzar();
    delay(1500);
    girarDerecha(73);
    delay(1500);
  }

}

void loop() {
  loopPistaA();
}
