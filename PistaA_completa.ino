#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

// Motores con el puente H 
// Se pueden cambiar los pines
const int IN1 = 0;  // Motor A
const int IN2 = 1;  // Motor A
const int IN3 = 2;  // Motor B
const int IN4 = 3;  // Motor B
// Son para regular la velocidad en caso de que se necesite ir más rapido o más lento y deben de ir a PWM
const int ENA = 12; // PWM
const int ENB = 11; // PWM

// Sensor ultrasonico enfrente
const int trig_frente = 6;
const int echo_frente = 7; 
// Sensor ultrasonico izquierdo
const int trig_izq = 4;
const int echo_izq = 5; 
// Sensor ultrasonico derecho 
const int trig_der = 1;
const int echo_der = 13; 

// Servomotores para la pala
Servo servo1;  
Servo servo2;

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

  // Servomotores
  servo1.attach(16);  
  servo2.attach(17);

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

void moverIzquierda(){
  girarIzquierda();
  avanzar();
}

void moverDerecha(){
  girarDerecha();
  avanzar();
}

// Clase Casilla
class Casilla {
  public:
    String movimiento;    // Puede ser "avanzar", "mover izquierda", "mover derecha", etc.
    int puntoDeInteres;   // Número de punto de interés en la casilla
    
    // Constructor
    Casilla(String mov, int punto) : movimiento(mov), puntoDeInteres(punto) {}

    // Método para ejecutar la acción de la casilla
    void ejecutarMovimiento() {
      if (movimiento == "avanzar") {
        avanzar();
      } else if (movimiento == "mover izquierda") {
        moverIzquierda();
      } else if (movimiento == "mover derecha") {
        moverDerecha();
      } 
    }
};
// No se esta tomando en cuenta como primer paso la casilla que esta enfrente del checkpoint de inicio
Casilla recorrido_clocwise[] = {
  Casilla("mover izquierda", 0),
  Casilla("mover derecha", 2),
  Casilla("avanzar", 0),
  Casilla("mover derecha", 3),
  Casilla("avanzar", 0),
  Casilla("mover derecha",4),
  Casilla("avanzar",0),
  Casilla("mover derecha",1)
};

Casilla recorrido_anticlockwise[]={
  Casilla("avanzar",0),
  Casilla("mover izquierda",4),
  Casilla("avanzar",0),
  Casilla("mover izquierda",3),
  Casilla("avanzar",0),
  Casilla("mover izquierda",2),
  Casilla("avanzar",0),
  Casilla("mover izquierda",1)
};

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

}

void loop() {
  loopPistaA();
}
