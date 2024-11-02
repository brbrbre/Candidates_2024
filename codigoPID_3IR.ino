// Pines de los sensores
const int sensorIzquierdo = 50;  // Sensor HW-570 izquierdo
const int sensorCentro = 52;     // Sensor FC-51 centro
const int sensorDerecho = 53;    // Sensor HW-570 derecho

// Pines del motor driver
const int motorIzquierdoA = 22;   // IN1
const int motorIzquierdoB = 24;   // IN2
const int motorDerechoA = 26;     // IN3
const int motorDerechoB = 28;     // IN4
const int enableIzquierdo = 2;   // ENA
const int enableDerecho = 3;    // ENB

// Variables del PID
float Kp = 15.0;    // Ganancia proporcional
float Ki = 0.0;     // Ganancia integral
float Kd = 3.0;     // Ganancia derivativa
int velocidadBase = 150;  // Velocidad base del robot
float error = 0, prevError = 0, integral = 0, derivada = 0;
int ajuste = 0;  // Ajuste para la velocidad

void setup() {
  pinMode(sensorIzquierdo, INPUT);
  pinMode(sensorCentro, INPUT);
  pinMode(sensorDerecho, INPUT);

  pinMode(motorIzquierdoA, OUTPUT);
  pinMode(motorIzquierdoB, OUTPUT);
  pinMode(motorDerechoA, OUTPUT);
  pinMode(motorDerechoB, OUTPUT);
  pinMode(enableIzquierdo, OUTPUT);
  pinMode(enableDerecho, OUTPUT);

  Serial.begin(9600);  // Iniciar comunicación serial para depuración
}

void loop() {
  // Leer el estado de los sensores y asignar posición
  int estadoIzquierdo = digitalRead(sensorIzquierdo);
  int estadoCentro = digitalRead(sensorCentro);
  int estadoDerecho = digitalRead(sensorDerecho);

  // Determinar la posición actual de la línea (-1, 0, 1)
  int posicion = 0;
  if (estadoIzquierdo == LOW) posicion = -1;
  if (estadoDerecho == LOW) posicion = 1;
  if (estadoCentro == LOW) posicion = 0;

  // Calcular el error
  error = posicion;

  // Calcular términos PID
  integral += error;                  // Acumulado de error
  derivada = error - prevError;       // Cambio en el error
  prevError = error;                  // Actualizar error anterior

  // Calcular ajuste con PID
  ajuste = Kp * error + Ki * integral + Kd * derivada;

  // Ajustar la velocidad de cada motor
  int velocidadIzquierda = velocidadBase - ajuste;
  int velocidadDerecha = velocidadBase + ajuste;

  // Restringir velocidades entre 0 y 255
  velocidadIzquierda = constrain(velocidadIzquierda, 0, 255);
  velocidadDerecha = constrain(velocidadDerecha, 0, 255);

  // Controlar los motores según los valores de velocidad
  analogWrite(enableIzquierdo, velocidadIzquierda);
  analogWrite(enableDerecho, velocidadDerecha);
  digitalWrite(motorIzquierdoA, HIGH);
  digitalWrite(motorIzquierdoB, LOW);
  digitalWrite(motorDerechoA, HIGH);
  digitalWrite(motorDerechoB, LOW);

  // Mostrar los valores en el monitor serial para ajuste de PID
  Serial.print("Error: "); Serial.print(error);
  Serial.print(" Ajuste: "); Serial.print(ajuste);
  Serial.print(" Velocidad Izq: "); Serial.print(velocidadIzquierda);
  Serial.print(" Velocidad Der: "); Serial.println(velocidadDerecha);

  delay(50);  // Pequeña pausa para estabilidad
}

// Función para detener el robot
void detener() {
  analogWrite(enableIzquierdo, 0);
  analogWrite(enableDerecho, 0);
  digitalWrite(motorIzquierdoA, LOW);
  digitalWrite(motorIzquierdoB, LOW);
  digitalWrite(motorDerechoA, LOW);
  digitalWrite(motorDerechoB, LOW);
}
