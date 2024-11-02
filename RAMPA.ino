#include <I2Cdev.h>
#include <MPU6050.h>

const int IN1 = 22;  // Motor A
const int IN2 = 24;  // Motor A
const int IN3 = 26;  // Motor B
const int IN4 = 28;  // Motor B
const int ENA = 2;   // PWM
const int ENB = 3;   // PWM

int estadoAnterior = 0; // 0: Estable, 1: Avanzar, -1: Atras

bool enRampa = false; // Variable para verificar si el robot está en la rampa


MPU6050 sensor;
int ax, ay, az;
int gx, gy, gz;

long tiempo_prev;
float dt;
float ang_y;
float ang_y_prev;
float ang_y_offset = 0.0;
bool calibrado = false;

// Variables PID
float Kp = 8.0, Ki = 0.2, Kd = 1.0;
float error, prevError = 0, integral = 0, derivada;
float anguloObjetivo = 0;  // Ángulo objetivo (0 significa horizontal)
int velocidadBaseF = 80;   // Velocidad base de los motores
int velocidadBaseB = 130;

void setup() {
  Serial.begin(9600);    // Iniciando puerto serial
  Wire.begin();           // Iniciando I2C  
  sensor.initialize();    // Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}
void actualizarAngulo() {
  // Leer las velocidades angulares
  sensor.getRotation(&gx, &gy, &gz);
  
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  
  // Calcular ángulo en Y
  ang_y = ang_y_prev + (gy / 131) * dt;

  // Calibrar la posición inicial en Y
  if (!calibrado) {
    ang_y_offset = ang_y; // Fijar el ángulo inicial
    calibrado = true;
  }

  // Aplicar el offset para que el valor inicial sea cero
  ang_y -= ang_y_offset;
  ang_y_prev = ang_y;
}

// Función para detener el movimiento
void detener() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void loop() {
  // Actualizar el ángulo antes de aplicar el control PID
  actualizarAngulo();

  // Verificar si el robot debe avanzar hacia la rampa
  if (!enRampa) {
    // Avanzar hasta que se detecte que el robot ha llegado a la rampa
    if (ang_y >= -5) { // Cambia 5 por el valor que creas que indica que está en la rampa
      // Control de motores para avanzar
      digitalWrite(IN1, HIGH); // Motor A hacia adelante
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); // Motor B hacia adelante
      digitalWrite(IN4, LOW);
      
      // Establecer velocidad base
      analogWrite(ENA, velocidadBaseF);
      analogWrite(ENB, velocidadBaseF);

      // Monitor serial para depuración
      Serial.println("Avanzando hacia la rampa...");
    } else {
      // Si el ángulo es mayor a 5, significa que el robot ha llegado a la rampa
      enRampa = true; // Cambiar el estado a enRampa
      Serial.println("Robot ha llegado a la rampa.");
    }
  } else {
    // Calcular error y aplicar control PID después de alcanzar la rampa
    error = 0.0 - ang_y; // Queremos mantener ang_y en 0 (nivelado)
    integral += error * dt;
    derivada = (error - prevError) / dt;
    prevError = error;
    
    // Calcular ajuste con PID
    int ajuste = Kp * error + Ki * integral + Kd * derivada;

    // Control de motores para estabilizar la inclinación
    if (error < -2) { // Inclinado hacia adelante
      int velocidadIzquierda = velocidadBaseB + ajuste;
      int velocidadDerecha = velocidadBaseB + ajuste;
      velocidadIzquierda = constrain(velocidadIzquierda, 0, 255);
      velocidadDerecha = constrain(velocidadDerecha, 0, 255);
      Serial.println("Atras");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

    } else if (error > 2) { // Inclinado hacia atras
      int velocidadIzquierda = velocidadBaseF + ajuste;
      int velocidadDerecha = velocidadBaseF + ajuste;
      velocidadIzquierda = constrain(velocidadIzquierda, 0, 255);
      velocidadDerecha = constrain(velocidadDerecha, 0, 255);
      Serial.println("Avanzar");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else { // Estable
      if (estadoAnterior != 0) { // Verificar si no hemos estado en "Estable" antes
        Serial.println("Estable");
        estadoAnterior = 0; // Actualizar estado
      }
      detener();
    }
    
    // Aplicar las velocidades calculadas
    int velocidadIzquierda = velocidadBaseB + ajuste;
      int velocidadDerecha = velocidadBaseB + ajuste;
    analogWrite(ENA, velocidadIzquierda);
    analogWrite(ENB, velocidadDerecha);

    // Monitor serial para depuración
    Serial.print("Inclinación en Y: ");
    Serial.print(ang_y);
    Serial.print("\tError: ");
    Serial.print(error);
    Serial.print("\tAjuste: ");
    Serial.println(ajuste);
  }

  delay(10);
}
