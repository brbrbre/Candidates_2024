

// Pines de los sensores
const int sensorIzquierdo = 10;   // Sensor HW-570 izquierdo
const int sensorCentro = 8;     // Sensor FC-51 centro
const int sensorDerecho = 9;     // Sensor HW-570 derecho

// Pines del motor driver
const int motorIzqAdelante = 22; // IN1
const int motorIzqAtras = 24;    // IN2
const int motorDerAdelante = 26; // IN3
const int motorDerAtras = 28;    // IN4
const int enableMotorIzq = 2;    // ENA
const int enableMotorDer = 3;    // ENB

// Velocidad de los motores (0 - 255)
const int velocidadMotor = 100;  // Ajusta este valor para cambiar la velocidad
const int duracionPaso = 200;    // Duración de cada paso en milisegundos

void setup() {
  // Configurar pines de sensores como entrada
  pinMode(sensorIzquierdo, INPUT);
  pinMode(sensorCentro, INPUT);
  pinMode(sensorDerecho, INPUT);
  
  // Configurar pines del motor driver como salida
  pinMode(motorIzqAdelante, OUTPUT);
  pinMode(motorIzqAtras, OUTPUT);
  pinMode(motorDerAdelante, OUTPUT);
  pinMode(motorDerAtras, OUTPUT);
  pinMode(enableMotorIzq, OUTPUT);
  pinMode(enableMotorDer, OUTPUT);
  
  Serial.begin(9600);  // Iniciar comunicación serial para depuración
}

void loop() {
  // Leer el estado de los sensores
  int estadoIzquierdo = digitalRead(sensorIzquierdo);
  int estadoCentro = digitalRead(sensorCentro);
  int estadoDerecho = digitalRead(sensorDerecho);

  // Verificar cada combinación de sensores y ejecutar la acción correspondiente
  if (estadoIzquierdo == LOW && estadoCentro == LOW && estadoDerecho == LOW) {
    avanzar();
    delay(80);
    detener();
    delay(100);
    Serial.println("Avanzando: los tres sensores detectan blanco.");
  } else if (estadoIzquierdo == LOW && estadoCentro == LOW && estadoDerecho == HIGH) {
    girarDerecha();
    delay(60);
    detener();
    delay(100);
    Serial.println("Girando a la derecha: solo el sensor derecho detecta negro.");
  } else if (estadoIzquierdo == LOW && estadoCentro == HIGH && estadoDerecho == LOW) {
    avanzar();
    delay(80);
    detener();
    delay(100);
    Serial.println("Avanzando: solo el sensor central detecta negro.");
  } else if (estadoIzquierdo == LOW && estadoCentro == HIGH && estadoDerecho == HIGH) {
    girarDerecha();
    delay(60);
    detener();
    delay(100);
    Serial.println("Girando a la derecha: sensores central y derecho detectan negro.");
  } else if (estadoIzquierdo == HIGH && estadoCentro == LOW && estadoDerecho == LOW) {
    girarIzquierda();
    delay(60);
    detener();
    delay(100);
    Serial.println("Girando a la izquierda: solo el sensor izquierdo detecta negro.");
  } else if (estadoIzquierdo == LOW && estadoCentro == HIGH && estadoDerecho == HIGH) {
    girarDerecha();
    delay(60);
    detener();
    delay(100);
    Serial.println("Girando a la derecha: solo el sensor central es blanco.");
  } else if (estadoIzquierdo == HIGH && estadoCentro == HIGH && estadoDerecho == LOW) {
    girarIzquierda();
    delay(60);
    detener();
    delay(100);
    Serial.println("Girando a la izquierda: sensores central e izquierdo detectan negro.");
  } else if (estadoIzquierdo == HIGH && estadoCentro == HIGH && estadoDerecho == HIGH) {
    avanzar();
    delay(80);
    detener();
    delay(100);
    Serial.println("Avanzando: los tres sensores detectan negro.");
  } else if (estadoIzquierdo == HIGH && estadoCentro == LOW && estadoDerecho == HIGH) {
    girarDerecha();
    delay(60);
    detener();
    delay(100);
    Serial.println("Girando a la derecha: sensores izquierdo y derecho detectan negro.");
  } else {
    detener();
    Serial.println("Sin coincidencia: el robot se detiene.");
  }

  delay(30);  // Pausa para crear el efecto de "paso a paso"
  detener();            // Detenerse al final de cada paso
}

// Función para hacer que el robot avance
void avanzar() {
  analogWrite(enableMotorIzq, velocidadMotor);
  analogWrite(enableMotorDer, velocidadMotor);
  digitalWrite(motorIzqAdelante, HIGH);
  digitalWrite(motorIzqAtras, LOW);
  digitalWrite(motorDerAdelante, HIGH);
  digitalWrite(motorDerAtras, LOW);
}

// Función para detener el robot
void detener() {
  analogWrite(enableMotorIzq, 0);
  analogWrite(enableMotorDer, 0);
}

// Función para girar a la derecha
void girarDerecha() {
  analogWrite(enableMotorIzq, 130);
  analogWrite(enableMotorDer, 180);
  digitalWrite(motorIzqAdelante, HIGH);
  digitalWrite(motorIzqAtras, LOW);
  digitalWrite(motorDerAdelante, LOW);
  digitalWrite(motorDerAtras, HIGH);
}

// Función para girar a la izquierda
void girarIzquierda() {
  analogWrite(enableMotorIzq, 180);
  analogWrite(enableMotorDer, 130);
  digitalWrite(motorIzqAdelante, LOW);
  digitalWrite(motorIzqAtras, HIGH);
  digitalWrite(motorDerAdelante, HIGH);
  digitalWrite(motorDerAtras, LOW);
}
