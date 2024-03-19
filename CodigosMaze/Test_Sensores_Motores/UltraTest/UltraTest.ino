//frente
int trigF = 22;
int echoF = 24;

//izquierdo Delantero
int trig_id = 3; 
int echo_id = 2;

//derecho delantero
int trig_dd = 28; 
int echo_dd = 26;

//izquierdo trasero
int trig_it = 5;
int echo_it = 4;

// derecho trasero
int trig_dt = 35;
int echo_dt = 37;

//declare variables que uso para asignarselas a cada ultrasonico
float distancia;
float distancia1; 
float distancia2;
float distancia3;
float distancia4;

//este es el procedimiento recomendado por el fabricante para leer en cm
float medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return (duration / 2.0) / 29.1;
}

void setup() {
  //inicializo el monitor en 9600 que es lo recomendable
  Serial.begin(9600);

  //declaro en los pines de cada ultrasonico
  //declaro el trig como OUTPUT o salida porque el se encarga de enviar la onda de sonido
  //el echo como input o entrada porque el recibe la onda de sonido
  pinMode(trigF, OUTPUT);
  pinMode(echoF, INPUT);

  pinMode(trig_id, OUTPUT);
  pinMode(echo_id, INPUT);

  pinMode(trig_dd, OUTPUT);
  pinMode(echo_dd, INPUT);

  pinMode(trig_it, OUTPUT);
  pinMode(echo_it, INPUT);

  pinMode(trig_dt, OUTPUT);
  pinMode(echo_dt, INPUT);
  
}

void loop() {
  //asigno la variable y sus trig y echo al pin para medir en cm
  distancia = medirDistancia(trigF, echoF);
  distancia1 = medirDistancia(trig_id, echo_id);
  distancia2 = medirDistancia(trig_dd, echo_dd);
  distancia3 = medirDistancia(trig_it, echo_it);
  distancia4 = medirDistancia(trig_dt, echo_dt);

  //todo lo imprimo en el monitor serie
   Serial.println("enfrente");
  Serial.println(distancia);
  delay(800);
  Serial.println("izquierdo delantero");
  Serial.println(distancia1);
  delay(800);
  Serial.println("derecho delantero");
  Serial.println(distancia2);
  delay(800); 

  Serial.println("izquierdo trasero");
  Serial.println(distancia3);
  delay(800);

  Serial.println("derecho trasero");
  Serial.println(distancia4);
  delay(800);

  /*if (distancia <= 10) {
    Serial.println("alto");
  }*/
}
