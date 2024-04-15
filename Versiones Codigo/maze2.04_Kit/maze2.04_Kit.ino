/*-----------------------------------
 *            LIBRERIAS            *
-----------------------------------*/
#include <Wire.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MLX90614.h>
#include <SPI.h>
#include <SD.h>
#include <Pixy2I2C.h>
#include "Adafruit_TCS34725.h"
extern "C" {
#include "utility/twi.h" // from Wire library, so we can do bus scanning
#include "SparkFunISL29125.h"
}

// objeto de la pixy 
Pixy2I2C pixy;

// objeto del RGB 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X); 

//Procediemento que selecciona el puerto en el multiplexor
#define TCAADDR 0x70
void tcaselect(uint8_t i) {
    if (i > 7) return;
        Wire.beginTransmission(TCAADDR);
        Wire.write(1 << i);
        Wire.endTransmission();
}

String gris[]={"65D","369","368"};

/*---------------------------------
             MOTORES              *
-----------------------------------*/
  Servo mid;
  Servo mit;
  Servo mdd;
  Servo mdt;

// servo del dispensandor de victimas
Servo servoVic;

VL53L0X sensor;

/* SFE_ISL29125 RGB_1;
SFE_ISL29125 RGB_2; */


/*-----------------------------------
 *            VARIABLES            *
-----------------------------------*/

String inputString = "";      // Cadena para guardar el comando recibido
bool stringComplete = false;  // Bandera boleana que nos indica cuando el comando fue recibido y podemos compararlo con los 2 comandos válidos


Servo servo_1; //no esta en uso solo en la funcion paro que tampoco se usa 
int servo_pin=7;
int vuelta=22;
int cont = vuelta;
int wait= 110;
String a = "TRUE";

/////////// pines de conexion ////////////////////

// Motores
  const int pmid = 49; //1
  const int pmit = 45; //2
  const int pmdd = 47; //3
  const int pmdt = 43; //4  

// servo del dispensador de victimas
  const int servoVicP = 31; 

// Sensores Ultrasonicos (Trigger y Echo para cada sensor)
  const int trig_it = 5; 
  const int echo_it = 4; 

  const int trig_if = 3; 
  const int echo_if = 2; 

  const int trig_ft = 22;
  const int echo_ft = 24;

  const int trig_df = 28;
  const int echo_df = 26;

  const int trig_dt = 35;
  const int echo_dt = 37;

// Temperatura infrarroja  
  Adafruit_MLX90614 tempI = Adafruit_MLX90614(); 
  Adafruit_MLX90614 tempD = Adafruit_MLX90614();

//Giroscopio  
  const int mpu_dir = 0x68;
  MPU6050 mpu(0x68);

//SD  
  File archivo;
  const uint8_t BUFFER_SIZE = 20;
  char fileName[] = "datos.txt"; // SD library only supports up to 8.3 names
  char buff[BUFFER_SIZE+2] = "";  // Added two to allow a 2 char peek for EOF state
  uint8_t index = 0;
  const uint8_t chipSelect = 53;//8;p
  const uint8_t cardDetect = 52;//6;
  enum states: uint8_t { NORMAL, E, EO };
  uint8_t state = NORMAL;
  bool alreadyBegan = false;  // SD.begin() misbehaves if not first call

/* Variables de Distancia Mínima a Paredes */
  int LL = 200;          
  int LD = 60;           
  String inercia = "FT"; 
  int margen = 2;       
  int dif = 1;           
  String mensaje = "";   

/* Variables de Duración de los Movimientos */
  int imp = 25;        
  int giros_I = 20;    // ciclos para iniciar el giro a la izquierda 
  int giros_ISF = 55;  // clclos para iniciar el giro en la inzquierda sin frente
  int giros_D = 17;    // ciclos para iniciar el giro a la derecha    
  int giros_u = 42;    // ciclos para iniciar el giro en U       
  int avance_ISF = 0;  // variable para contar el avance leve de la ISF

  int ab_serial = 0;   // define gravacion a SD 0= NO se graba  1= SI se graba 
  int ab_grabar = 1;   // define gravacion a SD 0= NO se graba  1= SI se graba 

/* Variables de Giroscopio */
  // varibles de la aceleración lineales 
    int ax = 0;       
    int ay = 0;       
    int az = 0;  

  // variables de aceleración de rotacion    
    int gx = 0;       
    int gy = 0;       
    int gz = 0;       
  
  // variables del angulo de todos los ejes    
    float ang_x = 0.0;
    float ang_y = 0.0;
    float ang_z = 0.0;

    int anguloI = 0; // angulo de inclinación  1 hay inclinación 0 no hay

  unsigned int R = 0;
  unsigned int V = 0;
  unsigned int A = 0;

/* Variables de Sensores de Piso */ 
  String color = "";           
  uint16_t Rojo_Frec, Verde_Frec, Azul_Frec, C;
  int bl;                       
  int ng;                       

/* Otras Variables */

//definir el uso de cada variable 
  int ciclo = 0;    // esta variable para que sirve                 
  String dire = "";                 
  long MIT, MIF, MFT, MDF, MDT;     
  long MMIT, MMIF, MMFT, MMDF, MMDT;
  int LIT, LIF, LFT, LDF, LDT;      
  String col_vic = "";             
  String victima = "camina"; 
  int ledv = 10;        
  Servo myServo;        
  int delay_de = 170;   
  int delay_iz = 250;   
  int i;                            

/* variables de prueba */
  String izquierdaSinF = "";
  String strImpulso = "";

  // variables para imprimir los valore de la velocidad de las llantas
    int llantaD1 = 0;
    int llantaI1 = 0;
    int llantaD2 = 0;
    int llantaI2 = 0;

  int contA = 0;          // variable para contar cuanto timpo esta en el color azul
  int contB = 0;          // variable para contar cuanto timpo esta en el color blanco
  bool lecturas =  true;  // varibla para activar y desactivar las lecturas
  int pasos_salir = 10;   // variable para salir del giro
  int cont_Salir = 0;     // variable para contar los pasos para salir

  int contVic = 0;        // varble para contar los timpos de la entrega
  int contSalirVic = 0;   // variable para reiniciar la deteccion de victimas 

//incializacion de dispositivos 
void setup(){
  Serial.begin(9600);
  Wire.begin();  
  // Configura leds de las victimas
    pinMode(8, OUTPUT);  // rojo
    pinMode(13, OUTPUT);  // amarillo
    //pinMode(10, OUTPUT); // verde

  // iniciamos el sensor RGB
    tcaselect(7); 
    if (tcs.begin()) { 
        Serial.println("Inicializando"); 
    } else {
        Serial.println("Error"); 
        Serial.println("Debe de revisar las conexiones e iniciar nuevamente");
        while (1); 
    }
  
  // sensor de la pixyMon v2
    tcaselect(6);
    pixy.init();

  // sensor del giroscopio
    tcaselect(1);
    mpu.initialize(); 

  // Sensor del giroscopio
    myServo.attach(7);
    inputString.reserve(200);
    //Serial.println("setup");
    /* pinMode(8, OUTPUT);//luce de vic */

  /*  Motores  */
    mid.attach(pmid);
    mit.attach(pmit);
    mdd.attach(pmdd);
    mdt.attach(pmdt);

  // servo del dispensador de victimas
    servoVic.attach(servoVicP);

  // sensores de distancia ultrasonicos 
    pinMode(trig_it, OUTPUT); 
    pinMode(echo_it, INPUT);
    pinMode(trig_if, OUTPUT); 
    pinMode(echo_if, INPUT);
    pinMode(trig_ft, OUTPUT); 
    pinMode(echo_ft, INPUT);
    pinMode(trig_df, OUTPUT); 
    pinMode(echo_df, INPUT);
    pinMode(trig_dt, OUTPUT); 
    pinMode(echo_dt, INPUT);
    digitalWrite(trig_it,LOW);
    digitalWrite(trig_if,LOW);
    digitalWrite(trig_ft,LOW);
    digitalWrite(trig_df,LOW);
    digitalWrite(trig_dt,LOW);
   
  //inicia la comunicación I2C
  while (!Serial);
  //delay(1000);
    
  //Serial.println("\nTCAScanner ready!");

  //busca dispositivos I2C
  for (uint8_t t=0; t<8; t++) {
    // Serial.println(t);
    tcaselect(t);
    //Serial.print("TCA Port #"); Serial.println(t);
    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;
      Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
       // Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }

  //Giroscopio  
    tcaselect(1);
    if (mpu.testConnection()) {
      Serial.println("Giroscopio inicializado correctamente");
    } else {
      Serial.println("Error al inicializar el giroscopio");
    }
  
  // SD  
  //  while (!Serial);  // Wait for serial port to connect (ATmega32U4 type PCBAs)
    pinMode(cardDetect, INPUT);
    if (!SD.begin(chipSelect)){
        //Serial.println("error al iniciar SD");
    }else{
        //Serial.println("SD conectada");
    }
    if (SD.exists("Maze.txt")){
      SD.remove("Maze.txt");
    }
    archivo = SD.open("Maze.txt",FILE_WRITE);
    archivo.println("1");
    archivo.close();

    // sensores de piso 
    //aqui no hay nada 
    /* digitalWrite(8, HIGH); */
}


//--------------------------------------------------------------------------------
//*                            FUNCIONES Y PROCESOS                             // *
//--------------------------------------------------------------------------------

  
  //***************************************************
  //* Función para controlar Servo motores            *
  //***************************************************
  void moverse(int vi, int vd) {
    //valores >90 adelante, 90 detenidos, <90 atras
    //los motores de cada lado estan contrapuestos por eso se dan valores contrarios
    //valores por default para los motores (90 detenidos) 
    int i1 = 90;
    int i2 = 90; 
    int d1 = 90;
    int d2 = 90;
    if (vi == 0) {
      //motores izquierdos detenidos 
      i1 = 90;
      i2 = 90;
    }else {
      if (vi == 1){
        //motores izquierdos adelante 
        i1 = 180;
        i2 = 0;
      }else {
        if (vi == -1) {
          //motores izquierdos atras
          i1 = 0; 
          i2 = 180;
        }else {
        /* velocidades para ajustar el robot */
          if (vi == 2) {
            //motores izquierdos adelante lento 
            mensaje = "AJ-D";
            i1 = 95;
            i2 = 85;
          } else {
            if (vi == 3) {
              i1 = 110;
              i2 = 70;
              }else{
                d1 = vi;
                d2 = vi;
              }
          }
        }
      }
    }
    //motores derechos detenidos
    if (vd == 0) {
      d1 = 90;
      d2 = 90;
    } else {
      //motores derechos adelante
      if (vd == 1) {
        d1 = 180;
        d2 = 0;
      }else {
        //motores derechos atras
        if (vd == -1) {
          d1 = 0;
          d2 = 180;
        /* velocidades para ajustar el robot */
        }else {
          //motores derechos adelante lento 
          if (vd == 2) {
            mensaje = "AJ-I";
            d1 = 95;
            d2 = 85;
          } else {
            //para que se usan estos valores??????????????????????????
            if (vd == 3) {
              d1 = 110;
              d2 = 70;
              }else{
                d1 = vd;
                d2 = vd;
              }
          }
        }
      }
    }

    //se aplican las velocidades en los motores 
    mid.write(i1); // R
    mit.write(i2); // N
    mdd.write(d1); // R
    mdt.write(d2); // N

    //valores para grabar en la SD
    llantaI1 = i1; // R
    llantaI2 = i2; // N
    llantaD1 = d1; // R
    llantaD2 = d2; // N
  }

void pixyColores() {
  if (victima=="sale"){
    if (contSalirVic <= 20){
      contSalirVic++;
    }else{
      victima = "camina";
    }
  }else if (victima=="camina" && LIT == 0 && LIF == 0 ){
    contSalirVic=0;
    int blocks;
    tcaselect(6);
    blocks = pixy.ccc.getBlocks();
    if (blocks) {
      for (int j = 0; j < blocks; j++) {
        int signature = pixy.ccc.blocks[j].m_signature;
          switch(signature) {
            case 1:
              Serial.println("Color detectado: Rojo");
              digitalWrite(8, LOW); // rojo
              digitalWrite(13, HIGH);  // amarillo
              //digitalWrite(10, LOW); // verde
              if (victima=="camina"){
                victima = "detiene";
              }
              break;
            case 2:
              Serial.println("Color detectado: Amarillo");
              digitalWrite(8, LOW);  // rojo
              digitalWrite(13, HIGH); // amarillo
              //digitalWrite(10, LOW); // verde
              if (victima=="camina"){
                victima = "detiene";
              }
              break;
            case 3:
              Serial.println("Color detectado: Verde");
              digitalWrite(8, LOW);  // rojo
              digitalWrite(13, HIGH);  // amarillo
              //digitalWrite(10, LOW); // verde
              if (victima=="camina"){
                victima = "detiene";
              }
              break;
            default:
              Serial.println("no color ");
              break;
          }
        }
      } else {
      digitalWrite(8, LOW);  // rojo
      digitalWrite(13, LOW);  // amarillo
      //digitalWrite(10, LOW); // verde
    }
  }
}

  void leerPiso(){
    //lectura sensores de piso 
    tcaselect(7); 

    tcs.getRawData(&Rojo_Frec, &Verde_Frec, &Azul_Frec, &C);

    analogWrite(11, Rojo_Frec);
    analogWrite(10, Verde_Frec);
    analogWrite(9, Azul_Frec);

    Serial.println(" Rojo = " + String(Rojo_Frec) + " Verde = " + String(Verde_Frec) + " Azul = " + String(Azul_Frec));
    delay(100);     
  
    //Obtencion del color del piso 
    if((Rojo_Frec <= 30 && Rojo_Frec >= 10) and (Verde_Frec <= 30 && Verde_Frec >= 10) and (Azul_Frec <= 30 && Azul_Frec >= 10)){
      color = "NEGRO";
    }else{
  
      if((Rojo_Frec <= 50 && Rojo_Frec >= 20) and (Verde_Frec <= 50  && Verde_Frec >= 30) and (Azul_Frec <= 50 && Azul_Frec >= 20)){
        color = "AZUL";
        if (contA <= 4){
          contA++;
          contB = 0;
        }
        
      }else{
        color = "BLANCO";
        contB++;
        if (contB == 5){
          contA = 0;
          }
        bl=1;
      }
    }
  }

  void leerGiroscopio(){
    //  Lectura de girsocopio 
    if (lecturas){
      tcaselect(1);
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      ang_x = atan2(ax, az) * (180.0 / M_PI);
      ang_y = atan2(ay, az) * (180.0 / M_PI);
      ang_z = atan2(az, ax) * (180.0 / M_PI);
      delay(10);
  
      //determina si esta subiendo algo 
      // Verificar la condición en el rango de -10 a -20
    
      if (ang_y >= -170 && ang_y <= -140) {
      anguloI = 1;
      moverse(1,1);
        
      }else if(ang_y <= 165 && ang_y >= 140){
        anguloI = 2;
        moverse(-1,-1);
    
      }else{
        anguloI = 0;
      }
    }
  }

  void leerUltrasonico(){
    if (lecturas){
      //  Distancia Ultrasonicos en cm 
      delay(5);
      //izquerda trasera
      MIT = ultra(trig_it,echo_it);
      delay(5);
      //frontal
      MFT = ultra(trig_ft,echo_ft);
      delay(5);
      //derecha trasera
      MDT = ultra(trig_dt,echo_dt);
      delay(5);
      //izquierda frontal
      MIF = ultra(trig_if,echo_if);
      delay(5);
      //derecha frontal
      MDF = ultra(trig_df,echo_df);
  
      //  Distancia Ultrasonicos en mm
      //izquerda trasera 
      MMIT = ultra_mm(trig_it,echo_it);
      delay(5);
      //frontal
      MMFT = ultra_mm(trig_ft,echo_ft);
      delay(5); 
      //derecha trasera
      MMDT = ultra_mm(trig_dt,echo_dt);
      delay(5);
      //izquierda frontal
      MMIF = ultra_mm(trig_if,echo_if);
      delay(5);
      //derecha frontal
      MMDF = ultra_mm(trig_df,echo_df);
      delay(5);
      //  Valores logicos Ultrasonicos 
      if(MMFT>LD){
        LFT=1;
      }else{
        LFT=0;
      }
      if(MMIT>LL){
        LIT=1;
      }else{
        LIT=0;
      }
      if(MMIF>LL){
        LIF=1;
      }else{
        LIF=0;
      }
      
      if(MMDF>LL){
        LDF=1;
      }else{
        LDF=0;
      }
      if(MMDT>LL){
        LDT=1;
      }else{
        LDT=0;
      }
    }
  }

  //lectura de sensores 
  void Leer(){
  
    //leerPiso();    
    //leerGiroscopio(); 
    leerUltrasonico();  
    // dectecta colores de la victima
    pixyColores();
  }

  //imprime valores al puerto serial 
  void aserie(){
    //si esta habilitado el envio de datos al puerto serial 
    if (ab_serial==1){
      Serial.print("ciclo ");Serial.println(ciclo);
      Serial.print("Distancias MM IT: ");Serial.print(MMIT);Serial.print("mm---");
      Serial.print("IF: ");Serial.print(MMIF);Serial.print("mm---");
      Serial.print("FT: ");Serial.print(MMFT);Serial.print("mm---");
      Serial.print("DF: ");Serial.print(MMDF);Serial.print("mm---");
      Serial.print("DT: ");Serial.print(MMDT);Serial.println("mm---");    
      Serial.println("--------------------------------------------------  ----------------------------------------------");
      Serial.print("Distancias MM IT: ");Serial.print(MMIT);Serial.print("mm---");
      Serial.print("IF: ");Serial.print(MMIF);Serial.print("mm---");
      Serial.print("FT: ");Serial.print(MMFT);Serial.print("mm---");
      Serial.print("DF: ");Serial.print(MMDF);Serial.print("mm---");
      Serial.print("DT: ");Serial.print(MMDT);Serial.println("mm---");
      Serial.println("------------------------------------------------------------------------------------------------");
    }
  }
  
  //grabar SD 
  void grabar(){
    //si esta habilitado el grabar de datos en la SD
    if (ab_grabar==1){  
        unsigned long tiempo = millis();
        archivo = SD.open("Maze.txt",FILE_WRITE);
        archivo.write("");archivo.println("");
        archivo.println("                                Variables de prueba");
        archivo.println("\n-----------------------------------------------------------------------------------------");
        archivo.print("Var de I-S-F: ");archivo.println(izquierdaSinF);
        archivo.print("Var de IM: ");archivo.println(strImpulso);
        archivo.print("Var de victima: ");archivo.println(victima);
        archivo.print("Var de bl: ");archivo.println(bl);
        archivo.print("Llanta I 1r: ");archivo.print(llantaI1);archivo.print("------Llanta D 1r: ");archivo.println(llantaD1);
        archivo.print("Llanta I 2: ");archivo.print(llantaI2);archivo.print("------Llanta D 2: ");archivo.println(llantaD2);
        archivo.print("Cont de avance I-S-F: ");archivo.println(avance_ISF);archivo. println("------");
        archivo.print("Cont de avance salir: ");archivo.println(cont_Salir);
        archivo.print("Cont de avance salir pixy: ");archivo.println(contSalirVic);
        archivo.print("Cont color A: ");archivo.println(contA);
        archivo.print("Cont color B: ");archivo.println(contB);
        archivo.print("Ciclo ");archivo.println(ciclo);
        archivo.print("->  tiempo: ");archivo.println(tiempo);
        archivo.print("Inercia ");archivo.println(inercia);
        archivo.print("Mensaje ");archivo.println(mensaje);
        archivo.print("Distancias MM IT: ");archivo.print(MMIT);archivo.print("mm---");
        archivo.print("IF: ");archivo.print(MMIF);archivo.print("mm---");
        archivo.print("FT: ");archivo.print(MMFT);archivo.print("mm---");
        archivo.print("DF: ");archivo.print(MMDF);archivo.print("mm---");
        archivo.print("DT: ");archivo.print(MMDT);archivo.println("mm");
        archivo.print("Val.Log.      IT: ");archivo.print(LIT);archivo.print("------");
        archivo.print("IF: ");archivo.print(LIF);archivo.print("------");
        archivo.print("FT: ");archivo.print(LFT);archivo.print("------");
        archivo.print("DF: ");archivo.print(LDF);archivo.print("------");
        archivo.print("DT: ");archivo.println(LDT);archivo.println("-----"); 
        archivo.print("Red1: ");archivo.print(Rojo_Frec);archivo.print("-----");
        archivo.print("Green1: ");archivo.print(Verde_Frec);archivo.print("-----");
        archivo.print("Blue1: ");archivo.println(Azul_Frec);archivo.println("-----");
        archivo.print("Color: ");archivo.println(color);archivo.println("-----");
        archivo.print("DIRECCION: ");archivo.println(dire);archivo.println("-----");
        archivo.println("-----------------------------------------------------------------------------------------");
        archivo.write("-----------------------------------------------------------------------------------------");archivo.println("");
        archivo.write("Ang X: ");archivo.print(ang_x);archivo.print(" --- Y: ");archivo.print(ang_y);archivo.print(" --- Z: ");archivo.print(ang_z);
        archivo.write("         Angulo de inclinacion: ");archivo.println(anguloI);
        archivo.write("*");archivo.println("");
        archivo.close(); 
    }
  } 
  
  //lectura de ultrasónicos 
  long ultra(int trigPin,int echoPin){
    long t=0;
    long d=0;
     digitalWrite(trigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin,LOW);
    t=pulseIn(echoPin,HIGH);
    d = t/59;
    return d;
  }
  
  //lectura de ultrasónicos en milimetros 
  long ultra_mm(int trigPin,int echoPin){
    float t=0;
    float d=0;
    digitalWrite(trigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin,LOW);
    t=pulseIn(echoPin,HIGH);
    d= 340000*t/2000000;
    //d = t/59;
    return d;
  }
  
  
  // no esta en uso 
  void paro(){ 
    //cual es la funcion del servo_1?
    servo_1.write(90);
    delay(1000);
  }
  
  // para que sirve esta funcion?
  void derecha_vic(){
    //cual es la funcion del myServo?
    myServo.write (90);
    delay (delay_de);
    myServo.write (130);
    delay (delay_de);
  }
  
  // para que sirve esta funcion?
  void izquierda_vic(){
    myServo.write (90);
    delay (delay_iz);
    myServo.write (50);
    delay (360);
  }
  
  // para que sirve esta funcion?
  void kit(int dir){
    if(dir==0){
      izquierda_vic();
    }
    else{
      derecha_vic();
    }
  }
  
  // para que sirve esta funcion?
  void serialEvent() {
    while (Serial.available()) {//Mientras tengamos caracteres disponibles en el buffer
      char inChar = (char)Serial.read();//Leemos el siguiente caracter
      if (inChar == '\n') {//Si el caracter recibido corresponde a un salto de línea
        stringComplete = true;//Levantamos la bandera 
      }
      else{//Si el caracter recibido no corresponde a un salto de línea
        inputString += inChar;//Agregamos el caracter a la cadena 
      }
    }
  }
  
  
  
  
//***************************************************
//*     Movimientos del robot                       *
//***************************************************

  // adelante 
  void adelante(){
    //valida si NO esta en una rampa 
    if (anguloI==0){
        // pared en sensor izquierdo frontal 
        if(LIF==0){
          dire="Adelante";
          //ajustes para centrarse hacia la derecha
          if (MMIF<=60){
            moverse(1,3);
          }else{
            //ajustes para centrarse hacia la izquierda 
            if (MMIF >= 70){ 
              moverse(2,1);
            }else{
              //de frente 
              moverse(1,1);
            }
          }
        }else{
          //pared en sensor derecho frontal 
          if(LDF==0){
            //ajustes para centrarse hacia la izquierda
            if (MMDF<=60){
              moverse(3,1);
            }else{
              //ajustes para centrarse hacia la derecha
              if (MMDF >= 70){
                moverse(1,2);
              }else{
                //de frente
                moverse(1,1);
              }
            }
          }else{
            //????????????????????
            if (MMIF==MMDF){
              moverse(1,1);
            }else{
              if(MMIF > MMDF){
                moverse(3,3);
              }else{
                if(MMDF > MMIF){
                  moverse(3,3);
                }
              }
            }
          }
        }
    }else{
      dire="rampa";
    }
  }
  
  //se detiene el robot 
  void detenerse(){
    dire = "Alto";
    moverse(0,0);
  }

  //gira a la izquierda 
  void izquierda(){
    dire="izquierda";
    //inicia el giro a la izquierda 
    if (ciclo<=giros_I){
      moverse(-1,1);
      inercia="IZ";
      ciclo++;
    }else{
      //continua girado mientras la distancia del sensor derecho trasero 
      //sea mayor a la distancia de sensor derecho frontal            
      if (MDT > 80){   
          moverse(-1,1);
          inercia="IZ";
      }else{
        // salr durante un numero de ciclos 
        if(cont_Salir<=pasos_salir && MDT < 90){
          moverse(1,1);
          inercia="IZ";
          cont_Salir++;
        }else{
          ciclo=0;
          cont_Salir=0;
          moverse(1,1);
          inercia="FT";
        }
      }
    }
  }
  
  //gira a la izquierda cuando no hay pared al frente 
  //es diferente porque no se puede tomar como referencia la 
  //pared frontal al girar  
  //hay que hacer un analisis completo de esta funcion 
  void izquierda_sin_fte(){
    //MMFT (la distancia en milimetros aqui debe haber un parametro NO un valor absoluto  
    if(MMFT > 155){
      dire="Izquierda_sin_frente";
      if(avance_ISF<11){
        if(LFT == 0){
          inercia="FT";
          avance_ISF=0;
          ciclo=0;
          moverse(1,1);
          izquierdaSinF = "H-PA";
          }else{
              moverse(3,3); 
              izquierdaSinF = "Adelnate";
              inercia="IZF";
              avance_ISF++;
          }
        }
        else{
          if (ciclo<=giros_ISF){
            moverse(-1, 1);
            inercia="IZF";
            ciclo++;  
            izquierdaSinF = "Giro";
            lecturas = false;
          }else{
            ciclo=0;
            avance_ISF=0;
            moverse(1,1);
            inercia="IM";
            izquierdaSinF = "Rinicio";
            lecturas = true;
          }
        }
      }else{
        // no se pudo iniciar izquierda sin frente
        izquierdaSinF = "N-IN-ISF";
        if(LFT == 0){
          izquierdaSinF = "I-H-P";
          avance_ISF=0;
          izquierda();
        }else{
          if (LFT == 1){
            moverse(1,1);
            izquierdaSinF = "AD-C-F";
          }
        }
      }
    }

  //el robot gira a la derecha 
  void derecha(){ 
    dire="Derecha";
    //inicia el giro a la derecha 
    if (ciclo<=giros_D){
      moverse(1,-1);
      inercia="DER";
      ciclo++;
    }else{
       //continua el giro mientras la distancia del sensor izquierdo trasero
       //sea mayor que la distancia del sensor izquierdo frontal 
       if (MIT > 80){ 
          moverse(1,-1);
          inercia="DER";
       }else{
          // salr durante un numero de ciclos 
          if (cont_Salir<=pasos_salir){
            moverse(1,1);
            inercia="DER";
            cont_Salir++;
          }else{
            ciclo=0;
            cont_Salir=0;
            moverse(1,1);
            inercia="FT";
          }
        }
    }
  }

  //giro en U 
  void giro_u(){
    dire="Giro_u";
    //inicia el giro en U 
    if (ciclo<=giros_u){
      moverse(1,-1);

      inercia="GU";
      ciclo++;
    }else{
      if (MIT > 80){ 
        moverse(1,-1);
        inercia="DER";
      }else{
        if (LFT==1){
          moverse(1,1);
          inercia="FT";
          ciclo=0;
        }
      }
    }
  }

  //codificar las acciones en trampa azul 
  void trampa_azul(){
    Serial.print("tramp azul");
  }

  //??????????????????????????????????????????????????????
  void impulso(){
    dire="impulso";
    if (ciclo<=imp){
      if(LFT==0){
        ciclo=0;
        inercia="FT";
        strImpulso = "H-PA";
      }else{
        if(LDT==0 && LDF==0){
          if (MMDF<=50){
            moverse(2,1);
            strImpulso = "PE-D-I";
          }else{
            if (MMDF>=70){ 
              moverse(1,2);
              strImpulso = "AL-D-D";
            }else{
              moverse(1,1);
              strImpulso = "AD";
            }
          }
        //revisar este else pues NO tenia llave que se supone hace???????????????????
        }else{ 
          if (LIT==0 && LIF==0){
            if (MMIF<=50){
              moverse(1,2);
              strImpulso = "PE-I-D";
            }else{
              if (MMIF>=70){
                moverse(2,1);
                strImpulso = "AL-I-I";
              }else{
                moverse(1,1);
                strImpulso = "AD";
              }
            }
          }else{
            strImpulso = "N-PA";
            moverse(1,1);
          }
          inercia="IM";
        }
      }
      ciclo++;
    }else{
      //moverse(0,0);
      //delay(5000);
      inercia="FT";
      ciclo=0;
      strImpulso = "R";
    }
  }

  void paredFrente(){
    // NO HAY PARED A LA IZQUIERDA Y HAY PARED AL FRENTE
    if (LIT ==1 && LIF==1){
      izquierda();
    }else{
      // NO HAY PARED A LA DERECHA Y HAY PARED AL FRENTE
      if ((LDF==1 && LDT==1) && (LIT ==0 && LIF==0)){
        derecha();   
        //Serial.print("derecha");         
      }else{
        //HAY PARED EN TODAS LAS DIRECCIONES
        if ((LDF==0 && LDT==0) && (LIF==0 && LIT==0) ){
          giro_u();
        }else{
          //SI HAY UNA INDETERMINACION
          adelante();
        }
      }
    }
  }

  void sinParedFrente(){
    //No hay pared a la izquierda y no hay pared al frente
    if (inercia=="IM"){
      impulso();
    }else{
      if (LIT==1 && LIF==1){
        izquierda_sin_fte();
      }else{
        adelante(); 
      }
    }
  }

  void inerciaFrente(){
    if (LFT==0){
      //Con pared al frente
      paredFrente();
      
    }else{
      // sin pared al frente
      sinParedFrente();
    }
  }

  void pisoBlanco(){
    //estas variables que hacen
    bl=1;
    ng=1;
    if (inercia=="FT"){
      //si el robot va de frente 
      inerciaFrente();
    }else{
      //inercia a la izquierda 
      if (inercia=="IZ"){
        izquierda();
      }else{
        //inercia a la izquierda sin frente
        if (inercia=="IZF"){
          izquierda_sin_fte(); 
        }else{
          //inercia derecha 
          if(inercia=="DER"){
            derecha();
          }else{
            //inercia a giro en U
            if (inercia=="GU"){
             giro_u();
            }else{
              //inercia impulso
              if (inercia=="IM"){
                impulso();
              }
            }
          }
        }
      }
    }
  }


  void pisoNegro(){
    // detiene las lecturas de sensores?
    lecturas = false;
    //deja de grabar en SD????????????????????????????????????????????
    ab_grabar=0;
    //se detiene medio segundo 
    detenerse();
    delay(500);
    //reversa por dos segundos 
    moverse(-1,-1);
    delay(2000);
    //giro en U 
    giro_u();
    //habilita grabar en SD??????????????????????????????????????????
    ab_grabar = 1;
    //Reinicia lecturas de sensores
    lecturas = true;   
  }

  //este procedimiento tiene MUCHO codigo repetitivo 
  //checarlo TODO 
  void pisoAzul(){
    //bl?
    if(bl==1){
      //detiene la lecturas de sensores 
      lecturas = false;
      //deja de grabar en SD???????????????????????????????????????
      ab_grabar=0;
      //avanza 3.5 segundos 
      adelante();
      delay(3500);
      //se detiene 5 segundos 
      detenerse();
      delay(5000);
      //bl?
      bl=0;
      //habilita grabar en SD??????????????????????????????????
      ab_grabar = 1;
      //Reinicia lecturas de sensores
      lecturas = true;
      //la identacion es incorrecta parecia que cierra el if de color azul 
    }else{      
      bl=0;
      //inercia al frente 
      if (inercia=="FT"){
        //Con pared al frente
        if (LFT==0){
          // NO HAY PARED A LA IZQUIERDA Y HAY PARED AL FRENTE
          if (LIT ==1 && LIF==1){
           izquierda();
          }else{
            // NO HAY PARED A LA DERECHA Y HAY PARED AL FRENTE
            if ((LDF==1 && LDT==1) && (LIT ==0 && LIF==0)){
              derecha();            
            }else{
              //HAY PARED EN TODAS LAS DIRECCIONES
              if ((LDF==0 && LDT==0) && (LIF==0 && LIT==0) ){
                giro_u();
              }else{
                //SI HAY UNA INDETERMINACION
               adelante();
              }
            }
          }
        }else{
          //No hay pared a la izquierda y no hay pared al frente
          if (inercia=="IM"){
            impulso();
          }else{
            if (LIT==1 && LIF==1){
              izquierda_sin_fte();
            }else{
              adelante(); 
            }
          }
        }
      }else{
        if (inercia=="IZ"){
          izquierda();
        }else{
          if (inercia=="IZF"){
            izquierda_sin_fte(); 
          }else{
            if (inercia=="DER"){
              derecha();
            }else{
              if (inercia=="GU"){
                giro_u();
              }else{
                if (inercia=="IM"){
                  impulso();
                }
              }
            }
          }
        }
      }
    }
  }


  void victimaEncontrada(){
    //victima
    if (victima == "detiene"){
      //verifica que exista victima
      detenerse();
      delay(1000);
      victima="entrega";
    }
    else if (victima=="entrega"){
        detenerse();
        servoVic.writeMicroseconds(1440);
        delay(600);
        detenerse();
        delay(2000);
        victima="sale";
    }else if (victima == "sale"){
      pisoBlanco();
    }
  }
  
  //movimiento en piso plano
  void plano(){
    if(color != "NEGRO" && victima == "camina" && color != "AZUL"  ){
      // el piso es camino normal y no se encuentra una victima 
        pisoBlanco();
    }else{
      //el color negro es prohibido en el piso hay que girar en U
      if(color == "NEGRO"){
        pisoNegro();
      }else{
        //el piso es de color azul ¿ESTO QUE SIGNIFICA?
        //contA????????????????????????????????????????
        if (color=="AZUL" && contA == 3){
          pisoAzul();
        }else{
          victimaEncontrada();
        }
      }
    }
  }

  void direccion(){
    mensaje="";
    if (anguloI == 0){
      plano();
    }
  }

/////////////////////////////////////////////////////////////////////////////////////
//          procedimiento principal LOOP infinito de funcionamiento del robot      //
/////////////////////////////////////////////////////////////////////////////////////
void loop(){
  //lee los sensores 
  Leer();
  //determina la direccion a tomar 
  direccion();
  //graba en la SD 
  grabar();
  
}

/*  En esta versión 1.11

  - En esta versión, se integró la parte de la programación PixyMon v2,
    que cuando detecte un color sé prendera un led correspondiente.

*/
