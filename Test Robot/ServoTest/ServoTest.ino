#include <Servo.h>


Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;



void setup() {
    servo1.attach(49);
    servo2.attach(45);
    servo3.attach(47);
    servo4.attach(43);


  pinMode(13, OUTPUT);//luces de abajo
  pinMode(12, OUTPUT);  
  pinMode(11, OUTPUT);



  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(11, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  servo1.write(180);
  servo2.write(180);
  servo3.write(0);
  servo4.write(0);

}