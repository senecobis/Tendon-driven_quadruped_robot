#include <Servo.h>

Servo left_back_upper;
Servo left_back_lower;
Servo left_back_feet;

Servo right_back_upper;
Servo right_back_lower;
Servo right_back_feet;

Servo left_front_upper;
Servo left_front_lower;
Servo left_front_feet;

Servo right_front_upper;
Servo right_front_lower;
Servo right_front_feet;

const int middle = 90;
const int start = 0;
const int mov_delay = 10;

void setup() {
  left_back_upper.attach(12); 
  left_back_lower.attach(11);
  left_back_feet.attach(10);

  right_back_upper.attach(9);
  right_back_lower.attach(8);
  right_back_feet.attach(7);

  left_front_upper.attach(6);
  left_front_lower.attach(5);
  left_front_feet.attach(4);

  right_front_upper.attach(3);
  right_front_lower.attach(2);
  right_front_feet.attach(13);

  Serial.begin(9600);

  set_aestetic_pose();
  delay(100);
}


void set_constrained_feet_pos(int pos, Servo feet_){
  // limits for feet is 0-90 deg, this function imposes the constraint
  if (pos < 0){
    pos = 0;
  }
  if (pos > 90){
    pos = 90;
  }
  feet_.write(pos);
}


void set_default_pose(){
  // Here we report the defualt degree angles for the robot joint 
  // just for the back of the body
  left_back_upper.write(0);
  left_back_lower.write(0);
  set_constrained_feet_pos(0, left_back_feet);

  right_back_upper.write(180);
  right_back_lower.write(180);
  set_constrained_feet_pos(90, right_back_feet);

  left_front_upper.write(0);
  left_front_lower.write(180);
  set_constrained_feet_pos(90, left_front_feet);

  right_front_upper.write(180);
  right_front_lower.write(0);
  set_constrained_feet_pos(0, right_front_feet);
}


void set_aestetic_pose(){
  // just aestetic static pose for photos
  left_back_upper.write(80);
  left_back_lower.write(110);
  set_constrained_feet_pos(30, left_back_feet);

  right_back_upper.write(100);
  right_back_lower.write(70);
  set_constrained_feet_pos(60, right_back_feet);

  left_front_upper.write(90);
  left_front_lower.write(50);
  set_constrained_feet_pos(60, left_front_feet);

  right_front_upper.write(90);
  right_front_lower.write(130);
  set_constrained_feet_pos(30, right_front_feet);

}

void loop() {
    //set_default_pose();
    // set_aestetic_pose();
    // delay(1000);

  if (Serial.available() > 0) {
    int angle = Serial.parseInt();  // Read angle from serial input
    right_front_lower.write(angle);         // Move servo to the specified angle
    Serial.print("Servo moved to: ");
    Serial.println(angle);
  }

}



