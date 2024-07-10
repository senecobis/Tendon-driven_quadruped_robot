#include <Servo.h>

Servo upper_leg;
Servo lower_leg;
Servo feet;

const int middle = 90;
const int start = 0;
const int mov_delay = 10;

void setup() {
  upper_leg.attach(9); 
  lower_leg.attach(10);
  feet.attach(11);
  Serial.begin(9600);
}

void move_forward_leg(int start, int middle, int mov_delay){
  for(int pos = start; pos <= middle; pos++){
    upper_leg.write(pos);
    Serial.println(pos);
    delay(mov_delay);
  };
}

void move_backward_leg(int start, int middle, int mov_delay){
  for(int pos = middle; pos >= start; pos--){
    upper_leg.write(pos);
    Serial.println(pos);
    delay(mov_delay);
  };
}

void move_feet(int start = 0, int middle = 360){
    for(int pos = start; pos <= middle; pos++){
      feet.write(pos);
      Serial.println(pos);
      delay(mov_delay);
    };
    for(int pos = middle; pos >= start; pos--){
      feet.write(pos);
      Serial.println(pos);
      delay(mov_delay);
  };
}


void set_upper_leg_pos(int pos){
  upper_leg.write(pos);
}

void set_lower_leg_pos(int pos){
  lower_leg.write(pos);
}

void set_feet_pos(int pos){
  // limits for feet is 0-70 deg
  if (pos < 0){
    pos = 0;
  }
  if (pos > 70){
    pos = 70;
  }
  feet.write(pos);
}

void set_leg_default_pos(){
  set_upper_leg_pos(0);
  set_lower_leg_pos(0);
  set_feet_pos(0);
}


void loop() {
    set_leg_default_pos();
    delay(1000);
}



