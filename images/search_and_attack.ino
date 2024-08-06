#include <QTRSensors.h>

QTRSensors qtr;
int start_pin = 9;
int finish_pin = 10;
int motorLPin = 5; // PWM output pin connected to the motor
int dirLPin = 7; // direction pin connected to the motor
int motorRPin = 6; // PWM output pin connected to the motor
int dirRPin = 8; // direction pin connected to the motor
int speed = 0; // speed value between 0 and 255

#define FORWARD_SPEED 75
#define BACKWARD_SPEED 75
#define FORWARD_FULL_SPEED 255

// Line sensors
int d_sensor_1 = 2; // Digital sensor connected to D2
int d_sensor_2 = 3; // Digital sensor connected to D3
int d_sensor_3 = 4; // Digital sensor connected to D4
float distance_sensor_2_total = 0.0;

// Next, we need to set up the pins as outputs:

void setup() {
  Serial.begin(9600);
  pinMode(start_pin, OUTPUT);
  pinMode(finish_pin, OUTPUT);
  pinMode(motorLPin, OUTPUT);
  pinMode(dirLPin, OUTPUT);
  pinMode(motorRPin, OUTPUT);
  pinMode(dirRPin, OUTPUT);
  pinMode(d_sensor_1, INPUT);
  pinMode(d_sensor_2, INPUT);
  pinMode(d_sensor_3, INPUT);

  // 5 second delay
  digitalWrite(start_pin, HIGH);
  digitalWrite(finish_pin, LOW);
  delay(5000);
  digitalWrite(start_pin, LOW);
  digitalWrite(finish_pin, HIGH);
  delay(500);
  digitalWrite(finish_pin, LOW);

  // Initialize the sensors.
  // In this example we have three sensors on pins A1, A2 & A3
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A1, A2, A3}, 3);

  randomSeed(analogRead(4));
}

void loop() {
  
  // Read line sensors
  int sensors[2];
  qtr.read(sensors);

  // Read distance sensors - take 10 readings and get the average to avoid flicker
  for(int i=0; i<10; i++) {    
    distance_sensor_2_total += digitalRead(d_sensor_2); // Read sensor 2 value    
  }
  //print_distance_sensor_average_value(distance_sensor_2_total);

  print_line_sensor_readings(sensors);

  // If on white line, move off
  if (sensors[0] < 300 || sensors[1] < 300) { // Left or Right front
    // turn around at a random angle
    Serial.println("line detected...");
    brake();
    backward(BACKWARD_SPEED, 0, random(250, 500));
    brake();
    forward(FORWARD_SPEED, -1);
  }
  else if (sensors[2] < 600) {
    // move forward 
    forward(FORWARD_SPEED, -1);
  }
  else {
    // move forward
    // Can set a variable for direction and not call this if already moving forward
    forward(FORWARD_SPEED, -1);
  }

  // Attack if sense an object
  if(distance_sensor_centre_average_value(distance_sensor_2_total) > 0.75f) {
    attack();
  }
}

void setSpeed(int s) {
  // set the motor speed to a value between 0 and 255
  analogWrite(motorLPin, s);
  analogWrite(motorRPin, s);
}

void setSpeed(int sl, int sr) {
  // set the motor speed to a value between 0 and 255
  analogWrite(motorLPin, sl);
  analogWrite(motorRPin, sr);
}

void forward(int s, int t) {
    digitalWrite(dirLPin, HIGH);
    digitalWrite(dirRPin, HIGH);
    setSpeed(s);

    if(t != -1)
      delay(t);
}

void forward(int sl, int sr, int t) {
    digitalWrite(dirLPin, HIGH);
    digitalWrite(dirRPin, HIGH);
    setSpeed(sl, sr);

    if(t != -1)
      delay(t);
}

void backward(int s, int t) {
    digitalWrite(dirLPin, LOW);
    digitalWrite(dirRPin, LOW);
    setSpeed(s);

    if(t != -1)
      delay(t);
}

void backward(int sl, int sr, int t) {
    digitalWrite(dirLPin, LOW);
    digitalWrite(dirRPin, LOW);
    setSpeed(sl, sr);
    
    if(t != -1)
      delay(t);
}

void brake() {
  Serial.println("braking....");
  while (speed > 0) {
    speed -= 50;
    setSpeed(speed);
  }
  speed = max(speed, 0);
  setSpeed(speed);
}

void print_line_sensor_readings(int sensors[]) {
    // Print out sensor readings
  Serial.println("##################################################");
  Serial.print("Left:"); Serial.print(sensors[0]);
  Serial.print(",  Right:"); Serial.print(sensors[1]);
  Serial.print(",  Back:"); Serial.println(sensors[2]);
  Serial.println("##################################################");
  Serial.println();
}

void print_distance_sensor_average_value(float centre) {
  Serial.print("D2:");
  Serial.println(centre/10);
}

float distance_sensor_centre_average_value(float centre) {
  return centre/10;
}

void attack() {
  forward(FORWARD_FULL_SPEED, -1);
  Serial.println("attacking....");
  distance_sensor_2_total = 0;
}
