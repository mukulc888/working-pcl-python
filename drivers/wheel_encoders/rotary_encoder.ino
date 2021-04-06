// Define constants
//vel = 2*3.14*radius * RPM / 60

#define outputCLK 2
#define outputDT 3
#define encoderPPR 400
#define pie 3.14159
#define w_radius 0.5

volatile int currentState = 0;
volatile int lastEncoded = 0;
volatile float encoderValue = 0;

float RPM = 0;
float vel = 0;
unsigned long time_delay = 100;
float lastEncoderValue = 0;
unsigned long prev_time = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(outputCLK, INPUT_PULLUP);
  pinMode(outputDT, INPUT_PULLUP);

  // Pull-up The pins
  digitalWrite(outputCLK, HIGH);
  digitalWrite(outputDT, HIGH);

  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
}

void loop() {
  unsigned long curr_time = millis();
  float encoderValueDiff;

  if ((curr_time - prev_time) >= time_delay){
    encoderValueDiff = encoderValue - lastEncoderValue;
    RPM = encoderValueDiff / encoderPPR;
    
    vel = (2 * pie * w_radius * RPM);
    lastEncoderValue = encoderValue;
    prev_time = curr_time;
  }
// Serial.println(RPM);
 Serial.println(vel); // raw value 
// Serial.println(encoderValue);  //Wheel Velocity

 //Serial.print('\n'); 
 delay(100); 
}

void updateEncoder(){
  currentState = digitalRead(outputCLK);
  
  // Clockwise change != to == for Counter-Clockwise
  if (digitalRead(outputDT) != lastEncoded){
    encoderValue++;
  } 

// Imma gonna remove this cause Race Car moving backwards is unlikely
//  else {
//    encoderValue--;
//  }

  lastEncoded = currentState;
}
