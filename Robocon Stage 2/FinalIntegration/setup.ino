void setup() {
  //Serial begins
  Serial.begin(115200);
  delay(20);
  Serial1.begin(9600);
  delay(20);
  Serial2.begin(115200);
  delay(20);
  Serial3.begin(115200);
  delay(20);
  //Servo and esc
  myServo.attach(60);
  myServo.write(90);
//  esc1.attach(6);
//  esc1.write(30);
//  esc2.attach(7);
//  esc2.write(30);
    esc3.attach(8);
  esc3.write(30);
    esc4.attach(9);
  esc4.write(30);
  delay(1000);
  //Base motors
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(M1_pwm, OUTPUT);
  pinMode(M2_pwm, OUTPUT);
  pinMode(M3_pwm, OUTPUT);
  pinMode(M4_pwm, OUTPUT);
  //Pid
  PIDX.SetMode(AUTOMATIC);
  PIDY.SetMode(AUTOMATIC);
  PIDX.SetOutputLimits(-100, 100);
  PIDY.SetOutputLimits(-100, 100);
  PIDDEG.SetOutputLimits(-100, 100);
  digitalWrite(johnl1, LOW);
  digitalWrite(johnl2, HIGH);
  digitalWrite(johnr1, LOW);
  digitalWrite(johnr2, HIGH);
  //Stepper
  stepper1.setMaxSpeed(5000);
  stepper1.setAcceleration(5000);
  stepper2.setMaxSpeed(5000);
  stepper2.setAcceleration(5000);
  stepper3.setMaxSpeed(5000);
  stepper3.setAcceleration(5000);
  stepper4.setMaxSpeed(5000);
  stepper4.setAcceleration(5000);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin4, OUTPUT);
  pinMode(dirPin4, OUTPUT);
  pinMode(pnu1, OUTPUT);
  pinMode(pnu2, OUTPUT);
  pinMode(shootpnu1, OUTPUT);
  pinMode(shootpnu2, OUTPUT);
  FastLED.addLeds<WS2811, 6, GRB>(leds, 26).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(255);
  fill_solid(leds,26,CRGB::Red);
  FastLED.show();
}
