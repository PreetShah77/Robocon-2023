int limit1 = 54;


void setup() {
  // put your setup code here, to run once:
    pinMode(47,OUTPUT);
        pinMode(49,OUTPUT);
            pinMode(51,OUTPUT);
                pinMode(53,OUTPUT);
        pinMode(limit1,INPUT);
        Serial.begin(9600);
  
}

void loop() {

  
  int leftpickmax = analogRead(limit1);
  Serial.println(leftpickmax);


//   put your main code here, to run repeatedly:
if(leftpickmax!=0){
    digitalWrite(47,HIGH);
  digitalWrite(49,HIGH);
  digitalWrite(51,HIGH);
  digitalWrite(53,HIGH);
  delay(1000);
        digitalWrite(47,LOW);
  digitalWrite(49,LOW);
  digitalWrite(51,LOW);
  digitalWrite(53,LOW);
  delay(1000);
}
else{
          digitalWrite(47,LOW);
  digitalWrite(49,LOW);
  digitalWrite(51,LOW);
  digitalWrite(53,LOW);
  }
}
