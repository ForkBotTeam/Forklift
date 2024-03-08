#define unfiltered A0
//#define filtered A5
int en1 = 4;
int en2 = 5;
float steering_pwm = 6;

int Kp = 6;

float setpoint = 0;
float error = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(steering_pwm, OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  Serial.begin(9600);

}

void loop() {
  
  float filtered = analogRead(filtered);
  float unfiltered = analogRead(unfiltered);
  filtered = (filtered-512)/1024.0*180*1.5;
  unfiltered = (unfiltered-512)/1024.0*180*1.5;
  
  error = setpoint - filtered;
    
  while(abs(error) < 0.1){
    if(Serial.available())
      setpoint = Serial.parseInt();
    filtered = analogRead(filtered);
    filtered = (filtered-512)/1024*180.0*1.5;
    error = setpoint - filtered;
  }
  //Serial.print("error=");
  //Serial.print(error);
  //Serial.print(" pot=");
  Serial.println(filtered);
  Serial.println(unfiltered);
  //Serial.print(" setpoint="); 
  //Serial.println(setpoint);

  analogWrite(steering_pwm, abs(error)*Kp);
  if(error<0){
    digitalWrite(en1, 0);
    digitalWrite(en2, 1);}
  else{
    digitalWrite(en2, 0);
    digitalWrite(en1, 1);}    
}
