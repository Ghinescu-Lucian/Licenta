
int phase1 = 9;
int phase2 = 10;
int phase3 = 11;
int speed = 20;
int direction = 1;
int contor =0;
// 1 = forward , 2 = backward


const int buttonPin1 = 2;
int buttonState1 = 0;


void setup() {
  pinMode(buttonPin1, INPUT);
  pinMode(phase1, OUTPUT);
  pinMode(phase2, OUTPUT);
  pinMode(phase3, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(9600);
}

void loop(){
 delay(500);
   digitalWrite(phase1, HIGH); 
      digitalWrite(phase3, LOW); 
      digitalWrite(12,LOW);
      digitalWrite(phase2, LOW); 
  
}

void rotateMotor(){

  if(direction == 1){
    contor=contor+1;
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
      digitalWrite(phase2, HIGH); delay(speed);
      digitalWrite(phase1, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
     

  }
  else{
      digitalWrite(phase2, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
      digitalWrite(phase1, LOW); delay(speed);
      }

}