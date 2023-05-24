
int phase1 = 9;
int phase2 = 10;
int phase3 = 11;
int speed = 7 ; 
int direction = 1;
int contor =0;
// 1 = forward , 2 = backward


void setup() {
  pinMode(phase1, OUTPUT);
  pinMode(phase2, OUTPUT);
  pinMode(phase3, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  // Serial.println("Contor: "+contor);
  // if(contor!= 300){
  if(direction == 1  ){
    contor=contor+1;
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
      digitalWrite(phase2, HIGH); delay(speed);
      digitalWrite(phase1, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
    //  Serial.println("Contor: "+direction);
      // if(contor % 50==0) {
      //   direction ==0;
      //   contor=0;
      // }

  }
  else{
      digitalWrite(phase2, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
      digitalWrite(phase1, LOW); delay(speed);
      // if(contor % 50==0){
      //           direction ==1;
      //   contor=0;

      // }
  }
  // }
}