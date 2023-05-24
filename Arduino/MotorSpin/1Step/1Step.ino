
int phase1 = 9;
int phase2 = 10;
int phase3 = 11;
int speed = 10;
int direction = -1;
int contor =0;
// 1 = forward , -1 = backward


const int buttonPin1 = 2;
int buttonState1 = 0;


void setup() {
  pinMode(buttonPin1, INPUT);
  pinMode(phase1, OUTPUT);
  pinMode(phase2, OUTPUT);
  pinMode(phase3, OUTPUT);
  Serial.begin(9600);
}

void loop(){
//  delay(500);
  buttonState1 = digitalRead(buttonPin1);
  
  if(digitalRead(buttonPin1) == HIGH){
   Serial.print(digitalRead(buttonPin1));
    while(digitalRead(buttonPin1)== HIGH){}
    //  direction = -direction;
     Serial.print("Am citit butonul!: ");
     Serial.println(direction);
     rotateMotor();  
  }

  //  rotateMotor();  
  
  
}

void rotateMotor(){

contor=contor+1;
  if(direction == 1){
   if(contor % 2 == 0){
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
      digitalWrite(phase2, HIGH); delay(speed);
    }
    else{
      digitalWrite(phase1, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
   }
     

  }
 else{

   if(contor%2==0){

      digitalWrite(phase1, LOW); delay(speed);
      digitalWrite(phase2, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
   }
   else{
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
   }
      
      
    
  }

}