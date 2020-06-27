int val = 0;
int pin = 2;
bool high = true;
bool low = false;
void setup()
{
  Serial.begin(9600); 
  Serial.println();
}

void loop() {
  val = analogRead(A0);
  if(val < 1000hkx && high == true){
    for(int i = 0; i<100; i++){
      Serial.println("0");}
    high = false;
    low = true;
    }
   else{
    Serial.println("1");
    high = true;
    low = false;
     
   }
    
  // put your main code here, to run repeatedly:

}
