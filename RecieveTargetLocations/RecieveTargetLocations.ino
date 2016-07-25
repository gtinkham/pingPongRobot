//Grant Tinkham
// 4/10/14
//


/*This File is meant to recieve data from the Matlab file
    
*/
int x[6];
int y[6];

void setup(){
 Serial.begin(9600);
  int k;
  byte X_highbyte, X_lowbyte, Y_highbyte, Y_lowbyte;
  
  for (k=0; k<6; k++){
    while (Serial.available()<2);
    X_highbyte = Serial.read();
    X_lowbyte = Serial.read();
    //Y_highbyte = Serial.read();
   // Y_lowbyte = Serial.read();
   // y[k] = Y_highbyte * 256 + Y_lowbyte;
    x[k] = X_highbyte * 256 + X_lowbyte;
//    delay(50);
    Serial.print("The X coordinate is ");
    Serial.println(x[k]);
    //Serial.print("The Y coordinate is ");
    //Serial.println(y[k]);
  
  
}

for (k=0; k<6; k++){
    while (Serial.available()<2);
     Y_highbyte = Serial.read();
     Y_lowbyte = Serial.read();
    y[k] = Y_highbyte * 256 + Y_lowbyte;
   
//    delay(50);
    Serial.print("The Y coordinate is ");
    Serial.println(y[k]);
  
  
  }
}
void loop() {
}
