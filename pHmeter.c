#include <math.h>

const int ph= A0; //This will be info from analog ph
const int pump1=11; //pinNum of pump for alkali
const int pump2=9; //pinNum of pump for acid
float measpH;
int phv;
float desired_pH = 10;

unsigned long T ,T1,T2 ;// time
float V = 0;// volume of liquid in the cup
const float flowrate = 0.83;//0.82 ml/s
// linear relationship between pH and Voltage, here is y-intercept
const float Y =77 ;            
//linear relationship between pH and Voltage, here is gradient
const float m = 19.7;
//Desired control parameters:

const int numReadings = 100;// sample values
int readings[numReadings]; // Array to store the readings
int readIndex = 0; // Index for current reading
long total = 0; // Running total of readings
float average = 0; // Calculated average

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  pinMode(pump1,OUTPUT);
  pinMode(pump2,OUTPUT);
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0; // Initialise all readings to 0}
}
}
void loop() {
  // put your main code here, to run repeatedly:


while ( V< 500){



total -= readings[readIndex];
readings[readIndex] = analogRead(ph);
total += readings[readIndex];
readIndex = (readIndex + 1) % numReadings;
average = (float)total / numReadings;
 
  
 phv=analogRead(ph);
 //calibration here
 measpH = (average-Y)/(m) ;
 
 // get Time
 T = millis();


 if(T > T1){ T1 = T1 + 1000;
 
 Serial.println(phv);
 Serial.print("PH is");
 Serial.println(measpH);
 Serial.print("Average: ");
 Serial.println(average);

 }


if (measpH - desired_pH > 1 )
{
   digitalWrite(pump2,HIGH);
   digitalWrite(pump1,LOW);
}

else if (measpH - desired_pH < -1)
{
  digitalWrite(pump1,HIGH);
  digitalWrite(pump2,LOW);

}
else
{
  
  digitalWrite (pump1,LOW);
  digitalWrite(pump2,LOW);
}


if (T > T2){T2 = T2 + 1000; V = V + flowrate*1.5; Serial.print("Volume is"); Serial.println(V);}
}
// not in while loop,so pumps should stop
digitalWrite (pump1,LOW);
digitalWrite(pump2,LOW);

}
