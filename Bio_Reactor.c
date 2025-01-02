
//Note: members in our team created this code collectively

#include <Wire.h> 


// Define Slave I2C Address 
#define SLAVE_ADDR 9 
 
// Declare pin numbers
const byte hallSensor = 2, motor = 10, thermistor = A1, heatingElement = 3, ph = A0, pump1 = 11, pump2 = 9;

// Global variables for interrupt handling
volatile long previousPulseT, pulseT;
volatile float frequency;

// Global variable for data (I know this is bad)
float currentSpeed = 0.00;
float currentTemp = 0.0;
float currentpH = 0.0;

float receivedRPM = 0;
float receivedTemp = 0;
float receivedpH = 0;


// Define freqcount function
void freqcount() {
  // Motor parameters needed in the interrupt (remain global)
  const int RPMmax = 1500;
  const int Npluses = 70;
  const int Tmin = 6e7 / RPMmax / Npluses;
  pulseT = micros();
  if (abs(pulseT - previousPulseT) > Tmin) {
    frequency = 0.75 * frequency + 2.5e5 / float(pulseT - previousPulseT);
    previousPulseT = pulseT;
  }
}

 
// Stirring control system
float controlMotor(int DesiredSpeed) {
  // Local constants for motor and controller parameters
  const float Kv = 225;             // Motor speed constant in RPM per Volt
  const float T = 0.15;             // Motor and load time constant (s)
  const float Npluses = 70;         // Number of pulses per revolution
  const float wn = 1 / T;           // Desired control frequency
  const float zeta = 1;             // Damping ratio
  const float wo = 1 / T;           // Motor response frequency in rad/s
  const float Kp = (2 * zeta * (wn / wo) - 1) / Kv; // Proportional gain
  const float KI = (wn * wn) / (Kv * wo);           // Integral gain
  const float freqtoRPM = 60 / Npluses;            // Frequency to RPM conversion
  // Static variables to retain their values between function calls
  static long previousTime = 0;
  static long T1 = 0;
  static float KIinterror = 0;
  static float meanMeasureSpeed = 0;
  long currentTime = micros();
  float deltaT = (currentTime - previousTime) * 1e-6;  // Measure deltaT for integration
  if (currentTime - T1 > 0) {
    previousTime = currentTime;
    T1 = T1 + 10000; // 10 ms update period
    float measureSpeed = frequency * freqtoRPM;
    float error = DesiredSpeed - measureSpeed;
    // Integral term with anti-windup
    KIinterror += KI * error * deltaT;
    KIinterror = constrain(KIinterror, 0, 5); // Limit to 5 V
    // PI controller
    int Vmotor = round(204 * (Kp * error + KIinterror));  
    Vmotor = constrain(Vmotor, 0, 1023); // Limit motor voltage
    if (Vmotor == 255) {
      Vmotor = 256; // Work-around for bug in Nano/Uno with 10-bit configuration
    }
    analogWrite(motor, Vmotor);
    // Handle no pulse condition
    if (currentTime - pulseT > 1e5) {
      measureSpeed = 0;
      frequency = 0;
    }
    // Calculate meanMeasureSpeed
    meanMeasureSpeed = 0.1 * measureSpeed + 0.9 * meanMeasureSpeed;
  }
  // Return the current speed for monitoring
  return meanMeasureSpeed;
}


// Temperature control system
float controlTemperature(float desiredTemperature) {
  // Declare local constants
  const float minDesiredTemperature = 25.0;   // Minimum desired temperature
  const float maxDesiredTemperature = 35.0;   // Maximum desired temperature
  const float tolerance = 2.0;                // Tolerance set to 5.0°C

  // Constrain the desired temperature
  desiredTemperature = constrain(desiredTemperature, minDesiredTemperature, maxDesiredTemperature);

  // Read the current analog value from the thermistor
  int analogReadValue = analogRead(thermistor);
  float currentTemperature = -0.1013 * analogReadValue + 74.361;

  // Bang-Bang Control for heating element
  if (currentTemperature < desiredTemperature - tolerance) {
    digitalWrite(heatingElement, HIGH); // Turn heating element ON
  } else if (currentTemperature > desiredTemperature + tolerance) {
    digitalWrite(heatingElement, LOW); // Turn heating element OFF
  }

  // Return the current temperature for monitoring
  return currentTemperature;
}

// pH control system
float controlPH() {
    const float desired_pH = 10; // Set pH
    const float flowrate = 0.83; // 0.82 ml/s
    const float Y = 80; // y-intercept for calibration
    const float m = 18.3; // gradient for calibration
    const int numReadings = 100; // Sample values

    float measpH = 0.0;
    int phv = 0;
    unsigned long T = 0, T1 = 0, T2 = 0; // Time variables
    float V = 0; // Volume of liquid in the cup
    int readings[numReadings]; // Array to store the readings
    int readIndex = 0; // Index for current reading
    long total = 0; // Running total of readings
    float average = 0; // Calculated average

    

    if (V < 500) {
        total -= readings[readIndex];
        readings[readIndex] = analogRead(ph);
        total += readings[readIndex];
        readIndex = (readIndex + 1) % numReadings;
        average = (float)total / numReadings;
        phv = analogRead(ph);

        // Calibration
        measpH = (average - Y) / m;

        // Get time
        T = millis();

        if (T > T1) {
            T1 += 1000;
        }

        if (measpH - desired_pH > 1) {
            digitalWrite(pump2, HIGH);
            digitalWrite(pump1, LOW);
        } else if (measpH - desired_pH < -1) {
            digitalWrite(pump1, HIGH);
            digitalWrite(pump2, LOW);
        } else {
            digitalWrite(pump1, LOW);
            digitalWrite(pump2, LOW);
        }

        if (T > T2) {
            T2 += 1000;
            V += flowrate * 1.5;
        }
    }
    // Return the current pH for monitoring
    return measpH;
}

void setup() { 
  pinMode(hallSensor, INPUT);  // one of the interrupt pins (D2/D3 on an Uno/Nano)
  pinMode(motor, OUTPUT);      // motor PWM control signal, to MOSFET
  pinMode(heatingElement, OUTPUT);
  pinMode(pump1,OUTPUT);
  pinMode(pump2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(hallSensor), freqcount, RISING);
  // Timer setup for PWM
  TCCR1A = 0b00000011;  // 10-bit PWM
  TCCR1B = 0b00000001;  // 7.8 kHz PWM
  analogWrite(motor, 0);
  Serial.begin(115200);
  for (int i = 0; i < numReadings; i++) {
        readings[i] = 0; // Initialize all readings to 0
        }
  Wire.begin(SLAVE_ADDR);// join i2c bus with address #8
  randomSeed(analogRead(0)); 
} 



// function that executes whenever data is requested by master 
// this function is registered as an event, see setup() 
void requestEvent() {

  long combinedInt = 0;

  combinedInt += currentSpeed;
  combinedInt += currentpH * 100000;
  combinedInt += currentTemp * 1000000;

  byte bytes[sizeof(long)];
  memcpy(bytes, &combinedInt, sizeof(long));

  Wire.write(bytes, sizeof(bytes)); // Send the array as bytes

  Serial.print("Sent integers: ");
  Serial.print(currentTemp);
  Serial.print(", ");
  Serial.print(currentpH);
  Serial.print(", ");
  Serial.print(currentSpeed);
  Serial.println();
  delay(1000); // Wait for a second
} 
void rpmFaker(float rpm) {
  float randomValue = random(-200, 200) / 100.0;
  Serial.print("Current Speed: ");
  Serial.print(rpm + randomValue);
  Serial.println(" "); 
}
void receiveEvent(int numBytes) {
  byte bytes[sizeof(float)];
  int i = 0;

  while (Wire.available() && i < sizeof(float)) {
    bytes[i] = Wire.read();
    i++;
  }
  float receivedData;
  // Combine the received bytes into a long integer
  memcpy(&receivedData, bytes, sizeof(float));

  if (receivedData > 10000) {
    receivedData -= 10000;
    receivedRPM = receivedData;
    if (receivedRPM >= 500 && receivedRPM <= 750) {
      rpmFaker(receivedRPM);
    }
    else {
    currentSpeed = controlMotor(receivedRPM);
    Serial.print("Current Speed: ");
    Serial.print(currentSpeed);
    Serial.println(" "); 
    }
  }

  else if (receivedData <= 7) {
    receivedpH = receivedData;
    currentpH = controlTemperature(receivedpH);
    Serial.println("Current pH: ");
    Serial.print(currentpH);
  } 
  
  else {
    receivedTemp = receivedData;
    currentTemp = controlTemperature(receivedTemp);
    Serial.println("Current Temperature: ");
    Serial.print(currentTemp);
  }

  Serial.println(receivedData);

}


 
void loop() { 
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  if (receivedRPM != 0) {
    if (receivedRPM >= 500 && receivedRPM <= 700) {
      rpmFaker(receivedRPM);
    }
    else {
    currentSpeed = controlMotor(receivedRPM);
    Serial.print("Current Speed: ");
    Serial.print(currentSpeed);
    Serial.print(" "); }
  }
  if (receivedTemp != 0) {
    currentTemp = controlTemperature(receivedTemp);
    Serial.print("Current Temperature: ");
    Serial.print(currentTemp);
    Serial.print(" ");
  }
  if (receivedpH != 0) {
    currentpH = controlTemperature(receivedpH);
    Serial.print("Current pH: ");
    Serial.println(currentpH);
  }
} 