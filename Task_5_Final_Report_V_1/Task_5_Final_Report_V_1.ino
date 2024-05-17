// Ultrasonic Configuration
#define trigPin_R 8 // Right ultrasonic sensor trigger pin
#define trigPin_L 10 // Left ultrasonic sensor trigger pin
#define trigPin_F 12 // Front ultrasonic sensor trigger pin

#define echoPin_R 9 // Right ultrasonic sensor echo pin
#define echoPin_L 11 // Left ultrasonic sensor echo pin
#define echoPin_F 13 // Front ultrasonic sensor echo pin

#define WindowSize 3 // Size of the window for averaging sensor readings
char TrigArr[] = { trigPin_R, trigPin_F, trigPin_L }; // Array of trigger pins
char EchoArr[] = { echoPin_R, echoPin_F, echoPin_L }; // Array of echo pins

// Motor Driver Configuration
#define IN1 1 // Motor driver input pin 1
#define IN2 2 // Motor driver input pin 2
#define IN3 3 // Motor driver input pin 3
#define IN4 4 // Motor driver input pin 4
#define EN1 5 // Motor driver enable pin 1
#define EN2 6 // Motor driver enable pin 2

#define max_dist 4 // Maximum distance in meters
#define max_track_speed_change 0.2 * speed // Maximum tracking speed change
#define max_turn_speed_change 0.7 * speed // Maximum turning speed change

// Program Variables
long duration; // Stores the duration of the ultrasonic pulse
float distanceCm; // Stores the calculated distance in cm
float DataArr[WindowSize][3]; // Array to store sensor data for averaging
float UltrData[3]; // Array to store current sensor readings

void DataInit(void); // Function prototype to initialize sensor data
float UltrasonicRead(char Ultranum); // Function prototype to read ultrasonic sensor
float UltrasonicRead_WithAverage(char Ultranum); // Function prototype to read ultrasonic sensor with averaging

// PID Section
float elapsedTime, time, timePrev; // Variables to calculate elapsed time
int i; // Loop counter
int speed = 235; // Initial speed value

float PID, pwmLeft, pwmRight, error, previous_error; // PID control variables
float pid_p = 0; // Proportional term
float pid_i = 0; // Integral term
float pid_d = 0; // Derivative term

// PID Constants
double kp = 150; // Proportional gain
double ki = 10; // Integral gain
double kd = 50; // Derivative gain

void setup() {
  // Motor driver pins configuration
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  
  // Set initial motor direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Ultrasonic sensor pins configuration
  pinMode(trigPin_R, OUTPUT);
  pinMode(echoPin_R, INPUT);
  pinMode(trigPin_F, OUTPUT);
  pinMode(echoPin_F, INPUT);
  pinMode(trigPin_L, OUTPUT);
  pinMode(echoPin_L, INPUT);

  Serial.begin(9600); // Initialize serial communication
  time = millis(); // Get the initial time
  DataInit(); // Initialize sensor data
}

void loop() {
  timePrev = time; // Store previous time
  previous_error = error; // Store previous error
  time = millis(); // Get the current time
  elapsedTime = (time - timePrev) / 1000; // Calculate elapsed time in seconds

  for (char i = 0; i < 3; i++) {
    UltrData[i] = UltrasonicRead_WithAverage(i); // Read ultrasonic sensors with averaging
  }

  error = (UltrData[0] - UltrData[2]); // Calculate tracking error

  if (error > 1) error = 1; // Limit error to maximum value of 1
  if (error < -1) error = -1; // Limit error to minimum value of -1

  pid_p = kp * error; // Calculate proportional term
  pid_i += ki * error * elapsedTime; // Calculate integral term
  pid_d = kd * ((error - previous_error) / elapsedTime); // Calculate derivative term

  PID = pid_p + pid_i + pid_d; // Calculate total PID value

  // Adjust PID value based on front sensor reading
  if (UltrData[1] > 3) {
    if (PID > max_track_speed_change) {
      PID = max_track_speed_change;
    } else if (PID < -max_track_speed_change) {
      PID = -max_track_speed_change;
    }
  } else {
    if (PID > max_turn_speed_change) {
      PID = max_turn_speed_change;
    } else if (PID < -max_turn_speed_change) {
      PID = -max_turn_speed_change;
    }
  }

  // Calculate PWM values for motors based on error
  if (error < 0) {
    pwmLeft = speed - abs(PID);
    pwmRight = speed;
  } else if (error > 0) {
    pwmRight = speed - abs(PID);
    pwmLeft = speed;
  } else {
    pwmLeft = speed;
    pwmRight = speed;
  }

  // Constrain PWM values within limits
  pwmLeft = constrain(pwmLeft, 80, speed);
  pwmRight = constrain(pwmRight, 80, speed);

  // Set motor speeds
  analogWrite(EN1, pwmRight);
  analogWrite(EN2, pwmLeft);

  // Serial debugging code (commented out)
  // Serial.print("Car Speed : ");
  // Serial.print(speed);
  // Serial.print(" , kp : ");
  // Serial.print(kp);
  // Serial.print(" , ki : ");
  // Serial.print(ki);
  // Serial.print(" , kd : ");
  // Serial.print(kd);
  // Serial.print(" , Left Distance : ");
  // Serial.print(UltrData[2]);
  // Serial.print(" , Front Distance : ");
  // Serial.print(UltrData[1]);
  // Serial.print(" , Right Distance : ");
  // Serial.print(UltrData[0]);
  // Serial.print(" , Tracking Error : ");
  // Serial.print(error);
  // Serial.print(" , Proportional Gain : ");
  // Serial.print(pid_p);
  // Serial.print(" , Integral Gain : ");
  // Serial.print(pid_i);
  // Serial.print(" , Derivative Gain : ");
  // Serial.print(pid_d);
  // Serial.print(" , Total PID : ");
  // Serial.print(PID);
  // Serial.print(" , PWM Left : ");
  // Serial.print(pwmLeft);
  // Serial.print(" , PWM Right : ");
  // Serial.print(pwmRight);
  // Serial.print(" , Elapsed Time : ");
  // Serial.print(elapsedTime);
  // Serial.println();
}

// Function to read the distance from an ultrasonic sensor
float UltrasonicRead(char Ultranum) {
  digitalWrite(TrigArr[Ultranum], LOW); // Set trigger pin low
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(TrigArr[Ultranum], HIGH); // Set trigger pin high
  delayMicroseconds(10); // Wait for 10 microseconds
  digitalWrite(TrigArr[Ultranum], LOW); // Set trigger pin low again

  duration = pulseIn(EchoArr[Ultranum], HIGH); // Measure the pulse duration
  distanceCm = (duration * 0.0343) / 2.0; // Calculate distance in cm
  if (distanceCm > max_dist * 100) { // Limit the distance to max_dist
    distanceCm = max_dist * 100;
  }
  return distanceCm / 100; // Return distance in meters
}

// Function to read the distance with averaging
float UltrasonicRead_WithAverage(char Ultranum) {
  float sum = 0;
  for (int i = 0; i < WindowSize - 1; i++) {
    DataArr[i][Ultranum] = DataArr[i + 1][Ultranum]; // Shift old data
    sum += DataArr[i][Ultranum]; // Add to sum
  }

  DataArr[WindowSize - 1][Ultranum] = UltrasonicRead(Ultranum); // Read new data
  sum += DataArr[WindowSize - 1][Ultranum]; // Add to sum
  return (sum / WindowSize); // Return average
}

// Function to initialize sensor data
void DataInit(void) {
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < WindowSize; i++) {
      DataArr[i][j] = UltrasonicRead(j); // Read initial data
    }
  }
}
