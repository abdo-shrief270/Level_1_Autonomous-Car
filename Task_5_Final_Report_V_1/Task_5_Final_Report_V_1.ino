// Ultrasonic Configuration
#define trigPin_R 8
#define trigPin_L 10
#define trigPin_F 12

#define echoPin_R 9
#define echoPin_L 11
#define echoPin_F 13

#define WindowSize 3
char TrigArr[] = { trigPin_R, trigPin_F, trigPin_L };
char EchoArr[] = { echoPin_R, echoPin_F, echoPin_L };

// Motor Driver Configuration
#define IN1 1
#define IN2 2
#define IN3 3
#define IN4 4
#define EN1 5
#define EN2 6

#define max_dist 4
#define max_track_speed_change 0.2 * speed
#define max_turn_speed_change 0.7 * speed

// Program Variables
long duration;
float distanceCm;
float DataArr[WindowSize][3];
float UltrData[3];
void  DataInit(void);
float UltrasonicRead(char Ultranum);
float UltrasonicRead_WithAverage(char Ultranum);

// PID Section
float elapsedTime, time, timePrev;
int i;
int speed = 235;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

/////////////////PID CONSTANTS/////////////////
double kp = 150;
double ki = 0;
double kd = 0;


void setup() {

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  pinMode(trigPin_R, OUTPUT);
  pinMode(echoPin_R, INPUT);
  pinMode(trigPin_F, OUTPUT);
  pinMode(echoPin_F, INPUT);
  pinMode(trigPin_L, OUTPUT);
  pinMode(echoPin_L, INPUT);

  Serial.begin(9600);
  time = millis();
  DataInit();
}

void loop() {

  timePrev = time;
  previous_error = error;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;

  for (char i = 0; i < 3; i++) {
    UltrData[i] = UltrasonicRead_WithAverage(i);
  }


  error = (UltrData[0] - UltrData[2]);

  if (error > 1)
    error = 1;
  if (error < -1)
    error = -1;

  pid_p = kp * error;

  pid_i += ki * error * elapsedTime;

  pid_d = kd * ((error - previous_error) / elapsedTime);

  PID = pid_p + pid_i + pid_d;


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

  pwmLeft = constrain(pwmLeft, 80, speed);
  pwmRight = constrain(pwmRight, 80, speed);

  analogWrite(EN1, pwmRight);
  analogWrite(EN2, pwmLeft);


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
  // Serial.print(" , Propotional Gain : ");
  // Serial.print(pid_p);
  // Serial.print(" , Integrale Gain : ");
  // Serial.print(pid_i);
  // Serial.print(" , Direvitive Gain : ");
  // Serial.print(pid_d);
  // Serial.print(" , Total PID : ");
  // Serial.print(PID);
  // Serial.print(" , PWM Left : ");
  // Serial.print(pwmLeft);
  // Serial.print(" , Pwm Right : ");
  // Serial.print(pwmRight);
  // Serial.print(" , Elapsed Time : ");
  // Serial.print(elapsedTime);
  // Serial.println();
}


//////////// Helper Functions

float UltrasonicRead(char Ultranum) {

  digitalWrite(TrigArr[Ultranum], LOW);
  delayMicroseconds(2);
  digitalWrite(TrigArr[Ultranum], HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigArr[Ultranum], LOW);

  duration = pulseIn(EchoArr[Ultranum], HIGH);
  distanceCm = (duration * 0.0343) / 2.0;
  if (distanceCm > max_dist * 100) {
    distanceCm = max_dist * 100;
  }
  return distanceCm / 100;
}


float UltrasonicRead_WithAverage(char Ultranum) {

  float sum = 0;
  for (int i = 0; i < WindowSize - 1; i++) {
    DataArr[i][Ultranum] = DataArr[i + 1][Ultranum];
    sum += DataArr[i][Ultranum];
  }

  DataArr[WindowSize - 1][Ultranum] = UltrasonicRead(Ultranum);
  sum += DataArr[WindowSize - 1][Ultranum];
  return (sum / WindowSize);
}

void DataInit(void) {
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < WindowSize; i++) {
      DataArr[i][j] = UltrasonicRead(j);
    }
  }
}
