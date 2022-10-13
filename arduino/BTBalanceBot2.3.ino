#include <Arduino_LSM9DS1.h>



#define enA 10
#define inA1 11
#define inA2 12
#define encodeA1 5
#define encodeA2 6
#define enB 9
#define inB1 7
#define inB2 8
#define encodeB1 3
#define encodeB2 4
#define SPF 20
#define POSF 10
#define IPF 4
#define RATE_FAC 10
#define MAX_PWM 255
#define MAX_SETPOINT 50
#define MAX_ANGLE 70
#define MAX_SPEED 13
#define MAX_TURN 150
#define MAX_INPUT 11
#define MAX_TURNSPEED 25

float a_x, a_y, a_z, g_x, g_y, g_z; // variables for accelerometer raw data
float x_angle = 0;

float bkp = 20; //6
float bki = 0;
float bkd = 85; //17

float skp = 5; //3.5
float skd = 0; //0
float ski = 0; //0

float pkp = 0.00003; //0.00005
float pkd = 0.17; //0.85
float pki = 0.001;

float tukp1 = 4; // 4
float tukd1 = 0.035; // 0.03
float tuki1 = 0.2; 
float tukp2 = 1;
float tukd2 = 1;

float tkp = 1.7; //1.2
float tkd = 1.5; //0
float tki = 0.0005;


float set_point = 0;
int desiredSpeed = 0;
float turnspeed = 0;

bool reset = 0;
bool balance = 0;
volatile long int motorA = 0;
volatile long int motorB = 0;
int speedA, speedB;
long int posA, posB, ABrelation, wanted_posA = 0, wanted_posB = 0;
int moveA = 0, moveB = 0;
int16_t pwmA = 0, pwmB = 0;
int16_t balancePWM = 0;
float wantedSpeed;
float travelSpeed;
float botspeed;
long int wantedRelAB = 0;
int speedABRel = 0;

char tmp_str[200]; // temporay variable used in convert function
unsigned long timer = 0;
unsigned long timestamp = 0, milli;
double looptime = 0;


void setup() {
  Serial.begin(115200);
  Serial1.begin(230400);
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  pinMode(encodeA1, INPUT_PULLUP);
  pinMode(encodeA2, INPUT_PULLUP);
  pinMode(encodeB1, INPUT_PULLUP);
  pinMode(encodeB2, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encodeA1), encodeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encodeB1), encodeB, CHANGE);
  
}

void encodeA(){
  if (digitalRead(encodeA2) == digitalRead(encodeA1)){
    motorA++;
  }
  else{
    motorA--;
  }
}

void encodeB(){
  if (digitalRead(encodeB2) != digitalRead(encodeB1)){
    motorB++;
  }
  else{
    motorB--;
  }
}

void get_angle(float g_x, float a_y, float* x_angle, double looptime){
  static double y_acc, x_gyro;
  y_acc = a_y * 90;
  x_gyro = -g_x;

  *x_angle = (0.97)*(*x_angle + (x_gyro * (looptime / 1000))) + (0.03)*(y_acc);
  //sprintf(tmp_str, "y_acc: %.2f, x_gyro: %.2f, angle: %.2f",y_acc, x_gyro, x_angle);
}

void receive_vals(int* drive, float* turn){
  static int8_t buff[2];
  static int8_t pointsT[IPF], pointsD[IPF];
  static float Dout;
  static float Tout;
  static unsigned long failsafeTimer = 0;
  bool go_filter = 0;
  static int i = 0;
  if(Serial1.available()){
    int8_t in = Serial1.read();
    if(in == '\n' && i >= 2){
      i = 0;
    }
    else{
      buff[i] = in;
      i++;
    }
    if(i == 2){
      go_filter = 1;
    }
  }
  if(millis() > failsafeTimer + 200){
    *drive = 0;
    *turn = 0;
  }
  if(go_filter){
    failsafeTimer = millis();
    Dout = buff[0];
    Tout = buff[1];
    //sprintf(tmp_str, "drive: %3d, turn: %3d", buff[0], buff[1]);
    for(uint8_t i = IPF - 1; i > 0; i--){
      pointsT[i] = pointsT[i-1];
      pointsD[i] = pointsD[i-1];
      Tout += pointsT[i];
      Dout += pointsD[i];
    }
    pointsD[0] = buff[0];
    pointsT[0] = buff[1];
    Tout /= IPF;
    Dout /= IPF;
    *drive = MAX_SPEED * (Dout / MAX_INPUT);
    if(Tout > 0){
      *turn = ((Tout*Tout) / MAX_INPUT) / MAX_INPUT;
    }
    else{
      *turn = -((Tout*Tout) / MAX_INPUT) / MAX_INPUT;
    }
    //sprintf(tmp_str, "Dout: %3.2f, Tout: %3.2f, drive: %3.2f, turn: %3.2f", Dout, Tout, *drive, *turn);
  }
  //sprintf(tmp_str, "Dout: %2.3f, Tout: %2.3f, i: %d",Dout, Tout, i);
}

void safe_start(){
  static bool angleReady = 0;
  if(x_angle < 5 && x_angle > -5 && angleReady){
    balance = 1;
  }
  else if(x_angle < -MAX_ANGLE || x_angle > MAX_ANGLE){
    angleReady = 1;
    reset = 1;
    motorA = 0;
    motorB = 0;
    balance = 0;
    wanted_posA = 0;
    wanted_posB = 0;
    wantedRelAB = 0;
    pwmA = 0;
    pwmB = 0;
  }
}

void stand_still(){
  static bool stopped = 0;
  static bool upright = 0;
  static bool once = 0;
  if(desiredSpeed != 0){
    stopped = 0;
    upright = 0;
    once = 1;
  }
  else{
    if(botspeed <= 1 && botspeed >= -1){
      stopped = 1;
    }
    if(x_angle <= 7 && x_angle >= -7){
      upright = 1;
    }
  }
  if(once && stopped && upright){
    once = 0;
    wanted_posA = motorA;
    wanted_posB = motorB;
  }
  
}

void get_speedandpos(){
  static long int lastA, lastB;
  static int last_ABrel = 0;
  speedA = motorA - lastA;
  speedB = motorB - lastB;
  botspeed = (speedA + speedB) / 2;
  lastA = motorA;
  lastB = motorB;
  posA = motorA;
  posB = motorB;
  ABrelation = motorA - motorB;
  speedABRel = ABrelation - last_ABrel;
  last_ABrel = ABrelation;
  
}

int16_t speed_to_pwm(float wanted, float actual){
  static bool is_neg = 0;
  static float error, error_rate = 0, last_error = 0, cum_error = 0;
  error = wanted - actual;
  error_rate = error - last_error;
  cum_error += error;
  int out = (error * skp) + (error_rate * skd) + (cum_error * ski);
  last_error = error;
  
  if(reset){
    cum_error = 0;
    reset = 0;
  }

  if(cum_error < -MAX_PWM){
    cum_error = -MAX_PWM;
  }
  else if(cum_error > MAX_PWM){
    cum_error = MAX_PWM;
  }
  
  if(out < -MAX_PWM){
    out = -MAX_PWM;
  }
  else if(out > MAX_PWM){
    out = MAX_PWM;
  }
  //sprintf(tmp_str, "Angle: %3.2f, Wspeed: %4.2f, wanted: %3.2f, actual: %3d, speedA: %3d, speedB %3d, error: %3.2f, pwmA %4d, pwmB: %4d",x_angle, wantedSpeed, wanted, actual, speedA, speedB, error, pwmA, pwmB);
  //sprintf(tmp_str, "Wspeed: %4.2f, speed %4.2d", wanted, actual);
  //sprintf(tmp_str, "error: %.2f, rate: %.2f, cum: %.2f", (error * skp), (error_rate * skd), (cum_error * ski));
  //Serial.println(tmp_str);
  balancePWM = out;
  pwmA = out;
  pwmB = out;
  //sprintf(tmp_str, "wanted: %.2f, speedB %3d, error: %3.2f, pwmA %4d", wanted, speedB, error, pwmA);
}


void assign_turn(int (*turn)(float, float), float turnspeed, float desiredSpeed, float g_z){
  int turn_pwm = (*turn)(turnspeed, g_z);
  float speed_rate = desiredSpeed / MAX_SPEED;
  //float balance_rate = balancePWM / 255.0;
  float balance_rate = x_angle / MAX_ANGLE;
  float used_rate = 0;
  if(abs(speed_rate) >= abs(balance_rate)){
    used_rate = speed_rate;
  }
  else{
    used_rate = balance_rate;
  }
  pwmA += (turn_pwm + (abs(turn_pwm) * used_rate));
  pwmB -= (turn_pwm - (abs(turn_pwm) * used_rate));
   
  //sprintf(tmp_str, "turnspeed: %4.2f, speed_rate %4.2f, fakeA %4.2f, fakeB %4.2f, brate: %2.2f", turnspeed, speed_rate, (turn_pwm + (abs(turn_pwm) * used_rate)), -(turn_pwm - (abs(turn_pwm) * used_rate)), used_rate);
  sprintf(tmp_str, "balance_rate: %4.2f, speed_rate %4.2f, brate: %2.2f", balance_rate, speed_rate, used_rate);
}

/*
int turn(float turnspeed, float g_z){
  static float errorT, errorRateT, lastErrorT;
  int Tout;
   if(turnspeed != 0){
    wantedRelAB = ABrelation;
    Tout = (turnspeed * MAX_TURN) + (g_z * tukd1);
    if(Tout > MAX_TURN){
      Tout = MAX_TURN;
    }
    else if(Tout < -MAX_TURN){
      Tout = -MAX_TURN;
    }
  }
  else{
    errorT = wantedRelAB - ABrelation;
    errorRateT = errorT - lastErrorT;
    Tout = (errorT * tukp2) + (errorRateT * tukd2); 
    //sprintf(tmp_str,"out: %.2f", Tout);
    lastErrorT = errorT;
  
    if(Tout > MAX_TURN){
      Tout = MAX_TURN;
    }
    else if(Tout < -MAX_TURN){
      Tout = -MAX_TURN;
    }
  }
  //sprintf(tmp_str, "errorT: %3.2f, out: %4d, TS: %.2f", errorT, Tout, turnspeed);
  return Tout;
}
*/
int turn(float turnspeed, float g_z){
  static float errorT, errorRateT, lastErrorT, cumError = 0;
  float wanted_turn = turnspeed * MAX_TURNSPEED;
  int Tout;
   if(turnspeed != 0){
    wantedRelAB = ABrelation;
    errorT = wanted_turn - speedABRel;
    errorRateT = errorT - lastErrorT;
    cumError += errorT;
    
    Tout = (errorT * tukp1) + (errorRateT * tukd1) + (cumError * tuki1);
    if(Tout > MAX_TURN){
      Tout = MAX_TURN;
    }
    else if(Tout < -MAX_TURN){
      Tout = -MAX_TURN;
    }
    sprintf(tmp_str, "wantedT: %3.2f, errorT: %3.2f, out: %4d, speed: %3d, TS: %.2f", wanted_turn, errorT, Tout, speedABRel, turnspeed);
  }
  else{
    cumError = 0;
    errorT = wantedRelAB - ABrelation;
    errorRateT = errorT - lastErrorT;
    Tout = (errorT * tukp2) + (errorRateT * tukd2); 
    //sprintf(tmp_str,"out: %.2f", Tout);
    lastErrorT = errorT;
  
    if(Tout > MAX_TURN){
      Tout = MAX_TURN;
    }
    else if(Tout < -MAX_TURN){
      Tout = -MAX_TURN;
    }
  }
  return Tout;
}

float positionPD(){
  static int last_error = 0, error, error_rate, cum_error = 0;
  static int outs[POSF];
  static bool is_neg = 0;
  error = (wanted_posA + wanted_posB) - (posA + posB);
  error_rate = (error - last_error);
  cum_error += error;

  if(((posA + posB) < 0) != is_neg){
    cum_error = 0;
    is_neg = !is_neg;
  }

  if(cum_error < -500){
    cum_error = -500;
  }
  else if(cum_error > 500){
    cum_error = 500;
  }
  outs[0] = (error * abs(error)) * pkp + (error_rate * pkd) + (cum_error * pki);
  float out = 0;
  for(uint8_t i = POSF - 1; i > 0; i--){
    outs[i] = outs[i-1];
    out += outs[i];
  }
  out /= POSF;
 
  //sprintf(tmp_str, "set_point: %f, error: %d, errorR: %f", set_point, error, (error_rate * pkd));
  //sprintf(tmp_str, "error: %f, error_rate: %f", error * pkp, error_rate * pkd, (error * pkp) + (error_rate * pkd));
  

  if(out < -MAX_SPEED){
    out = -MAX_SPEED;
  }
  else if(out > MAX_SPEED){
    out = MAX_SPEED;
  }
  last_error = error;
  travelSpeed = out;
  //sprintf(tmp_str, "sp: %.2f, out: %.2f", set_point, out);
  //Serial.println(tmp_str);
  
}

int travelPD(){
  static float error, error_rate, last_error = 0, cum_error = 0;
  static bool is_neg = 0;
  error = travelSpeed - botspeed;
  error_rate = last_error - error;
  cum_error += error;
  /*if(((botspeed) < 0) != is_neg){
    cum_error = 0;
    is_neg = !is_neg;
  }*/

  if(!balance){
    cum_error = 0;
  }
  /*if(cum_error > MAX_SETPOINT){
    cum_error = MAX_SETPOINT;
  }
  else if(cum_error < -MAX_SETPOINT){
    cum_error = -MAX_SETPOINT;
  }*/

  float out = (error * tkp) + (error_rate * tkd) + (cum_error * tki);
  
  if(out > MAX_SETPOINT){
    out = MAX_SETPOINT;
  }
  else if(out < -MAX_SETPOINT){
    out = -MAX_SETPOINT;
  }
  //sprintf(tmp_str, "error: %3.2f, error_rate: %3.2f, sp: %2.2f", error, (error_rate * tkd), set_point);
  //sprintf(tmp_str, "Angle: %f, wanted: %4d, actual: %4d, error_rate: %2.2f",x_angle, (travelSpeedA + travelSpeedB), (speedA + speedB), (error_rate * tkd));
  //sprintf(tmp_str, "error: %f, SP: %f", error, set_point);
  //Serial.println(tmp_str);
  last_error = error;
  return out;
}

float setpoint_filter(float wanted_point){
  static float points[SPF];
  float out = wanted_point;
  for(uint8_t i = SPF - 1; i > 0; i--){
    points[i] = points[i-1];
    out += points[i];
  }
  points[0] = wanted_point;
  return out / (float)SPF;
  
}

int balancePID(float angle){
  static bool is_neg = 0;
  static float error, cumError, errorRate, lastError, lastRerror = 1;
  float out;
  error = angle - set_point;
  cumError += error;
  errorRate = (error - lastError);
  
  out = (error * bkp) + (cumError * bki) + (errorRate * bkd);
  //sprintf(tmp_str,"errorR: %.2f, error: %.2f, angle: %2.2f, ",  (errorRate * bkd),  (error * bkp), angle);
  lastError = error;
  
  if((angle < 0) != is_neg){
    cumError = 0;
    is_neg = !is_neg;
  }
  
  if(cumError > 100){
    cumError = 100;
  }
  else if(cumError < -100){
    cumError = -100;
  }
  
  //wantedSpeed = out;
  balancePWM = out;
  //sprintf(tmp_str,"WantedA: %5.2f, error: %.2f, cum: %.2f, rate: %.2f, setpoint: %.2f",  wantedSpeed, (error * bkp), (cumError * bki), (errorRate * bkd), set_point);
}


void loop() {
  receive_vals(&desiredSpeed, &turnspeed);
  float measure_time;
  if(millis() > timer + 10){
    milli = millis();
    timer = milli;
    looptime = (double)(milli - timestamp);
    timestamp = milli;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(g_x, g_y, g_z);
    }
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(a_x, a_y, a_z);
    }
    safe_start();
    get_angle(g_x, a_y, &x_angle, looptime);
    get_speedandpos();
    stand_still();
    if(balance){
      balancePID(x_angle);
      pwmA = balancePWM;
      pwmB = balancePWM;
      //speed_to_pwm(wantedSpeed, botspeed);
      assign_turn(turn, turnspeed, desiredSpeed, g_z);
      //sprintf(tmp_str, " pwmA: %4d, pwmB: %4d, wantedRel: %4d, Rel: %4d, motorA: %6d, motorB: %6d",pwmA, pwmB, wantedRelAB, ABrelation, motorA, motorB);
    }
    if(desiredSpeed != 0){
      wanted_posA = motorA;
      wanted_posB = motorB;
      travelSpeed = desiredSpeed;
    }
    else{
      positionPD(); 
    }
    set_point = setpoint_filter(travelPD());
    

    if(pwmA > 0){
      digitalWrite(inA1, HIGH);
      digitalWrite(inA2, LOW);
      analogWrite(enA, pwmA);
    }
    else{
      digitalWrite(inA1, LOW);
      digitalWrite(inA2, HIGH);
      analogWrite(enA, -pwmA);
    }
    if(pwmB > 0){
      digitalWrite(inB1, HIGH);
      digitalWrite(inB2, LOW);
      analogWrite(enB, pwmB);
    }
    else{
      digitalWrite(inB1, LOW);
      digitalWrite(inB2, HIGH);
      analogWrite(enB, -pwmB);
    }
    measure_time = millis() - milli;
    //sprintf(tmp_str, "looptime: %f, time: %f", looptime, measure_time);
    //sprintf(tmp_str, "A: %d, B: %d", digitalRead(encodeB1), digitalRead(encodeB2));
    //sprintf(tmp_str, "A: %2d, B: %2d, Angle: %2.2f", motorA, motorB, x_angle);
    //sprintf(tmp_str, "set_point: %f", set_point);
    //sprintf(tmp_str, "PWMA: %4d, PWMB: %4d, Angle %.2f", pwmA, pwmB, x_angle);
    //sprintf(tmp_str, "X: %.2f, Y: %.2f, Z: %.2f", g_x, g_y, g_z);
    //sprintf(tmp_str, "Wanted: %5d, actual: %5d, pwmA %3d, pwmB %3d, angle: %2.2f", wantedRelAB, ABrelation, pwmA, pwmB, x_angle);
    //sprintf(tmp_str, "Angle %f", x_angle);
    //sprintf(tmp_str, "Angle %d, T: %.2f",(int)x_angle, looptime);
    //sprintf(tmp_str, "botspeed: %3.2f, speedA: %3d, speedB %3d, travelspeed: %.2f",botspeed, speedA, speedB, travelSpeed);
    //sprintf(tmp_str, "Angle: %f, desiredS: %d, speed: %d, PWM: %d, set_point: %f, pos: %d, wantedpos: %d",x_angle, desiredSpeed, botspeed, pwm, set_point, posit, wanted_pos);
    //sprintf(tmp_str, "dspeed: %4d, turn: %2.2f",desiredSpeed, turnspeed);
    //sprintf(tmp_str, "Angle: %3.2f, Wspeed: %3.2f, speed %.2f, pwma: %4.2d",x_angle, wantedSpeed, botspeed, pwmA);
    //sprintf(tmp_str, "Angle %f, SP %3.2f", x_angle, set_point);
    //sprintf(tmp_str, "relspeed: %3d",speedABRel);
    Serial.println(tmp_str);
  }

}
