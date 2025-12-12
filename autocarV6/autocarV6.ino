// ======================================================
//                Maze Car Full Control v9
//        — 新 PID + 高稳定性 + 更多调试输出 —
// ======================================================

// =================== 参数区 ===================
const int EMA_SHIFT = 2;

float Kp = 0.08;
float Ki = 0.0;
float Kd = 1;

const int BASE_SPEED = 255;
const int MAX_SPEED = 255;
const int MIN_SPEED = 70;

const int SAFE_DIST = 20;
const int SLOW_DIST = 40;
const int CLEAR_DIST = 25;
const int IR_TOO_CLOSE = 980;
const int ALIGN_THRESHOLD = 75;
const int ALIGN_THRESHOLD_B = 100;

// =================== 红外引脚 ===================
#define IR_LF A2
#define IR_LB A3
#define IR_RF A0
#define IR_RB A1

// =================== 超声波 ===================
#define TRIG 10
#define ECHO 9

// =================== 电机 ===================
#define ENA 5
#define ENB 6
#define IN1 8
#define IN2 7
#define IN3 3
#define IN4 4


// =================== 全局 IR 滤波值 ===================
int irLF_f = 0, irLB_f = 0, irRF_f = 0, irRB_f = 0;
int dist = 30;
int targetLeft = 0;
int targetRight = 0;
int targetLeftBack = 0;
int targetRightBack = 0;
int rc = 0;


// PID
long prevError = 0;
float integral = 0;

bool turnR = 0;

// =================== 工具函数区 ===================
inline int clampPWM(int v) {
  return constrain(v, MIN_SPEED, MAX_SPEED);
}

// EMA
inline int emaFilter(int raw, int f, int SHIFT) {
  return f + ((raw - f) >> SHIFT);
}

// 读取 IR
inline void readIRSensors() {
  irLF_f = emaFilter(analogRead(IR_LF), irLF_f, EMA_SHIFT);
  irLB_f = emaFilter(analogRead(IR_LB), irLB_f, EMA_SHIFT);
  irRF_f = emaFilter(analogRead(IR_RF), irRF_f, EMA_SHIFT);
  irRB_f = emaFilter(analogRead(IR_RB), irRB_f, EMA_SHIFT);
}

// ======== 超声波测距（快速 + 高稳定）========
int readUltrasonic() {
  // ----- 超高速触发（5us）-----
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG, LOW);

  // ----- 使用 pulseInLong，超快且更稳定 -----
  unsigned long d = pulseInLong(ECHO, HIGH, 20000);  // 20ms 超时

  // 无回波
  if (d == 0) return -1;

  // ----- 整数快速换算：d/58 太慢 -----
  // 距离(cm) = d * 34 / 2000
  return (d * 34) / 2000;
}


// 电机
inline void drive(int L, int R) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, clampPWM(L));
  analogWrite(ENB, clampPWM(R));
}

inline void stopNow() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}


// ======================================================
//                      新 PID
// ======================================================
int computeSteering(int error) {
  integral += error;
  float d = error - prevError;

  float P = Kp * error;
  float I = Ki * integral;
  float D = Kd * d;

  prevError = error;

  float steering = P + I + D;

  // // ========== 输出 PID 三项 ==========
  // Serial.print(" PID[P:");
  // Serial.print(P, 3);
  // Serial.print(" I:");
  // Serial.print(I, 3);
  // Serial.print(" D:");
  // Serial.print(D, 3);
  // Serial.print("] ");

  return steering;
}


// =================== 标定 ===================
void autoCalibrateSide() {

  // 前方标定
  long sumL = 0, sumR = 0, sumF = 0, sumB = 0;

  for (int i = 0; i < 20; i++) {
    sumL += analogRead(IR_LF);
    sumR += analogRead(IR_RF);
    delay(1);
  }
  targetLeft = sumL / 20;
  targetRight = sumR / 20;
}

// =================== 判断转弯 ===================
bool decideTurnDirection() {
  const int N = 10;
  long sumLF = 0, sumRF = 0, sumLB = 0, sumRB = 0;

  for (int i = 0; i < N; i++) {
    sumLF += analogRead(IR_LF);
    sumRF += analogRead(IR_RF);
    sumLB += analogRead(IR_LB);
    sumRB += analogRead(IR_RB);
  }

  bool right = (sumRF + sumRB) / 2 < (sumLF + sumLB) / 2;
  Serial.print("Turn decision: ");
  Serial.println(right ? "RIGHT" : "LEFT");

  return right;
}

bool decideTurnDirectionUtrun() {
  const int N = 10;
  long sumLF = 0, sumRF = 0, sumLB = 0, sumRB = 0;

  for (int i = 0; i < N; i++) {
    sumLF += analogRead(IR_LF);
    sumRF += analogRead(IR_RF);
    sumLB += analogRead(IR_LB);
    sumRB += analogRead(IR_RB);
  }

  bool Ut = (sumLF/N > 80 && sumRF/N > 80) && (sumLB/N > 80 && sumRB/N > 80);


  return Ut;
}


// =================== 旋转（右=1，左=0） ===================
void rotateBase(bool turnRight) {
  Serial.print("Rotating: ");
  Serial.println(turnRight ? "RIGHT" : "LEFT");

  if (turnRight) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

// int rtime = millis();
// ============= 旋转直到齐平 + 超声波清晰 =============
void rotateUntilAligned(bool turnRight) {
  bool t180 = 0;
  // if (millis() - rtime > 3000){
  //   //rc = rc + 1;
  //   rtime = 0;
  // }
  int dista = 20;

  if( decideTurnDirectionUtrun()){
    rc = 0;
    t180 = 1;
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
    rotateBase(1);
    analogWrite(ENA, 130);
    analogWrite(ENB, 130);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    
  }else{
      if (!turnRight) {
      digitalWrite(13, HIGH);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, 0);
      analogWrite(ENB, 220);
      digitalWrite(13, LOW);
    } else {
      digitalWrite(12, HIGH);    
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, 220);
      analogWrite(ENB, 0);
      digitalWrite(12, LOW);
    }
    
  }
  
  Serial.println("=== Rotating until aligned ===");

  bool aligned = 0;
  while (true) {


    readIRSensors();
    int distr = readUltrasonic();

    dista = emaFilter(distr, dista, 2);

    //bool aligned = (abs(err) < ALIGN_THRESHOLD);
    if (turnRight) {
      aligned = (irLB_f*1.4  > ALIGN_THRESHOLD && irLF_f*1.2   > ALIGN_THRESHOLD_B);
    } else {
      aligned = (irRB_f*1.1 > ALIGN_THRESHOLD && irRF_f > ALIGN_THRESHOLD_B);
    }
    if (t180 ==  1){
      aligned = (irRB_f*1.1 > ALIGN_THRESHOLD && irRF_f > ALIGN_THRESHOLD_B) || (irLB_f*1.3 > ALIGN_THRESHOLD && irLF_f > ALIGN_THRESHOLD_B);
    }

    bool clear = (dista > CLEAR_DIST && dista < 200);
    // Serial.print(" t180:");
    //  Serial.print(t180);
    // Serial.print(" Tr:");
    // Serial.print(turnRight);
    // Serial.print(" AlignErr=");
    // Serial.print(err);
    //  Serial.print(" | dista=");
    //  Serial.print(dista);
    //  Serial.print(" | distr=");
    //  Serial.print(distr);
    //   Serial.print(" RF=");
    // Serial.print(irRF_f);
    // Serial.print(" LF=");
    // Serial.print(irLF_f);
    // Serial.print(" RB=");
    // Serial.print(irRB_f);
    // Serial.print(" LB=");
    // Serial.print(irLB_f);
    //  Serial.print(" | aligned:");
    //  Serial.print(aligned);
    //  Serial.print(" clear:");
    //  Serial.println(clear);
    Serial.println("");
    if (aligned) {
      digitalWrite(12, HIGH);
    } else {
      digitalWrite(12, LOW);
    }
    if (clear) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }
    if (aligned && clear) {
      digitalWrite(13, HIGH);
      digitalWrite(12, HIGH);
      break;
    }
    delay(2);
  }


  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  // Serial.println(">> Alignment success!");
}

// =================== setup ===================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);


  readIRSensors();
  if(decideTurnDirectionUtrun()){
        digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
    delay(500);
        digitalWrite(12, LOW);
    digitalWrite(13, LOW);
  }
  autoCalibrateSide();
}

// 状态机
// 0 ： 前进
// 1 : 准备转向
// 2 : 转向
int state = 0;
int timeS1 = 0;
float GC = 1;
// =================== loop ===================
void loop() {

  readIRSensors();
  dist = emaFilter(readUltrasonic(), dist, 2);  // 更快响应

  if (irLF_f > IR_TOO_CLOSE || irRF_f > IR_TOO_CLOSE || dist <= 7) {
    state = 3;
  }

  switch (state) {
    case 0:
      {
        int errorF = (irLF_f - irRF_f) - (targetLeft - targetRight);
        int steering = computeSteering(errorF);
        if (dist < SLOW_DIST ){
          GC = GC * dist/SLOW_DIST;
        }else{
          GC = 1;
        }
        if(GC*BASE_SPEED <= 120.){
          GC = 120./BASE_SPEED;
        }

        int L = (BASE_SPEED*GC + steering);
        int R = (BASE_SPEED*GC  - steering);
        drive(L, R);

        if (dist > 0 && dist < SAFE_DIST) {
          state = 2;
          GC = 1;
          stopNow();
          delay(1);
          turnR = decideTurnDirection();
        }

        break;
      }
    case 1:
      {
        break;
      }
    case 2:
      {
        break;
        if (turnR) {
          rotateUntilAligned(turnR);
          state = 0;
        } else {
          rotateUntilAligned(turnR);
          state = 0;
        }
        break;
      }
    case 3:
      {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 180);
        analogWrite(ENB, 180);
        delay(500);
        state = 0;
        break;
      }
    default:
      {
        Serial.print("DDD");
        break;
      }
  }
  // Serial.print(" state=");
  // Serial.print(state);
  // Serial.print(" RF=");
  // Serial.print(irRF_f);
  // Serial.print(" LF=");
  // Serial.print(irLF_f);
  // Serial.print(" RB=");
  // Serial.print(irRB_f);
  // Serial.print(" LB=");
  // Serial.print(irLB_f);
  // Serial.print(" T_RB=");
  // Serial.print(irRB_f + targetRightFB);
  // Serial.print(" T_LB=");
  // Serial.print(irLB_f + targetLeftFB);
  // Serial.print(" L=");
  // Serial.print(L);
  // Serial.print(" R=");
  // Serial.print(R);
  // Serial.print(" dist=");
  // Serial.print(dist);
  //   Serial.print(" GC=");
  // Serial.print(GC);
  // Serial.print(" f=");
  // Serial.print(wallAlignError(0));
  // Serial.print(" Steering=");
  // Serial.println(steering);


  // drive(L, R);
  //  Serial.println("");
  delay(10);
}
