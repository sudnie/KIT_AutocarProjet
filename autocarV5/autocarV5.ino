// ======================================================
//                Maze Car Full Control v9
//        — 新 PID + 高稳定性 + 更多调试输出 —
// ======================================================

// =================== 参数区 ===================
const int EMA_SHIFT = 2;

float Kp = 0.1;
float Ki = 0.0;
float Kd = 1.5;

const int BASE_SPEED = 160;
const int MAX_SPEED  = 200;
const int MIN_SPEED  = 50;


const int SAFE_DIST     = 18;
const int CLEAR_DIST    = 28;
const int IR_TOO_CLOSE  = 850;
const int ALIGN_THRESHOLD = 70;
const float LTT  = 1.8;
const float RTT  = 1;

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
int irLF_f=0, irLB_f=0, irRF_f=0, irRB_f=0;
int dist = 20;
int targetLeft = 0;
int targetRight = 0;

// PID
long prevError = 0;
float integral = 0;


// =================== 工具函数区 ===================
inline int clampPWM(int v){ return constrain(v,MIN_SPEED,MAX_SPEED); }

// EMA
inline int emaFilter(int raw,int f, int SHIFT){ return f + ((raw - f) >> SHIFT); }

// 读取 IR
inline void readIRSensors(){
    irLF_f = emaFilter(analogRead(IR_LF), irLF_f,EMA_SHIFT);
    irLB_f = emaFilter(analogRead(IR_LB), irLB_f,EMA_SHIFT);
    irRF_f = emaFilter(analogRead(IR_RF), irRF_f,EMA_SHIFT);
    irRB_f = emaFilter(analogRead(IR_RB), irRB_f,EMA_SHIFT);
}

// 超声波
int readUltrasonic(){
    digitalWrite(TRIG,LOW); delayMicroseconds(2);
    digitalWrite(TRIG,HIGH); delayMicroseconds(10);
    digitalWrite(TRIG,LOW);

    unsigned long d = pulseIn(ECHO,HIGH,25000);
    return (d==0) ? -1 : d / 58;
}

// 电机
inline void drive(int L,int R){
    digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
    analogWrite(ENA, clampPWM(L));
    analogWrite(ENB, clampPWM(R));
}

inline void stopNow(){
    analogWrite(ENA,0);
    analogWrite(ENB,0);
}


// ======================================================
//                      新 PID
// ======================================================
int computeSteering(int error){
    integral += error;
    float d = error - prevError;

    float P = Kp * error;
    float I = Ki * integral;
    float D = Kd * d;

    prevError = error;

    float steering = P + I + D;

    // ========== 输出 PID 三项 ==========
    Serial.print(" PID[P:");
    Serial.print(P, 3);
    Serial.print(" I:");
    Serial.print(I, 3);
    Serial.print(" D:");
    Serial.print(D, 3);
    Serial.print("] ");

    return steering;
}


// =================== 前方标定 ===================
void autoCalibrateSide(){
    long sumL=0, sumR=0;

    for(int i=0;i<60;i++){
        sumL += analogRead(IR_LF);
        sumR += analogRead(IR_RF);
        delay(5);
    }
    targetLeft  = sumL/60;
    targetRight = sumR/60;

    Serial.print("Cal LF="); Serial.println(targetLeft);
    Serial.print("Cal RF="); Serial.println(targetRight);
}


// =================== 判断转弯 ===================
bool decideTurnDirection(){
    const int N = 70;
    long sumLF=0, sumRF=0;

    for(int i=0;i<N;i++){
        sumLF += analogRead(IR_LF);
        sumRF += analogRead(IR_RF);
        delay(1);
    }

    bool right = (sumRF < sumLF);
    Serial.print("Turn decision: ");
    Serial.println(right ? "RIGHT" : "LEFT");

    return right;
}


// 前后差
inline int wallAlignError(bool useLeft){ return useLeft ? (irLF_f - irLB_f*RTT) : (irRF_f - irRB_f*LTT); }


// =================== 旋转（右=1，左=0） ===================
void rotateBase(bool turnRight){
    Serial.print("Rotating: ");
    Serial.println(turnRight ? "RIGHT" : "LEFT");

    if(turnRight){
        digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
        digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
    }else{
        digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
        digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
    }
}


// ============= 旋转直到齐平 + 超声波清晰 =============
void rotateUntilAligned(bool turnRight){
    int dista = 10;

    rotateBase(turnRight);

    Serial.println("=== Rotating until aligned ===");

    while(true){
        analogWrite(ENA,150);
        analogWrite(ENB,150);

        readIRSensors();
        int distr = readUltrasonic();

        dista = emaFilter(distr, dista, 2);
        int err = wallAlignError(turnRight);
        Serial.print(" Tr:");
        Serial.print(turnRight);
        Serial.print(" AlignErr=");
        Serial.print(err);
        Serial.print(" | dista=");
        Serial.print(dista);
        Serial.print(" | distr=");
        Serial.print(distr);
        bool aligned = (abs(err) < ALIGN_THRESHOLD);
        bool clear   = (dista > CLEAR_DIST && dista < 200);

        Serial.print(" | aligned:");
        Serial.print(aligned);
        Serial.print(" clear:");
        Serial.println(clear);

        if(aligned && clear) break;
    }

    Serial.println(">> Alignment success!");

    stopNow();
    delay(100);

    drive(180,180);
    delay(200);
    stopNow();
}


// =================== 逃逸 ===================
inline void escapeRight(){
    Serial.println("Escape RIGHT (left wall too close!)");
    digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
    analogWrite(ENA, 120);
    analogWrite(ENB, 120); delay(100);
    stopNow();
}
inline void escapeLeft(){
    Serial.println("Escape LEFT (right wall too close!)");
    digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
    digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
    analogWrite(ENA, 120);
    analogWrite(ENB, 120); delay(100);
    stopNow();
}


// =================== setup ===================
void setup(){
    Serial.begin(115200);

    pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
    pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
    pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);

    readIRSensors();
    delay(300);
    autoCalibrateSide();
}


// =================== loop ===================
void loop(){

    readIRSensors();
    dist = emaFilter(readUltrasonic(), dist,4);


    // ----------- 前方阻挡 → 转弯 -------------
    if(dist > 0 && dist < SAFE_DIST){
        Serial.println("Front blocked! Turning...");
        bool turnRight = decideTurnDirection();
        rotateUntilAligned(turnRight);
        return;
    }
    
    // ----------- 两侧过近 → 逃逸 ----------
    //if(irLF_f > IR_TOO_CLOSE || irLB_f > IR_TOO_CLOSE){ escapeRight(); return; }
    //if(irRF_f > IR_TOO_CLOSE || irRB_f > IR_TOO_CLOSE){ escapeLeft();  return; }

    // =============== 新 PID 控制（LF vs RF） ===============
    int error = (irLF_f - irRF_f) - (targetLeft - targetRight);
    int steering = computeSteering(error);

    int L = BASE_SPEED + steering;
    int R = BASE_SPEED - steering;

    Serial.print(" RF="); Serial.print(irRF_f);
    Serial.print(" LF="); Serial.print(irLF_f);
    Serial.print(" RB="); Serial.print(irRB_f);
    Serial.print(" LB="); Serial.print(irLB_f);
    Serial.print(" L="); Serial.print(L);
    Serial.print(" R="); Serial.print(R);
    Serial.print(" dist="); Serial.print(dist);
    Serial.print(" Error="); Serial.print(error);
    Serial.print(" Steering="); Serial.println(steering);
    

    drive(L, R);
    delay(10);
}
