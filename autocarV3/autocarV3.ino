// ======================================================
//                Maze Car Full Control v5.5 (Optimized)
//       — 精简结构 / 完整功能 / 高效率 —
// ======================================================


// =================== 参数区 ===================
const int EMA_SHIFT = 4;

float Kp = 0.2;
float Ki = 0.0;
float Kd = 2.0;

const int BASE_SPEED = 120;
const int MAX_SPEED  = 255;
const int MIN_SPEED  = 50;

const int SAFE_DIST     = 17;
const int CLEAR_DIST    = 35;
const int IR_TOO_CLOSE  = 850;
const int ALIGN_THRESHOLD = 40;


// =================== 红外引脚 ===================
#define IR_LF A0
#define IR_LB A1
#define IR_RF A2
#define IR_RB A3

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

// 标定值（前方版）
int targetLeft = 0;
int targetRight = 0;

// PID
long prevError = 0;
float integral = 0;


// =================== 工具函数区 ===================

// ----------- 简化数字约束 ----------
inline int clampPWM(int v){
    return constrain(v,MIN_SPEED,MAX_SPEED);
}

// ----------- EMA 滤波 ----------
inline int emaFilter(int raw,int f){
    return f + ((raw - f) >> EMA_SHIFT);
}

// ----------- 读取所有 IR ----------
inline void readIRSensors(){
    irLF_f = emaFilter(analogRead(IR_LF), irLF_f);
    irLB_f = emaFilter(analogRead(IR_LB), irLB_f);
    irRF_f = emaFilter(analogRead(IR_RF), irRF_f);
    irRB_f = emaFilter(analogRead(IR_RB), irRB_f);
}


// ----------- 超声波读取 -----------
int readUltrasonic(){
    digitalWrite(TRIG,LOW); delayMicroseconds(2);
    digitalWrite(TRIG,HIGH); delayMicroseconds(10);
    digitalWrite(TRIG,LOW);

    unsigned long d = pulseIn(ECHO,HIGH,25000);
    return (d==0) ? -1 : d / 58;
}


// ----------- 电机 ----------
inline void drive(int L,int R){
    digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
    Serial.print(" PWML=");Serial.print( clampPWM(L));
    Serial.print(" PWMR=");Serial.print( clampPWM(R));
    analogWrite(ENA, clampPWM(L));
    analogWrite(ENB, clampPWM(R));
}

inline void stopNow(){
    analogWrite(ENA,0);
    analogWrite(ENB,0);
}


// =================== PID 控制 ===================
int computePID(int error){
    integral += error;
    float d = error - prevError;
    prevError = error;
    return Kp*error + Ki*integral + Kd*d;
}


// =================== 前方标定（只读 LF/RF） ===================
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


// =================== 判断转向（连续 N 次平均） ===================
bool decideTurnDirection(){
    const int N = 10;
    long sumLF=0, sumRF=0;

    for(int i=0;i<N;i++){
        sumLF += analogRead(IR_LF);
        sumRF += analogRead(IR_RF);
        delay(1);
    }
    return (sumRF > sumLF);   // RF 更近 → 右转
}


// =================== 返回左右侧墙的“前后差” ===================
inline int wallAlignError(bool useLeft){
    return useLeft ? (irLF_f - irLB_f) : (irRF_f - irRB_f);
}


// =================== 统一旋转函数（右=1，左=0） ===================
void rotateBase(bool turnRight){
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
    int stt = millis();

    rotateBase(turnRight);

    while(true){
        analogWrite(ENA,120);
        analogWrite(ENB,120);

        readIRSensors();
        int dist = readUltrasonic();
        int err = wallAlignError(!turnRight);

        bool aligned = (abs(err) < ALIGN_THRESHOLD);
        bool clear   = (dist > CLEAR_DIST && dist < 200);

        if(aligned && clear && (millis() - stt > 400)) break;
    }

    stopNow();
    delay(100);

    // 向新通道前进少量
    drive(180,180);
    delay(200);
    stopNow();
}


// =================== 紧急逃逸 ===================
inline void escapeRight(){ rotateBase(true);  drive(180,180); delay(300); stopNow(); }
inline void escapeLeft() { rotateBase(false); drive(180,180); delay(300); stopNow(); }



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
    int dist = readUltrasonic();

    // ----------- 前方阻挡 → 转弯 -------------
    if(dist > 0 && dist < SAFE_DIST){
        bool turnRight = decideTurnDirection();
        rotateUntilAligned(turnRight);
        return;
    }

    // ----------- 两侧过近 → 紧急逃逸 ----------
    if(irLF_f > IR_TOO_CLOSE || irLB_f > IR_TOO_CLOSE) { escapeRight(); return; }
    if(irRF_f > IR_TOO_CLOSE || irRB_f > IR_TOO_CLOSE) { escapeLeft();  return; }


    // ======== PID（只用 LF / RF）========
    int diff = (irLF_f - irRF_f) - (targetLeft - targetRight);
    int corr = computePID(diff);

    drive(BASE_SPEED - corr, BASE_SPEED + corr);


    // ----------- 串口调试输出 -----------
    Serial.print("LF=");Serial.print(irLF_f);
    Serial.print(" LB=");Serial.print(irLB_f);
    Serial.print(" RF=");Serial.print(irRF_f);
    Serial.print(" RB=");Serial.print(irRB_f);
    Serial.print(" dist=");Serial.print(dist);
    Serial.print(" diff=");Serial.println(diff);

    delay(1);
}
