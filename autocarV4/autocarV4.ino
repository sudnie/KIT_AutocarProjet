// ======================================================
//         Maze Car Full Control v4 (Auto Tune Edition)
//  — 自动阈值标定 / 自动 PID / 三点平均 + EMA双滤波 —
// ======================================================


// =================== 参数区 ===================
const int EMA_SHIFT = 2;
const int SAFE_DIST = 17;
const int CLEAR_DIST = 35;
const int IR_TOO_CLOSE  = 850;

int ALIGN_THRESHOLD = 40;     // 自动标定后会覆盖

float Kp = 0.013;             // 将被自动调节
float Ki = 0.0;
float Kd = 2.0;

int BASE_SPEED = 120;
const int MAX_SPEED  = 255;
const int MIN_SPEED  = 50;


// =================== 红外引脚 ===================
#define IR_LF A0
#define IR_LB A1
#define IR_RF A2
#define IR_RB A3

// 超声波
#define TRIG 10
#define ECHO 9

// 驱动
#define ENA 5
#define ENB 6
#define IN1 8
#define IN2 7
#define IN3 3
#define IN4 4


// =================== 全局 IR 滤波值 ===================
int irLF_raw[3]={0}, irRF_raw[3]={0};
int irLB_raw[3]={0}, irRB_raw[3]={0};

int irLF_f=0, irLB_f=0, irRF_f=0, irRB_f=0;

// 标定值（前方 L/R）
int targetLeft = 0;
int targetRight = 0;

// PID 变量
long prevError = 0;
float integral = 0;


// =================== 工具函数 ===================
inline int clampPWM(int v){
    return constrain(v,MIN_SPEED,MAX_SPEED);
}


// ----------- 三点平均 ----------
inline int movAvg(int *arr){
    return (arr[0] + arr[1] + arr[2]) / 3;
}

// ----------- EMA + 3点平均综合滤波 ----------
int filterIR(int pin, int *last3, int &ema){
    // shift: arr[2] ← arr[1] ← arr[0]
    last3[2]=last3[1];
    last3[1]=last3[0];
    last3[0]=analogRead(pin);

    int avg3 = movAvg(last3);

    ema = ema + ((avg3 - ema) >> EMA_SHIFT);

    return ema;
}


// ----------- 读取全部 IR（双重滤波） ----------
void readIRSensors(){
    irLF_f = filterIR(IR_LF, irLF_raw, irLF_f);
    irLB_f = filterIR(IR_LB, irLB_raw, irLB_f);
    irRF_f = filterIR(IR_RF, irRF_raw, irRF_f);
    irRB_f = filterIR(IR_RB, irRB_raw, irRB_f);
}


// ----------- 超声波 ----------
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
    analogWrite(ENA, clampPWM(L));
    analogWrite(ENB, clampPWM(R));
}

inline void stopNow(){
    analogWrite(ENA,0);
    analogWrite(ENB,0);
}


// =================== 自动标定阈值 ===================
void autoCalibrateThreshold(){
    long Lsum=0, Rsum=0, Lvar=0, Rvar=0;
    const int N=80;

    int Lprev=analogRead(IR_LF);
    int Rprev=analogRead(IR_RF);

    for(int i=0;i<N;i++){
        int L=analogRead(IR_LF);
        int R=analogRead(IR_RF);
        Lsum+=L; Rsum+=R;

        Lvar += abs(L-Lprev);
        Rvar += abs(R-Rprev);

        Lprev=L;
        Rprev=R;
        delay(5);
    }

    targetLeft  = Lsum/N;      // 中位点
    targetRight = Rsum/N;

    int noiseL = Lvar/N;
    int noiseR = Rvar/N;

    ALIGN_THRESHOLD = max(20, (noiseL+noiseR)*2);

    Serial.print("Auto Target L="); Serial.println(targetLeft);
    Serial.print("Auto Target R="); Serial.println(targetRight);
    Serial.print("Auto ALIGN TH="); Serial.println(ALIGN_THRESHOLD);
}


// =================== 自动 PID 调参 ===================
void autoTunePID(){
    // 小幅摆动测试系统响应
    long L0=0,R0=0;
    for(int i=0;i<10;i++){
        readIRSensors();
        L0 += irLF_f;
        R0 += irRF_f;
        delay(5);
    }
    L0/=10; R0/=10;

    // 轻微扰动
    drive(150, 100);
    delay(100);
    drive(100,150);
    delay(100);
    stopNow();

    // 测试后变化
    readIRSensors();
    long L1=irLF_f, R1=irRF_f;

    long dL=abs(L1-L0);
    long dR=abs(R1-R0);

    long gain = dL+dR;

    // 防止异常情况
    if(gain < 5) gain = 5;

    // Z-N 类似经验式
    Kp = 0.002 * gain;
    Kd = 0.3   * Kp;

    Serial.print("Auto PID: Kp="); Serial.println(Kp);
    Serial.print("Auto PID: Kd="); Serial.println(Kd);
}


// =================== PID ===================
int computePID(int error){
    integral += error;
    float d = error - prevError;
    prevError = error;
    return Kp*error + Ki*integral + Kd*d;
}


// =================== 自动判断转向（均值） ===================
bool decideTurnDirection(){
    const int N=10;
    long L=0,R=0;

    for(int i=0;i<N;i++){
        L+=analogRead(IR_LF);
        R+=analogRead(IR_RF);
        delay(1);
    }
    return (R>L);
}


// 后方齐平误差
inline int wallAlignError(bool useLeft){
    return useLeft ? (irLF_f - irLB_f) : (irRF_f - irRB_f);
}


// =================== 旋转直到齐平 ===================
void rotateBase(bool right){
    if(right){
        digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
        digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
    }else{
        digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
        digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
    }
}

void rotateUntilAligned(bool right){
    rotateBase(right);

    while(true){
        analogWrite(ENA,120);
        analogWrite(ENB,120);

        readIRSensors();
        int dist = readUltrasonic();
        int err = wallAlignError(!right);

        if(abs(err) < ALIGN_THRESHOLD &&
           dist > CLEAR_DIST && dist < 200) break;
    }

    stopNow();
    delay(100);

    drive(180,180);
    delay(200);
    stopNow();
}


// 紧急逃逸
inline void escapeRight(){ rotateBase(true);  drive(180,180); delay(300); stopNow(); }
inline void escapeLeft() { rotateBase(false); drive(180,180); delay(300); stopNow(); }



// =================== setup ===================
void setup(){
    Serial.begin(115200);

    pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
    pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
    pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);

    // 初读取用于 EMA 初值
    readIRSensors();
    delay(200);

    // ---------- 自动标定阈值 ----------
    autoCalibrateThreshold();

    // ---------- 自动调 PID ----------
    autoTunePID();

    Serial.println("Init Complete!");
}



// =================== loop ===================
void loop(){

    readIRSensors();
    int dist = readUltrasonic();

    // ----------- 前方阻挡 → 转弯 --------
    if(dist > 0 && dist < SAFE_DIST){
        bool turnRight = decideTurnDirection();
        rotateUntilAligned(turnRight);
        return;
    }

    // ----------- 两侧过近 → 逃逸 --------
    if(irLF_f > IR_TOO_CLOSE || irLB_f > IR_TOO_CLOSE) { escapeRight(); return; }
    if(irRF_f > IR_TOO_CLOSE || irRB_f > IR_TOO_CLOSE) { escapeLeft();  return; }


    // ======== PID 前方控制 ========
    int diff = (irLF_f - irRF_f) - (targetLeft - targetRight);
    int corr = computePID(diff);

    drive(BASE_SPEED + corr, BASE_SPEED - corr);


    // -------- 调试输出 --------
    Serial.print("LF=");Serial.print(irLF_f);
    Serial.print(" RF=");Serial.print(irRF_f);
    Serial.print(" diff=");Serial.println(diff);

    delay(1);
}
