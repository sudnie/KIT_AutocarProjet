// ======================================================
//                Maze Car Full Control v3
//   - 左右 TCRT5000 带 EMA + 防抖 + 自动标定
//   - 超声波前方测距
//   - PID 居中控制
//   - 默认最高速 255，靠近障碍物减速
//   - L298N 双电机驱动
// ======================================================

// =================== 参数区 ===================
const int EMA_SHIFT = 4;        // EMA 滤波强度
const int DEBOUNCE_COUNT = 0;   // 防抖次数

float Kp = 0.01;
float Ki = 0;
float Kd = 2.0;

const int BASE_SPEED = 120;     // 中心速度
const int MAX_SPEED  = 255;     // 最大PWM
const int MIN_SPEED  = 50;      // 最低PWM，防止轮子停转

const int MAX_DIST = 200;       // 超声波有效测距
const int MIN_DIST = 17;        // 小于此距离明显减速

const unsigned long TURN_COOLDOWN = 700; // 旋转冷却时间ms

// =================== 引脚定义 ===================
#define IR_L A0
#define IR_R A1

#define TRIG 10
#define ECHO 9

#define ENA 5
#define ENB 6
#define IN1 8
#define IN2 7
#define IN3 3
#define IN4 4

// =================== 全局变量 ===================
int irL_f = 0, irR_f = 0;        // EMA输出
int irL_db = 0, irR_db = 0;      // 防抖值
int irL_buf = 0, irR_buf = 0;    // 防抖缓冲计数

int turnCount = 0;               // 左转计数
bool turnRightMode = false;      // 左/右转模式切换
int target = 0;                  // 自动标定中心值（R-L差）

unsigned long lastTurnTime = 0;  // 上一次旋转时间

long prevError = 0;
float integral = 0;

// =================== EMA滤波 ===================
int emaFilter(int raw, int f) {
    return f + ((raw - f) >> EMA_SHIFT);
}

// =================== 防抖 ===================
int debounceFilter(int raw, int &buf, int &state) {
    if (raw == state) buf = 0;
    else {
        buf++;
        if (buf >= DEBOUNCE_COUNT) {
            state = raw;
            buf = 0;
        }
    }
    return state;
}

// =================== 超声波测距 ===================
int readUltrasonic() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    unsigned long duration = pulseIn(ECHO, HIGH, 25000); // 超时25ms
    if (duration == 0) return -1; // 没检测到回波
    return duration / 58;          // cm
}

// =================== 差速驱动 ===================
void motorDrive(int pwmL, int pwmR) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, pwmL);
    analogWrite(ENB, pwmR);
}

// =================== PID计算 ===================
int computePID(int error) {
    integral += error;
    float derivative = error - prevError;
    prevError = error;
    float out = Kp * error + Ki * integral + Kd * derivative;

    // 限幅，PID输出可以正负
    if (out > MAX_SPEED) out = MAX_SPEED;
    if (out < -MAX_SPEED) out = -MAX_SPEED;
    return (int)out;
}

// =================== PID → 左右轮 ===================
void convertToWheelPWM(int corr, int &pwmR, int &pwmL) {
    pwmL = BASE_SPEED + corr;  // 正值 → 右偏 → 左轮加速
    pwmR = BASE_SPEED - corr;  // 正值 → 右偏 → 右轮减速

    // 限制PWM范围
    if (pwmL > MAX_SPEED) pwmL = MAX_SPEED;
    if (pwmL < MIN_SPEED) pwmL = MIN_SPEED;
    if (pwmR > MAX_SPEED) pwmR = MAX_SPEED;
    if (pwmR < MIN_SPEED) pwmR = MIN_SPEED;
}

// =================== 自动标定 ===================
void autoCalibrate() {
    long sum = 0;
    for (int i = 0; i < 60; i++) {
        int l = analogRead(IR_L);
        int r = analogRead(IR_R);
        sum += (r - l);
        delay(10);
    }
    target = sum / 60;
    Serial.print("Center calibrated = ");
    Serial.println(target);
}

// =================== 基础动作 ===================
void St() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void Tr(int speed = 255, int duration = 500) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    delay(duration);
    St();
}

void Tl(int speed = 255, int duration = 500) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    delay(duration);
    St();
}

// =================== setup ===================
void setup() {
    Serial.begin(115200);

    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

    pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);

    irL_f = analogRead(IR_L);
    irR_f = analogRead(IR_R);

    delay(200);
    autoCalibrate();
}

// =================== loop ===================
void loop() {
    int pwmL = 0, pwmR = 0;
    unsigned long now = millis();

    // 读取红外
    int rawL = analogRead(IR_L);
    int rawR = analogRead(IR_R);

    irL_f = emaFilter(rawL, irL_f);
    irR_f = emaFilter(rawR, irR_f);

    int cleanL = debounceFilter(irL_f, irL_buf, irL_db);
    int cleanR = debounceFilter(irR_f, irR_buf, irR_db);

    int diff = (cleanL - cleanR) - target;

    // PID
    int corr = computePID(diff);






123
   int dist = readUltrasonic();
    bool inCooldown = (now - lastTurnTime < TURN_COOLDOWN);

    if (dist > 0 && dist < MIN_DIST && !inCooldown) {
        // 障碍物 → 转向
        if (!turnRightMode) {
            Tr(200, 200);
            turnCount++;
            if (turnCount >= 7) {
                Tr(200, 200);  // 180度
                turnCount = 0;
                turnRightMode = true;
            }
        } else {
            Tl(200, 200);
        }
        pwmL = pwmR = 130;
        motorDrive(pwmL, pwmR);
        delay(500);
        pwmL = pwmR = 0;
        lastTurnTime = now;
    } else if (!inCooldown) {
        convertToWheelPWM(corr, pwmR, pwmL);
        motorDrive(pwmL, pwmR);
    } else {
        pwmL = pwmR = 0;
        motorDrive(pwmL, pwmR);
    }

    // 串口调试
    Serial.print("L="); Serial.print(cleanL);
    Serial.print(" R="); Serial.print(cleanR);
    Serial.print(" diff="); Serial.print(diff);
    Serial.print(" corr="); Serial.print(corr);
    Serial.print(" pwmL="); Serial.print(pwmL);
    Serial.print(" pwmR="); Serial.print(pwmR);
    Serial.print(" dist="); Serial.print(dist);
    Serial.print(" turnCount="); Serial.print(turnCount);
    Serial.print(" turnRightMode="); Serial.print(turnRightMode);
    Serial.print(" inCooldown="); Serial.println(inCooldown);

    delay(10);
}
