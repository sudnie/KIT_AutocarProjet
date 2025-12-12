// ======================================================
//                Maze Car Full Control v5
// ======================================================
//  保留全部功能，只增强旋转方向切换逻辑：
//  - 第 1~7 次固定左转
//  - 第 7 次后全部右转
// ======================================================


// =================== 参数区 ===================
const int EMA_SHIFT = 2;

float Kp = 0.013;
float Ki = 0.0;
float Kd = 2.0;

const int BASE_SPEED = 120;
const int MAX_SPEED  = 255;
const int MIN_SPEED  = 50;

const int SAFE_DIST  = 17;
const int CLEAR_DIST = 35;
const int IR_TOO_CLOSE = 850;


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
int irL_f = 0, irR_f = 0;
int target = 0;

long prevError = 0;
float integral = 0;

int rotateCount = 0;   // 旋转计数

// =================== EMA滤波 ===================
int emaFilter(int raw, int f) {
    return f + ((raw - f) >> EMA_SHIFT);
}


// =================== 超声波 ===================
int readUltrasonic() {
    digitalWrite(TRIG, LOW); delayMicroseconds(2);
    digitalWrite(TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    unsigned long duration = pulseIn(ECHO, HIGH, 25000);
    if (duration == 0) return -1;
    return duration / 58;
}


// =================== 电机驱动 ===================
void drive(int pwmL, int pwmR) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);

    analogWrite(ENA, pwmL);
    analogWrite(ENB, pwmR);
}

void stopNow() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}


// =================== PID ===================
int computePID(int error) {
    integral += error;
    float derivative = error - prevError;
    prevError = error;
    return Kp * error + Ki * integral + Kd * derivative;
}


// =================== PID → PWM ===================
void convertToPWM(int corr, int &pwmL, int &pwmR) {
    pwmL = BASE_SPEED + corr;
    pwmR = BASE_SPEED - corr;

    pwmL = constrain(pwmL, MIN_SPEED, MAX_SPEED);
    pwmR = constrain(pwmR, MIN_SPEED, MAX_SPEED);
}


// =================== 自动标定 ===================
void autoCalibrate() {
    long sum = 0;
    for (int i = 0; i < 60; i++) {
        sum += (analogRead(IR_R) - analogRead(IR_L));
        delay(10);
    }
    target = sum / 60;
    Serial.print("Calibrated = ");
    Serial.println(target);
}


// =================== 旋转方向逻辑 ===================
bool getRotateDirection() {
    if (irL_f > 500 && irL_f > 500)
        return false;   // false = 左转
    else
        return true;    // true = 右转
}


// =================== 旋转直到安全 ===================
void rotateUntilSafe(bool turnRight) {

    Serial.println(turnRight ? "Rotate Right" : "Rotate Left");
    unsigned long start = millis();

    if (turnRight) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    }

    while (true) {
        analogWrite(ENA, 120);
        analogWrite(ENB, 120);

        int dist = readUltrasonic();

        if (dist > CLEAR_DIST && dist < 200) break;
        if (millis() - start > 3000) break;
    }

    stopNow();

    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);

    analogWrite(ENA, 180);
    analogWrite(ENB, 180);

    delay(300);  // 闭眼前进

    stopNow();
}


// =================== 紧急反转 ===================
void escapeRight() {
    Serial.println("ESCAPE RIGHT!");

    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

    analogWrite(ENA, 180);
    analogWrite(ENB, 180);
    delay(300);

    stopNow();
}

void escapeLeft() {
    Serial.println("ESCAPE LEFT!");

    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);

    analogWrite(ENA, 180);
    analogWrite(ENB, 180);
    delay(300);

    stopNow();
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
    delay(300);
    autoCalibrate();
}


// =================== loop ===================
void loop() {

    // ---- 前方检测 ----
    int dist = readUltrasonic();
    irL_f = emaFilter(analogRead(IR_L), irL_f);
    irR_f = emaFilter(analogRead(IR_R), irR_f);

    if (dist > 0 && dist < SAFE_DIST) {

        rotateCount++;                    //计次 +1
        bool turnRight = getRotateDirection();   // 获取方向

        rotateUntilSafe(turnRight);
        return;
    }


    // ---- 红外 ----

    int diff = (irL_f - irR_f) - target;


    // --- 一侧太近：反转 ---
    if (irL_f > IR_TOO_CLOSE) {
        escapeRight();
        return;
    }
    if (irR_f > IR_TOO_CLOSE) {
        escapeLeft();
        return;
    }


    // ---- PID ----
    int corr = computePID(diff);

    // ---- 差速 ----
    int pwmL, pwmR;
    convertToPWM(corr, pwmL, pwmR);
    drive(pwmL, pwmR);


    // ---- 调试 ----
    Serial.print("irL_f="); Serial.print(irL_f);
    Serial.print("irR_f="); Serial.print(irR_f);
    Serial.print("diff="); Serial.print(diff);
    Serial.print(" corr="); Serial.print(corr);
    Serial.print(" pwmL="); Serial.print(pwmL);
    Serial.print(" pwmR="); Serial.print(pwmR);
    Serial.print(" dist="); Serial.println(dist);

    delay(1);
}
