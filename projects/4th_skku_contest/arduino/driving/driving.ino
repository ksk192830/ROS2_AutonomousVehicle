#include <EEPROM.h>

// 최대 입력 문자 수
const unsigned int MAX_INPUT = 15;

// 핀 번호 변수
const int STEERING_1 = 3;
const int STEERING_2 = 2;
const int FORWARD_RIGHT_1 = 7;
const int FORWARD_RIGHT_2 = 6;
const int FORWARD_LEFT_1 = 4;
const int FORWARD_LEFT_2 =5;
const int POT = A0;

// 조향 속도 상수
const int STEERING_SPEED = 150;
const int STEERING_SPEED_MID = 130;
const int STEERING_SPEED_NEAR_TARGET = 100;
const int CALIBRATION_SPEED = 100;
const int POT_SAMPLE_COUNT = 8;
const unsigned long CALIBRATION_TIMEOUT = 6000;
const unsigned long CALIBRATION_MIN_RUN_TIME = 500;
const int CALIBRATION_STABLE_SAMPLES = 10;
const int CALIBRATION_STABLE_TOLERANCE = 1;
const int CALIBRATION_MIN_RANGE = 50;

// 가변저항 값 범위
int resistance_most_left = 430;
int resistance_most_right = 325;

const unsigned long CALIBRATION_MAGIC = 0x534B4B55;
struct CalibrationData {
    unsigned long magic;
    int most_left;
    int most_right;
};

// 조향 최대 단계 수 (한 쪽 기준)
const int MAX_STEERING_STEP = 10;

// 제어 상태 변수
int angle = 0, resistance = 0, mapped_resistance = 0;
int left_speed = 0, right_speed = 0;

// 명령 주기 제한 변수
unsigned long lastCommandTime = 0; // 마지막 명령 처리 시간
const unsigned int COMMAND_INTERVAL = 50; // 명령 처리 간 최소 대기 시간(ms)

// 함수 선언
void steerRight(int speed);
void steerLeft(int speed);
void maintainSteering();
void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);
void processIncomingByte(const byte inByte);
void processData(const char *data);
int readPotentiometer();
bool loadCalibration();
bool autoCalibrate();
int findSteeringLimit(bool move_left);

void setup() {
    Serial.begin(115200);

    // 핀 모드 설정
    pinMode(POT, INPUT);
    pinMode(STEERING_1, OUTPUT);
    pinMode(STEERING_2, OUTPUT);
    pinMode(FORWARD_RIGHT_1, OUTPUT);
    pinMode(FORWARD_RIGHT_2, OUTPUT);
    pinMode(FORWARD_LEFT_1, OUTPUT);
    pinMode(FORWARD_LEFT_2, OUTPUT);

    setLeftMotorSpeed(0);
    setRightMotorSpeed(0);
    maintainSteering();

    if (!loadCalibration()) {
        delay(1000);
        autoCalibrate();
    }
}

void loop() {
    // 현재 시간 가져오기
    unsigned long currentTime = millis();

    // 직렬 데이터 처리
    while (Serial.available() > 0) {
        char incoming = Serial.read();
        processIncomingByte(incoming);
    }

    // 일정 시간 간격으로만 제어 명령 실행
    if (currentTime - lastCommandTime >= COMMAND_INTERVAL) {
        // 포텐셔미터 값을 읽어 조향 계산
        resistance = readPotentiometer();
        mapped_resistance = map(resistance, resistance_most_left, resistance_most_right, -MAX_STEERING_STEP, MAX_STEERING_STEP);
        int steering_error = mapped_resistance - angle;
        int steering_error_abs = abs(steering_error);
        int steering_speed = STEERING_SPEED;
        if (steering_error_abs == 1) {
            steering_speed = STEERING_SPEED_NEAR_TARGET;
        } else if (steering_error_abs == 2) {
            steering_speed = STEERING_SPEED_MID;
        }

        // 조향 상태에 따라 동작 제어
        if (mapped_resistance == angle) {
            maintainSteering();
        } else if (mapped_resistance > angle) {
            steerLeft(steering_speed);
        } else {
            steerRight(steering_speed);
        }

        // 모터 속도 설정
        setLeftMotorSpeed(left_speed);
        setRightMotorSpeed(right_speed);

        // 마지막 명령 시간 갱신
        lastCommandTime = currentTime;
    }
}

// 조향 제어 함수
void steerRight(int speed) {
    analogWrite(STEERING_1, speed);
    analogWrite(STEERING_2, LOW);
}

void steerLeft(int speed) {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, speed);
}

void maintainSteering() {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, LOW);
}

int readPotentiometer() {
    long sum = 0;
    for (int i = 0; i < POT_SAMPLE_COUNT; i++) {
        sum += analogRead(POT);
    }
    return sum / POT_SAMPLE_COUNT;
}

bool loadCalibration() {
    CalibrationData data;
    EEPROM.get(0, data);

    bool valid = data.magic == CALIBRATION_MAGIC
        && data.most_left >= 0
        && data.most_left <= 1023
        && data.most_right >= 0
        && data.most_right <= 1023
        && data.most_left > data.most_right
        && data.most_left - data.most_right >= CALIBRATION_MIN_RANGE;

    if (!valid) {
        return false;
    }

    resistance_most_left = data.most_left;
    resistance_most_right = data.most_right;
    return true;
}

int findSteeringLimit(bool move_left) {
    unsigned long start_time = millis();
    int previous = readPotentiometer();
    int stable_count = 0;

    while (millis() - start_time < CALIBRATION_TIMEOUT) {
        if (move_left) {
            steerLeft(CALIBRATION_SPEED);
        } else {
            steerRight(CALIBRATION_SPEED);
        }

        delay(50);
        int current = readPotentiometer();

        if (millis() - start_time >= CALIBRATION_MIN_RUN_TIME
            && abs(current - previous) <= CALIBRATION_STABLE_TOLERANCE) {
            stable_count++;
            if (stable_count >= CALIBRATION_STABLE_SAMPLES) {
                maintainSteering();
                return current;
            }
        } else {
            stable_count = 0;
        }

        previous = current;
    }

    maintainSteering();
    return readPotentiometer();
}

bool autoCalibrate() {
    left_speed = 0;
    right_speed = 0;
    setLeftMotorSpeed(0);
    setRightMotorSpeed(0);

    int measured_left = findSteeringLimit(true);
    Serial.print("CALIBRATION_LEFT:");
    Serial.println(measured_left);
    delay(300);

    int measured_right = findSteeringLimit(false);
    Serial.print("CALIBRATION_RIGHT:");
    Serial.println(measured_right);
    delay(300);

    if (measured_left <= measured_right
        || measured_left - measured_right < CALIBRATION_MIN_RANGE) {
        maintainSteering();
        Serial.print("CALIBRATION_FAILED:left=");
        Serial.print(measured_left);
        Serial.print(",right=");
        Serial.println(measured_right);
        return false;
    }

    resistance_most_left = measured_left;
    resistance_most_right = measured_right;

    CalibrationData data = {
        CALIBRATION_MAGIC,
        resistance_most_left,
        resistance_most_right
    };
    EEPROM.put(0, data);

    Serial.print("CALIBRATION_SUCCESS:left=");
    Serial.print(resistance_most_left);
    Serial.print(",right=");
    Serial.print(resistance_most_right);
    Serial.print(",range=");
    Serial.println(resistance_most_left - resistance_most_right);

    angle = 0;
    return true;
}

// 모터 속도 설정 함수
void setLeftMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(FORWARD_LEFT_1, speed);
        analogWrite(FORWARD_LEFT_2, LOW);
    } else {
        analogWrite(FORWARD_LEFT_1, LOW);
        analogWrite(FORWARD_LEFT_2, (-1) * speed);
    }
}

void setRightMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(FORWARD_RIGHT_1, speed);
        analogWrite(FORWARD_RIGHT_2, LOW);
    } else {
        analogWrite(FORWARD_RIGHT_1, LOW);
        analogWrite(FORWARD_RIGHT_2, (-1) * speed);
    }
}

// 직렬 데이터 처리
void processIncomingByte(const byte inByte) {
    static char input_line[MAX_INPUT];
    static unsigned int input_pos = 0;

    switch (inByte) {
        case '\n':
            input_line[input_pos] = 0; // 종료 문자 추가
            processData(input_line);  // 데이터 처리
            input_pos = 0; // 버퍼 초기화
            break;

        case '\r':
            break; // 캐리지 리턴 무시

        default:
            if (input_pos < (MAX_INPUT - 1)) {
                input_line[input_pos++] = inByte;
            }
            break;
    }
}

// 데이터 패킷 처리
void processData(const char *data) {
    if (strcmp(data, "calibrate") == 0) {
        autoCalibrate();
        return;
    }

    int sIndex = -1, lIndex = -1, rIndex = -1;

    // 명령 파싱
    for (int i = 0; data[i] != '\0'; i++) {
        if (data[i] == 's') sIndex = i;
        else if (data[i] == 'l') lIndex = i;
        else if (data[i] == 'r') rIndex = i;
    }

    if (sIndex != -1 && lIndex != -1 && rIndex != -1) {
        int newAngle = atoi(data + sIndex + 1);
        int newLeftSpeed = atoi(data + lIndex + 1);
        int newRightSpeed = atoi(data + rIndex + 1);

        // 명령 값 업데이트 (중복 명령 무시)
        if (newAngle != angle || newLeftSpeed != left_speed || newRightSpeed != right_speed) {
            angle = newAngle;
            left_speed = newLeftSpeed;
            right_speed = newRightSpeed;

            // 조향 값 제한
            if (angle > MAX_STEERING_STEP) angle = MAX_STEERING_STEP;
            else if (angle < -MAX_STEERING_STEP) angle = -MAX_STEERING_STEP;
        }
    }
}
