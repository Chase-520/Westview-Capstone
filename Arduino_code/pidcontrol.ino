#include <Adafruit_BNO08x.h>

// 电机控制引脚
const int R_MOTOR_DIR = 4;  // AIN1 (Direction pin 1)
const int R_MOTOR_PWM = 25;  // PWMA (Speed control)
const int L_MOTOR_DIR = 2;  // AIN1 (Direction pin 1)
const int L_MOTOR_PWM = 0;  // PWMA (Speed control)

// BNO08x IMU引脚
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1

// PID参数
float Kp = 5.0;   // 比例增益 - 调整这个值以获得响应
float Ki = 0.05;  // 积分增益
float Kd = 0.5;   // 微分增益

float setpoint = -87.5;  // 目标pitch角度为0度
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float output = 0.0;

unsigned long lastTime = 0;
float sampleTime = 0.01;  // 10ms采样时间

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;  // 200Hz

void setup() {
  // 设置电机引脚为输出
  pinMode(L_MOTOR_DIR, OUTPUT);
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_DIR, OUTPUT);
  pinMode(R_MOTOR_PWM, OUTPUT);
  
  Serial.begin(115200);
  while (!Serial) delay(10);  // 等待串口连接
  
  Serial.println("A4990 Motor Control with BNO08x IMU PID");
  Serial.println("Starting in 2 seconds...");
  delay(2000);
  
  // 尝试初始化BNO08x
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
  
  // 设置报告类型
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, reportIntervalUs)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  
  Serial.println("Reading events");
  delay(100);
  
  // 初始化时间
  lastTime = millis();
  
  Serial.println("PID Control Active - Target Pitch: 0°");
}



// 电机控制函数
void setMotor(int speed, bool forwardDirection) {
  // 确保速度在0-255范围内
  if (speed > 255) speed = 255;
  if (speed < 0) speed = 0;
  
  // 设置方向
  digitalWrite(L_MOTOR_DIR, forwardDirection ? HIGH : LOW);
  digitalWrite(R_MOTOR_DIR, forwardDirection ? LOW : HIGH);
  
  // 设置PWM速度
  analogWrite(L_MOTOR_PWM, speed);
  analogWrite(R_MOTOR_PWM, speed);
}

// 四元数转欧拉角函数
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);
  
  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  
  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {
  // 检查IMU是否有新数据
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    if (!bno08x.enableReport(reportType, reportIntervalUs)) {
      Serial.println("Could not enable stabilized remote vector");
    }
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // 将四元数转换为欧拉角
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    
    // 计算时间间隔
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;  // 转换为秒
    
    // 只有在达到采样时间时才计算PID
    if (deltaTime >= sampleTime) {
      // PID计算
      error = setpoint - ypr.roll;
      
      // 积分项（带抗饱和）
      integral += error * deltaTime;
      // 限制积分项
      if (integral > 100) integral = 100;
      if (integral < -100) integral = -100;
      
      // 微分项
      derivative = (error - lastError) / deltaTime;
      
      // PID输出
      output = Kp * error + Ki * integral + Kd * derivative;
      
      // 限制输出到电机范围 (-255 到 255)
      if (output > 255) output = 255;
      if (output < -255) output = -255;
      
      // 保存当前误差供下次使用
      lastError = error;
      
      // 更新时间
      lastTime = currentTime;
      
      // 控制电机
      if (abs(output) < 10) {
        // 死区 - 如果输出很小，停止电机以避免抖动
        setMotor(0, true);
      } else {
        // 根据输出正负设置方向
        if (output > 0) {
          setMotor(abs(output), true);  // 正向
        } else {
          setMotor(abs(output), false); // 反向
        }
      }
      
      // 打印调试信息（可降低频率避免串口堵塞）
      static unsigned long lastPrintTime = 0;
      if (currentTime - lastPrintTime > 100) {  // 每100ms打印一次
        Serial.print("Pitch: ");
        Serial.print(ypr.pitch);
        Serial.print("° | Error: ");
        Serial.print(error);
        Serial.print("° | Output: ");
        Serial.print(output);
        Serial.print(" | Motor: ");
        if (abs(output) < 10) {
          Serial.println("STOP");
        } else {
          Serial.println(output > 0 ? "FORWARD" : "BACKWARD");
        }
        lastPrintTime = currentTime;
      }
    }
  }
}
