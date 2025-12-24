#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- PINLER ---
const int ENC_CLK_PIN = 2; 
const int ENC_DT_PIN  = 6; 
const int MOTOR_PWM_PIN = 3;
const int MOTOR_IN1_PIN = 4;
const int MOTOR_IN2_PIN = 5;

// --- DEĞİŞKENLER ---
volatile long encoderPosition = 0; 
long previousEncoderPosition = 0; // Hız hesabı için
int lastClkState;

unsigned long previousTime = 0;
const int samplingInterval = 50; //Hız hesabı için süre (50ms)

int motorSpeed = 130; // Motor PWM Hızı

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600); 

  pinMode(ENC_CLK_PIN, INPUT);
  pinMode(ENC_DT_PIN, INPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  lastClkState = digitalRead(ENC_CLK_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), updateEncoder, CHANGE);

  if (!mpu.begin()) {
    while (1);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // PLX-DAQ Başlıkları
  Serial.println("CLEARDATA"); 
  Serial.println("LABEL,BilgisayarSaati,ArduinoSuresi,EncoderCount,MotorRPM,SliderAccelY"); 
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();

  motorSur(motorSpeed, true); 

  if (currentTime - previousTime >= samplingInterval) {
    // --- HIZ HESAPLAMA (RPM) ---
    // Delta Zaman (Saniye cinsinden)
    float deltaTime = (currentTime - previousTime) / 1000.0;
    
    // Delta Encoder (Ne kadar döndü?)
    long currentEncoder = encoderPosition;
    long deltaEncoder = currentEncoder - previousEncoderPosition;
    
    // RPM Formülü: (DeltaAdım / TurBaşınaAdım) / (DeltaDakika)
    // KY-040 Encoder genellikle tur başına 20 adımdır.
    float rpm = (deltaEncoder / 20.0) * (60.0 / deltaTime);
    
    // Değerleri güncelle
    previousEncoderPosition = currentEncoder;
    previousTime = currentTime;

    // --- MPU VERİSİ ---
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // --- VERİ GÖNDERME ---
    Serial.print("DATA,TIME,"); 
    Serial.print(currentTime);  
    Serial.print(",");
    Serial.print(currentEncoder); 
    Serial.print(",");
    Serial.print(rpm); // Hesaplanan Motor Hızı
    Serial.print(",");
    Serial.println(a.acceleration.y); // Slider İvmesi
  }
}

// --- ALT FONKSİYONLAR ---
void updateEncoder() {
  int clkState = digitalRead(ENC_CLK_PIN);
  if (clkState != lastClkState) {
    if (digitalRead(ENC_DT_PIN) != clkState) {
      encoderPosition++;
    } else {
      encoderPosition--;
    }
  }
  lastClkState = clkState;
}

void motorSur(int speed, boolean direction) {
  analogWrite(MOTOR_PWM_PIN, speed);
  if (direction) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  } else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  }
}