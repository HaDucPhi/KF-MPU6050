#include <Wire.h>
#include <math.h> 

#define MPU6050_ADDR 0x68 // Địa chỉ I2C mặc định của MPU6050
#define GYRO_XOUT_H 0x43  // Thanh ghi bắt đầu của dữ liệu gyroscope
#define ACCEL_XOUT_H 0x3B  // Thanh ghi bắt đầu của dữ liệu Accelometer
typedef struct {
  float GX;
  float GY;
  float GZ;
}Gyro_type;

typedef struct {
    float AX; // Gia tốc theo trục X
    float AY; // Gia tốc theo trục Y
    float AZ; // Gia tốc theo trục Z
} Accel_type;

typedef struct {
    float roll;
    float pitch;
} Angles_type;

Gyro_type Gyro;
Accel_type Accel;
Angles_type Angles;
bool Flag = 0;
float AgX,AgX_1;
float AgY,AgY_1;
float AgZ,AgZ_1;
float Ag_Y,Ag_X,Ag_Z;
// Phục vụ tính offset.
int k;
//
float Gx[20];
float Gy[20];
float Gz[20];
float Offset_gx, Offset_gy, Offset_gz;
float Ogx, Ogy, Ogz;
//
float Ax[20];
float Ay[20];
float Az[20];
float Offset_ax, Offset_ay, Offset_az;
float Oax, Oay, Oaz;
//
float Ts = 0.02;

/// Khai bao tham so cua bo loc Kalman.
float r = 1.2;
float q1 = 0.002; // Phương sai nhiễu đo lường và nhiễu hệ thống.
float q2 = 0.5; // Phương sai nhiễu đo lường và nhiễu hệ thống.
typedef struct {
  float h11;
  float h21;
}Matx21;

typedef struct {
  float h11;
  float h12;
  float h21;
  float h22;
}Matx22;


Matx21 Xk_k,Yk_k,Zk_k; // Vector trạng thái ước lượng.
Matx22 PXk_k,PYk_k,PZk_k; // Ma trận uớc lượng  phương sai.
// Hàm khởi tạo MPU6050

void MPU6050_BEGIN() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Thanh ghi PWR_MGMT_1
  Wire.write(0);    // Đặt chế độ ngủ về 0 (đánh thức cảm biến)
  Wire.endTransmission(true);
}

// Hàm đọc 16-bit từ một thanh ghi bắt đầu
int16_t READ_REG(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true); // Đọc 2 byte dữ liệu
  int16_t value = (Wire.read() << 8) | Wire.read(); // Gộp 2 byte lại thành giá trị 16-bit
  return value;
}

void setup() {
  // Cau hinh giao thuc.
  Serial.begin(9600); // Khởi tạo Serial để xuất dữ liệu
  Wire.begin();       // Khởi tạo giao tiếp I2C
  Wire.setClock(400000); // Đặt tốc độ I2C lên 400kHz
  MPU6050_BEGIN();     // Khởi tạo MPU6050
  // Tao ngat.
    cli();              // Tắt ngắt toàn cục
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler = 64
    TCNT1 = 60536; // Giá trị khởi tạo cho ngắt 1ms
    TIMSK1 |= (1 << TOIE1);  // Kích hoạt ngắt tràn Timer1
    sei();              // Bật ngắt toàn cục     
    PXk_k.h11 = 1;
    PXk_k.h22 = 1;
    PYk_k.h11 = 1;
    PYk_k.h22 = 1;
    PZk_k.h11 = 1;
    PZk_k.h22 = 1;  
           
}

void loop() {
  if(Flag == 1){
      Flag = 0;
      Gyro = Read_Gyro();
      Accel = Read_Accel();
      if( k < 20){
          Gx[k] = Gyro.GX;
          Gy[k] = Gyro.GY;
          Gz[k] = Gyro.GZ;
          //
          Ax[k] = Accel.AX;
          Ay[k] = Accel.AY;
          Az[k] = Accel.AZ;
          k++;
      }
      else if(k == 20){
        for(int i = 0; i<20;i++){
          Ogx = Ogx + Gx[i];
          Ogy = Ogy + Gy[i];
          Ogz = Ogz + Gz[i];
          Oax = Oax + Ax[i];
          Oay = Oay + Ay[i];
          Oaz = Oaz + Az[i];          
        }
        Offset_gx = -Ogx/20.0;
        Offset_gy = -Ogy/20.0;
        Offset_gz = -Ogz/20.0;
        Offset_ax = -Oax/20.0;
        Offset_ay = -Oay/20.0;
        Offset_az =  0.981 - Oaz/20;        
        k = 21;
      }
      else{
      Cal_Angle(Gyro);
      Angles = Cal_Roll_Pitch(Accel);
      Ag_X = Kalman_filter(Xk_k ,PXk_k ,Gyro.GX , Angles.roll,q1,q2,r);
      Ag_Y = Kalman_filter(Yk_k ,PYk_k ,Gyro.GY , Angles.pitch,q1,q2,r);
      Ag_Z = Kalman_Yaw_filter(Zk_k , PZk_k ,Gyro.GZ,q1,q2,r);
      
      Serial.print(Ag_Y);
      Serial.print(","); // Dấu phân cách giữa các giá trị
      Serial.print(Ag_X);
      Serial.print(","); // Dấu phân cách giữa các giá trị
      Serial.println(Ag_Z); // Dấu newline để kết thúc một chuỗi
   }
  }
}

Gyro_type Read_Gyro(){
  Gyro_type Out;
  int16_t GX_Raw,GY_Raw,GZ_Raw;
  GX_Raw = READ_REG(GYRO_XOUT_H);
  GY_Raw = READ_REG(GYRO_XOUT_H +2);
  GZ_Raw = READ_REG(GYRO_XOUT_H +4);
  Out.GX = GX_Raw/131.0 ;
  Out.GY = GY_Raw/131.0 ;
  Out.GZ = GZ_Raw/131.0 ;    
  return Out;
}

Accel_type Read_Accel(){
  Accel_type Out;
  int16_t AX_Raw, AY_Raw, AZ_Raw;
  // Đọc giá trị thô từ các thanh ghi
  AX_Raw = READ_REG(ACCEL_XOUT_H);       // Thanh ghi trục X
  AY_Raw = READ_REG(ACCEL_XOUT_H + 2);   // Thanh ghi trục Y
  AZ_Raw = READ_REG(ACCEL_XOUT_H + 4);   // Thanh ghi trục Z
  
  Out.AX = (float)AX_Raw / 16384.0 + Offset_ax ; // Tính gia tốc trục X
  Out.AY = (float)AY_Raw / 16384.0 + Offset_ay; // Tính gia tốc trục Y
  Out.AZ = (float)AZ_Raw / 16384.0 + Offset_az; // Tính gia tốc trục Z
    
  return Out;
}

Angles_type Cal_Roll_Pitch (Accel_type Accel) {
    Angles_type Angles;    
    // Tính Roll và Pitch
    // Tính Roll (xung quanh trục X)
    Angles.roll = atan2(Accel.AY, sqrt(Accel.AX * Accel.AX + Accel.AZ * Accel.AZ)) * 180.0 / M_PI;

    // Tính Pitch (xung quanh trục Y)
    Angles.pitch = atan2(-Accel.AX, sqrt(Accel.AY * Accel.AY + Accel.AZ * Accel.AZ)) * 180.0 / M_PI;

    if (AgY > 90) {
        Angles.pitch = 180 - Angles.pitch;
    }
    else if (AgY < -90) {
        Angles.pitch = -180 - Angles.pitch;
    }
    if (AgX > 90) {
        Angles.roll = 180 - Angles.roll;
    }
    else if (AgX < -90) {
        Angles.roll = -180 - Angles.roll;
    }
    return Angles;
}

void Cal_Angle(Gyro_type GYRO){ // Cong thuc so khai.
     AgX = AgX_1 + (Offset_gx + GYRO.GX)*Ts;
     AgY = AgY_1 + (Offset_gy + GYRO.GY)*Ts;
     AgZ = AgZ_1 + (Offset_gz + GYRO.GZ)*Ts;
     AgX_1 = AgX;
     AgY_1 = AgY;
     AgZ_1 = AgZ;
}

float Kalman_filter( Matx21& xk_k , Matx22& Pk_k , float wk , float Angle , float q1 , float q2 , float r) {
     Matx21 xk_k1;  // Vector trạng thái dự báo.
     Matx21 Kk;   // Ma trận độ lợi.
     Matx22 Pk_k1;  // Ma trận dự báo phương sai.
     // Dự báo vector trạng thái.
     xk_k1.h11 = xk_k.h11 + Ts*( wk - xk_k.h21) ;
     xk_k1.h21 = xk_k.h21;
     // Dự báo ma trận phương sai.
     Pk_k1.h11 = Pk_k.h11 - Ts*(Pk_k.h12 + Pk_k.h21) + Ts*Ts*Pk_k.h22 + q1;
     Pk_k1.h12 = Pk_k.h12 - Ts*Pk_k.h22;
     Pk_k1.h21 = Pk_k.h21 - Ts*Pk_k.h22;
     Pk_k1.h22 = Pk_k.h22 + q2;
    // Tính ma trận độ lợi.
    Kk.h11 = Pk_k1.h11/(Pk_k1.h11 + r);
    Kk.h21 = Pk_k1.h21/(Pk_k1.h11 + r);
    // Ước lượng vector trạng thái.
    xk_k.h11 = xk_k1.h11 + ( Angle - xk_k1.h11)*Kk.h11;
    xk_k.h21 = xk_k1.h21 + ( Angle - xk_k1.h11)*Kk.h21;
    // Ước lượng ma trận phương sai.
    Pk_k.h11 = Pk_k1.h11 - Kk.h11*Pk_k1.h11;
    Pk_k.h12 = Pk_k1.h12 - Kk.h11*Pk_k1.h12;
    Pk_k.h21 = Pk_k1.h21 - Kk.h21*Pk_k1.h11;
    Pk_k.h22 = Pk_k1.h22 - Kk.h21*Pk_k1.h12;
    return xk_k.h11;
}
//
float Kalman_Yaw_filter( Matx21& xk_k , Matx22& Pk_k , float wk , float q1 , float q2 , float r) {
     Matx21 xk_k1;  // Vector trạng thái dự báo.
     Matx21 Kk;   // Ma trận độ lợi.
     Matx22 Pk_k1;  // Ma trận dự báo phương sai.
     // Dự báo vector trạng thái.
     xk_k1.h11 = xk_k.h11 + Ts*xk_k.h21;
     xk_k1.h21 = xk_k.h21;
     // Dự báo ma trận phương sai.
     Pk_k1.h11 = Pk_k.h11 + Ts*(Pk_k.h12 + Pk_k.h21) + Ts*Ts*Pk_k.h22 + q1;
     Pk_k1.h12 = Pk_k.h12 + Ts*Pk_k.h22;
     Pk_k1.h21 = Pk_k.h21 + Ts*Pk_k.h22;
     Pk_k1.h22 = Pk_k.h22 + q2;
    // Tính ma trận độ lợi.
    Kk.h11 = Pk_k1.h12/(Pk_k1.h22 + r);
    Kk.h21 = Pk_k1.h22/(Pk_k1.h22 + r);
    // Ước lượng vector trạng thái.
    xk_k.h11 = xk_k1.h11 + ( wk - xk_k1.h21)*Kk.h11;
    xk_k.h21 = xk_k1.h21 + ( wk - xk_k1.h21)*Kk.h21;
    // Ước lượng ma trận phương sai.
    Pk_k.h11 = Pk_k1.h11 - Kk.h11*Pk_k1.h11;
    Pk_k.h12 = Pk_k1.h12 - Kk.h11*Pk_k1.h12;
    Pk_k.h21 = Pk_k1.h21 - Kk.h21*Pk_k1.h11;
    Pk_k.h22 = Pk_k1.h22 - Kk.h21*Pk_k1.h12;
    return xk_k.h11;
}
//
ISR (TIMER1_OVF_vect) 
{
  TCNT1 = 60536; // Tải lại giá trị cho 1ms
  if( Flag == 0){
  Flag = 1;
  }
}
