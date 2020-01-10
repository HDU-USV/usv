//1000hz调用   80-100的陷波处理

float Gyro_b1 = 0.9408;
float Gyro_b2 = -1.5918;
float Gyro_b3 = 0.9408;
float Gyro_a2 = -1.5918;
float Gyro_a3 = 0.8816;

typedef struct{
    float RollRate;
    float PitchRate;
    float YawRate;

  }d_IMU_Data;

d_IMU_Data  IMU_Data_old;
d_IMU_Data  *pIMU_Data_old=&IMU_Data_old;

d_IMU_Data  IMU_Data_pre_old;
d_IMU_Data *pIMU_Data_pre_old=&IMU_Data_pre_old;

d_IMU_Data  IMU_Data_filtered;
d_IMU_Data  *pIMU_Data_filtered=&IMU_Data_filtered;

d_IMU_Data  IMU_Data_filtered_old;
d_IMU_Data  *pIMU_Data_filtered_old=&IMU_Data_filtered_old;

d_IMU_Data  IMU_Data_filtered_pre_old;
d_IMU_Data  *pIMU_Data_filtered_pre_old=&IMU_Data_filtered_pre_old;

void IMUdata_notch_filter(void)
 {
   pIMU_Data_filtered->RollRate   = (Gyro_b1*pIMU_Data->RollRate  + Gyro_b2*pIMU_Data_old->RollRate  + Gyro_b3*pIMU_Data_pre_old->RollRate  - Gyro_a2* pIMU_Data_filtered_old->RollRate  - Gyro_a3*pIMU_Data_filtered_pre_old->RollRate);
   pIMU_Data_filtered->PitchRate  = (Gyro_b1*pIMU_Data->PitchRate + Gyro_b2*pIMU_Data_old->PitchRate + Gyro_b3*pIMU_Data_pre_old->PitchRate - Gyro_a2* pIMU_Data_filtered_old->PitchRate - Gyro_a3*pIMU_Data_filtered_pre_old->PitchRate);
   pIMU_Data_filtered->YawRate    = (Gyro_b1*pIMU_Data->YawRate   + Gyro_b2*pIMU_Data_old->YawRate   + Gyro_b3*pIMU_Data_pre_old->YawRate   - Gyro_a2* pIMU_Data_filtered_old->YawRate   - Gyro_a3*pIMU_Data_filtered_pre_old->YawRate);
   
   pIMU_Data_pre_old->RollRate  = pIMU_Data_old->RollRate;
   pIMU_Data_old->RollRate      = pIMU_Data->RollRate;
   pIMU_Data_pre_old->PitchRate = pIMU_Data_old->PitchRate;
   pIMU_Data_old->PitchRate     = pIMU_Data->PitchRate;
   pIMU_Data_pre_old->YawRate   = pIMU_Data_old->YawRate;
   pIMU_Data_old->YawRate       = pIMU_Data->YawRate;
   pIMU_Data_filtered_pre_old->RollRate  = pIMU_Data_filtered_old->RollRate;
   pIMU_Data_filtered_old->RollRate      = pIMU_Data_filtered->RollRate;
   pIMU_Data_filtered_pre_old->PitchRate = pIMU_Data_filtered_old->PitchRate;
   pIMU_Data_filtered_old->PitchRate     = pIMU_Data_filtered->PitchRate;
   pIMU_Data_filtered_pre_old->YawRate   = pIMU_Data_filtered_old->YawRate;
   pIMU_Data_filtered_old->YawRate       = pIMU_Data_filtered->YawRate;
   
 }