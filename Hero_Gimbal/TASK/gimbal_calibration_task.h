extern int Gimbal_Calibration_Read_Angle_Flag;	//用于标志读取校准后角度
extern int Gimbal_Encoder_Horizontal_Angle;		//水平角，需要校准后使用
extern int Gimbal_Calibration_Times;			
extern int Gimbal_Calibration_Target_Times;
extern int Motor_Alive_Flag;
void Gimbal_Calibration_Task(int S_Cnt, int MS_Cnt);