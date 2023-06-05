
// 宏定义设置
#define Get_MultiRound_Angle_ID 0x94
// 供外部函数读取的结构体
typedef struct
{
    uint8_t Encoder_Position;
    int Speed;
    int Current;
    int Tempreture;
    int Tempreture_Status; // 1为高温保护，0为正常
    int Voltage_Status; // 1为低压保护，0为正常
	int Round_Cnt;
    uint32_t Tatget_Speed;
    uint32_t Send_Status;
	uint64_t SingleRound_Angle;
	uint64_t Last_SingleRound_Angle;
	int Calibrated_Angle;
	float Converted_Calibrated_Angle;
	int Total_Angle;
} LK_Pitch_Motor_t;

typedef struct
{
    uint8_t Spin_Direction;
    uint16_t Max_Speed;
    uint16_t Tatget_Angle;
    uint16_t Last_Tatget_Angle;
    float Position_Kp;
    float Position_Ki;
    float Speed_Kp;
    float Speed_Ki;
    float IQ_Kp;
    float IQ_Ki;
} LK_Pitch_Motor_PID_t;

extern LK_Pitch_Motor_t LK_Pitch_Motor;
extern LK_Pitch_Motor_PID_t LK_Pitch_Motor_PID;
void Get_Pitch_Motor_SingleRound_Angle(void);
void Get_Pitch_Motor_Basic_Data(void);
void Send_Pitch_Motor_Add_Angle(int32_t pitch);
void Send_Pitch_Motor_Target_Speed(int32_t pitch);
void Send_Pitch_Motor_Start_Instruction(void);
void Send_Pitch_Motor_Shutdown_Instruction(void);