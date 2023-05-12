#ifndef __chassis_task_H
#define __chassis_task_H

#define CHASSIS_REMOTE_CLOSE   	 1    //关闭遥控器                                左高位
#define CHASSIS_NORMAL           3    //正常模式                                  左中位
#define CHASSIS_SPIN             2    //小陀螺模式                                    左低位

void remote_chassis(void);
#endif
