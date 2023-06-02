#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "bsp_referee.h"
#include "remote_control.h"

unpack_data_t referee_unpack_obj;

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_robot_command_t							robot_command_t;

void referee_data_solve(uint8_t *frame);


void init_referee_struct_data(void)
{
	memset(&robot_command_t, 0, sizeof(ext_robot_command_t));
}
int temp;
char byte_global=0;
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;
	temp=fifo_s_used(&referee_fifo);

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
		byte_global=byte;
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
		case REFEREE_KEYBOARD_CONTROL_DATA_ID:
		{
			Transmission_Mode = Transmission_Mode_ON;
			memcpy(&robot_command_t, frame + index, sizeof(ext_robot_command_t));
			extern void get_transmission_remote_control_data(RC_ctrl_t *rc_ctrl);
			get_transmission_remote_control_data(&rc_ctrl);
		}
		break;
        default:
        {
            break;
        }
    }
}

void get_transmission_remote_control_data(RC_ctrl_t *rc_ctrl)
{
	rc_ctrl->mouse.x = robot_command_t.mouse_x;
	rc_ctrl->mouse.y = robot_command_t.mouse_y;
	rc_ctrl->mouse.z = robot_command_t.mouse_z;
	rc_ctrl->mouse.press_l = robot_command_t.left_button_down;
	rc_ctrl->mouse.press_r = robot_command_t.right_button_down;
	rc_ctrl->key.v = robot_command_t.keyboard_value;
}
