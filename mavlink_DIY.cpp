#include <mavlink_DIY.h>

HardwareSerial SerialPX4(2); // UART2：GPIO16(RX), GPIO17(TX)
float current_x = 0;
float current_y = 0;
float current_z = 0;
float current_roll = 0;
float current_pitch = 0;
float current_yaw = 0;

void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_heartbeat_t hb = {
    MAV_TYPE_GCS,
    MAV_AUTOPILOT_INVALID,
    0,
    0,
    0
  };

  mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &hb);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialPX4.write(buf, len);
}

//-------------------------------------------------------
void send_request_data_stream() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_request_data_stream_pack(
    system_id, component_id,
    &msg,
    target_system, target_component,
    MAV_DATA_STREAM_POSITION,
    2,
    1
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialPX4.write(buf, len);

  Serial.println("请求姿态数据流 (MAV_DATA_STREAM_POSITION @2Hz)");
}

//------------------------------------------------------------
void command_set_mode(uint8_t base_mode, uint32_t custom_mode) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_command_long_t cmd;
  memset(&cmd, 0, sizeof(cmd));

  cmd.target_system = target_system;
  cmd.target_component = target_component;
  cmd.command = MAV_CMD_DO_SET_MODE;
  cmd.confirmation = 0;
  cmd.param1 = base_mode;
  cmd.param2 = custom_mode;

  mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialPX4.write(buf, len);
  
  Serial.println("已发送设置模式指令！");
}

//---------------------------------------------------------------
void command_set_mode_test(uint8_t base_mode, uint8_t main_mode, uint8_t sub_mode){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_command_long_t cmd;
  memset(&cmd, 0, sizeof(cmd));

  cmd.target_system = target_system;
  cmd.target_component = target_component;
  cmd.command = MAV_CMD_DO_SET_MODE;
  cmd.confirmation = 0;
  cmd.param1 = base_mode;
  cmd.param2 = main_mode;
  cmd.param3 = sub_mode;

  mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialPX4.write(buf, len);
}

//--------------------------------------------------------------------
void command_arm() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_command_long_t cmd;
  memset(&cmd, 0, sizeof(cmd));

  cmd.target_system = target_system;
  cmd.target_component = target_component;
  cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
  cmd.confirmation = 0;
  cmd.param1 = 1; // 1 = arm

  mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialPX4.write(buf, len);
}

//-------------------------------------------------------------------
void command_position_target_local_ned_test(local_ned_mode mode, const float params[11]) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t type_mask = 0;
    
  switch(mode) {
      case POSITION_ONLY:
          type_mask = (1 << 3) | (1 << 4) | (1 << 5) | // ignore V
                      (1 << 6) | (1 << 7) | (1 << 8) | // ignore A
                      (1 << 9) | (1 << 10);            // ignore Yaw & Yawrate
          break;
          
      case VELOCITY_ONLY:
          type_mask = (1 << 0) | (1 << 1) | (1 << 2) | // ignore Position
                      (1 << 6) | (1 << 7) | (1 << 8) | // ignore A
                      (1 << 9) | (1 << 10);            // ignore Yaw & Yawrate
          break;
             
      case POSITION_YAW:
          type_mask = (1 << 3) | (1 << 4) | (1 << 5) | // ignore V
                      (1 << 6) | (1 << 7) | (1 << 8) | // ignore A
                      (1 << 10);                       // ignore Yawrate
          break;
          
      case VELOCITY_YAW:
          type_mask = (1 << 0) | (1 << 1) | (1 << 2) | // ignore Position
                      (1 << 6) | (1 << 7) | (1 << 8) | // ignore A
                      (1 << 10);                       // ignore Yawrate
          break;
      
      case POSITION_YAWRATE:
          type_mask = (1 << 3) | (1 << 4) | (1 << 5) | // ignore V
                      (1 << 6) | (1 << 7) | (1 << 8) | // ignore A
                      (1 << 9);                       // ignore Yaw
          break;
  }
  
  mavlink_msg_set_position_target_local_ned_pack(
      system_id, component_id, &msg,
      millis(),                // time_boot_ms
      target_system,
      target_component,
      MAV_FRAME_LOCAL_NED,     // NED frame
      type_mask,
      params[0], params[1], params[2], // x, y, z
      params[3], params[4], params[5], // vx, vy, vz
      params[6], params[7], params[8], // afx, afy, afz               
      params[9], params[10] // yaw, yaw_rate
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialPX4.write(buf, len);
}

//--------------------------------------------------------------------
void command_position_target_local_ned(float x, float y, float z) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t type_mask = 
      (1 << 3) |  // ignore vx
      (1 << 4) |  // ignore vy
      (1 << 5) |  // ignore vz
      (1 << 6) |  // ignore afx
      (1 << 7) |  // ignore afy
      (1 << 8) |  // ignore afz
      (1 << 9) |  // ignore yaw
      (1 << 10);  // ignore yaw_rate

  mavlink_msg_set_position_target_local_ned_pack(
    system_id, component_id, &msg,
    millis(),  // time_boot_ms
    target_system,
    target_component,
    MAV_FRAME_LOCAL_NED, // NED frame
    type_mask,
    x, y, z,
    0, 0, 0,  // vx, vy, vz
    0, 0, 0,  // afx, afy, afz
    0, 0      // yaw, yaw_rate
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialPX4.write(buf, len);

  Serial.println("Position target sent.");
}

//--------------------------------------------------------------------------
bool position_check(float target_x, float target_y, float target_z){
  const float threshold = 0.08;
  if (
      abs(current_z - target_z) <= threshold) {
    return true;  
  }

  return false;  
}

//---------------------------------------------------------------------------
void set_attitude_target(float roll, float pitch, float yaw, bool rate_control, float yaw_rate){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t type_mask = 0;

  float cr = cosf((roll / 360.0) * M_PI); // half angle
  float sr = sinf((roll / 360.0) * M_PI);
  float cp = cosf((pitch / 360.0) * M_PI);
  float sp = sinf((pitch / 360.0) * M_PI);
  float cy = cosf((yaw / 360.0) * M_PI);
  float sy = sinf((yaw / 360.0) * M_PI);

  float quaternion[4] = {
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy};

  if (rate_control)
      type_mask = 0b11000011;
  else{
      type_mask = 0b01000111;
      yaw_rate = 0;
  }

  mavlink_msg_set_attitude_target_pack(
      system_id, component_id, &msg,
      millis(),  
      target_system,
      target_component,
      type_mask,
      quaternion,   
      0.0f, 0.0f, yaw_rate,       
      0.0f,   // thrust not used  
      nullptr           
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialPX4.write(buf, len);
}

//------------------------------------------------------------------------
bool attitude_check(float target_roll, float target_pitch, float target_yaw){
  const float threshold = (7/180.0) * M_PI; //degree to radian

  if (
      abs(current_yaw - target_yaw) <= threshold) {
    return true;  
  }

  return false;  
}

//-------------------------------------------------------------------------
void receive_mavlink() {
  static mavlink_message_t msg;
  static mavlink_status_t status;

  while (SerialPX4.available() > 0) {
    uint8_t c = SerialPX4.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);
          Serial.println("收到PX4心跳！");
          Serial.print("System status: ");
          Serial.println(hb.system_status);
          Serial.print("Base mode: ");
          Serial.println(hb.base_mode);
          break;
        }

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
          mavlink_local_position_ned_t lpos;
          mavlink_msg_local_position_ned_decode(&msg, &lpos);
          current_x = lpos.x;
          current_y = lpos.y;
          current_z = lpos.z;
          printf("Local Position NED:\n");
          printf("X: %.2f m\n", lpos.x);
          printf("Y: %.2f m\n", lpos.y);
          printf("Z: %.2f m\n", lpos.z);
          printf("VX: %.2f m/s\n", lpos.vx);
          printf("VY: %.2f m/s\n", lpos.vy);
          printf("VZ: %.2f m/s\n", lpos.vz);
          break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
          mavlink_attitude_t att;
          mavlink_msg_attitude_decode(&msg, &att);
          current_roll = att.roll;
          current_pitch = att.pitch;
          current_yaw = att.yaw;
          printf("Attitude:\n");
          printf("Roll: %.2f rad (%.2f deg)\n", att.roll, att.roll * 180.0 / M_PI);
          printf("Pitch: %.2f rad (%.2f deg)\n", att.pitch, att.pitch * 180.0 / M_PI);
          printf("Yaw: %.2f rad (%.2f deg)\n", att.yaw, att.yaw * 180.0 / M_PI);
          printf("Roll Rate: %.2f rad/s\n", att.rollspeed);
          printf("Pitch Rate: %.2f rad/s\n", att.pitchspeed);
          printf("Yaw Rate: %.2f rad/s\n", att.yawspeed);
          break;
      }

        case MAVLINK_MSG_ID_COMMAND_ACK: {
          mavlink_command_ack_t ack;
          mavlink_msg_command_ack_decode(&msg, &ack);
        
          Serial.print("ACK for Command: ");
          Serial.println(ack.command);
          Serial.print("Result: ");
          switch (ack.result) {
            case MAV_RESULT_ACCEPTED:
              Serial.println("ACCEPTED (执行成功)");
              break;
            case MAV_RESULT_TEMPORARILY_REJECTED:
              Serial.println("TEMPORARILY_REJECTED (暂时拒绝)");
              break;
            case MAV_RESULT_DENIED:
              Serial.println("DENIED (拒绝)");
              break;
            case MAV_RESULT_UNSUPPORTED:
              Serial.println("UNSUPPORTED (不支持)");
              break;
            case MAV_RESULT_FAILED:
              Serial.println("FAILED (失败)");
              break;
            default:
              Serial.print("Unknown result: ");
              Serial.println(ack.result);
              break;
          }
          break;
        }

        default:
          Serial.print("接收到消息ID: ");
          Serial.println(msg.msgid);
          break;
      }
    }
  }
}
