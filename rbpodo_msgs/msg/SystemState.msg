# SystemState
# Please check the following for more details. 
# https://githubcom/RainbowRobotics/rbpodo/blob/main/include/rbpodo/data_type.hpp

float32     time                          # basic timer
float32[6]  jnt_ref                       # reference (desired) joint position (unit: rad)
float32[6]  jnt_ang                       # mesaured joint position (unit: rad)
float32[6]  jnt_cur                       # mesaured joint current (unit: amp)
float32[6]  tcp_ref                       # TCP posture info based on reference-joint-angles (unit: m & rad) [x, y, z, rx, ry, rz] (orientation = EulerZYX(rz, ry, rx))
float32[6]  tcp_pos                       # TCP posture info based on encoder-joint-angles (unit: m & rad)
float32[4]  analog_in                     # control box analog input measurement information of each channel (unit: Voltage)
float32[4]  analog_out                    # control box analog output information of each channel (unit: Voltage)
bool[16]    digital_in                    # control box digital input measurement information of each channel
bool[16]    digital_out                   # control box digital output information of each channel
float32[6]  jnt_temperature               # measured temperature of each joint (unit: celsius)
int32       task_pc                       # target program counter position during [STEP] function
int32       task_repeat                   # target program execution number in [PLAY] page
int32       task_run_id                   # running program counter position
int32       task_run_num                  # current program execution number in [PLAY] page
int32       task_run_time                 # time since the program started (unit: second)
int32       task_state                    # basic state of 'Program Execution'
float32     default_speed                 # default speed multiplier value of robot motion (=speed bar in UI) (value: 0 ~ 1)
int32       robot_state                   # move (motion) state
int32       information_chunk_1           # information chunk to deliver various state information (power and others) It consists of a combination of bits
int32[6]    jnt_info                      # basic state of each joint
bool        collision_detect_onoff        # out collision detection On/Off State (true=On / false=Off)
bool        is_freedrive_mode             # free-drive (Gravity-compensation) On/Off State (true=On / false=Off)
bool        real_vs_simulation_mode       # mode of operation: Simulation mode=true / Real Robot mode=false
int8        init_state_info               # robot arm activation (Initialization) stage info (0 -> 6)
int32       init_error                    # error code during the arm activation (return value for UI)
float32[2]  tfb_analog_in                 # robot-tool-flange analog input measurement information of each channel (unit: Voltage)
bool[2]     tfb_digital_in                # robot-tool-flange digital input measurement information of each channel 
bool[2]     tfb_digital_out               # robot-tool-flange digital output information of each channel
float32     tfb_voltage_out               # robot-Tool-Flage output voltage level (unit: Voltage)
bool        op_stat_collision_occur       # if true, out-collision is detected
int8        op_stat_sos_flag              # robot Arm device error code during operation
bool        op_stat_self_collision        # if true, self-collision is detected
bool        op_stat_soft_estop_occur      # pause state flag
int8        op_stat_ems_flag              # software (kinematics) emergency stop situation
int32       information_chunk_2           # information chunk to deliver various state information (please check document)
int32       information_chunk_3           # information chunk to deliver various state information (please check document)
bool[2]     inbox_trap_flag               # if true, detected by inbox # check-function
int8[2]     inbox_check_mode              # check-function mode of Inbox #
float32[6]  eft                           # external F/T force sensor value
int32       information_chunk_4           # information chunk to deliver various state information (please check document)
float32[4]  extend_io1_analog_in          # extended I/O board analog input measurement information of each channel (unit: Voltage)
float32[4]  extend_io1_analog_out         # extended I/O board analog output information of each channel (unit: Voltage)
uint32      extend_io1_digital_info       # extended I/O board digital input/output information (please check document)
float32[6]  aa_joint_ref                  # reference angle of each external-joint (auxiliary joint) (unit: rad)
uint32      safety_board_stat_info        # data chunk about the control box safety board