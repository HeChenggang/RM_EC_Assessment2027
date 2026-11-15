# Portable control core: deliberately no HAL or board output side effects.
add_library(robot_logic STATIC
    control/pid.c control/motion_math.c control/robot_control.c
    remote/command.c safety/safety_gate.c
    chassis/mecanum.c chassis/chassis_control.c
    gimbal/gimbal_control.c imu/attitude.c communication/board_link.c)
target_include_directories(robot_logic PUBLIC
    control remote safety chassis gimbal imu communication)
target_compile_features(robot_logic PUBLIC c_std_11)
target_compile_options(robot_logic PRIVATE -Wall -Wextra -Werror)
target_link_libraries(robot_logic PUBLIC m)
