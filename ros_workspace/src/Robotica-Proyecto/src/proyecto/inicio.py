import control_robot

control = control_robot.ControlRobot()

angles = control.load_motor_angles_from_yaml("angulo_inicial")
control.move_motors(angles)