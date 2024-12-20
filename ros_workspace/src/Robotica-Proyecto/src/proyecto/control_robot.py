#!/usr/bin/python3

import sys
import copy
import rospy
import yaml  # Importa la biblioteca YAML
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Int32
from moveit_commander.conversions import pose_to_list
from typing import List
from geometry_msgs.msg import Pose, PoseStamped

class ControlRobot:
    def __init__(self) -> None:
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()

        self.capture_control_pub = rospy.Publisher('/capture_control', Int32, queue_size=10)
        rospy.Subscriber('/numero', Int32, self.numero_callback)


        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)

        self.contador_dibujados = 0
        self.contador_filas = 0


    def numero_callback(self, msg):
        self.dibuja_numero(msg.data)

    # Coger ángulos de los motores
    def get_motor_angles(self) -> list:
        return self.move_group.get_current_joint_values()

    # Mover motores en base a una lista de ángulos
    def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    # Trabajar con poses
    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    # Crear cubo
    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)

    # Mover en línea recta
    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, True)

        mensaje_publicar = Int32() # Crear un mensaje de tipo String
        mensaje_publicar.data = 1
        self.capture_control_pub.publish(mensaje_publicar)
        self.move_group.execute(plan, wait=wait)

        mensaje_publicar.data = 0
        self.capture_control_pub.publish(mensaje_publicar)
        return
    
    # Crear suelo
    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2, 2, .05))
        

    # Guardar configuración en YAML
    def save_motor_angles_to_yaml(self, name: str) -> None:
        angles = self.get_motor_angles()
        with open("/home/laboratorio/ros_workspace/src/Robotica-Proyecto/src/proyecto/paint_angles.yaml", 'w') as f:
            yaml.dump({name: angles}, f)

    # Cargar configuración desde YAML
    def load_motor_angles_from_yaml(self, name: str) -> list:
        with open("/home/laboratorio/ros_workspace/src/Robotica-Proyecto/src/proyecto/motor_angles.yaml", 'r') as f:
            data = yaml.load(f, Loader=yaml.Loader)
            return data.get(name, [])
        
    def load_paint_angles_from_yaml(self, name: str) -> list:
        with open("/home/laboratorio/ros_workspace/src/Robotica-Proyecto/src/proyecto/paint_angles.yaml", 'r') as f:
            data = yaml.load(f, Loader=yaml.Loader)
            return data.get(name, [])

    # Guardar pose en YAML
    def save_pose_to_yaml(self, filename: str) -> None:
        pose = self.get_pose()
        pose_data = {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            }
        }
        with open(filename, 'w') as f:
            yaml.dump({"robot_pose": pose_data}, f)

    # Cargar pose desde YAML
    def load_pose_from_yaml(self, filename: str) -> Pose:
        with open(filename, 'r') as f:
            data = yaml.load(f, Loader=yaml.Loader)
            position = data["robot_pose"]["position"]
            orientation = data["robot_pose"]["orientation"]
            pose = Pose()
            pose.position.x = position["x"]
            pose.position.y = position["y"]
            pose.position.z = position["z"]
            pose.orientation.x = orientation["x"]
            pose.orientation.y = orientation["y"]
            pose.orientation.z = orientation["z"]
            pose.orientation.w = orientation["w"]
            return pose
        
    def dibuja_numero(self, numero):

        trayectorias = {
            0: [
                (0, 0, -0.05), (0.04, 0, 0), (0, -0.03, 0), (-0.04, 0, 0), (0, 0.03, 0), (0, 0, 0.05)
            ],
            1: [
                (0.04, -0.03, 0), (0, 0, -0.05), (-0.04, 0, 0), (0, 0, 0.05)
            ],
            2: [
                (0.04, 0, 0), (0, 0, -0.05), (0, -0.03, 0), (-0.02, 0, 0), (0, 0.03, 0), (-0.02, 0, 0), (0, -0.03, 0), (0, 0, 0.05)
            ],
            3: [
                (0, 0, -0.05), (0, -0.03, 0), (0.02, 0, 0), (0, 0.03, 0), (0, -0.03, 0), (0.02, 0, 0), (0, 0.03, 0), (0, 0, 0.05)
            ],
            4: [
                (0.04, 0, 0), (0, 0, -0.05), (-0.02, 0, 0), (0, -0.03, 0), (0.02, 0, 0), (-0.04, 0, 0), (0, 0, 0.05)
            ],
            5: [
                (0.04, -0.03, 0), (0, 0, -0.05), (0, 0.03, 0), (-0.02, 0, 0), (0, -0.03, 0), (-0.02, 0, 0), (0, 0.03, 0), (0, 0, 0.05)
            ],
            6: [
                (0.04, -0.03, 0), (0, 0, -0.05), (0, 0.03, 0), (-0.02, 0, 0), (0, -0.03, 0), (-0.02, 0, 0), (0, 0.03, 0), (0.02, 0, 0), (0, 0, 0.05)
            ],
            7: [
                (0.04, 0, 0), (0, 0, -0.05), (0, -0.03, 0), (-0.04, 0, 0), (0, 0, 0.05)
            ],
            8: [
                (0.02, 0, 0), (0, 0, -0.05), (0.02, 0, 0), (0, -0.03, 0), (-0.04, 0, 0), (0, 0.03, 0), (0.02, 0, 0), (0, -0.03, 0), (0, 0, 0.05)
            ],
            9: [
                (0, 0, -0.05), (0, -0.03, 0), (0.04, 0, 0), (0, 0.03, 0), (-0.02, 0, 0), (0, -0.03, 0), (0, 0, 0.05)
            ]
        }

        if numero not in trayectorias:
            rospy.logwarn(f"Número {numero} no soportado")
            return

    
        pose = self.get_pose()

        poseFin = copy.deepcopy(pose)
        poseFin.position.y -= 0.05

        trajectory = []

        for dx, dy, dz in trayectorias[numero]:
            pose.position.x += dx
            pose.position.y += dy
            pose.position.z += dz
            trajectory.append(copy.deepcopy(pose))

        trajectory.append(poseFin)
        self.move_trajectory(trajectory)

        self.contador_dibujados += 1

        if self.contador_dibujados % 5 == 0:
            angles = self.load_paint_angles_from_yaml("pintar")
            self.move_motors(angles)
            self.contador_filas += 1
            pose = copy.deepcopy(self.get_pose())
            pose.position.x -= 0.05 * self.contador_filas
            self.move_to_pose(pose)




if __name__ == '__main__':
    control = ControlRobot()

    angles = control.load_paint_angles_from_yaml("pintar")
    
    control.move_motors(angles)
    
    rospy.spin()


