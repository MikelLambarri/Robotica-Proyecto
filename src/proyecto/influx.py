#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

class SignalCaptureNode:
    def __init__(self):
        rospy.init_node('signal_capture_node')

        # Configuración de la base de datos
        self.db_client = InfluxDBClient(
            url="https://deusto-influxdb-001-v2qrmk5znqme3f.eu-west-1.timestream-influxdb.amazonaws.com:8086/",
            token="52vF3WnRBEo-a7_QiTC5nCai1mhjvHnGddwR76Y_wL5ZY9DABsncGCYyKduaG1shN9RDW-bq3J6DOz7ydxrRtA==",
            org="deusto"
        )
        self.write_api = self.db_client.write_api(write_options=SYNCHRONOUS)
        self.bucket = "Grupo_6"

        # Variables
        self.is_capturing = False
        self.start_time = None
        self.traceability_code = None
        self.data = []

        # Suscripciones
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/capture_control', Int32, self.capture_control_callback)

        rospy.spin()

    def capture_control_callback(self, msg):
        """Callback para controlar el inicio y finalización de la captura."""
        command = msg.data
        if command == 1:
            self.is_capturing = True
            self.start_time = time.time()
            self.traceability_code = f"TRACE_{int(self.start_time)}"
            self.data = []
            rospy.loginfo("Captura iniciada.")
        elif command == 0:
            if self.is_capturing:
                self.is_capturing = False
                rospy.loginfo("Captura finalizada. Registrando datos en la base de datos...")
                self.register_data()
                self.data = []
            else:
                rospy.logwarn("Intento de detener captura, pero no estaba activa.")

    def joint_state_callback(self, msg):
        """Callback para capturar datos del topic /joint_states."""
        if self.is_capturing:
            print(msg)
            timestamp = time.time() - self.start_time
            velocity = list(msg.velocity)
            print(velocity)
            position = list(msg.position)
            effort = list(msg.effort)

            rospy.loginfo(f"Velocidad: {velocity}, Esfuerzo: {effort}, Posición: {position}")

            self.data.append({
                "time": timestamp,
                "velocity": velocity,
                "position": position,
                "effort": effort
            })

    def register_data(self):
        """Registrar los datos capturados en InfluxDB."""
        try:
            for entry in self.data:
                # Convierte las listas a cadenas (ejemplo: '[1.0, 2.0, 3.0]')
                velocity_str = str(entry['velocity'])
                position_str = str(entry['position'])
                effort_str = str(entry['effort'])

                # Crea un punto en InfluxDB
                point = Point("joint_states") \
                    .field("velocity", velocity_str) \
                    .field("position", position_str) \
                    .field("effort", effort_str) \
                    .time(entry["time"], WritePrecision.S)

                # Escribe el punto en el bucket
                self.write_api.write(bucket=self.bucket, org="deusto", record=point)
            
            rospy.loginfo("Datos registrados en InfluxDB.")
        except Exception as e:
            rospy.logerr(f"Error al registrar datos en InfluxDB: {e.args}")


if __name__ == '__main__':
    try:
        SignalCaptureNode()
    except rospy.ROSInterruptException:
        pass
