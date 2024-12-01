#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime

class SignalCaptureNode:
    def __init__(self):
        rospy.init_node('signal_capture_node')

        self.db_client = InfluxDBClient(
            url="https://deusto-influxdb-001-v2qrmk5znqme3f.eu-west-1.timestream-influxdb.amazonaws.com:8086/",
            token="fTwdAa81OtrQATLX7eWfSGHD62XgmOrfZ5REm5F8UDmPouj1SvsQKdOZgUJiku5HP4tFM_Amzl83gBnRDLKg4Q==",
            org="deusto"
        )
        self.write_api = self.db_client.write_api(write_options=SYNCHRONOUS)
        self.bucket = "Grupo_1"

        self.is_capturing = False
        self.start_time = None
        self.traceability_code = None
        self.data = []
        
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/capture_control', Int32, self.capture_control_callback)
        self.delete_data()

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
            velocity = [v if v is not None else 0 for v in list(msg.velocity)] if msg.velocity else []
            position = [p if p is not None else 0 for p in list(msg.position)] if msg.position else []
            effort = [e if e is not None else 0 for e in list(msg.effort)] if msg.effort else []

            max_length = max(len(velocity), len(position), len(effort))
            velocity += [0] * (max_length - len(velocity))
            position += [0] * (max_length - len(position))
            effort += [0] * (max_length - len(effort))

            rospy.loginfo(f"Velocidad: {velocity}, Esfuerzo: {effort}, Posición: {position}")

            self.data.append({
                "id": self.generate_custom_id("LLL", "AA"),
                "velocity": velocity,
                "position": position,
                "effort": effort
            })


    def generate_custom_id(self, line, area):
        """
        Genera un ID en el formato LLLAAYYJJJHHMMSSmmmuu (milisegundos y microsegundos incluidos).
        """
        now = datetime.now()
        year = now.year % 100  
        day_of_year = now.timetuple().tm_yday  
        hour_min_sec = now.strftime("%H%M%S") 
        milliseconds = int(now.microsecond / 1000)  
        microseconds = now.microsecond % 1000 

        custom_id = f"{line}{area}{year:02d}{day_of_year:03d}{hour_min_sec}{milliseconds:03d}{microseconds:03d}"
        return custom_id
    
    def delete_data(self):
        """Borra todos los datos del bucket antes de registrar nuevos datos."""
        try:
            from datetime import datetime

            now = datetime.utcnow().isoformat() + "Z"  

            self.db_client.delete_api().delete(
                start="1970-01-01T00:00:00Z", 
                stop=now,                      
                bucket=self.bucket,
                org="deusto",               
                predicate=""                 
            )
            rospy.loginfo("Datos eliminados del bucket antes de registrar nuevos datos.")
        except Exception as e:
            rospy.logerr(f"Error al eliminar datos del bucket: {e.args}")

    def register_data(self):
        """Registrar los datos capturados en InfluxDB con unidades específicas según el tipo de dato."""
        try:
            points = []
            for entry in self.data:
                for i, value in enumerate(entry['velocity']):
                    point = Point("joint_states") \
                        .tag("TraceabilityCode", entry["id"]) \
                        .tag("Unit", "rad/s") \
                        .field(f"VelocityValues_Axis{i+1}", value)
                    points.append(point)

                for i, value in enumerate(entry['position']):
                    point = Point("joint_states") \
                        .tag("TraceabilityCode", entry["id"]) \
                        .tag("Unit", "rad") \
                        .field(f"RotationValues_Axis{i+1}", value)
                    points.append(point)

                for i, value in enumerate(entry['effort']):
                    point = Point("joint_states") \
                        .tag("TraceabilityCode", entry["id"]) \
                        .tag("Unit", "N·m") \
                        .field(f"TorqueValues_Axis{i+1}", value)
                    points.append(point)

            if points:
                self.write_api.write(bucket=self.bucket, org="deusto", record=points)
                rospy.loginfo(f"Se registraron {len(points)} puntos en InfluxDB.")
            
            self.data = []
        except Exception as e:
            rospy.logerr(f"Error al registrar datos en InfluxDB: {e.args}")

if __name__ == '__main__':
    try:
        SignalCaptureNode()
    except rospy.ROSInterruptException:
        pass
