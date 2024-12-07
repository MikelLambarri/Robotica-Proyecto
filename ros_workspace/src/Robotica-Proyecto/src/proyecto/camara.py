import cv2
import numpy as np
import imutils
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from copy import deepcopy

class NodoCamara:
    def __init__(self):
        rospy.init_node('nodo_camara', anonymous=True)
        self.bridge = CvBridge()
        self.cv_image = None
        rospy.Subscriber('/usb_cam/image_raw', Image, self.__cb_image)
        self.number_publisher = rospy.Publisher('/numero', Int32, queue_size=10)


    def __cb_image(self, image: Image):
        # Convertir el mensaje de ROS a una imagen de OpenCV
        self.cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    
    def run(self):
        bg = None
        # Colores de visualización
        color_start = (0, 255, 127)  # Verde menta
        color_end = (255, 153, 51)   # Naranja suave
        color_far = (102, 178, 255)  # Azul claro

        color_start_far = (0, 255, 127)   # Verde menta
        color_far_end = (255, 153, 51)    # Naranja suave
        color_start_end = (255, 204, 153) # Crema

        color_contorno = (153, 204, 0)    # Verde lima
        color_ymin = (255, 102, 178)      # Rosa fuerte
        color_fingers = (102, 255, 178)   # Verde agua

        while not rospy.is_shutdown():
            if self.cv_image is not None:
                # Trabajar con una copia de la imagen actual
                frame = deepcopy(self.cv_image)
                frame = imutils.resize(frame, width=640)
                frame = cv2.flip(frame, 1)
                frameAux = frame.copy()
                
                if bg is not None:
                    # Determinar la región de interés
                    ROI = frame[50:300, 380:600]
                    cv2.rectangle(frame, (380-2, 50-2), (600+2, 300+2), color_fingers, 1)
                    grayROI = cv2.cvtColor(ROI, cv2.COLOR_BGR2GRAY)
                    
                    # Región de interés del fondo de la imagen
                    bgROI = bg[50:300, 380:600]
                    
                    # Determinar la imagen binaria (background vs foreground)
                    dif = cv2.absdiff(grayROI, bgROI)
                    _, th = cv2.threshold(dif, 30, 255, cv2.THRESH_BINARY)
                    th = cv2.medianBlur(th, 7)

                    # Encontrando los contornos de la imagen binaria
                    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:1]

                    for cnt in cnts:
                        # Encontrar el centro del contorno
                        M = cv2.moments(cnt)
                        if M["m00"] == 0: M["m00"] = 1
                        x = int(M["m10"]/M["m00"])
                        y = int(M["m01"]/M["m00"])
                        cv2.circle(ROI, tuple([x, y]), 5, (0, 255, 0), -1)

                        # Punto más alto del contorno
                        ymin = cnt.min(axis=1)
                        cv2.circle(ROI, tuple(ymin[0]), 5, color_ymin, -1)

                        # Contorno encontrado a través de cv2.convexHull
                        hull1 = cv2.convexHull(cnt)
                        cv2.drawContours(ROI, [hull1], 0, color_contorno, 2)

                        # Defectos convexos
                        hull2 = cv2.convexHull(cnt, returnPoints=False)
                        defects = cv2.convexityDefects(cnt, hull2)

                        if defects is not None:
                            inicio = []
                            fin = []
                            fingers = 0
                            angle_detected = False  # Variable para detectar ángulo grande
                            pulgar_detectado = False  # Variable para detectar si solo el pulgar está levantado


                            for i in range(defects.shape[0]):
                                s, e, f, d = defects[i, 0]
                                start = cnt[s][0]
                                end = cnt[e][0]
                                far = cnt[f][0]

                                # Calcular ángulo
                                a = np.linalg.norm(far-end)
                                b = np.linalg.norm(far-start)
                                c = np.linalg.norm(start-end)

                                angulo = np.arccos((a**2 + b**2 - c**2) / (2 * a * b))
                                angulo = np.degrees(angulo)

                                if np.linalg.norm(start-end) > 20 and angulo < 90 and d > 12000:
                                    inicio.append(start)
                                    fin.append(end)

                                     # Si se encuentra un ángulo mayor a 70, activar bandera
                                    if angulo > 70:
                                        angle_detected = True
                                    
                                    # Revisar si el punto de inicio está en una posición característica del pulgar
                                    if start[0] < x:  # Verificar si el punto está al lado izquierdo (asumiendo que es el pulgar)
                                        pulgar_detectado = True
                        

                                    cv2.circle(ROI, tuple(start), 5, color_start, 2)
                                    cv2.circle(ROI, tuple(end), 5, color_end, 2)
                                    cv2.circle(ROI, tuple(far), 7, color_far, -1)

                            if len(inicio) == 0:
                                minY = np.linalg.norm(ymin[0] - [x, y])
                                if minY >= 110:
                                    fingers += 1
                                    
                            for i in range(len(inicio)):
                                fingers += 1
                                if i == len(inicio) - 1:
                                    fingers += 1
                                
                            # Determinación final del número en función de angle_detected y pulgar_detectado
                            if angle_detected:
                                # Números del 6 al 9 basados en la cantidad de dedos detectados y posición del pulgar
                                if fingers == 1 and pulgar_detectado:
                                    # Verificar si el dedo único está más abajo que ymin
                                    pulgar_mas_bajo = all(start[1] > ymin[0][1] for start in inicio)
                                    if pulgar_mas_bajo:
                                        number = 6  # Solo el pulgar levantado
                                    else:
                                        number = 1  # Caso incorrecto, ajustamos a "1"
                                elif fingers == 2:
                                    number = 7  # Pulgar e índice levantados
                                elif fingers == 3:
                                    number = 8  # Pulgar, índice y medio levantados
                                elif fingers == 4:
                                    number = 9  # Todos los dedos levantados excepto meñique
                            else:
                                # Números del 0 al 5
                                number = fingers

                            
                            cv2.putText(frame, '{}'.format(number), (390, 45), 1, 4, color_fingers, 2, cv2.LINE_AA)

                            

                    cv2.imshow('th', th)
                cv2.imshow('Frame', frame)

                k = cv2.waitKey(20)
                if k == ord('i'):
                    bg = cv2.cvtColor(frameAux, cv2.COLOR_BGR2GRAY)
                if k == 27:
                    break
                if k == ord('s'):
                    print(f"envio {fingers}")
                    mensaje_publicar = Int32() # Crear un mensaje de tipo String
                    mensaje_publicar.data = fingers
                    self.number_publisher.publish(mensaje_publicar)

            rospy.sleep(0.03)
        
        cv2.destroyAllWindows()

if __name__ == "__main__":
    nodo = NodoCamara()
    nodo.run()
