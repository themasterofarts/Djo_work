import rclpy
import cv2 as cv
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

from std_msgs.msg import String
from std_msgs.msg import Bool
from my_interfaces.msg import Detect

from cv_bridge import CvBridge

class ObjectDetect(Node):
    def __init__(self):
        super().__init__('object_detect')

        self.get_logger().info("Node started")

        #### Création du subscriber pour s'abonner au topic LaserScan ####
        self.subscription_laser = self.create_subscription(
            LaserScan,
            "scan",
            self.laser_callback,  # Remarque : On passe la référence à la fonction sans parenthèses
            10
        )
        
        #### Création du subscriber pour s'abonner au topic Image ####
        self.subscription_image = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10
        )

        ### Création du publisher pour envoyer les messages de détection ####
        self.pub = self.create_publisher(Detect, "detection", 10)

        self.pub

        ### publiser la distance et l'angle de l'objet détecté ###
        self.pub_distance_angle = self.create_publisher(Detect, "distance_angle", 10)
        
        #### Création d'un timer pour publier tous les 5 secondes ####
        self.timer_period_sec = 5.0  # en secondes
        self.timer = self.create_timer(self.timer_period_sec, self.time_callback)

        #### Instanciation du pont entre ROS et OpenCV ####
        self.bridge = CvBridge()

        #### Initialisation de l'objet message pour la détection ####
        self.detect_msg = Detect()

        self.detect_msg.zone = ""

        self.detect_msg.object_detect = False

    ### Callback du LaserScan pour détecter les objets ###
    def laser_callback(self, msg: LaserScan):
        self.get_logger().info("Laser callback ok")

        long = len(msg.ranges)

        # Vérification de la présence d'un objet dans chaque zone
        for i in range(long):
            distance = msg.ranges[i]

            # Zone de droite
            if 0 < i < long // 3:
                if distance <= 10.0:
                    self.detect_msg.zone = "ddroite"
                    self.detect_msg.object_detect = True
                    ## calcule de la distnce et l'orientation de l'objet detecté ##
                   # angle = 

            # Zone du milieu
            if  long // 3 < i < 2 * long // 3:
                if distance <= 15.0:
                    self.detect_msg.zone = "milieu"
                    self.detect_msg.object_detect = True

            # Zone de gauche
            if 2 * long // 3 < i < long:
                if distance <= 15.0:
                    self.detect_msg.zone = "droite"
                    self.detect_msg.object_detect = True
    
    # Publication du message de détection ####
    def time_callback(self):
        self.get_logger().info("Time callback ok")
        self.pub.publish(self.detect_msg)
        self.get_logger().info(f"Zone: {self.detect_msg.zone}, Object detected: {self.detect_msg.object_detect}")
        
        # publier la distance et lorientation de l'objet détecté
       # self.pub_distance_angle.publish(s)

    ### Callback pour la capture des images ###
    def image_callback(self, msg: Image):
        self.get_logger().info("Image callback ok")

        # Conversion de l'image ROS en image OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        
        if self.detect_msg.object_detect:
            
            cv.putText(cv_image, f"Object detected in {self.detect_msg.zone}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # Capture de l'image############
            cv.imwrite("object_detected.jpg", cv_image)

        # Affichage de l'image
        cv.imshow("Camera", cv_image)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetect()  # Instance du nœud
    try:
        rclpy.spin(node)  # Le nœud reste actif jusqu'à interruption
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
