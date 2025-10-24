# import rclpy
# import cv2 as cv 

# from rclpy.node import Node 
# from sensor_msgs.msg import Image, LaserScan

# from my_interfaces.msg import Detect
# from cv_bridge import CvBridge



# class ObjectDetect(Node):
#     def __init__(self):
#         super().__init__('object_detect')

#         self.get_logger().info("node started")

#         #### Cretion du subscriber pour abonner  de la camera ####

#         self.subscription=self.create_subscription(
#             LaserScan,
#             "scan",
#             self.laser_callback(), 
#             10)
        
#         ### cration d'un publissher

#         self.pub= self.create_publisher(Detect, "detection",10)
        
#         #### cretaion d'un timer  pour publier tous les 5 seconde####
#         self.timer_periode_sec = 5.0 # en seconde
#         self.time= self.create_timer(self.timer_periode_sec, self.time_callback)

#          ##### instancian d'object our effectuer le pont entre ros et cv2 #####
#         self.brigde=CvBridge()
#         self.subscription

    
#     ### creation de l'abonner pour visualiser les image#####

#     def laser_callback(self):



#         return 

        




# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetect() #instance du nœud
#     try:
#         rclpy.spin(node)  # le nœud reste actif jusqu'à interruption
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()