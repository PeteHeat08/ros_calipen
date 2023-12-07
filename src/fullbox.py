#! /usr/bin/env python
import numpy as np
import rospy
import moveit_commander
import math
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Joy

class Integrator:

    def quaternion_between_vectors(this, v1, v2):
        # Normalize the input vectors
        v1 = v1 / np.linalg.norm(v1)
        v2 = v2 / np.linalg.norm(v2)
    
        # Calculate the dot product and the cross product
        dot_product = np.dot(v1, v2)
        cross_product = np.cross(v1, v2)
    
        # Calculate the quaternion components
        w = dot_product + np.sqrt(np.linalg.norm(v1) * np.linalg.norm(v2))
        xyz = cross_product
        
        # Create the quaternion
        quaternion = np.concatenate(([w], xyz))
        if (np.linalg.norm(quaternion) != 0):
            quaternion /= np.linalg.norm(quaternion)  # Normalize the quaternion
        
        return quaternion

    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
     
    def pose_callback(self, pose: PoseStamped):
        #print(pose.pose)
        self.pose = pose

    def click_callback(self, click: Joy):
        print("click_callback")
        if (click.buttons[0] == 1):
            self.clicked = 1

    def run(self):
        while not rospy.is_shutdown():

            #print(self.clicked)
            if (self.clicked == 1):
                self.counter = 4
                self.points.append((0,0,0))
                self.points.append((1,1,0))
                self.points.append((0,1,0))
                self.points.append((0,0,1))

                self.clicked = 0
                if (self.counter < 3):
                    curr_pose = Point()
                    curr_pose = self.pose.pose
                    self.points.append(curr_pose)
                    self.counter += 1
                else:
                    x0, y0, z0 = p0 = np.asarray(self.points[0])
                    x1, y1, z1 = p1 = np.asarray(self.points[1])
                    x2, y2, z2 = p2 = np.asarray(self.points[2])
                    x3, y3, z3 = p3 = np.asarray(self.points[3])

                    n = np.cross( p1-p0 , p2-p0 )
                    n = n/np.linalg.norm(n)

                    dim1 = math.dist(p0, p1)
                    dim2 = np.linalg.norm(np.cross(p1-p0, p0-p2))/np.linalg.norm(p1-p0)
                    #dim2 = math.dist(p0, p2)
                    dim3 = np.linalg.norm(np.dot( p3-p0 , n ))

                    print("Dimensions: " + str(dim1) + " " + str(dim2) + " " + str(dim3))

                    box_pose = PoseStamped()
                    box_pose.header.frame_id = "world"

                    box_pose.pose.position.x = x0
                    box_pose.pose.position.y = y0
                    box_pose.pose.position.z = z0

                    print("Angles are: " + str(n[0]) + " " + str(n[1]) + " " + str(n[2]))

                    #box_pose.pose.orientation.x = n[0]
                    #box_pose.pose.orientation.y = n[1]
                    #box_pose.pose.orientation.z = n[2]

                    qx, qy, qz, qw = integrator.euler_to_quaternion(n[0], n[1], n[2])

                    quats = integrator.quaternion_between_vectors(n, [0,0,1])
                    [qx, qy, qz, qw] = quats

                    box_pose.pose.orientation.x = qx
                    box_pose.pose.orientation.y = qy
                    box_pose.pose.orientation.z = qz
                    box_pose.pose.orientation.w = qw

                    print("Quaternions are: " + str(qx) + " " + str(qy) + " " + str(qz) + " " + str(qw))

                    box_name = "box"
                    self.scene.add_box(box_name, box_pose, size=(dim1, dim2, dim3))

                    box_pose_0 = PoseStamped()
                    box_pose_0.header.frame_id = "world"
                    box_pose_0.pose.position.x = p0[0]
                    box_pose_0.pose.position.y = p0[1]
                    box_pose_0.pose.position.z = p0[2]

                    box_pose_1 = PoseStamped()
                    box_pose_1.header.frame_id = "world"
                    box_pose_1.pose.position.x = p1[0]
                    box_pose_1.pose.position.y = p1[1]
                    box_pose_1.pose.position.z = p1[2]

                    box_pose_2 = PoseStamped()
                    box_pose_2.header.frame_id = "world"
                    box_pose_2.pose.position.x = p2[0]
                    box_pose_2.pose.position.y = p2[1]
                    box_pose_2.pose.position.z = p2[2]

                    self.scene.add_box("box0", box_pose_0, size=(0.3, 0,3, 0.3))
                    self.scene.add_box("box1", box_pose_1, size=(0.3, 0,3, 0.3))
                    self.scene.add_box("box2", box_pose_2, size=(0.3, 0,3, 0.3))

                    #print("Added box in " + str(p0))

                    self.counter = 0
                    # box_pose.pose.orientation.w = 1.0
            pass

    def __init__(self):
        rospy.loginfo("Integrator started")
        rospy.Subscriber('/vrpn_client_node/Kalipen/pose', PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)

        self.clicked = 1 #0
        self.counter = 0
        self.points = list()
        self.pose = PoseStamped()
        self.scene = moveit_commander.PlanningSceneInterface()

if __name__ == '__main__':
    try:
        rospy.init_node('integrator_node')
        integrator = Integrator()
        integrator.run()
    except rospy.ROSInterruptException:
        pass
