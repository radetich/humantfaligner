##MAIN DRIVER 

### Curr issue in properly processing bag data


# Import the pygame library and initialise the game engine
import pygame
import ros_numpy
from geometry_msgs.msg import TransformStamped
import rosbag
from interface import *
import sensor_msgs.point_cloud2
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
# from tf.transformations import quaternion_from_euler
import numpy as np
import cv2
import matplotlib.pyplot as plt
# import tf2_ros
# import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from cv_bridge import CvBridge

trans = TransformStamped()



def tf_update(x, y, z, r, p, yaw):
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.translation.z = z
    # q = quaternion_from_euler(r, p, yaw)
    # trans.transform.rotation.x = q[0]
    # trans.transform.rotation.y = q[1]
    # trans.transform.rotation.z = q[2]
    # trans.transform.rotation.w = q[3]

# this should save the transform to a file
def save_tf(tf):
    with open('tf.txt', 'w') as file:
        file.write(tf)
    return True


##BOBBY FUNCTIONS

# Project lidar points into camera coordinates
def lidar2cam(lidar_pc,rvec,tvec,intrinsics):
    
    dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
    xyz_arr = ros_numpy.point_cloud2.pointcloud2_to_array(lidar_pc)

    lidar_px, _ = cv2.projectPoints(xyz_arr, rvec, tvec, intrinsics, dist_coeffs)
    lidar_px = np.squeeze(lidar_px)
    return lidar_px 


def lid_pre_process(xyz):
    o =  0 
    if np.size(xyz,1) != 3:
        xyz = np.transpose(xyz)
        
    if np.size(xyz,1) != 3:
        xyz = np.transpose(np.array([xyz[:,0],xyz[:,1],xyz[:,2]]))

    #lidar_pc = np.transpose(lidar_pc[0:3,:])
    
    #lidar_pc[:,o] = -lidar_pc[:,o] 
    #lidar_pc = np.transpose(np.array([lidar_pc[:,1],lidar_pc[:,0],lidar_pc[:,2]]))
    return xyz


bridge = CvBridge()


# get the information -- FIXED
bag = rosbag.Bag('test.bag')
images = [ bridge.compressed_imgmsg_to_cv2(msg, 'bgr8') for topic, msg, t in bag.read_messages() if topic == "/frontCamera"]
velodyne = [ msg for _, msg, _ in bag.read_messages(topics=['/VelodyneFrontLeft'])]
bag.close()

print(type(images[0]))

print(len(images))

def ret():
    return True

forward_button = Button('next', (100, 100), ret)
back_button = Button('prev', (100, 200), ret)
x = Slider("Pen", 10, 15, 1, 25)
y = Slider("Freq", 1, 3, 0.2, 150)
z = Slider("Jump", 10, 20, 1, 275)
roll = Slider("Size", 200, 200, 20, 400)
pitch = Slider("Focus", 0, 6, 0, 525)
yaw = Slider("Phase", 3.14, 6, 0.3, 650)
slides = [x, y, z, roll, pitch, yaw]

rvec = [roll, pitch, yaw]
tvec = [x, y, z]

i = 0

pygame.init()

# open a new window
size = (700, 500)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("transform me")

go = True

clock = pygame.time.Clock()

while go:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            go = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            for s in slides: 
                if s.button_rect.collidepoint(pos):
                    s.hit = True
            if forward_button.rect.collidepoint(pos):
                i += 1
            if back_button.rect.collidepoint(pos):
                i -= 1
        elif event.type == pygame.MOUSEBUTTONUP:
            for s in slides:
                s.hit = False

    img = images[i]
    pc = velodyne[i]

    for s in slides:
        if s.hit:
            s.move()

    # JAM INPUTS HERE
    tf_update(x.get(), y.get(), z.get(), roll.get(), pitch.get(), yaw.get())

    # put image here
    screen.fill(WHITE)

    # Camera internals
    focal_length = [606.1387,603.1641]
    center = [959.0102,599.6154]
    intrinsics = np.array([[focal_length[0], 0, center[0]],
                       [0, focal_length[1], center[1]],
                       [0, 0, 1]], dtype = "double"
                       )

    screen.blit(lidar2cam(pc,rvec,tvec,intrinsics), (0,0))

    # display
    pygame.display.flip()

    # 60fps
    clock.tick(60)

pygame.quit()