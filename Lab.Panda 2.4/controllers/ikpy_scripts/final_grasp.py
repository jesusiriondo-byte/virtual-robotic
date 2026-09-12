import time
from ikpy.chain import Chain
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64

chain = Chain.from_urdf_file('/tmp/panda_ikpy.urdf')
BASE = np.array([0.5, -0.3, 0.74])
SNAPSHOT = np.array([0.0, 0.0, 0.0, -1.7708, -1.6, 1.6, 0.79])
GRASP_R = np.array([[1.0,0,0],[0,-1.0,0],[0,0,-1.0]])
CORRECTION_Z = -0.16
LO = np.array([-2.9671,-1.8326,-2.9671,-3.1416,-2.9671,-0.0873,-2.9671])
HI = np.array([2.9671,1.8326,2.9671,-0.4,2.9671,3.8223,2.9671])

def solve(target_world, seed):
    target_local = target_world - BASE
    result = chain.inverse_kinematics(target_local, target_orientation=GRASP_R, orientation_mode='all', initial_position=seed)
    delta_solved = np.array(result[2:9])
    real_theta = delta_solved + SNAPSHOT
    ok = bool(np.all(real_theta>=LO) and np.all(real_theta<=HI))
    return real_theta, result, ok

CURRENT_REAL = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
seed = [0.0,0.0] + list(CURRENT_REAL - SNAPSHOT) + [0.0,0.0]
CUBE = (0.5, 0.15)

rclpy.init()
node = Node('final_grasp')
pub_j = node.create_publisher(Float64MultiArray, '/joint_positions', 10)
pub_g = node.create_publisher(Float64, '/gripper_position', 10)
t0=time.time()
while pub_j.get_subscription_count()==0 and time.time()-t0<10:
    rclpy.spin_once(node, timeout_sec=0.1)

cur = seed
z = 0.97
zs = []
while z > 0.701:
    z = max(0.70, z-0.015)
    zs.append(z)

for z in zs:
    th, cur, ok = solve(np.array([CUBE[0],CUBE[1],z+CORRECTION_Z]), cur)
    if not ok:
        print('LIMITE en z=', z); break
    pub_j.publish(Float64MultiArray(data=th.tolist()))
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.5)
    print('z=', round(z,3), th.round(4))

print('cerrando pinza')
pub_g.publish(Float64(data=0.0))
time.sleep(2.0)

print('DONE (sin subir, para comprobar solo el cierre)')
rclpy.shutdown()
