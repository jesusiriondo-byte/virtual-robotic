import sys, time
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
    ok = np.all(real_theta >= LO) and np.all(real_theta <= HI)
    return real_theta, result, ok

CUBE_XY = (0.2898, -0.0896)
CURRENT_REAL = np.array([1.0365, -1.4788, 1.3972, -3.0049, -0.2222, 1.6063, 1.3394])  # hover actual conocido
seed = [0.0, 0.0] + list(CURRENT_REAL - SNAPSHOT) + [0.0, 0.0]

waypoints = []
# ajustar XY primero, en alto (z deseado 0.95), en pasos de 1cm
cur_xy = np.array([0.3, -0.05])
target_xy = np.array(CUBE_XY)
n = 5
for i in range(1, n + 1):
    xy = cur_xy + (target_xy - cur_xy) * i / n
    waypoints.append((xy[0], xy[1], 0.95))
# descenso en pasos de 2cm de 0.95 a 0.70 (altura real deseada)
z = 0.95
while z > 0.701:
    z = max(0.70, z - 0.02)
    waypoints.append((CUBE_XY[0], CUBE_XY[1], z))

thetas = []
cur_seed = seed
for wp in waypoints:
    target = np.array([wp[0], wp[1], wp[2] + CORRECTION_Z])
    th, cur_seed, ok = solve(target, cur_seed)
    thetas.append((wp, th, ok))
    if not ok:
        print(f'AVISO limite excedido en waypoint {wp}: {th.round(3)}')

print(f'{len(thetas)} pasos calculados, todos ok={all(t[2] for t in thetas)}')

rclpy.init()
node = Node('ikpy_grasp')
pub_joint = node.create_publisher(Float64MultiArray, '/joint_positions', 10)
pub_grip = node.create_publisher(Float64, '/gripper_position', 10)
t0 = time.time()
while pub_joint.get_subscription_count() == 0 and time.time() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.1)

for wp, th, ok in thetas:
    if not ok:
        print('parando en waypoint invalido', wp)
        break
    pub_joint.publish(Float64MultiArray(data=th.tolist()))
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.5)
    print('->', wp, th.round(4))

print('cerrando pinza')
pub_grip.publish(Float64(data=0.0))
time.sleep(2.0)

print('subiendo')
up_seed = cur_seed
for z in [0.75, 0.80, 0.85, 0.90, 0.95]:
    target = np.array([CUBE_XY[0], CUBE_XY[1], z + CORRECTION_Z])
    th, up_seed, ok = solve(target, up_seed)
    if not ok:
        print('limite excedido subiendo en z=', z)
        break
    pub_joint.publish(Float64MultiArray(data=th.tolist()))
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.5)
    print('sube ->', z, th.round(4))

print('DONE')
rclpy.shutdown()
