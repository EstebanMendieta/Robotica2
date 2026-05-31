import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import quintic
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

L1 = 0.30
L2 = 0.25
d1 = 0.40
client = RemoteAPIClient()
sim = client.require('sim')
joint1 = sim.getObject('/Joint1')
joint2 = sim.getObject('/Joint2')
joint3 = sim.getObject('/Joint3')
time_data = []
tau1_data = []
tau2_data = []
force3_data = []
global_time = 0.0

robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(
            a=L1,
            alpha=0,
            d=d1,
            m=2.0,
            r=[0.15, 0, 0],
            I=[0, 0, 0.015, 0, 0, 0],
            qlim=[-np.pi/2, np.pi/2]
        ),
        rtb.RevoluteDH(
            a=L2,
            alpha=0,
            d=0,
            m=1.5,
            r=[0.125, 0, 0],
            I=[0, 0, 0.0078, 0, 0, 0],
            qlim=[-0.75*np.pi, 0.75*np.pi]
        ),
        rtb.PrismaticDH(
            theta=0,
            alpha=0,
            a=0,
            m=0.5,
            r=[0, 0, 0],
            I=[0, 0, 0, 0, 0, 0],
            qlim=[0.0, 0.05]
        )
    ],
    name='SCARA'
)
q_pick_up = np.array([
    np.deg2rad(-40),
    np.deg2rad(60),
    0.05
])
q_pick_down = np.array([
    np.deg2rad(-40),
    np.deg2rad(60),
    0.00
])
q_transport = np.array([
    np.deg2rad(35),
    np.deg2rad(-20),
    0.05
])
q_place = np.array([
    np.deg2rad(35),
    np.deg2rad(-20),
    0.00
])
def move_quintic(q0, qf, duration):

    global global_time

    dt = 0.05

    t = np.arange(0, duration + dt, dt)

    traj = []

    for i in range(3):

        qi = quintic(
            q0[i],
            qf[i],
            t
        )

        traj.append(qi)

    for k in range(len(t)):

        q = np.array([
            traj[0].q[k],
            traj[1].q[k],
            traj[2].q[k]
        ])

        qd = np.array([
            traj[0].qd[k],
            traj[1].qd[k],
            traj[2].qd[k]
        ])

        qdd = np.array([
            traj[0].qdd[k],
            traj[1].qdd[k],
            traj[2].qdd[k]
        ])

        tau = robot.rne(q, qd, qdd)

        # Guardar resultados
        time_data.append(global_time)

        tau1_data.append(tau[0])
        tau2_data.append(tau[1])
        force3_data.append(tau[2])

        global_time += dt

        sim.setJointPosition(joint1, float(q[0]))
        sim.setJointPosition(joint2, float(q[1]))
        sim.setJointPosition(joint3, float(q[2]))

        time.sleep(dt) 
              
print("Iniciando ciclo")
sim.setJointPosition(joint1, float(q_pick_up[0]))
sim.setJointPosition(joint2, float(q_pick_up[1]))
sim.setJointPosition(joint3, float(q_pick_up[2]))
time.sleep(1)

move_quintic(
    q_pick_up,
    q_pick_down,
    1.0
)
sim.setInt32Signal('close_gripper', 1)
time.sleep(0.5)

move_quintic(
    q_pick_down,
    q_pick_up,
    1.0
)
move_quintic(
    q_pick_up,
    q_transport,
    2.0
)
move_quintic(
    q_transport,
    q_place,
    1.0
)
sim.setInt32Signal('close_gripper', 0)
time.sleep(0.5)

move_quintic(
    q_place,
    q_transport,
    1.0
)
print(f"Tau1 máximo = {max(tau1_data):.3f} N·m")
print(f"Tau1 mínimo = {min(tau1_data):.3f} N·m")
print(f"Tau2 máximo = {max(tau2_data):.3f} N·m")
print(f"Tau2 mínimo = {min(tau2_data):.3f} N·m")
print(f"F3 máxima = {max(force3_data):.3f} N")
print(f"F3 mínima = {min(force3_data):.3f} N")
print("Ciclo completado")