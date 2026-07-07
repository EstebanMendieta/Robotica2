import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

m1, m2, m3 = 2.0, 1.5, 0.5
mu_v = 2.5
mu_s = 0.8 
L1 = rtb.RevoluteMDH(a=0.0, alpha=0.0, d=0.2, m=m1, r=[0.15, 0, 0], 
                     I=[0.01, 0.05, 0.05, 0, 0, 0], Jm=1e-4, B=mu_v, Tc=[mu_s, -mu_s])
L2 = rtb.RevoluteMDH(a=0.3, alpha=0.0, d=0.0, m=m2, r=[0.125, 0, 0], 
                     I=[0.008, 0.03, 0.03, 0, 0, 0], Jm=1e-4, B=mu_v, Tc=[mu_s, -mu_s])
L3 = rtb.PrismaticMDH(a=0.25, alpha=np.pi, theta=0.0, m=m3, r=[0, 0, 0.05], 
                      I=[0.002, 0.002, 0.001, 0, 0, 0], Jm=1e-4, B=mu_v, Tc=[mu_s, -mu_s])
scara = rtb.DHRobot([L1, L2, L3], name="SCARA_Agricola", gravity=[0, 0, 9.81])

q_home  = [0.0, 0.0, 0.0]
q_pick  = [np.pi/4, -np.pi/3, 0.10]
q_place = [-np.pi/4, np.pi/4, 0.12]

dt = 0.05
t1 = np.linspace(0, 2.0, int(2.0/dt)) 
t2 = np.linspace(0, 2.0, int(2.0/dt)) 
t3 = np.linspace(0, 1.0, int(1.0/dt)) 

fase1 = rtb.jtraj(q_home, q_pick, t1)
fase2 = rtb.jtraj(q_pick, q_place, t2)
fase3 = rtb.jtraj(q_place, q_home, t3)

Q = np.vstack((fase1.q, fase2.q, fase3.q))
Qd = np.vstack((fase1.qd, fase2.qd, fase3.qd))
Qdd = np.vstack((fase1.qdd, fase2.qdd, fase3.qdd))
pasos_totales = len(Q)

client = RemoteAPIClient()
sim = client.getObject('sim')
motor_base = sim.getObject('/Joint1')
motor_codo = sim.getObject('/Joint2')
motor_z = sim.getObject('/Joint3')
motores = [motor_base, motor_codo, motor_z]
client.setStepping(True)

sim.startSimulation()
print("Iniciando simulacion")
try:
    for i in range(pasos_totales):
        q_des = Q[i, :]
        qd_des = Qd[i, :]
        qdd_des = Qdd[i, :]
        tau = scara.rne(q_des, qd_des, qdd_des)
        for j, motor in enumerate(motores):
            sim.setJointTargetPosition(motor, q_des[j])
        client.step()
finally:
    sim.stopSimulation()
    print("Fin de la simulacion")