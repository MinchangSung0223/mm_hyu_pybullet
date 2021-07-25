import pybullet as p
import numpy as np
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.5f}".format(x)})

from parameter import *
from func.mr import *
import time
import pybullet_data
from functions import *
pi = np.pi




def getJointState(robotId,ArmJoint):
	jointState = p.getJointStates(robotId,ArmJoint)
	q = [jointState[0][0],jointState[1][0],jointState[2][0],jointState[3][0],jointState[4][0],jointState[5][0]]
	qdot =[jointState[0][1],jointState[1][1],jointState[2][1],jointState[3][1],jointState[4][1],jointState[5][1]]
	return q,qdot
def MR_setup():
	
	w = []
	w.append([0,0,1])
	w.append([0,1,0])
	w.append([0,0,1])
	w.append([-1,0,0])
	w.append([0,0,1])
	w.append([0,1,0])

	p = []
	p.append([BASE_X,BASE_Y,BASE_Z+LINK_01])
	p.append([BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12])
	p.append([BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23])
	p.append([BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34])
	p.append([BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45])
	p.append([BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56])

	CoM_URDF = []
	CoM_URDF.append([0.0 ,0 ,0.049473 ])
	CoM_URDF.append([ 0.001501, 0, 0.042572])
	CoM_URDF.append([0.000454 ,0.002601 ,0.084558])	
	CoM_URDF.append([-0.002133 ,-0.000212 ,0.098828])
	CoM_URDF.append([0.002354, -0.00032, 0.094781])
	CoM_URDF.append([ 0 ,0.002249, 0.09774])	
	CoM_URDF.append([0.000084,-0.001022,0.00897])	
	
	CoM = []
	CoM.append([BASE_X+CoM_URDF[1][0],BASE_Y+CoM_URDF[1][1],BASE_Z+LINK_01+CoM_URDF[1][2]])
	CoM.append([BASE_X+CoM_URDF[2][0],BASE_Y+CoM_URDF[2][1],BASE_Z+LINK_01+LINK_12+CoM_URDF[2][2]])
	CoM.append([BASE_X+CoM_URDF[3][0],BASE_Y+CoM_URDF[3][1],BASE_Z+LINK_01+LINK_12+LINK_23+CoM_URDF[3][2]])
	CoM.append([BASE_X+CoM_URDF[4][0],BASE_Y+CoM_URDF[4][1],BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+CoM_URDF[4][2]])
	CoM.append([BASE_X+CoM_URDF[5][0],BASE_Y+CoM_URDF[5][1],BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+CoM_URDF[5][2]])
	CoM.append([BASE_X+CoM_URDF[6][0],BASE_Y+CoM_URDF[6][1],BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56+CoM_URDF[6][2]])
	CoM.append([BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56+LINK_6E])

	Slist = w_p_to_Slist(w,p,6);
	thetalist = np.array([0,0,0,0,0,0])
	M = np.eye(4);
	M[0,3] = BASE_X
	M[1,3] = BASE_Y
	M[2,3] = BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56+LINK_6E

	inertia = []
	inertia.append([[J_Ixx_1,J_Ixy_1,J_Ixz_1],
					[J_Ixy_1,J_Iyy_1,J_Iyz_1],
					[J_Ixz_1,J_Iyz_1,J_Izz_1]])
	inertia.append([[J_Ixx_2,J_Ixy_2,J_Ixz_2],
					[J_Ixy_2,J_Iyy_2,J_Iyz_2],
					[J_Ixz_2,J_Iyz_2,J_Izz_2]])
	inertia.append([[J_Ixx_3,J_Ixy_3,J_Ixz_3],
					[J_Ixy_3,J_Iyy_3,J_Iyz_3],
					[J_Ixz_3,J_Iyz_3,J_Izz_3]])
	inertia.append([[J_Ixx_4,J_Ixy_4,J_Ixz_4],
					[J_Ixy_4,J_Iyy_4,J_Iyz_4],
					[J_Ixz_4,J_Iyz_4,J_Izz_4]])
	inertia.append([[J_Ixx_5,J_Ixy_5,J_Ixz_5],
					[J_Ixy_5,J_Iyy_5,J_Iyz_5],
					[J_Ixz_5,J_Iyz_5,J_Izz_5]])
	inertia.append([[J_Ixx_6,J_Ixy_6,J_Ixz_6],
					[J_Ixy_6,J_Iyy_6,J_Iyz_6],
					[J_Ixz_6,J_Iyz_6,J_Izz_6]])
	mass = [MASS_1,MASS_2,MASS_3,MASS_4,MASS_5,MASS_6]

	CoM_Mlist = getCoM_Mlist(M,CoM,ROBOT_DOF);
	Glist = getGlist(inertia,mass,ROBOT_DOF)
	T=FKinSpace(M, Slist, thetalist)	
	return CoM_Mlist,Glist,Slist

def main():
	def setTorque(torque):
		n = 0
		for j in ArmJoint:
			p.setJointMotorControl2(robotId, j, p.TORQUE_CONTROL, targetPosition=0, force=torque[n])
			n = n+1

	MR_setup()
	p.connect(p.GUI)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())

	useRealTimeSim = False
	p.setRealTimeSimulation(useRealTimeSim)
	plane= p.loadURDF("urdf/plane/plane.urdf")
	robotPATH = "urdf/right_sim.urdf"
	p.setGravity(0, 0, -10)
	robotId = p.loadURDF(robotPATH,[0,0,0.1], p.getQuaternionFromEuler([0,0,0]))
	NumberofJoint = p.getNumJoints(robotId)
	MobileJoint = [0,1,2]
	ArmJoint = [12,13,14,15,16,17]
	FTsensor = [11]
	for j in MobileJoint:
		p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL, targetPosition=0, force=0)
	for j in ArmJoint:
		p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL, targetPosition=0, force=0)
	for i in range(0,NumberofJoint):
		print(p.getJointInfo(robotId,i))
	dt = 0.001
	EndTime=1

	Mlist,Glist,Slist = MR_setup();
	g = np.array([0, 0, -9.8])



	thetastart = np.array([0,0,0,0,0,0])
	thetaend = np.array([pi/5,pi/2,pi/5,pi/5,pi/2,pi/2])
	
	jt = JointTrajectory(thetastart, thetaend, EndTime, int(EndTime/dt), 5)
	jvt = JointVelocityTrajectory(thetastart, thetaend, EndTime, int(EndTime/dt), 5)
	i=0;

	save_dq = []
	save_q = []
	save_qdot = []
	save_dqdot = []
	save_t =[]


	Kp = np.diag([28.7931,32.5279,18.764,23.2564,14.1946,15])
	Kd = np.diag([0.00575482*Kp[0,0],0.005499895*Kp[1,1],0.0045007*Kp[2,2],0.004599985*Kp[3,3],0.00459971*Kp[4,4],0.00459971*Kp[5,5]])*100
	Ki = np.diag([20,20,20,20,20,20])*5

	print(Kp)
	print(Kd)
	print(Ki)



	prev_e=0;

	for t in range(0,int(EndTime/dt)):
		q,qdot = getJointState(robotId,ArmJoint)
		
		#dq = jt[i];
		#dqdot = jvt[i]


		dq = np.array([0,0,0,0,0,0]);
		dqdot =np.array([0,0,0,0,0,0]);

		e = dq -q
		edot = dqdot-qdot
		eint = prev_e + e*dt
		#G = GravityForces(np.array(q), g, Mlist, Glist, Slist)
		#C = VelQuadraticForces(np.array(q),np.array(qdot), Mlist, Glist, Slist)
		#M = MassMatrix(np.array(q),Mlist,Glist,Slist)

		#u =np.dot(M, \
        #          Kp.dot(e) + Ki.dot(np.array(eint) + e) \
        #          + Kd.dot(np.subtract(np.array(dqdot), np.array(dq))))+C.dot(dq)+G 
		u = np.dot(MassMatrix(np.array(q), Mlist, Glist, Slist), Kp.dot(e) + Ki.dot(np.array(eint) + e) + Kd.dot(edot))
		+ InverseDynamics(np.array(q), np.array(dq), np.array([0,0,0,0,0,0]), g, [0, 0, 0, 0, 0, 0], Mlist, Glist, Slist)
		prev_e = e;



		save_t.append(t*dt)
		save_dq.append(dq)
		save_q.append(q)
		save_dqdot.append(dqdot)
		save_qdot.append(qdot)
		
		setTorque(u)





		p.stepSimulation()
		i=i+1;
		time.sleep(dt)




	drawqplot(save_t,np.array(save_q),np.array(save_dq),"q","Time","Value",6)
	drawqplot(save_t,np.array(save_qdot),np.array(save_dqdot),"qdot","Time","Value",6)

if __name__ == "__main__":
    # execute only if run as a script
    main()