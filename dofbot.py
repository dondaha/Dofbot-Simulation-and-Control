import pybullet as p
import numpy as np
import time



class dofbot:
    def __init__(self, urdfPath):
        # # upper limits for null space
        self.ll = [-3.14,0,-1.05,-0.16,-np.pi]
        # upper limits for null space
        self.ul = [3.14,3.14,4.19,3.3,np.pi]
        # joint ranges for null space
        self.jr = [6.28,3.14,5.24,3.46,2*np.pi]
        # rest poses for null space
        self.rp = [1.57, 1.57, 1.57, 1.57, 1.57]

        self.maxForce = 200.
        self.fingerAForce = 2.5
        self.fingerBForce = 2.5
        self.fingerTipForce = 2

        self.dofbotUid = p.loadURDF(urdfPath,baseOrientation =p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        # self.numJoints = p.getNumJoints(self.dofbotUid)
        self.numJoints = 5
        self.gripper_joints = [5,6,7,8,9,10]
        self.gripper_angle = 0
        self.endEffectorPos = [0.55, 0.0, 0.6]

        self.jointPositions = [1.57, 1.57, 1.57, 1.57, 1.57]

        self.motorIndices = []
        for jointIndex in range(self.numJoints):
            p.resetJointState(self.dofbotUid, jointIndex, self.jointPositions[jointIndex])
            qIndex = p.getJointInfo(self.dofbotUid, jointIndex)[3]
            if qIndex > -1:
                self.motorIndices.append(jointIndex)

        for i,jointIndex in enumerate(self.gripper_joints):
            p.resetJointState(self.dofbotUid, jointIndex, self.gripper_angle)





    def reset(self):
        self.endEffectorPos = [0.55, 0.0, 0.6]

        self.endEffectorAngle = 0
        self.gripper_angle = 0.0
        for jointIndex in range(self.numJoints):
            p.resetJointState(self.dofbotUid, jointIndex, self.jointPositions[jointIndex])
        for i,jointIndex in enumerate(self.gripper_joints):
            p.resetJointState(self.dofbotUid, jointIndex, self.gripper_angle)

    def forwardKinematic(self,jointPoses):
        for i in range(self.numJoints):
            p.resetJointState(self.dofbotUid,
                              jointIndex=i,targetValue=jointPoses[i],targetVelocity=0)
        return self.get_pose()


    def joint_control(self,jointPoses):

        for i in range(self.numJoints):
            p.setJointMotorControl2(bodyUniqueId=self.dofbotUid, jointIndex=i, controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i], targetVelocity=0, force=200,
                                    maxVelocity=1.0, positionGain=0.3, velocityGain=1)



    def setInverseKine(self, pos, orn):
        if orn is None:
            jointPoses = p.calculateInverseKinematics(self.dofbotUid, 4, pos,
                                                      self.ll, self.ul, self.jr, self.rp)
        else:
            jointPoses = p.calculateInverseKinematics(self.dofbotUid, 4, pos, orn,
                                                      self.ll, self.ul, self.jr, self.rp)
        return jointPoses[:self.numJoints]

    def get_jointPoses(self):
        jointPoses= []
        for i in range(self.numJoints+1):
            state = p.getJointState(self.dofbotUid, i)
            jointPoses.append(state[0])
        return jointPoses[:self.numJoints],jointPoses[self.numJoints]


    def get_pose(self):
        state = p.getLinkState(self.dofbotUid, 4)
        pos = state[0]
        orn = state[1]
        return pos,orn

    def getObservation(self):
        observation = []
        state = p.getLinkState(self.dofbotUid, 4)

        pos = state[0]
        orn = state[1]
        euler = p.getEulerFromQuaternion(orn)
        observation.extend(list(pos))
        observation.extend(list(euler))
        return observation

    def gripper_control(self,gripperAngle):

        p.setJointMotorControl2(self.dofbotUid,
                                5,
                                p.POSITION_CONTROL,
                                targetPosition=gripperAngle,
                                force=self.fingerAForce)
        p.setJointMotorControl2(self.dofbotUid,
                                6,
                                p.POSITION_CONTROL,
                                targetPosition=gripperAngle,
                                force=self.fingerBForce)
        p.setJointMotorControl2(self.dofbotUid,
                                7,
                                p.POSITION_CONTROL,
                                targetPosition=gripperAngle,
                                force=self.fingerAForce)
        p.setJointMotorControl2(self.dofbotUid,
                                8,
                                p.POSITION_CONTROL,
                                targetPosition=gripperAngle,
                                force=self.fingerBForce)
        p.setJointMotorControl2(self.dofbotUid,
                                9,
                                p.POSITION_CONTROL,
                                targetPosition=gripperAngle,
                                force=self.fingerAForce)
        p.setJointMotorControl2(self.dofbotUid,
                                10,
                                p.POSITION_CONTROL,
                                targetPosition=gripperAngle,
                                force=self.fingerAForce)




class Object:
    def __init__(self, urdfPath, block,num):
        self.id = p.loadURDF(urdfPath)
        self.half_height = 0.015 if block else 0.0745
        self.num = num

        self.block = block
    def reset(self):

        if self.num==1:
            p.resetBasePositionAndOrientation(self.id,
                                         np.array([ 0.20, 0.1,
                                                   self.half_height]),
                                        p.getQuaternionFromEuler([0, 0,np.pi/6]))
        else:
            p.resetBasePositionAndOrientation(self.id,
                                         np.array([ 0.2, -0.1,
                                                   0.005]),
                                        p.getQuaternionFromEuler([0, 0,0]))

    def pos_and_orn(self):
        pos, orn = p.getBasePositionAndOrientation(self.id)
        # euler = p.getEulerFromQuaternion(quat)
        return pos, orn


def check_pairwise_collisions(bodies):
    for body1 in bodies:
        for body2 in bodies:
            if body1 != body2 and \
                    len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=0., physicsClientId=0)) != 0:
                return True
    return False


class DofbotEnv:
    def __init__(self):
        self._timeStep = 0.001
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(1.0, 90, -40, [0, 0, 0])
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(self._timeStep)
        p.setGravity(0, 0, -9.8)


        p.loadURDF("models/floor.urdf", [0, 0, -0.625], useFixedBase=True)
        p.loadURDF("models/table_collision/table.urdf", [0.5, 0, -0.625],p.getQuaternionFromEuler([0, 0, 0]),
                   useFixedBase=True)
        self._dofbot = dofbot("models/dofbot_urdf_with_gripper/dofbot_with_gripper.urdf")
        self._object1 = Object("models/box_green.urdf", block=True,num=1)
        self._object2 = Object("models/box_purple.urdf", block=True,num=2)
        self.target_pos = np.array([0.2, -0.1, 0.15])


    def reset(self):

        self._object1.reset()
        self._object2.reset()
        self._dofbot.reset()
        p.stepSimulation()


    def dofbot_control(self,jointPoses,gripperAngle):
        '''

        :param jointPoses: 数组，机械臂五个关节角度
        :param gripperAngle: 浮点数，机械臂夹爪角度，负值加紧，真值张开
        :return:
        '''

        self._dofbot.joint_control(jointPoses)
        self._dofbot.gripper_control(gripperAngle)
        p.stepSimulation()
        time.sleep(self._timeStep)

    def dofbot_setInverseKine(self,pos,orn = None):
        '''

        :param pos: 机械臂末端位置，xyz
        :param orn: 机械臂末端方向，四元数
        :return: 机械臂各关节角度
        '''
        jointPoses = self._dofbot.setInverseKine(pos, orn)
        return jointPoses

    def dofbot_forwardKine(self,jointStates):
        return self._dofbot.forwardKinematic(jointStates)

    def get_dofbot_jointPoses(self):
        '''
        :return: 机械臂五个关节位置+夹爪角度
        '''
        jointPoses,gripper_angle = self._dofbot.get_jointPoses()

        return jointPoses,gripper_angle

    def get_dofbot_pose(self):
        '''
        :return: 机械臂末端位姿，xyz+四元数
        '''
        pos, orn = self._dofbot.get_pose()
        return pos, orn

    def get_block_pose(self):
        '''
        :return: 物块位姿，xyz+四元数
        '''
        pos,orn = self._object1.pos_and_orn()
        return pos, orn

    def get_target_pose(self):
        '''
        :return: 目标位置，xyz
        '''
        return self.target_pos


    def reward(self):
        '''
        :return: 是否完成抓取放置
        '''
        pos, orn = self._object1.pos_and_orn()
        dist = np.sqrt((pos[0] - self.target_pos[0]) ** 2 + (pos[1] - self.target_pos[1]) ** 2)
        if dist < 0.01 and pos[2] < 0.02:
            return True
        return False




