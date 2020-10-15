import pybullet as p
import time
import pybullet_data
import IPython
import threading

class PyBulletThread(threading.Thread):
    def __init__(self, robotname, simtime):
        threading.Thread.__init__(self)
        self.robotId = None
        self.robotname = robotname
        self.simtime = simtime
        self.killed = False
        
    def pybullet_setup(self):
        # PyBullet set-up
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,0)

        # Load a plane to act as ground
        planeId = p.loadURDF('plane.urdf')

        # Load the robot model
        cubeStartPos = [0,0,1]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.robotId = p.loadURDF(self.robotname, cubeStartPos, cubeStartOrientation)

        #print("Number of joints: ", p.getNumJoints(boxId))
        #jointinfo = p.getJointInfo(boxId, 2)
        #print("Info being queried: ", jointinfo[7])

        timeinsecs = self.simtime  
        for i in range(timeinsecs*240):
           p.stepSimulation()
           time.sleep(1./240.)

        p.disconnect()

    def run(self):
        # Code for the thread
        print('Start PyBullet thread')
        self.pybullet_setup()

    def getalljointinfo(self):
        # 4 is fixed, 0 is revolute")

        print("Number of joints: ", p.getNumJoints(self.robotId))
        for i in range(p.getNumJoints(self.robotId)):
            info = p.getJointInfo(self.robotId, i)
            name = info[1]
            joint_type = info[2]
            if joint_type == 0:
                print("Number: {}, Name: {}, Type: {}".format(i, name, joint_type))

    def reset(self):
        for i in range(p.getNumJoints(self.robotId)):
            p.setJointMotorControl2(self.robotId, i, controlMode=p.VELOCITY_CONTROL, targetVelocity=0)
            
        p.restoreState(fileName="reset.bullet")

if __name__ == "__main__":
    sim = PyBulletThread("4-legged-robot-model/src/4leggedRobot.urdf", 1000)
    sim.start()



    # Start an ipython session
    ipy = threading.Thread(IPython.embed())
    ipy.start()
