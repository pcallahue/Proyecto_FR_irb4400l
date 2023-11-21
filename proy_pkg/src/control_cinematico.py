#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

from markers import *
from irb4400_functions import *


if __name__ == '__main__':

    # Initialize the node
    rospy.init_node("KineControlPosition")
    # Publisher: publish to the joint_states topic
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    
    # Files for the logs
    fxact = open("/home/user/cc_xact.txt", "w")                
    fxdes = open("/home/user/cc_xdes.txt", "w")
    fq = open("/home/user/cc_q.txt", "w")

    # Markers for the current and desired positions
    bmarker_current  = BallMarker(color['RED'])
    bmarker_desired = BallMarker(color['GREEN'])

    # Joint names
    jnames = ("joint_1", "joint_2", "joint_3","joint_4", "joint_4n", "joint_5", "joint_6")

    # Desired position
    xd = np.array([1.6, -1.89, 2.07])
    
    # Initial configuration
    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Resulting initial position (end effector with respect to the base link)
    T = fkine_irb(q0)
    x0 = T[0:3,3]

    # Red marker shows the achieved position
    bmarker_current.xyz(x0)
    # Green marker shows the desired position
    bmarker_desired.xyz(xd)


    # Instance of the JointState message
    jstate = JointState()
    # Values of the message
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames
    # Add the head joint value (with value 0) to the joints
    jstate.position = q0


    # Frequency (in Hz) and control period 
    freq = 100
    dt = 1.0/freq
    rate = rospy.Rate(freq)

    # Initial joint configuration
    q = copy(q0)
    # Max error
    epsilon = 0.001
    # Values
    k = 1 # Ganancia
    cnt = 1 # Contador

    # Initial joint configuration
    q = copy(q0)
    
    # Main loop
    while not rospy.is_shutdown():
        # Current time (needed for ROS)
        jstate.header.stamp = rospy.Time.now()

        # Kinematic control law for position (complete here)
        # -----------------------------
        J = jacobian_analitico(q)
        T = fkine_irb(q)
        x = T[0:3,3]
        e = x - xd

    
        if (np.linalg.norm(e)<epsilon):
            print(np.linalg.norm(e))
            break

        #Derivada del error
        de = -k*e
        # Variacion de la configuracion articular
        dq = np.linalg.pinv(J).dot(de)
        # Integracion para obtener la nueva configuracion articular
        q = q + dt*dq
        

        # Condicion para finalizar
        if q[0]<-2.87979:
            q[0]=-2.87979		
        if q[0]>2.87979:
            q[0]=2.87979
        if q[1]<-1.2217:		
            q[1]=-1.2217

        if q[1]>1.658:
            q[1]=1.658
        if q[2]<-1.0472:
            q[2]=-1.0472		
        if q[2]>1.1345:
            q[2]=1.1345

        if q[3]<-3.49:
            q[3]=-3.49		
        if q[3]>3.49:
            q[3]=3.49

        if q[4]<-0.35:		
            q[4]=-0.35		
        if q[4]>0.35:
            q[4]=0.35

        if q[5]<-2.0944:		
            q[5]=-2.0944	
        if q[5]>2.0944:
            q[5]=2.0944

        if q[6]<-6.9813:
            q[6]=-6.9813
        if q[6]>6.9813:
            q[6]=6.9813
        


        # Solamente para evitar un bucle infinito si algo sale mal
        cnt = cnt+1
        if (cnt > 1e5):
            break


        # Log values
        fxact.write(str(x[0])+' '+str(x[1]) +' '+str(x[2])+'\n')
        fxdes.write(str(e[0])+' '+str(e[1])+' '+str(e[2])+'\n')
        fq.write(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+ str(q[4])+" "+str(q[5])+" "+str(q[6])+"\n")

        # Publish the message
        jstate.position = q
        pub.publish(jstate)
        bmarker_desired.xyz(xd)
        bmarker_current.xyz(x)
        # Wait for the next iteration
        rate.sleep()


    fxact.close()
    fxdes.close()
    fq.close()

