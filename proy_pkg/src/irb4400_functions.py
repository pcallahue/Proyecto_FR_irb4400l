import numpy as np
from copy import copy
#import rbdl

cos=np.cos; sin=np.sin; pi=np.pi

def dh(d, theta, a, alpha):
    cth = np.cos(theta)
    sth = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    T = np.array([[cth, -ca * sth, sa * sth, a * cth],
                    [sth, ca * cth, -sa * cth, a * sth],
                    [0, sa, ca, d],
                    [0, 0, 0, 1]])
    return T
    
    

def fkine_irb(q):

    # Longitudes (en metros)
    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(0.68, q[0],0.2,-pi/2)
    T2 = dh(0,pi/2+q[1],-0.89,0)
    T3 = dh(0,q[2],-0.15,pi/2)
    T4 = dh(1.733+q[4], q[3],0,-pi/2)
    T5 = dh(0,q[5],0,pi/2)
    T6 = dh(0.14,q[6],0,0)

    T = T1.dot(T2.dot(T3.dot(T4.dot(T5.dot(T6)))))
    return T

def jacobian_analitico(q, delta=0.0001):
    # Crear una matriz 3x6
    J = np.zeros((3,7))
    # Transformacion homogenea inicial (usando q)
    T=fkine_irb(q)
    # Iteracion para la derivada de cada columna
    for i in range(7):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i]=dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
        dT=fkine_irb(dq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
	    #columna =1/delta * (fkine(dq) - fkine(q))
        col_i = 1/delta * (dT[0:3,3] - T[0:3,3])
        J[:,i] = col_i
    return J

def jacobiano_geometrico(q):

    T01 = dh(0.68, q[0],0.2,-pi/2)
    T12 = dh(0,pi/2+q[1],-0.89,0)
    T23 = dh(0,q[2],-0.15,pi/2)
    T34 = dh(1.383+q[4], q[3],0,-pi/2)
    T45 = dh(0,q[5],0,pi/2)
    T56 = dh(0.14,q[6],0,0)

    # Matrices de transformacion homogenea con respecto a 0
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T05 = np.dot(T04, T45)
    T06 = np.dot(T05, T56)

    # Ejes z (con respecto al sistema 0)
    z0 = np.array([0, 0, 1])
    z1 = T01[0:3, 2]
    z2 = T02[0:3, 2]
    z3 = T03[0:3, 2]
    z4 = T04[0:3, 2]
    z5 = T05[0:3, 2]

    # Puntos con respecto al sistema 0
    p0 = np.array([0, 0, 0])
    p1 = T01[0:3, 3]
    p2 = T02[0:3, 3]
    p3 = T03[0:3, 3]
    p4 = T04[0:3, 3]
    p5 = T05[0:3, 3]
    p6 = T06[0:3, 3]

    # Componentes del Jacobiano (velocidad lineal)
    Jv1 = np.cross(z0, (p6-p0))
    Jv2 = np.cross(z1, (p6-p1))
    Jv3 = np.cross(z2, (p6-p2))
    Jv4 = np.cross(z3, (p6-p3))
    Jv5 = np.cross(z4, (p6-p4))
    Jv6 = np.cross(z5, (p6-p5))

    # Componentes del Jacobiano (velocidad angular)
    Jw1 = z0
    Jw2 = z1
    Jw3 = z2
    Jw4 = z3
    Jw5 = z4
    Jw6 = z5

    # Columas del Jacobiano geometrico
    Jcol1 = np.vstack((Jv1, Jw1))
    Jcol2 = np.vstack((Jv2, Jw2))
    Jcol3 = np.vstack((Jv3, Jw3))
    Jcol4 = np.vstack((Jv4, Jw4))
    Jcol5 = np.vstack((Jv5, Jw5))
    Jcol6 = np.vstack((Jv6, Jw6))

    # Jacobiano geometrico
    J = np.hstack((Jcol1, Jcol2, Jcol3, Jcol4, Jcol5, Jcol6))

    return J

def ikine_irb_newton(xd, q0):

    fqact = open("/home/user/cinv_xactual.txt", "w")
    fqdes = open("/home/user/cinv_xdeseado.txt", "w")
    qact=open("/home/user/cinv_qact.txt", "w")

    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.001

    q  = copy(q0)


    for i in range(max_iter):
        # Main loop
        J = jacobian_analitico(q,delta)
        T = fkine_irb(q)
        f = T[0:3,3]
        e = xd-f
        q = q+np.dot(np.linalg.pinv(J),e)
             

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
        
        if np.linalg.norm(e)<epsilon:
            break

        # Almacenamiento de datos
        fqact.write(str(f[0])+' '+str(f[1])+' '+str(f[2])+'\n')
        fqdes.write(str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
        qact.write(str(q[0])+' '+ str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+ str(q[4])+' '+ str(q[5])+' '+ str(q[6])+'\n')

#    fqact.close()
#    fqdes.close()
#    qact.close()


    return q

def ikine_irb_gradient(xdes,q0):

    epsilon = 0.0001
    max_iter = 10000
    delta = 0.0001
    alpha = 0.1

    q = copy(q0)

    for i in range(max_iter):
        # Main loop
        J = jacobian_analitico(q, delta)

        #  transformada homogenea, posicion
        T = fkine_irb(q)
        f = T[0:3, 3]

        # Error
        e = xdes - f

        # q(k+1)
        q = q + alpha*np.dot(J.T, e)

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
        
        if np.linalg.norm(e)<epsilon:
            break
        
    return q

################################################################
def rot2quat(R):
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):

    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R

#class Robot(object):
 #   def __init__(self, q0, dq0, ndof, dt):
   #     self.q = q0    # numpy array (ndof x 1)
  #      self.dq = dq0  # numpy array (ndof x 1)
    #    self.M = np.zeros([ndof, ndof])
     #   self.b = np.zeros(ndof)
      #  self.dt = dt
       # self.robot = rbdl.loadModel('../urdf/irb4400.urdf')

    #def send_command(self, tau):
     #   rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
      #  rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
       # ddq = np.linalg.inv(self.M).dot(tau-self.b)
        #self.q = self.q + self.dt*self.dq
        #self.dq = self.dq + self.dt*ddq

    #def read_joint_positions(self):
        #return self.q

    #def read_joint_velocities(self):
        #return self.dq