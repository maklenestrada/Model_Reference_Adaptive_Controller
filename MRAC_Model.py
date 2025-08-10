import numpy as np
import math as m
import matplotlib.pyplot as plt



#Runga-Kutta 4 Integrator 
def rk4(func, x,dt) :
    k1 = dt*func(x);
    k2 = dt*func(x + 0.5*k1);
    k3 = dt*func(x + 0.5*k2);
    k4 = dt*func(x + k3);
    return x + (k1 + 2*k2 + 2*k3 + k4)/6

#Dynamics 
def Dynamics(q,u) :
     #Define Constants
     Mq = -0.61;
     Md = -6.65;    
     Theta = -0.01;

     #Known Regressor
     Phi = np.tanh((360/np.pi) * q);
     f = Theta*Phi;
     
     #Pitch Rate
     dqdt = (Mq*q) + Md*(u+f);
     return dqdt

def AdaptiveLaws(q, q_ref, q_cmd) :
    #Define Constants 
    gamma_q = 6000;
    gamma_cmd = 6000;
    Gamma_theta = 8;
    
    Phi = np.tanh(360/np.pi *q);
    kq_dot = gamma_q * q * (q - q_ref);
    kcmd_dot = gamma_cmd * q_cmd * (q - q_ref);
    theta_dot = -Gamma_theta * Phi * (q - q_ref);
    
    return [kq_dot,kcmd_dot,theta_dot]

def Controller(k_q, k_q_cmd,theta,q_cmd,q):
    Phi = np.tanh((360/np.pi)*q);
    u = k_q * q + k_q_cmd * q_cmd - theta*Phi;
    return u

def ReferenceModel(q_ref,q_cmd) :
    q_ref_dot = 4 * (q_cmd - q_ref);
    return q_ref_dot

########################################################################

#Initial Conditions 
q = 0.01;
q_ref = 0.01;

#Adaptive Parameters
k_q = 0.01;
k_q_cmd = 0.01;
theta = 0.01;

#Simulation Time
t0 = 0.0;
t_end = 30;
dt = 0.01;
t_arr =  np.arange(t0, t_end, step = dt);

#Sin wave parameters for q_cmd
amplitude = 1/np.pi;

#Array to store data
q_data = [];
q_ref_data = [];
u_data = [];
k_q_data = [];
k_q_cmd_data = [];
theta_data = [];
e_data = [];

#Simulation Loop
for t in np.arange(t0, t_end, step = dt):
    #Command Signal - Sin Wave
    q_cmd = amplitude * m.sin(t);
    
    #Control Input
    u = Controller(k_q, k_q_cmd, theta, q_cmd, q);
    
    #Integrate System Dynamics 
    q = rk4(lambda q_val: Dynamics(q, u),q ,dt);
    
    #Integrate Reference Model
    q_ref = rk4(lambda q_ref_val: ReferenceModel(q_ref, q_cmd), q_ref, dt);
    
    #Integrate Adaptive Laws
    d_kq, d_kqcmd, d_th = AdaptiveLaws(q, q_ref, q_cmd)
    k_q = rk4(lambda AL: d_kq, k_q, dt);
    k_q_cmd = rk4(lambda AL: d_kqcmd, k_q_cmd, dt);
    theta = rk4(lambda AL: d_th, theta, dt);
    #Tracking Error
    e = q - q_ref;
    
    #Add data
    q_data.append(q);
    q_ref_data.append(q_ref);
    u_data.append(u);
    k_q_data.append(k_q);
    k_q_cmd_data.append(k_q_cmd);
    theta_data.append(theta);
    e_data.append(e);

#Plots 
plt.figure()
plt.plot(t_arr, q_data, color = 'r');
plt.xlabel('Time(s)')
plt.ylabel('q (deg/s)');
plt.grid(visible=True);
plt.show

plt.figure()
plt.plot(t_arr,q_ref_data, color = 'r');
plt.xlabel('Time(s)')
plt.ylabel('$q_{ref}$ (deg/s)')
plt.grid(visible=True);
plt.title('Reference Model')
plt.show

plt.figure()
plt.plot(t_arr,u_data, color = 'r');
plt.xlabel('Time(s)')
plt.ylabel('Control Input');
plt.grid(visible=True);
plt.show

plt.figure()
plt.subplot(3,1,1)
plt.plot(t_arr,k_q_data, color = 'r');
plt.xlabel('Time (s)');
plt.ylabel('$k_q$');
plt.grid(visible=True);

plt.subplot(3,1,2)
plt.plot(t_arr,k_q_cmd_data, color = 'r');
plt.xlabel('Time (s)');
plt.ylabel('$k_{q_{cmd}}$');
plt.grid(visible=True);

plt.subplot(3,1,3)
plt.plot(t_arr,theta_data,color = 'r');
plt.xlabel('Time (s)');
plt.ylabel('$\Theta$');
plt.grid(visible=True);
plt.tight_layout()
plt.show

plt.figure()
plt.plot(t_arr,e_data, color = "r");
plt.xlabel('Time (s)');
plt.ylabel('Error');
plt.title('q - $q_{ref}$')
plt.show()
