%Maklen A. Estrada
%Nonlinear Control 
%Model Reference Adaptive Control

clear all
clc
close all

%Initial Condition for intigrator in Simulink
q0 = .001;
u0 = 0;
q_ref0 = .001;

%Simulation Run Time
TF = 30;

%Pitch Rate Command Sin Function
% q_cmd = Apmplitude * sin ( Frequency * t)
Amplitude = 1/pi;
Frequency = 1;  %rad/s

%Run the Model
ans = sim('MRAC.slx');
ans2 = sim('MRAC_Ideal.slx');
ans3 = sim('MRAC_Ref.slx');

%Simulation Data
t = ans.Gains.Time;
q = ans.q.Data(:,1);
u = ans.u.Data(:,1);
kq = ans.Gains.Data(:,1);
kcmd = ans.Gains.Data(:,2);
Theta = ans.Gains.Data(:,3);

t_ideal = ans2.q_ideal.Time;
q_ideal = ans2.q_ideal.Data(:,1);
f_ideal = ans2.f.Data(:,1);

t_ref = ans3.q_ref.Time;
q_ref = ans3.q_ref.Data(:,1);

%Ideal Gains 
a_ref = -4;
b_ref = 4;
Mq = -.61;
Md = -6.65;
kq_i = (a_ref - Mq)/Md;
k_cmd_i = b_ref/Md;
Theta_ideal = -f_ideal;
kq_ideal = kq_i*ones(length(t_ideal));
k_cmd_ideal = k_cmd_i*ones(length(t_ideal));

%% Resample Cpp Data
q_cpp_interp = Cpp_Sim_Result(t);
%% Plot the Results 

figure
hold on
plot(t,q,'k','LineWidth',2)
plot(t,q_cpp_interp,'b','LineWidth',2)
hold off
xlabel('Time (s)')
ylabel('q (deg/s)')
legend('Matlab & Simulink Model','C++ Model')
grid on

figure
hold on
plot(t,q,'k','LineWidth',2)
plot(t_ideal,q_ideal,'b','LineWidth',2)
plot(t_ref,q_ref,'g','LineWidth',2)
hold off
xlabel('Time (s)')
ylabel('q (deg/s)')
legend('Adaptive','Fixed-Gain')
grid on

figure
plot(t,u,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Control Input')
grid on

figure
subplot(3,1,1)
hold on
plot(t,kq,'k', 'LineWidth',2)
plot(t_ideal,kq_ideal,'b','LineWidth',2)
hold off
xlabel('Time (s)')
ylabel('k_q')
legend('Adaptive','Fixed-Gain')
grid on

subplot(3,1,2)
hold on
plot(t,kcmd, 'k','LineWidth',2)
plot(t_ideal,k_cmd_ideal,'b', 'LineWidth',2)
hold off
xlabel('Time (s)')
ylabel('k_{qcmd}')
legend('Adaptive','Fixed-Gain')
grid on

subplot(3,1,3)
hold on
plot(t,Theta, 'k','LineWidth',2)
plot(t_ideal,Theta_ideal,'b', 'LineWidth',2)
hold off
xlabel('Time (s)')
ylabel('Theta')
legend('Adaptive','Fixed-Gain')
grid on

figure
q_n = imresize(q, [length(q_ref),1]);
e = q_n - q_ref;
plot(t_ref,e,'b', 'LineWidth',2)
xlabel('Time (s)')
ylabel('Tracking Error')
grid on
