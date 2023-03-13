close all
clear all
clc
%% Tracking trajectory generate
% Generate the trajectory
DeltaT = 0.1;
T_end = 30;
t = 0:DeltaT:T_end;
x_r = t;
y_r = zeros(size(x_r));
Theta_r = zeros(size(x_r));
% y_r = sin(0.01*x_r);
% Theta_r = cos(0.01*x_r);% geometry
% Theta_r = acosd(Theta_r);
% Plot the trajectory
figure('Name','Trajectory');
subplot(2,1,1);
plot(x_r,y_r);
title('Tracking Trajectory');
xlabel('x_r');
ylabel('y_r');
subplot(2,1,2);
plot(t,Theta_r);
title('Tracking Orientation');
xlabel('x_r');
ylabel('Theta_r');
%% System parameters
v = 1;
A = [1 1+v;1 1];
B = [0;1];
% Weight matrix for cost function
Q = eye(2);
R = 1;
%% Compute the LQ Controller
K = -dlqr(A,B,Q,R);
%% Set Constraint
% Constraint for tracking error
Fx = [eye(2);-eye(2)];
gx = [5 5 5 5]';
X=Polyhedron('A', Fx, 'b', gx);
% Constraint for control signal
Fu = [1;-1];
gu = [1;1];
U = Polyhedron('A',Fu,'b',gu);

Fc = [Fx; Fu*K];
gc = [gx; gu];
C=Polyhedron('A', Fc, 'b', gc);
%% Invarient Set Compute(Omega_Max)
Ac = A+B*K;
% Build the system
sys = LTISystem('A',Ac);
sys.x.with('setConstraint');
sys.x.setConstraint = C;
Omega = sys.invariantSet();
clear sys
%% C_N Computation
N = 10;
C0 = Omega;
figure('Name',"C_N Plot")
X.plot('color','b');
hold on
for i = 1:N
    Pre_C0 = Pre_Controlled_Compute(C0,A,B,U);
    C1 = Pre_C0 & X;
    if C1==C0
        fprintf('Maximum C_N found. Interation num:%d \n',i);
        break
    end
    C0 = C1;
end
C1.plot('color','y');
Omega.plot('color','r')
hold off
title('Controlled Invariant set');
xlabel('x_1');
ylabel('x_2');
legend('X','C_N','Omega');
CN = C1;
%% Improved IC with Minkowski
% e0 = [-4.8;4];
e0 = [-4.8;4]/1.9;
Target_point = [y_r;Theta_r];
% Actual system state = target+error
x_sys = [x_r(1);Target_point(:,1)+e0];

N = length(t);

e1 = zeros(1,N); e1(1) = e0(1);
e2 = zeros(1,N); e2(1) = e0(2);
u = zeros(1,N);
x_record = zeros(3,N);
Target_i = 1;
for i=1:N-1
    if ~Omega.contains(10*e0)
        Target_i = Target_i + 1;
    end
    e1(i+1) = x_sys(2)-y_r(Target_i); 
    e2(i+1) = x_sys(3)-Theta_r(Target_i);
    e0 = [e1(i+1);e2(i+1)];

    s2 = linprog([1 0 0 0]',[-CN.b CN.A*B -CN.A*(A+B*K); Omega.b zeros(size(Omega.A,1),1) Omega.A ],[-CN.A*A*e0;Omega.b],[],[],[0 -gu(1) -inf -inf], [1 gu(2) inf inf]);
    u(i)=s2(2);
    
    x_record(:,i) = x_sys;
    x_sys = SysUpdate(x_sys,u(i),DeltaT);
end
%%
figure('Name','System state and Reference');
subplot 311
plot(t(1:N-1),x_record(1,1:N-1),'b');
hold on
plot(t,x_r,'r');
hold off
xlabel('t')
ylabel('x')
legend('x','x_r','--')
subplot 312
plot(t(1:N-1),x_record(2,1:N-1),'b');
hold on
plot(t,y_r,'--r');
hold off
xlabel('t')
ylabel('y')
legend('y','y_r','--')
subplot 313
plot(t(1:N-1),x_record(3,1:N-1),'b');
hold on
plot(t,Theta_r,'--r');
hold off
xlabel('t')
ylabel('Theta')
legend('Theta','Theta_r')

figure('Name','Control u');
stairs(t,u,'b',LineWidth=2);
xlabel('t')
ylabel('u') 

% figure('Name','c-t');
% stairs(t,c,'linewidth',2)
% xlabel('t')
% ylabel('c')

figure('Name',"X trajectory");
CN.plot('color','y');
hold on
Omega.plot('color','r');
xlabel('e_1');
ylabel('e_2');
plot(e1,e2,'-*b');
hold off
%%

function x = SysUpdate(x_0,u,T)
v = 1;
A = zeros(3);
A(2,3) = 1;
B = [1 0;0 0;0 1];
x = (eye(3)+v*A*T)*x_0 + T*B*[v;u]; 
end