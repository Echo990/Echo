close all
clear all
clc
%% System parameters
v = 1;
A = [1 1+v;1 1];
B = [0;1];
% Weight matrix for cost function
Q = eye(2);
R = 0.1;
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
x0 = [-4.9;4.1];

T_end = 30;
t = 0:T_end;
N = length(t);

x1 = zeros(1,N); x1(1) = x0(1);
x2 = zeros(1,N); x2(1) = x0(2);
u = zeros(1,N);

for i=1:N-1
    s2 = linprog([1 0 0 0]',[-CN.b CN.A*B -CN.A*(A+B*K); Omega.b zeros(size(Omega.A,1),1) Omega.A ],[-CN.A*A*x0;Omega.b],[],[],[0 -gu(1) -inf -inf], [1 gu(2) inf inf]);
    u(i)=s2(2);
    x0 = A*x0+B*u(i);
    x1(i+1) = x0(1);
    x2(i+1) = x0(2);
end
figure('Name','System state x_1 and x_2');
subplot 211
stairs(t,x1,'b','linewidth',2);
xlabel('t')
ylabel('x_1')
subplot 212
stairs(t,x2,'b','linewidth',2);
xlabel('t')
ylabel('x_2') 

figure('Name','Control u');
stairs(t,u,'b','linewidth',2);
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
xlabel('x_1');
ylabel('x_2');
plot(x1,x2,'-*b');
legend('C_N','Omega','Trajectory')
hold off