% This is the Example 3.1 on book
clear all
close all
clc
%%
A = [1 1;0 1];
B = [1;0.7];
%% Simulation parameter
T_end = 20;
Dt = 1;
t = 0:Dt:T_end;
Num = length(t);
% Penalty
Q = eye(2);
R = 1;
% For MPC
N = 3;
%% Constraint for system
Fx = [eye(2);-eye(2)];
gx = [2;5];
gx = [gx;gx];
X_Constraint = Polyhedron('A',Fx,'b',gx);

Fu = [1;-1];
gu = [1;1];
U_Constraint = Polyhedron('A',Fu,'b',gu);

K = -dlqr(A,B,Q,R);
Fc = [Fx; Fu*K];
gc = [gx; gu];
C=Polyhedron('A', Fc, 'b', gc);
%% Using the command to get C_N
sys = LTISystem('A',A,'B',B);
sys.x.min = [-2 -5];
sys.x.max = [2 5];
sys.u.min = -1;
sys.u.max = 1;
CN = sys.invariantSet();
%%
X_record = zeros(2,Num);
U_record = zeros(Num,1);
% Notice that X0 should be inside N-step Controlled Invariant Set
X_record(:,1) = [2,1];

[Aa,Ba] = ExpandSystem2(A,B,N);
Qa = Expand2N(Q,N);
Ra = Expand2N(R,N);
[G,E,S] = GESCompute(X_Constraint,U_Constraint,Aa,Ba,N);
H = Ba'*Qa*Ba + Ra;
% H = (H+H')/2;
F = Aa'*Qa*Ba;
% Cons = Polyhedron('A',G,'b',E*X_record(:,i)+S);
% Cons = Cons.minHRep;
for i = 1:Num-1
    % Using quadratic Programming get u
    f = X_record(:,i)'*F;
    u = quadprog(H,f',G,E*X_record(:,i)+S,[],[],-gu(1),gu(2));
    U_record(i) = u(1);
    % Update system
    X_record(:,i+1) = A*X_record(:,i) + B*U_record(i)*Dt;
end
% Plot
% Plot the actual system state and the reference system state
figure('Name','System state compare with Reference')
subplot(2,1,1);
stairs(t,X_record(1,:));
xlabel('Time t');
ylabel('x_1');
subplot(2,1,2);
stairs(t,X_record(2,:));
xlabel('Time t');
ylabel('x_2');

v_record = U_record(:,1)';
% Plot the control signal
figure('Name','Control signal generated');
stairs(t,v_record);
hold on
stairs(t,gu(1)*ones(1,length(t)),'b--');
stairs(t,-gu(2)*ones(1,length(t)),'b--');
xlabel('Time t');
ylabel('u');

figure
CN.plot