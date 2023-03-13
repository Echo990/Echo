close all
clear all
clc
%% Simulation parameter
T_end = 35;
Dt = 0.1;
t = 0:Dt:T_end;
Num = length(t);
% Penalty
Q = 0.5*eye(3);
R = eye(2);
% For MPC
N = 5;
%% Reference Model
v_r = 0.8*(cos(0.1*t-pi/2)+1);
w_r = 0.2*sin(0.1*t)-0.1;
X_r = zeros(3,Num);
for i = 1:Num-1
    X_r(:,i+1) = SysUpdate(X_r(:,i),[v_r(i);w_r(i)],Dt);
end
%% Plot the reference position and reference control
figure('Name','v_r and w_r plot');
subplot(2,1,1);
plot(t,v_r);
xlabel('t');
ylabel('v_r');
title('reference velocity');
subplot(2,1,2);
plot(t,w_r);
xlabel('t');
ylabel('w_r');
title('reference angle velocity');

figure('Name','Reference position');
subplot(3,1,1);
plot(t,X_r(1,:));
xlabel('t');
ylabel('x_r');
subplot(3,1,2);
plot(t,X_r(2,:));
xlabel('t');
ylabel('y_r');
subplot(3,1,3);
plot(t,X_r(3,:));
xlabel('t');
ylabel('Theta_r');

figure('Name','Trajectory')
plot(X_r(1,:),X_r(2,:));
xlabel("x_r");
ylabel('y_r');
%% Constraint for system
Fx = [eye(3);-eye(3)];
gx = [50;50;pi];
gx = [gx;gx];
X_Constraint = Polyhedron('A',Fx,'b',gx);

Fu = [eye(2);-eye(2)];
gu = [5;pi/2];
gu = [gu;gu];
U_Constraint = Polyhedron('A',Fu,'b',gu);

% K = -dlqr(A,B,Q,R);
% Fc = [Fx; Fu*K];
% gc = [gx; gu];
% C=Polyhedron('A', Fc, 'b', gc);
%% Simulation
Error0 = [2 2 pi/30]';
X_record = zeros(3,Num);
Error_record = zeros(3,Num);
Error_record(:,1) = Error0;
X_record(:,1) = Error0 + X_r(:,1);
U_record = zeros(Num,2);
for i = 1:Num-1
    A = eye(3)+[0 0 -v_r(i)*sin(X_r(3,i));0 0 v_r(i)*cos(X_r(3,i));0 0 1];
    B = [Dt*sin(X_r(3,i)) 0;Dt*cos(X_r(3,i)) 0;0 Dt];
    model = LTISystem('A',A,'B',B);
    model.x.min = [-50 -50 -pi/2];
    model.x.max = [50 50 pi/2];
    model.u.min = [-5 -pi/2];
    model.u.max = [5 pi/2];
    model.x.penalty = QuadFunction(Q);
    model.u.penalty = QuadFunction(R);
    mpc = MPCController(model,N);
    u = mpc.evaluate(Error_record(:,i));
    U_record(i,:) = u'+ [v_r(i) w_r(i)];
    % Update system
%     Error_record(:,i+1) = SysUpdate(Error_record(:,i),U_record(i,:)',Dt);
%     X_record(:,i+1) = Error_record(:,i+1) + X_r(:,i+1);
    X_record(:,i+1) = SysUpdate(X_record(:,i),U_record(i,:)',Dt);
    Error_record(:,i+1) = X_record(:,i+1) - X_r(:,i+1);
    fprintf('Interation num = %d/%d\n',i,Num-1);
end
%% Plot
% Plot the actual system state and the reference system state
figure('Name','System state compare with Reference')
subplot(3,1,1);
plot(t,X_record(1,:),t,X_r(1,:),'--');
xlabel('Time t');
ylabel('x and x_r');
legend('x','x_r');
subplot(3,1,2);
plot(t,X_record(2,:),t,X_r(2,:),'--');
xlabel('Time t');
ylabel('y and y_r');
legend('y','y_r');
subplot(3,1,3);
plot(t,X_record(3,:),t,X_r(3,:),'--');
xlabel('Time t');
ylabel('Theta and Theta_r');
legend('Theta','Theta_r');

% Plot the Error
figure('Name','Tracking error');
subplot(3,1,1);
plot(t,Error_record(1,:),t,gx(1)*ones(1,length(t)),'b--',t,-gx(4)*ones(1,length(t)),'b--');
xlabel('Time t');
ylabel('x Error');
subplot(3,1,2);
plot(t,Error_record(2,:),t,gx(2)*ones(1,length(t)),'b--',t,-gx(5)*ones(1,length(t)),'b--');
xlabel('Time t');
ylabel('y Error');
subplot(3,1,3);
plot(t,Error_record(3,:),t,gx(3)*ones(1,length(t)),'b--',t,-gx(6)*ones(1,length(t)),'b--');
xlabel('Time t');
ylabel('Theta Error');

v_record = U_record(:,1)';
w_record = U_record(:,2)';
% Plot the control signal
figure('Name','Control signal generated');
subplot(2,1,1);
plot(t,v_record,t,v_r);
hold on
plot(t,gu(1)*ones(1,length(t)),'b--',t,-gu(3)*ones(1,length(t)),'b--');
legend('v','v_r')
xlabel('Time t');
ylabel('v');
subplot(2,1,2);
plot(t,w_record,t,w_r);
hold on
plot(t,gu(2)*ones(1,length(t)),'b--',t,-gu(4)*ones(1,length(t)),'b--');
legend('w','w_r')
xlabel('Time t');
ylabel('w');