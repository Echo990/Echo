clear all
clc
%% System Parameters
% system matrix
v.min = 1;
v.max = 1.2;
A.min = [0 v.min;0 0];
A.max = [0 v.max;0 0];
% Discritisize the equation
A.min = ones(size(A.min)) + A.min;
A.max = ones(size(A.max)) + A.max;
B = [0;1];
% Weight matrix for cost function
Q = eye(2);
R = 1;
%% Compute the LQ controller
K.min = -dlqr(A.min,B,Q,R);
K.max = -dlqr(A.max,B,Q,R);

%% Set the constraint
Fx = [eye(2);-eye(2)];
gx = [5 5 5 5]';
X=Polyhedron('A', Fx, 'b', gx);

Fu = [1;-1];
gu = [1;1];
U = Polyhedron('A',Fu,'b',gu);

Fc = [Fx; Fu*K.min];
gc = [gx; gu];
C=Polyhedron('A', Fc, 'b', gc);

Fw = Fx;
gw = [0;0;0;0];
W = Polyhedron('A',Fw,'b',gw);
%%
Ac1 = A.min+B*K.min;
Ac2 = A.max+B*K.min;
%% Invarient Set Compute(Omega_Max)
% Use my own function to compute Omega
disp('Robust Omega max Computation start');
Omega = RobustOmegaCompute(cat(3,Ac1,Ac2),C,W);
disp('Found success');
figure('Name','Robust_Omega_{max}(Custom)');
xlabel('x_1')
ylabel('x_2')
Omega.plot;
hold on
X.plot('color','b', 'alpha', 0.1);% the limit of X
C.plot('color', 'w', 'alpha', 0.05);
legend('Omega','X','Constraint')
%% Using MPT3 compute the C_N
% Will not consider W
sys1 = LTISystem('A',A.min,'B',B);
sys1.x.min = [-5 -5];
sys1.x.max = [5 5];
sys1.u.min = -1;
sys1.u.max = 1;
CN1 = sys1.invariantSet();

sys2 = LTISystem('A',A.max,'B',B);
sys2.x.min = [-5 -5];
sys2.x.max = [5 5];
sys2.u.min = -1;
sys2.u.max = 1;
CN2 = sys2.invariantSet();
CN_Test = CN1&CN2;

figure('Name','C_N and Omega_{max} (MPT3)')
CN_Test.plot('color','y');
hold on
Omega.plot('color','r');
xlabel('x_1');
ylabel('x_2');
legend('C_N','Omega_{max}');
%plot(-5,4.4,'Marker','o')
hold off
clear sys1 sys2
%% Robust C_N Computation
% Computation is slow
% need to use "mincx" command to find "min(g-w)"

CN = RobustCNCompute(cat(3,A.min,A.max),cat(3,B,B),eye(2),Omega,U,W,X);
figure('Name','C_N and Omega_{max} (Custom)')
CN.plot('color','y');
hold on
Omega.plot('color','r');
xlabel('x_1');
ylabel('x_2');
legend('C_N','Omega_{max}');
hold off
%% Explicit Vertex control law
% (prepare for x_v in interpolation control)
V = CN_Test.V;
temp1 = V(1,:);
temp2 = V(2,:);
temp3 = V(3,:);
temp4 = V(4,:);
temp5 = V(5,:);
temp6 = V(6,:);
V = [temp1;temp2;temp4;temp5;temp6;temp3];
vertex_x = V;
vertex_u = [1 1 0 -1 -1 0];
Pn = [];
for i=1:5
    Pp = Polyhedron('V',[V(i:i+1,:);zeros(1,2)]);
    Pn = [Pn Pp];
    K_aux = [vertex_u(i) vertex_u(i+1) 0]/[V(i,:)' V(i+1,:)' [0;0];ones(1,3)];
    Ki{i} = K_aux;
end

Pp = Polyhedron('V',[V(6,:); V(1,:); zeros(1,2)]);
Pn = [Pn Pp];
K_aux = [vertex_u(6) vertex_u(1) 0]/[V(6,:)' V(1,:)' [0;0];ones(1,3)];
Ki{4} = K_aux;
figure, Pn.plot
%% interpolation control
X0 = [-5;4];
f = [0 0 1]';
G = [CN_Test.A -CN_Test.b; -Omega.A Omega.b];
W = [zeros(size(CN_Test.b)); Omega.b];
E = [zeros(size(CN_Test.A)); -Omega.A];
% Set simulation time
T_end = 40;
t = 0:T_end;
N = length(t);
% Initialize x,u,c
x1 = zeros(1,N); x1(1) = X0(1);
x2 = zeros(1,N); x2(1) = X0(2);
u = zeros(1,N); c = zeros(1,N);
% Start loop to simulate
for i=1:N-1
    if(Omega.contains(X0))
        u(i) = K.min*X0;
    else
        s = linprog(f,G,W+E*X0,[],[],[-inf,-inf,0],[inf,inf,1]);
        rv = s(1:2);
        ro = X0-rv;
        c(i) = s(3);
        xv=rv/c(i);
        % vertex control for rv
    for j=1:length(Pn)
        if Pn(j).contains(xv)
            uv = Ki{j}(1:2)*xv+Ki{j}(3);
            break
        end
    end
        % uo = K*xo; 
        % (1-c)xo = ro;
        % (1-c)*uo = (1-c)*K*xo = K*ro
        u(i) = c(i)*uv + K.min*ro;
    end
    Alpha = rand;
    A1 = Alpha*A.min+(1-Alpha)*A.max;
    X0 = A1*X0 + B*u(i);
    x1(i+1) = X0(1); 
    x2(i+1) = X0(2);
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

figure('Name','c-t');
stairs(t,c,'linewidth',2)
xlabel('t')
ylabel('c')

figure('Name',"X trajectory");
CN_Test.plot('color','y');
hold on
Omega.plot('color','r');
xlabel('x_1');
ylabel('x_2');
plot(x1,x2,'-*b');
legend('C_N','Omega_{max}','x trajectory');
hold off