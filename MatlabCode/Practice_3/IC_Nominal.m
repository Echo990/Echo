close all
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
%% Invarient Set Compute(Omega_Max)
% Use the min to compute a Omega as a test
sys = LTISystem('A',A.min+B*K.min);
Fx = [eye(2);-eye(2)];
gx = [5 5 5 5]';
X=Polyhedron('A', Fx, 'b', gx);

Fu = [1;-1];
gu = [1;1];
U = Polyhedron('A',Fu,'b',gu);

Fc = [Fx; Fu*K.min];
gc = [gx; gu];
C=Polyhedron('A', Fc, 'b', gc);

sys.x.with('setConstraint');
sys.x.setConstraint = C;
Omega = sys.invariantSet();

figure('Name','Omega_{max}');
xlabel('x_1')
ylabel('x_2')
Omega.plot;
hold on
X.plot('color','b', 'alpha', 0.1);% the limit of X
C.plot('color', 'w', 'alpha', 0.05);
legend('Omega','X','Constraint')

clear sys
%% Controlled_set Compute
%% Compute using my function
% Input the Constraint of X
%           target set Omega(C_0)
% Output a serious of C_k until C_N(max)
Max_iter_num = 100000;
C0 = Omega;% Can be Omega
figure('Name',"C_N Plot")
X.plot('color','b')
hold on
for i = 1:Max_iter_num
    Pre_C0 = Pre_Controlled_Compute(C0,A.min,B,U);
    C1 = Pre_C0 & X;
    C0.plot('color','w','alpha',0.2);
    if C1==C0
        fprintf('Maximum C_N found. Interation num:%d \n',i);
        break
    end
    C0 = C1;
end
hold off
%% Using the command to get C_N
sys = LTISystem('A',A.min,'B',B);
sys.x.min = [-5 -5];
sys.x.max = [5 5];
sys.u.min = -1;
sys.u.max = 1;
C_N = sys.invariantSet();
% Check if The C_N are the same
if C1==C_N
    disp('The Max controlled invarient set same.')
end
%% Plot C_N and Omega_max on the same plot
figure('Name','C_N and Omega_{max}')
C_N.plot('color','y');
hold on
Omega.plot('color','r');
xlabel('x_1');
ylabel('x_2');
legend('C_N','Omega_{max}');
%plot(-5,4.4,'Marker','o')
hold off
%% Explicit Vertex control law
% (prepare for x_v in interpolation control)
V = C_N.V;
temp1 = V(1,:);
temp2 = V(2,:);
temp3 = V(3,:);
temp4 = V(4,:);
V = [temp4;temp2;temp1;temp3];
vertex_x = V;
vertex_u = [-1 -1 1 1];
Pn = [];
for i=1:3
    Pp = Polyhedron('V',[V(i:i+1,:);zeros(1,2)]);
    Pn = [Pn Pp];
    K_aux = [vertex_u(i) vertex_u(i+1) 0]/[V(i,:)' V(i+1,:)' [0;0];ones(1,3)];
    Ki{i} = K_aux;
end

Pp = Polyhedron('V',[V(4,:); V(1,:); zeros(1,2)]);
Pn = [Pn Pp];
K_aux = [vertex_u(4) vertex_u(1) 0]/[V(4,:)' V(1,:)' [0;0];ones(1,3)];
Ki{4} = K_aux;
figure, Pn.plot
%% interpolation control
X0 = [-5;4.2];
f = [0 0 1]';
G = [C_N.A -C_N.b; -Omega.A Omega.b];
W = [zeros(size(C_N.b)); Omega.b];
E = [zeros(size(C_N.A)); -Omega.A];
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
    X0 = A.min*X0 + B*u(i);
    x1(i+1) = X0(1); 
    x2(i+1) = X0(2);
end
%%
figure();
subplot 211
stairs(t,x1,'b','linewidth',2);
xlabel('t')
ylabel('x_1')
subplot 212
stairs(t,x2,'b','linewidth',2);
xlabel('t')
ylabel('x_2') 

figure();
stairs(t,u,'b','linewidth',2);
xlabel('t')
ylabel('u') 

figure();
stairs(t,c,'linewidth',2)
xlabel('t')
ylabel('c')

figure('Name',"X trajectory");
C_N.plot('color','y');
hold on
Omega.plot('color','r');
xlabel('x_1');
ylabel('x_2');
plot(x1,x2,'-*b');
legend('C_N','Omega_{max}','x trajectory');
hold off