%% System Model
v = 1;% velocity of mobile robot
A = [0 v;0 0];
B = [0;1];

Q = eye(2);
R = 1;
% Algorithm4.2: Interpolation based control-Explicit solution(Book: HNN, page126)
% Input 
% -----
% C_N: N-step controlled set
% Omega_max: Maximum invarient set
% U: Control values at C_N

%% Step1: Calculate the LQ controller for Omega_max
K = -lqr(A,B,Q,R);
% [~,G,~] = idare(A,B,Q,R);
%K = -G;
%% Step2: Calculate the Maximum invarient set with K
sys = LTISystem('A',A+B*K)

Fx = [eye(2); -eye(2)] 
gx = [5 5 5 5]' 
Fu = [1; -1] 
gu = [1; 1] 
Fc = [Fx; Fu*K] 
gc = [gx; gu] 

C=Polyhedron('A', Fc, 'b', gc);
sys.x.with('setConstraint');
sys.x.setConstraint = C;
Omega = sys.invariantSet()
figure();

Omega.plot
title('MAS Omega')
xlabel('x_1')
ylabel('x_2')

X=Polyhedron('A', Fx, 'b', gx)
hold on
X.plot('color','b', 'alpha', 0.1)
C.plot('color', 'w', 'alpha', 0.05) 

%% N-step controlled set C_N
syst = LTISystem('A', A,'B',B) 
syst.x.min = [-5; -5] 
syst.x.max = [5; 5] 
syst.u.min = [-1] 
syst.u.max = [1] 
Q = [1 0; 0 1];
syst.x.penalty = QuadFunction(Q);
R = 1;
syst.u.penalty = QuadFunction(R);
InvSet = syst.invariantSet() %note that Omega is not needed in the mpt3 algo
figure
InvSet.plot('color','y')
hold on
Omega.plot
legend('C_N','Omega_{max}')
%% Find the Vertices and control values at them
V = [5,5;5,-5;-5,5;-5,-5];
vertex_x=V';
vertex_u=[1 1 -1 -1];
vertex_x_evol=A*vertex_x+B*vertex_u
CN_evol=Polyhedron('V', vertex_x_evol');

figure
InvSet.plot('color','y', 'alpha', 0.2)
hold on
CN_evol.plot('color','k', 'alpha', 0.3)
title('CN and evolution of its vertices with given control values')
xlabel('x_1')
ylabel('x_2')
legend('C_N','C_{N-1}')
