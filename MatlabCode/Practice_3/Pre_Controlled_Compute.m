function Pre = Pre_Controlled_Compute(Constraint_x,A,B,Constraint_u)
% Input the Constraint of x (C_k)
%           Constraint of u (U)
% Output the pre(C_k)  (.A is F;.b is g)
FoA = Constraint_x.A*A;
FoB = Constraint_x.A*B;
x_num = size(FoA);
x_num = x_num(2);
u_num = length(Constraint_u.b);
P =  Polyhedron('A', [FoA FoB;zeros(x_num,u_num) Constraint_u.A],...
    'b', [Constraint_x.b;Constraint_u.b]);
Pre = P.projection(1:x_num);