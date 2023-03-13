function P0=RobustCNCompute(Ai,Bi,D,Omega,U,W,X)
% This function can compute the C_N with W and Ac
% If there is no W in the system, then set W.A same as X.A and W.b=zeros(size(X.b))
% Ai = [1,1,:]
% D is positive
% Xc and W are both polyhedron
% Xc is constraint for both X and U
P0 = Omega;
max_iter_num = 1000;
for i = 1:max_iter_num
    P1 = P_Compute2(Ai,Bi,D,P0,U,W,X);
    if P0==P1
        fprintf('Max C_N found! Total iteration Num = %d\n',i);
        break
    else
        %P0.plot
        fprintf('Iteration num:%d\n',i);
    end
    % Record the new P
    P0 = P1;
end
end
function P1 = P_Compute2(Ai,Bi,D,P0,U,W,X)
    % Input P_0(F_0, g_0) and Aci
    % Output next P1
    n_w = length(W.b)/2;% Length of w(same as x)
    Size_u = size(U.A);
    n = length(Ai(1,1,:));% The number of A matrix(q)
    F0 = P0.A;
    g0 = P0.b;
    F = [];
    g = [];
    % get max w
    w = [];
    for l = 1:n_w
        w = [w;max(W.b(2*l-1:2*l))];
    end
    for j = 1:n
        %g = [g;g0-F0*D*w];
        g = [g;g0];
        F = [F;F0*Ai(:,:,j) F0*Bi(:,:,j)];
    end
    g = [g;U.b];
    F = [F;zeros(Size_u(1),n_w) U.A];
    P1 = Polyhedron('A',F,'b',g);
    P1 = P1.projection(1:n_w);
    P1 = P1&X;
    P1 = P1.minHRep();
end