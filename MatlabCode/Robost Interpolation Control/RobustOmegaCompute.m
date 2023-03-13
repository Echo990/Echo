function P1 = RobustOmegaCompute(Aci,Xc,W)
% This function can compute the Omega with W and Ac
% If there is no W in the system, then set W.A same as X.A and W.b=zeros(size(X.b))
% Because in this Demo, D = eye(2) so 'Dw = w'
% For other D, please add input D and change the code in line 39
% Aci = [1,1,:]
% Xc and W are both polyhedron
% Xc is constraint for both X and U

P0 = Xc;
max_iter_num = 1000;
for i = 1:max_iter_num
    P1 = P_Compute1(Aci,P0,W);
    if P0==P1
        disp('Max Omega found!');
        break
    else
        fprintf('Iteration num:%d\n',i);
    end
    % Record the new P
    P0 = P1;
end
end

function P1 = P_Compute1(Aci,P0,W)
    % Input P_0(F_0, g_0) and Aci
    % Output next P1
    n = length(Aci(1,1,:));% The number of A matrix(q)
    F = P0.A;
    g = P0.b;
    % get max w
    n_w = length(W.b);
    w = [];
    for l = 1:n_w/2
        w = [w;min(W.b(2*l-1:2*l))];
    end
    for j = 1:n
        g = [g;g-F*w];% If D!=eye, then use "g = [g;g-F*D*w]" instead
        F = [F;F*Aci(:,:,j)];
    end
    P1 = Polyhedron('A',F,'b',g);
    P1 = P1.minHRep();
end