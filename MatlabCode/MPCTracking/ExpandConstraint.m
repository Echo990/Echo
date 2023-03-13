function Xa = ExpandConstraint(X,N)
Fx = X.A;
gx = X.b;
Fa = Expand2N(Fx,N);
ga = repmat(gx,N,1);
% for i = 1:N
%     Fai = [];
%     gai = [];
%     for j = 1:N
%         if i==j
%             Fai = [Fai Fx];
%             gai = [gai gx];
%         else
%             Fai = [Fai zeros(size(Fai))];
%             gai = [gai zeros(size(gai))];
%         end
%     end
%     Fa = [Fa;Fai];
%     ga = [ga;gai];
% end
Xa = Polyhedron('A',Fa,'b',ga);
