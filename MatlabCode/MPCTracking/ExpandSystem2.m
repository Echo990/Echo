function [Aa,Ba] = ExpandSystem2(A,B,N)
% function [Aa,Ba,Qa,Ra] = ExpandSystem(v_r,X_r,N,Dt)
% Input the reference velocitu v_r(e.g: 2)
%       the reference position X_r(e.g: [1,2,1])
%       the size of prediction horizon N(e.g: 5)
%       the sampling time Dt(e.g: 0.1)
% Need to set the initial Q and R matrix here

% Q = eye(3);
% R = eye(2);

Aa = [];
Temp = eye(2);
for i = 1:N
    Temp = A*Temp;
    Aa = [Aa;Temp];
end
Ba = [];
for i= 1:N
    Bi = [];
    Temp = eye(2);
    for j =1:i
        Bi = [Bi Temp*B]; 
        Temp = Temp*A;
    end
    for j = i:N-1
        Bi = [Bi zeros(size(B))];
    end
    Ba = [Ba;Bi];
end
% Qa = [];
% Ra = [];
% for i = 1:N
%     Qi = [];
%     Ri = [];
%     for j = 1:N
%         if i==j
%             Qi = [Qi Q];
%             Ri = [Ri R];
%         else
%             Qi = [Qi zeros(size(Q))];
%             Ri = [Ri zeros(size(R))];
%         end
%     end
%     Qa = [Qa;Qi];
%     Ra = [Ra;Ri];
% end