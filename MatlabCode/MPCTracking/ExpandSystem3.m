function [Aa,Ba] = ExpandSystem3(v_r,X_r,N,Dt)
% function [Aa,Ba,Qa,Ra] = ExpandSystem(v_r,X_r,N,Dt)
% Input the reference velocitu v_r(e.g: 2)
%       the reference position X_r(e.g: [1,2,1])
%       the size of prediction horizon N(e.g: 5)
%       the sampling time Dt(e.g: 0.1)
% Need to set the initial Q and R matrix here
% A1 = eye(3)+[0 0 -v_r*sin(X_r(3));0 0 v_r*cos(X_r(3));0 0 1];
% B1 = [Dt*sin(X_r(3)) 0;Dt*cos(X_r(3)) 0;0 Dt];
% Q = eye(3);
% R = eye(2);

Aa = [];
Temp = eye(3);
for i = 1:N
    A1 = eye(3)+[0 0 -v_r(i)*sin(X_r(3,i));0 0 v_r(i)*cos(X_r(3,i));0 0 1];
    Temp = A1*Temp;
    Aa = [Aa;Temp];
end
Ba = [];
for i= 1:N
    Bi = [];
    B1 = [Dt*sin(X_r(3,i)) 0;Dt*cos(X_r(3,i)) 0;0 Dt];
    Temp = eye(3);
    for j =1:i
        A1 = eye(3)+[0 0 -v_r(i)*sin(X_r(3,i));0 0 v_r(i)*cos(X_r(3,i));0 0 1];
        Bi = [Bi Temp*B1]; 
        Temp = Temp*A1;
    end
    for j = i:N-1
        Bi = [Bi zeros(size(B1))];
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