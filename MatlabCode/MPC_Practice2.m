% System defined by matrix A and B
A = [1 1; 0 1];
B = [1; 0.3];

% LQ controller for the previous system
%[~,~,G] = dare(A,B,eye(2),1);    %A,B, Q, R
K_lqr = -dlqr(A,B,eye(2),1);
K = -G;
%K