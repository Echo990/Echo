function x = SysUpdate(x_0,u,T)
B = [cos(x_0(3)) 0;sin(x_0(3)) 0;0 1];
x = eye(3)*x_0 + T*B*u; 
end