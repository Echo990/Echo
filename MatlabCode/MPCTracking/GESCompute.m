function [G,E,S] = GESCompute(X,U,Aa,Ba,N)
xa = ExpandConstraint(X,N);
ua = ExpandConstraint(U,N);
Fxa = xa.A;
Fua = ua.A;
gxa = xa.b;
gua = ua.b;
G = [Fua;Fxa*Ba];
m = size(ua.A);
m = m(1);
n = size(-xa.A*Aa);
n = n(2);
E = [zeros(m,n);-Fxa*Aa];
S = [gua;gxa];