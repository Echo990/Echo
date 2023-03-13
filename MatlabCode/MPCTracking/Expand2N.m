function Aa = Expand2N(A,N)
Aa = [];
for i = 1:N
    Ai = [];
    for j = 1:N
        if i==j
            Ai = [Ai A];
        else
            Ai = [Ai zeros(size(A))];
        end
    end
    Aa = [Aa;Ai];
end