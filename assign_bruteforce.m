function pairs = assign_bruteforce(cost)
[R,C] = size(cost);
L = min(R,C);
pairs = zeros(L,2);
if L==0,return; end
bestVal = inf;
if R <= C
    P = perms(1:C);
    for i=1:size(P,1)
        sel = P(i,1:R);
        val = 0;
        for r=1:R
            val = val + cost(r,sel(r));
        end
        if val < bestVal
            bestVal = val;
            pairs = [(1:R)',sel(:)];
        end
    end
else
    P = perms(1:R);
    for i=1:size(P,1)
        sel = P(i,1:C);
        val = 0;
        for cc=1:C
            val = val + cost(sel(cc),cc);
        end
        if val < bestVal
            bestVal = val;
            pairs = [sel(:),(1:C)'];
        end
    end
    pairs = pairs(1:L,:);
end
pairs = pairs(1:L,:);
end
%% ============================================================
%                  TRAJECTORY UPDATE HELPERS
%% ============================================================
