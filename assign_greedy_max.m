function pairs = assign_greedy_max(score)
[R,C] = size(score);
L = min(R,C);
pairs = zeros(0,2);
if L == 0
    return;
end
usedR = false(R,1);
usedC = false(C,1);
for ell = 1:L
    bestVal = -inf;
    bestR = 0;
    bestC = 0;
    for r = 1:R
        if usedR(r),continue; end
        for cc = 1:C
            if usedC(cc),continue; end
            if score(r,cc) > bestVal
                bestVal = score(r,cc);
                bestR = r;
                bestC = cc;
            end
        end
    end
    if bestR == 0
        break;
    end
    pairs = [pairs; bestR,bestC]; %#ok<AGROW>
    usedR(bestR) = true;
    usedC(bestC) = true;
end
end
