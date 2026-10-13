function [pOut,addedTotal] = increase_from_own_band_headroom(pIn,cap,PmaxBand)
% Increase power using ONLY the same band's own unused budget and safe caps.
% No clipped power from the other band is used as an input.
%
% pOut satisfies:
%   sum(pOut) <= PmaxBand
%   pOut(ell) <= cap(ell)
%   pOut(ell) >= pIn(ell)
pIn = pIn(:);
cap = cap(:);
pOut = pIn;
addedTotal = 0;
if isempty(pIn) || PmaxBand <= 0
    return;
end
% First ensure the input itself does not violate caps or budget.
pOut = min(max(pOut,0),cap);
if sum(pOut) > PmaxBand
    pOut = pOut * (PmaxBand/max(sum(pOut),1e-30));
    pOut = min(pOut,cap);
end
safeHeadroom   = max(cap - pOut,0);
budgetHeadroom = max(PmaxBand - sum(pOut),0);
addBudget      = min(budgetHeadroom,sum(safeHeadroom));
if addBudget <= 1e-15
    return;
end
% Distribute own-band remaining power over streams with safe headroom.
remaining = addBudget;
for iter = 1:20
    head = max(cap - pOut,0);
    active = head > 1e-12;
    if ~any(active) || remaining <= 1e-12
        break;
    end
    add = zeros(size(pOut));
    add(active) = remaining * head(active) / sum(head(active));
    pTrial = min(pOut + add,cap);
    actual = sum(pTrial - pOut);
    pOut = pTrial;
    remaining = remaining - actual;
    if actual <= 1e-12
        break;
    end
end
% Final numerical guards.
if sum(pOut) > PmaxBand
    pOut = pOut * (PmaxBand/max(sum(pOut),1e-30));
end
pOut = min(pOut,cap);
addedTotal = sum(pOut - pIn);
end
