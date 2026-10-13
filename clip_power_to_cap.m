function [pNew,delta,nClip] = clip_power_to_cap(pOld,cap)
% Clip powers to their interference-safe caps.
pOld = pOld(:);
cap = cap(:);
if isempty(pOld)
    pNew = pOld; delta = 0; nClip = 0; return;
end
pNew = min(pOld,cap);
pNew = max(pNew,0);
diff = max(pOld - pNew,0);
delta = sum(diff);
nClip = nnz(diff > 1e-12);
end
