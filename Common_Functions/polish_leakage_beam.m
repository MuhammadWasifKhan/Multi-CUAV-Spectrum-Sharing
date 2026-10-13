function v = polish_leakage_beam(vIn,hSU,hProt,leakCapUnit,alphaFallback)
% Reduce leakage while retaining useful signal as much as possible.
Nt = numel(hSU);
% Candidate 1: leakage-aware RZF.
v1 = leakage_rzf_unit(hSU,hProt,alphaFallback);
% Candidate 2: null-space beam if possible.
Pnull = eye(Nt) - (hProt*hProt')/max(norm(hProt)^2,1e-30);
v2 = Pnull*hSU;
if norm(v2) > 1e-12
    v2 = v2/norm(v2);
else
    v2 = v1;
end
% Candidate 3: blend current beam and null-space beam.
v3 = vIn / max(norm(vIn),1e-30);
if abs(hProt'*v3)^2 > leakCapUnit && abs(hProt'*v2)^2 <= leakCapUnit*(1+1e-9)
    lo = 0; hi = 1;
    for bb = 1:40
        aa = 0.5*(lo+hi);
        vb = (1-aa)*v3 + aa*v2;
        vb = vb/max(norm(vb),1e-30);
        if abs(hProt'*vb)^2 <= leakCapUnit
            hi = aa;
        else
            lo = aa;
        end
    end
    v3 = (1-hi)*v3 + hi*v2;
    v3 = v3/max(norm(v3),1e-30);
end
cands = {v1,v2,v3};
best = cands{1};
bestScore = -inf;
for ii = 1:numel(cands)
    vc = cands{ii};
    vc = vc/max(norm(vc),1e-30);
    leak = abs(hProt'*vc)^2;
    sig  = abs(hSU'*vc)^2;
    if leak <= leakCapUnit*(1+1e-6)
        score = sig;
    else
        score = sig/(1 + 1e6*(leak/leakCapUnit - 1));
    end
    if score > bestScore
        bestScore = score;
        best = vc;
    end
end
v = best/max(norm(best),1e-30);
end
