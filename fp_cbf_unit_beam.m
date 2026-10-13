function [v,gSig,gLeak] = fp_cbf_unit_beam(hSU,hProt,denom,Gamma,pRef,FP_iter,leakFrac,alphaFallback)
% FP-based CBF unit-norm beam design.
% ------------------------------------------------------------
% Uses a quadratic-transform style update for the SINR objective:
%   log(1 + p|h^H v|^2/denom)
% combined with a leakage regularization term for PU/WU protection.
% This is CVX-free and suited for repeated Monte-Carlo simulation.
Nt = numel(hSU);
Hs = hSU*hSU';
Hp = hProt*hProt';
leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
leakCapUnit = max(leakCapUnit,0);
v = leakage_rzf_unit(hSU,hProt,alphaFallback);
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-9)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
mu = pRef / max(Gamma,1e-30);
for it = 1:max(FP_iter,1)
    hv = hSU' * v;
    y = sqrt(max(pRef,0)) * hv / max(pRef*abs(hv)^2 + denom,1e-30);
    A = abs(y)^2*pRef*Hs + mu*Hp + alphaFallback*eye(Nt);
    b = conj(y)*sqrt(max(pRef,0))*hSU;
    vNew = A \ b;
    if norm(vNew) < 1e-30
        vNew = mrt_unit(hSU);
    else
        vNew = vNew/norm(vNew);
    end
    if leakCapUnit > 0 && abs(hProt'*vNew)^2 > leakCapUnit*(1+1e-9)
        vNew = polish_leakage_beam(vNew,hSU,hProt,leakCapUnit,alphaFallback);
    end
    if norm(vNew-v) < 1e-6
        v = vNew;
        break;
    end
    v = vNew;
end
v = v/max(norm(v),1e-30);
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-6)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
gSig  = abs(hSU'*v)^2;
gLeak = abs(hProt'*v)^2;
end
