function [v,gSig,gLeak] = wmmse_unit_beam(hSU,hProt,denom,Gamma,pRef,WMMSE_iter,leakFrac,alphaFallback)
% WMMSE-based leakage-aware unit-norm beam design.
% ------------------------------------------------------------
% For one selected stream,the scalar WMMSE updates are:
%   u = sqrt(pRef) h^H v / (pRef |h^H v|^2 + denom)
%   e = 1 - 2 Re{u^* sqrt(pRef) h^H v} + |u|^2(pRef |h^H v|^2 + denom)
%   w = 1/e
% Then the beam is updated from a regularized least-squares/WMMSE surrogate
% with leakage penalty to protect PU/WU receivers:
%   v <- (w|u|^2 pRef hh^H + mu Hp + alpha I)^(-1) (w u sqrt(pRef) h)
% followed by normalization and leakage polishing.
Nt = numel(hSU);
Hs = hSU*hSU';
Hp = hProt*hProt';
leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
leakCapUnit = max(leakCapUnit,0);
% Stable initial beam.
v = leakage_rzf_unit(hSU,hProt,alphaFallback);
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-9)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
mu = pRef / max(Gamma,1e-30);
for it = 1:max(WMMSE_iter,1)
    hv = hSU' * v;
    rxDen = max(pRef*abs(hv)^2 + denom,1e-30);
    u = sqrt(max(pRef,0)) * hv / rxDen;
    mse = 1 - 2*real(conj(u)*sqrt(max(pRef,0))*hv) + abs(u)^2 * rxDen;
    mse = max(real(mse),1e-12);
    w = 1/mse;
    A = w*abs(u)^2*pRef*Hs + mu*Hp + alphaFallback*eye(Nt);
    b = w*conj(u)*sqrt(max(pRef,0))*hSU;
    vNew = A \ b;
    if norm(vNew) < 1e-30
        vNew = mrt_unit(hSU);
    else
        vNew = vNew/norm(vNew);
    end
    if leakCapUnit > 0 && abs(hProt'*vNew)^2 > leakCapUnit*(1+1e-9)
        vNew = polish_leakage_beam(vNew,hSU,hProt,leakCapUnit,alphaFallback);
    end
    if norm(vNew - v) < 1e-6
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
