function [v,gSig,gLeak] = mm_unit_beam(hSU,hProt,denom,Gamma,pRef,MM_iter,leakFrac,alphaFallback)
global AIIBPC_BEAM_CALLS;
if ~isempty(AIIBPC_BEAM_CALLS), AIIBPC_BEAM_CALLS=AIIBPC_BEAM_CALLS+1; end
% MM-based leakage-aware unit-norm beam design.
% ------------------------------------------------------------
% This replaces the previous FP/SDR CVX-based beamformer.
% It solves a surrogate sequence for the useful-signal/leakage tradeoff:
%
%   maximize_v   v^H (Hs/denom - mu*Hp) v
%   subject to   ||v||_2 = 1,%
% where Hs = hSU*hSU^H and Hp = hProt*hProt^H. The leakage constraint is
% finally polished by a leakage-aware projection/RZF step. Actual PU/WU
% interference protection is still enforced later by the proposed 4-case
% per-band power clipping function.
%
% Important: this is an MM-style iterative beam update,not FP/SDR and not CVX.
Nt = numel(hSU);
Hs = hSU*hSU';
Hp = hProt*hProt';
leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
leakCapUnit = max(leakCapUnit,0);
% Initial feasible/stable beam: leakage-aware RZF.
v = leakage_rzf_unit(hSU,hProt,alphaFallback);
% If the RZF beam still violates the unit leakage cap,try null-space beam.
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-9)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
% Penalty weight for leakage. Larger pRef/Gamma means stronger protection.
mu = pRef / max(Gamma,1e-30);
% Hermitian quadratic matrix for MM surrogate/power-iteration update.
C = Hs/max(denom,1e-30) - mu*Hp;
C = (C + C')/2;
% Shifted positive-semidefinite surrogate matrix.
lamMin = min(real(eig(C)));
tau = max(0,-lamMin) + alphaFallback;
B = C + tau*eye(Nt);
B = (B + B')/2;
for it = 1:max(MM_iter,1)
    vNew = B*v;
    if norm(vNew) < 1e-30
        vNew = mrt_unit(hSU);
    else
        vNew = vNew/norm(vNew);
    end
    % Leakage polishing after each MM step.
    if leakCapUnit > 0 && abs(hProt'*vNew)^2 > leakCapUnit*(1+1e-9)
        vNew = polish_leakage_beam(vNew,hSU,hProt,leakCapUnit,alphaFallback);
    end
    if norm(vNew - v) < 1e-6
        v = vNew;
        break;
    end
    v = vNew;
end
% Final normalization and leakage polishing.
v = v / max(norm(v),1e-30);
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-6)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
gSig  = abs(hSU'*v)^2;
gLeak = abs(hProt'*v)^2;
end
