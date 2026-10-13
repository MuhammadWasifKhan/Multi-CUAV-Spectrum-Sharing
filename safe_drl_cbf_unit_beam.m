function [v,gSig,gLeak] = safe_drl_cbf_unit_beam(hSU,hProt,denom,Gamma,pRef,SafeIter,leakFrac,alphaFallback)
% Safe DRL-based CBF lightweight baseline.
% ------------------------------------------------------------
% It emulates a safe policy-improvement layer: generate candidate beam actions,% project unsafe candidates through the safety layer,evaluate reward with an
% interference-temperature cost,and select the safest/highest-reward action.
% This avoids heavy DNN training while preserving the safety idea in simulation.
Nt = numel(hSU);
leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
leakCapUnit = max(leakCapUnit,0);
cand = {};
cand{end+1} = mrt_unit(hSU);
cand{end+1} = leakage_rzf_unit(hSU,hProt,alphaFallback);
[c1,~,~] = mm_unit_beam(hSU,hProt,denom,Gamma,pRef,max(2,ceil(SafeIter/2)),leakFrac,alphaFallback); cand{end+1} = c1;
[c2,~,~] = wmmse_unit_beam(hSU,hProt,denom,Gamma,pRef,max(2,ceil(SafeIter/2)),leakFrac,alphaFallback); cand{end+1} = c2;
[c3,~,~] = fp_cbf_unit_beam(hSU,hProt,denom,Gamma,pRef,max(2,ceil(SafeIter/2)),leakFrac,alphaFallback); cand{end+1} = c3;
% Deterministic exploration actions around the useful-channel direction.
base = leakage_rzf_unit(hSU,hProt,alphaFallback);
Pnull = eye(Nt) - (hProt*hProt')/max(norm(hProt)^2,1e-30);
nullDir = Pnull*hSU;
if norm(nullDir) > 1e-12
    nullDir = nullDir/norm(nullDir);
else
    nullDir = base;
end
for rr = 1:max(SafeIter,1)
    beta = rr/(max(SafeIter,1)+1);
    vb = (1-beta)*base + beta*nullDir;
    if norm(vb) > 1e-30
        cand{end+1} = vb/norm(vb); %#ok<AGROW>
    end
end
bestReward = -inf;
v = cand{1};
for ii = 1:numel(cand)
    vc = cand{ii};
    vc = vc/max(norm(vc),1e-30);
    if leakCapUnit > 0 && abs(hProt'*vc)^2 > leakCapUnit*(1+1e-9)
        vc = polish_leakage_beam(vc,hSU,hProt,leakCapUnit,alphaFallback);
    end
    sig = abs(hSU'*vc)^2;
    leak = abs(hProt'*vc)^2;
    rateReward = log2(1 + max(pRef*sig/max(denom,1e-30),0));
    safetyCost = max(0,pRef*leak - Gamma)/max(Gamma,1e-30);
    reward = rateReward - 20*safetyCost^2;
    if reward > bestReward
        bestReward = reward;
        v = vc;
    end
end
v = v/max(norm(v),1e-30);
gSig  = abs(hSU'*v)^2;
gLeak = abs(hProt'*v)^2;
end
