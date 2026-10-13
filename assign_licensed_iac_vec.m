function rho = assign_licensed_iac_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,alphaFallback,pRef)
% Interference-aware licensed channel selection (new proactive idea).
% The metric rewards useful SU gain/rate and penalizes:
%   1) received licensed interference at the SU from PBS,%   2) leakage caused by the UAV toward the PU on that licensed subchannel.
% Power is still allocated later by the same CBF-MM/KKT solver.
Rmat = zeros(K,S);
lambdaLeak = 1.00;
lambdaRx   = 0.10;
for k = 1:K
    for s = 1:S
        denomRx = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
        else
            v = mrt_unit(Hsu{k});
        end
        gSig  = abs(Hsu{k}'*v)^2;
        gLeak = abs(Hprot{s}'*v)^2;
        p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
        rateEst = Bsc*log2(1 + max(p_app*gSig/denomRx,0));
        leakRatio = (p_app*gLeak) / max(GammaVec(s),1e-30);
        rxRatio   = (hI(k)*P_I_vec(s)) / max(sigma2,1e-30);
        iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio);
        Rmat(k,s) = rateEst / iacPenalty;
    end
end
if strcmpi(allocMode,'greedy')
    pairs = assign_greedy_max(Rmat);
else
    pairs = assign_hungarian_max(Rmat);
end
rho = zeros(K,S);
for ii = 1:size(pairs,1)
    rho(pairs(ii,1),pairs(ii,2)) = 1;
end
end
