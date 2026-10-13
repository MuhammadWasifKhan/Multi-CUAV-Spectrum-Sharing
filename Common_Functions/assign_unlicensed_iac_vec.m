function rho = assign_unlicensed_iac_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,alphaFallback,pRef)
% Interference-aware unlicensed channel selection (new proactive idea).
% Active users are all associated users only. The metric rewards useful SU
% rate and penalizes WiFi interference plus leakage toward WUs.
rho = zeros(K,S);
activeUsers = find(needUn(:));
if isempty(activeUsers) || Pmax <= 0
    return;
end
Ka = numel(activeUsers);
Rmat = zeros(Ka,S);
lambdaLeak = 1.00;
lambdaRx   = 0.10;
for aa = 1:Ka
    k = activeUsers(aa);
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
        Rmat(aa,s) = rateEst / iacPenalty;
    end
end
if strcmpi(allocMode,'greedy')
    pairsLocal = assign_greedy_max(Rmat);
else
    pairsLocal = assign_hungarian_max(Rmat);
end
for ii = 1:size(pairsLocal,1)
    aa = pairsLocal(ii,1);
    s  = pairsLocal(ii,2);
    rho(activeUsers(aa),s) = 1;
end
end
