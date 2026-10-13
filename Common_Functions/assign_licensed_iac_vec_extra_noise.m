function rho = assign_licensed_iac_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,alphaFallback,pRef)
% IAC licensed assignment with previous-AO inter-UAV interference.
Rmat = zeros(K,S);
if isempty(extraNoise)
    extraNoise = zeros(K,S);
end
lambdaLeak = 1.00;
lambdaRx   = 0.10;
lambdaIuav = 0.25;
for k = 1:K
    for s = 1:S
        denomRx = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
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
        iUavRatio = extraNoise(k,s) / max(sigma2,1e-30);
        iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio) + lambdaIuav*log1p(iUavRatio);
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
