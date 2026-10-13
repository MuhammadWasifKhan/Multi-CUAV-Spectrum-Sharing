function rho = assign_unlicensed_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
% Unlicensed assignment with previous-AO inter-UAV interference in the metric.
rho = zeros(K,S);
activeUsers = find(needUn(:));
if isempty(activeUsers) || Pmax <= 0
    return;
end
if isempty(extraNoise)
    extraNoise = zeros(K,S);
end
Ka = numel(activeUsers);
Rmat = zeros(Ka,S);
for aa = 1:Ka
    k = activeUsers(aa);
    for s = 1:S
        denom = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            v = mrt_unit(Hsu{k}); % Initial allocation: no protection
        else
            v = mrt_unit(Hsu{k});
        end
        gSig  = abs(Hsu{k}'*v)^2;
        gLeak = abs(Hprot{s}'*v)^2;
        p_app = Pmax/max(S,1); % No pre-control interference cap
        Rmat(aa,s) = Bsc*log2(1 + max(p_app*gSig/denom,0));
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
