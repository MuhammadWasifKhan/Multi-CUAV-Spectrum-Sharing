function rho = assign_licensed_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
% Licensed assignment with previous-AO inter-UAV interference in the metric.
Rmat = zeros(K,S);
if isempty(extraNoise)
    extraNoise = zeros(K,S);
end
for k = 1:K
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
        Rmat(k,s) = Bsc*log2(1 + max(p_app*gSig/denom,0));
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
