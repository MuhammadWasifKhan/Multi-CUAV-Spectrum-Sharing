function rho = assign_unlicensed_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
rho = zeros(K,S);
activeUsers = find(needUn(:));
if isempty(activeUsers) || Pmax <= 0
    return;
end
Ka = numel(activeUsers);
Rmat = zeros(Ka,S);
for aa = 1:Ka
    k = activeUsers(aa);
    for s = 1:S
        denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            % Do NOT call CVX inside the assignment matrix. Calling CVX for every
            % candidate (k,s) creates nested/non-empty CVX-scope warnings and is
            % extremely slow. Use a leakage-aware RZF metric for assignment;
            % true MM/WMMSE is applied later only to the selected pairs in
            % solve_band_mm_given_rho() or solve_band_wmmse_given_rho().
            v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
            gSig = abs(Hsu{k}'*v)^2;
            gLeak = abs(Hprot{s}'*v)^2;
        else
            v = mrt_unit(Hsu{k});
            gSig = abs(Hsu{k}'*v)^2;
            gLeak = abs(Hprot{s}'*v)^2;
        end
        p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
        SINR = p_app * gSig / denom;
        Rmat(aa,s) = Bsc*log2(1 + max(SINR,0));
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
