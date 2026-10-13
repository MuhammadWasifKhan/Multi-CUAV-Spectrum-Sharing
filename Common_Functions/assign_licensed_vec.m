function rho = assign_licensed_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
Rmat = zeros(K,S);
for k = 1:K
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
        Rmat(k,s) = Bsc*log2(1 + max(SINR,0));
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
