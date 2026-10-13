function sumRate = compute_sum_rate_vec(idxLic,pLic,vLic,idxUn,pUn,vUn,...
                                    HsuL,HsuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
                                    sigma2_lic,sigma2_un,Bsc_lic,Bsc_un)
sumRate = 0;
for ell=1:size(idxLic,1)
    k=idxLic(ell,1); j=idxLic(ell,2);
    v = vLic{ell};
    denom = max(hp2(k)*P_PBS_j(j) + sigma2_lic,1e-30);
    gSig = abs(HsuL{k}'*v)^2;
    SINR = pLic(ell)*gSig/denom;
    sumRate = sumRate + Bsc_lic*log2(1 + max(SINR,0));
end
for ell=1:size(idxUn,1)
    k=idxUn(ell,1); m=idxUn(ell,2);
    v = vUn{ell};
    denom = max(hw2(k)*P_WIFI_m(m) + sigma2_un,1e-30);
    gSig = abs(HsuU{k}'*v)^2;
    SINR = pUn(ell)*gSig/denom;
    sumRate = sumRate + Bsc_un*log2(1 + max(SINR,0));
end
end
%% ============================================================
%                  SCALAR TRAJECTORY ALLOCATION HELPERS
%% ============================================================
