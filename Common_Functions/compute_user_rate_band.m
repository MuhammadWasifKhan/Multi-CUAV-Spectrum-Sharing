function Ruser = compute_user_rate_band(idx,p,hu2,hI,P_I_vec,sigma2,Bsc,K)
Ruser = zeros(K,1);
for ell = 1:size(idx,1)
    k = idx(ell,1);
    s = idx(ell,2);
    denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
    SINR = hu2(k)*p(ell)/denom;
    Ruser(k) = Ruser(k) + Bsc*log2(1 + max(SINR,0));
end
end
