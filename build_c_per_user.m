function c_k = build_c_per_user(idx,p_assigned,hI,P_I_vec,sigma2,K)
c_k = zeros(K,1);
for ell = 1:size(idx,1)
    k = idx(ell,1);
    s = idx(ell,2);
    denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
    c_k(k) = c_k(k) + p_assigned(ell) / denom;
end
end
