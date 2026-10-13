function B_k = build_B_per_user(idx,Bsc,K)
B_k = zeros(K,1);
for ell = 1:size(idx,1)
    k = idx(ell,1);
    B_k(k) = B_k(k) + Bsc;
end
end
%% ============================================================
%                  ASSIGNMENT HELPERS
%% ============================================================
