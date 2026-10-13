function assoc = associate_users_highest_channel_gain(HsuL)
% Highest licensed-band channel-gain association.
% For each SU k, select the C-UAV u maximizing ||h_{u,k}^{lic}||^2.
% Only the association rule is changed; all downstream Hungarian, CBF-MM,
% KKT, AI-IBPC/IA-RSS control, and trajectory-SCA logic remain unchanged.
[U,K] = size(HsuL);
metric = -inf(U,K);
for u = 1:U
    for k = 1:K
        hL = HsuL{u,k};
        metric(u,k) = real(hL' * hL);
    end
end
[~,assoc] = max(metric,[],1);
assoc = assoc(:);
end
