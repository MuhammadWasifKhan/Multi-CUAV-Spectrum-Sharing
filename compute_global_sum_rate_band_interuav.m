function sumRate = compute_global_sum_rate_band_interuav(tx,HsuFull,hI,P_I_vec,sigma2,Bsc,bandName)
% Sum-rate for one band including inter-UAV co-channel interference.
sumRate = 0;
U = numel(tx);
for u = 1:U
    if strcmpi(bandName,'lic')
        idx = tx(u).idxLic; p = tx(u).pLic; vList = tx(u).vLic;
    else
        idx = tx(u).idxUn;  p = tx(u).pUn;  vList = tx(u).vUn;
    end
    users_u = tx(u).users;
    for ell = 1:size(idx,1)
        local_k = idx(ell,1);
        sc      = idx(ell,2);
        kglob   = users_u(local_k);
        v       = vList{ell};
        signal = p(ell) * abs(HsuFull{u,kglob}' * v)^2;
        denom  = hI(kglob)*P_I_vec(sc) + sigma2;
        denom  = denom + inter_uav_power_same_sc(tx,HsuFull,u,kglob,sc,bandName);
        denom  = max(denom,1e-30);
        SINR = signal/denom;
        sumRate = sumRate + Bsc*log2(1 + max(SINR,0));
    end
end
end
