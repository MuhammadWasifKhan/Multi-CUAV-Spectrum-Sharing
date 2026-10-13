function I = aggregate_interference_by_channel(tx,Hprot,GammaVec,bandName)
% Aggregate leakage from all UAVs to each protected receiver/channel.
S = numel(GammaVec);
I = zeros(S,1);
for u = 1:numel(tx)
    [idx,p,vList] = get_band_fields(tx,u,bandName);
    if isempty(idx),continue; end
    for ell = 1:size(idx,1)
        sc = idx(ell,2);
        if sc < 1 || sc > S
            continue;
        end
        v = vList{ell};
        gLeak = abs(Hprot{u,sc}' * v)^2;
        I(sc) = I(sc) + p(ell) * gLeak;
    end
end
end
