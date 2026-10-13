function tx = enforce_per_uav_band_budget(tx,PmaxBand,bandName)
% Enforce each UAV's own per-band power budget.
for u = 1:numel(tx)
    [idx,p,~] = get_band_fields(tx,u,bandName); %#ok<ASGLU>
    if isempty(p),continue; end
    p = max(p(:),0);
    if sum(p) > PmaxBand
        p = p * (PmaxBand / max(sum(p),1e-30));
    end
    tx = set_band_power(tx,u,bandName,p);
end
end
