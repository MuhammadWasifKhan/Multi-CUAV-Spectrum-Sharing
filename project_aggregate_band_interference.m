function tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName)
% Project one band so aggregate interference per protected channel is safe.
%
% For each protected channel s:
%   I_s = sum_{all UAV streams on s} p_stream * |h_prot^H v_stream|^2
%
% If I_s > GammaVec(s),all active stream powers on that channel are scaled
% by GammaVec(s)/I_s. This preserves the relative power split among active
% UAVs while guaranteeing aggregate per-slot protection.
tol = 1e-12;
GammaVec = GammaVec(:);
S = numel(GammaVec);
tx = enforce_per_uav_band_budget(tx,PmaxBand,bandName);
I = aggregate_interference_by_channel(tx,Hprot,GammaVec,bandName);
for s = 1:S
    if I(s) > GammaVec(s)*(1+tol)
        scale = GammaVec(s) / max(I(s),1e-30);
        for u = 1:numel(tx)
            [idx,p,vList] = get_band_fields(tx,u,bandName); %#ok<ASGLU>
            if isempty(idx),continue; end
            onS = (idx(:,2) == s);
            if any(onS)
                p(onS) = p(onS) * scale;
                tx = set_band_power(tx,u,bandName,p);
            end
        end
    end
end
tx = enforce_per_uav_band_budget(tx,PmaxBand,bandName);
end
