function tx = increase_from_aggregate_band_headroom(tx,Hprot,GammaVec,PmaxBand,bandName)
% Increase powers using ONLY the same band's unused power budget and aggregate
% interference headroom.
%
% This function does NOT receive clipped power from the other band. Therefore,% it is per-band control,not common-total power sharing and not cross-band
% transfer.
if PmaxBand <= 0
    return;
end
GammaVec = GammaVec(:);
S = numel(GammaVec);
% Ensure current point is safe before trying to increase.
tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName);
for iter = 1:20
    I = aggregate_interference_by_channel(tx,Hprot,GammaVec,bandName);
    changed = false;
    for s = 1:S
        interfHeadroom = GammaVec(s) - I(s);
        if interfHeadroom <= 1e-15
            continue;
        end
        % Collect active streams on channel s.
        entries = [];
        for u = 1:numel(tx)
            [idx,p,vList] = get_band_fields(tx,u,bandName);
            if isempty(idx),continue; end
            for ell = 1:size(idx,1)
                if idx(ell,2) == s
                    v = vList{ell};
                    gLeak = abs(Hprot{u,s}' * v)^2;
                    uBudget = max(PmaxBand - sum(p),0);
                    if gLeak > 1e-30 && uBudget > 1e-15
                        % columns: u,ell,gLeak,uBudget
                        entries = [entries; u,ell,gLeak,uBudget]; %#ok<AGROW>
                    end
                end
            end
        end
        if isempty(entries)
            continue;
        end
        % Distribute the aggregate interference headroom across active streams.
        % Equal interference-share is used to avoid biasing one UAV only.
        nE = size(entries,1);
        shareI = interfHeadroom / nE;
        for ee = 1:nE
            u     = entries(ee,1);
            ell   = entries(ee,2);
            gLeak = entries(ee,3);
            [~,p,~] = get_band_fields(tx,u,bandName);
            uBudget = max(PmaxBand - sum(p),0);
            if uBudget <= 1e-15
                continue;
            end
            dp = min(shareI / max(gLeak,1e-30),uBudget);
            if dp > 1e-15
                p(ell) = p(ell) + dp;
                tx = set_band_power(tx,u,bandName,p);
                changed = true;
            end
        end
        % Re-project immediately to remove any small aggregate overshoot.
        tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName);
    end
    if ~changed
        break;
    end
end
tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName);
end
