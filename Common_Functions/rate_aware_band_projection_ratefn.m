function tx=rate_aware_band_projection_ratefn(tx,HP,Gamma,Pmax,band,rateFn)
% Protect one band using rate-aware power REDUCTIONS only.
% For each unsafe protected subchannel, test proportional reduction and
% deterministic single-stream reduction orders. Keep the feasible candidate
% with the largest TRUE network sum rate.
tx=enforce_per_uav_band_budget(tx,Pmax,band);
for sc=1:numel(Gamma)
    I=aggregate_interference_by_channel(tx,HP,Gamma,band);
    if I(sc)<=Gamma(sc)*(1+1e-10),continue;end

    entries=zeros(0,4); % [u,ell,p,leakGain]
    for u=1:numel(tx)
        [idx,p,v]=get_band_fields(tx,u,band);
        for ell=1:size(idx,1)
            if idx(ell,2)==sc && p(ell)>0
                entries(end+1,:)=[u,ell,p(ell),...
                    abs(HP{u,sc}'*v{ell})^2]; %#ok<AGROW>
            end
        end
    end
    if isempty(entries)
        error('Unsafe protected channel has no active stream to reduce.');
    end

    % Candidate A: proportional reduction of all streams on this SC.
    scale=min(1,Gamma(sc)/max(I(sc),realmin));
    best=tx;
    for z=1:size(entries,1)
        u=entries(z,1); ell=entries(z,2);
        [~,p,~]=get_band_fields(best,u,band);
        p(ell)=p(ell)*scale;
        best=set_band_power(best,u,band,p);
    end
    bestRate=rateFn(best);

    % Candidate B: different deterministic stream-reduction orders.
    for first=1:size(entries,1)
        cand=tx;
        order=[first,setdiff(1:size(entries,1),first,'stable')];
        for oi=order
            Icur=aggregate_interference_by_channel(cand,HP,Gamma,band);
            if Icur(sc)<=Gamma(sc)*(1+1e-10),break;end
            u=entries(oi,1); ell=entries(oi,2);
            [~,p,~]=get_band_fields(cand,u,band);
            gain=entries(oi,4);
            dp=min(p(ell),(Icur(sc)-Gamma(sc))/max(gain,realmin));
            p(ell)=max(0,p(ell)-dp);
            cand=set_band_power(cand,u,band,p);
        end
        Ic=aggregate_interference_by_channel(cand,HP,Gamma,band);
        if Ic(sc)>Gamma(sc)*(1+1e-7),continue;end
        r=rateFn(cand);
        if r>bestRate+1e-9
            best=cand;
            bestRate=r;
        end
    end
    tx=best;
end

% Verification only. The search above must itself have protected the band.
I=aggregate_interference_by_channel(tx,HP,Gamma,band);
assert(all(I(:)<=Gamma(:).*(1+1e-7)),...
    'AI-IBPC rate-aware reduction did not satisfy aggregate protection.');
end

