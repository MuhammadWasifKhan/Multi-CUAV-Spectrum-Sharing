function [tx,stats]=ai_ibpc_integrated_four_case_rateaware(raw,HP,HW,GL,GU,PL,PU,rateFn)
% Integrated four-case AI-IBPC controller.
% ------------------------------------------------------------
% INPUT raw is the solution AFTER association, subchannel allocation,
% CBF-MM beamforming and interference-unaware KKT power allocation.
%
% The four cases are determined from RAW aggregate interference:
%   Case 1: licensed high, unlicensed safe.
%   Case 2: unlicensed high, licensed safe.
%   Case 3: neither high.
%   Case 4: both high.
%
% Every accepted action must satisfy the corresponding PU/WU protection
% threshold and the independent per-UAV power budget. The objective used
% to select among tested feasible power actions is the TRUE network sum rate
% supplied by rateFn. This is a rate-aware coordinate/candidate search; it is
% not a proof of the global non-convex optimum.
%
% IMPORTANT: protection is integrated here. The caller must NOT apply a
% separate safety-clipping/protection block after this function.

raw=enforce_per_uav_band_budget(raw,PL,'lic');
raw=enforce_per_uav_band_budget(raw,PU,'un');
IL0=aggregate_interference_by_channel(raw,HP,GL,'lic');
IU0=aggregate_interference_by_channel(raw,HW,GU,'un');
highL=any(IL0(:)>GL(:).*(1+1e-9));
highU=any(IU0(:)>GU(:).*(1+1e-9));

stats=struct('caseId',0,'caseName','','rawRate',rateFn(raw),...
    'finalRate',NaN,'licensedHigh',highL,'unlicensedHigh',highU,...
    'licensedReduced_W',0,'unlicensedReduced_W',0,...
    'licensedIncreased_W',0,'unlicensedIncreased_W',0);

pL0=arrayfun(@(z)sum(z.pLic(:)),raw);
pU0=arrayfun(@(z)sum(z.pUn(:)),raw);

if highL && ~highU
    % CASE 1: protect licensed side by rate-aware reduction. Then use only
    % unused UNLICENSED-band budget/headroom when that increases sum rate.
    stats.caseId=1; stats.caseName='licensed-high-only';
    tx=rate_aware_band_projection_ratefn(raw,HP,GL,PL,'lic',rateFn);
    tx=rate_aware_increase_band_only(tx,HW,GU,PU,'un',rateFn);

elseif ~highL && highU
    % CASE 2: mirror of Case 1.
    stats.caseId=2; stats.caseName='unlicensed-high-only';
    tx=rate_aware_band_projection_ratefn(raw,HW,GU,PU,'un',rateFn);
    tx=rate_aware_increase_band_only(tx,HP,GL,PL,'lic',rateFn);

elseif ~highL && ~highU
    % CASE 3: already protected. Keep the KKT power solution unchanged.
    stats.caseId=3; stats.caseName='neither-high';
    tx=raw;

else
    % CASE 4: both bands violate protection. Reduce BOTH bands using the
    % same global sum-rate objective. Alternate the order and retain the
    % better feasible protected result so the result is not tied to one
    % arbitrary licensed-first/unlicensed-first ordering.
    stats.caseId=4; stats.caseName='both-high';

    candLU=rate_aware_band_projection_ratefn(raw,HP,GL,PL,'lic',rateFn);
    candLU=rate_aware_band_projection_ratefn(candLU,HW,GU,PU,'un',rateFn);

    candUL=rate_aware_band_projection_ratefn(raw,HW,GU,PU,'un',rateFn);
    candUL=rate_aware_band_projection_ratefn(candUL,HP,GL,PL,'lic',rateFn);

    if ai_ibpc_is_feasible(candLU,HP,HW,GL,GU,PL,PU) && ...
            (~ai_ibpc_is_feasible(candUL,HP,HW,GL,GU,PL,PU) || ...
             rateFn(candLU)>=rateFn(candUL))
        tx=candLU;
    else
        tx=candUL;
    end
end

% This is verification, NOT another protection operation.
assert(ai_ibpc_is_feasible(tx,HP,HW,GL,GU,PL,PU),...
    'Integrated AI-IBPC failed to produce a protected feasible solution.');

pLf=arrayfun(@(z)sum(z.pLic(:)),tx);
pUf=arrayfun(@(z)sum(z.pUn(:)),tx);
stats.licensedReduced_W=sum(max(0,pL0-pLf));
stats.unlicensedReduced_W=sum(max(0,pU0-pUf));
stats.licensedIncreased_W=sum(max(0,pLf-pL0));
stats.unlicensedIncreased_W=sum(max(0,pUf-pU0));
stats.finalRate=rateFn(tx);
end

