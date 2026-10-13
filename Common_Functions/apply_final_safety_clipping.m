function tx = apply_final_safety_clipping(tx,HpuL,HwuU,GammaLic,GammaUn,PmaxLic,PmaxUn,whichBands)
% Lightweight final protection for the IAC curve.
% ------------------------------------------------------------
% Corrected for multi-UAV aggregate per-slot protection.
% This is not a 4-case controller. It only projects the active powers so that
% the aggregate interference from ALL UAVs to each protected PU/WU channel is
% below the corresponding threshold:
%
%   sum_u I_lic(u,j,n) <= GammaLic(j),for every licensed channel j
%   sum_u I_un (u,m,n) <= GammaUn (m),for every unlicensed channel m
%
% It also preserves each UAV's own per-band transmit-power budget.
if strcmpi(whichBands,'lic') || strcmpi(whichBands,'both')
    tx = project_aggregate_band_interference(tx,HpuL,GammaLic,PmaxLic,'lic');
end
if strcmpi(whichBands,'un') || strcmpi(whichBands,'both')
    tx = project_aggregate_band_interference(tx,HwuU,GammaUn,PmaxUn,'un');
end
end
