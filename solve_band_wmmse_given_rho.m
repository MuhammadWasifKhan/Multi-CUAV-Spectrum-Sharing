function [p_assigned,idxList,vList] = solve_band_wmmse_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,WMMSE_iter,leakFrac,alphaFallback)
% WMMSE-based leakage-aware beamforming for selected user-subchannel pairs.
% This is CVX-free. It performs scalar receive-filter/weight updates and
% transmitter beam updates with a leakage regularization term.
[~,S] = size(rho);
idxList = zeros(0,2);
a_list = [];
cap_list = [];
vList = {};
active = nnz(rho);
pRef = Pmax/max(active,1);
for s = 1:S
    k = find(rho(:,s)==1,1);
    if ~isempty(k)
        denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
        [v,gSig,gLeak] = wmmse_unit_beam(Hsu{k},Hprot{s},denom,GammaVec(s),pRef,WMMSE_iter,leakFrac,alphaFallback);
        a = max(gSig/denom,1e-30);
        pcap = max(GammaVec(s)/max(gLeak,1e-30),0); %#ok<NASGU>
        idxList = [idxList; k,s]; %#ok<AGROW>
        a_list = [a_list; a]; %#ok<AGROW>
        cap_list = [cap_list; pcap]; %#ok<AGROW>
        vList{end+1,1} = v; %#ok<AGROW>
    end
end
p_assigned = kkt_waterfill_capped_original(a_list,Pmax,cap_list);
% Per-band 4-case interference-aware clipping is applied later in
% apply_perband_4case_power_control().
end
