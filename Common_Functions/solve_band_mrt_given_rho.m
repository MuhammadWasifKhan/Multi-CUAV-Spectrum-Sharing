function [p_assigned,idxList,vList] = solve_band_mrt_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax)
[~,S] = size(rho);
idxList = zeros(0,2);
a_list = [];
cap_list = [];
vList = {};
for s = 1:S
    k = find(rho(:,s)==1,1);
    if ~isempty(k)
        v = mrt_unit(Hsu{k});
        gSig = abs(Hsu{k}'*v)^2;
        gLeak = abs(Hprot{s}'*v)^2;
        denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
        a = max(gSig/denom,1e-30);
        pcap = max(GammaVec(s)/max(gLeak,1e-30),0);
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
