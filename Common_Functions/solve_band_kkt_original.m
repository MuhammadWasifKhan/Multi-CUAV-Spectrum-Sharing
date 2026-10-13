function [p_assigned,idxList] = solve_band_kkt_original(rho,hu2,hI,P_I_vec,sigma2,GammaVec,hProtect,Pmax)
[~,S] = size(rho);
idxList = zeros(0,2);
a_list  = [];
cap_list= [];
for s = 1:S
    k = find(rho(:,s)==1,1);
    if ~isempty(k)
        denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
        a = max(hu2(k) / denom,1e-30);
        pcap = max(GammaVec(s) / max(hProtect(s),1e-30),0);
        idxList = [idxList; k,s]; %#ok<AGROW>
        a_list  = [a_list; a]; %#ok<AGROW>
        cap_list= [cap_list; pcap]; %#ok<AGROW>
    end
end
p_assigned = kkt_waterfill_capped_original(a_list,Pmax,cap_list);
end
