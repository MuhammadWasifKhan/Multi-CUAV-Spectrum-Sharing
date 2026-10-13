function [p_assigned,idxList,vList] = solve_band_safe_drl_cbf_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,SafeIter,leakFrac,alphaFallback)
% Safe-DRL-style CBF for selected pairs.
% This is a lightweight safety-layer policy selector over candidate beam actions.
% It is not a full offline-trained neural DRL implementation.
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
        [v,gSig,gLeak] = safe_drl_cbf_unit_beam(Hsu{k},Hprot{s},denom,GammaVec(s),pRef,SafeIter,leakFrac,alphaFallback);
        a = max(gSig/denom,1e-30);
        pcap = max(GammaVec(s)/max(gLeak,1e-30),0); %#ok<NASGU>
        idxList = [idxList; k,s]; %#ok<AGROW>
        a_list = [a_list; a]; %#ok<AGROW>
        cap_list = [cap_list; pcap]; %#ok<AGROW>
        vList{end+1,1} = v; %#ok<AGROW>
    end
end
p_assigned = kkt_waterfill_capped_original(a_list,Pmax,cap_list);
end
