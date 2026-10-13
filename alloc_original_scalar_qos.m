function [idxLic,pLic,idxUn,pUn] = alloc_original_scalar_qos(...
        hu2L,hu2U,hp2,hw2,h_up2,h_uw2,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,...
        GammaLic,GammaUn,Pmax_lic,Pmax_un,QoS_target,...
        Bsc_lic,Bsc_un,AO_AP_iter,allocMode)
K = numel(hu2L);
M = numel(h_uw2);
idxLic = zeros(0,2); pLic = [];
idxUn  = zeros(0,2); pUn  = [];
for it = 1:AO_AP_iter %#ok<NASGU>
    rhoL = assign_licensed_original(hu2L,hp2,h_up2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,Bsc_lic,K,numel(h_up2),allocMode);
    [pLic,idxLic] = solve_band_kkt_original(rhoL,hu2L,hp2,P_PBS_j,sigma2_lic,GammaLic,h_up2,Pmax_lic);
    needUn = true(K,1); % All SUs eligible in both bands
    if any(needUn)
        rhoU = assign_unlicensed_for_users(hu2U,hw2,h_uw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,Bsc_un,K,M,needUn,allocMode);
        [pUn,idxUn] = solve_band_kkt_original(rhoU,hu2U,hw2,P_WIFI_m,sigma2_un,GammaUn,h_uw2,Pmax_un);
    else
        pUn = [];
        idxUn = zeros(0,2);
    end
end
end
