function [idxLic,pLic,vLic,idxUn,pUn,vUn] = alloc_mrt_qos_vec(...
        HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
        sigma2_lic,sigma2_un,GammaLic,GammaUn,Pmax_lic,Pmax_un,...
        QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,allocMode)
K = numel(HsuL);
J = numel(HpuL);
M = numel(HwuU);
idxLic = zeros(0,2); pLic = []; vLic = {};
idxUn  = zeros(0,2); pUn  = []; vUn  = {};
for it = 1:AO_AP_iter %#ok<NASGU>
    rhoL = assign_licensed_vec(HsuL,HpuL,hp2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,Bsc_lic,K,J,allocMode,'mrt',[],[],[],[]);
    [pLic,idxLic,vLic] = solve_band_mrt_given_rho(rhoL,HsuL,HpuL,hp2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic);
    needUn = true(K,1); % All SUs eligible in both bands
    if any(needUn)
        rhoU = assign_unlicensed_vec(HsuU,HwuU,hw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,Bsc_un,K,M,needUn,allocMode,'mrt',[],[],[],[]);
        [pUn,idxUn,vUn] = solve_band_mrt_given_rho(rhoU,HsuU,HwuU,hw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un);
    else
        idxUn = zeros(0,2); pUn = []; vUn = {};
    end
end
end
