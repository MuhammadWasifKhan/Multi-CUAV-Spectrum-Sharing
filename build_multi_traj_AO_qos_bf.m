function Qout = build_multi_traj_AO_qos_bf(Qinit,q_start_all,q_end_all,WS,WP,WW,w_b,w_wifi,H,...
        f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,...
        AO_traj_iter,stepMax,alpha_traj,fd_eps,area,Nsamp,Dmin_UAV,seedBase,...
        useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,...
        methodMode,displayName)
% OUTER AO WITH TRAJECTORY AS THE FINAL BLOCK.
% ------------------------------------------------------------
% At the current trajectory Q:
%   (1) association
%   (2) subchannel allocation on both bands
%   (3) CBF-MM beamforming
%   (4) KKT/water-filling power allocation
%   (5) integrated four-case AI-IBPC
% are solved by eval_slot_multi_qos_bf().
%
% Only AFTER those communication variables are obtained do we perform:
%   (6) one JOINT all-UAV/all-waypoint analytical first-order Taylor SCA
%       trajectory update.
%
% During the SCA step the accepted communication variables are frozen while
% differentiating the original Al-Hourani large-scale gain analytically.
% The resulting model is local, so a trust region/backtracking is used.
% A trajectory candidate is accepted ONLY after the complete communication
% problem (including AI-IBPC) is re-solved at Qtrial and its true sum rate is
% no smaller than the incumbent.  Hence the sequence is:
%
%   resources + AI-IBPC  ->  trajectory SCA  ->  re-solve resources + AI-IBPC
%
% and then repeat until AO_traj_iter or convergence.
% fd_eps is retained only for interface compatibility; no finite differences.
if exist('cvx_begin','file') ~= 2
    error('CVX is required for joint trajectory optimization.');
end
U=size(Qinit,1); N=size(Qinit,2);
if check_multi_traj_feasibility(Qinit,q_start_all,q_end_all,stepMax,area,Dmin_UAV)
    Qout=Qinit;
else
    Qout=repair_multi_trajectory_cvx(Qinit,area,stepMax,q_start_all,q_end_all,Dmin_UAV);
end
fixedEvalSeed=seedBase+700000;
prevRate=evaluate_multi_trajectory_qos_bf(Qout,WS,WP,WW,w_b,w_wifi,H,...
    f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
    P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
    Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,fixedEvalSeed,...
    useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
maxBacktracks=3; minRadius=1e-3;
for ao=1:AO_traj_iter
    Qbase=Qout;
    gradRate=zeros(U,N,2);
    baseLeak=zeros(numel(GammaLic)+numel(GammaUn),N);
    gradLeak=zeros(numel(GammaLic)+numel(GammaUn),U,N,2);
    % All slot gradients are assembled BEFORE the single global CVX solve.
    for n=2:N-1
        Qslot=squeeze(Qbase(:,n,:));
        seed=fixedEvalSeed+n; % same realized fading as the acceptance objective
        % Communication solution FIRST at fixed Qslot.  tx returned here is
        % already the protected, rate-aware AI-IBPC solution.
        [~,~,~,tx] = eval_slot_multi_qos_bf(Qslot,WS,WP,WW,w_b,w_wifi,H,...
            f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
            P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
            Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,...
            seed,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
        [gr,leak,gl]=joint_analytic_slot_derivatives(Qslot,tx,WS,WP,WW,w_b,w_wifi,H,...
            f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
            P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
            seed,useRayleigh,useShadow,useShadowA2G,Nt,Bsc_lic,Bsc_un);
        gradRate(:,n,:)=reshape(gr,[U,1,2]);
        baseLeak(:,n)=leak;
        gradLeak(:,:,n,:)=reshape(gl,[numel(leak),U,1,2]);
    end
    if ~all(isfinite(gradRate(:))) || ~all(isfinite(gradLeak(:)))
        warning('Nonfinite analytical derivatives: retaining previous trajectory.');
        break;
    end
    radius=min(stepMax,max(minRadius,alpha_traj*stepMax));
    accepted=false;
    for bt=1:maxBacktracks
        [Qtrial,solved]=joint_traj_cvx_analytic(Qbase,gradRate,baseLeak,gradLeak,...
            q_start_all,q_end_all,stepMax,area,radius,Dmin_UAV);
        if solved && check_multi_traj_feasibility(Qtrial,q_start_all,q_end_all,stepMax,area,Dmin_UAV)
            % Re-solve ALL communication blocks, including AI-IBPC, at the
            % candidate trajectory before accepting the SCA waypoint update.
            trialRate=evaluate_multi_trajectory_qos_bf(Qtrial,WS,WP,WW,w_b,w_wifi,H,...
                f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
                P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
                Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,fixedEvalSeed,...
                useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
            if isfinite(trialRate) && trialRate>=prevRate-1e-9*max(1,abs(prevRate))
                Qout=Qtrial;
                accepted=true;
                if abs(trialRate-prevRate)<=1e-5*max(1,abs(prevRate))
                    return;
                end
                prevRate=trialRate;
                break;
            end
        end
        radius=radius/2;
        if radius<minRadius, break; end
    end
    if ~accepted
        break;
    end
end
end
