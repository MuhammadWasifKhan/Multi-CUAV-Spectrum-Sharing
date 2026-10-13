function [Qout,rateHist,stopReason]=build_multi_traj_AO_reserved(Qinit,q_start_all,q_end_all,WS,WP,WW,w_b,w_wifi,H,...
    f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,...
    PL,PU,QoS,BL,BU,AO_AP_iter,AO_traj_iter,stepMax,alpha_traj,area,Dmin,seedBase,...
    useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU)
% Proposed-II trajectory AO. Same analytical SCA/trust-region machinery as
% Proposed I, but every incumbent/trial trajectory is evaluated after the
% Reserved-subchannel controller, not AI-IBPC.
if check_multi_traj_feasibility(Qinit,q_start_all,q_end_all,stepMax,area,Dmin)
    Qout=Qinit;
else
    Qout=repair_multi_trajectory_cvx(Qinit,area,stepMax,q_start_all,q_end_all,Dmin);
end
fixedSeed=seedBase+700000;
prevRate=evaluate_reserved_trajectory(Qout,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,...
    n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,fixedSeed,...
    useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU);
rateHist=nan(AO_traj_iter+1,1); rateHist(1)=prevRate; histIdx=1;
stopReason='maximum iterations';
U=size(Qout,1); N=size(Qout,2); maxBacktracks=3; minRadius=1e-3;
for ao=1:AO_traj_iter
    Qbase=Qout; gradRate=zeros(U,N,2); baseLeak=zeros(numel(GL)+numel(GU),N);
    gradLeak=zeros(numel(GL)+numel(GU),U,N,2);
    for n=2:N-1
        Qslot=squeeze(Qbase(:,n,:)); seed=fixedSeed+n;
        [tx,~]=eval_slot_reserved_final(Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,...
            n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,seed,...
            useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU);
        [gr,leak,gl]=joint_analytic_slot_derivatives(Qslot,tx,WS,WP,WW,w_b,w_wifi,H,...
            f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,...
            seed,useRayleigh,useShadow,useShadowA2G,Nt,BL,BU);
        gradRate(:,n,:)=reshape(gr,[U,1,2]); baseLeak(:,n)=leak;
        gradLeak(:,:,n,:)=reshape(gl,[numel(leak),U,1,2]);
    end
    if ~all(isfinite(gradRate(:))) || ~all(isfinite(gradLeak(:)))
        stopReason='nonfinite derivatives';
        break;
    end
    radius=min(stepMax,max(minRadius,alpha_traj*stepMax)); accepted=false;
    for bt=1:maxBacktracks
        [Qtrial,solved]=joint_traj_cvx_analytic(Qbase,gradRate,baseLeak,gradLeak,...
            q_start_all,q_end_all,stepMax,area,radius,Dmin);
        if solved && check_multi_traj_feasibility(Qtrial,q_start_all,q_end_all,stepMax,area,Dmin)
            trialRate=evaluate_reserved_trajectory(Qtrial,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,...
                n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,fixedSeed,...
                useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU);
            if isfinite(trialRate) && trialRate>=prevRate-1e-9*max(1,abs(prevRate))
                Qout=Qtrial; accepted=true;
                if abs(trialRate-prevRate)<=1e-5*max(1,abs(prevRate))
                    prevRate=trialRate; histIdx=histIdx+1; rateHist(histIdx)=prevRate;
                    rateHist=rateHist(1:histIdx);
                    stopReason='relative improvement tolerance';
                    return;
                end
                prevRate=trialRate; histIdx=histIdx+1; rateHist(histIdx)=prevRate; break;
            end
        end
        radius=radius/2; if radius<minRadius,break;end
    end
    if ~accepted
        stopReason='no acceptable trajectory update';
        break;
    end
end
rateHist=rateHist(1:histIdx);
end

