clc; clear; close all;
% Reproducible function-path setup. Keeps existing CVX installation on path.
projectRoot=fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot,'-begin');
configure_simulation_paths(projectRoot,'convergence');
% CONVERGENCE: Proposed I AI-IBPC versus Proposed II IA-RSS.
% Records the TRUE accepted outer AO/SCA objective after each trajectory iteration.
% No AI-IBPC or IA-RSS logic is changed.

area=1000; H=100; U=3; K=6; Ninitial=6; J=10; M=10; Nt=8;
Bsc_lic=180e3; Bsc_un=180e3; N0_dBmHz=-169;
sigma2_lic=10.^((N0_dBmHz+10*log10(Bsc_lic)-30)/10);
sigma2_un=10.^((N0_dBmHz+10*log10(Bsc_un)-30)/10);
Pmax_lic=0.5; Pmax_un_fixed=0.5; P_PBS=2.5; P_WIFI=1;
P_PBS_j=P_PBS*ones(J,1); P_WIFI_m=P_WIFI*ones(M,1);
c=3e8; f_lic=2e9; f_un=5e9; f_pbs=f_lic; f_wifi=f_un;
n_pbs=3.7; n_wifi=3; sigmaSh_pbs_dB=8; sigmaSh_wifi_dB=6;
A2G=struct('a',9.61,'b',0.16,'etaLoS_dB',1,'etaNLoS_dB',20,...
    'sigmaLoS_dB',4,'sigmaNLoS_dB',6);
useRayleigh=true; useShadow=true; useShadowA2G=true;
MM_iter=10; MM_leakFrac=1; MM_reg_alpha=1e-3; QoS_target=0;
threshold_dBm=-80;
GammaLic=10.^((threshold_dBm-30)/10)*ones(J,1);
GammaUn=10.^((threshold_dBm-30)/10)*ones(M,1);
w_b=[area/2 area/2]; w_wifi=[area/2 area/2+200];
q_start_all=[120 850;260 180;430 620]; q_end_all=q_start_all;
dt_fixed=5; Tflight=220; N=round(Tflight/dt_fixed)+1; Vmax_UAV=5;
STEP_MAX_FIXED=Vmax_UAV*dt_fixed; Dmin_UAV=100;
AO_AP_iter=3; AO_traj_iter=25; alpha_traj=0.30; fd_eps=1;
Nsamp_traj=5; trajSeedBase=77777; dropSeedBase=9000;
NDrops=30; MC=300; % Independent topology/shadowing drops; one fixed fading draw per slot.
reserveLic=(Ninitial+1):J; reserveUn=(Ninitial+1):M;

% Each curve reports the mean over independent network drops. The fading
% sequence is fixed within each drop for a fair deterministic AO comparison.
% MC=1 is intentional: 300 independent fading trials are NOT performed here.
iter_list=1:AO_traj_iter;
AI_all=nan(NDrops,AO_traj_iter);
RES_all=nan(NDrops,AO_traj_iter);
AI_initial=nan(NDrops,1); RES_initial=nan(NDrops,1);
AI_accepted=zeros(NDrops,1); RES_accepted=zeros(NDrops,1);
AI_reason=cell(NDrops,1); RES_reason=cell(NDrops,1);
AI_final=nan(NDrops,1); RES_final=nan(NDrops,1);

for drop=1:NDrops
    rng(dropSeedBase+drop,'twister');
    WS=area*rand(K,2); WP=area*rand(J,2); WW=area*rand(M,2);
    Shadow=struct();
    Shadow.pbs_su_dB=sigmaSh_pbs_dB*randn(K,1);
    Shadow.wifi_su_dB=sigmaSh_wifi_dB*randn(K,1);
    Shadow.a2g_su_z=randn(K,1);
    Shadow.a2g_pu_z=randn(J,1);
    Shadow.a2g_wu_z=randn(M,1);
    Sinit=Shadow;
    Sinit.a2g_pu_z=Shadow.a2g_pu_z(1:Ninitial);
    Sinit.a2g_wu_z=Shadow.a2g_wu_z(1:Ninitial);
    q_hover_all=make_multi_hover_points_closed(q_start_all,WS,area);
    Qinit=build_multi_fly_hover_trajectory(q_start_all,q_hover_all,q_end_all,N,STEP_MAX_FIXED,area);
    if ~check_multi_traj_feasibility(Qinit,q_start_all,q_end_all,STEP_MAX_FIXED,area,Dmin_UAV)
        Qinit=repair_multi_trajectory_cvx(Qinit,area,STEP_MAX_FIXED,q_start_all,q_end_all,Dmin_UAV);
    end

    % Different fixed fading streams for the two schemes, as in the source
    % implementation. Their initial objectives are recorded separately.
    [Q_AI,hAI,statusAI]=build_multi_traj_AO_qos_bf(Qinit,q_start_all,q_end_all,WS,...
        WP(1:Ninitial,:),WW(1:Ninitial,:),w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Sinit,...
        P_PBS_j(1:Ninitial),P_WIFI_m(1:Ninitial),sigma2_lic,sigma2_un,GammaLic(1:Ninitial),GammaUn(1:Ninitial),...
        Pmax_lic,Pmax_un_fixed,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,AO_traj_iter,STEP_MAX_FIXED,alpha_traj,fd_eps,...
        area,Nsamp_traj,Dmin_UAV,trajSeedBase+10010+drop*1000000,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,...
        'prop-pb-mm','AI-IBPC joint AO trajectory');

    [Q_RES,hRES,statusRES]=build_multi_traj_AO_reserved(Qinit,q_start_all,q_end_all,WS,WP,WW,w_b,w_wifi,H,...
        f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un_fixed,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,AO_traj_iter,STEP_MAX_FIXED,alpha_traj,area,Dmin_UAV,...
        trajSeedBase+10020+drop*1000000,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,Ninitial,reserveLic,reserveUn);

    % h(1) is iteration zero. h(2) is the first accepted outer AO step.
    AI_initial(drop)=hAI(1)/1e6;
    RES_initial(drop)=hRES(1)/1e6;
    AI_accepted(drop)=numel(hAI)-1;
    RES_accepted(drop)=numel(hRES)-1;
    AI_reason{drop}=statusAI;
    RES_reason{drop}=statusRES;
    AI_final(drop)=hAI(end)/1e6;
    RES_final(drop)=hRES(end)/1e6;
    if AI_accepted(drop)>0
        AI_all(drop,1:AI_accepted(drop))=hAI(2:end)'/1e6;
    end
    if RES_accepted(drop)>0
        RES_all(drop,1:RES_accepted(drop))=hRES(2:end)'/1e6;
    end
    fprintf('Drop %2d/%2d | AI: %2d accepted, %-24s %.5f -> %.5f Mbps | RSS: %2d accepted, %-24s %.5f -> %.5f Mbps\n',...
        drop,NDrops,AI_accepted(drop),statusAI,AI_initial(drop),AI_final(drop),...
        RES_accepted(drop),statusRES,RES_initial(drop),RES_final(drop));
end

% The display uses a converged-value extension ONLY for runs that actually
% terminated; these extended values are not counted as executed iterations.
AI_display=AI_all; RES_display=RES_all;
for drop=1:NDrops
    if AI_accepted(drop)==0
        AI_display(drop,:)=AI_initial(drop);
    elseif AI_accepted(drop)<AO_traj_iter
        AI_display(drop,AI_accepted(drop)+1:end)=AI_final(drop);
    end
    if RES_accepted(drop)==0
        RES_display(drop,:)=RES_initial(drop);
    elseif RES_accepted(drop)<AO_traj_iter
        RES_display(drop,RES_accepted(drop)+1:end)=RES_final(drop);
    end
end
AI_Mbps=mean(AI_display,1);
RES_Mbps=mean(RES_display,1);
AI_executed=arrayfun(@(i)sum(AI_accepted>=i),iter_list);
RES_executed=arrayfun(@(i)sum(RES_accepted>=i),iter_list);
Results=table(iter_list(:),AI_Mbps(:),RES_Mbps(:),AI_executed(:),RES_executed(:),...
    'VariableNames',{'Iteration','AI_IBPC_Mbps','IA_RSS_Mbps','AI_Executed_Drops','RSS_Executed_Drops'});
DropResults=table((1:NDrops)',AI_initial,AI_final,AI_accepted,AI_reason,...
    RES_initial,RES_final,RES_accepted,RES_reason,...
    'VariableNames',{'Drop','AI_Initial_Mbps','AI_Final_Mbps','AI_Accepted','AI_StopReason',...
    'RSS_Initial_Mbps','RSS_Final_Mbps','RSS_Accepted','RSS_StopReason'});
disp(Results);
disp(DropResults);
writetable(Results,'AI_IBPC_IA_RSS_Convergence.csv');
writetable(DropResults,'AI_IBPC_IA_RSS_Convergence_Diagnostics.csv');
save('AI_IBPC_IA_RSS_Convergence.mat','Results','DropResults','AI_all','RES_all','AI_display','RES_display');

figure('Color','w','Position',[100 100 620 470]); hold on;
plot(iter_list,RES_Mbps,'-p','LineWidth',2,'MarkerSize',7,'MarkerFaceColor','w');
plot(iter_list,AI_Mbps,'-o','LineWidth',2,'MarkerSize',7,'MarkerFaceColor','w');
ax=gca;
set(ax,'FontName','Times New Roman','FontSize',16,'FontWeight','normal',...
    'LineWidth',1.0,'Position',[0.13 0.13 0.84 0.72]);
grid on; set(ax,'GridLineStyle',':','GridAlpha',0.25);
xlabel('Number of AO iterations'); ylabel('Average sum rate (Mbps)');
legend({'Proposed (IA-RSS)','Proposed (AI-IBPC)'},'Location','northoutside',...
    'Orientation','horizontal','FontName','Times New Roman','FontSize',12,'Box','on');
xticks(1:4:AO_traj_iter); xlim([1 AO_traj_iter]);
exportgraphics(gcf,'AI_IBPC_IA_RSS_Convergence.png','Resolution',300);

