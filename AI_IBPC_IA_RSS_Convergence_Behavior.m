% clc; clear; close all;
% % CONVERGENCE: Proposed I AI-IBPC versus Proposed II IA-RSS.
% % Records the TRUE accepted outer AO/SCA objective after each trajectory iteration.
% % No AI-IBPC or IA-RSS logic is changed.
% 
% area=1000; H=100; U=3; K=6; Ninitial=6; J=10; M=10; Nt=8;
% Bsc_lic=180e3; Bsc_un=180e3; N0_dBmHz=-169;
% sigma2_lic=10.^((N0_dBmHz+10*log10(Bsc_lic)-30)/10);
% sigma2_un=10.^((N0_dBmHz+10*log10(Bsc_un)-30)/10);
% Pmax_lic=0.5; Pmax_un_fixed=0.5; P_PBS=2.5; P_WIFI=1;
% P_PBS_j=P_PBS*ones(J,1); P_WIFI_m=P_WIFI*ones(M,1);
% c=3e8; f_lic=2e9; f_un=5e9; f_pbs=f_lic; f_wifi=f_un;
% n_pbs=3.7; n_wifi=3; sigmaSh_pbs_dB=8; sigmaSh_wifi_dB=6;
% A2G=struct('a',9.61,'b',0.16,'etaLoS_dB',1,'etaNLoS_dB',20,...
%     'sigmaLoS_dB',4,'sigmaNLoS_dB',6);
% useRayleigh=true; useShadow=true; useShadowA2G=true;
% MM_iter=10; MM_leakFrac=1; MM_reg_alpha=1e-3; QoS_target=0;
% threshold_dBm=-80;
% GammaLic=10.^((threshold_dBm-30)/10)*ones(J,1);
% GammaUn=10.^((threshold_dBm-30)/10)*ones(M,1);
% w_b=[area/2 area/2]; w_wifi=[area/2 area/2+200];
% q_start_all=[120 850;260 180;430 620]; q_end_all=q_start_all;
% dt_fixed=5; Tflight=220; N=round(Tflight/dt_fixed)+1; Vmax_UAV=5;
% STEP_MAX_FIXED=Vmax_UAV*dt_fixed; Dmin_UAV=100;
% AO_AP_iter=3; AO_traj_iter=25; alpha_traj=0.30; fd_eps=1;
% Nsamp_traj=5; trajSeedBase=77777; dropSeedBase=9000;
% NDrops=30; MC=300; % Same Monte Carlo settings as the earlier sweep codes.
% reserveLic=(Ninitial+1):J; reserveUn=(Ninitial+1):M;
% 
% rng(dropSeedBase+1,'twister');
% WS=area*rand(K,2); WP=area*rand(J,2); WW=area*rand(M,2);
% Shadow=struct();
% Shadow.pbs_su_dB=sigmaSh_pbs_dB*randn(K,1);
% Shadow.wifi_su_dB=sigmaSh_wifi_dB*randn(K,1);
% Shadow.a2g_su_z=randn(K,1);
% Shadow.a2g_pu_z=randn(J,1);
% Shadow.a2g_wu_z=randn(M,1);
% Sinit=Shadow; Sinit.a2g_pu_z=Shadow.a2g_pu_z(1:Ninitial); Sinit.a2g_wu_z=Shadow.a2g_wu_z(1:Ninitial);
% q_hover_all=make_multi_hover_points_closed(q_start_all,WS,area);
% Qinit=build_multi_fly_hover_trajectory(q_start_all,q_hover_all,q_end_all,N,STEP_MAX_FIXED,area);
% if ~check_multi_traj_feasibility(Qinit,q_start_all,q_end_all,STEP_MAX_FIXED,area,Dmin_UAV)
%     Qinit=repair_multi_trajectory_cvx(Qinit,area,STEP_MAX_FIXED,q_start_all,q_end_all,Dmin_UAV);
% end
% 
% fprintf('True convergence comparison: AI-IBPC and IA-RSS\n');
% [Q_AI,hAI]=build_multi_traj_AO_qos_bf(Qinit,q_start_all,q_end_all,WS,...
%     WP(1:Ninitial,:),WW(1:Ninitial,:),w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Sinit,...
%     P_PBS_j(1:Ninitial),P_WIFI_m(1:Ninitial),sigma2_lic,sigma2_un,GammaLic(1:Ninitial),GammaUn(1:Ninitial),...
%     Pmax_lic,Pmax_un_fixed,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,AO_traj_iter,STEP_MAX_FIXED,alpha_traj,fd_eps,...
%     area,Nsamp_traj,Dmin_UAV,trajSeedBase+10010,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,...
%     'prop-pb-mm','AI-IBPC joint AO trajectory');
% 
% [Q_RES,hRES]=build_multi_traj_AO_reserved(Qinit,q_start_all,q_end_all,WS,WP,WW,w_b,w_wifi,H,...
%     f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%     Pmax_lic,Pmax_un_fixed,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,AO_traj_iter,STEP_MAX_FIXED,alpha_traj,area,Dmin_UAV,...
%     trajSeedBase+10020,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,Ninitial,reserveLic,reserveUn);
% 
% % Iteration 0 is the common initial trajectory. After convergence, hold the final accepted value.
% iter_list=1:AO_traj_iter;
% AI_Mbps=nan(size(iter_list)); RES_Mbps=nan(size(iter_list));
% AI_Mbps(1:numel(hAI))=hAI(:)'/1e6; RES_Mbps(1:numel(hRES))=hRES(:)'/1e6;
% AI_Mbps(numel(hAI)+1:end)=AI_Mbps(numel(hAI));
% RES_Mbps(numel(hRES)+1:end)=RES_Mbps(numel(hRES));
% Results=table(iter_list(:),AI_Mbps(:),RES_Mbps(:),'VariableNames',{'Iteration','AI_IBPC_Mbps','IA_RSS_Mbps'});
% disp(Results);
% writetable(Results,'AI_IBPC_IA_RSS_Convergence.csv');
% save('AI_IBPC_IA_RSS_Convergence.mat','Results','hAI','hRES','Q_AI','Q_RES');
% 
% figure('Color','w','Position',[100 100 620 470]); hold on;
% plot(iter_list,AI_Mbps,'-o','LineWidth',2,'MarkerSize',7,'MarkerFaceColor','w');
% plot(iter_list,RES_Mbps,'-d','LineWidth',2,'MarkerSize',7,'MarkerFaceColor','w');
% ax=gca; set(ax,'FontName','Times New Roman','FontSize',16,'FontWeight','normal','LineWidth',1.0,'Position',[0.13 0.13 0.84 0.72]);
% grid on; set(ax,'GridLineStyle',':','GridAlpha',0.25);
% xlabel('Number of iterations'); ylabel('Average sum rate (Mbps)');
% legend({'Proposed I (AI-IBPC)','Proposed II (IA-RSS)'},'Location','northoutside','Orientation','horizontal','FontName','Times New Roman','FontSize',12,'Box','on');
% xticks(0:5:AO_traj_iter); xlim([0 AO_traj_iter]);
% exportgraphics(gcf,'AI_IBPC_IA_RSS_Convergence.png','Resolution',300);
% 
% function same=compare_tx_numeric_stage(a,b,tol)
% same=numel(a)==numel(b);
% if ~same,return;end
% for uu=1:numel(a)
%     fields={'users','idxLic','idxUn','pLic','pUn'};
%     for ff=1:numel(fields)
%         x=a(uu).(fields{ff}); y=b(uu).(fields{ff});
%         if ~isequal(size(x),size(y)) || numel(x)~=numel(y)
%             same=false;return;
%         end
%         if isempty(x),continue;end
%         if max(abs(double(x(:))-double(y(:))))>tol
%             same=false;return;
%         end
%     end
%     for field={'vLic','vUn'}
%         x=a(uu).(field{1});y=b(uu).(field{1});
%         if numel(x)~=numel(y),same=false;return;end
%         for ii=1:numel(x)
%             if ~isequal(size(x{ii}),size(y{ii})) || ...
%                     max(abs(x{ii}(:)-y{ii}(:)))>tol
%                 same=false;return;
%             end
%         end
%     end
% end
% end
% function Qnew = resample_multi_trajectory_local(Qold,Nnew)
% U = size(Qold,1);
% Nold = size(Qold,2);
% Qnew = zeros(U,Nnew,2);
% xOld = linspace(0,1,Nold);
% xNew = linspace(0,1,Nnew);
% for u = 1:U
%     Qnew(u,:,1) = interp1(xOld,squeeze(Qold(u,:,1)),xNew,'linear','extrap');
%     Qnew(u,:,2) = interp1(xOld,squeeze(Qold(u,:,2)),xNew,'linear','extrap');
% end
% end
% %% ============================================================
% %                  MULTI-UAV TRAJECTORY FUNCTIONS
% %% ============================================================
% function Q = build_multi_straight_trajectory(q_start_all,q_end_all,N)
% U = size(q_start_all,1);
% Q = zeros(U,N,2);
% for u = 1:U
%     for n = 1:N
%         a = (n-1)/(N-1);
%         Q(u,n,:) = (1-a)*q_start_all(u,:) + a*q_end_all(u,:);
%     end
% end
% end
% function q_hover_all = make_multi_hover_points_closed(q_start_all,WS,area)
% % Closed-loop compatible hover/service point selection.
% % Each SU is assigned to its nearest UAV initial point. Each UAV then uses
% % the centroid of its assigned SUs as the hover/service point.
% U = size(q_start_all,1);
% K = size(WS,1);
% assign = zeros(K,1);
% q_hover_all = zeros(U,2);
% for k = 1:K
%     distToUAV = vecnorm(q_start_all - WS(k,:),2,2);
%     [~,uBest] = min(distToUAV);
%     assign(k) = uBest;
% end
% for u = 1:U
%     ids = find(assign == u);
%     if isempty(ids)
%         q_hover_all(u,:) = q_start_all(u,:);
%     else
%         q_hover_all(u,:) = mean(WS(ids,:),1);
%     end
%     q_hover_all(u,:) = min(max(q_hover_all(u,:),0),area);
% end
% end
% function Q = build_multi_fly_hover_trajectory(q_start_all,q_hover_all,q_end_all,N,stepMax,area)
% % Closed-loop/non-stationary initialization.
% % If q_end_all = q_start_all,this creates q0 -> hover/service point -> q0.
% % If the hover point is too far for the available N and stepMax,it uses the
% % farthest feasible point along the q0-to-hover direction and still returns.
% U = size(q_start_all,1);
% Q = zeros(U,N,2);
% for u = 1:U
%     qs = q_start_all(u,:);
%     qh = q_hover_all(u,:);
%     qe = q_end_all(u,:);
%     d1 = norm(qh - qs);
%     d2 = norm(qe - qh);
%     totalDist = d1 + d2;
%     maxDist = stepMax*(N-1);
%     % If the selected hover point is infeasible,pull it back along the
%     % start-hover direction so that a return path remains feasible.
%     if totalDist > maxDist && d1 > 1e-12
%         % For closed-loop qs=qe,the feasible hover distance is maxDist/2.
%         % For general qs~=qe this scale is a safe conservative fallback.
%         scale = min(1,0.95*maxDist/max(totalDist,1e-12));
%         qh = qs + scale*(qh - qs);
%         qh = min(max(qh,0),area);
%         d1 = norm(qh - qs);
%         d2 = norm(qe - qh);
%     end
%     n1 = ceil(d1/max(stepMax,1e-12));
%     n2 = ceil(d2/max(stepMax,1e-12));
%     totalMove = n1 + n2;
%     pts = zeros(N,2);
%     idx = 1;
%     pts(idx,:) = qs;
%     if totalMove <= N-1 && totalMove > 0
%         hoverSteps = (N-1) - totalMove;
%         for ii = 1:n1
%             idx = idx + 1;
%             a = ii/max(n1,1);
%             pts(idx,:) = (1-a)*qs + a*qh;
%         end
%         for ii = 1:hoverSteps
%             idx = idx + 1;
%             pts(idx,:) = qh;
%         end
%         for ii = 1:n2
%             idx = idx + 1;
%             a = ii/max(n2,1);
%             pts(idx,:) = (1-a)*qh + a*qe;
%         end
%         if idx < N
%             pts(idx+1:N,:) = repmat(qe,N-idx,1);
%         end
%     else
%         % Final fallback: direct path. This only becomes stationary when
%         % qs=qe and no feasible hover displacement exists.
%         for n = 1:N
%             a = (n-1)/(N-1);
%             pts(n,:) = (1-a)*qs + a*qe;
%         end
%     end
%     Q(u,:,:) = min(max(pts(1:N,:),0),area);
% end
% Q = project_multi_trajectory(Q,area,stepMax,q_start_all,q_end_all,30);
% % Collision avoidance is checked/repaired by the caller after initialization.
% end
% function [Qout,rateHist] = build_multi_traj_AO_qos_bf(Qinit,q_start_all,q_end_all,WS,WP,WW,w_b,w_wifi,H,...
%         f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%         P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%         Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,...
%         AO_traj_iter,stepMax,alpha_traj,fd_eps,area,Nsamp,Dmin_UAV,seedBase,...
%         useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,...
%         methodMode,displayName)
% % OUTER AO WITH TRAJECTORY AS THE FINAL BLOCK.
% % ------------------------------------------------------------
% % At the current trajectory Q:
% %   (1) association
% %   (2) subchannel allocation on both bands
% %   (3) CBF-MM beamforming
% %   (4) KKT/water-filling power allocation
% %   (5) integrated four-case AI-IBPC
% % are solved by eval_slot_multi_qos_bf().
% %
% % Only AFTER those communication variables are obtained do we perform:
% %   (6) one JOINT all-UAV/all-waypoint analytical first-order Taylor SCA
% %       trajectory update.
% %
% % During the SCA step the accepted communication variables are frozen while
% % differentiating the original Al-Hourani large-scale gain analytically.
% % The resulting model is local, so a trust region/backtracking is used.
% % A trajectory candidate is accepted ONLY after the complete communication
% % problem (including AI-IBPC) is re-solved at Qtrial and its true sum rate is
% % no smaller than the incumbent.  Hence the sequence is:
% %
% %   resources + AI-IBPC  ->  trajectory SCA  ->  re-solve resources + AI-IBPC
% %
% % and then repeat until AO_traj_iter or convergence.
% % fd_eps is retained only for interface compatibility; no finite differences.
% if exist('cvx_begin','file') ~= 2
%     error('CVX is required for joint trajectory optimization.');
% end
% U=size(Qinit,1); N=size(Qinit,2);
% if check_multi_traj_feasibility(Qinit,q_start_all,q_end_all,stepMax,area,Dmin_UAV)
%     Qout=Qinit;
% else
%     Qout=repair_multi_trajectory_cvx(Qinit,area,stepMax,q_start_all,q_end_all,Dmin_UAV);
% end
% fixedEvalSeed=seedBase+700000;
% prevRate=evaluate_multi_trajectory_qos_bf(Qout,WS,WP,WW,w_b,w_wifi,H,...
%     f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%     P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%     Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,fixedEvalSeed,...
%     useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
% rateHist=nan(AO_traj_iter+1,1); rateHist(1)=prevRate; histIdx=1;
% maxBacktracks=3; minRadius=1e-3;
% for ao=1:AO_traj_iter
%     Qbase=Qout;
%     gradRate=zeros(U,N,2);
%     baseLeak=zeros(numel(GammaLic)+numel(GammaUn),N);
%     gradLeak=zeros(numel(GammaLic)+numel(GammaUn),U,N,2);
%     % All slot gradients are assembled BEFORE the single global CVX solve.
%     for n=2:N-1
%         Qslot=squeeze(Qbase(:,n,:));
%         seed=fixedEvalSeed+n; % same realized fading as the acceptance objective
%         % Communication solution FIRST at fixed Qslot.  tx returned here is
%         % already the protected, rate-aware AI-IBPC solution.
%         [~,~,~,tx] = eval_slot_multi_qos_bf(Qslot,WS,WP,WW,w_b,w_wifi,H,...
%             f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%             P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%             Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,...
%             seed,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
%         [gr,leak,gl]=joint_analytic_slot_derivatives(Qslot,tx,WS,WP,WW,w_b,w_wifi,H,...
%             f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%             P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%             seed,useRayleigh,useShadow,useShadowA2G,Nt,Bsc_lic,Bsc_un);
%         gradRate(:,n,:)=reshape(gr,[U,1,2]);
%         baseLeak(:,n)=leak;
%         gradLeak(:,:,n,:)=reshape(gl,[numel(leak),U,1,2]);
%     end
%     if ~all(isfinite(gradRate(:))) || ~all(isfinite(gradLeak(:)))
%         warning('Nonfinite analytical derivatives: retaining previous trajectory.');
%         break;
%     end
%     radius=min(stepMax,max(minRadius,alpha_traj*stepMax));
%     accepted=false;
%     for bt=1:maxBacktracks
%         [Qtrial,solved]=joint_traj_cvx_analytic(Qbase,gradRate,baseLeak,gradLeak,...
%             q_start_all,q_end_all,stepMax,area,radius,Dmin_UAV);
%         if solved && check_multi_traj_feasibility(Qtrial,q_start_all,q_end_all,stepMax,area,Dmin_UAV)
%             % Re-solve ALL communication blocks, including AI-IBPC, at the
%             % candidate trajectory before accepting the SCA waypoint update.
%             trialRate=evaluate_multi_trajectory_qos_bf(Qtrial,WS,WP,WW,w_b,w_wifi,H,...
%                 f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%                 P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%                 Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,fixedEvalSeed,...
%                 useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
%             if isfinite(trialRate) && trialRate>=prevRate-1e-9*max(1,abs(prevRate))
%                 Qout=Qtrial;
%                 accepted=true;
%                 if abs(trialRate-prevRate)<=1e-5*max(1,abs(prevRate))
%                     prevRate=trialRate; histIdx=histIdx+1; rateHist(histIdx)=prevRate;
%                     rateHist=rateHist(1:histIdx);
%                     return;
%                 end
%                 prevRate=trialRate; histIdx=histIdx+1; rateHist(histIdx)=prevRate;
%                 break;
%             end
%         end
%         radius=radius/2;
%         if radius<minRadius, break; end
%     end
%     if ~accepted
%         break;
%     end
% end
% rateHist=rateHist(1:histIdx);
% end
% function [Qout,rateHist]=build_multi_traj_AO_reserved(Qinit,q_start_all,q_end_all,WS,WP,WW,w_b,w_wifi,H,...
%     f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,...
%     PL,PU,QoS,BL,BU,AO_AP_iter,AO_traj_iter,stepMax,alpha_traj,area,Dmin,seedBase,...
%     useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU)
% % Proposed-II trajectory AO. Same analytical SCA/trust-region machinery as
% % Proposed I, but every incumbent/trial trajectory is evaluated after the
% % Reserved-subchannel controller, not AI-IBPC.
% if check_multi_traj_feasibility(Qinit,q_start_all,q_end_all,stepMax,area,Dmin)
%     Qout=Qinit;
% else
%     Qout=repair_multi_trajectory_cvx(Qinit,area,stepMax,q_start_all,q_end_all,Dmin);
% end
% fixedSeed=seedBase+700000;
% prevRate=evaluate_reserved_trajectory(Qout,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,...
%     n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,fixedSeed,...
%     useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU);
% rateHist=nan(AO_traj_iter+1,1); rateHist(1)=prevRate; histIdx=1;
% U=size(Qout,1); N=size(Qout,2); maxBacktracks=3; minRadius=1e-3;
% for ao=1:AO_traj_iter
%     Qbase=Qout; gradRate=zeros(U,N,2); baseLeak=zeros(numel(GL)+numel(GU),N);
%     gradLeak=zeros(numel(GL)+numel(GU),U,N,2);
%     for n=2:N-1
%         Qslot=squeeze(Qbase(:,n,:)); seed=fixedSeed+n;
%         [tx,~]=eval_slot_reserved_final(Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,...
%             n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,seed,...
%             useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU);
%         [gr,leak,gl]=joint_analytic_slot_derivatives(Qslot,tx,WS,WP,WW,w_b,w_wifi,H,...
%             f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,...
%             seed,useRayleigh,useShadow,useShadowA2G,Nt,BL,BU);
%         gradRate(:,n,:)=reshape(gr,[U,1,2]); baseLeak(:,n)=leak;
%         gradLeak(:,:,n,:)=reshape(gl,[numel(leak),U,1,2]);
%     end
%     if ~all(isfinite(gradRate(:))) || ~all(isfinite(gradLeak(:))), break; end
%     radius=min(stepMax,max(minRadius,alpha_traj*stepMax)); accepted=false;
%     for bt=1:maxBacktracks
%         [Qtrial,solved]=joint_traj_cvx_analytic(Qbase,gradRate,baseLeak,gradLeak,...
%             q_start_all,q_end_all,stepMax,area,radius,Dmin);
%         if solved && check_multi_traj_feasibility(Qtrial,q_start_all,q_end_all,stepMax,area,Dmin)
%             trialRate=evaluate_reserved_trajectory(Qtrial,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,...
%                 n_pbs,n_wifi,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,fixedSeed,...
%                 useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU);
%             if isfinite(trialRate) && trialRate>=prevRate-1e-9*max(1,abs(prevRate))
%                 Qout=Qtrial; accepted=true;
%                 if abs(trialRate-prevRate)<=1e-5*max(1,abs(prevRate))
%                     prevRate=trialRate; histIdx=histIdx+1; rateHist(histIdx)=prevRate;
%                     rateHist=rateHist(1:histIdx);
%                     return;
%                 end
%                 prevRate=trialRate; histIdx=histIdx+1; rateHist(histIdx)=prevRate; break;
%             end
%         end
%         radius=radius/2; if radius<minRadius,break;end
%     end
%     if ~accepted,break;end
% end
% rateHist=rateHist(1:histIdx);
% end
% 
% function avgRate=evaluate_reserved_trajectory(Q,WS,WP,WW,w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,nP,nW,Shadow,...
%     Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,seedBase,useRayleigh,useShadow,...
%     useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU)
% s=0; N=size(Q,2);
% for n=1:N
%     [~,r]=eval_slot_reserved_final(squeeze(Q(:,n,:)),WS,WP,WW,w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,...
%         nP,nW,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,seedBase+n,...
%         useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU);
%     s=s+r;
% end
% avgRate=s/N;
% end
% 
% function [txReserved,rateReserved]=eval_slot_reserved_final(Qslot,WS,WP,WW,w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,...
%     nP,nW,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,seedNow,useRayleigh,...
%     useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU)
% % Ordinary allocation is generated on the active (non-reserved) channels.
% S0=Shadow; S0.a2g_pu_z=Shadow.a2g_pu_z(1:Ninitial); S0.a2g_wu_z=Shadow.a2g_wu_z(1:Ninitial);
% [~,~,~,~,~,dd]=eval_slot_multi_qos_bf(Qslot,WS,WP(1:Ninitial,:),WW(1:Ninitial,:),w_b,w_wifi,H,...
%     fL,fU,fP,fW,c,A2G,nP,nW,S0,Ppbs(1:Ninitial),Pwifi(1:Ninitial),nL,nU,GL(1:Ninitial),GU(1:Ninitial),...
%     PL,PU,QoS,BL,BU,AO_AP_iter,seedNow,useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,'prop-pb-mm',0,true);
% raw=dd.rawTx;
% [HL0,HU0,HP0,HW0,hp,hw]=gen_multi_channels_vec(Qslot,WS,WP(1:Ninitial,:),WW(1:Ninitial,:),...
%     w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,nP,nW,S0,seedNow,useRayleigh,useShadow,useShadowA2G,Nt);
% [~,~,HPx,HWx]=gen_multi_channels_vec(Qslot,WS,WP,WW,w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,nP,nW,...
%     Shadow,seedNow+250000,useRayleigh,useShadow,useShadowA2G,Nt);
% HL=HL0; HU=HU0; HP=cell(size(Qslot,1),numel(GL)); HW=cell(size(Qslot,1),numel(GU));
% HP(:,1:Ninitial)=HP0; HW(:,1:Ninitial)=HW0; HP(:,resL)=HPx(:,resL); HW(:,resU)=HWx(:,resU);
% cases=swap_classify_cases(raw,HP,HW,GL,GU);
% [txReserved,~]=fourcase_crossband_reserved_controller(raw,cases,HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,...
%     Ppbs,Pwifi,nL,nU,BL,BU,mmIter,leakFrac,regAlpha,resL,resU);
% rateReserved=diagnostic_tx_rate(txReserved,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
% end
% 
% function [Qtrial,solved]=joint_traj_cvx_analytic(Q0,gradR,L0,gradL,qStart,qEnd,stepMax,area,radius,Dmin_UAV)
% % A SINGLE convex program for all 3 UAVs and all N waypoints.
% % Linearized normalized aggregate interference is enforced conservatively
% % locally; true protected interference is rechecked by slot allocation.
% [U,N,~]=size(Q0); S=size(L0,1);
% X0=reshape(permute(Q0,[2 3 1]),N,2*U);
% G=reshape(permute(gradR,[2 3 1]),N,2*U);
% % Scale rate gradient to improve numerical conditioning.
% scale=max(1,max(abs(G(:))));
% solved=false; Qtrial=Q0;
% try
%     cvx_begin quiet
%         variable X(N,2*U)
%         expressions leakageLinear(S,N)
%         maximize( sum(sum((G/scale).*(X-X0))) - ...
%             (1e-4/max(radius^2,1e-9))*sum_square(reshape(X-X0,N*2*U,1)) )
%         subject to
%             X(:)>=0; X(:)<=area;
%             for u=1:U
%                 cols=2*u-1:2*u;
%                 X(1,cols)==qStart(u,:);
%                 X(N,cols)==qEnd(u,:);
%                 for n=1:N-1
%                     norm(X(n+1,cols)-X(n,cols),2)<=stepMax;
%                 end
%                 for n=2:N-1
%                     norm(X(n,cols)-X0(n,cols),2)<=radius;
%                 end
%             end
%             % Convex inner approximation of pairwise non-collision.
%             % At feasible Q0: ||d||^2 >= Dmin^2 is conservatively enforced
%             % by ||d0||^2 + 2*d0'*(d-d0) >= Dmin^2.
%             % The same check applies to all time slots, including endpoints.
%             for n=1:N
%                 for u=1:U-1
%                     for v=u+1:U
%                         cu=2*u-1:2*u; cv=2*v-1:2*v;
%                         d0=X0(n,cu)-X0(n,cv);
%                         if sum(d0.^2)<Dmin_UAV^2-1e-5
%                             error('Joint SCA requires a collision-free expansion path.');
%                         end
%                         2*d0*(X(n,cu)-X(n,cv)).' - sum(d0.^2) >= Dmin_UAV^2;
%                     end
%                 end
%             end
%             % Aggregate PU/WU protection is re-enforced by the complete
%             % AI-IBPC / IA-RSS communication re-optimization at Qtrial.
%             % Do not impose an additional frozen-resource linearized leakage
%             % constraint here; it can make the local trajectory SCA infeasible.
%     cvx_end
%     solved=contains(lower(cvx_status),'solved') && all(isfinite(X(:)));
%     if solved
%         Qtrial=permute(reshape(X,N,2,U),[3 1 2]);
%     end
% catch ME
%     warning('Joint trajectory CVX: %s',ME.message);
%     try,cvx_clear;catch,end
% end
% end
% function [gradRate,L0,gradLeak]=joint_analytic_slot_derivatives(Q,tx,WS,WP,WW,w_b,w_wifi,H,...
%     fL,fU,fP,fW,c,A2G,nP,nW,Shadow,Ppbs,Pwifi,sigmaL,sigmaU,GammaL,GammaU,...
%     seed,useRayleigh,useShadow,useShadowA2G,Nt,BscL,BscU)
% % Exact derivative of frozen-resource, fixed-fading SINR w.r.t. positions.
% % h(q)=h(q0)*sqrt(g(q)/g(q0)) under common small-scale fading.
% [HL,HU,HP,HW,hp2,hw2]=gen_multi_channels_vec(Q,WS,WP,WW,w_b,w_wifi,H,...
%     fL,fU,fP,fW,c,A2G,nP,nW,Shadow,seed,useRayleigh,useShadow,useShadowA2G,Nt);
% U=size(Q,1); S=numel(GammaL)+numel(GammaU);
% gradRate=zeros(U,2); L0=zeros(S,1); gradLeak=zeros(S,U,2);
% for band=1:2
%     if band==1
%         HS=HL; HT=HP; W=WP; f=fL; hI=hp2; PI=Ppbs; noise=sigmaL;
%         gamma=GammaL; offset=0; bw='lic'; Bsc=BscL;
%     else
%         HS=HU; HT=HW; W=WW; f=fU; hI=hw2; PI=Pwifi; noise=sigmaU;
%         gamma=GammaU; offset=numel(GammaL); bw='un'; Bsc=BscU;
%     end
%     K=size(WS,1); gainGrad=zeros(U,K,2);
%     for u=1:U
%         for k=1:K
%             [~,dg]=a2g_gain_and_gradient(Q(u,:),WS(k,:),H,f,c,A2G,...
%                 useShadowA2G,Shadow.a2g_su_z(k));
%             gainGrad(u,k,:)=dg;
%         end
%         [idx,p,vList]=get_band_fields(tx,u,bw);
%         for ell=1:size(idx,1)
%             sc=idx(ell,2);
%             if sc<1 || sc>numel(gamma),continue;end
%             [~,dg]=a2g_gain_and_gradient(Q(u,:),W(sc,:),H,f,c,A2G,...
%                 useShadowA2G,ternary_shadow_z(Shadow,sc,band));
%             term=p(ell)*abs(HT{u,sc}'*vList{ell})^2;
%             L0(offset+sc)=L0(offset+sc)+term/max(gamma(sc),realmin);
%             gradLeak(offset+sc,u,:)=reshape(squeeze(gradLeak(offset+sc,u,:)).' +...
%                 term/max(gamma(sc),realmin)*dg,[1,1,2]);
%         end
%     end
%     for u=1:U
%         [idx,p,vList]=get_band_fields(tx,u,bw);
%         for ell=1:size(idx,1)
%             k=tx(u).users(idx(ell,1)); sc=idx(ell,2);
%             if sc<1 || sc>numel(PI),continue;end
%             sig=p(ell)*abs(HS{u,k}'*vList{ell})^2;
%             den=noise+hI(k)*PI(sc);
%             denGrad=zeros(U,2);
%             for v=1:U
%                 if v==u,continue;end
%                 [idxV,pV,vV]=get_band_fields(tx,v,bw);
%                 for ee=1:size(idxV,1)
%                     if idxV(ee,2)==sc
%                         inter=pV(ee)*abs(HS{v,k}'*vV{ee})^2;
%                         den=den+inter;
%                         denGrad(v,:)=denGrad(v,:)+inter*squeeze(gainGrad(v,k,:)).';
%                     end
%                 end
%             end
%             den=max(den,1e-30);
%             coeff=1/(log(2)*(den+sig));
%             gradRate(u,:)=gradRate(u,:)+sig*coeff*squeeze(gainGrad(u,k,:)).'*Bsc;
%             for v=1:U
%                 if v~=u
%                     gradRate(v,:)=gradRate(v,:)-sig/den*coeff*denGrad(v,:)*Bsc;
%                 end
%             end
%         end
%     end
% end
% end
% function z=ternary_shadow_z(Shadow,sc,band)
% if band==1,z=Shadow.a2g_pu_z(sc);else,z=Shadow.a2g_wu_z(sc);end
% end
% function [g,gradLog]=a2g_gain_and_gradient(q,w,H,f,c,A,useShadow,z)
% % gradLog = gradient of log(g), not gradient of g.
% d=q-w; r2=sum(d.^2); r=sqrt(r2); D=r2+H^2;
% theta=atan2d(H,max(r,1e-9));
% p=1/(1+A.a*exp(-A.b*(theta-A.a)));
% if r<1e-9
%     gradTheta=[0,0];
% else
%     gradTheta=-(180/pi)*H/D*d/r;
% end
% gradP=A.b*p*(1-p)*gradTheta;
% eta=p*A.etaLoS_dB+(1-p)*A.etaNLoS_dB;
% gradEta=(A.etaLoS_dB-A.etaNLoS_dB)*gradP;
% if useShadow
%     variance=p*A.sigmaLoS_dB^2+(1-p)*A.sigmaNLoS_dB^2;
%     sd=sqrt(max(variance,1e-30));
%     sh=sd*z;
%     gradSh=z*(A.sigmaLoS_dB^2-A.sigmaNLoS_dB^2)/(2*sd)*gradP;
% else
%     sh=0; gradSh=[0,0];
% end
% pl=20*log10(4*pi*f*sqrt(D)/c)+eta+sh;
% g=10^(-pl/10);
% gradLog=-d/D-(log(10)/10)*(gradEta+gradSh);
% % FSPL is 20log10(distance), so its log-gain derivative is -2*d/D.
% gradLog=gradLog-d/D;
% end
% function [fAvg,Lworst] = eval_actual_slot_objective_for_traj(Qslot,WS,WP,WW,w_b,w_wifi,H,...
%         f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%         P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%         Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,seedBase,...
%         useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,...
%         methodMode,nAvg)
% % Evaluate reoptimized and protected communication variables at each point.
% % Worst leakage across identical channel samples is used for the constraints.
% fAvg=0;
% Lworst=zeros(numel(GammaLic)+numel(GammaUn),1);
% for ss=1:max(nAvg,1)
%     [rate,leakL,leakU]=eval_slot_multi_qos_bf(Qslot,WS,WP,WW,w_b,w_wifi,H,...
%         f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%         P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%         Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,seedBase+1000*ss,...
%         useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
%     fAvg=fAvg+rate;
%     Lworst=max(Lworst,[leakL(:)./GammaLic(:);leakU(:)./GammaUn(:)]);
% end
% fAvg=fAvg/max(nAvg,1);
% end
% function Qfixed=repair_multi_trajectory_cvx(Qraw,area,stepMax,qStart,qEnd,Dmin_UAV)
% % Joint convex projection with a safe collision-free straight-line anchor.
% % Collision halfspaces linearized at the anchor are an INNER approximation;
% % the safe anchor is a fallback, never an unchecked per-UAV projection.
% U=size(Qraw,1); N=size(Qraw,2);
% anchor=zeros(U,N,2);
% for u=1:U
%     if norm(qEnd(u,:)-qStart(u,:))>(N-1)*stepMax+1e-8
%         error('UAV %d: endpoints unreachable.',u);
%     end
%     for n=1:N
%         t=(n-1)/(N-1);
%         anchor(u,n,:)=(1-t)*qStart(u,:)+t*qEnd(u,:);
%     end
% end
% if ~check_multi_traj_feasibility(anchor,qStart,qEnd,stepMax,area,Dmin_UAV)
%     error(['Straight-line collision-free anchor unavailable. Supply a ' ...
%            'collision-free initial trajectory for this geometry.']);
% end
% if check_multi_traj_feasibility(Qraw,qStart,qEnd,stepMax,area,Dmin_UAV)
%     Qfixed=Qraw; return;
% end
% Xraw=reshape(permute(Qraw,[2 3 1]),N,2*U);
% X0=reshape(permute(anchor,[2 3 1]),N,2*U);
% Qfixed=anchor;
% try
%     cvx_begin quiet
%         variable X(N,2*U)
%         minimize(sum_square(reshape(X-Xraw,N*2*U,1)))
%         subject to
%             X(:)>=0; X(:)<=area;
%             for u=1:U
%                 cols=2*u-1:2*u;
%                 X(1,cols)==qStart(u,:);
%                 X(N,cols)==qEnd(u,:);
%                 for n=1:N-1
%                     norm(X(n+1,cols)-X(n,cols),2)<=stepMax;
%                 end
%             end
%             for n=1:N
%                 for u=1:U-1
%                     for v=u+1:U
%                         cu=2*u-1:2*u; cv=2*v-1:2*v;
%                         d0=X0(n,cu)-X0(n,cv);
%                         2*d0*(X(n,cu)-X(n,cv)).'-sum(d0.^2)>=Dmin_UAV^2;
%                     end
%                 end
%             end
%     cvx_end
%     if contains(lower(cvx_status),'solved') && all(isfinite(X(:)))
%         candidate=permute(reshape(X,N,2,U),[3 1 2]);
%         if check_multi_traj_feasibility(candidate,qStart,qEnd,stepMax,area,Dmin_UAV)
%             Qfixed=candidate;
%         else
%             warning('Joint collision-safe repair failed feasibility check; using safe anchor.');
%         end
%     else
%         warning('Joint collision-safe repair status %s; using safe anchor.',cvx_status);
%     end
% catch ME
%     warning('Joint collision-safe repair: %s; using safe anchor.',ME.message);
%     try,cvx_clear;catch,end
% end
% assert(check_multi_traj_feasibility(Qfixed,qStart,qEnd,stepMax,area,Dmin_UAV), ...
%     'Unable to construct a collision-free feasible path.');
% end
% function report_multi_traj_violations(Q,qStart,qEnd,stepMax,area)
% for u=1:size(Q,1)
%     X=squeeze(Q(u,:,:));
%     steps=sqrt(sum(diff(X,1,1).^2,2));
%     fprintf('  UAV %d: start error %.3g m; end error %.3g m; max step %.9f / %.9f m; coordinate range [%.5f, %.5f].\n', ...
%         u,norm(X(1,:)-qStart(u,:)),norm(X(end,:)-qEnd(u,:)), ...
%         max(steps),stepMax,min(X(:)),max(X(:)));
% end
% end
% function ok=check_multi_traj_feasibility(Q,qStart,qEnd,stepMax,area,Dmin_UAV)
% ok=true;
% for u=1:size(Q,1)
%     X=squeeze(Q(u,:,:));
%     tolFeas=1e-3; % Numerical CVX feasibility tolerance only (meters).
%     ok=ok && norm(X(1,:)-qStart(u,:))<tolFeas && ...
%         norm(X(end,:)-qEnd(u,:))<tolFeas && ...
%         all(X(:)>=-tolFeas) && all(X(:)<=area+tolFeas) && ...
%         all(sqrt(sum(diff(X,1,1).^2,2))<=stepMax+tolFeas);
% end
% if nargin>=6 && Dmin_UAV>0
%     for n=1:size(Q,2)
%         for u=1:size(Q,1)-1
%             for v=u+1:size(Q,1)
%                 duv=squeeze(Q(u,n,:)-Q(v,n,:));
%                 ok=ok && norm(duv,2)>=Dmin_UAV-tolFeas;
%             end
%         end
%     end
% end
% end
% function avgRate = evaluate_multi_trajectory_qos_bf(Q,WS,WP,WW,w_b,w_wifi,H,...
%         f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%         P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%         Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,seedBase,...
%         useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode)
% N = size(Q,2);
% sumRate = 0;
% for n = 1:N
%     sumRate = sumRate + eval_slot_multi_qos_bf(...
%         squeeze(Q(:,n,:)),WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
%         A2G,n_pbs,n_wifi,Shadow,P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,...
%         GammaLic,GammaUn,Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,...
%         seedBase+n,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
% end
% avgRate = sumRate/N;
% end
% function [slotRate,leakLic,leakUn,txBest,userRate,diagnostics] = eval_slot_multi_qos_bf(Qslot,WS,WP,WW,w_b,w_wifi,H,...
%         f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%         P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%         Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,seedNow,...
%         useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode,compareMode,stageDebug,forcedAssoc)
% if ~exist('compareMode','var') || isempty(compareMode), compareMode=0; end
% if ~exist('stageDebug','var') || isempty(stageDebug),stageDebug=false;end
% if ~exist('forcedAssoc','var'),forcedAssoc=[];end
% % MATLAB requires persistent declarations at function scope before executable
% % statements, not inside an AO iteration loop.
% persistent aoRejectCount aoRejectInfeasible aoRejectRate aoRejectNonfinite
% if isempty(aoRejectCount)
%     aoRejectCount = 0;
%     aoRejectInfeasible = 0;
%     aoRejectRate = 0;
%     aoRejectNonfinite = 0;
% end
% % ============================================================
% % INNER AO: 3 C-UAV DUAL-BAND INDEPENDENT-BUDGET CBF-MM
% % ------------------------------------------------------------
% % Fixed Qslot and fixed instantaneous channels:
% %   Block 1: User association.
% %   Block 2: Licensed subchannel allocation.
% %   Block 3: Licensed CBF-MM beamforming + KKT/water-filling power.
% %   Block 4: Unlicensed allocation for all associated users.
% %   Block 5: Unlicensed CBF-MM beamforming + KKT/water-filling power.
% %   Block 6: Measure BOTH initial aggregate PU and WU interference.
% %   Block 7: Integrated four-case AI-IBPC:
% %            protection + rate-aware power adjustment.
% % There is NO separate post-AI-IBPC protection/clipping block.
% %
% % The unlicensed band uses its own fixed per-UAV budget Pmax_un; no cross-band transfer is allowed.
% %
% % These blocks are repeated for AO_AP_iter or until the slot sum-rate
% % converges. The previous AO iterate is used in the allocation metric through
% % inter-UAV co-channel interference terms,so the loop is not a dummy repeat.
% % ============================================================
% [HsuL,HsuU,HpuL,HwuU,hp2,hw2,hu2L_all] = gen_multi_channels_vec(...
%     Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
%     A2G,n_pbs,n_wifi,Shadow,seedNow,useRayleigh,useShadow,useShadowA2G,Nt);
% U = size(Qslot,1);
% Kglob = size(WS,1);
% J = numel(P_PBS_j);
% M = numel(P_WIFI_m);
% AO_AP_iter = max(1,round(AO_AP_iter));
% AO_tol = 1e-4;
% txPrev = repmat(struct('users',[],'idxLic',zeros(0,2),'pLic',[],'vLic',{{}},...
%                        'idxUn',zeros(0,2),'pUn',[],'vUn',{{}}),U,1);
% % The all-silent configuration is a feasible initial accepted AO iterate.
% % Every subsequent accepted communication update is checked against the
% % SAME realized channels and SAME global two-band sum-rate objective.
% slotRateOld = 0;
% txBest = txPrev;
% slotRateBest = 0;
% acceptedAOHistory = nan(AO_AP_iter+1,1);
% acceptedAOHistory(1) = 0;
% acceptedCount = 0;
% diagnostics=struct('transferCount',0,'donorLossMbps',0);
% for itAO = 1:AO_AP_iter
%     % ===== Block 1: user association for fixed Qslot =====
%     % Highest licensed-band channel-gain association.
%     % Each SU is associated with the C-UAV providing the largest
%     % instantaneous licensed-band channel gain ||h_{u,k}^{lic}[n]||^2.
%     assoc = associate_users_highest_channel_gain(HsuL);
%     if ~isempty(forcedAssoc)
%         assert(numel(forcedAssoc)==Kglob && all(forcedAssoc>=1) && ...
%             all(forcedAssoc<=U),'Invalid forced association');
%         assoc=forcedAssoc(:);
%     end
%     tx = repmat(struct('users',[],'idxLic',zeros(0,2),'pLic',[],'vLic',{{}},...
%                        'idxUn',zeros(0,2),'pUn',[],'vUn',{{}}),U,1);
%     for u = 1:U
%         tx(u).users = find(assoc == u).';
%     end
%     %% ===== Block 2: licensed Hungarian + CBF-MM + KKT =====
%     for u = 1:U
%         users_u = tx(u).users;
%         if isempty(users_u)
%             continue;
%         end
%         HsuL_u = HsuL(u,users_u);
%         HpuL_u = HpuL(u,:);
%         hp2_u  = hp2(users_u);
%         extraNoiseL = previous_interuav_noise_matrix(...
%             txPrev,HsuL,u,users_u,J,'lic');
%         rhoL = assign_licensed_vec_extra_noise(...
%             HsuL_u,HpuL_u,hp2_u,P_PBS_j,sigma2_lic,extraNoiseL,...
%             GammaLic,Pmax_lic,Bsc_lic,numel(users_u),J,...
%             'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,...
%             Pmax_lic/max(J,1));
%         [tx(u).pLic,tx(u).idxLic,tx(u).vLic] = solve_band_mm_given_rho(...
%             rhoL,HsuL_u,HpuL_u,hp2_u,P_PBS_j,sigma2_lic,...
%             GammaLic,Pmax_lic,MM_iter,MM_leakFrac,MM_reg_alpha);
%     end
%     % Do not clip yet: complete both interference-unaware allocations first.
%     %% ===== Block 3: all-user unlicensed Hungarian + CBF-MM + KKT =====
%     % No QoS gating: every associated SU is eligible on both bands.
%     for u = 1:U
%         users_u = tx(u).users;
%         Pmax_un_u=Pmax_un;
%         if compareMode>=2 && compareMode<=4
%             % Estimate interference condition from the already-allocated licensed band.
%             % Reserve ONLY for UAVs contributing to an unsafe licensed SC.
%             licI=aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
%             unsafeLic=licI(:)>GammaLic(:)*(1+1e-9);
%             localIdx=tx(u).idxLic;
%             if ~isempty(localIdx) && any(unsafeLic(localIdx(:,2)))
%                 Pmax_un_u=Pmax_un*(1-0.1*(compareMode-1));
%             end
%         end
%         if isempty(users_u) || Pmax_un_u <= 1e-15
%             continue;
%         end
%         needUn_local = true(numel(users_u),1); % Every associated SU is eligible
%         HsuU_u = HsuU(u,users_u);
%         HwuU_u = HwuU(u,:);
%         hw2_u  = hw2(users_u);
%         extraNoiseU = previous_interuav_noise_matrix(...
%             txPrev,HsuU,u,users_u,M,'un');
%         rhoU = assign_unlicensed_vec_extra_noise(...
%             HsuU_u,HwuU_u,hw2_u,P_WIFI_m,sigma2_un,extraNoiseU,...
%             GammaUn,Pmax_un_u,Bsc_un,numel(users_u),M,needUn_local(:),...
%             'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,...
%             Pmax_un_u/max(M,1));
%         [tx(u).pUn,tx(u).idxUn,tx(u).vUn] = solve_band_mm_given_rho(...
%             rhoU,HsuU_u,HwuU_u,hw2_u,P_WIFI_m,sigma2_un,...
%             GammaUn,Pmax_un_u,MM_iter,MM_leakFrac,MM_reg_alpha);
%     end
%     %% ===== Block 4: measure both PRE-CONTROL aggregate interferences =====
%     % Aggregate across ALL C-UAVs on each corresponding protected channel.
%     % These are the raw initial allocations, before any aggregate clipping.
%     Ilic_initial = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic'); %#ok<NASGU>
%     Iun_initial  = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un'); %#ok<NASGU>
%     %% ===== Block 5: integrated AI-IBPC four-case protection =====
%     % Interference is measured AFTER association, subchannel allocation,
%     % CBF-MM beamforming and initial KKT power allocation.
%     %
%     % Case 1: licensed high, unlicensed safe -> rate-aware licensed reduction
%     %         plus rate-aware unlicensed increase using only unlicensed budget
%     %         and WU-interference headroom.
%     % Case 2: unlicensed high, licensed safe -> mirror of Case 1.
%     % Case 3: neither high -> keep the original power allocation unchanged.
%     % Case 4: both high -> rate-aware reductions on both bands until both
%     %         protection constraints are satisfied.
%     %
%     % Protection is part of AI-IBPC itself. There is NO separate final
%     % safety-clipping/protection block after this controller.
%     txRaw=tx;
%     rateFnLocal=@(z)diagnostic_tx_rate(z,HsuL,HsuU,hp2,hw2,...
%         P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);
%     [tx,caseStats]=ai_ibpc_integrated_four_case_rateaware(...
%         txRaw,HpuL,HwuU,GammaLic,GammaUn,...
%         Pmax_lic,Pmax_un,rateFnLocal);
%     diagnostics=struct('transferCount',0,'donorLossMbps',0);
%     if stageDebug
%         diagnostics.rawTx=txRaw;
%     end
%     % The controller processes each SU and updates shared-channel interference.
%     %% ===== Block 6: rate evaluation and AO acceptance =====
%     %% ===== Convergence check for fixed-Qslot AO =====
%     slotRateLic = compute_global_sum_rate_band_interuav(tx,HsuL,hp2,P_PBS_j,...
%         sigma2_lic,Bsc_lic,'lic');
%     slotRateUn  = compute_global_sum_rate_band_interuav(tx,HsuU,hw2,P_WIFI_m,...
%         sigma2_un,Bsc_un,'un');
%     slotRateNew = slotRateLic + slotRateUn;
%     % The preceding power-protection routines ensure per-band feasibility.
%     % Reject any candidate that fails an explicit feasibility check.
%     IlicNow = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
%     IunNow = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un');
%     candidateFeasible = all(IlicNow(:) <= GammaLic(:)*(1+1e-6)) && ...
%         all(IunNow(:) <= GammaUn(:)*(1+1e-6));
%     for uu = 1:U
%         candidateFeasible = candidateFeasible && ...
%             sum(tx(uu).pLic(:)) <= Pmax_lic*(1+1e-6) && ...
%             sum(tx(uu).pUn(:)) <= Pmax_un*(1+1e-6);
%     end
%     % Only accept a complete resource-allocation sweep, never assume that
%     % Hungarian, MM, or KKT individually guarantee global improvement.
%     % A rejected AO proposal is not a solver error. Hungarian, MM, and
%     % clipped KKT blocks do not jointly form a guaranteed ascent step.
%     % Retain the incumbent rather than accepting a lower-rate proposal.
%     % Terminating immediately avoids expensive, repeatedly rejected sweeps.
%     % Avoid printing on every finite-difference/channel-sample evaluation.
%     rateDecreased = isfinite(slotRateNew) && ...
%         slotRateNew < slotRateBest - 1e-10*max(1,abs(slotRateBest));
%     if ~candidateFeasible || ~isfinite(slotRateNew) || rateDecreased
%         aoRejectCount = aoRejectCount + 1;
%         if ~candidateFeasible
%             aoRejectInfeasible=aoRejectInfeasible+1;
%             cause='constraint violation';
%         elseif ~isfinite(slotRateNew)
%             aoRejectNonfinite=aoRejectNonfinite+1;
%             cause='nonfinite rate';
%         else
%             aoRejectRate=aoRejectRate+1;
%             cause='rate decrease';
%         end
%         % Show two representative failures and occasional aggregate statistics.
%         % This avoids flooding the console during finite-difference SCA.
%         if false % VTC-style concise output; safety checks still enforced
%             licRatio=max(IlicNow(:)./max(GammaLic(:),realmin));
%             unRatio=max(IunNow(:)./max(GammaUn(:),realmin));
%             licBudget=max(arrayfun(@(z)sum(z.pLic(:)),tx))/max(Pmax_lic,realmin);
%             unBudget=max(arrayfun(@(z)sum(z.pUn(:)),tx))/max(Pmax_un,realmin);
%             fprintf(['Inner AO stopped (%s): candidate %.6g Mbps, ' ...
%                 'incumbent %.6g Mbps; Ilic %.4g, Iun %.4g, ' ...
%                 'Plic %.4g, Pun %.4g [ratios; limit 1].\n'], ...
%                 cause,slotRateNew/1e6,slotRateBest/1e6, ...
%                 licRatio,unRatio,licBudget,unBudget);
%             if mod(aoRejectCount,500)==0
%                 fprintf('AO rejection totals: infeasible=%d, rate=%d, nonfinite=%d.\n', ...
%                     aoRejectInfeasible,aoRejectRate,aoRejectNonfinite);
%             end
%         end
%         if ~stageDebug,diagnostics=struct('transferCount',0,'donorLossMbps',0);end
%         break;
%     end
%     previousAcceptedRate = slotRateBest;
%     txBest = tx;
%     slotRateBest = max(slotRateBest,slotRateNew);
%     acceptedCount = acceptedCount + 1;
%     acceptedAOHistory(acceptedCount+1) = slotRateBest;
%     txPrev = txBest;
%     slotRateOld = slotRateBest;
%     if acceptedCount > 1 && ...
%             (slotRateBest-previousAcceptedRate)/max(1,abs(previousAcceptedRate)) <= AO_tol
%         break;
%     end
% end
% % Return best safe AO iterate,not necessarily the last one.
% % ------------------------------------------------------------------
% % Final verification checks after each slot. These assertions do not change
% % the algorithm. They only stop the simulation if the selected safe AO iterate
% % violates any aggregate interference or per-UAV power-budget constraint.
% Ilic_best = aggregate_interference_by_channel(txBest,HpuL,GammaLic,'lic');
% Iun_best  = aggregate_interference_by_channel(txBest,HwuU,GammaUn,'un');
% assert(all(Ilic_best(:) <= GammaLic(:).*(1+1e-6)),...
%     'Licensed aggregate interference violation after final protection.');
% assert(all(Iun_best(:) <= GammaUn(:).*(1+1e-6)),...
%     'Unlicensed aggregate interference violation after final protection.');
% for uu = 1:U
%     pLicSum = 0;
%     pUnSum  = 0;
%     if isfield(txBest(uu),'pLic') && ~isempty(txBest(uu).pLic)
%         pLicSum = sum(txBest(uu).pLic(:));
%     end
%     if isfield(txBest(uu),'pUn') && ~isempty(txBest(uu).pUn)
%         pUnSum = sum(txBest(uu).pUn(:));
%     end
%     assert(pLicSum <= Pmax_lic*(1+1e-6), ...
%         'Licensed AI-IBPC power-budget violation.');
%     assert(pUnSum <= Pmax_un*(1+1e-6), ...
%         'Unlicensed AI-IBPC power-budget violation.');
% end
% slotRateLic = compute_global_sum_rate_band_interuav(txBest,HsuL,hp2,P_PBS_j,...
%     sigma2_lic,Bsc_lic,'lic');
% slotRateUn  = compute_global_sum_rate_band_interuav(txBest,HsuU,hw2,P_WIFI_m,...
%     sigma2_un,Bsc_un,'un');
% slotRate = slotRateLic + slotRateUn;
% userRate=compute_global_user_rate_band_interuav(txBest,HsuL,hp2,P_PBS_j,sigma2_lic,Bsc_lic,Kglob,'lic')+ ...
%          compute_global_user_rate_band_interuav(txBest,HsuU,hw2,P_WIFI_m,sigma2_un,Bsc_un,Kglob,'un');
% % Actual aggregate post-protection leakage, available for trajectory SCA.
% leakLic = aggregate_interference_by_channel(txBest,HpuL,GammaLic,'lic');
% leakUn  = aggregate_interference_by_channel(txBest,HwuU,GammaUn,'un');
% end
% function extraNoise = previous_interuav_noise_matrix(txPrev,HsuFull,servingUAV,users_u,S,bandName)
% % Previous-iterate inter-UAV co-channel interference used in the AO
% % assignment metric. Size: (#local users) x (#subchannels).
% extraNoise = zeros(numel(users_u),S);
% if isempty(txPrev)
%     return;
% end
% for kk = 1:numel(users_u)
%     kglob = users_u(kk);
%     for s = 1:S
%         extraNoise(kk,s) = inter_uav_power_same_sc(txPrev,HsuFull,servingUAV,kglob,s,bandName);
%     end
% end
% end
% function rho = assign_licensed_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
% % Licensed assignment with previous-AO inter-UAV interference in the metric.
% Rmat = zeros(K,S);
% if isempty(extraNoise)
%     extraNoise = zeros(K,S);
% end
% for k = 1:K
%     for s = 1:S
%         denom = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
%         if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
%             v = mrt_unit(Hsu{k}); % Initial allocation: no protection
%         else
%             v = mrt_unit(Hsu{k});
%         end
%         gSig  = abs(Hsu{k}'*v)^2;
%         gLeak = abs(Hprot{s}'*v)^2;
%         p_app = Pmax/max(S,1); % No pre-control interference cap
%         Rmat(k,s) = Bsc*log2(1 + max(p_app*gSig/denom,0));
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairs = assign_greedy_max(Rmat);
% else
%     pairs = assign_hungarian_max(Rmat);
% end
% rho = zeros(K,S);
% for ii = 1:size(pairs,1)
%     rho(pairs(ii,1),pairs(ii,2)) = 1;
% end
% end
% function rho = assign_unlicensed_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
% % Unlicensed assignment with previous-AO inter-UAV interference in the metric.
% rho = zeros(K,S);
% activeUsers = find(needUn(:));
% if isempty(activeUsers) || Pmax <= 0
%     return;
% end
% if isempty(extraNoise)
%     extraNoise = zeros(K,S);
% end
% Ka = numel(activeUsers);
% Rmat = zeros(Ka,S);
% for aa = 1:Ka
%     k = activeUsers(aa);
%     for s = 1:S
%         denom = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
%         if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
%             v = mrt_unit(Hsu{k}); % Initial allocation: no protection
%         else
%             v = mrt_unit(Hsu{k});
%         end
%         gSig  = abs(Hsu{k}'*v)^2;
%         gLeak = abs(Hprot{s}'*v)^2;
%         p_app = Pmax/max(S,1); % No pre-control interference cap
%         Rmat(aa,s) = Bsc*log2(1 + max(p_app*gSig/denom,0));
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairsLocal = assign_greedy_max(Rmat);
% else
%     pairsLocal = assign_hungarian_max(Rmat);
% end
% for ii = 1:size(pairsLocal,1)
%     aa = pairsLocal(ii,1);
%     s  = pairsLocal(ii,2);
%     rho(activeUsers(aa),s) = 1;
% end
% end
% function rho = assign_licensed_iac_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,alphaFallback,pRef)
% % IAC licensed assignment with previous-AO inter-UAV interference.
% Rmat = zeros(K,S);
% if isempty(extraNoise)
%     extraNoise = zeros(K,S);
% end
% lambdaLeak = 1.00;
% lambdaRx   = 0.10;
% lambdaIuav = 0.25;
% for k = 1:K
%     for s = 1:S
%         denomRx = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
%         if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
%             v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
%         else
%             v = mrt_unit(Hsu{k});
%         end
%         gSig  = abs(Hsu{k}'*v)^2;
%         gLeak = abs(Hprot{s}'*v)^2;
%         p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
%         rateEst = Bsc*log2(1 + max(p_app*gSig/denomRx,0));
%         leakRatio = (p_app*gLeak) / max(GammaVec(s),1e-30);
%         rxRatio   = (hI(k)*P_I_vec(s)) / max(sigma2,1e-30);
%         iUavRatio = extraNoise(k,s) / max(sigma2,1e-30);
%         iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio) + lambdaIuav*log1p(iUavRatio);
%         Rmat(k,s) = rateEst / iacPenalty;
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairs = assign_greedy_max(Rmat);
% else
%     pairs = assign_hungarian_max(Rmat);
% end
% rho = zeros(K,S);
% for ii = 1:size(pairs,1)
%     rho(pairs(ii,1),pairs(ii,2)) = 1;
% end
% end
% function rho = assign_unlicensed_iac_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,alphaFallback,pRef)
% % IAC unlicensed assignment with previous-AO inter-UAV interference.
% rho = zeros(K,S);
% activeUsers = find(needUn(:));
% if isempty(activeUsers) || Pmax <= 0
%     return;
% end
% if isempty(extraNoise)
%     extraNoise = zeros(K,S);
% end
% Ka = numel(activeUsers);
% Rmat = zeros(Ka,S);
% lambdaLeak = 1.00;
% lambdaRx   = 0.10;
% lambdaIuav = 0.25;
% for aa = 1:Ka
%     k = activeUsers(aa);
%     for s = 1:S
%         denomRx = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
%         if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
%             v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
%         else
%             v = mrt_unit(Hsu{k});
%         end
%         gSig  = abs(Hsu{k}'*v)^2;
%         gLeak = abs(Hprot{s}'*v)^2;
%         p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
%         rateEst = Bsc*log2(1 + max(p_app*gSig/denomRx,0));
%         leakRatio = (p_app*gLeak) / max(GammaVec(s),1e-30);
%         rxRatio   = (hI(k)*P_I_vec(s)) / max(sigma2,1e-30);
%         iUavRatio = extraNoise(k,s) / max(sigma2,1e-30);
%         iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio) + lambdaIuav*log1p(iUavRatio);
%         Rmat(aa,s) = rateEst / iacPenalty;
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairsLocal = assign_greedy_max(Rmat);
% else
%     pairsLocal = assign_hungarian_max(Rmat);
% end
% for ii = 1:size(pairsLocal,1)
%     aa = pairsLocal(ii,1);
%     s  = pairsLocal(ii,2);
%     rho(activeUsers(aa),s) = 1;
% end
% end
% function Ruser = compute_global_user_rate_band_interuav(tx,HsuFull,hI,P_I_vec,sigma2,Bsc,Kglob,bandName)
% % Returns per-global-user rates for one band including inter-UAV co-channel interference.
% Ruser = zeros(Kglob,1);
% U = numel(tx);
% for u = 1:U
%     if strcmpi(bandName,'lic')
%         idx = tx(u).idxLic; p = tx(u).pLic; vList = tx(u).vLic;
%     else
%         idx = tx(u).idxUn;  p = tx(u).pUn;  vList = tx(u).vUn;
%     end
%     users_u = tx(u).users;
%     for ell = 1:size(idx,1)
%         local_k = idx(ell,1);
%         sc      = idx(ell,2);
%         kglob   = users_u(local_k);
%         v       = vList{ell};
%         signal = p(ell) * abs(HsuFull{u,kglob}' * v)^2;
%         denom  = hI(kglob)*P_I_vec(sc) + sigma2;
%         denom  = denom + inter_uav_power_same_sc(tx,HsuFull,u,kglob,sc,bandName);
%         denom  = max(denom,1e-30);
%         SINR = signal/denom;
%         Ruser(kglob) = Ruser(kglob) + Bsc*log2(1 + max(SINR,0));
%     end
% end
% end
% function sumRate = compute_global_sum_rate_band_interuav(tx,HsuFull,hI,P_I_vec,sigma2,Bsc,bandName)
% % Sum-rate for one band including inter-UAV co-channel interference.
% sumRate = 0;
% U = numel(tx);
% for u = 1:U
%     if strcmpi(bandName,'lic')
%         idx = tx(u).idxLic; p = tx(u).pLic; vList = tx(u).vLic;
%     else
%         idx = tx(u).idxUn;  p = tx(u).pUn;  vList = tx(u).vUn;
%     end
%     users_u = tx(u).users;
%     for ell = 1:size(idx,1)
%         local_k = idx(ell,1);
%         sc      = idx(ell,2);
%         kglob   = users_u(local_k);
%         v       = vList{ell};
%         signal = p(ell) * abs(HsuFull{u,kglob}' * v)^2;
%         denom  = hI(kglob)*P_I_vec(sc) + sigma2;
%         denom  = denom + inter_uav_power_same_sc(tx,HsuFull,u,kglob,sc,bandName);
%         denom  = max(denom,1e-30);
%         SINR = signal/denom;
%         sumRate = sumRate + Bsc*log2(1 + max(SINR,0));
%     end
% end
% end
% function Iuav = inter_uav_power_same_sc(tx,HsuFull,servingUAV,victimUserGlobal,sc,bandName)
% % Interference at victimUserGlobal caused by all other UAVs using the same subchannel.
% Iuav = 0;
% U = numel(tx);
% for u2 = 1:U
%     if u2 == servingUAV
%         continue;
%     end
%     if strcmpi(bandName,'lic')
%         idx2 = tx(u2).idxLic; p2 = tx(u2).pLic; vList2 = tx(u2).vLic;
%     else
%         idx2 = tx(u2).idxUn;  p2 = tx(u2).pUn;  vList2 = tx(u2).vUn;
%     end
%     for ell2 = 1:size(idx2,1)
%         sc2 = idx2(ell2,2);
%         if sc2 == sc
%             v2 = vList2{ell2};
%             hInterf = HsuFull{u2,victimUserGlobal};
%             Iuav = Iuav + p2(ell2) * abs(hInterf' * v2)^2;
%         end
%     end
% end
% end
% function tx = apply_final_safety_clipping(tx,HpuL,HwuU,GammaLic,GammaUn,PmaxLic,PmaxUn,whichBands)
% % Lightweight final protection for the IAC curve.
% % ------------------------------------------------------------
% % Corrected for multi-UAV aggregate per-slot protection.
% % This is not a 4-case controller. It only projects the active powers so that
% % the aggregate interference from ALL UAVs to each protected PU/WU channel is
% % below the corresponding threshold:
% %
% %   sum_u I_lic(u,j,n) <= GammaLic(j),for every licensed channel j
% %   sum_u I_un (u,m,n) <= GammaUn (m),for every unlicensed channel m
% %
% % It also preserves each UAV's own per-band transmit-power budget.
% if strcmpi(whichBands,'lic') || strcmpi(whichBands,'both')
%     tx = project_aggregate_band_interference(tx,HpuL,GammaLic,PmaxLic,'lic');
% end
% if strcmpi(whichBands,'un') || strcmpi(whichBands,'both')
%     tx = project_aggregate_band_interference(tx,HwuU,GammaUn,PmaxUn,'un');
% end
% end
% function [tx,caseStats] = apply_perband_4case_power_control(tx,HpuL,HwuU,GammaLic,GammaUn,PmaxLic,PmaxUn,enableCompensation,detailedDebug)
% if nargin < 9, detailedDebug = false; end
% % USER-WISE AI-IBPC, performed only AFTER both bands have been allocated.
% % Every associated SU is examined using the aggregate PU/WU interference on
% % ITS selected licensed/unlicensed subchannels (all UAVs sharing each SC).
% % Compensation adds power to the SAME SU's opposite-band stream only, from
% % that opposite band's OWN remaining UAV budget and protected headroom.
% % There is no cross-band power transfer. SU order is deterministic.
% % Sequential changes update aggregate leakage before the next SU is checked.
% % The final projection protects shared channels after all user adjustments.
% caseStats = struct('case1_licHighOnly',0,'case2_unHighOnly',0,...
%     'case3_bothHigh',0,'case4_normal',0,'licensedReduced_W',0,...
%     'unlicensedReduced_W',0,'licensedCompensated_W',0,...
%     'unlicensedCompensated_W',0,'usersProcessed',0);
% tx = enforce_per_uav_band_budget(tx,PmaxLic,'lic');
% tx = enforce_per_uav_band_budget(tx,PmaxUn,'un');
% for u = 1:numel(tx)
%     for loc = 1:numel(tx(u).users)
%         [idxL,pL,~] = get_band_fields(tx,u,'lic');
%         [idxU,pU,~] = get_band_fields(tx,u,'un');
%         eL = find(idxL(:,1)==loc,1);
%         eU = find(idxU(:,1)==loc,1);
%         if isempty(eL) && isempty(eU), continue; end
%         caseStats.usersProcessed = caseStats.usersProcessed+1;
%         IL = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
%         IU = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un');
%         highL = ~isempty(eL) && IL(idxL(eL,2)) > GammaLic(idxL(eL,2))*(1+1e-9);
%         highU = ~isempty(eU) && IU(idxU(eU,2)) > GammaUn(idxU(eU,2))*(1+1e-9);
%         if false && detailedDebug
%             suGlobal = tx(u).users(loc);
%             fprintf('\nSU %d | UAV %d | L stream %d | U stream %d | case Lhigh=%d Uhigh=%d\n', ...
%                 suGlobal,u,~isempty(eL),~isempty(eU),highL,highU);
%             fprintf('  UAV budget: L used %.8g / %.8g W; U used %.8g / %.8g W\n', ...
%                 sum(tx(u).pLic(:)),PmaxLic,sum(tx(u).pUn(:)),PmaxUn);
%             if ~isempty(eL)
%                 fprintf('  Selected licensed SC %d: aggregate ratio %.6g\n', ...
%                     idxL(eL,2),IL(idxL(eL,2))/GammaLic(idxL(eL,2)));
%             end
%             if ~isempty(eU)
%                 fprintf('  Selected unlicensed SC %d: aggregate ratio %.6g\n', ...
%                     idxU(eU,2),IU(idxU(eU,2))/GammaUn(idxU(eU,2)));
%             end
%         end
%         if highL && ~highU
%             caseStats.case1_licHighOnly = caseStats.case1_licHighOnly+1;
%             [tx,dp] = reduce_su_stream_to_aggregate_limit(tx,u,eL,HpuL,GammaLic,'lic');
%             caseStats.licensedReduced_W = caseStats.licensedReduced_W+dp;
%             if enableCompensation && ~isempty(eU)
%                 if false && detailedDebug
%                     [ii,pp,vv] = get_band_fields(tx,u,'un');
%                     ss = ii(eU,2);
%                     allI = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un');
%                     gg = abs(HwuU{u,ss}'*vv{eU})^2;
%                     budgetLeft = max(0,PmaxUn-sum(pp(:)));
%                     headLeft = max(0,GammaUn(ss)-allI(ss));
%                     if gg > 1e-30
%                         interferencePowerLeft = headLeft/gg;
%                     else
%                         interferencePowerLeft = Inf;
%                     end
%                     fprintf('  -> COMPENSATE UN: pre-power %.9g W | UAV budget left %.9g W | WU/PU headroom %.9g W | equivalent power headroom %.9g W\n', ...
%                         pp(eU),budgetLeft,headLeft,interferencePowerLeft);
%                 end
%                 [tx,dpU] = increase_su_stream_if_safe(tx,u,eU,HwuU,GammaUn,PmaxUn,'un');
%                 if false && detailedDebug
%                     fprintf('  -> ACTUAL UN COMPENSATION: %.9g W\n',dpU);
%                     if dpU <= 1e-12
%                         fprintf('  -> BLOCKED: inspect zero UAV budget or zero aggregate interference headroom above.\n');
%                     end
%                 end
%                 caseStats.unlicensedCompensated_W = caseStats.unlicensedCompensated_W+dpU;
%             end
%         elseif ~highL && highU
%             caseStats.case2_unHighOnly = caseStats.case2_unHighOnly+1;
%             [tx,dp] = reduce_su_stream_to_aggregate_limit(tx,u,eU,HwuU,GammaUn,'un');
%             caseStats.unlicensedReduced_W = caseStats.unlicensedReduced_W+dp;
%             if enableCompensation && ~isempty(eL)
%                 if false && detailedDebug
%                     [ii,pp,vv] = get_band_fields(tx,u,'lic');
%                     ss = ii(eL,2);
%                     allI = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
%                     gg = abs(HpuL{u,ss}'*vv{eL})^2;
%                     budgetLeft = max(0,PmaxLic-sum(pp(:)));
%                     headLeft = max(0,GammaLic(ss)-allI(ss));
%                     if gg > 1e-30
%                         interferencePowerLeft = headLeft/gg;
%                     else
%                         interferencePowerLeft = Inf;
%                     end
%                     fprintf('  -> COMPENSATE LIC: pre-power %.9g W | UAV budget left %.9g W | WU/PU headroom %.9g W | equivalent power headroom %.9g W\n', ...
%                         pp(eL),budgetLeft,headLeft,interferencePowerLeft);
%                 end
%                 [tx,dpL] = increase_su_stream_if_safe(tx,u,eL,HpuL,GammaLic,PmaxLic,'lic');
%                 if false && detailedDebug
%                     fprintf('  -> ACTUAL LIC COMPENSATION: %.9g W\n',dpL);
%                     if dpL <= 1e-12
%                         fprintf('  -> BLOCKED: inspect zero UAV budget or zero aggregate interference headroom above.\n');
%                     end
%                 end
%                 caseStats.licensedCompensated_W = caseStats.licensedCompensated_W+dpL;
%             end
%         elseif highL && highU
%             caseStats.case3_bothHigh = caseStats.case3_bothHigh+1;
%             [tx,dpL] = reduce_su_stream_to_aggregate_limit(tx,u,eL,HpuL,GammaLic,'lic');
%             [tx,dpU] = reduce_su_stream_to_aggregate_limit(tx,u,eU,HwuU,GammaUn,'un');
%             caseStats.licensedReduced_W = caseStats.licensedReduced_W+dpL;
%             caseStats.unlicensedReduced_W = caseStats.unlicensedReduced_W+dpU;
%         else
%             caseStats.case4_normal = caseStats.case4_normal+1;
%         end
%     end
% end
% % Aggregate multi-UAV safety guard; another UAV can use the same SC.
% tx = project_aggregate_band_interference(tx,HpuL,GammaLic,PmaxLic,'lic');
% tx = project_aggregate_band_interference(tx,HwuU,GammaUn,PmaxUn,'un');
% end
% 
% function [tx,reduced] = reduce_su_stream_to_aggregate_limit(tx,u,ell,Hprot,Gamma,band)
% [idx,p,v] = get_band_fields(tx,u,band);
% s = idx(ell,2); g = abs(Hprot{u,s}'*v{ell})^2;
% I = aggregate_interference_by_channel(tx,Hprot,Gamma,band);
% % Hold all OTHER streams fixed. The SU can only remove its own contribution.
% otherI = max(0,I(s)-p(ell)*g);
% newP = min(p(ell),max(0,(Gamma(s)-otherI)/max(g,1e-30)));
% reduced = max(0,p(ell)-newP);
% p(ell)=newP;
% tx=set_band_power(tx,u,band,p);
% end
% 
% function [tx,added] = increase_su_stream_if_safe(tx,u,ell,Hprot,Gamma,Pmax,band)
% [idx,p,v] = get_band_fields(tx,u,band);
% s=idx(ell,2);
% I=aggregate_interference_by_channel(tx,Hprot,Gamma,band);
% g=abs(Hprot{u,s}'*v{ell})^2;
% unusedBudget=max(0,Pmax-sum(p));
% headroom=max(0,Gamma(s)-I(s));
% % An already unsafe shared channel cannot receive compensating power.
% if I(s)>Gamma(s)*(1+1e-9), added=0; return; end
% if g<=1e-30
%     added=unusedBudget;
% else
%     added=min(unusedBudget,headroom/max(g,1e-30));
% end
% added=max(0,added);
% p(ell)=p(ell)+added;
% tx=set_band_power(tx,u,band,p);
% end
% 
% function tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName)
% % Project one band so aggregate interference per protected channel is safe.
% %
% % For each protected channel s:
% %   I_s = sum_{all UAV streams on s} p_stream * |h_prot^H v_stream|^2
% %
% % If I_s > GammaVec(s),all active stream powers on that channel are scaled
% % by GammaVec(s)/I_s. This preserves the relative power split among active
% % UAVs while guaranteeing aggregate per-slot protection.
% tol = 1e-12;
% GammaVec = GammaVec(:);
% S = numel(GammaVec);
% tx = enforce_per_uav_band_budget(tx,PmaxBand,bandName);
% I = aggregate_interference_by_channel(tx,Hprot,GammaVec,bandName);
% for s = 1:S
%     if I(s) > GammaVec(s)*(1+tol)
%         scale = GammaVec(s) / max(I(s),1e-30);
%         for u = 1:numel(tx)
%             [idx,p,vList] = get_band_fields(tx,u,bandName); %#ok<ASGLU>
%             if isempty(idx),continue; end
%             onS = (idx(:,2) == s);
%             if any(onS)
%                 p(onS) = p(onS) * scale;
%                 tx = set_band_power(tx,u,bandName,p);
%             end
%         end
%     end
% end
% tx = enforce_per_uav_band_budget(tx,PmaxBand,bandName);
% end
% function tx = increase_from_aggregate_band_headroom(tx,Hprot,GammaVec,PmaxBand,bandName)
% % Increase powers using ONLY the same band's unused power budget and aggregate
% % interference headroom.
% %
% % This function does NOT receive clipped power from the other band. Therefore,% it is per-band control,not common-total power sharing and not cross-band
% % transfer.
% if PmaxBand <= 0
%     return;
% end
% GammaVec = GammaVec(:);
% S = numel(GammaVec);
% % Ensure current point is safe before trying to increase.
% tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName);
% for iter = 1:20
%     I = aggregate_interference_by_channel(tx,Hprot,GammaVec,bandName);
%     changed = false;
%     for s = 1:S
%         interfHeadroom = GammaVec(s) - I(s);
%         if interfHeadroom <= 1e-15
%             continue;
%         end
%         % Collect active streams on channel s.
%         entries = [];
%         for u = 1:numel(tx)
%             [idx,p,vList] = get_band_fields(tx,u,bandName);
%             if isempty(idx),continue; end
%             for ell = 1:size(idx,1)
%                 if idx(ell,2) == s
%                     v = vList{ell};
%                     gLeak = abs(Hprot{u,s}' * v)^2;
%                     uBudget = max(PmaxBand - sum(p),0);
%                     if gLeak > 1e-30 && uBudget > 1e-15
%                         % columns: u,ell,gLeak,uBudget
%                         entries = [entries; u,ell,gLeak,uBudget]; %#ok<AGROW>
%                     end
%                 end
%             end
%         end
%         if isempty(entries)
%             continue;
%         end
%         % Distribute the aggregate interference headroom across active streams.
%         % Equal interference-share is used to avoid biasing one UAV only.
%         nE = size(entries,1);
%         shareI = interfHeadroom / nE;
%         for ee = 1:nE
%             u     = entries(ee,1);
%             ell   = entries(ee,2);
%             gLeak = entries(ee,3);
%             [~,p,~] = get_band_fields(tx,u,bandName);
%             uBudget = max(PmaxBand - sum(p),0);
%             if uBudget <= 1e-15
%                 continue;
%             end
%             dp = min(shareI / max(gLeak,1e-30),uBudget);
%             if dp > 1e-15
%                 p(ell) = p(ell) + dp;
%                 tx = set_band_power(tx,u,bandName,p);
%                 changed = true;
%             end
%         end
%         % Re-project immediately to remove any small aggregate overshoot.
%         tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName);
%     end
%     if ~changed
%         break;
%     end
% end
% tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName);
% end
% function I = aggregate_interference_by_channel(tx,Hprot,GammaVec,bandName)
% % Aggregate leakage from all UAVs to each protected receiver/channel.
% S = numel(GammaVec);
% I = zeros(S,1);
% for u = 1:numel(tx)
%     [idx,p,vList] = get_band_fields(tx,u,bandName);
%     if isempty(idx),continue; end
%     for ell = 1:size(idx,1)
%         sc = idx(ell,2);
%         if sc < 1 || sc > S
%             continue;
%         end
%         v = vList{ell};
%         gLeak = abs(Hprot{u,sc}' * v)^2;
%         I(sc) = I(sc) + p(ell) * gLeak;
%     end
% end
% end
% function tx = enforce_per_uav_band_budget(tx,PmaxBand,bandName)
% % Enforce each UAV's own per-band power budget.
% for u = 1:numel(tx)
%     [idx,p,~] = get_band_fields(tx,u,bandName); %#ok<ASGLU>
%     if isempty(p),continue; end
%     p = max(p(:),0);
%     if sum(p) > PmaxBand
%         p = p * (PmaxBand / max(sum(p),1e-30));
%     end
%     tx = set_band_power(tx,u,bandName,p);
% end
% end
% function Ptot = total_band_power(tx,bandName)
% Ptot = 0;
% for u = 1:numel(tx)
%     [~,p,~] = get_band_fields(tx,u,bandName);
%     if ~isempty(p)
%         Ptot = Ptot + sum(p);
%     end
% end
% end
% function [idx,p,vList] = get_band_fields(tx,u,bandName)
% if strcmpi(bandName,'lic')
%     idx = tx(u).idxLic;
%     p = tx(u).pLic(:);
%     vList = tx(u).vLic;
% else
%     idx = tx(u).idxUn;
%     p = tx(u).pUn(:);
%     vList = tx(u).vUn;
% end
% end
% function tx = set_band_power(tx,u,bandName,p)
% if strcmpi(bandName,'lic')
%     tx(u).pLic = p(:);
% else
%     tx(u).pUn = p(:);
% end
% end
% function [cap,leakage] = leakage_caps_vec(tx,u,Hprot,GammaVec,bandName)
% % Backward-compatible helper.
% % In the corrected aggregate version this function is not used for final
% % protection,because final protection must consider all UAVs together.
% % It is kept only in case another helper calls it.
% if strcmpi(bandName,'lic')
%     idx = tx(u).idxLic;
%     p   = tx(u).pLic(:);
%     vList = tx(u).vLic;
% else
%     idx = tx(u).idxUn;
%     p   = tx(u).pUn(:);
%     vList = tx(u).vUn;
% end
% L = size(idx,1);
% cap = zeros(L,1);
% leakage = zeros(L,1);
% for ell = 1:L
%     sc = idx(ell,2);
%     v  = vList{ell};
%     gLeak = abs(Hprot{u,sc}' * v)^2;
%     cap(ell) = max(GammaVec(sc) / max(gLeak,1e-30),0);
%     leakage(ell) = p(ell) * gLeak;
% end
% end
% function [pNew,delta,nClip] = clip_power_to_cap(pOld,cap)
% % Clip powers to their interference-safe caps.
% pOld = pOld(:);
% cap = cap(:);
% if isempty(pOld)
%     pNew = pOld; delta = 0; nClip = 0; return;
% end
% pNew = min(pOld,cap);
% pNew = max(pNew,0);
% diff = max(pOld - pNew,0);
% delta = sum(diff);
% nClip = nnz(diff > 1e-12);
% end
% function [pOut,addedTotal] = increase_from_own_band_headroom(pIn,cap,PmaxBand)
% % Increase power using ONLY the same band's own unused budget and safe caps.
% % No clipped power from the other band is used as an input.
% %
% % pOut satisfies:
% %   sum(pOut) <= PmaxBand
% %   pOut(ell) <= cap(ell)
% %   pOut(ell) >= pIn(ell)
% pIn = pIn(:);
% cap = cap(:);
% pOut = pIn;
% addedTotal = 0;
% if isempty(pIn) || PmaxBand <= 0
%     return;
% end
% % First ensure the input itself does not violate caps or budget.
% pOut = min(max(pOut,0),cap);
% if sum(pOut) > PmaxBand
%     pOut = pOut * (PmaxBand/max(sum(pOut),1e-30));
%     pOut = min(pOut,cap);
% end
% safeHeadroom   = max(cap - pOut,0);
% budgetHeadroom = max(PmaxBand - sum(pOut),0);
% addBudget      = min(budgetHeadroom,sum(safeHeadroom));
% if addBudget <= 1e-15
%     return;
% end
% % Distribute own-band remaining power over streams with safe headroom.
% remaining = addBudget;
% for iter = 1:20
%     head = max(cap - pOut,0);
%     active = head > 1e-12;
%     if ~any(active) || remaining <= 1e-12
%         break;
%     end
%     add = zeros(size(pOut));
%     add(active) = remaining * head(active) / sum(head(active));
%     pTrial = min(pOut + add,cap);
%     actual = sum(pTrial - pOut);
%     pOut = pTrial;
%     remaining = remaining - actual;
%     if actual <= 1e-12
%         break;
%     end
% end
% % Final numerical guards.
% if sum(pOut) > PmaxBand
%     pOut = pOut * (PmaxBand/max(sum(pOut),1e-30));
% end
% pOut = min(pOut,cap);
% addedTotal = sum(pOut - pIn);
% end
% function assoc = associate_users_highest_channel_gain(HsuL)
% % Highest licensed-band channel-gain association.
% % For each SU k, select the C-UAV u maximizing ||h_{u,k}^{lic}||^2.
% % Only the association rule is changed; all downstream Hungarian, CBF-MM,
% % KKT, AI-IBPC/IA-RSS control, and trajectory-SCA logic remain unchanged.
% [U,K] = size(HsuL);
% metric = -inf(U,K);
% for u = 1:U
%     for k = 1:K
%         hL = HsuL{u,k};
%         metric(u,k) = real(hL' * hL);
%     end
% end
% [~,assoc] = max(metric,[],1);
% assoc = assoc(:);
% end
% function Q = project_multi_trajectory(Q,area,stepMax,q_start_all,q_end_all,projIter)
% U = size(Q,1);
% for u = 1:U
%     Qu = squeeze(Q(u,:,:));
%     Qu = project_full_trajectory(Qu,area,stepMax,q_start_all(u,:),q_end_all(u,:),projIter);
%     Q(u,:,:) = Qu;
% end
% end
% function v = unit_vec(x)
% nx = norm(x);
% if nx < 1e-12
%     v = [0,0];
% else
%     v = x/nx;
% end
% end
% %% ============================================================
% %                  VECTOR CHANNEL GENERATION,Nt = 8
% %% ============================================================
% function [HsuL,HsuU,HpuL,HwuU,hp2,hw2,hu2L_all] = gen_multi_channels_vec(...
%     Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
%     A2G,n_pbs,n_wifi,Shadow,seedNow,useRayleigh,useShadow,useShadowA2G,Nt)
% U = size(Qslot,1);
% K = size(WS,1);
% J = size(WP,1);
% M = size(WW,1);
% HsuL = cell(U,K);
% HsuU = cell(U,K);
% HpuL = cell(U,J);
% HwuU = cell(U,M);
% hu2L_all = zeros(U,K);
% rng(seedNow + 555,'twister');
% [hp2,hw2] = gen_terrestrial_interference_channels(WS,w_b,w_wifi,f_pbs,f_wifi,c,...
%     n_pbs,n_wifi,Shadow,useRayleigh,useShadow);
% for u = 1:U
%     rng(seedNow + 10000*u,'twister');
%     q = Qslot(u,:);
%     for k = 1:K
%         gL = a2g_gain_alhourani(q,WS(k,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
%         gU = a2g_gain_alhourani(q,WS(k,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
%         HsuL{u,k} = gen_miso_vector(gL,Nt,useRayleigh);
%         HsuU{u,k} = gen_miso_vector(gU,Nt,useRayleigh);
%         hu2L_all(u,k) = norm(HsuL{u,k})^2;
%     end
%     for j = 1:J
%         g = a2g_gain_alhourani(q,WP(j,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_pu_z(j));
%         HpuL{u,j} = gen_miso_vector(g,Nt,useRayleigh);
%     end
%     for m = 1:M
%         g = a2g_gain_alhourani(q,WW(m,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_wu_z(m));
%         HwuU{u,m} = gen_miso_vector(g,Nt,useRayleigh);
%     end
% end
% end
% function h = gen_miso_vector(gLarge,Nt,useRayleigh)
% % MISO channel vector Nt x 1. The expected norm^2 is approximately Nt*gLarge.
% % This gives array gain to both MRT and MM while retaining spatial DoF.
% if useRayleigh
%     h = sqrt(max(gLarge,0)) * (randn(Nt,1) + 1j*randn(Nt,1))/sqrt(2);
% else
%     h = sqrt(max(gLarge,0)) * ones(Nt,1);
% end
% end
% function [hu2L_all,hu2U_all,hp2,hw2,h_up2_all,h_uw2_all] = gen_multi_channels_avg_scalar(...
%     Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
%     A2G,n_pbs,n_wifi,Shadow,Nsamp,seed,useRayleigh,useShadow,useShadowA2G)
% U = size(Qslot,1);
% K = size(WS,1);
% J = size(WP,1);
% M = size(WW,1);
% hu2L_all  = zeros(U,K);
% hu2U_all  = zeros(U,K);
% hp2       = zeros(K,1);
% hw2       = zeros(K,1);
% h_up2_all = zeros(U,J);
% h_uw2_all = zeros(U,M);
% for s = 1:Nsamp
%     [a,b,c1,d1,e,f] = gen_multi_channels_scalar(Qslot,WS,WP,WW,w_b,w_wifi,H,...
%         f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
%         seed + s,useRayleigh,useShadow,useShadowA2G);
%     hu2L_all  = hu2L_all  + a;
%     hu2U_all  = hu2U_all  + b;
%     hp2       = hp2       + c1;
%     hw2       = hw2       + d1;
%     h_up2_all = h_up2_all + e;
%     h_uw2_all = h_uw2_all + f;
% end
% hu2L_all  = hu2L_all  / Nsamp;
% hu2U_all  = hu2U_all  / Nsamp;
% hp2       = hp2       / Nsamp;
% hw2       = hw2       / Nsamp;
% h_up2_all = h_up2_all / Nsamp;
% h_uw2_all = h_uw2_all / Nsamp;
% end
% function [hu2L_all,hu2U_all,hp2,hw2,h_up2_all,h_uw2_all] = gen_multi_channels_scalar(...
%     Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
%     A2G,n_pbs,n_wifi,Shadow,seedNow,useRayleigh,useShadow,useShadowA2G)
% U = size(Qslot,1);
% K = size(WS,1);
% J = size(WP,1);
% M = size(WW,1);
% hu2L_all  = zeros(U,K);
% hu2U_all  = zeros(U,K);
% h_up2_all = zeros(U,J);
% h_uw2_all = zeros(U,M);
% rng(seedNow + 555,'twister');
% [hp2,hw2] = gen_terrestrial_interference_channels(WS,w_b,w_wifi,f_pbs,f_wifi,c,...
%     n_pbs,n_wifi,Shadow,useRayleigh,useShadow);
% for u = 1:U
%     rng(seedNow + 10000*u,'twister');
%     q = Qslot(u,:);
%     [hu2L,hu2U,h_up2,h_uw2] = gen_uav_channels_only_scalar(...
%         q,WS,WP,WW,H,f_lic,f_un,c,A2G,Shadow,useRayleigh,useShadowA2G);
%     hu2L_all(u,:)  = hu2L(:).';
%     hu2U_all(u,:)  = hu2U(:).';
%     h_up2_all(u,:) = h_up2(:).';
%     h_uw2_all(u,:) = h_uw2(:).';
% end
% end
% function [hu2_lic,hu2_un,h_up2,h_uw2] = gen_uav_channels_only_scalar(...
%     q,WS,WP,WW,H,f_lic,f_un,c,A2G,Shadow,useRayleigh,useShadowA2G)
% K = size(WS,1);
% J = size(WP,1);
% M = size(WW,1);
% hu2_lic = zeros(K,1);
% hu2_un  = zeros(K,1);
% for k=1:K
%     gL = a2g_gain_alhourani(q,WS(k,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
%     gU = a2g_gain_alhourani(q,WS(k,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
%     if useRayleigh
%         z1 = (randn + 1j*randn)/sqrt(2);
%         z2 = (randn + 1j*randn)/sqrt(2);
%         hu2_lic(k) = gL * abs(z1)^2;
%         hu2_un(k)  = gU * abs(z2)^2;
%     else
%         hu2_lic(k) = gL;
%         hu2_un(k)  = gU;
%     end
% end
% h_up2 = zeros(J,1);
% for j=1:J
%     g = a2g_gain_alhourani(q,WP(j,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_pu_z(j));
%     if useRayleigh
%         z = (randn + 1j*randn)/sqrt(2);
%         h_up2(j) = g * abs(z)^2;
%     else
%         h_up2(j) = g;
%     end
% end
% h_uw2 = zeros(M,1);
% for m=1:M
%     g = a2g_gain_alhourani(q,WW(m,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_wu_z(m));
%     if useRayleigh
%         z = (randn + 1j*randn)/sqrt(2);
%         h_uw2(m) = g * abs(z)^2;
%     else
%         h_uw2(m) = g;
%     end
% end
% end
% function [hp2,hw2] = gen_terrestrial_interference_channels(WS,w_b,w_wifi,f_pbs,f_wifi,c,...
%     n_pbs,n_wifi,Shadow,useRayleigh,useShadow)
% K = size(WS,1);
% hp2 = zeros(K,1);
% hw2 = zeros(K,1);
% PL1_pbs_dB  = 20*log10(4*pi*f_pbs/c);
% PL1_wifi_dB = 20*log10(4*pi*f_wifi/c);
% for k=1:K
%     d = max(norm(w_b - WS(k,:)),1.0);
%     Xp = 0;
%     if useShadow,Xp = Shadow.pbs_su_dB(k); end
%     PL = PL1_pbs_dB + 10*n_pbs*log10(d) + Xp;
%     g  = 10^(-PL/10);
%     if useRayleigh
%         z = (randn + 1j*randn)/sqrt(2);
%         hp2(k) = g * abs(z)^2;
%     else
%         hp2(k) = g;
%     end
%     d = max(norm(w_wifi - WS(k,:)),1.0);
%     Xw = 0;
%     if useShadow,Xw = Shadow.wifi_su_dB(k); end
%     PL = PL1_wifi_dB + 10*n_wifi*log10(d) + Xw;
%     g  = 10^(-PL/10);
%     if useRayleigh
%         z = (randn + 1j*randn)/sqrt(2);
%         hw2(k) = g * abs(z)^2;
%     else
%         hw2(k) = g;
%     end
% end
% end
% %% ============================================================
% %                  VECTOR BF SLOT ALGORITHMS
% %% ============================================================
% function sumRate = eval_slot_aodss_mrt_vec(...
%         HsuL,HsuU,HpuL,HwuU,hp2,hw2,...
%         P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%         Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,allocMode)
% [idxLic,pLic,vLic,idxUn,pUn,vUn] = alloc_mrt_qos_vec(...
%     HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
%     sigma2_lic,sigma2_un,GammaLic,GammaUn,Pmax_lic,Pmax_un,...
%     QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,allocMode);
% sumRate = compute_sum_rate_vec(idxLic,pLic,vLic,idxUn,pUn,vUn,...
%     HsuL,HsuU,hp2,hw2,P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);
% end
% function sumRate = eval_slot_proposed_pb_mm_vec(...
%         HsuL,HsuU,HpuL,HwuU,hp2,hw2,...
%         P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
%         Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,...
%         MM_iter,MM_leakFrac,MM_reg_alpha)
% [idxLic,pLic,vLic,idxUn,pUn,vUn] = alloc_mm_pb_qos_vec(...
%     HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
%     sigma2_lic,sigma2_un,GammaLic,GammaUn,Pmax_lic,Pmax_un,...
%     QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,MM_iter,MM_leakFrac,MM_reg_alpha);
% sumRate = compute_sum_rate_vec(idxLic,pLic,vLic,idxUn,pUn,vUn,...
%     HsuL,HsuU,hp2,hw2,P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);
% end
% function [idxLic,pLic,vLic,idxUn,pUn,vUn] = alloc_mrt_qos_vec(...
%         HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
%         sigma2_lic,sigma2_un,GammaLic,GammaUn,Pmax_lic,Pmax_un,...
%         QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,allocMode)
% K = numel(HsuL);
% J = numel(HpuL);
% M = numel(HwuU);
% idxLic = zeros(0,2); pLic = []; vLic = {};
% idxUn  = zeros(0,2); pUn  = []; vUn  = {};
% for it = 1:AO_AP_iter %#ok<NASGU>
%     rhoL = assign_licensed_vec(HsuL,HpuL,hp2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,Bsc_lic,K,J,allocMode,'mrt',[],[],[],[]);
%     [pLic,idxLic,vLic] = solve_band_mrt_given_rho(rhoL,HsuL,HpuL,hp2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic);
%     needUn = true(K,1); % All SUs eligible in both bands
%     if any(needUn)
%         rhoU = assign_unlicensed_vec(HsuU,HwuU,hw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,Bsc_un,K,M,needUn,allocMode,'mrt',[],[],[],[]);
%         [pUn,idxUn,vUn] = solve_band_mrt_given_rho(rhoU,HsuU,HwuU,hw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un);
%     else
%         idxUn = zeros(0,2); pUn = []; vUn = {};
%     end
% end
% end
% function [idxLic,pLic,vLic,idxUn,pUn,vUn] = alloc_mm_pb_qos_vec(...
%         HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
%         sigma2_lic,sigma2_un,GammaLic,GammaUn,Pmax_lic,Pmax_un,...
%         QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,MM_iter,MM_leakFrac,MM_reg_alpha)
% K = numel(HsuL);
% J = numel(HpuL);
% M = numel(HwuU);
% idxLic = zeros(0,2); pLic = []; vLic = {};
% idxUn  = zeros(0,2); pUn  = []; vUn  = {};
% for it = 1:AO_AP_iter %#ok<NASGU>
%     rhoL = assign_licensed_vec(HsuL,HpuL,hp2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,Bsc_lic,K,J,...
%         'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,Pmax_lic/max(J,1));
%     [pLic,idxLic,vLic] = solve_band_mm_given_rho(rhoL,HsuL,HpuL,hp2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,...
%         MM_iter,MM_leakFrac,MM_reg_alpha);
%     needUn = true(K,1); % All SUs eligible in both bands
%     if any(needUn)
%         rhoU = assign_unlicensed_vec(HsuU,HwuU,hw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,Bsc_un,K,M,needUn,...
%             'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,Pmax_un/max(M,1));
%         [pUn,idxUn,vUn] = solve_band_mm_given_rho(rhoU,HsuU,HwuU,hw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,...
%             MM_iter,MM_leakFrac,MM_reg_alpha);
%     else
%         idxUn = zeros(0,2); pUn = []; vUn = {};
%     end
% end
% end
% function rho = assign_licensed_iac_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,alphaFallback,pRef)
% % Interference-aware licensed channel selection (new proactive idea).
% % The metric rewards useful SU gain/rate and penalizes:
% %   1) received licensed interference at the SU from PBS,%   2) leakage caused by the UAV toward the PU on that licensed subchannel.
% % Power is still allocated later by the same CBF-MM/KKT solver.
% Rmat = zeros(K,S);
% lambdaLeak = 1.00;
% lambdaRx   = 0.10;
% for k = 1:K
%     for s = 1:S
%         denomRx = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
%             v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
%         else
%             v = mrt_unit(Hsu{k});
%         end
%         gSig  = abs(Hsu{k}'*v)^2;
%         gLeak = abs(Hprot{s}'*v)^2;
%         p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
%         rateEst = Bsc*log2(1 + max(p_app*gSig/denomRx,0));
%         leakRatio = (p_app*gLeak) / max(GammaVec(s),1e-30);
%         rxRatio   = (hI(k)*P_I_vec(s)) / max(sigma2,1e-30);
%         iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio);
%         Rmat(k,s) = rateEst / iacPenalty;
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairs = assign_greedy_max(Rmat);
% else
%     pairs = assign_hungarian_max(Rmat);
% end
% rho = zeros(K,S);
% for ii = 1:size(pairs,1)
%     rho(pairs(ii,1),pairs(ii,2)) = 1;
% end
% end
% function rho = assign_unlicensed_iac_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,alphaFallback,pRef)
% % Interference-aware unlicensed channel selection (new proactive idea).
% % Active users are all associated users only. The metric rewards useful SU
% % rate and penalizes WiFi interference plus leakage toward WUs.
% rho = zeros(K,S);
% activeUsers = find(needUn(:));
% if isempty(activeUsers) || Pmax <= 0
%     return;
% end
% Ka = numel(activeUsers);
% Rmat = zeros(Ka,S);
% lambdaLeak = 1.00;
% lambdaRx   = 0.10;
% for aa = 1:Ka
%     k = activeUsers(aa);
%     for s = 1:S
%         denomRx = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
%             v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
%         else
%             v = mrt_unit(Hsu{k});
%         end
%         gSig  = abs(Hsu{k}'*v)^2;
%         gLeak = abs(Hprot{s}'*v)^2;
%         p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
%         rateEst = Bsc*log2(1 + max(p_app*gSig/denomRx,0));
%         leakRatio = (p_app*gLeak) / max(GammaVec(s),1e-30);
%         rxRatio   = (hI(k)*P_I_vec(s)) / max(sigma2,1e-30);
%         iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio);
%         Rmat(aa,s) = rateEst / iacPenalty;
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairsLocal = assign_greedy_max(Rmat);
% else
%     pairsLocal = assign_hungarian_max(Rmat);
% end
% for ii = 1:size(pairsLocal,1)
%     aa = pairsLocal(ii,1);
%     s  = pairsLocal(ii,2);
%     rho(activeUsers(aa),s) = 1;
% end
% end
% function rho = assign_licensed_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
% Rmat = zeros(K,S);
% for k = 1:K
%     for s = 1:S
%         denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
%             % Do NOT call CVX inside the assignment matrix. Calling CVX for every
%             % candidate (k,s) creates nested/non-empty CVX-scope warnings and is
%             % extremely slow. Use a leakage-aware RZF metric for assignment;
%             % true MM/WMMSE is applied later only to the selected pairs in
%             % solve_band_mm_given_rho() or solve_band_wmmse_given_rho().
%             v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
%             gSig = abs(Hsu{k}'*v)^2;
%             gLeak = abs(Hprot{s}'*v)^2;
%         else
%             v = mrt_unit(Hsu{k});
%             gSig = abs(Hsu{k}'*v)^2;
%             gLeak = abs(Hprot{s}'*v)^2;
%         end
%         p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
%         SINR = p_app * gSig / denom;
%         Rmat(k,s) = Bsc*log2(1 + max(SINR,0));
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairs = assign_greedy_max(Rmat);
% else
%     pairs = assign_hungarian_max(Rmat);
% end
% rho = zeros(K,S);
% for ii = 1:size(pairs,1)
%     rho(pairs(ii,1),pairs(ii,2)) = 1;
% end
% end
% function rho = assign_unlicensed_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
% rho = zeros(K,S);
% activeUsers = find(needUn(:));
% if isempty(activeUsers) || Pmax <= 0
%     return;
% end
% Ka = numel(activeUsers);
% Rmat = zeros(Ka,S);
% for aa = 1:Ka
%     k = activeUsers(aa);
%     for s = 1:S
%         denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
%             % Do NOT call CVX inside the assignment matrix. Calling CVX for every
%             % candidate (k,s) creates nested/non-empty CVX-scope warnings and is
%             % extremely slow. Use a leakage-aware RZF metric for assignment;
%             % true MM/WMMSE is applied later only to the selected pairs in
%             % solve_band_mm_given_rho() or solve_band_wmmse_given_rho().
%             v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
%             gSig = abs(Hsu{k}'*v)^2;
%             gLeak = abs(Hprot{s}'*v)^2;
%         else
%             v = mrt_unit(Hsu{k});
%             gSig = abs(Hsu{k}'*v)^2;
%             gLeak = abs(Hprot{s}'*v)^2;
%         end
%         p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
%         SINR = p_app * gSig / denom;
%         Rmat(aa,s) = Bsc*log2(1 + max(SINR,0));
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairsLocal = assign_greedy_max(Rmat);
% else
%     pairsLocal = assign_hungarian_max(Rmat);
% end
% for ii = 1:size(pairsLocal,1)
%     aa = pairsLocal(ii,1);
%     s  = pairsLocal(ii,2);
%     rho(activeUsers(aa),s) = 1;
% end
% end
% function [p_assigned,idxList,vList] = solve_band_mrt_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax)
% [~,S] = size(rho);
% idxList = zeros(0,2);
% a_list = [];
% cap_list = [];
% vList = {};
% for s = 1:S
%     k = find(rho(:,s)==1,1);
%     if ~isempty(k)
%         v = mrt_unit(Hsu{k});
%         gSig = abs(Hsu{k}'*v)^2;
%         gLeak = abs(Hprot{s}'*v)^2;
%         denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         a = max(gSig/denom,1e-30);
%         pcap = max(GammaVec(s)/max(gLeak,1e-30),0);
%         idxList = [idxList; k,s]; %#ok<AGROW>
%         a_list = [a_list; a]; %#ok<AGROW>
%         cap_list = [cap_list; pcap]; %#ok<AGROW>
%         vList{end+1,1} = v; %#ok<AGROW>
%     end
% end
% p_assigned = kkt_waterfill_capped_original(a_list,Pmax,cap_list);
% % Per-band 4-case interference-aware clipping is applied later in
% % apply_perband_4case_power_control().
% end
% function [p_assigned,idxList,vList] = solve_band_mm_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,MM_iter,leakFrac,alphaFallback)
% [~,S] = size(rho);
% idxList = zeros(0,2);
% a_list = [];
% cap_list = [];
% vList = {};
% active = nnz(rho);
% pRef = Pmax/max(active,1);
% for s = 1:S
%     k = find(rho(:,s)==1,1);
%     if ~isempty(k)
%         denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         [v,gSig,gLeak] = mm_unit_beam_unprotected(Hsu{k},denom,MM_iter,alphaFallback);
%         a = max(gSig/denom,1e-30);
%         pcap = Pmax; % No PU/WU cap in initial KKT
%         idxList = [idxList; k,s]; %#ok<AGROW>
%         a_list = [a_list; a]; %#ok<AGROW>
%         cap_list = [cap_list; pcap]; %#ok<AGROW>
%         vList{end+1,1} = v; %#ok<AGROW>
%     end
% end
% p_assigned = kkt_waterfill_capped_original(a_list,Pmax,cap_list);
% % Per-band 4-case interference-aware clipping is applied later in
% % apply_perband_4case_power_control().
% end
% function [p_assigned,idxList,vList] = solve_band_wmmse_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,WMMSE_iter,leakFrac,alphaFallback)
% % WMMSE-based leakage-aware beamforming for selected user-subchannel pairs.
% % This is CVX-free. It performs scalar receive-filter/weight updates and
% % transmitter beam updates with a leakage regularization term.
% [~,S] = size(rho);
% idxList = zeros(0,2);
% a_list = [];
% cap_list = [];
% vList = {};
% active = nnz(rho);
% pRef = Pmax/max(active,1);
% for s = 1:S
%     k = find(rho(:,s)==1,1);
%     if ~isempty(k)
%         denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         [v,gSig,gLeak] = wmmse_unit_beam(Hsu{k},Hprot{s},denom,GammaVec(s),pRef,WMMSE_iter,leakFrac,alphaFallback);
%         a = max(gSig/denom,1e-30);
%         pcap = max(GammaVec(s)/max(gLeak,1e-30),0); %#ok<NASGU>
%         idxList = [idxList; k,s]; %#ok<AGROW>
%         a_list = [a_list; a]; %#ok<AGROW>
%         cap_list = [cap_list; pcap]; %#ok<AGROW>
%         vList{end+1,1} = v; %#ok<AGROW>
%     end
% end
% p_assigned = kkt_waterfill_capped_original(a_list,Pmax,cap_list);
% % Per-band 4-case interference-aware clipping is applied later in
% % apply_perband_4case_power_control().
% end
% function [p_assigned,idxList,vList] = solve_band_fp_cbf_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,FP_iter,leakFrac,alphaFallback)
% % FP-based CBF for selected user-subchannel pairs.
% % CVX-free quadratic-transform style update with leakage regularization.
% [~,S] = size(rho);
% idxList = zeros(0,2);
% a_list = [];
% cap_list = [];
% vList = {};
% active = nnz(rho);
% pRef = Pmax/max(active,1);
% for s = 1:S
%     k = find(rho(:,s)==1,1);
%     if ~isempty(k)
%         denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         [v,gSig,gLeak] = fp_cbf_unit_beam(Hsu{k},Hprot{s},denom,GammaVec(s),pRef,FP_iter,leakFrac,alphaFallback);
%         a = max(gSig/denom,1e-30);
%         pcap = max(GammaVec(s)/max(gLeak,1e-30),0); %#ok<NASGU>
%         idxList = [idxList; k,s]; %#ok<AGROW>
%         a_list = [a_list; a]; %#ok<AGROW>
%         cap_list = [cap_list; pcap]; %#ok<AGROW>
%         vList{end+1,1} = v; %#ok<AGROW>
%     end
% end
% p_assigned = kkt_waterfill_capped_original(a_list,Pmax,cap_list);
% end
% function [p_assigned,idxList,vList] = solve_band_safe_drl_cbf_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,SafeIter,leakFrac,alphaFallback)
% % Safe-DRL-style CBF for selected pairs.
% % This is a lightweight safety-layer policy selector over candidate beam actions.
% % It is not a full offline-trained neural DRL implementation.
% [~,S] = size(rho);
% idxList = zeros(0,2);
% a_list = [];
% cap_list = [];
% vList = {};
% active = nnz(rho);
% pRef = Pmax/max(active,1);
% for s = 1:S
%     k = find(rho(:,s)==1,1);
%     if ~isempty(k)
%         denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         [v,gSig,gLeak] = safe_drl_cbf_unit_beam(Hsu{k},Hprot{s},denom,GammaVec(s),pRef,SafeIter,leakFrac,alphaFallback);
%         a = max(gSig/denom,1e-30);
%         pcap = max(GammaVec(s)/max(gLeak,1e-30),0); %#ok<NASGU>
%         idxList = [idxList; k,s]; %#ok<AGROW>
%         a_list = [a_list; a]; %#ok<AGROW>
%         cap_list = [cap_list; pcap]; %#ok<AGROW>
%         vList{end+1,1} = v; %#ok<AGROW>
%     end
% end
% p_assigned = kkt_waterfill_capped_original(a_list,Pmax,cap_list);
% end
% function v = mrt_unit(h)
% nh = norm(h);
% if nh < 1e-30
%     v = ones(numel(h),1)/sqrt(numel(h));
% else
%     v = h/nh;
% end
% end
% function [v,gSig,gLeak] = mm_unit_beam(hSU,hProt,denom,Gamma,pRef,MM_iter,leakFrac,alphaFallback)
% global AIIBPC_BEAM_CALLS;
% if ~isempty(AIIBPC_BEAM_CALLS), AIIBPC_BEAM_CALLS=AIIBPC_BEAM_CALLS+1; end
% % MM-based leakage-aware unit-norm beam design.
% % ------------------------------------------------------------
% % This replaces the previous FP/SDR CVX-based beamformer.
% % It solves a surrogate sequence for the useful-signal/leakage tradeoff:
% %
% %   maximize_v   v^H (Hs/denom - mu*Hp) v
% %   subject to   ||v||_2 = 1,%
% % where Hs = hSU*hSU^H and Hp = hProt*hProt^H. The leakage constraint is
% % finally polished by a leakage-aware projection/RZF step. Actual PU/WU
% % interference protection is still enforced later by the proposed 4-case
% % per-band power clipping function.
% %
% % Important: this is an MM-style iterative beam update,not FP/SDR and not CVX.
% Nt = numel(hSU);
% Hs = hSU*hSU';
% Hp = hProt*hProt';
% leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
% leakCapUnit = max(leakCapUnit,0);
% % Initial feasible/stable beam: leakage-aware RZF.
% v = leakage_rzf_unit(hSU,hProt,alphaFallback);
% % If the RZF beam still violates the unit leakage cap,try null-space beam.
% if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-9)
%     v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
% end
% % Penalty weight for leakage. Larger pRef/Gamma means stronger protection.
% mu = pRef / max(Gamma,1e-30);
% % Hermitian quadratic matrix for MM surrogate/power-iteration update.
% C = Hs/max(denom,1e-30) - mu*Hp;
% C = (C + C')/2;
% % Shifted positive-semidefinite surrogate matrix.
% lamMin = min(real(eig(C)));
% tau = max(0,-lamMin) + alphaFallback;
% B = C + tau*eye(Nt);
% B = (B + B')/2;
% for it = 1:max(MM_iter,1)
%     vNew = B*v;
%     if norm(vNew) < 1e-30
%         vNew = mrt_unit(hSU);
%     else
%         vNew = vNew/norm(vNew);
%     end
%     % Leakage polishing after each MM step.
%     if leakCapUnit > 0 && abs(hProt'*vNew)^2 > leakCapUnit*(1+1e-9)
%         vNew = polish_leakage_beam(vNew,hSU,hProt,leakCapUnit,alphaFallback);
%     end
%     if norm(vNew - v) < 1e-6
%         v = vNew;
%         break;
%     end
%     v = vNew;
% end
% % Final normalization and leakage polishing.
% v = v / max(norm(v),1e-30);
% if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-6)
%     v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
% end
% gSig  = abs(hSU'*v)^2;
% gLeak = abs(hProt'*v)^2;
% end
% function [v,gSig,gLeak] = wmmse_unit_beam(hSU,hProt,denom,Gamma,pRef,WMMSE_iter,leakFrac,alphaFallback)
% % WMMSE-based leakage-aware unit-norm beam design.
% % ------------------------------------------------------------
% % For one selected stream,the scalar WMMSE updates are:
% %   u = sqrt(pRef) h^H v / (pRef |h^H v|^2 + denom)
% %   e = 1 - 2 Re{u^* sqrt(pRef) h^H v} + |u|^2(pRef |h^H v|^2 + denom)
% %   w = 1/e
% % Then the beam is updated from a regularized least-squares/WMMSE surrogate
% % with leakage penalty to protect PU/WU receivers:
% %   v <- (w|u|^2 pRef hh^H + mu Hp + alpha I)^(-1) (w u sqrt(pRef) h)
% % followed by normalization and leakage polishing.
% Nt = numel(hSU);
% Hs = hSU*hSU';
% Hp = hProt*hProt';
% leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
% leakCapUnit = max(leakCapUnit,0);
% % Stable initial beam.
% v = leakage_rzf_unit(hSU,hProt,alphaFallback);
% if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-9)
%     v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
% end
% mu = pRef / max(Gamma,1e-30);
% for it = 1:max(WMMSE_iter,1)
%     hv = hSU' * v;
%     rxDen = max(pRef*abs(hv)^2 + denom,1e-30);
%     u = sqrt(max(pRef,0)) * hv / rxDen;
%     mse = 1 - 2*real(conj(u)*sqrt(max(pRef,0))*hv) + abs(u)^2 * rxDen;
%     mse = max(real(mse),1e-12);
%     w = 1/mse;
%     A = w*abs(u)^2*pRef*Hs + mu*Hp + alphaFallback*eye(Nt);
%     b = w*conj(u)*sqrt(max(pRef,0))*hSU;
%     vNew = A \ b;
%     if norm(vNew) < 1e-30
%         vNew = mrt_unit(hSU);
%     else
%         vNew = vNew/norm(vNew);
%     end
%     if leakCapUnit > 0 && abs(hProt'*vNew)^2 > leakCapUnit*(1+1e-9)
%         vNew = polish_leakage_beam(vNew,hSU,hProt,leakCapUnit,alphaFallback);
%     end
%     if norm(vNew - v) < 1e-6
%         v = vNew;
%         break;
%     end
%     v = vNew;
% end
% v = v/max(norm(v),1e-30);
% if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-6)
%     v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
% end
% gSig  = abs(hSU'*v)^2;
% gLeak = abs(hProt'*v)^2;
% end
% function v = polish_leakage_beam(vIn,hSU,hProt,leakCapUnit,alphaFallback)
% % Reduce leakage while retaining useful signal as much as possible.
% Nt = numel(hSU);
% % Candidate 1: leakage-aware RZF.
% v1 = leakage_rzf_unit(hSU,hProt,alphaFallback);
% % Candidate 2: null-space beam if possible.
% Pnull = eye(Nt) - (hProt*hProt')/max(norm(hProt)^2,1e-30);
% v2 = Pnull*hSU;
% if norm(v2) > 1e-12
%     v2 = v2/norm(v2);
% else
%     v2 = v1;
% end
% % Candidate 3: blend current beam and null-space beam.
% v3 = vIn / max(norm(vIn),1e-30);
% if abs(hProt'*v3)^2 > leakCapUnit && abs(hProt'*v2)^2 <= leakCapUnit*(1+1e-9)
%     lo = 0; hi = 1;
%     for bb = 1:40
%         aa = 0.5*(lo+hi);
%         vb = (1-aa)*v3 + aa*v2;
%         vb = vb/max(norm(vb),1e-30);
%         if abs(hProt'*vb)^2 <= leakCapUnit
%             hi = aa;
%         else
%             lo = aa;
%         end
%     end
%     v3 = (1-hi)*v3 + hi*v2;
%     v3 = v3/max(norm(v3),1e-30);
% end
% cands = {v1,v2,v3};
% best = cands{1};
% bestScore = -inf;
% for ii = 1:numel(cands)
%     vc = cands{ii};
%     vc = vc/max(norm(vc),1e-30);
%     leak = abs(hProt'*vc)^2;
%     sig  = abs(hSU'*vc)^2;
%     if leak <= leakCapUnit*(1+1e-6)
%         score = sig;
%     else
%         score = sig/(1 + 1e6*(leak/leakCapUnit - 1));
%     end
%     if score > bestScore
%         bestScore = score;
%         best = vc;
%     end
% end
% v = best/max(norm(best),1e-30);
% end
% function [v,gSig,gLeak] = fp_cbf_unit_beam(hSU,hProt,denom,Gamma,pRef,FP_iter,leakFrac,alphaFallback)
% % FP-based CBF unit-norm beam design.
% % ------------------------------------------------------------
% % Uses a quadratic-transform style update for the SINR objective:
% %   log(1 + p|h^H v|^2/denom)
% % combined with a leakage regularization term for PU/WU protection.
% % This is CVX-free and suited for repeated Monte-Carlo simulation.
% Nt = numel(hSU);
% Hs = hSU*hSU';
% Hp = hProt*hProt';
% leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
% leakCapUnit = max(leakCapUnit,0);
% v = leakage_rzf_unit(hSU,hProt,alphaFallback);
% if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-9)
%     v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
% end
% mu = pRef / max(Gamma,1e-30);
% for it = 1:max(FP_iter,1)
%     hv = hSU' * v;
%     y = sqrt(max(pRef,0)) * hv / max(pRef*abs(hv)^2 + denom,1e-30);
%     A = abs(y)^2*pRef*Hs + mu*Hp + alphaFallback*eye(Nt);
%     b = conj(y)*sqrt(max(pRef,0))*hSU;
%     vNew = A \ b;
%     if norm(vNew) < 1e-30
%         vNew = mrt_unit(hSU);
%     else
%         vNew = vNew/norm(vNew);
%     end
%     if leakCapUnit > 0 && abs(hProt'*vNew)^2 > leakCapUnit*(1+1e-9)
%         vNew = polish_leakage_beam(vNew,hSU,hProt,leakCapUnit,alphaFallback);
%     end
%     if norm(vNew-v) < 1e-6
%         v = vNew;
%         break;
%     end
%     v = vNew;
% end
% v = v/max(norm(v),1e-30);
% if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-6)
%     v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
% end
% gSig  = abs(hSU'*v)^2;
% gLeak = abs(hProt'*v)^2;
% end
% function [v,gSig,gLeak] = safe_drl_cbf_unit_beam(hSU,hProt,denom,Gamma,pRef,SafeIter,leakFrac,alphaFallback)
% % Safe DRL-based CBF lightweight baseline.
% % ------------------------------------------------------------
% % It emulates a safe policy-improvement layer: generate candidate beam actions,% project unsafe candidates through the safety layer,evaluate reward with an
% % interference-temperature cost,and select the safest/highest-reward action.
% % This avoids heavy DNN training while preserving the safety idea in simulation.
% Nt = numel(hSU);
% leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
% leakCapUnit = max(leakCapUnit,0);
% cand = {};
% cand{end+1} = mrt_unit(hSU);
% cand{end+1} = leakage_rzf_unit(hSU,hProt,alphaFallback);
% [c1,~,~] = mm_unit_beam(hSU,hProt,denom,Gamma,pRef,max(2,ceil(SafeIter/2)),leakFrac,alphaFallback); cand{end+1} = c1;
% [c2,~,~] = wmmse_unit_beam(hSU,hProt,denom,Gamma,pRef,max(2,ceil(SafeIter/2)),leakFrac,alphaFallback); cand{end+1} = c2;
% [c3,~,~] = fp_cbf_unit_beam(hSU,hProt,denom,Gamma,pRef,max(2,ceil(SafeIter/2)),leakFrac,alphaFallback); cand{end+1} = c3;
% % Deterministic exploration actions around the useful-channel direction.
% base = leakage_rzf_unit(hSU,hProt,alphaFallback);
% Pnull = eye(Nt) - (hProt*hProt')/max(norm(hProt)^2,1e-30);
% nullDir = Pnull*hSU;
% if norm(nullDir) > 1e-12
%     nullDir = nullDir/norm(nullDir);
% else
%     nullDir = base;
% end
% for rr = 1:max(SafeIter,1)
%     beta = rr/(max(SafeIter,1)+1);
%     vb = (1-beta)*base + beta*nullDir;
%     if norm(vb) > 1e-30
%         cand{end+1} = vb/norm(vb); %#ok<AGROW>
%     end
% end
% bestReward = -inf;
% v = cand{1};
% for ii = 1:numel(cand)
%     vc = cand{ii};
%     vc = vc/max(norm(vc),1e-30);
%     if leakCapUnit > 0 && abs(hProt'*vc)^2 > leakCapUnit*(1+1e-9)
%         vc = polish_leakage_beam(vc,hSU,hProt,leakCapUnit,alphaFallback);
%     end
%     sig = abs(hSU'*vc)^2;
%     leak = abs(hProt'*vc)^2;
%     rateReward = log2(1 + max(pRef*sig/max(denom,1e-30),0));
%     safetyCost = max(0,pRef*leak - Gamma)/max(Gamma,1e-30);
%     reward = rateReward - 20*safetyCost^2;
%     if reward > bestReward
%         bestReward = reward;
%         v = vc;
%     end
% end
% v = v/max(norm(v),1e-30);
% gSig  = abs(hSU'*v)^2;
% gLeak = abs(hProt'*v)^2;
% end
% function v = leakage_rzf_unit(hSU,hProt,alpha)
% Nt = numel(hSU);
% A = hProt*hProt' + alpha*eye(Nt);
% v = A \ hSU;
% if norm(v) < 1e-30
%     v = mrt_unit(hSU);
% else
%     v = v/norm(v);
% end
% end
% function Ruser = compute_user_rate_band_vec(idx,p,vList,Hsu,hI,P_I_vec,sigma2,Bsc,K)
% Ruser = zeros(K,1);
% for ell = 1:size(idx,1)
%     k = idx(ell,1);
%     s = idx(ell,2);
%     v = vList{ell};
%     denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%     gSig = abs(Hsu{k}'*v)^2;
%     SINR = p(ell)*gSig/denom;
%     Ruser(k) = Ruser(k) + Bsc*log2(1 + max(SINR,0));
% end
% end
% function sumRate = compute_sum_rate_vec(idxLic,pLic,vLic,idxUn,pUn,vUn,...
%                                     HsuL,HsuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
%                                     sigma2_lic,sigma2_un,Bsc_lic,Bsc_un)
% sumRate = 0;
% for ell=1:size(idxLic,1)
%     k=idxLic(ell,1); j=idxLic(ell,2);
%     v = vLic{ell};
%     denom = max(hp2(k)*P_PBS_j(j) + sigma2_lic,1e-30);
%     gSig = abs(HsuL{k}'*v)^2;
%     SINR = pLic(ell)*gSig/denom;
%     sumRate = sumRate + Bsc_lic*log2(1 + max(SINR,0));
% end
% for ell=1:size(idxUn,1)
%     k=idxUn(ell,1); m=idxUn(ell,2);
%     v = vUn{ell};
%     denom = max(hw2(k)*P_WIFI_m(m) + sigma2_un,1e-30);
%     gSig = abs(HsuU{k}'*v)^2;
%     SINR = pUn(ell)*gSig/denom;
%     sumRate = sumRate + Bsc_un*log2(1 + max(SINR,0));
% end
% end
% %% ============================================================
% %                  SCALAR TRAJECTORY ALLOCATION HELPERS
% %% ============================================================
% function [idxLic,pLic,idxUn,pUn] = alloc_original_scalar_qos(...
%         hu2L,hu2U,hp2,hw2,h_up2,h_uw2,...
%         P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,...
%         GammaLic,GammaUn,Pmax_lic,Pmax_un,QoS_target,...
%         Bsc_lic,Bsc_un,AO_AP_iter,allocMode)
% K = numel(hu2L);
% M = numel(h_uw2);
% idxLic = zeros(0,2); pLic = [];
% idxUn  = zeros(0,2); pUn  = [];
% for it = 1:AO_AP_iter %#ok<NASGU>
%     rhoL = assign_licensed_original(hu2L,hp2,h_up2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,Bsc_lic,K,numel(h_up2),allocMode);
%     [pLic,idxLic] = solve_band_kkt_original(rhoL,hu2L,hp2,P_PBS_j,sigma2_lic,GammaLic,h_up2,Pmax_lic);
%     needUn = true(K,1); % All SUs eligible in both bands
%     if any(needUn)
%         rhoU = assign_unlicensed_for_users(hu2U,hw2,h_uw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,Bsc_un,K,M,needUn,allocMode);
%         [pUn,idxUn] = solve_band_kkt_original(rhoU,hu2U,hw2,P_WIFI_m,sigma2_un,GammaUn,h_uw2,Pmax_un);
%     else
%         pUn = [];
%         idxUn = zeros(0,2);
%     end
% end
% end
% function rhoL = assign_licensed_original(hu2L,hp2,h_up2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,Bsc_lic,K,J,allocMode)
% if nargin < 11 || isempty(allocMode)
%     allocMode = 'hungarian';
% end
% RmatL = zeros(K,J);
% for k = 1:K
%     for j = 1:J
%         p_app = min(Pmax_lic/max(J,1),GammaLic(j)/max(h_up2(j),1e-30));
%         denom = max(hp2(k)*P_PBS_j(j) + sigma2_lic,1e-30);
%         SINR  = (hu2L(k)*p_app)/denom;
%         RmatL(k,j) = Bsc_lic*log2(1 + max(SINR,0));
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairsL = assign_greedy_max(RmatL);
% else
%     pairsL = assign_hungarian_max(RmatL);
% end
% rhoL = zeros(K,J);
% for ii = 1:size(pairsL,1)
%     rhoL(pairsL(ii,1),pairsL(ii,2)) = 1;
% end
% end
% function rhoU = assign_unlicensed_for_users(hu2U,hw2,h_uw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,Bsc_un,K,M,needUn,allocMode)
% if nargin < 12 || isempty(allocMode)
%     allocMode = 'hungarian';
% end
% rhoU = zeros(K,M);
% activeUsers = find(needUn(:));
% if isempty(activeUsers) || Pmax_un <= 0
%     return;
% end
% Ka = numel(activeUsers);
% RmatU = zeros(Ka,M);
% for aa = 1:Ka
%     k = activeUsers(aa);
%     for m = 1:M
%         p_app = min(Pmax_un/max(M,1),GammaUn(m)/max(h_uw2(m),1e-30));
%         denom = max(hw2(k)*P_WIFI_m(m) + sigma2_un,1e-30);
%         SINR  = (hu2U(k)*p_app)/denom;
%         RmatU(aa,m) = Bsc_un*log2(1 + max(SINR,0));
%     end
% end
% if strcmpi(allocMode,'greedy')
%     pairsLocal = assign_greedy_max(RmatU);
% else
%     pairsLocal = assign_hungarian_max(RmatU);
% end
% for ii = 1:size(pairsLocal,1)
%     aa = pairsLocal(ii,1);
%     m  = pairsLocal(ii,2);
%     rhoU(activeUsers(aa),m) = 1;
% end
% end
% function [p_assigned,idxList] = solve_band_kkt_original(rho,hu2,hI,P_I_vec,sigma2,GammaVec,hProtect,Pmax)
% [~,S] = size(rho);
% idxList = zeros(0,2);
% a_list  = [];
% cap_list= [];
% for s = 1:S
%     k = find(rho(:,s)==1,1);
%     if ~isempty(k)
%         denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%         a = max(hu2(k) / denom,1e-30);
%         pcap = max(GammaVec(s) / max(hProtect(s),1e-30),0);
%         idxList = [idxList; k,s]; %#ok<AGROW>
%         a_list  = [a_list; a]; %#ok<AGROW>
%         cap_list= [cap_list; pcap]; %#ok<AGROW>
%     end
% end
% p_assigned = kkt_waterfill_capped_original(a_list,Pmax,cap_list);
% end
% function p = kkt_waterfill_capped_original(a,Ptot,pcap)
% L = numel(a);
% p = zeros(L,1);
% if L==0 || Ptot<=0,return; end
% a = max(a(:),1e-30);
% pcap = max(pcap(:),0);
% if isfinite(sum(pcap)) && sum(pcap) <= Ptot
%     p = pcap;
%     return;
% end
% lam_lo = 1e-12;
% lam_hi = 1e12;
% for it = 1:100
%     lam = 0.5*(lam_lo + lam_hi);
%     p_try = max(0,1./lam - 1./a);
%     p_try = min(p_try,pcap);
%     if sum(p_try) > Ptot
%         lam_lo = lam;
%     else
%         lam_hi = lam;
%     end
% end
% p = max(0,1./lam_hi - 1./a);
% p = min(p,pcap);
% s = sum(p);
% if s > Ptot && s > 0
%     p = p * (Ptot/s);
% end
% end
% function Ruser = compute_user_rate_band(idx,p,hu2,hI,P_I_vec,sigma2,Bsc,K)
% Ruser = zeros(K,1);
% for ell = 1:size(idx,1)
%     k = idx(ell,1);
%     s = idx(ell,2);
%     denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%     SINR = hu2(k)*p(ell)/denom;
%     Ruser(k) = Ruser(k) + Bsc*log2(1 + max(SINR,0));
% end
% end
% function c_k = build_c_per_user(idx,p_assigned,hI,P_I_vec,sigma2,K)
% c_k = zeros(K,1);
% for ell = 1:size(idx,1)
%     k = idx(ell,1);
%     s = idx(ell,2);
%     denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
%     c_k(k) = c_k(k) + p_assigned(ell) / denom;
% end
% end
% function B_k = build_B_per_user(idx,Bsc,K)
% B_k = zeros(K,1);
% for ell = 1:size(idx,1)
%     k = idx(ell,1);
%     B_k(k) = B_k(k) + Bsc;
% end
% end
% %% ============================================================
% %                  ASSIGNMENT HELPERS
% %% ============================================================
% function pairs = assign_greedy_max(score)
% [R,C] = size(score);
% L = min(R,C);
% pairs = zeros(0,2);
% if L == 0
%     return;
% end
% usedR = false(R,1);
% usedC = false(C,1);
% for ell = 1:L
%     bestVal = -inf;
%     bestR = 0;
%     bestC = 0;
%     for r = 1:R
%         if usedR(r),continue; end
%         for cc = 1:C
%             if usedC(cc),continue; end
%             if score(r,cc) > bestVal
%                 bestVal = score(r,cc);
%                 bestR = r;
%                 bestC = cc;
%             end
%         end
%     end
%     if bestR == 0
%         break;
%     end
%     pairs = [pairs; bestR,bestC]; %#ok<AGROW>
%     usedR(bestR) = true;
%     usedC(bestC) = true;
% end
% end
% function pairs = assign_hungarian_max(score)
% if isempty(score)
%     pairs = zeros(0,2);
%     return;
% end
% if exist('matchpairs','file') == 2
%     cost = -score;
%     bigM = 1e12;
%     pairs = matchpairs(cost,bigM,'min');
%     pairs = sortrows(pairs,1);
% else
%     pairs = assign_bruteforce(-score);
% end
% end
% function pairs = assign_bruteforce(cost)
% [R,C] = size(cost);
% L = min(R,C);
% pairs = zeros(L,2);
% if L==0,return; end
% bestVal = inf;
% if R <= C
%     P = perms(1:C);
%     for i=1:size(P,1)
%         sel = P(i,1:R);
%         val = 0;
%         for r=1:R
%             val = val + cost(r,sel(r));
%         end
%         if val < bestVal
%             bestVal = val;
%             pairs = [(1:R)',sel(:)];
%         end
%     end
% else
%     P = perms(1:R);
%     for i=1:size(P,1)
%         sel = P(i,1:C);
%         val = 0;
%         for cc=1:C
%             val = val + cost(sel(cc),cc);
%         end
%         if val < bestVal
%             bestVal = val;
%             pairs = [sel(:),(1:C)'];
%         end
%     end
%     pairs = pairs(1:L,:);
% end
% pairs = pairs(1:L,:);
% end
% %% ============================================================
% %                  TRAJECTORY UPDATE HELPERS
% %% ============================================================
% function [q_next,solved] = traj_step_cvx_sca_from_gradient(q0,qPrev,qNext,...
%     gradF,L0,gradL,stepMax,area,radius)
% % Affine numerical first-order models of objective and normalized leakage.
% % Fixed neighboring positions preserve the entire trajectory's mobility.
% q_next=q0; solved=false;
% if exist('cvx_begin','file')~=2
%     error('CVX is required.');
% end
% try
%     cvx_begin quiet
%         variable q(2)
%         maximize(gradF(:).'*(q-q0(:)))
%         subject to
%             norm(q-q0(:),2) <= radius;
%             norm(q-qPrev(:),2) <= stepMax;
%             norm(q-qNext(:),2) <= stepMax;
%             q>=0;
%             q<=area;
%             L0(:)+gradL*(q-q0(:))<=1;
%     cvx_end
%     solved=contains(lower(cvx_status),'solved') && all(isfinite(q));
%     if solved
%         q_next=q(:).';
%     end
% catch ME
%     warning('CVX trajectory subproblem failed: %s',ME.message);
%     try,cvx_clear;catch,end
% end
% end
% %% Old unused helper retained below; main trajectory update now uses CVX/SCA.
% function q_next = traj_step_projected_gradient_mean(q0,q_prev,q_end,n,N,WS,H,f_lic,f_un,c,A2G,...
%         cLic_k,Blic_k,cUn_k,Bun_k,stepMax,area,epsFD)
% Kloc = size(WS,1);
% grad = [0,0];
% for k = 1:Kloc
%     wk = WS(k,:);
%     if cLic_k(k) > 0 && Blic_k(k) > 0
%         [dgdx,dgdy,g0] = fd_gain_gradient(q0,wk,H,f_lic,c,A2G,epsFD);
%         coeff = (Blic_k(k)/log(2)) * cLic_k(k) / max(1 + cLic_k(k)*g0,1e-12);
%         grad = grad + coeff * [dgdx,dgdy];
%     end
%     if cUn_k(k) > 0 && Bun_k(k) > 0
%         [dgdx,dgdy,g0] = fd_gain_gradient(q0,wk,H,f_un,c,A2G,epsFD);
%         coeff = (Bun_k(k)/log(2)) * cUn_k(k) / max(1 + cUn_k(k)*g0,1e-12);
%         grad = grad + coeff * [dgdx,dgdy];
%     end
% end
% if norm(grad) < 1e-12
%     q_candidate = q0;
% else
%     q_candidate = q_prev + stepMax * grad/norm(grad);
% end
% q_next = project_single_step(q_candidate,q_prev,q_end,n,N,stepMax,area);
% end
% function [dgdx,dgdy,g0] = fd_gain_gradient(q0,wk,H,f,c,A2G,epsFD)
% g0  = a2g_gain_alhourani_mean(q0,wk,H,f,c,A2G);
% gx1 = a2g_gain_alhourani_mean(q0+[epsFD,0],wk,H,f,c,A2G);
% gx2 = a2g_gain_alhourani_mean(q0-[epsFD,0],wk,H,f,c,A2G);
% gy1 = a2g_gain_alhourani_mean(q0+[0,epsFD],wk,H,f,c,A2G);
% gy2 = a2g_gain_alhourani_mean(q0-[0,epsFD],wk,H,f,c,A2G);
% dgdx = (gx1-gx2)/(2*epsFD);
% dgdy = (gy1-gy2)/(2*epsFD);
% end
% function q_projected = project_single_step(q,q_prev,q_end,n,N,stepMax,area)
% q_projected = min(max(q,[0,0]),[area,area]);
% d = q_projected - q_prev;
% nd = norm(d);
% if nd > stepMax
%     q_projected = q_prev + stepMax * d / max(nd,1e-12);
% end
% q_projected = min(max(q_projected,[0,0]),[area,area]);
% remSteps = N - n;
% rReach = remSteps * stepMax;
% v = q_projected - q_end;
% nv = norm(v);
% if nv > rReach
%     q_projected = q_end + rReach * v / max(nv,1e-12);
% end
% q_projected = min(max(q_projected,[0,0]),[area,area]);
% d = q_projected - q_prev;
% nd = norm(d);
% if nd > stepMax
%     q_projected = q_prev + stepMax * d / max(nd,1e-12);
% end
% q_projected = min(max(q_projected,[0,0]),[area,area]);
% end
% function Q = project_full_trajectory(Q,area,stepMax,q_start,q_end,projIter)
% N = size(Q,1);
% Q(:,1) = min(max(Q(:,1),0),area);
% Q(:,2) = min(max(Q(:,2),0),area);
% Q(1,:) = q_start;
% Q(N,:) = q_end;
% for it = 1:projIter
%     Q(1,:) = q_start;
%     for n = 2:N
%         d = Q(n,:) - Q(n-1,:);
%         nd = norm(d);
%         if nd > stepMax
%             Q(n,:) = Q(n-1,:) + stepMax * d / max(nd,1e-12);
%         end
%         Q(n,1) = min(max(Q(n,1),0),area);
%         Q(n,2) = min(max(Q(n,2),0),area);
%     end
%     Q(N,:) = q_end;
%     for n = N-1:-1:1
%         d = Q(n,:) - Q(n+1,:);
%         nd = norm(d);
%         if nd > stepMax
%             Q(n,:) = Q(n+1,:) + stepMax * d / max(nd,1e-12);
%         end
%         Q(n,1) = min(max(Q(n,1),0),area);
%         Q(n,2) = min(max(Q(n,2),0),area);
%     end
%     Q(1,:) = q_start;
%     Q(N,:) = q_end;
% end
% end
% %% ============================================================
% %                  A2G GAIN
% %% ============================================================
% function g = a2g_gain_alhourani(q,w,H,f,c,A2G,useShadowA2G,z)
% d2 = norm(q - w);
% d3 = sqrt(d2^2 + H^2);
% theta = atan2d(H,max(d2,1e-9));
% P_LoS = 1 / (1 + A2G.a * exp(-A2G.b * (theta - A2G.a)));
% P_NLoS = 1 - P_LoS;
% FSPL_dB = 20*log10(4*pi*f*d3/c);
% if useShadowA2G
%     sig_dB = sqrt(P_LoS*A2G.sigmaLoS_dB^2 + P_NLoS*A2G.sigmaNLoS_dB^2);
%     sh_dB  = sig_dB * z;
% else
%     sh_dB = 0;
% end
% eta_dB = P_LoS*A2G.etaLoS_dB + P_NLoS*A2G.etaNLoS_dB;
% PL_dB = FSPL_dB + eta_dB + sh_dB;
% g = 10^(-PL_dB/10);
% end
% function g = a2g_gain_alhourani_mean(q,w,H,f,c,A2G)
% d2 = norm(q - w);
% d3 = sqrt(d2^2 + H^2);
% theta = atan2d(H,max(d2,1e-9));
% P_LoS = 1 / (1 + A2G.a * exp(-A2G.b * (theta - A2G.a)));
% P_NLoS = 1 - P_LoS;
% FSPL_dB = 20*log10(4*pi*f*d3/c);
% eta_dB = P_LoS*A2G.etaLoS_dB + P_NLoS*A2G.etaNLoS_dB;
% PL_dB = FSPL_dB + eta_dB;
% g = 10^(-PL_dB/10);
% end
% function [v,gSig,gLeak] = mm_unit_beam_unprotected(hSU,denom,MM_iter,alphaFallback)
% global AIIBPC_BEAM_CALLS;
% if ~isempty(AIIBPC_BEAM_CALLS), AIIBPC_BEAM_CALLS=AIIBPC_BEAM_CALLS+1; end
% % Initial CBF-MM surrogate WITHOUT PU/WU interference penalties.
% % The rank-one desired-signal surrogate converges to MRT.
% Nt=numel(hSU); B=(hSU*hSU')/max(denom,1e-30)+alphaFallback*eye(Nt);
% v=ones(Nt,1)/sqrt(Nt);
% for it=1:max(1,MM_iter)
%     w=B*v; vNew=w/max(norm(w),1e-30);
%     if norm(vNew-v)<1e-8, v=vNew; break; end
%     v=vNew;
% end
% gSig=abs(hSU'*v)^2;
% gLeak=0; % Protection-channel leakage is measured only after allocation.
% end
% 
% 
% function [tx,transferLog] = optionB_same_uav_rate_reallocation( ...
%     txRaw,tx,HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m, ...
%     sigma2L,sigma2U,BscL,BscU,GammaL,GammaU,PmaxL,PmaxU,verbose)
% if nargin<19,verbose=true;end
% % Rate-aware Option B: redistribute existing power on the same UAV,
% % never transfer watts across bands. Only clipped streams qualify as
% % recipients. For each, test donors on the opposite band of the SAME UAV.
% % Each candidate is checked against the actual network-wide sum rate and
% % aggregate PU/WU interference. Accept strictly positive gains only.
% transferLog = zeros(0,6);
% for u = 1:numel(tx)
%     for srcBandNum = 1:2
%         if srcBandNum == 1
%             sourceBand = 'lic'; destBand = 'un'; Hdest = HwuU; Gdest = GammaU;
%         else
%             sourceBand = 'un'; destBand = 'lic'; Hdest = HpuL; Gdest = GammaL;
%         end
%         [srcIdx,srcNow] = get_band_fields(tx,u,sourceBand);
%         [srcIdxRaw,srcRaw] = get_band_fields(txRaw,u,sourceBand);
%         [dstIdx,dstP] = get_band_fields(tx,u,destBand);
%         if isempty(srcIdx) || isempty(dstIdx), continue; end
%         for r = 1:size(srcIdx,1)
%             originalRow = find(srcIdxRaw(:,1)==srcIdx(r,1) & ...
%                                srcIdxRaw(:,2)==srcIdx(r,2),1);
%             if isempty(originalRow) || srcRaw(originalRow)-srcNow(r)<1e-9
%                 continue;
%             end
%             recipient = find(dstIdx(:,1)==srcIdx(r,1),1);
%             if isempty(recipient), continue; end
%             % Donor stream must belong to another SU of this UAV.
%             for donor = 1:numel(dstP)
%                 if donor==recipient || dstP(donor)<1e-7, continue; end
%                 baseline = optionB_total_rate(tx,HsuL,HsuU,hp2,hw2, ...
%                     P_PBS_j,P_WIFI_m,sigma2L,sigma2U,BscL,BscU);
%                 bestGain = 0; bestTx = tx; bestDelta = 0;
%                 % Conservative transfer fractions; preserve the full budget.
%                 for frac = [0.05 0.10 0.20 0.35 0.50 0.75 1.00]
%                     delta = dstP(donor)*frac;
%                     trial = tx;
%                     pTry = dstP;
%                     pTry(donor) = pTry(donor)-delta;
%                     pTry(recipient) = pTry(recipient)+delta;
%                     trial = set_band_power(trial,u,destBand,pTry);
%                     Itest = aggregate_interference_by_channel(trial,Hdest,Gdest,destBand);
%                     if any(Itest(:)>Gdest(:).*(1+1e-8)),continue;end
%                     % Source band remains untouched and already protected.
%                     rateTry = optionB_total_rate(trial,HsuL,HsuU,hp2,hw2, ...
%                         P_PBS_j,P_WIFI_m,sigma2L,sigma2U,BscL,BscU);
%                     if isfinite(rateTry) && rateTry-baseline>bestGain+1e-7
%                         bestGain=rateTry-baseline;
%                         bestTx=trial;
%                         bestDelta=delta;
%                     end
%                 end
%                 if bestGain>0
%                     recipientSU=tx(u).users(srcIdx(r,1));
%                     donorSU=tx(u).users(dstIdx(donor,1));
%                     tx=bestTx;
%                     transferLog(end+1,:)=[u recipientSU donorSU 3-srcBandNum bestDelta bestGain/1e6]; %#ok<AGROW>
%                     if verbose, fprintf('OPTION B: UAV %d, SU %d receives %.6g W from SU %d on %s; network gain %.6g Mbps\n', ...
%                         u,recipientSU,bestDelta,donorSU,destBand,bestGain/1e6); end
%                     [~,dstP] = get_band_fields(tx,u,destBand);
%                 end
%             end
%         end
%     end
% end
% % Independent per-UAV power budgets and aggregate interference verification.
% for u=1:numel(tx)
%     assert(sum(tx(u).pLic(:))<=PmaxL*(1+1e-7));
%     assert(sum(tx(u).pUn(:))<=PmaxU*(1+1e-7));
% end
% assert(all(aggregate_interference_by_channel(tx,HpuL,GammaL,'lic')<=GammaL(:).*(1+1e-7)));
% assert(all(aggregate_interference_by_channel(tx,HwuU,GammaU,'un')<=GammaU(:).*(1+1e-7)));
% end
% 
% function total = optionB_total_rate(tx,HsuL,HsuU,hp2,hw2,P_PBS_j,P_WIFI_m, ...
%     sigma2L,sigma2U,BscL,BscU)
% total=compute_global_sum_rate_band_interuav(tx,HsuL,hp2,P_PBS_j,sigma2L,BscL,'lic')+ ...
%       compute_global_sum_rate_band_interuav(tx,HsuU,hw2,P_WIFI_m,sigma2U,BscU,'un');
% end
% 
% function r=diagnostic_tx_rate(tx,HL,HU,hp2,hw2,Ppbs,Pwifi,nL,nU,bL,bU)
% global AIIBPC_RATE_CALLS;
% if ~isempty(AIIBPC_RATE_CALLS), AIIBPC_RATE_CALLS=AIIBPC_RATE_CALLS+1; end
% r=compute_global_sum_rate_band_interuav(tx,HL,hp2,Ppbs,nL,bL,'lic')+ ...
%   compute_global_sum_rate_band_interuav(tx,HU,hw2,Pwifi,nU,bU,'un');
% end
% function name=diagnostic_mode_name(mode)
% if mode==0,name='safety-only';else,name='four-case';end
% end
% 
% function [tx,stats]=ai_ibpc_integrated_four_case_rateaware(raw,HP,HW,GL,GU,PL,PU,rateFn)
% % Integrated four-case AI-IBPC controller.
% % ------------------------------------------------------------
% % INPUT raw is the solution AFTER association, subchannel allocation,
% % CBF-MM beamforming and interference-unaware KKT power allocation.
% %
% % The four cases are determined from RAW aggregate interference:
% %   Case 1: licensed high, unlicensed safe.
% %   Case 2: unlicensed high, licensed safe.
% %   Case 3: neither high.
% %   Case 4: both high.
% %
% % Every accepted action must satisfy the corresponding PU/WU protection
% % threshold and the independent per-UAV power budget. The objective used
% % to select among tested feasible power actions is the TRUE network sum rate
% % supplied by rateFn. This is a rate-aware coordinate/candidate search; it is
% % not a proof of the global non-convex optimum.
% %
% % IMPORTANT: protection is integrated here. The caller must NOT apply a
% % separate safety-clipping/protection block after this function.
% 
% raw=enforce_per_uav_band_budget(raw,PL,'lic');
% raw=enforce_per_uav_band_budget(raw,PU,'un');
% IL0=aggregate_interference_by_channel(raw,HP,GL,'lic');
% IU0=aggregate_interference_by_channel(raw,HW,GU,'un');
% highL=any(IL0(:)>GL(:).*(1+1e-9));
% highU=any(IU0(:)>GU(:).*(1+1e-9));
% 
% stats=struct('caseId',0,'caseName','','rawRate',rateFn(raw),...
%     'finalRate',NaN,'licensedHigh',highL,'unlicensedHigh',highU,...
%     'licensedReduced_W',0,'unlicensedReduced_W',0,...
%     'licensedIncreased_W',0,'unlicensedIncreased_W',0);
% 
% pL0=arrayfun(@(z)sum(z.pLic(:)),raw);
% pU0=arrayfun(@(z)sum(z.pUn(:)),raw);
% 
% if highL && ~highU
%     % CASE 1: protect licensed side by rate-aware reduction. Then use only
%     % unused UNLICENSED-band budget/headroom when that increases sum rate.
%     stats.caseId=1; stats.caseName='licensed-high-only';
%     tx=rate_aware_band_projection_ratefn(raw,HP,GL,PL,'lic',rateFn);
%     tx=rate_aware_increase_band_only(tx,HW,GU,PU,'un',rateFn);
% 
% elseif ~highL && highU
%     % CASE 2: mirror of Case 1.
%     stats.caseId=2; stats.caseName='unlicensed-high-only';
%     tx=rate_aware_band_projection_ratefn(raw,HW,GU,PU,'un',rateFn);
%     tx=rate_aware_increase_band_only(tx,HP,GL,PL,'lic',rateFn);
% 
% elseif ~highL && ~highU
%     % CASE 3: already protected. Keep the KKT power solution unchanged.
%     stats.caseId=3; stats.caseName='neither-high';
%     tx=raw;
% 
% else
%     % CASE 4: both bands violate protection. Reduce BOTH bands using the
%     % same global sum-rate objective. Alternate the order and retain the
%     % better feasible protected result so the result is not tied to one
%     % arbitrary licensed-first/unlicensed-first ordering.
%     stats.caseId=4; stats.caseName='both-high';
% 
%     candLU=rate_aware_band_projection_ratefn(raw,HP,GL,PL,'lic',rateFn);
%     candLU=rate_aware_band_projection_ratefn(candLU,HW,GU,PU,'un',rateFn);
% 
%     candUL=rate_aware_band_projection_ratefn(raw,HW,GU,PU,'un',rateFn);
%     candUL=rate_aware_band_projection_ratefn(candUL,HP,GL,PL,'lic',rateFn);
% 
%     if ai_ibpc_is_feasible(candLU,HP,HW,GL,GU,PL,PU) && ...
%             (~ai_ibpc_is_feasible(candUL,HP,HW,GL,GU,PL,PU) || ...
%              rateFn(candLU)>=rateFn(candUL))
%         tx=candLU;
%     else
%         tx=candUL;
%     end
% end
% 
% % This is verification, NOT another protection operation.
% assert(ai_ibpc_is_feasible(tx,HP,HW,GL,GU,PL,PU),...
%     'Integrated AI-IBPC failed to produce a protected feasible solution.');
% 
% pLf=arrayfun(@(z)sum(z.pLic(:)),tx);
% pUf=arrayfun(@(z)sum(z.pUn(:)),tx);
% stats.licensedReduced_W=sum(max(0,pL0-pLf));
% stats.unlicensedReduced_W=sum(max(0,pU0-pUf));
% stats.licensedIncreased_W=sum(max(0,pLf-pL0));
% stats.unlicensedIncreased_W=sum(max(0,pUf-pU0));
% stats.finalRate=rateFn(tx);
% end
% 
% function tx=rate_aware_band_projection_ratefn(tx,HP,Gamma,Pmax,band,rateFn)
% % Protect one band using rate-aware power REDUCTIONS only.
% % For each unsafe protected subchannel, test proportional reduction and
% % deterministic single-stream reduction orders. Keep the feasible candidate
% % with the largest TRUE network sum rate.
% tx=enforce_per_uav_band_budget(tx,Pmax,band);
% for sc=1:numel(Gamma)
%     I=aggregate_interference_by_channel(tx,HP,Gamma,band);
%     if I(sc)<=Gamma(sc)*(1+1e-10),continue;end
% 
%     entries=zeros(0,4); % [u,ell,p,leakGain]
%     for u=1:numel(tx)
%         [idx,p,v]=get_band_fields(tx,u,band);
%         for ell=1:size(idx,1)
%             if idx(ell,2)==sc && p(ell)>0
%                 entries(end+1,:)=[u,ell,p(ell),...
%                     abs(HP{u,sc}'*v{ell})^2]; %#ok<AGROW>
%             end
%         end
%     end
%     if isempty(entries)
%         error('Unsafe protected channel has no active stream to reduce.');
%     end
% 
%     % Candidate A: proportional reduction of all streams on this SC.
%     scale=min(1,Gamma(sc)/max(I(sc),realmin));
%     best=tx;
%     for z=1:size(entries,1)
%         u=entries(z,1); ell=entries(z,2);
%         [~,p,~]=get_band_fields(best,u,band);
%         p(ell)=p(ell)*scale;
%         best=set_band_power(best,u,band,p);
%     end
%     bestRate=rateFn(best);
% 
%     % Candidate B: different deterministic stream-reduction orders.
%     for first=1:size(entries,1)
%         cand=tx;
%         order=[first,setdiff(1:size(entries,1),first,'stable')];
%         for oi=order
%             Icur=aggregate_interference_by_channel(cand,HP,Gamma,band);
%             if Icur(sc)<=Gamma(sc)*(1+1e-10),break;end
%             u=entries(oi,1); ell=entries(oi,2);
%             [~,p,~]=get_band_fields(cand,u,band);
%             gain=entries(oi,4);
%             dp=min(p(ell),(Icur(sc)-Gamma(sc))/max(gain,realmin));
%             p(ell)=max(0,p(ell)-dp);
%             cand=set_band_power(cand,u,band,p);
%         end
%         Ic=aggregate_interference_by_channel(cand,HP,Gamma,band);
%         if Ic(sc)>Gamma(sc)*(1+1e-7),continue;end
%         r=rateFn(cand);
%         if r>bestRate+1e-9
%             best=cand;
%             bestRate=r;
%         end
%     end
%     tx=best;
% end
% 
% % Verification only. The search above must itself have protected the band.
% I=aggregate_interference_by_channel(tx,HP,Gamma,band);
% assert(all(I(:)<=Gamma(:).*(1+1e-7)),...
%     'AI-IBPC rate-aware reduction did not satisfy aggregate protection.');
% end
% 
% function tx=rate_aware_increase_band_only(tx,HP,Gamma,Pmax,band,rateFn)
% % Increase powers ONLY on the safe opposite band and ONLY when the TRUE
% % network sum rate improves. Independent per-band budgets are preserved;
% % no watts are transferred from the violating band.
% for pass=1:2
%     improved=false;
%     for u=1:numel(tx)
%         [idx,p,v]=get_band_fields(tx,u,band);
%         for e=1:numel(p)
%             sc=idx(e,2);
%             I=aggregate_interference_by_channel(tx,HP,Gamma,band);
%             leak=abs(HP{u,sc}'*v{e})^2;
%             current=p(e);
%             budgetLeft=max(0,Pmax-sum(p));
%             headroom=max(0,Gamma(sc)-I(sc));
%             if leak<=1e-30
%                 addMax=budgetLeft;
%             else
%                 addMax=min(budgetLeft,headroom/leak);
%             end
%             if addMax<=1e-14,continue;end
%             upper=current+addMax;
%             baseRate=rateFn(tx);
%             trialP=p;
%             fun=@(power)ai_ibpc_trial_rate(tx,u,band,trialP,e,power,rateFn);
%             opts=optimset('TolX',max(1e-11,upper*1e-5),'Display','off');
%             pStar=fminbnd(@(power)-fun(power),current,upper,opts);
%             tests=unique([current;upper;pStar]);
%             scores=zeros(numel(tests),1);
%             for tt=1:numel(tests)
%                 scores(tt)=fun(tests(tt));
%             end
%             [bestRate,kbest]=max(scores);
%             if bestRate>baseRate+1e-9
%                 p(e)=tests(kbest);
%                 tx=set_band_power(tx,u,band,p);
%                 improved=true;
%             end
%         end
%     end
%     if ~improved,break;end
% end
% 
% I=aggregate_interference_by_channel(tx,HP,Gamma,band);
% assert(all(I(:)<=Gamma(:).*(1+1e-7)),...
%     'AI-IBPC opposite-band increase violated aggregate protection.');
% for u=1:numel(tx)
%     [~,p,~]=get_band_fields(tx,u,band);
%     assert(sum(p(:))<=Pmax*(1+1e-7),...
%         'AI-IBPC opposite-band increase violated per-UAV power budget.');
% end
% end
% 
% function r=ai_ibpc_trial_rate(tx,u,band,p,e,power,rateFn)
% p(e)=power;
% tx=set_band_power(tx,u,band,p);
% r=rateFn(tx);
% end
% 
% function ok=ai_ibpc_is_feasible(tx,HP,HW,GL,GU,PL,PU)
% IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
% IU=aggregate_interference_by_channel(tx,HW,GU,'un');
% ok=all(IL(:)<=GL(:).*(1+1e-7)) && ...
%    all(IU(:)<=GU(:).*(1+1e-7));
% for u=1:numel(tx)
%     ok=ok && sum(tx(u).pLic(:))<=PL*(1+1e-7) && ...
%              sum(tx(u).pUn(:))<=PU*(1+1e-7);
% end
% end
% 
% function [tx,stats]=rate_aware_protect_both(tx,HP,HW,GL,GU,PL,PU,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU)
% % Same protection for baseline and proposed modes; fixed beamformers.
% stats=struct();
% tx=rate_aware_band_projection(tx,HP,GL,PL,'lic',HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
% tx=rate_aware_band_projection(tx,HW,GU,PU,'un',HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
% end
% function tx=rate_aware_band_projection(tx,HP,Gamma,Pmax,band,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU)
% % For each unsafe protected channel, try feasible single-stream reductions
% % and proportional reduction, choose the candidate with highest TRUE rate.
% % Each candidate protects the CURRENT channel; later channel processing
% % does not increase power, so previous channels remain protected.
% tx=enforce_per_uav_band_budget(tx,Pmax,band);
% for sc=1:numel(Gamma)
%     I=aggregate_interference_by_channel(tx,HP,Gamma,band);
%     if I(sc)<=Gamma(sc)*(1+1e-10),continue;end
%     entries=zeros(0,4); % [u,ell,p,leakGain]
%     for u=1:numel(tx)
%         [idx,p,v]=get_band_fields(tx,u,band);
%         for ell=1:size(idx,1)
%             if idx(ell,2)==sc && p(ell)>0
%                 entries(end+1,:)=[u,ell,p(ell),abs(HP{u,sc}'*v{ell})^2]; %#ok<AGROW>
%             end
%         end
%     end
%     if isempty(entries),continue;end
%     scale=Gamma(sc)/max(I(sc),realmin);
%     best=tx;
%     for z=1:size(entries,1)
%         u=entries(z,1);ell=entries(z,2);
%         [~,p,~]=get_band_fields(best,u,band);
%         p(ell)=p(ell)*scale;
%         best=set_band_power(best,u,band,p);
%     end
%     bestRate=diagnostic_tx_rate(best,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
%     % Greedy order search: each iteration removes enough interference from
%     % one stream to meet threshold, or zeroes it if that is insufficient.
%     for first=1:size(entries,1)
%         cand=tx;
%         order=[first,setdiff(1:size(entries,1),first,'stable')];
%         for oi=order
%             Icur=aggregate_interference_by_channel(cand,HP,Gamma,band);
%             if Icur(sc)<=Gamma(sc)*(1+1e-10),break;end
%             u=entries(oi,1);ell=entries(oi,2);
%             [~,p,~]=get_band_fields(cand,u,band);
%             gain=entries(oi,4);
%             dp=min(p(ell),(Icur(sc)-Gamma(sc))/max(gain,realmin));
%             p(ell)=max(0,p(ell)-dp);
%             cand=set_band_power(cand,u,band,p);
%         end
%         Ic=aggregate_interference_by_channel(cand,HP,Gamma,band);
%         if Ic(sc)>Gamma(sc)*(1+1e-7),continue;end
%         r=diagnostic_tx_rate(cand,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
%         if r>bestRate+1e-9
%             best=cand;bestRate=r;
%         end
%     end
%     tx=best;
% end
% % Final guard against numerical errors.
% tx=project_aggregate_band_interference(tx,HP,Gamma,Pmax,band);
% end
% function tx=compensate_original_violators(tx,raw,HP,HW,GL,GU,PL,PU,unused)
% % Four-case flags from PRE-protection raw aggregate interference. Only
% % opposite-band increases from UNUSED SAME-BAND budget are allowed.
% IL=aggregate_interference_by_channel(raw,HP,GL,'lic');
% IU=aggregate_interference_by_channel(raw,HW,GU,'un');
% for u=1:numel(tx)
%     for loc=1:numel(tx(u).users)
%         [idxL,~,~]=get_band_fields(raw,u,'lic');
%         [idxU,~,~]=get_band_fields(raw,u,'un');
%         eL=find(idxL(:,1)==loc,1);
%         eU=find(idxU(:,1)==loc,1);
%         highL=~isempty(eL)&&IL(idxL(eL,2))>GL(idxL(eL,2))*(1+1e-9);
%         highU=~isempty(eU)&&IU(idxU(eU,2))>GU(idxU(eU,2))*(1+1e-9);
%         if highL && ~highU && ~isempty(eU)
%             [tx,~]=increase_su_stream_if_safe(tx,u,eU,HW,GU,PU,'un');
%         elseif highU && ~highL && ~isempty(eL)
%             [tx,~]=increase_su_stream_if_safe(tx,u,eL,HP,GL,PL,'lic');
%         end
%     end
% end
% tx=apply_final_safety_clipping(tx,HP,HW,GL,GU,PL,PU,'both');
% end
% 
% function [best,info]=fixed_order_alternating_power(raw,safe,HP,HW,GL,GU,PL,PU,rateFn,opts,maxSweeps,order)
% % Fixed-order AO control: exactly the same optimizer and stopping rule.
% % Both bands initially assigned. Cases are defined from RAW aggregate
% % interference; they determine the first block and whether both blocks
% % are activated. No power is transferred across bands or between SUs.
% % Optimizes the TRUE coupled sum rate at each accepted block step.
% caseCounts=zeros(1,4);
% best=safe; bestRate=rateFn(best);accepted=0;history=bestRate/1e6;
% solverCalls=0;totalIterations=0;totalEvaluations=0;
% for sweep=1:maxSweeps
%     previous=bestRate;
%     for b=order
%         [candidate,flag,solverStat]=case_optimize_one_band(raw,best,b,HP,HW,GL,GU,PL,PU,rateFn,opts);
%         solverCalls=solverCalls+solverStat.calls;
%         totalIterations=totalIterations+solverStat.iterations;
%         totalEvaluations=totalEvaluations+solverStat.evaluations;
%         r=rateFn(candidate);
%         if flag>=0 && isfinite(r) && r>bestRate+1e-5
%             best=candidate;bestRate=r;accepted=accepted+1;
%         end
%     end
%     history(end+1)=bestRate/1e6; %#ok<AGROW>
%     if bestRate-previous<1e-4,break;end
% end
% IL=aggregate_interference_by_channel(best,HP,GL,'lic');
% IU=aggregate_interference_by_channel(best,HW,GU,'un');
% assert(all(IL(:)<=GL(:)*(1+1e-5))&&all(IU(:)<=GU(:)*(1+1e-5)), ...
%     'Case-conditioned optimizer violated aggregate interference');
% for u=1:numel(best)
%     assert(sum(best(u).pLic)<=PL*(1+1e-5));
%     assert(sum(best(u).pUn)<=PU*(1+1e-5));
% end
% info=struct('nCase1',caseCounts(1),'nCase2',caseCounts(2), ...
%     'nCase3',caseCounts(3),'nCase4',caseCounts(4), ...
%     'acceptedSteps',accepted,'history_Mbps',history,'order',order, ...
%     'solverCalls',solverCalls,'totalIterations',totalIterations, ...
%     'totalEvaluations',totalEvaluations);
% end
% 
% function plotOrderAblation(x,Y,names,ylabelText,filename)
% figure('Color','w','Position',[100 100 620 470]);
% styles={'s-','o--','^-','d-.'};
% for i=1:size(Y,2)
%     plot(x,Y(:,i),styles{i},'LineWidth',2,'MarkerSize',7);hold on;
% end
% ax=gca;set(ax,'FontName','Times New Roman','FontSize',16, ...
%     'FontWeight','normal','LineWidth',1.0,'Position',[0.13 0.15 0.84 0.66]);
% grid on;ax.GridLineStyle=':';ax.GridAlpha=0.25;
% xlabel('Paired channel realization');ylabel(ylabelText);
% legend(names,'FontName','Times New Roman','FontSize',11, ...
%     'NumColumns',2,'Location','northoutside');
% saveas(gcf,filename);
% end
% 
% function [best,info]=case_conditioned_alternating_power(raw,safe,HP,HW,GL,GU,PL,PU,rateFn,opts,maxSweeps)
% % A separate, feasible case-conditioned block-coordinate optimizer.
% % Both bands initially assigned. Cases are defined from RAW aggregate
% % interference; they determine the first block and whether both blocks
% % are activated. No power is transferred across bands or between SUs.
% % Optimizes the TRUE coupled sum rate at each accepted block step.
% IL=aggregate_interference_by_channel(raw,HP,GL,'lic');
% IU=aggregate_interference_by_channel(raw,HW,GU,'un');
% caseCounts=zeros(1,4);
% for u=1:numel(raw)
%     for loc=1:numel(raw(u).users)
%         [iL,~,~]=get_band_fields(raw,u,'lic');
%         [iU,~,~]=get_band_fields(raw,u,'un');
%         eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
%         highL=~isempty(eL)&&IL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
%         highU=~isempty(eU)&&IU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
%         if highL&&highU,caseCounts(3)=caseCounts(3)+1;
%         elseif highL,caseCounts(1)=caseCounts(1)+1;
%         elseif highU,caseCounts(2)=caseCounts(2)+1;
%         else,caseCounts(4)=caseCounts(4)+1;end
%     end
% end
% % Begin at safe powers. The order is determined by the active case flags.
% % Even safe bands can be optimized because coupled SINR may benefit.
% if caseCounts(2)>caseCounts(1),order=[2 1];else,order=[1 2];end
% best=safe; bestRate=rateFn(best);accepted=0;history=bestRate/1e6;
% solverCalls=0;totalIterations=0;totalEvaluations=0;
% for sweep=1:maxSweeps
%     previous=bestRate;
%     for b=order
%         [candidate,flag,solverStat]=case_optimize_one_band(raw,best,b,HP,HW,GL,GU,PL,PU,rateFn,opts);
%         solverCalls=solverCalls+solverStat.calls;
%         totalIterations=totalIterations+solverStat.iterations;
%         totalEvaluations=totalEvaluations+solverStat.evaluations;
%         r=rateFn(candidate);
%         if flag>=0 && isfinite(r) && r>bestRate+1e-5
%             best=candidate;bestRate=r;accepted=accepted+1;
%         end
%     end
%     history(end+1)=bestRate/1e6; %#ok<AGROW>
%     if bestRate-previous<1e-4,break;end
% end
% IL=aggregate_interference_by_channel(best,HP,GL,'lic');
% IU=aggregate_interference_by_channel(best,HW,GU,'un');
% assert(all(IL(:)<=GL(:)*(1+1e-5))&&all(IU(:)<=GU(:)*(1+1e-5)), ...
%     'Case-conditioned optimizer violated aggregate interference');
% for u=1:numel(best)
%     assert(sum(best(u).pLic)<=PL*(1+1e-5));
%     assert(sum(best(u).pUn)<=PU*(1+1e-5));
% end
% info=struct('nCase1',caseCounts(1),'nCase2',caseCounts(2), ...
%     'nCase3',caseCounts(3),'nCase4',caseCounts(4), ...
%     'acceptedSteps',accepted,'history_Mbps',history,'order',order, ...
%     'solverCalls',solverCalls,'totalIterations',totalIterations, ...
%     'totalEvaluations',totalEvaluations);
% end
% 
% function [best,flag,solverStat]=case_optimize_one_band(raw,current,b,HP,HW,GL,GU,PL,PU,rateFn,opts)
% % Optimize one band, fixing the other. Linear per-UAV and per-SC
% % constraints; full nonlinear inter-user sum rate in objective.
% map=zeros(0,2); x0=[];
% for u=1:numel(current)
%     if b==1,p=current(u).pLic;else,p=current(u).pUn;end
%     for ell=1:numel(p)
%         map(end+1,:)=[u,ell]; %#ok<AGROW>
%         x0(end+1,1)=p(ell); %#ok<AGROW>
%     end
% end
% best=current;flag=0;solverStat=struct('calls',0,'iterations',0,'evaluations',0);
% if isempty(map),return;end
% if b==1, Gamma=GL; HPband=HP; Pmax=PL;band='lic';
% else,Gamma=GU;HPband=HW;Pmax=PU;band='un';end
% A=zeros(numel(current)+numel(Gamma),numel(x0));
% rhs=[Pmax*ones(numel(current),1);Gamma(:)];
% for z=1:size(map,1)
%     u=map(z,1);ell=map(z,2);
%     if b==1,sc=raw(u).idxLic(ell,2);v=raw(u).vLic{ell};
%     else,sc=raw(u).idxUn(ell,2);v=raw(u).vUn{ell};end
%     A(u,z)=1;
%     A(numel(current)+sc,z)=abs(HPband{u,sc}'*v)^2;
% end
% for row=numel(current)+1:size(A,1)
%     sca=max(rhs(row),realmin);
%     A(row,:)=A(row,:)/sca;rhs(row)=rhs(row)/sca;
% end
% makeTx=@(p) case_set_band_vector(current,map,p,b);
% objective=@(p) -rateFn(makeTx(p))/1e6;
% try
%     [p,~,ef,solverOutput]=fmincon(objective,x0,A,rhs,[],[],zeros(size(x0)), ...
%         Pmax*ones(size(x0)),[],opts);
%     solverStat.calls=1;
%     solverStat.iterations=solverOutput.iterations;
%     solverStat.evaluations=solverOutput.funcCount;
%     if all(A*p<=rhs+1e-7)&&all(p>=-1e-9)
%         cand=makeTx(p);
%         if rateFn(cand)>rateFn(current)+1e-5
%             best=cand;flag=ef;
%         end
%     end
% catch ME
%     warning('Case-conditioned band %s failed: %s',band,ME.message);
% end
% end
% 
% function tx=case_set_band_vector(tx,map,p,b)
% for z=1:numel(p)
%     u=map(z,1);ell=map(z,2);
%     if b==1,tx(u).pLic(ell)=p(z);
%     else,tx(u).pUn(ell)=p(z);end
% end
% end
% 
% function [best,info]=joint_power_fixed_beams(raw,safe,HP,HW,GL,GU,PL,PU,rateFn,opts)
% % Joint licensed+unlicensed power optimization for FIXED assignments/beams.
% % fmincon handles true global SINR objective; all constraints are linear.
% % SQP is local, not a guarantee of global optimality.
% map=zeros(0,3); p0=[]; ps=[];
% for u=1:numel(raw)
%     for b=1:2
%         if b==1, p=raw(u).pLic; q=safe(u).pLic;
%         else, p=raw(u).pUn; q=safe(u).pUn; end
%         for ell=1:numel(p)
%             map(end+1,:)=[u,b,ell]; %#ok<AGROW>
%             p0(end+1,1)=p(ell); %#ok<AGROW>
%             ps(end+1,1)=q(ell); %#ok<AGROW>
%         end
%     end
% end
% if isempty(map),best=safe;info=struct('exitflag',0,'maxViolation',0, ...
%     'solverCalls',0,'totalIterations',0,'totalEvaluations',0);return;end
% rows=2*numel(raw)+numel(GL)+numel(GU);
% A=zeros(rows,numel(p0)); rhs=[repmat([PL;PU],numel(raw),1);GL(:);GU(:)];
% for z=1:size(map,1)
%     u=map(z,1);b=map(z,2);ell=map(z,3);
%     A(2*(u-1)+b,z)=1;
%     if b==1
%         sc=raw(u).idxLic(ell,2);v=raw(u).vLic{ell};
%         A(2*numel(raw)+sc,z)=abs(HP{u,sc}'*v)^2;
%     else
%         sc=raw(u).idxUn(ell,2);v=raw(u).vUn{ell};
%         A(2*numel(raw)+numel(GL)+sc,z)=abs(HW{u,sc}'*v)^2;
%     end
% end
% % Normalize tiny interference constraints to improve SQP conditioning.
% for row=2*numel(raw)+1:rows
%     sca=max(rhs(row),realmin);A(row,:)=A(row,:)/sca;rhs(row)=rhs(row)/sca;
% end
% % Safety allocation is a guaranteed feasible warm start.
% objective=@(p) -rateFn(joint_tx_from_vector(raw,map,p))/1e6;
% starts=[ps,zeros(size(ps))];
% % Use raw as additional start ONLY if already feasible.
% if all(A*p0<=rhs+1e-8),starts=[starts,p0];end
% best=safe; bestRate=rateFn(best);bestFlag=0;
% solverCalls=0;totalIterations=0;totalEvaluations=0;
% for k=1:size(starts,2)
%     x=starts(:,k);
%     try
%         [p,~,ef,solverOutput]=fmincon(objective,x,A,rhs,[],[], ...
%             zeros(size(p0)),repmat(max(PL,PU),size(p0)),[],opts);
%         solverCalls=solverCalls+1;
%         totalIterations=totalIterations+solverOutput.iterations;
%         totalEvaluations=totalEvaluations+solverOutput.funcCount;
%         viol=max([A*p-rhs;-p]);
%         if viol<=1e-6
%             cand=joint_tx_from_vector(raw,map,p); r=rateFn(cand);
%             if isfinite(r)&&r>bestRate+1e-5
%                 best=cand;bestRate=r;bestFlag=ef;
%             end
%         end
%     catch ME
%         warning('Joint power start %d failed: %s',k,ME.message);
%     end
% end
% % Final exact power/leakage check in physical units.
% IL=aggregate_interference_by_channel(best,HP,GL,'lic');
% IU=aggregate_interference_by_channel(best,HW,GU,'un');
% assert(all(IL(:)<=GL(:)*(1+1e-5))&&all(IU(:)<=GU(:)*(1+1e-5)));
% for u=1:numel(best)
%     assert(sum(best(u).pLic)<=PL*(1+1e-5));
%     assert(sum(best(u).pUn)<=PU*(1+1e-5));
% end
% info=struct('exitflag',bestFlag,'maxViolation', ...
%     max([max(IL(:)./GL(:))-1,max(IU(:)./GU(:))-1,0]), ...
%     'solverCalls',solverCalls,'totalIterations',totalIterations, ...
%     'totalEvaluations',totalEvaluations);
% end
% function tx=joint_tx_from_vector(raw,map,p)
% tx=raw;
% for z=1:numel(p)
%     u=map(z,1);ell=map(z,3);
%     if map(z,2)==1,tx(u).pLic(ell)=p(z);
%     else,tx(u).pUn(ell)=p(z);end
% end
% end
% 
% function [best,info]=enhance_case_ao_power(raw,safe,original,originalInfo,HP,HW,GL,GU,PL,PU,rateFn,opts)
% % Feasible, rate-monotone refinement of original four-case alternating AO.
% % 1. Cheap all-UAV band shutdown candidates (both licensed and unlicensed).
% % 2. For promising shutdowns, rerun ORIGINAL case-conditioned AO from the
% %    feasible shutdown state, and retain only a strictly better solution.
% % 3. If original AO was below a simple leakage-aware start, refine that start.
% % No cross-band transfer, no change in association/scheduling/beamforming.
% best=original;bestRate=rateFn(best);
% info=struct('solverCalls',0,'totalIterations',0,'totalEvaluations',0, ...
%     'acceptedShutdowns',0,'testedShutdowns',0,'acceptedRestarts',0, ...
%     'history_Mbps',bestRate/1e6);
% % Test every active UAV-band shutdown; one quick objective evaluation each.
% % Restart only from candidates that are competitive before reoptimization.
% for u=1:numel(best)
%     for b=1:2
%         trial=best;
%         if b==1
%             if ~any(trial(u).pLic>1e-12),continue;end
%             trial(u).pLic(:)=0;
%         else
%             if ~any(trial(u).pUn>1e-12),continue;end
%             trial(u).pUn(:)=0;
%         end
%         info.testedShutdowns=info.testedShutdowns+1;
%         r0=rateFn(trial);
%         % A shutdown is always feasible when the original solution is.
%         if r0>bestRate+1e-5
%             best=trial;bestRate=r0;info.acceptedShutdowns=info.acceptedShutdowns+1;
%             info.history_Mbps(end+1)=bestRate/1e6;
%         end
%         % Refinement from a shutdown can uncover a different AO basin.
%         % Use a bounded number of iterations to preserve the time advantage.
%         if r0>=bestRate-0.25e6
%             [candidate,ci]=case_conditioned_alternating_power( ...
%                 raw,trial,HP,HW,GL,GU,PL,PU,rateFn,opts,3);
%             info.solverCalls=info.solverCalls+ci.solverCalls;
%             info.totalIterations=info.totalIterations+ci.totalIterations;
%             info.totalEvaluations=info.totalEvaluations+ci.totalEvaluations;
%             rc=rateFn(candidate);
%             if rc>bestRate+1e-5
%                 best=candidate;bestRate=rc;info.acceptedRestarts=info.acceptedRestarts+1;
%                 info.history_Mbps(end+1)=bestRate/1e6;
%             end
%         end
%     end
% end
% % A low-cost leakage-aware feasible multistart, constructed by reducing
% % powers on the highest leakage-weighted streams before safety projection.
% % Only use this extra start when original AO had a limited initial gain.
% if rateFn(original)-rateFn(safe)<0.25e6
%     trial=safe;
%     for u=1:numel(trial)
%         for b=1:2
%             if b==1,idx=trial(u).idxLic; pp=trial(u).pLic; HH=HP; gg=GL;
%             else,idx=trial(u).idxUn; pp=trial(u).pUn; HH=HW; gg=GU;end
%             for e=1:numel(pp)
%                 sc=idx(e,2);
%                 if b==1,v=trial(u).vLic{e};else,v=trial(u).vUn{e};end
%                 leakage=pp(e)*abs(HH{u,sc}'*v)^2/max(gg(sc),realmin);
%                 if leakage>0.15,pp(e)=0.5*pp(e);end
%             end
%             if b==1,trial(u).pLic=pp;else,trial(u).pUn=pp;end
%         end
%     end
%     [candidate,ci]=case_conditioned_alternating_power( ...
%         raw,trial,HP,HW,GL,GU,PL,PU,rateFn,opts,3);
%     info.solverCalls=info.solverCalls+ci.solverCalls;
%     info.totalIterations=info.totalIterations+ci.totalIterations;
%     info.totalEvaluations=info.totalEvaluations+ci.totalEvaluations;
%     if rateFn(candidate)>bestRate+1e-5
%         best=candidate;bestRate=rateFn(candidate);
%         info.acceptedRestarts=info.acceptedRestarts+1;
%         info.history_Mbps(end+1)=bestRate/1e6;
%     end
% end
% IL=aggregate_interference_by_channel(best,HP,GL,'lic');
% IU=aggregate_interference_by_channel(best,HW,GU,'un');
% assert(all(IL(:)<=GL(:)*(1+1e-5))&&all(IU(:)<=GU(:)*(1+1e-5)), ...
%     'Enhanced AI-IBPC violates aggregate protection');
% for u=1:numel(best)
%     assert(sum(best(u).pLic)<=PL*(1+1e-5) && ...
%         sum(best(u).pUn)<=PU*(1+1e-5), ...
%         'Enhanced AI-IBPC violates an independent per-band budget');
% end
% assert(bestRate>=rateFn(original)-1e-5,'Enhanced result reduced original rate');
% info.originalIterations=originalInfo.totalIterations;
% end
% 
% 
% function [best,info]=refine_association_full_rebuild( ...
%     incumbent,assoc0,maxCandidates,Qslot,WS,WP,WW,w_b,w_wifi,H, ...
%     f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow, ...
%     P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn, ...
%     Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un, ...
%     seedNow,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter, ...
%     MM_leakFrac,MM_reg_alpha,methodMode,HP,HW,HL,HU,hp,hw,rateFn,opts)
% % One-user relocation. Full rebuilding ensures the moved user is not
% % evaluated with stale Hungarian subchannels or stale beamforming vectors.
% % Candidate channel realizations, geometry and random seed are held fixed.
% % This is a discrete local search; it does not guarantee global optimality.
% U=size(Qslot,1); K=numel(assoc0);
% info=struct('tested',0,'accepted',0,'moves',zeros(0,3), ...
%     'history_Mbps',rateFn(incumbent)/1e6);
% best=incumbent;bestRate=rateFn(best);
% % Use one-move neighborhood around the ORIGINAL association, so results
% % remain comparable and no multi-move geometry-induced effects are hidden.
% for k=1:K
%     for newU=1:U
%         if newU==assoc0(k),continue;end
%         if info.tested>=maxCandidates,return;end
%         assocTry=assoc0;assocTry(k)=newU;
%         % The helper runs one full resource-allocation sweep with a forced
%         % association. The stage diagnostic returns its pre-control RAW tx.
%         [~,~,~,~,~,dd]=eval_slot_multi_qos_bf( ...
%             Qslot,WS,WP,WW,w_b,w_wifi,H, ...
%             f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow, ...
%             P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn, ...
%             Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,1, ...
%             seedNow,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter, ...
%             MM_leakFrac,MM_reg_alpha,methodMode,1,true,assocTry);
%         info.tested=info.tested+1;
%         if ~isfield(dd,'rawTx') || isempty(dd.rawTx)
%             error('Candidate raw allocation unavailable for reassociation');
%         end
%         candidateRaw=dd.rawTx;
%         [candidateSafe,~]=rate_aware_protect_both( ...
%             candidateRaw,HP,HW,GammaLic,GammaUn,Pmax_lic,Pmax_un, ...
%             HL,HU,hp,hw,P_PBS_j,P_WIFI_m, ...
%             sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);
%         [candidateCase,ci]=case_conditioned_alternating_power( ...
%             candidateRaw,candidateSafe,HP,HW,GammaLic,GammaUn, ...
%             Pmax_lic,Pmax_un,rateFn,opts,5);
%         % Only run the more expensive shutdown/multistart stage if this
%         % candidate is within 0.25 Mbps of the current best.
%         if rateFn(candidateCase)>=bestRate-0.25e6
%             [candidate,~]=enhance_case_ao_power( ...
%                 candidateRaw,candidateSafe,candidateCase,ci, ...
%                 HP,HW,GammaLic,GammaUn,Pmax_lic,Pmax_un,rateFn,opts);
%         else
%             candidate=candidateCase;
%         end
%         IL=aggregate_interference_by_channel(candidate,HP,GammaLic,'lic');
%         IU=aggregate_interference_by_channel(candidate,HW,GammaUn,'un');
%         feasible=all(IL(:)<=GammaLic(:)*(1+1e-5)) && ...
%             all(IU(:)<=GammaUn(:)*(1+1e-5));
%         for uu=1:U
%             feasible=feasible && ...
%                 sum(candidate(uu).pLic(:))<=Pmax_lic*(1+1e-5) && ...
%                 sum(candidate(uu).pUn(:))<=Pmax_un*(1+1e-5);
%         end
%         r=rateFn(candidate);
%         if feasible && isfinite(r) && r>bestRate+1e-5
%             best=candidate;bestRate=r;
%             info.accepted=info.accepted+1;
%             info.moves(end+1,:)=[k,assoc0(k),newU];
%             info.history_Mbps(end+1)=bestRate/1e6;
%         end
%     end
% end
% end
% 
% 
% function [tx,st]=pure_refined_fourcase(raw,HP,HW,GL,GU,PL,PU,rateFn,allowComp)
% if nargin<9,allowComp=true;end
% % Case-specific feasible power reduction. Only affected SU can gain power
% % on the OPPOSITE band, and only when that band's budget/headroom permits.
% % Case 4 is left unchanged unless another SU's necessary reductions affect it.
% % All reductions are selected by evaluating the ACTUAL coupled network rate.
% tx=enforce_per_uav_band_budget(raw,PL,'lic');
% tx=enforce_per_uav_band_budget(tx,PU,'un');
% IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
% IU=aggregate_interference_by_channel(tx,HW,GU,'un');
% st=struct('cases',zeros(1,4),'compL',0,'compU',0, ...
%     'blockedBudget',0,'tested',0,'accepted',0);
% % Freeze the original case membership before any control changes.
% for u=1:numel(tx)
%     for loc=1:numel(tx(u).users)
%         [iL,~,~]=get_band_fields(tx,u,'lic');
%         [iU,~,~]=get_band_fields(tx,u,'un');
%         eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
%         highL=~isempty(eL)&&IL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
%         highU=~isempty(eU)&&IU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
%         if highL&&highU,st.cases(3)=st.cases(3)+1;
%         elseif highL,st.cases(1)=st.cases(1)+1;
%         elseif highU,st.cases(2)=st.cases(2)+1;
%         else,st.cases(4)=st.cases(4)+1;end
%     end
% end
% % Fix overloaded channels. Each trial removes ONLY necessary power;
% % enumerate which stream bears reduction, then try same-SU compensation.
% % A full aggregate projection is used as the initial feasible fallback.
% % To avoid comparing a feasible result to infeasible intermediate points,
% % construct complete candidates from RAW and rank their final rates.
% % Coordinate descent over the order of violating streams (fixed association).
% for pass=1:3
%     changed=false;
%     for b=1:2
%         if b==1,band='lic';other='un';HPb=HP;G=GL;Po=PU;Hother=HW;Go=GU;
%         else,band='un';other='lic';HPb=HW;G=GU;Po=PL;Hother=HP;Go=GL;end
%         for sc=1:numel(G)
%             I=aggregate_interference_by_channel(tx,HPb,G,band);
%             if I(sc)<=G(sc)*(1+1e-9),continue;end
%             % Include proportional clipping as feasible fallback for this SC.
%             fallback=tx;
%             scale=G(sc)/max(I(sc),realmin);
%             for u=1:numel(tx)
%                 [idx,p,~]=get_band_fields(fallback,u,band);
%                 hit=find(idx(:,2)==sc);
%                 if ~isempty(hit)
%                     p(hit)=p(hit)*scale;
%                     fallback=set_band_power(fallback,u,band,p);
%                 end
%             end
%             best=fallback;bestRate=rateFn(best);
%             % Test each offending stream as first reduction target.
%             for firstU=1:numel(tx)
%                 [idx,p,~]=get_band_fields(tx,firstU,band);
%                 for firstE=find(idx(:,2)==sc).'
%                     trial=tx;
%                     [trial,dp]=reduce_su_stream_to_aggregate_limit( ...
%                         trial,firstU,firstE,HPb,G,band);
%                     if dp<=0,continue;end
%                     % Complete any residual protection on this SC only.
%                     It=aggregate_interference_by_channel(trial,HPb,G,band);
%                     if It(sc)>G(sc)*(1+1e-9)
%                         remScale=G(sc)/max(It(sc),realmin);
%                         for uu=1:numel(trial)
%                             [ii,pp,~]=get_band_fields(trial,uu,band);
%                             ee=find(ii(:,2)==sc);
%                             if ~isempty(ee)
%                                 pp(ee)=pp(ee)*remScale;
%                                 trial=set_band_power(trial,uu,band,pp);
%                             end
%                         end
%                     end
%                     % Compensation is permitted only for this affected SU
%                     % and only when it was initially in case 1 or case 2.
%                     loc=idx(firstE,1);
%                     [ii,pp,~]=get_band_fields(trial,firstU,other);
%                     oppositeE=find(ii(:,1)==loc,1);
%                     IrawL=aggregate_interference_by_channel(raw,HP,GL,'lic');
%                     IrawU=aggregate_interference_by_channel(raw,HW,GU,'un');
%                     [rawL,~,~]=get_band_fields(raw,firstU,'lic');
%                     [rawU,~,~]=get_band_fields(raw,firstU,'un');
%                     el=find(rawL(:,1)==loc,1);eu=find(rawU(:,1)==loc,1);
%                     hL=~isempty(el)&&IrawL(rawL(el,2))>GL(rawL(el,2))*(1+1e-9);
%                     hU=~isempty(eu)&&IrawU(rawU(eu,2))>GU(rawU(eu,2))*(1+1e-9);
%                     canComp=(b==1 && hL && ~hU)||(b==2 && hU && ~hL);
%                     if allowComp && canComp && ~isempty(oppositeE)
%                         [trial,added]=pure_marginal_compensate(trial,firstU, ...
%                             oppositeE,Hother,Go,Po,other,rateFn);
%                         if added==0,st.blockedBudget=st.blockedBudget+1;end
%                     end
%                     st.tested=st.tested+1;
%                     trialRate=rateFn(trial);
%                     if trialRate>bestRate+1e-6
%                         best=trial;bestRate=trialRate;
%                     end
%                 end
%             end
%             % Every accepted change eliminates violation on this channel.
%             tx=best;changed=true;st.accepted=st.accepted+1;
%         end
%     end
%     if ~changed,break;end
% end
% % Optional compensation AFTER protection for original case-1/2 SUs:
% % evaluate actual rate and reject any harmful compensation.
% IrawL=aggregate_interference_by_channel(raw,HP,GL,'lic');
% IrawU=aggregate_interference_by_channel(raw,HW,GU,'un');
% for u=1:numel(tx)
%     for loc=1:numel(tx(u).users)
%         [iL,~,~]=get_band_fields(raw,u,'lic');
%         [iU,~,~]=get_band_fields(raw,u,'un');
%         eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
%         hL=~isempty(eL)&&IrawL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
%         hU=~isempty(eU)&&IrawU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
%         if allowComp && hL && ~hU && ~isempty(eU)
%             [t,dp]=pure_marginal_compensate(tx,u,eU,HW,GU,PU,'un',rateFn);
%             if rateFn(t)>rateFn(tx)+1e-6,tx=t;st.compU=st.compU+dp;end
%         elseif allowComp && hU && ~hL && ~isempty(eL)
%             [t,dp]=pure_marginal_compensate(tx,u,eL,HP,GL,PL,'lic',rateFn);
%             if rateFn(t)>rateFn(tx)+1e-6,tx=t;st.compL=st.compL+dp;end
%         end
%     end
% end
% tx=apply_final_safety_clipping(tx,HP,HW,GL,GU,PL,PU,'both');
% end
% 
% function [tx,st]=pure_post_compensate(base,raw,HP,HW,GL,GU,PL,PU,rateFn,mode)
% % Both policies start from exactly the same reduction-only base.
% % Generic: same-user opposite-band compensation whenever OWN stream was
% % reduced, regardless of original interference case. Case: only original
% % case 1 or case 2, on its safe opposite band. Same marginal solver.
% % No donor-user power, no cross-band budget transfer, no reserve.
% tx=base;
% IrawL=aggregate_interference_by_channel(raw,HP,GL,'lic');
% IrawU=aggregate_interference_by_channel(raw,HW,GU,'un');
% st=struct('cases',zeros(1,4),'compL',0,'compU',0, ...
%     'eligible',0,'blocked',0,'accepted',0);
% for u=1:numel(raw)
%     for loc=1:numel(raw(u).users)
%         [iL,pL,~]=get_band_fields(raw,u,'lic');
%         [iU,pU,~]=get_band_fields(raw,u,'un');
%         [bL,qL,~]=get_band_fields(base,u,'lic');
%         [bU,qU,~]=get_band_fields(base,u,'un');
%         eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
%         fL=find(bL(:,1)==loc,1);fU=find(bU(:,1)==loc,1);
%         highL=~isempty(eL)&&IrawL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
%         highU=~isempty(eU)&&IrawU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
%         if highL&&highU,st.cases(3)=st.cases(3)+1;
%         elseif highL,st.cases(1)=st.cases(1)+1;
%         elseif highU,st.cases(2)=st.cases(2)+1;
%         else,st.cases(4)=st.cases(4)+1;end
%         reducedL=~isempty(eL)&&~isempty(fL)&&pL(eL)>qL(fL)+1e-12;
%         reducedU=~isempty(eU)&&~isempty(fU)&&pU(eU)>qU(fU)+1e-12;
%         if strcmp(mode,'case')
%             wantU=highL && ~highU && reducedL;
%             wantL=highU && ~highL && reducedU;
%         else
%             wantU=reducedL;
%             wantL=reducedU;
%         end
%         % One user may qualify for both directions in generic mode.
%         % Fixed order in both methods; only eligibility differs.
%         if wantU && ~isempty(fU)
%             st.eligible=st.eligible+1;
%             [trial,dp]=pure_marginal_compensate(tx,u,fU,HW,GU,PU,'un',rateFn);
%             if dp>1e-12
%                 tx=trial;st.compU=st.compU+dp;st.accepted=st.accepted+1;
%             else
%                 st.blocked=st.blocked+1;
%             end
%         end
%         if wantL && ~isempty(fL)
%             st.eligible=st.eligible+1;
%             [trial,dp]=pure_marginal_compensate(tx,u,fL,HP,GL,PL,'lic',rateFn);
%             if dp>1e-12
%                 tx=trial;st.compL=st.compL+dp;st.accepted=st.accepted+1;
%             else
%                 st.blocked=st.blocked+1;
%             end
%         end
%     end
% end
% end
% 
% function [best,added]=pure_marginal_compensate(tx,u,ell,Hprot,Gamma,Pmax,band,rateFn)
% best=tx;added=0;
% [idx,p,v]=get_band_fields(tx,u,band);
% if ell>numel(p),return;end
% s=idx(ell,2);I=aggregate_interference_by_channel(tx,Hprot,Gamma,band);
% g=abs(Hprot{u,s}'*v{ell})^2;
% unused=max(0,Pmax-sum(p));
% head=max(0,Gamma(s)-I(s));
% if I(s)>Gamma(s)*(1+1e-9),return;end
% if g>1e-30,cap=min(unused,head/g);else,cap=unused;end
% if cap<=1e-12,return;end
% % Maximize actual total rate along SAME-SU feasible compensation dimension.
% objective=@(dp) pure_negative_rate_step(tx,u,ell,band,p,dp,rateFn);
% options=optimset('Display','off','TolX',1e-8);
% [dp,~]=fminbnd(objective,0,cap,options);
% for x=[dp,cap]
%     candidate=tx;pp=p;pp(ell)=pp(ell)+x;
%     candidate=set_band_power(candidate,u,band,pp);
%     if rateFn(candidate)>rateFn(best)+1e-6
%         best=candidate;added=x;
%     end
% end
% end
% function val=pure_negative_rate_step(tx,u,ell,band,p,dp,rateFn)
% p(ell)=p(ell)+dp;
% tx=set_band_power(tx,u,band,p);
% val=-rateFn(tx);
% end
% function [rL,rU]=pure_check(tx,HP,HW,GL,GU,PL,PU)
% IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
% IU=aggregate_interference_by_channel(tx,HW,GU,'un');
% rL=max(IL(:)./GL(:));rU=max(IU(:)./GU(:));
% assert(rL<=1+1e-5 && rU<=1+1e-5,'Aggregate interference violated');
% for u=1:numel(tx)
%     assert(sum(tx(u).pLic(:))<=PL*(1+1e-5) && ...
%         sum(tx(u).pUn(:))<=PU*(1+1e-5),'Independent power budget violated');
% end
% end
% 
% %% ===== Case-triggered same-user opposite-band subchannel SWAP helpers =====
% function out=swap_classify_cases(raw,HP,HW,GL,GU)
% IL=aggregate_interference_by_channel(raw,HP,GL,'lic');
% IU=aggregate_interference_by_channel(raw,HW,GU,'un');
% out.byUser=cell(numel(raw),1);out.counts=zeros(1,4);
% for u=1:numel(raw)
%     nu=numel(raw(u).users);out.byUser{u}=4*ones(nu,1);
%     for loc=1:nu
%         iL=find(raw(u).idxLic(:,1)==loc,1);
%         iU=find(raw(u).idxUn(:,1)==loc,1);
%         highL=~isempty(iL)&&IL(raw(u).idxLic(iL,2))> ...
%             GL(raw(u).idxLic(iL,2))*(1+1e-9);
%         highU=~isempty(iU)&&IU(raw(u).idxUn(iU,2))> ...
%             GU(raw(u).idxUn(iU,2))*(1+1e-9);
%         if highL&&highU,caseId=3;
%         elseif highL,caseId=1;
%         elseif highU,caseId=2;
%         else,caseId=4;end
%         out.byUser{u}(loc)=caseId;
%         out.counts(caseId)=out.counts(caseId)+1;
%     end
% end
% end
% 
% function [best,info]=swap_search_same_user(start,cases,caseTriggered, ...
%         HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
%         MM_iter,leakFrac,alpha)
% % One accepted swap maximum per affected user; same-UAV one-to-one swaps.
% % Candidate evaluation identical for generic and case-triggered variants.
% rateFn=@(tx) diagnostic_tx_rate(tx,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
% best=start;bestRate=rateFn(best);
% info=struct('tested',0,'accepted',0,'compL',0,'compU',0, ...
%     'moves',zeros(0,5));
% for u=1:numel(start)
%     for loc=1:numel(start(u).users)
%         caseId=cases.byUser{u}(loc);
%         if caseTriggered
%             if caseId==1,bands=2; % Licensed excessive: unlicensed swap.
%             elseif caseId==2,bands=1; % Unlicensed excessive: licensed swap.
%             elseif caseId==3,bands=[1 2];
%             else,bands=[];end
%         else
%             bands=[1 2];
%         end
%         % Only one accepted swap per SU, as in the proposed experiment.
%         selected=best;selectedRate=bestRate;selectedMeta=[];
%         for b=bands
%             if b==1,idx=best(u).idxLic;
%             else,idx=best(u).idxUn;end
%             e=find(idx(:,1)==loc,1);
%             if isempty(e),continue;end
%             for partner=1:size(idx,1)
%                 if partner==e,continue;end
%                 info.tested=info.tested+1;
%                 [trial,delta]=swap_trial(best,u,e,partner,b, ...
%                     HL,HU,HP,HW,GL,GU,PL,PU,rateFn, ...
%                     hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
%                     MM_iter,leakFrac,alpha);
%                 r=rateFn(trial);
%                 if isfinite(r)&&r>selectedRate+1e-5
%                     selected=trial;selectedRate=r;
%                     selectedMeta=[u loc b idx(e,2) idx(partner,2) delta];
%                 end
%             end
%         end
%         if ~isempty(selectedMeta)
%             best=selected;bestRate=selectedRate;
%             info.accepted=info.accepted+1;
%             info.moves(end+1,:)=selectedMeta(1:5); %#ok<AGROW>
%             if selectedMeta(3)==1,info.compL=info.compL+selectedMeta(6);
%             else,info.compU=info.compU+selectedMeta(6);end
%         end
%     end
% end
% end
% 
% function [trial,delta]=swap_trial(base,u,e,partner,b, ...
%         HL,HU,HP,HW,GL,GU,PL,PU,rateFn, ...
%         hp,hw,Ppbs,Pwifi,nL,nU,BL,BU,MM_iter,leakFrac,alpha)
% trial=base;delta=0;
% if b==1
%     idx=trial(u).idxLic; p=trial(u).pLic;
%     Hs=HL;Hprot=HP;G=GL;Pmax=PL;vList=trial(u).vLic;
%     denomInterf=hp;Pint=Ppbs;noise=nL;
% else
%     idx=trial(u).idxUn;p=trial(u).pUn;
%     Hs=HU;Hprot=HW;G=GU;Pmax=PU;vList=trial(u).vUn;
%     denomInterf=hw;Pint=Pwifi;noise=nU;
% end
% % Swap subchannel labels while preserving each stream's SU and its power.
% % Recalculate BOTH affected users' leakage-aware beams for their NEW SCs.
% scA=idx(e,2);scB=idx(partner,2);
% idx(e,2)=scB;idx(partner,2)=scA;
% for ell=[e partner]
%     loc=idx(ell,1);globalK=trial(u).users(loc);sc=idx(ell,2);
%     den=max(denomInterf(globalK)*Pint(sc)+noise,1e-30);
%     [v,~,~]=mm_unit_beam(Hs{u,globalK},Hprot{u,sc}, ...
%         den,G(sc),max(p(ell),1e-12),MM_iter,leakFrac,alpha);
%     vList{ell}=v;
% end
% if b==1
%     trial(u).idxLic=idx;trial(u).vLic=vList;
% else
%     trial(u).idxUn=idx;trial(u).vUn=vList;
% end
% % Protect all shared channels after changing beams/subchannel occupancy.
% trial=apply_final_safety_clipping(trial,HP,HW,GL,GU,PL,PU,'both');
% % SAME-SU compensation only on the newly selected subchannel, using
% % leftover power of the same band and aggregate interference headroom.
% % No donor-user power changes and no cross-band budget transfer.
% if b==1
%     band='lic';G=GL;Hprot=HP;
% else
%     band='un';G=GU;Hprot=HW;
% end
% [~,pNow,vNow]=get_band_fields(trial,u,band);
% remaining=max(0,Pmax-sum(pNow));
% sc=idx(e,2);leak=abs(Hprot{u,sc}'*vNow{e})^2;
% I=aggregate_interference_by_channel(trial,Hprot,G,band);
% headroom=max(0,(G(sc)-I(sc))/max(leak,realmin));
% upper=min(remaining,headroom);
% if upper>1e-10
%     originalRate=rateFn(trial);
%     obj=@(extra) -rateFn(swap_add_stream_power(trial,u,e,b,extra));
%     [x,~]=fminbnd(obj,0,upper,optimset('TolX',1e-9,'Display','off'));
%     % Also evaluate the boundary; fminbnd need not find global optimum.
%     xs=[0 x upper];
%     baseComp=trial;
%     for t=1:numel(xs)
%         cand=swap_add_stream_power(baseComp,u,e,b,xs(t));
%         r=rateFn(cand);
%         if r>originalRate+1e-5
%             trial=cand;originalRate=r;delta=xs(t);
%         end
%     end
% end
% end
% 
% function out=swap_add_stream_power(base,u,e,b,extra)
% out=base;
% if b==1,out(u).pLic(e)=out(u).pLic(e)+extra;
% else,out(u).pUn(e)=out(u).pUn(e)+extra;end
% end
% 
% function swap_verify_assignment(tx,raw)
% % All initial SU associations, per-band user coverage and per-UAV SC sets
% % remain unchanged. Swaps are a permutation of existing SC assignments.
% for u=1:numel(raw)
%     assert(isequal(tx(u).users,raw(u).users),'Association changed');
%     assert(isequal(tx(u).idxLic(:,1),raw(u).idxLic(:,1)), ...
%         'Licensed user assignments changed');
%     assert(isequal(tx(u).idxUn(:,1),raw(u).idxUn(:,1)), ...
%         'Unlicensed user assignments changed');
%     assert(isequal(sort(tx(u).idxLic(:,2)),sort(raw(u).idxLic(:,2))), ...
%         'Licensed channel set changed');
%     assert(isequal(sort(tx(u).idxUn(:,2)),sort(raw(u).idxUn(:,2))), ...
%         'Unlicensed channel set changed');
% end
% end
% 
% 
% %% =========== FOUR-CASE RESERVED SUBCHANNEL CONTROLLERS ===========
% function [best,info]=reserved_sc_search(start,policy,replacement, ...
%     cases,HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
%     mmIter,leakFrac,alpha,resL,resU)
% % Legacy unlimited version is intentionally not used in this experiment.
% error('Use budgeted_reserved_search instead');
% end
% function [best,info]=budgeted_reserved_search(start,policy,maxCandidates, ...
%     cases,HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
%     mmIter,leakFrac,alpha,resL,resU)
% % Both arms call exactly the same candidate evaluator. Budget counts
% % FULL candidate evaluations, including rejected/infeasible trials.
% rateFn=@(tx) diagnostic_tx_rate(tx,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
% best=start;bestRate=rateFn(best);
% info=struct('tested',0,'accepted',0,'testedByCase',zeros(1,4), ...
%     'acceptedByCase',zeros(1,4),'gainByCase',zeros(1,4));
% for u=1:numel(start)
%     for loc=1:numel(start(u).users)
%         caseId=cases.byUser{u}(loc);
%         if policy==0
%             bands=[1 2];
%         elseif policy==2
%             if caseId==1,bands=2;
%             elseif caseId==2,bands=1;
%             else,bands=[1 2];end
%         elseif policy==3
%             if caseId==4,continue;end
%             if caseId==1,bands=2;
%             elseif caseId==2,bands=1;
%             else,bands=[1 2];end
%         elseif policy==4
%             if caseId~=4,continue;end
%             bands=[1 2];
%         else
%             error('Unknown replacement policy');
%         end
%         selected=best;selectedRate=bestRate;
%         for b=bands
%             if b==1,idx=best(u).idxLic;reserve=resL;
%             else,idx=best(u).idxUn;reserve=resU;end
%             e=find(idx(:,1)==loc,1);
%             if isempty(e),continue;end
%             for sc=reserve
%                 if reserved_used_anywhere(best,sc,b),continue;end
%                 if info.tested>=maxCandidates,break;end
%                 info.tested=info.tested+1;
%                 info.testedByCase(caseId)=info.testedByCase(caseId)+1;
%                 [trial,~]=reserved_sc_trial(best,u,e,sc,b,true, ...
%                     HL,HU,HP,HW,GL,GU,PL,PU,rateFn,hp,hw, ...
%                     Ppbs,Pwifi,nL,nU,mmIter,leakFrac,alpha);
%                 r=rateFn(trial);
%                 if isfinite(r)&&r>selectedRate+1e-5
%                     selected=trial;selectedRate=r;
%                 end
%             end
%             if info.tested>=maxCandidates,break;end
%         end
%         if selectedRate>bestRate+1e-5
%             info.accepted=info.accepted+1;
%             info.acceptedByCase(caseId)=info.acceptedByCase(caseId)+1;
%             info.gainByCase(caseId)=info.gainByCase(caseId)+ ...
%                 (selectedRate-bestRate);
%             best=selected;bestRate=selectedRate;
%         end
%         if info.tested>=maxCandidates,return;end
%     end
% end
% end
% 
% function tf=reserved_used_anywhere(tx,sc,b)
% tf=false;
% for u=1:numel(tx)
%     if b==1,idx=tx(u).idxLic;else,idx=tx(u).idxUn;end
%     if any(idx(:,2)==sc),tf=true;return;end
% end
% end
% function [trial,delta]=reserved_sc_trial(base,u,e,sc,b,replacement, ...
%     HL,HU,HP,HW,GL,GU,PL,PU,rateFn,hp,hw, ...
%     Ppbs,Pwifi,nL,nU,mmIter,leakFrac,alpha)
% trial=base;delta=0;
% if b==1
%     idx=base(u).idxLic;p=base(u).pLic(:);v=base(u).vLic;
%     HS=HL;Hprot=HP;G=GL;Pmax=PL;hi=hp;Pi=Ppbs;noise=nL;
%     band='lic';
% else
%     idx=base(u).idxUn;p=base(u).pUn(:);v=base(u).vUn;
%     HS=HU;Hprot=HW;G=GU;Pmax=PU;hi=hw;Pi=Pwifi;noise=nU;
%     band='un';
% end
% loc=idx(e,1);k=base(u).users(loc);
% den=max(hi(k)*Pi(sc)+noise,1e-30);
% [vNew,~,~]=mm_unit_beam(HS{u,k},Hprot{u,sc}, ...
%     den,G(sc),max(p(e),1e-12),mmIter,leakFrac,alpha);
% if replacement
%     % Move existing power/stream to a RESERVED channel. No extra bandwidth
%     % used by this SU, and no additional power is created.
%     idx(e,2)=sc;v{e}=vNew;
% else
%     % Add a new independent OFDMA stream for the SAME SU, only if its
%     % corresponding UAV/band has unused power and the reserved SC is safe.
%     remaining=max(0,Pmax-sum(p));
%     if remaining<=1e-12,return;end
%     idx(end+1,:)=[loc sc];p(end+1,1)=0;v{end+1,1}=vNew;
% end
% if b==1
%     trial(u).idxLic=idx;trial(u).pLic=p;trial(u).vLic=v;
% else
%     trial(u).idxUn=idx;trial(u).pUn=p;trial(u).vUn=v;
% end
% % Replacement may alter aggregate protection on a reserved channel.
% % Do NOT clip: protection is a feasibility test of the reserved candidate.
% [preRatioL,preRatioU]=pure_check(trial,HP,HW,GL,GU,PL,PU);
% if preRatioL>1+1e-5 || preRatioU>1+1e-5
%     trial=base;
%     return;
% end
% if replacement
%     % Optimize extra power only if unused power exists after moving.
%     [~,pNow,vNow]=get_band_fields(trial,u,band);
%     stream=e;
% else
%     [~,pNow,vNow]=get_band_fields(trial,u,band);
%     stream=numel(pNow);
% end
% remaining=max(0,Pmax-sum(pNow));
% I=aggregate_interference_by_channel(trial,Hprot,G,band);
% leak=abs(Hprot{u,sc}'*vNow{stream})^2;
% head=max(0,G(sc)-I(sc));
% if leak>1e-30,upper=min(remaining,head/leak);
% else,upper=remaining;end
% if upper>1e-12
%     % Same scalar optimization for generic and case-triggered arms.
%     obj=@(x) -rateFn(reserved_add_power(trial,u,stream,b,x));
%     [x,~]=fminbnd(obj,0,upper,optimset('TolX',1e-9,'Display','off'));
%     choices=[0 x upper];
%     baseRate=rateFn(trial);bestExtra=0;
%     for ii=1:numel(choices)
%         candidate=reserved_add_power(trial,u,stream,b,choices(ii));
%         r=rateFn(candidate);
%         if isfinite(r)&&r>baseRate+1e-5
%             baseRate=r;bestExtra=choices(ii);
%         end
%     end
%     trial=reserved_add_power(trial,u,stream,b,bestExtra);
%     delta=bestExtra;
% end
% % Do not retain a zero-power added stream. This also prevents an empty
% % reservation from being counted as an accepted channel activation.
% if ~replacement && delta<=1e-12
%     trial=base;return;
% end
% % All accepted changes must satisfy aggregate interference and budgets.
% [ratioL,ratioU]=pure_check(trial,HP,HW,GL,GU,PL,PU);
% assert(ratioL<=1+1e-5&&ratioU<=1+1e-5,'Reserved trial infeasible');
% end
% function out=reserved_add_power(base,u,e,b,extra)
% out=base;
% if b==1,out(u).pLic(e)=out(u).pLic(e)+extra;
% else,out(u).pUn(e)=out(u).pUn(e)+extra;end
% end
% function reserved_verify(tx,raw,resL,resU,allowReplacement)
% for u=1:numel(tx)
%     assert(isequal(tx(u).users,raw(u).users), ...
%         'User association changed unexpectedly');
%     for b=1:2
%         if b==1,idx=tx(u).idxLic;original=raw(u).idxLic;reserve=resL;
%         else,idx=tx(u).idxUn;original=raw(u).idxUn;reserve=resU;end
%         assert(numel(unique(idx(:,2)))==size(idx,1), ...
%             'One UAV has two streams on one subchannel');
%         for ell=1:size(idx,1)
%             assert(any(original(:,1)==idx(ell,1)), ...
%                 'An unassociated user received a stream');
%         end
%         if ~allowReplacement
%             assert(isequal(idx(1:size(original,1),:),original), ...
%                 'Initial assignment changed during addition-only test');
%             assert(all(ismember(idx(size(original,1)+1:end,2),reserve)), ...
%                 'Added stream is not on a reserved subchannel');
%         else
%             assert(isequal(idx(:,1),original(:,1))&& ...
%                 size(idx,1)==size(original,1), ...
%                 'Replacement changed the number of original streams');
%             changed=idx(:,2)~=original(:,2);
%             assert(all(ismember(idx(changed,2),reserve)), ...
%                 'Replacement used a non-reserved channel');
%         end
%     end
% end
% for b=1:2
%     if b==1,reserve=resL;else,reserve=resU;end
%     for sc=reserve
%         n=0;
%         for u=1:numel(tx)
%             if b==1,idx=tx(u).idxLic;else,idx=tx(u).idxUn;end
%             n=n+sum(idx(:,2)==sc);
%         end
%         assert(n<=1,'Reserved SC assigned to multiple UAVs');
%     end
% end
% end
% function reserved_plot(x,Y,names,ylab,filename)
% figure('Position',[100 100 620 470]);
% styles={'o-','s--','d-.','^-','v:'};hold on;
% for ii=1:size(Y,2)
%     plot(x,Y(:,ii),styles{ii},'LineWidth',2,'MarkerSize',7);
% end
% ax=gca;ax.FontName='Times New Roman';ax.FontSize=16;
% ax.FontWeight='normal';ax.LineWidth=1.2;
% ax.Position=[0.13 0.13 0.84 0.68];grid on;
% ax.GridLineStyle=':';ax.GridAlpha=0.30;
% xlabel('Paired channel realization');ylabel(ylab);
% legend(names,'FontName','Times New Roman','FontSize',12, ...
%     'NumColumns',2,'Location','northoutside');
% saveas(gcf,filename);
% end
% 
% 
% %% ===== FINAL RAW-INTERFERENCE RESERVED-SUBCHANNEL CONTROLLER =====
% function [best,stats]=fourcase_crossband_reserved_controller(base,cases,...
%     HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU,...
%     mmIter,leakFrac,alpha,resL,resU)
% % RESERVED ALGORITHM USED IN PROPOSED II
% % ------------------------------------------------------------
% % 1) BASE is the RAW allocation after ordinary power allocation.
% % 2) No protection/clipping is applied before the reserved search.
% % 3) For every interference-affected SU, candidate replacements are tested
% %    on the licensed band, unlicensed band, and both bands jointly.
% % 4) A candidate is preferred first by aggregate-protection feasibility;
% %    while the current allocation is still infeasible, the search selects
% %    the candidate that most reduces the worst normalized violation.
% % 5) Once protection is achieved, only protected candidates are eligible
% %    and the candidate with the largest actual network sum rate is retained.
% % 6) No post-search safety clipping is used. The returned allocation must
% %    itself satisfy PU/WU aggregate-interference and per-UAV power budgets.
% %
% % This is a coordinated sequential candidate search (local search), not a
% % claim of globally optimal mixed-integer enumeration.
% rateFn=@(tx) diagnostic_tx_rate(tx,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
% best=base;
% stats=struct('tested',0,'accepted',0,'caseCounts',zeros(1,4),...
%     'testedByCase',zeros(1,4),'acceptedByCase',zeros(1,4));
% 
% % Count raw interference cases.
% for u=1:numel(base)
%     for loc=1:numel(base(u).users)
%         cid=cases.byUser{u}(loc);
%         stats.caseCounts(cid)=stats.caseCounts(cid)+1;
%     end
% end
% 
% % Multiple coordinated passes allow replacements for several affected SUs
% % to collectively remove a network-wide aggregate-interference violation.
% maxPass=max(1,2*sum(stats.caseCounts(1:3)));
% for pass=1:maxPass
%     changed=false;
%     for u=1:numel(best)
%         for loc=1:numel(best(u).users)
%             caseId=cases.byUser{u}(loc);
%             if caseId==4
%                 continue; % raw allocation is safe for this SU on both bands
%             end
% 
%             incumbent=best;
%             incumbentRate=rateFn(incumbent);
%             incumbentViolation=reserved_violation_metric(incumbent,HP,HW,GL,GU,PL,PU);
%             chosen=incumbent;
%             chosenRate=incumbentRate;
%             chosenViolation=incumbentViolation;
% 
%             % Check BOTH bands. NaN means keep the original SC on that band.
%             candidateL=[NaN resL];
%             candidateU=[NaN resU];
%             for il=1:numel(candidateL)
%                 for iu=1:numel(candidateU)
%                     newL=candidateL(il); newU=candidateU(iu);
%                     if isnan(newL) && isnan(newU)
%                         continue;
%                     end
%                     if ~isnan(newL) && reserved_used_anywhere(incumbent,newL,1)
%                         continue;
%                     end
%                     if ~isnan(newU) && reserved_used_anywhere(incumbent,newU,2)
%                         continue;
%                     end
% 
%                     trial=incumbent;
%                     if ~isnan(newL)
%                         e=find(trial(u).idxLic(:,1)==loc,1);
%                         if isempty(e),continue;end
%                         trial=reserved_replace_no_clip(trial,u,e,newL,1,...
%                             HL,HU,HP,HW,GL,GU,hp,hw,Ppbs,Pwifi,nL,nU,...
%                             mmIter,leakFrac,alpha);
%                     end
%                     if ~isnan(newU)
%                         e=find(trial(u).idxUn(:,1)==loc,1);
%                         if isempty(e),continue;end
%                         trial=reserved_replace_no_clip(trial,u,e,newU,2,...
%                             HL,HU,HP,HW,GL,GU,hp,hw,Ppbs,Pwifi,nL,nU,...
%                             mmIter,leakFrac,alpha);
%                     end
% 
%                     % Re-optimize powers under the actual aggregate protection
%                     % headroom. This routine uses capped KKT only; no clipping.
%                     trial=fourcase_kkt_refine_candidate(trial,u,...
%                         HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,...
%                         nL,nU,BL,BU);
% 
%                     stats.tested=stats.tested+1;
%                     stats.testedByCase(caseId)=stats.testedByCase(caseId)+1;
%                     v=reserved_violation_metric(trial,HP,HW,GL,GU,PL,PU);
%                     r=rateFn(trial);
%                     if ~isfinite(r),continue;end
% 
%                     if incumbentViolation>1+1e-6
%                         % Before feasibility is reached, reduce violation first;
%                         % use rate only as the tie-breaker.
%                         better=(v<chosenViolation-1e-9) || ...
%                             (abs(v-chosenViolation)<=1e-9 && r>chosenRate+1e-5);
%                     else
%                         % Once protected, maximize rate over protected candidates.
%                         better=(v<=1+1e-6) && r>chosenRate+1e-5;
%                     end
%                     if better
%                         chosen=trial;
%                         chosenRate=r;
%                         chosenViolation=v;
%                     end
%                 end
%             end
% 
%             if chosenViolation<incumbentViolation-1e-9 || ...
%                     (chosenViolation<=1+1e-6 && incumbentViolation<=1+1e-6 && ...
%                      chosenRate>incumbentRate+1e-5)
%                 best=chosen;
%                 changed=true;
%                 stats.accepted=stats.accepted+1;
%                 stats.acceptedByCase(caseId)=stats.acceptedByCase(caseId)+1;
%             end
%         end
%     end
%     if reserved_violation_metric(best,HP,HW,GL,GU,PL,PU)<=1+1e-6
%         % One additional protected rate-improvement pass is enough; if this
%         % pass made no change, the coordinated sequential search has converged.
%         if ~changed || pass>1
%             break;
%         end
%     elseif ~changed
%         break;
%     end
% end
% 
% % Final verification only. Candidate evaluation above is intentionally
% % non-asserting because the RAW allocation may violate protection.
% ILfinal=aggregate_interference_by_channel(best,HP,GL,'lic');
% IUfinal=aggregate_interference_by_channel(best,HW,GU,'un');
% ratioL=max(ILfinal(:)./max(GL(:),realmin));
% ratioU=max(IUfinal(:)./max(GU(:),realmin));
% powerOK=true;
% for uu=1:numel(best)
%     powerOK=powerOK && sum(best(uu).pLic(:))<=PL*(1+1e-5) && ...
%         sum(best(uu).pUn(:))<=PU*(1+1e-5);
% end
% assert(ratioL<=1+1e-5 && ratioU<=1+1e-5 && powerOK,...
%     ['No feasible reserved-subchannel combination was found without ' ...
%      'power clipping. Increase the reserved pool or revise the raw allocation.']);
% end
% 
% function trial=reserved_replace_no_clip(base,u,e,sc,b,...
%     HL,HU,HP,HW,GL,GU,hp,hw,Ppbs,Pwifi,nL,nU,...
%     mmIter,leakFrac,alpha)
% % Replace one existing stream by a reserved subchannel while preserving its
% % current power. Recompute its leakage-aware beam on the new subchannel.
% % NO clipping or safety scaling is performed here.
% trial=base;
% if b==1
%     idx=trial(u).idxLic; p=trial(u).pLic(:); v=trial(u).vLic;
%     HS=HL; Hprot=HP; G=GL; hi=hp; Pi=Ppbs; noise=nL;
% else
%     idx=trial(u).idxUn; p=trial(u).pUn(:); v=trial(u).vUn;
%     HS=HU; Hprot=HW; G=GU; hi=hw; Pi=Pwifi; noise=nU;
% end
% loc=idx(e,1);
% k=trial(u).users(loc);
% den=max(hi(k)*Pi(sc)+noise,1e-30);
% [vNew,~,~]=mm_unit_beam(HS{u,k},Hprot{u,sc},...
%     den,G(sc),max(p(e),1e-12),mmIter,leakFrac,alpha);
% idx(e,2)=sc;
% v{e}=vNew;
% if b==1
%     trial(u).idxLic=idx;
%     trial(u).vLic=v;
% else
%     trial(u).idxUn=idx;
%     trial(u).vUn=v;
% end
% end
% 
% function metric=reserved_violation_metric(tx,HP,HW,GL,GU,PL,PU)
% % Worst normalized aggregate-interference or per-UAV power-budget ratio.
% % IMPORTANT: this is a NON-ASSERTING candidate metric. During the reserved
% % search, infeasible raw/trial allocations are expected and must be scored,
% % not rejected by pure_check(). Final protection is asserted only after the
% % reserved-subchannel search has selected its output.
% IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
% IU=aggregate_interference_by_channel(tx,HW,GU,'un');
% ratioL=max(IL(:)./max(GL(:),realmin));
% ratioU=max(IU(:)./max(GU(:),realmin));
% ratioPL=0; ratioPU=0;
% for uu=1:numel(tx)
%     ratioPL=max(ratioPL,sum(tx(uu).pLic(:))/max(PL,realmin));
%     ratioPU=max(ratioPU,sum(tx(uu).pUn(:))/max(PU,realmin));
% end
% metric=max([ratioL,ratioU,ratioPL,ratioPU]);
% end
% 
% function trial=fourcase_kkt_refine_candidate(base,u,...
%     HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU)
% % Same-UAV per-band capped KKT refinement. Other UAVs are held fixed.
% % Protection is imposed through residual aggregate-interference caps inside
% % the KKT allocation itself. There is NO safety clipping after this step.
% trial=base;
% for b=1:2
%     if b==1
%         band='lic';HS=HL;Hprot=HP;G=GL;Pmax=PL;
%         hI=hp;Pi=Ppbs;noise=nL;
%         idx=trial(u).idxLic;vs=trial(u).vLic;
%     else
%         band='un';HS=HU;Hprot=HW;G=GU;Pmax=PU;
%         hI=hw;Pi=Pwifi;noise=nU;
%         idx=trial(u).idxUn;vs=trial(u).vUn;
%     end
%     if isempty(idx),continue;end
%     a=zeros(size(idx,1),1);
%     caps=zeros(size(idx,1),1);
%     Iall=aggregate_interference_by_channel(trial,Hprot,G,band);
%     for e=1:size(idx,1)
%         sc=idx(e,2);
%         k=trial(u).users(idx(e,1));
%         if b==1
%             pcur=trial(u).pLic(e);
%         else
%             pcur=trial(u).pUn(e);
%         end
%         leak=abs(Hprot{u,sc}'*vs{e})^2;
%         otherLeak=max(0,Iall(sc)-pcur*leak);
%         caps(e)=min(Pmax,max(0,G(sc)-otherLeak)/max(leak,realmin));
%         den=max(hI(k)*Pi(sc)+noise+...
%             inter_uav_power_same_sc(trial,HS,u,k,sc,band),1e-30);
%         a(e)=max(abs(HS{u,k}'*vs{e})^2/den,1e-30);
%     end
%     pNew=kkt_waterfill_capped_original(a,Pmax,caps);
%     trial=set_band_power(trial,u,band,pNew);
% end
% end
% 
% function [txBest,stats] = original_aiibpc_rateaware_diagnostic(raw,txConv,txFour, ...
%     HP,HW,GL,GU,PL,PU,rateFn)
%     % EXPERIMENTAL rate-aware, fallback-enhanced variant, NOT the
%     % original AI-IBPC definition. Both protected references are polished.
%     % Protection was applied in the original four-case block (txFour),
%     % or conventional reference protection (txConv), never in raw allocation.
%     %#ok<INUSD> raw is retained in interface for auditability.
%     [txConv,convStats] = rateaware_polish_existing( ...
%         txConv,HP,HW,GL,GU,PL,PU,rateFn);
%     [txFour,fourStats] = rateaware_polish_existing( ...
%         txFour,HP,HW,GL,GU,PL,PU,rateFn);
%     rConv=rateFn(txConv);rFour=rateFn(txFour);
%     if rFour > rConv+1e-9
%         txBest=txFour;stats.source='fourcase';
%     else
%         txBest=txConv;stats.source='protected_reference';
%     end
%     [rL,rU]=pure_check(txBest,HP,HW,GL,GU,PL,PU);
%     assert(max(rL,rU)<=1+1e-5,'Rate-aware original violates protection');
%     stats.rate=rateFn(txBest);
%     stats.objectiveCalls=convStats.objectiveCalls+fourStats.objectiveCalls+3;
%     stats.scalarSearches=convStats.scalarSearches+fourStats.scalarSearches;
%     stats.acceptedUpdates=convStats.acceptedUpdates+fourStats.acceptedUpdates;
%     stats.conventionalPolish=convStats;
%     stats.fourcasePolish=fourStats;
% end
% 
% function [tx,stats]=rateaware_polish_existing(tx,HP,HW,GL,GU,PL,PU,rateFn)
%     % Count all rate objective calls including fminbnd internal trials.
%     stats=struct('objectiveCalls',0,'scalarSearches',0,'acceptedUpdates',0);
%     bestRate=counted_rate(tx);
%     for pass=1:2
%         improved=false;
%         for u=1:numel(tx)
%             for bb=1:2
%                 if bb==1
%                     band='lic';Hprot=HP;Gamma=GL;Pmax=PL;
%                 else
%                     band='un';Hprot=HW;Gamma=GU;Pmax=PU;
%                 end
%                 [idx,p,v]=get_band_fields(tx,u,band);
%                 for e=1:numel(p)
%                     sc=idx(e,2);
%                     leak=abs(Hprot{u,sc}'*v{e})^2;
%                     I=aggregate_interference_by_channel(tx,Hprot,Gamma,band);
%                     other=max(0,I(sc)-p(e)*leak);
%                     budget=max(0,Pmax-(sum(p)-p(e)));
%                     if leak>1e-30
%                         protect=max(0,(Gamma(sc)-other)/leak);
%                     else
%                         protect=inf;
%                     end
%                     upper=min(budget,protect);
%                     if upper<=0,continue;end
%                     current=min(max(0,p(e)),upper);
%                     fun=@(power)counted_trial(tx,u,band,p,e,power);
%                     opts=optimset('TolX',max(1e-11,upper*1e-5),'Display','off');
%                     stats.scalarSearches=stats.scalarSearches+1;
%                     pStar=fminbnd(@(power)-fun(power),0,upper,opts);
%                     tests=unique([0;current;upper;pStar]);
%                     scores=zeros(numel(tests),1);
%                     for tt=1:numel(tests)
%                         scores(tt)=fun(tests(tt));
%                     end
%                     [candidateRate,k]=max(scores);
%                     if candidateRate>bestRate+1e-6
%                         p(e)=tests(k);
%                         tx=set_band_power(tx,u,band,p);
%                         bestRate=candidateRate;
%                         stats.acceptedUpdates=stats.acceptedUpdates+1;
%                         improved=true;
%                     end
%                 end
%             end
%         end
%         if ~improved,break;end
%     end
%     function r=counted_rate(t)
%         stats.objectiveCalls=stats.objectiveCalls+1;
%         r=rateFn(t);
%     end
%     function r=counted_trial(t,uu,bandName,powers,ee,power)
%         stats.objectiveCalls=stats.objectiveCalls+1;
%         r=rateaware_trial_rate(t,uu,bandName,powers,ee,power,rateFn);
%     end
% end
% 
% function r=rateaware_trial_rate(tx,u,band,p,e,power,rateFn)
%     p(e)=power;
%     tx=set_band_power(tx,u,band,p);
%     r=rateFn(tx);
% end
% 
% 
% 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
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

function same=compare_tx_numeric_stage(a,b,tol)
same=numel(a)==numel(b);
if ~same,return;end
for uu=1:numel(a)
    fields={'users','idxLic','idxUn','pLic','pUn'};
    for ff=1:numel(fields)
        x=a(uu).(fields{ff}); y=b(uu).(fields{ff});
        if ~isequal(size(x),size(y)) || numel(x)~=numel(y)
            same=false;return;
        end
        if isempty(x),continue;end
        if max(abs(double(x(:))-double(y(:))))>tol
            same=false;return;
        end
    end
    for field={'vLic','vUn'}
        x=a(uu).(field{1});y=b(uu).(field{1});
        if numel(x)~=numel(y),same=false;return;end
        for ii=1:numel(x)
            if ~isequal(size(x{ii}),size(y{ii})) || ...
                    max(abs(x{ii}(:)-y{ii}(:)))>tol
                same=false;return;
            end
        end
    end
end
end
function Qnew = resample_multi_trajectory_local(Qold,Nnew)
U = size(Qold,1);
Nold = size(Qold,2);
Qnew = zeros(U,Nnew,2);
xOld = linspace(0,1,Nold);
xNew = linspace(0,1,Nnew);
for u = 1:U
    Qnew(u,:,1) = interp1(xOld,squeeze(Qold(u,:,1)),xNew,'linear','extrap');
    Qnew(u,:,2) = interp1(xOld,squeeze(Qold(u,:,2)),xNew,'linear','extrap');
end
end
%% ============================================================
%                  MULTI-UAV TRAJECTORY FUNCTIONS
%% ============================================================
function Q = build_multi_straight_trajectory(q_start_all,q_end_all,N)
U = size(q_start_all,1);
Q = zeros(U,N,2);
for u = 1:U
    for n = 1:N
        a = (n-1)/(N-1);
        Q(u,n,:) = (1-a)*q_start_all(u,:) + a*q_end_all(u,:);
    end
end
end
function q_hover_all = make_multi_hover_points_closed(q_start_all,WS,area)
% Closed-loop compatible hover/service point selection.
% Each SU is assigned to its nearest UAV initial point. Each UAV then uses
% the centroid of its assigned SUs as the hover/service point.
U = size(q_start_all,1);
K = size(WS,1);
assign = zeros(K,1);
q_hover_all = zeros(U,2);
for k = 1:K
    distToUAV = vecnorm(q_start_all - WS(k,:),2,2);
    [~,uBest] = min(distToUAV);
    assign(k) = uBest;
end
for u = 1:U
    ids = find(assign == u);
    if isempty(ids)
        q_hover_all(u,:) = q_start_all(u,:);
    else
        q_hover_all(u,:) = mean(WS(ids,:),1);
    end
    q_hover_all(u,:) = min(max(q_hover_all(u,:),0),area);
end
end
function Q = build_multi_fly_hover_trajectory(q_start_all,q_hover_all,q_end_all,N,stepMax,area)
% Closed-loop/non-stationary initialization.
% If q_end_all = q_start_all,this creates q0 -> hover/service point -> q0.
% If the hover point is too far for the available N and stepMax,it uses the
% farthest feasible point along the q0-to-hover direction and still returns.
U = size(q_start_all,1);
Q = zeros(U,N,2);
for u = 1:U
    qs = q_start_all(u,:);
    qh = q_hover_all(u,:);
    qe = q_end_all(u,:);
    d1 = norm(qh - qs);
    d2 = norm(qe - qh);
    totalDist = d1 + d2;
    maxDist = stepMax*(N-1);
    % If the selected hover point is infeasible,pull it back along the
    % start-hover direction so that a return path remains feasible.
    if totalDist > maxDist && d1 > 1e-12
        % For closed-loop qs=qe,the feasible hover distance is maxDist/2.
        % For general qs~=qe this scale is a safe conservative fallback.
        scale = min(1,0.95*maxDist/max(totalDist,1e-12));
        qh = qs + scale*(qh - qs);
        qh = min(max(qh,0),area);
        d1 = norm(qh - qs);
        d2 = norm(qe - qh);
    end
    n1 = ceil(d1/max(stepMax,1e-12));
    n2 = ceil(d2/max(stepMax,1e-12));
    totalMove = n1 + n2;
    pts = zeros(N,2);
    idx = 1;
    pts(idx,:) = qs;
    if totalMove <= N-1 && totalMove > 0
        hoverSteps = (N-1) - totalMove;
        for ii = 1:n1
            idx = idx + 1;
            a = ii/max(n1,1);
            pts(idx,:) = (1-a)*qs + a*qh;
        end
        for ii = 1:hoverSteps
            idx = idx + 1;
            pts(idx,:) = qh;
        end
        for ii = 1:n2
            idx = idx + 1;
            a = ii/max(n2,1);
            pts(idx,:) = (1-a)*qh + a*qe;
        end
        if idx < N
            pts(idx+1:N,:) = repmat(qe,N-idx,1);
        end
    else
        % Final fallback: direct path. This only becomes stationary when
        % qs=qe and no feasible hover displacement exists.
        for n = 1:N
            a = (n-1)/(N-1);
            pts(n,:) = (1-a)*qs + a*qe;
        end
    end
    Q(u,:,:) = min(max(pts(1:N,:),0),area);
end
Q = project_multi_trajectory(Q,area,stepMax,q_start_all,q_end_all,30);
% Collision avoidance is checked/repaired by the caller after initialization.
end
function [Qout,rateHist,stopReason] = build_multi_traj_AO_qos_bf(Qinit,q_start_all,q_end_all,WS,WP,WW,w_b,w_wifi,H,...
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
rateHist=nan(AO_traj_iter+1,1); rateHist(1)=prevRate; histIdx=1;
stopReason='maximum iterations';
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
        stopReason='nonfinite derivatives';
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
                    prevRate=trialRate; histIdx=histIdx+1; rateHist(histIdx)=prevRate;
                    rateHist=rateHist(1:histIdx);
                    stopReason='relative improvement tolerance';
                    return;
                end
                prevRate=trialRate; histIdx=histIdx+1; rateHist(histIdx)=prevRate;
                break;
            end
        end
        radius=radius/2;
        if radius<minRadius, break; end
    end
    if ~accepted
        stopReason='no acceptable trajectory update';
        break;
    end
end
rateHist=rateHist(1:histIdx);
end
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

function avgRate=evaluate_reserved_trajectory(Q,WS,WP,WW,w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,nP,nW,Shadow,...
    Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,seedBase,useRayleigh,useShadow,...
    useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU)
s=0; N=size(Q,2);
for n=1:N
    [~,r]=eval_slot_reserved_final(squeeze(Q(:,n,:)),WS,WP,WW,w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,...
        nP,nW,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,seedBase+n,...
        useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU);
    s=s+r;
end
avgRate=s/N;
end

function [txReserved,rateReserved]=eval_slot_reserved_final(Qslot,WS,WP,WW,w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,...
    nP,nW,Shadow,Ppbs,Pwifi,nL,nU,GL,GU,PL,PU,QoS,BL,BU,AO_AP_iter,seedNow,useRayleigh,...
    useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,Ninitial,resL,resU)
% Ordinary allocation is generated on the active (non-reserved) channels.
S0=Shadow; S0.a2g_pu_z=Shadow.a2g_pu_z(1:Ninitial); S0.a2g_wu_z=Shadow.a2g_wu_z(1:Ninitial);
[~,~,~,~,~,dd]=eval_slot_multi_qos_bf(Qslot,WS,WP(1:Ninitial,:),WW(1:Ninitial,:),w_b,w_wifi,H,...
    fL,fU,fP,fW,c,A2G,nP,nW,S0,Ppbs(1:Ninitial),Pwifi(1:Ninitial),nL,nU,GL(1:Ninitial),GU(1:Ninitial),...
    PL,PU,QoS,BL,BU,AO_AP_iter,seedNow,useRayleigh,useShadow,useShadowA2G,Nt,mmIter,leakFrac,regAlpha,'prop-pb-mm',0,true);
raw=dd.rawTx;
[HL0,HU0,HP0,HW0,hp,hw]=gen_multi_channels_vec(Qslot,WS,WP(1:Ninitial,:),WW(1:Ninitial,:),...
    w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,nP,nW,S0,seedNow,useRayleigh,useShadow,useShadowA2G,Nt);
[~,~,HPx,HWx]=gen_multi_channels_vec(Qslot,WS,WP,WW,w_b,w_wifi,H,fL,fU,fP,fW,c,A2G,nP,nW,...
    Shadow,seedNow+250000,useRayleigh,useShadow,useShadowA2G,Nt);
HL=HL0; HU=HU0; HP=cell(size(Qslot,1),numel(GL)); HW=cell(size(Qslot,1),numel(GU));
HP(:,1:Ninitial)=HP0; HW(:,1:Ninitial)=HW0; HP(:,resL)=HPx(:,resL); HW(:,resU)=HWx(:,resU);
cases=swap_classify_cases(raw,HP,HW,GL,GU);
[txReserved,~]=fourcase_crossband_reserved_controller(raw,cases,HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,...
    Ppbs,Pwifi,nL,nU,BL,BU,mmIter,leakFrac,regAlpha,resL,resU);
rateReserved=diagnostic_tx_rate(txReserved,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
end

function [Qtrial,solved]=joint_traj_cvx_analytic(Q0,gradR,L0,gradL,qStart,qEnd,stepMax,area,radius,Dmin_UAV)
% A SINGLE convex program for all 3 UAVs and all N waypoints.
% Linearized normalized aggregate interference is enforced conservatively
% locally; true protected interference is rechecked by slot allocation.
[U,N,~]=size(Q0); S=size(L0,1);
X0=reshape(permute(Q0,[2 3 1]),N,2*U);
G=reshape(permute(gradR,[2 3 1]),N,2*U);
% Scale rate gradient to improve numerical conditioning.
scale=max(1,max(abs(G(:))));
solved=false; Qtrial=Q0;
try
    cvx_begin quiet
        variable X(N,2*U)
        expressions leakageLinear(S,N)
        maximize( sum(sum((G/scale).*(X-X0))) - ...
            (1e-4/max(radius^2,1e-9))*sum_square(reshape(X-X0,N*2*U,1)) )
        subject to
            X(:)>=0; X(:)<=area;
            for u=1:U
                cols=2*u-1:2*u;
                X(1,cols)==qStart(u,:);
                X(N,cols)==qEnd(u,:);
                for n=1:N-1
                    norm(X(n+1,cols)-X(n,cols),2)<=stepMax;
                end
                for n=2:N-1
                    norm(X(n,cols)-X0(n,cols),2)<=radius;
                end
            end
            % Convex inner approximation of pairwise non-collision.
            % At feasible Q0: ||d||^2 >= Dmin^2 is conservatively enforced
            % by ||d0||^2 + 2*d0'*(d-d0) >= Dmin^2.
            % The same check applies to all time slots, including endpoints.
            for n=1:N
                for u=1:U-1
                    for v=u+1:U
                        cu=2*u-1:2*u; cv=2*v-1:2*v;
                        d0=X0(n,cu)-X0(n,cv);
                        if sum(d0.^2)<Dmin_UAV^2-1e-5
                            error('Joint SCA requires a collision-free expansion path.');
                        end
                        2*d0*(X(n,cu)-X(n,cv)).' - sum(d0.^2) >= Dmin_UAV^2;
                    end
                end
            end
            % Aggregate PU/WU protection is re-enforced by the complete
            % AI-IBPC / IA-RSS communication re-optimization at Qtrial.
            % Do not impose an additional frozen-resource linearized leakage
            % constraint here; it can make the local trajectory SCA infeasible.
    cvx_end
    solved=contains(lower(cvx_status),'solved') && all(isfinite(X(:)));
    if solved
        Qtrial=permute(reshape(X,N,2,U),[3 1 2]);
    end
catch ME
    warning('Joint trajectory CVX: %s',ME.message);
    try,cvx_clear;catch,end
end
end
function [gradRate,L0,gradLeak]=joint_analytic_slot_derivatives(Q,tx,WS,WP,WW,w_b,w_wifi,H,...
    fL,fU,fP,fW,c,A2G,nP,nW,Shadow,Ppbs,Pwifi,sigmaL,sigmaU,GammaL,GammaU,...
    seed,useRayleigh,useShadow,useShadowA2G,Nt,BscL,BscU)
% Exact derivative of frozen-resource, fixed-fading SINR w.r.t. positions.
% h(q)=h(q0)*sqrt(g(q)/g(q0)) under common small-scale fading.
[HL,HU,HP,HW,hp2,hw2]=gen_multi_channels_vec(Q,WS,WP,WW,w_b,w_wifi,H,...
    fL,fU,fP,fW,c,A2G,nP,nW,Shadow,seed,useRayleigh,useShadow,useShadowA2G,Nt);
U=size(Q,1); S=numel(GammaL)+numel(GammaU);
gradRate=zeros(U,2); L0=zeros(S,1); gradLeak=zeros(S,U,2);
for band=1:2
    if band==1
        HS=HL; HT=HP; W=WP; f=fL; hI=hp2; PI=Ppbs; noise=sigmaL;
        gamma=GammaL; offset=0; bw='lic'; Bsc=BscL;
    else
        HS=HU; HT=HW; W=WW; f=fU; hI=hw2; PI=Pwifi; noise=sigmaU;
        gamma=GammaU; offset=numel(GammaL); bw='un'; Bsc=BscU;
    end
    K=size(WS,1); gainGrad=zeros(U,K,2);
    for u=1:U
        for k=1:K
            [~,dg]=a2g_gain_and_gradient(Q(u,:),WS(k,:),H,f,c,A2G,...
                useShadowA2G,Shadow.a2g_su_z(k));
            gainGrad(u,k,:)=dg;
        end
        [idx,p,vList]=get_band_fields(tx,u,bw);
        for ell=1:size(idx,1)
            sc=idx(ell,2);
            if sc<1 || sc>numel(gamma),continue;end
            [~,dg]=a2g_gain_and_gradient(Q(u,:),W(sc,:),H,f,c,A2G,...
                useShadowA2G,ternary_shadow_z(Shadow,sc,band));
            term=p(ell)*abs(HT{u,sc}'*vList{ell})^2;
            L0(offset+sc)=L0(offset+sc)+term/max(gamma(sc),realmin);
            gradLeak(offset+sc,u,:)=reshape(squeeze(gradLeak(offset+sc,u,:)).' +...
                term/max(gamma(sc),realmin)*dg,[1,1,2]);
        end
    end
    for u=1:U
        [idx,p,vList]=get_band_fields(tx,u,bw);
        for ell=1:size(idx,1)
            k=tx(u).users(idx(ell,1)); sc=idx(ell,2);
            if sc<1 || sc>numel(PI),continue;end
            sig=p(ell)*abs(HS{u,k}'*vList{ell})^2;
            den=noise+hI(k)*PI(sc);
            denGrad=zeros(U,2);
            for v=1:U
                if v==u,continue;end
                [idxV,pV,vV]=get_band_fields(tx,v,bw);
                for ee=1:size(idxV,1)
                    if idxV(ee,2)==sc
                        inter=pV(ee)*abs(HS{v,k}'*vV{ee})^2;
                        den=den+inter;
                        denGrad(v,:)=denGrad(v,:)+inter*squeeze(gainGrad(v,k,:)).';
                    end
                end
            end
            den=max(den,1e-30);
            coeff=1/(log(2)*(den+sig));
            gradRate(u,:)=gradRate(u,:)+sig*coeff*squeeze(gainGrad(u,k,:)).'*Bsc;
            for v=1:U
                if v~=u
                    gradRate(v,:)=gradRate(v,:)-sig/den*coeff*denGrad(v,:)*Bsc;
                end
            end
        end
    end
end
end
function z=ternary_shadow_z(Shadow,sc,band)
if band==1,z=Shadow.a2g_pu_z(sc);else,z=Shadow.a2g_wu_z(sc);end
end
function [g,gradLog]=a2g_gain_and_gradient(q,w,H,f,c,A,useShadow,z)
% gradLog = gradient of log(g), not gradient of g.
d=q-w; r2=sum(d.^2); r=sqrt(r2); D=r2+H^2;
theta=atan2d(H,max(r,1e-9));
p=1/(1+A.a*exp(-A.b*(theta-A.a)));
if r<1e-9
    gradTheta=[0,0];
else
    gradTheta=-(180/pi)*H/D*d/r;
end
gradP=A.b*p*(1-p)*gradTheta;
eta=p*A.etaLoS_dB+(1-p)*A.etaNLoS_dB;
gradEta=(A.etaLoS_dB-A.etaNLoS_dB)*gradP;
if useShadow
    variance=p*A.sigmaLoS_dB^2+(1-p)*A.sigmaNLoS_dB^2;
    sd=sqrt(max(variance,1e-30));
    sh=sd*z;
    gradSh=z*(A.sigmaLoS_dB^2-A.sigmaNLoS_dB^2)/(2*sd)*gradP;
else
    sh=0; gradSh=[0,0];
end
pl=20*log10(4*pi*f*sqrt(D)/c)+eta+sh;
g=10^(-pl/10);
gradLog=-d/D-(log(10)/10)*(gradEta+gradSh);
% FSPL is 20log10(distance), so its log-gain derivative is -2*d/D.
gradLog=gradLog-d/D;
end
function [fAvg,Lworst] = eval_actual_slot_objective_for_traj(Qslot,WS,WP,WW,w_b,w_wifi,H,...
        f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,seedBase,...
        useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,...
        methodMode,nAvg)
% Evaluate reoptimized and protected communication variables at each point.
% Worst leakage across identical channel samples is used for the constraints.
fAvg=0;
Lworst=zeros(numel(GammaLic)+numel(GammaUn),1);
for ss=1:max(nAvg,1)
    [rate,leakL,leakU]=eval_slot_multi_qos_bf(Qslot,WS,WP,WW,w_b,w_wifi,H,...
        f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,seedBase+1000*ss,...
        useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
    fAvg=fAvg+rate;
    Lworst=max(Lworst,[leakL(:)./GammaLic(:);leakU(:)./GammaUn(:)]);
end
fAvg=fAvg/max(nAvg,1);
end
function Qfixed=repair_multi_trajectory_cvx(Qraw,area,stepMax,qStart,qEnd,Dmin_UAV)
% Joint convex projection with a safe collision-free straight-line anchor.
% Collision halfspaces linearized at the anchor are an INNER approximation;
% the safe anchor is a fallback, never an unchecked per-UAV projection.
U=size(Qraw,1); N=size(Qraw,2);
anchor=zeros(U,N,2);
for u=1:U
    if norm(qEnd(u,:)-qStart(u,:))>(N-1)*stepMax+1e-8
        error('UAV %d: endpoints unreachable.',u);
    end
    for n=1:N
        t=(n-1)/(N-1);
        anchor(u,n,:)=(1-t)*qStart(u,:)+t*qEnd(u,:);
    end
end
if ~check_multi_traj_feasibility(anchor,qStart,qEnd,stepMax,area,Dmin_UAV)
    error(['Straight-line collision-free anchor unavailable. Supply a ' ...
           'collision-free initial trajectory for this geometry.']);
end
if check_multi_traj_feasibility(Qraw,qStart,qEnd,stepMax,area,Dmin_UAV)
    Qfixed=Qraw; return;
end
Xraw=reshape(permute(Qraw,[2 3 1]),N,2*U);
X0=reshape(permute(anchor,[2 3 1]),N,2*U);
Qfixed=anchor;
try
    cvx_begin quiet
        variable X(N,2*U)
        minimize(sum_square(reshape(X-Xraw,N*2*U,1)))
        subject to
            X(:)>=0; X(:)<=area;
            for u=1:U
                cols=2*u-1:2*u;
                X(1,cols)==qStart(u,:);
                X(N,cols)==qEnd(u,:);
                for n=1:N-1
                    norm(X(n+1,cols)-X(n,cols),2)<=stepMax;
                end
            end
            for n=1:N
                for u=1:U-1
                    for v=u+1:U
                        cu=2*u-1:2*u; cv=2*v-1:2*v;
                        d0=X0(n,cu)-X0(n,cv);
                        2*d0*(X(n,cu)-X(n,cv)).'-sum(d0.^2)>=Dmin_UAV^2;
                    end
                end
            end
    cvx_end
    if contains(lower(cvx_status),'solved') && all(isfinite(X(:)))
        candidate=permute(reshape(X,N,2,U),[3 1 2]);
        if check_multi_traj_feasibility(candidate,qStart,qEnd,stepMax,area,Dmin_UAV)
            Qfixed=candidate;
        else
            warning('Joint collision-safe repair failed feasibility check; using safe anchor.');
        end
    else
        warning('Joint collision-safe repair status %s; using safe anchor.',cvx_status);
    end
catch ME
    warning('Joint collision-safe repair: %s; using safe anchor.',ME.message);
    try,cvx_clear;catch,end
end
assert(check_multi_traj_feasibility(Qfixed,qStart,qEnd,stepMax,area,Dmin_UAV), ...
    'Unable to construct a collision-free feasible path.');
end
function report_multi_traj_violations(Q,qStart,qEnd,stepMax,area)
for u=1:size(Q,1)
    X=squeeze(Q(u,:,:));
    steps=sqrt(sum(diff(X,1,1).^2,2));
    fprintf('  UAV %d: start error %.3g m; end error %.3g m; max step %.9f / %.9f m; coordinate range [%.5f, %.5f].\n', ...
        u,norm(X(1,:)-qStart(u,:)),norm(X(end,:)-qEnd(u,:)), ...
        max(steps),stepMax,min(X(:)),max(X(:)));
end
end
function ok=check_multi_traj_feasibility(Q,qStart,qEnd,stepMax,area,Dmin_UAV)
ok=true;
for u=1:size(Q,1)
    X=squeeze(Q(u,:,:));
    tolFeas=1e-3; % Numerical CVX feasibility tolerance only (meters).
    ok=ok && norm(X(1,:)-qStart(u,:))<tolFeas && ...
        norm(X(end,:)-qEnd(u,:))<tolFeas && ...
        all(X(:)>=-tolFeas) && all(X(:)<=area+tolFeas) && ...
        all(sqrt(sum(diff(X,1,1).^2,2))<=stepMax+tolFeas);
end
if nargin>=6 && Dmin_UAV>0
    for n=1:size(Q,2)
        for u=1:size(Q,1)-1
            for v=u+1:size(Q,1)
                duv=squeeze(Q(u,n,:)-Q(v,n,:));
                ok=ok && norm(duv,2)>=Dmin_UAV-tolFeas;
            end
        end
    end
end
end
function avgRate = evaluate_multi_trajectory_qos_bf(Q,WS,WP,WW,w_b,w_wifi,H,...
        f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,seedBase,...
        useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode)
N = size(Q,2);
sumRate = 0;
for n = 1:N
    sumRate = sumRate + eval_slot_multi_qos_bf(...
        squeeze(Q(:,n,:)),WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
        A2G,n_pbs,n_wifi,Shadow,P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,...
        GammaLic,GammaUn,Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,...
        seedBase+n,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode);
end
avgRate = sumRate/N;
end
function [slotRate,leakLic,leakUn,txBest,userRate,diagnostics] = eval_slot_multi_qos_bf(Qslot,WS,WP,WW,w_b,w_wifi,H,...
        f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,seedNow,...
        useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode,compareMode,stageDebug,forcedAssoc)
if ~exist('compareMode','var') || isempty(compareMode), compareMode=0; end
if ~exist('stageDebug','var') || isempty(stageDebug),stageDebug=false;end
if ~exist('forcedAssoc','var'),forcedAssoc=[];end
% MATLAB requires persistent declarations at function scope before executable
% statements, not inside an AO iteration loop.
persistent aoRejectCount aoRejectInfeasible aoRejectRate aoRejectNonfinite
if isempty(aoRejectCount)
    aoRejectCount = 0;
    aoRejectInfeasible = 0;
    aoRejectRate = 0;
    aoRejectNonfinite = 0;
end
% ============================================================
% INNER AO: 3 C-UAV DUAL-BAND INDEPENDENT-BUDGET CBF-MM
% ------------------------------------------------------------
% Fixed Qslot and fixed instantaneous channels:
%   Block 1: User association.
%   Block 2: Licensed subchannel allocation.
%   Block 3: Licensed CBF-MM beamforming + KKT/water-filling power.
%   Block 4: Unlicensed allocation for all associated users.
%   Block 5: Unlicensed CBF-MM beamforming + KKT/water-filling power.
%   Block 6: Measure BOTH initial aggregate PU and WU interference.
%   Block 7: Integrated four-case AI-IBPC:
%            protection + rate-aware power adjustment.
% There is NO separate post-AI-IBPC protection/clipping block.
%
% The unlicensed band uses its own fixed per-UAV budget Pmax_un; no cross-band transfer is allowed.
%
% These blocks are repeated for AO_AP_iter or until the slot sum-rate
% converges. The previous AO iterate is used in the allocation metric through
% inter-UAV co-channel interference terms,so the loop is not a dummy repeat.
% ============================================================
[HsuL,HsuU,HpuL,HwuU,hp2,hw2,hu2L_all] = gen_multi_channels_vec(...
    Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
    A2G,n_pbs,n_wifi,Shadow,seedNow,useRayleigh,useShadow,useShadowA2G,Nt);
U = size(Qslot,1);
Kglob = size(WS,1);
J = numel(P_PBS_j);
M = numel(P_WIFI_m);
AO_AP_iter = max(1,round(AO_AP_iter));
AO_tol = 1e-4;
txPrev = repmat(struct('users',[],'idxLic',zeros(0,2),'pLic',[],'vLic',{{}},...
                       'idxUn',zeros(0,2),'pUn',[],'vUn',{{}}),U,1);
% The all-silent configuration is a feasible initial accepted AO iterate.
% Every subsequent accepted communication update is checked against the
% SAME realized channels and SAME global two-band sum-rate objective.
slotRateOld = 0;
txBest = txPrev;
slotRateBest = 0;
acceptedAOHistory = nan(AO_AP_iter+1,1);
acceptedAOHistory(1) = 0;
acceptedCount = 0;
diagnostics=struct('transferCount',0,'donorLossMbps',0);
for itAO = 1:AO_AP_iter
    % ===== Block 1: user association for fixed Qslot =====
    % Highest licensed-band channel-gain association.
    % Each SU is associated with the C-UAV providing the largest
    % instantaneous licensed-band channel gain ||h_{u,k}^{lic}[n]||^2.
    assoc = associate_users_highest_channel_gain(HsuL);
    if ~isempty(forcedAssoc)
        assert(numel(forcedAssoc)==Kglob && all(forcedAssoc>=1) && ...
            all(forcedAssoc<=U),'Invalid forced association');
        assoc=forcedAssoc(:);
    end
    tx = repmat(struct('users',[],'idxLic',zeros(0,2),'pLic',[],'vLic',{{}},...
                       'idxUn',zeros(0,2),'pUn',[],'vUn',{{}}),U,1);
    for u = 1:U
        tx(u).users = find(assoc == u).';
    end
    %% ===== Block 2: licensed Hungarian + CBF-MM + KKT =====
    for u = 1:U
        users_u = tx(u).users;
        if isempty(users_u)
            continue;
        end
        HsuL_u = HsuL(u,users_u);
        HpuL_u = HpuL(u,:);
        hp2_u  = hp2(users_u);
        extraNoiseL = previous_interuav_noise_matrix(...
            txPrev,HsuL,u,users_u,J,'lic');
        rhoL = assign_licensed_vec_extra_noise(...
            HsuL_u,HpuL_u,hp2_u,P_PBS_j,sigma2_lic,extraNoiseL,...
            GammaLic,Pmax_lic,Bsc_lic,numel(users_u),J,...
            'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,...
            Pmax_lic/max(J,1));
        [tx(u).pLic,tx(u).idxLic,tx(u).vLic] = solve_band_mm_given_rho(...
            rhoL,HsuL_u,HpuL_u,hp2_u,P_PBS_j,sigma2_lic,...
            GammaLic,Pmax_lic,MM_iter,MM_leakFrac,MM_reg_alpha);
    end
    % Do not clip yet: complete both interference-unaware allocations first.
    %% ===== Block 3: all-user unlicensed Hungarian + CBF-MM + KKT =====
    % No QoS gating: every associated SU is eligible on both bands.
    for u = 1:U
        users_u = tx(u).users;
        Pmax_un_u=Pmax_un;
        if compareMode>=2 && compareMode<=4
            % Estimate interference condition from the already-allocated licensed band.
            % Reserve ONLY for UAVs contributing to an unsafe licensed SC.
            licI=aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
            unsafeLic=licI(:)>GammaLic(:)*(1+1e-9);
            localIdx=tx(u).idxLic;
            if ~isempty(localIdx) && any(unsafeLic(localIdx(:,2)))
                Pmax_un_u=Pmax_un*(1-0.1*(compareMode-1));
            end
        end
        if isempty(users_u) || Pmax_un_u <= 1e-15
            continue;
        end
        needUn_local = true(numel(users_u),1); % Every associated SU is eligible
        HsuU_u = HsuU(u,users_u);
        HwuU_u = HwuU(u,:);
        hw2_u  = hw2(users_u);
        extraNoiseU = previous_interuav_noise_matrix(...
            txPrev,HsuU,u,users_u,M,'un');
        rhoU = assign_unlicensed_vec_extra_noise(...
            HsuU_u,HwuU_u,hw2_u,P_WIFI_m,sigma2_un,extraNoiseU,...
            GammaUn,Pmax_un_u,Bsc_un,numel(users_u),M,needUn_local(:),...
            'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,...
            Pmax_un_u/max(M,1));
        [tx(u).pUn,tx(u).idxUn,tx(u).vUn] = solve_band_mm_given_rho(...
            rhoU,HsuU_u,HwuU_u,hw2_u,P_WIFI_m,sigma2_un,...
            GammaUn,Pmax_un_u,MM_iter,MM_leakFrac,MM_reg_alpha);
    end
    %% ===== Block 4: measure both PRE-CONTROL aggregate interferences =====
    % Aggregate across ALL C-UAVs on each corresponding protected channel.
    % These are the raw initial allocations, before any aggregate clipping.
    Ilic_initial = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic'); %#ok<NASGU>
    Iun_initial  = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un'); %#ok<NASGU>
    %% ===== Block 5: integrated AI-IBPC four-case protection =====
    % Interference is measured AFTER association, subchannel allocation,
    % CBF-MM beamforming and initial KKT power allocation.
    %
    % Case 1: licensed high, unlicensed safe -> rate-aware licensed reduction
    %         plus rate-aware unlicensed increase using only unlicensed budget
    %         and WU-interference headroom.
    % Case 2: unlicensed high, licensed safe -> mirror of Case 1.
    % Case 3: neither high -> keep the original power allocation unchanged.
    % Case 4: both high -> rate-aware reductions on both bands until both
    %         protection constraints are satisfied.
    %
    % Protection is part of AI-IBPC itself. There is NO separate final
    % safety-clipping/protection block after this controller.
    txRaw=tx;
    rateFnLocal=@(z)diagnostic_tx_rate(z,HsuL,HsuU,hp2,hw2,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);
    [tx,caseStats]=ai_ibpc_integrated_four_case_rateaware(...
        txRaw,HpuL,HwuU,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,rateFnLocal);
    diagnostics=struct('transferCount',0,'donorLossMbps',0);
    if stageDebug
        diagnostics.rawTx=txRaw;
    end
    % The controller processes each SU and updates shared-channel interference.
    %% ===== Block 6: rate evaluation and AO acceptance =====
    %% ===== Convergence check for fixed-Qslot AO =====
    slotRateLic = compute_global_sum_rate_band_interuav(tx,HsuL,hp2,P_PBS_j,...
        sigma2_lic,Bsc_lic,'lic');
    slotRateUn  = compute_global_sum_rate_band_interuav(tx,HsuU,hw2,P_WIFI_m,...
        sigma2_un,Bsc_un,'un');
    slotRateNew = slotRateLic + slotRateUn;
    % The preceding power-protection routines ensure per-band feasibility.
    % Reject any candidate that fails an explicit feasibility check.
    IlicNow = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
    IunNow = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un');
    candidateFeasible = all(IlicNow(:) <= GammaLic(:)*(1+1e-6)) && ...
        all(IunNow(:) <= GammaUn(:)*(1+1e-6));
    for uu = 1:U
        candidateFeasible = candidateFeasible && ...
            sum(tx(uu).pLic(:)) <= Pmax_lic*(1+1e-6) && ...
            sum(tx(uu).pUn(:)) <= Pmax_un*(1+1e-6);
    end
    % Only accept a complete resource-allocation sweep, never assume that
    % Hungarian, MM, or KKT individually guarantee global improvement.
    % A rejected AO proposal is not a solver error. Hungarian, MM, and
    % clipped KKT blocks do not jointly form a guaranteed ascent step.
    % Retain the incumbent rather than accepting a lower-rate proposal.
    % Terminating immediately avoids expensive, repeatedly rejected sweeps.
    % Avoid printing on every finite-difference/channel-sample evaluation.
    rateDecreased = isfinite(slotRateNew) && ...
        slotRateNew < slotRateBest - 1e-10*max(1,abs(slotRateBest));
    if ~candidateFeasible || ~isfinite(slotRateNew) || rateDecreased
        aoRejectCount = aoRejectCount + 1;
        if ~candidateFeasible
            aoRejectInfeasible=aoRejectInfeasible+1;
            cause='constraint violation';
        elseif ~isfinite(slotRateNew)
            aoRejectNonfinite=aoRejectNonfinite+1;
            cause='nonfinite rate';
        else
            aoRejectRate=aoRejectRate+1;
            cause='rate decrease';
        end
        % Show two representative failures and occasional aggregate statistics.
        % This avoids flooding the console during finite-difference SCA.
        if false % VTC-style concise output; safety checks still enforced
            licRatio=max(IlicNow(:)./max(GammaLic(:),realmin));
            unRatio=max(IunNow(:)./max(GammaUn(:),realmin));
            licBudget=max(arrayfun(@(z)sum(z.pLic(:)),tx))/max(Pmax_lic,realmin);
            unBudget=max(arrayfun(@(z)sum(z.pUn(:)),tx))/max(Pmax_un,realmin);
            fprintf(['Inner AO stopped (%s): candidate %.6g Mbps, ' ...
                'incumbent %.6g Mbps; Ilic %.4g, Iun %.4g, ' ...
                'Plic %.4g, Pun %.4g [ratios; limit 1].\n'], ...
                cause,slotRateNew/1e6,slotRateBest/1e6, ...
                licRatio,unRatio,licBudget,unBudget);
            if mod(aoRejectCount,500)==0
                fprintf('AO rejection totals: infeasible=%d, rate=%d, nonfinite=%d.\n', ...
                    aoRejectInfeasible,aoRejectRate,aoRejectNonfinite);
            end
        end
        if ~stageDebug,diagnostics=struct('transferCount',0,'donorLossMbps',0);end
        break;
    end
    previousAcceptedRate = slotRateBest;
    txBest = tx;
    slotRateBest = max(slotRateBest,slotRateNew);
    acceptedCount = acceptedCount + 1;
    acceptedAOHistory(acceptedCount+1) = slotRateBest;
    txPrev = txBest;
    slotRateOld = slotRateBest;
    if acceptedCount > 1 && ...
            (slotRateBest-previousAcceptedRate)/max(1,abs(previousAcceptedRate)) <= AO_tol
        break;
    end
end
% Return best safe AO iterate,not necessarily the last one.
% ------------------------------------------------------------------
% Final verification checks after each slot. These assertions do not change
% the algorithm. They only stop the simulation if the selected safe AO iterate
% violates any aggregate interference or per-UAV power-budget constraint.
Ilic_best = aggregate_interference_by_channel(txBest,HpuL,GammaLic,'lic');
Iun_best  = aggregate_interference_by_channel(txBest,HwuU,GammaUn,'un');
assert(all(Ilic_best(:) <= GammaLic(:).*(1+1e-6)),...
    'Licensed aggregate interference violation after final protection.');
assert(all(Iun_best(:) <= GammaUn(:).*(1+1e-6)),...
    'Unlicensed aggregate interference violation after final protection.');
for uu = 1:U
    pLicSum = 0;
    pUnSum  = 0;
    if isfield(txBest(uu),'pLic') && ~isempty(txBest(uu).pLic)
        pLicSum = sum(txBest(uu).pLic(:));
    end
    if isfield(txBest(uu),'pUn') && ~isempty(txBest(uu).pUn)
        pUnSum = sum(txBest(uu).pUn(:));
    end
    assert(pLicSum <= Pmax_lic*(1+1e-6), ...
        'Licensed AI-IBPC power-budget violation.');
    assert(pUnSum <= Pmax_un*(1+1e-6), ...
        'Unlicensed AI-IBPC power-budget violation.');
end
slotRateLic = compute_global_sum_rate_band_interuav(txBest,HsuL,hp2,P_PBS_j,...
    sigma2_lic,Bsc_lic,'lic');
slotRateUn  = compute_global_sum_rate_band_interuav(txBest,HsuU,hw2,P_WIFI_m,...
    sigma2_un,Bsc_un,'un');
slotRate = slotRateLic + slotRateUn;
userRate=compute_global_user_rate_band_interuav(txBest,HsuL,hp2,P_PBS_j,sigma2_lic,Bsc_lic,Kglob,'lic')+ ...
         compute_global_user_rate_band_interuav(txBest,HsuU,hw2,P_WIFI_m,sigma2_un,Bsc_un,Kglob,'un');
% Actual aggregate post-protection leakage, available for trajectory SCA.
leakLic = aggregate_interference_by_channel(txBest,HpuL,GammaLic,'lic');
leakUn  = aggregate_interference_by_channel(txBest,HwuU,GammaUn,'un');
end
function extraNoise = previous_interuav_noise_matrix(txPrev,HsuFull,servingUAV,users_u,S,bandName)
% Previous-iterate inter-UAV co-channel interference used in the AO
% assignment metric. Size: (#local users) x (#subchannels).
extraNoise = zeros(numel(users_u),S);
if isempty(txPrev)
    return;
end
for kk = 1:numel(users_u)
    kglob = users_u(kk);
    for s = 1:S
        extraNoise(kk,s) = inter_uav_power_same_sc(txPrev,HsuFull,servingUAV,kglob,s,bandName);
    end
end
end
function rho = assign_licensed_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
% Licensed assignment with previous-AO inter-UAV interference in the metric.
Rmat = zeros(K,S);
if isempty(extraNoise)
    extraNoise = zeros(K,S);
end
for k = 1:K
    for s = 1:S
        denom = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            v = mrt_unit(Hsu{k}); % Initial allocation: no protection
        else
            v = mrt_unit(Hsu{k});
        end
        gSig  = abs(Hsu{k}'*v)^2;
        gLeak = abs(Hprot{s}'*v)^2;
        p_app = Pmax/max(S,1); % No pre-control interference cap
        Rmat(k,s) = Bsc*log2(1 + max(p_app*gSig/denom,0));
    end
end
if strcmpi(allocMode,'greedy')
    pairs = assign_greedy_max(Rmat);
else
    pairs = assign_hungarian_max(Rmat);
end
rho = zeros(K,S);
for ii = 1:size(pairs,1)
    rho(pairs(ii,1),pairs(ii,2)) = 1;
end
end
function rho = assign_unlicensed_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
% Unlicensed assignment with previous-AO inter-UAV interference in the metric.
rho = zeros(K,S);
activeUsers = find(needUn(:));
if isempty(activeUsers) || Pmax <= 0
    return;
end
if isempty(extraNoise)
    extraNoise = zeros(K,S);
end
Ka = numel(activeUsers);
Rmat = zeros(Ka,S);
for aa = 1:Ka
    k = activeUsers(aa);
    for s = 1:S
        denom = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            v = mrt_unit(Hsu{k}); % Initial allocation: no protection
        else
            v = mrt_unit(Hsu{k});
        end
        gSig  = abs(Hsu{k}'*v)^2;
        gLeak = abs(Hprot{s}'*v)^2;
        p_app = Pmax/max(S,1); % No pre-control interference cap
        Rmat(aa,s) = Bsc*log2(1 + max(p_app*gSig/denom,0));
    end
end
if strcmpi(allocMode,'greedy')
    pairsLocal = assign_greedy_max(Rmat);
else
    pairsLocal = assign_hungarian_max(Rmat);
end
for ii = 1:size(pairsLocal,1)
    aa = pairsLocal(ii,1);
    s  = pairsLocal(ii,2);
    rho(activeUsers(aa),s) = 1;
end
end
function rho = assign_licensed_iac_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,alphaFallback,pRef)
% IAC licensed assignment with previous-AO inter-UAV interference.
Rmat = zeros(K,S);
if isempty(extraNoise)
    extraNoise = zeros(K,S);
end
lambdaLeak = 1.00;
lambdaRx   = 0.10;
lambdaIuav = 0.25;
for k = 1:K
    for s = 1:S
        denomRx = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
        else
            v = mrt_unit(Hsu{k});
        end
        gSig  = abs(Hsu{k}'*v)^2;
        gLeak = abs(Hprot{s}'*v)^2;
        p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
        rateEst = Bsc*log2(1 + max(p_app*gSig/denomRx,0));
        leakRatio = (p_app*gLeak) / max(GammaVec(s),1e-30);
        rxRatio   = (hI(k)*P_I_vec(s)) / max(sigma2,1e-30);
        iUavRatio = extraNoise(k,s) / max(sigma2,1e-30);
        iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio) + lambdaIuav*log1p(iUavRatio);
        Rmat(k,s) = rateEst / iacPenalty;
    end
end
if strcmpi(allocMode,'greedy')
    pairs = assign_greedy_max(Rmat);
else
    pairs = assign_hungarian_max(Rmat);
end
rho = zeros(K,S);
for ii = 1:size(pairs,1)
    rho(pairs(ii,1),pairs(ii,2)) = 1;
end
end
function rho = assign_unlicensed_iac_vec_extra_noise(Hsu,Hprot,hI,P_I_vec,sigma2,extraNoise,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,alphaFallback,pRef)
% IAC unlicensed assignment with previous-AO inter-UAV interference.
rho = zeros(K,S);
activeUsers = find(needUn(:));
if isempty(activeUsers) || Pmax <= 0
    return;
end
if isempty(extraNoise)
    extraNoise = zeros(K,S);
end
Ka = numel(activeUsers);
Rmat = zeros(Ka,S);
lambdaLeak = 1.00;
lambdaRx   = 0.10;
lambdaIuav = 0.25;
for aa = 1:Ka
    k = activeUsers(aa);
    for s = 1:S
        denomRx = max(hI(k)*P_I_vec(s) + sigma2 + extraNoise(k,s),1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
        else
            v = mrt_unit(Hsu{k});
        end
        gSig  = abs(Hsu{k}'*v)^2;
        gLeak = abs(Hprot{s}'*v)^2;
        p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
        rateEst = Bsc*log2(1 + max(p_app*gSig/denomRx,0));
        leakRatio = (p_app*gLeak) / max(GammaVec(s),1e-30);
        rxRatio   = (hI(k)*P_I_vec(s)) / max(sigma2,1e-30);
        iUavRatio = extraNoise(k,s) / max(sigma2,1e-30);
        iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio) + lambdaIuav*log1p(iUavRatio);
        Rmat(aa,s) = rateEst / iacPenalty;
    end
end
if strcmpi(allocMode,'greedy')
    pairsLocal = assign_greedy_max(Rmat);
else
    pairsLocal = assign_hungarian_max(Rmat);
end
for ii = 1:size(pairsLocal,1)
    aa = pairsLocal(ii,1);
    s  = pairsLocal(ii,2);
    rho(activeUsers(aa),s) = 1;
end
end
function Ruser = compute_global_user_rate_band_interuav(tx,HsuFull,hI,P_I_vec,sigma2,Bsc,Kglob,bandName)
% Returns per-global-user rates for one band including inter-UAV co-channel interference.
Ruser = zeros(Kglob,1);
U = numel(tx);
for u = 1:U
    if strcmpi(bandName,'lic')
        idx = tx(u).idxLic; p = tx(u).pLic; vList = tx(u).vLic;
    else
        idx = tx(u).idxUn;  p = tx(u).pUn;  vList = tx(u).vUn;
    end
    users_u = tx(u).users;
    for ell = 1:size(idx,1)
        local_k = idx(ell,1);
        sc      = idx(ell,2);
        kglob   = users_u(local_k);
        v       = vList{ell};
        signal = p(ell) * abs(HsuFull{u,kglob}' * v)^2;
        denom  = hI(kglob)*P_I_vec(sc) + sigma2;
        denom  = denom + inter_uav_power_same_sc(tx,HsuFull,u,kglob,sc,bandName);
        denom  = max(denom,1e-30);
        SINR = signal/denom;
        Ruser(kglob) = Ruser(kglob) + Bsc*log2(1 + max(SINR,0));
    end
end
end
function sumRate = compute_global_sum_rate_band_interuav(tx,HsuFull,hI,P_I_vec,sigma2,Bsc,bandName)
% Sum-rate for one band including inter-UAV co-channel interference.
sumRate = 0;
U = numel(tx);
for u = 1:U
    if strcmpi(bandName,'lic')
        idx = tx(u).idxLic; p = tx(u).pLic; vList = tx(u).vLic;
    else
        idx = tx(u).idxUn;  p = tx(u).pUn;  vList = tx(u).vUn;
    end
    users_u = tx(u).users;
    for ell = 1:size(idx,1)
        local_k = idx(ell,1);
        sc      = idx(ell,2);
        kglob   = users_u(local_k);
        v       = vList{ell};
        signal = p(ell) * abs(HsuFull{u,kglob}' * v)^2;
        denom  = hI(kglob)*P_I_vec(sc) + sigma2;
        denom  = denom + inter_uav_power_same_sc(tx,HsuFull,u,kglob,sc,bandName);
        denom  = max(denom,1e-30);
        SINR = signal/denom;
        sumRate = sumRate + Bsc*log2(1 + max(SINR,0));
    end
end
end
function Iuav = inter_uav_power_same_sc(tx,HsuFull,servingUAV,victimUserGlobal,sc,bandName)
% Interference at victimUserGlobal caused by all other UAVs using the same subchannel.
Iuav = 0;
U = numel(tx);
for u2 = 1:U
    if u2 == servingUAV
        continue;
    end
    if strcmpi(bandName,'lic')
        idx2 = tx(u2).idxLic; p2 = tx(u2).pLic; vList2 = tx(u2).vLic;
    else
        idx2 = tx(u2).idxUn;  p2 = tx(u2).pUn;  vList2 = tx(u2).vUn;
    end
    for ell2 = 1:size(idx2,1)
        sc2 = idx2(ell2,2);
        if sc2 == sc
            v2 = vList2{ell2};
            hInterf = HsuFull{u2,victimUserGlobal};
            Iuav = Iuav + p2(ell2) * abs(hInterf' * v2)^2;
        end
    end
end
end
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
function [tx,caseStats] = apply_perband_4case_power_control(tx,HpuL,HwuU,GammaLic,GammaUn,PmaxLic,PmaxUn,enableCompensation,detailedDebug)
if nargin < 9, detailedDebug = false; end
% USER-WISE AI-IBPC, performed only AFTER both bands have been allocated.
% Every associated SU is examined using the aggregate PU/WU interference on
% ITS selected licensed/unlicensed subchannels (all UAVs sharing each SC).
% Compensation adds power to the SAME SU's opposite-band stream only, from
% that opposite band's OWN remaining UAV budget and protected headroom.
% There is no cross-band power transfer. SU order is deterministic.
% Sequential changes update aggregate leakage before the next SU is checked.
% The final projection protects shared channels after all user adjustments.
caseStats = struct('case1_licHighOnly',0,'case2_unHighOnly',0,...
    'case3_bothHigh',0,'case4_normal',0,'licensedReduced_W',0,...
    'unlicensedReduced_W',0,'licensedCompensated_W',0,...
    'unlicensedCompensated_W',0,'usersProcessed',0);
tx = enforce_per_uav_band_budget(tx,PmaxLic,'lic');
tx = enforce_per_uav_band_budget(tx,PmaxUn,'un');
for u = 1:numel(tx)
    for loc = 1:numel(tx(u).users)
        [idxL,pL,~] = get_band_fields(tx,u,'lic');
        [idxU,pU,~] = get_band_fields(tx,u,'un');
        eL = find(idxL(:,1)==loc,1);
        eU = find(idxU(:,1)==loc,1);
        if isempty(eL) && isempty(eU), continue; end
        caseStats.usersProcessed = caseStats.usersProcessed+1;
        IL = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
        IU = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un');
        highL = ~isempty(eL) && IL(idxL(eL,2)) > GammaLic(idxL(eL,2))*(1+1e-9);
        highU = ~isempty(eU) && IU(idxU(eU,2)) > GammaUn(idxU(eU,2))*(1+1e-9);
        if false && detailedDebug
            suGlobal = tx(u).users(loc);
            fprintf('\nSU %d | UAV %d | L stream %d | U stream %d | case Lhigh=%d Uhigh=%d\n', ...
                suGlobal,u,~isempty(eL),~isempty(eU),highL,highU);
            fprintf('  UAV budget: L used %.8g / %.8g W; U used %.8g / %.8g W\n', ...
                sum(tx(u).pLic(:)),PmaxLic,sum(tx(u).pUn(:)),PmaxUn);
            if ~isempty(eL)
                fprintf('  Selected licensed SC %d: aggregate ratio %.6g\n', ...
                    idxL(eL,2),IL(idxL(eL,2))/GammaLic(idxL(eL,2)));
            end
            if ~isempty(eU)
                fprintf('  Selected unlicensed SC %d: aggregate ratio %.6g\n', ...
                    idxU(eU,2),IU(idxU(eU,2))/GammaUn(idxU(eU,2)));
            end
        end
        if highL && ~highU
            caseStats.case1_licHighOnly = caseStats.case1_licHighOnly+1;
            [tx,dp] = reduce_su_stream_to_aggregate_limit(tx,u,eL,HpuL,GammaLic,'lic');
            caseStats.licensedReduced_W = caseStats.licensedReduced_W+dp;
            if enableCompensation && ~isempty(eU)
                if false && detailedDebug
                    [ii,pp,vv] = get_band_fields(tx,u,'un');
                    ss = ii(eU,2);
                    allI = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un');
                    gg = abs(HwuU{u,ss}'*vv{eU})^2;
                    budgetLeft = max(0,PmaxUn-sum(pp(:)));
                    headLeft = max(0,GammaUn(ss)-allI(ss));
                    if gg > 1e-30
                        interferencePowerLeft = headLeft/gg;
                    else
                        interferencePowerLeft = Inf;
                    end
                    fprintf('  -> COMPENSATE UN: pre-power %.9g W | UAV budget left %.9g W | WU/PU headroom %.9g W | equivalent power headroom %.9g W\n', ...
                        pp(eU),budgetLeft,headLeft,interferencePowerLeft);
                end
                [tx,dpU] = increase_su_stream_if_safe(tx,u,eU,HwuU,GammaUn,PmaxUn,'un');
                if false && detailedDebug
                    fprintf('  -> ACTUAL UN COMPENSATION: %.9g W\n',dpU);
                    if dpU <= 1e-12
                        fprintf('  -> BLOCKED: inspect zero UAV budget or zero aggregate interference headroom above.\n');
                    end
                end
                caseStats.unlicensedCompensated_W = caseStats.unlicensedCompensated_W+dpU;
            end
        elseif ~highL && highU
            caseStats.case2_unHighOnly = caseStats.case2_unHighOnly+1;
            [tx,dp] = reduce_su_stream_to_aggregate_limit(tx,u,eU,HwuU,GammaUn,'un');
            caseStats.unlicensedReduced_W = caseStats.unlicensedReduced_W+dp;
            if enableCompensation && ~isempty(eL)
                if false && detailedDebug
                    [ii,pp,vv] = get_band_fields(tx,u,'lic');
                    ss = ii(eL,2);
                    allI = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
                    gg = abs(HpuL{u,ss}'*vv{eL})^2;
                    budgetLeft = max(0,PmaxLic-sum(pp(:)));
                    headLeft = max(0,GammaLic(ss)-allI(ss));
                    if gg > 1e-30
                        interferencePowerLeft = headLeft/gg;
                    else
                        interferencePowerLeft = Inf;
                    end
                    fprintf('  -> COMPENSATE LIC: pre-power %.9g W | UAV budget left %.9g W | WU/PU headroom %.9g W | equivalent power headroom %.9g W\n', ...
                        pp(eL),budgetLeft,headLeft,interferencePowerLeft);
                end
                [tx,dpL] = increase_su_stream_if_safe(tx,u,eL,HpuL,GammaLic,PmaxLic,'lic');
                if false && detailedDebug
                    fprintf('  -> ACTUAL LIC COMPENSATION: %.9g W\n',dpL);
                    if dpL <= 1e-12
                        fprintf('  -> BLOCKED: inspect zero UAV budget or zero aggregate interference headroom above.\n');
                    end
                end
                caseStats.licensedCompensated_W = caseStats.licensedCompensated_W+dpL;
            end
        elseif highL && highU
            caseStats.case3_bothHigh = caseStats.case3_bothHigh+1;
            [tx,dpL] = reduce_su_stream_to_aggregate_limit(tx,u,eL,HpuL,GammaLic,'lic');
            [tx,dpU] = reduce_su_stream_to_aggregate_limit(tx,u,eU,HwuU,GammaUn,'un');
            caseStats.licensedReduced_W = caseStats.licensedReduced_W+dpL;
            caseStats.unlicensedReduced_W = caseStats.unlicensedReduced_W+dpU;
        else
            caseStats.case4_normal = caseStats.case4_normal+1;
        end
    end
end
% Aggregate multi-UAV safety guard; another UAV can use the same SC.
tx = project_aggregate_band_interference(tx,HpuL,GammaLic,PmaxLic,'lic');
tx = project_aggregate_band_interference(tx,HwuU,GammaUn,PmaxUn,'un');
end

function [tx,reduced] = reduce_su_stream_to_aggregate_limit(tx,u,ell,Hprot,Gamma,band)
[idx,p,v] = get_band_fields(tx,u,band);
s = idx(ell,2); g = abs(Hprot{u,s}'*v{ell})^2;
I = aggregate_interference_by_channel(tx,Hprot,Gamma,band);
% Hold all OTHER streams fixed. The SU can only remove its own contribution.
otherI = max(0,I(s)-p(ell)*g);
newP = min(p(ell),max(0,(Gamma(s)-otherI)/max(g,1e-30)));
reduced = max(0,p(ell)-newP);
p(ell)=newP;
tx=set_band_power(tx,u,band,p);
end

function [tx,added] = increase_su_stream_if_safe(tx,u,ell,Hprot,Gamma,Pmax,band)
[idx,p,v] = get_band_fields(tx,u,band);
s=idx(ell,2);
I=aggregate_interference_by_channel(tx,Hprot,Gamma,band);
g=abs(Hprot{u,s}'*v{ell})^2;
unusedBudget=max(0,Pmax-sum(p));
headroom=max(0,Gamma(s)-I(s));
% An already unsafe shared channel cannot receive compensating power.
if I(s)>Gamma(s)*(1+1e-9), added=0; return; end
if g<=1e-30
    added=unusedBudget;
else
    added=min(unusedBudget,headroom/max(g,1e-30));
end
added=max(0,added);
p(ell)=p(ell)+added;
tx=set_band_power(tx,u,band,p);
end

function tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName)
% Project one band so aggregate interference per protected channel is safe.
%
% For each protected channel s:
%   I_s = sum_{all UAV streams on s} p_stream * |h_prot^H v_stream|^2
%
% If I_s > GammaVec(s),all active stream powers on that channel are scaled
% by GammaVec(s)/I_s. This preserves the relative power split among active
% UAVs while guaranteeing aggregate per-slot protection.
tol = 1e-12;
GammaVec = GammaVec(:);
S = numel(GammaVec);
tx = enforce_per_uav_band_budget(tx,PmaxBand,bandName);
I = aggregate_interference_by_channel(tx,Hprot,GammaVec,bandName);
for s = 1:S
    if I(s) > GammaVec(s)*(1+tol)
        scale = GammaVec(s) / max(I(s),1e-30);
        for u = 1:numel(tx)
            [idx,p,vList] = get_band_fields(tx,u,bandName); %#ok<ASGLU>
            if isempty(idx),continue; end
            onS = (idx(:,2) == s);
            if any(onS)
                p(onS) = p(onS) * scale;
                tx = set_band_power(tx,u,bandName,p);
            end
        end
    end
end
tx = enforce_per_uav_band_budget(tx,PmaxBand,bandName);
end
function tx = increase_from_aggregate_band_headroom(tx,Hprot,GammaVec,PmaxBand,bandName)
% Increase powers using ONLY the same band's unused power budget and aggregate
% interference headroom.
%
% This function does NOT receive clipped power from the other band. Therefore,% it is per-band control,not common-total power sharing and not cross-band
% transfer.
if PmaxBand <= 0
    return;
end
GammaVec = GammaVec(:);
S = numel(GammaVec);
% Ensure current point is safe before trying to increase.
tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName);
for iter = 1:20
    I = aggregate_interference_by_channel(tx,Hprot,GammaVec,bandName);
    changed = false;
    for s = 1:S
        interfHeadroom = GammaVec(s) - I(s);
        if interfHeadroom <= 1e-15
            continue;
        end
        % Collect active streams on channel s.
        entries = [];
        for u = 1:numel(tx)
            [idx,p,vList] = get_band_fields(tx,u,bandName);
            if isempty(idx),continue; end
            for ell = 1:size(idx,1)
                if idx(ell,2) == s
                    v = vList{ell};
                    gLeak = abs(Hprot{u,s}' * v)^2;
                    uBudget = max(PmaxBand - sum(p),0);
                    if gLeak > 1e-30 && uBudget > 1e-15
                        % columns: u,ell,gLeak,uBudget
                        entries = [entries; u,ell,gLeak,uBudget]; %#ok<AGROW>
                    end
                end
            end
        end
        if isempty(entries)
            continue;
        end
        % Distribute the aggregate interference headroom across active streams.
        % Equal interference-share is used to avoid biasing one UAV only.
        nE = size(entries,1);
        shareI = interfHeadroom / nE;
        for ee = 1:nE
            u     = entries(ee,1);
            ell   = entries(ee,2);
            gLeak = entries(ee,3);
            [~,p,~] = get_band_fields(tx,u,bandName);
            uBudget = max(PmaxBand - sum(p),0);
            if uBudget <= 1e-15
                continue;
            end
            dp = min(shareI / max(gLeak,1e-30),uBudget);
            if dp > 1e-15
                p(ell) = p(ell) + dp;
                tx = set_band_power(tx,u,bandName,p);
                changed = true;
            end
        end
        % Re-project immediately to remove any small aggregate overshoot.
        tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName);
    end
    if ~changed
        break;
    end
end
tx = project_aggregate_band_interference(tx,Hprot,GammaVec,PmaxBand,bandName);
end
function I = aggregate_interference_by_channel(tx,Hprot,GammaVec,bandName)
% Aggregate leakage from all UAVs to each protected receiver/channel.
S = numel(GammaVec);
I = zeros(S,1);
for u = 1:numel(tx)
    [idx,p,vList] = get_band_fields(tx,u,bandName);
    if isempty(idx),continue; end
    for ell = 1:size(idx,1)
        sc = idx(ell,2);
        if sc < 1 || sc > S
            continue;
        end
        v = vList{ell};
        gLeak = abs(Hprot{u,sc}' * v)^2;
        I(sc) = I(sc) + p(ell) * gLeak;
    end
end
end
function tx = enforce_per_uav_band_budget(tx,PmaxBand,bandName)
% Enforce each UAV's own per-band power budget.
for u = 1:numel(tx)
    [idx,p,~] = get_band_fields(tx,u,bandName); %#ok<ASGLU>
    if isempty(p),continue; end
    p = max(p(:),0);
    if sum(p) > PmaxBand
        p = p * (PmaxBand / max(sum(p),1e-30));
    end
    tx = set_band_power(tx,u,bandName,p);
end
end
function Ptot = total_band_power(tx,bandName)
Ptot = 0;
for u = 1:numel(tx)
    [~,p,~] = get_band_fields(tx,u,bandName);
    if ~isempty(p)
        Ptot = Ptot + sum(p);
    end
end
end
function [idx,p,vList] = get_band_fields(tx,u,bandName)
if strcmpi(bandName,'lic')
    idx = tx(u).idxLic;
    p = tx(u).pLic(:);
    vList = tx(u).vLic;
else
    idx = tx(u).idxUn;
    p = tx(u).pUn(:);
    vList = tx(u).vUn;
end
end
function tx = set_band_power(tx,u,bandName,p)
if strcmpi(bandName,'lic')
    tx(u).pLic = p(:);
else
    tx(u).pUn = p(:);
end
end
function [cap,leakage] = leakage_caps_vec(tx,u,Hprot,GammaVec,bandName)
% Backward-compatible helper.
% In the corrected aggregate version this function is not used for final
% protection,because final protection must consider all UAVs together.
% It is kept only in case another helper calls it.
if strcmpi(bandName,'lic')
    idx = tx(u).idxLic;
    p   = tx(u).pLic(:);
    vList = tx(u).vLic;
else
    idx = tx(u).idxUn;
    p   = tx(u).pUn(:);
    vList = tx(u).vUn;
end
L = size(idx,1);
cap = zeros(L,1);
leakage = zeros(L,1);
for ell = 1:L
    sc = idx(ell,2);
    v  = vList{ell};
    gLeak = abs(Hprot{u,sc}' * v)^2;
    cap(ell) = max(GammaVec(sc) / max(gLeak,1e-30),0);
    leakage(ell) = p(ell) * gLeak;
end
end
function [pNew,delta,nClip] = clip_power_to_cap(pOld,cap)
% Clip powers to their interference-safe caps.
pOld = pOld(:);
cap = cap(:);
if isempty(pOld)
    pNew = pOld; delta = 0; nClip = 0; return;
end
pNew = min(pOld,cap);
pNew = max(pNew,0);
diff = max(pOld - pNew,0);
delta = sum(diff);
nClip = nnz(diff > 1e-12);
end
function [pOut,addedTotal] = increase_from_own_band_headroom(pIn,cap,PmaxBand)
% Increase power using ONLY the same band's own unused budget and safe caps.
% No clipped power from the other band is used as an input.
%
% pOut satisfies:
%   sum(pOut) <= PmaxBand
%   pOut(ell) <= cap(ell)
%   pOut(ell) >= pIn(ell)
pIn = pIn(:);
cap = cap(:);
pOut = pIn;
addedTotal = 0;
if isempty(pIn) || PmaxBand <= 0
    return;
end
% First ensure the input itself does not violate caps or budget.
pOut = min(max(pOut,0),cap);
if sum(pOut) > PmaxBand
    pOut = pOut * (PmaxBand/max(sum(pOut),1e-30));
    pOut = min(pOut,cap);
end
safeHeadroom   = max(cap - pOut,0);
budgetHeadroom = max(PmaxBand - sum(pOut),0);
addBudget      = min(budgetHeadroom,sum(safeHeadroom));
if addBudget <= 1e-15
    return;
end
% Distribute own-band remaining power over streams with safe headroom.
remaining = addBudget;
for iter = 1:20
    head = max(cap - pOut,0);
    active = head > 1e-12;
    if ~any(active) || remaining <= 1e-12
        break;
    end
    add = zeros(size(pOut));
    add(active) = remaining * head(active) / sum(head(active));
    pTrial = min(pOut + add,cap);
    actual = sum(pTrial - pOut);
    pOut = pTrial;
    remaining = remaining - actual;
    if actual <= 1e-12
        break;
    end
end
% Final numerical guards.
if sum(pOut) > PmaxBand
    pOut = pOut * (PmaxBand/max(sum(pOut),1e-30));
end
pOut = min(pOut,cap);
addedTotal = sum(pOut - pIn);
end
function assoc = associate_users_highest_channel_gain(HsuL)
% Highest licensed-band channel-gain association.
% For each SU k, select the C-UAV u maximizing ||h_{u,k}^{lic}||^2.
% Only the association rule is changed; all downstream Hungarian, CBF-MM,
% KKT, AI-IBPC/IA-RSS control, and trajectory-SCA logic remain unchanged.
[U,K] = size(HsuL);
metric = -inf(U,K);
for u = 1:U
    for k = 1:K
        hL = HsuL{u,k};
        metric(u,k) = real(hL' * hL);
    end
end
[~,assoc] = max(metric,[],1);
assoc = assoc(:);
end
function Q = project_multi_trajectory(Q,area,stepMax,q_start_all,q_end_all,projIter)
U = size(Q,1);
for u = 1:U
    Qu = squeeze(Q(u,:,:));
    Qu = project_full_trajectory(Qu,area,stepMax,q_start_all(u,:),q_end_all(u,:),projIter);
    Q(u,:,:) = Qu;
end
end
function v = unit_vec(x)
nx = norm(x);
if nx < 1e-12
    v = [0,0];
else
    v = x/nx;
end
end
%% ============================================================
%                  VECTOR CHANNEL GENERATION,Nt = 8
%% ============================================================
function [HsuL,HsuU,HpuL,HwuU,hp2,hw2,hu2L_all] = gen_multi_channels_vec(...
    Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
    A2G,n_pbs,n_wifi,Shadow,seedNow,useRayleigh,useShadow,useShadowA2G,Nt)
U = size(Qslot,1);
K = size(WS,1);
J = size(WP,1);
M = size(WW,1);
HsuL = cell(U,K);
HsuU = cell(U,K);
HpuL = cell(U,J);
HwuU = cell(U,M);
hu2L_all = zeros(U,K);
rng(seedNow + 555,'twister');
[hp2,hw2] = gen_terrestrial_interference_channels(WS,w_b,w_wifi,f_pbs,f_wifi,c,...
    n_pbs,n_wifi,Shadow,useRayleigh,useShadow);
for u = 1:U
    rng(seedNow + 10000*u,'twister');
    q = Qslot(u,:);
    for k = 1:K
        gL = a2g_gain_alhourani(q,WS(k,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
        gU = a2g_gain_alhourani(q,WS(k,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
        HsuL{u,k} = gen_miso_vector(gL,Nt,useRayleigh);
        HsuU{u,k} = gen_miso_vector(gU,Nt,useRayleigh);
        hu2L_all(u,k) = norm(HsuL{u,k})^2;
    end
    for j = 1:J
        g = a2g_gain_alhourani(q,WP(j,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_pu_z(j));
        HpuL{u,j} = gen_miso_vector(g,Nt,useRayleigh);
    end
    for m = 1:M
        g = a2g_gain_alhourani(q,WW(m,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_wu_z(m));
        HwuU{u,m} = gen_miso_vector(g,Nt,useRayleigh);
    end
end
end
function h = gen_miso_vector(gLarge,Nt,useRayleigh)
% MISO channel vector Nt x 1. The expected norm^2 is approximately Nt*gLarge.
% This gives array gain to both MRT and MM while retaining spatial DoF.
if useRayleigh
    h = sqrt(max(gLarge,0)) * (randn(Nt,1) + 1j*randn(Nt,1))/sqrt(2);
else
    h = sqrt(max(gLarge,0)) * ones(Nt,1);
end
end
function [hu2L_all,hu2U_all,hp2,hw2,h_up2_all,h_uw2_all] = gen_multi_channels_avg_scalar(...
    Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
    A2G,n_pbs,n_wifi,Shadow,Nsamp,seed,useRayleigh,useShadow,useShadowA2G)
U = size(Qslot,1);
K = size(WS,1);
J = size(WP,1);
M = size(WW,1);
hu2L_all  = zeros(U,K);
hu2U_all  = zeros(U,K);
hp2       = zeros(K,1);
hw2       = zeros(K,1);
h_up2_all = zeros(U,J);
h_uw2_all = zeros(U,M);
for s = 1:Nsamp
    [a,b,c1,d1,e,f] = gen_multi_channels_scalar(Qslot,WS,WP,WW,w_b,w_wifi,H,...
        f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
        seed + s,useRayleigh,useShadow,useShadowA2G);
    hu2L_all  = hu2L_all  + a;
    hu2U_all  = hu2U_all  + b;
    hp2       = hp2       + c1;
    hw2       = hw2       + d1;
    h_up2_all = h_up2_all + e;
    h_uw2_all = h_uw2_all + f;
end
hu2L_all  = hu2L_all  / Nsamp;
hu2U_all  = hu2U_all  / Nsamp;
hp2       = hp2       / Nsamp;
hw2       = hw2       / Nsamp;
h_up2_all = h_up2_all / Nsamp;
h_uw2_all = h_uw2_all / Nsamp;
end
function [hu2L_all,hu2U_all,hp2,hw2,h_up2_all,h_uw2_all] = gen_multi_channels_scalar(...
    Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
    A2G,n_pbs,n_wifi,Shadow,seedNow,useRayleigh,useShadow,useShadowA2G)
U = size(Qslot,1);
K = size(WS,1);
J = size(WP,1);
M = size(WW,1);
hu2L_all  = zeros(U,K);
hu2U_all  = zeros(U,K);
h_up2_all = zeros(U,J);
h_uw2_all = zeros(U,M);
rng(seedNow + 555,'twister');
[hp2,hw2] = gen_terrestrial_interference_channels(WS,w_b,w_wifi,f_pbs,f_wifi,c,...
    n_pbs,n_wifi,Shadow,useRayleigh,useShadow);
for u = 1:U
    rng(seedNow + 10000*u,'twister');
    q = Qslot(u,:);
    [hu2L,hu2U,h_up2,h_uw2] = gen_uav_channels_only_scalar(...
        q,WS,WP,WW,H,f_lic,f_un,c,A2G,Shadow,useRayleigh,useShadowA2G);
    hu2L_all(u,:)  = hu2L(:).';
    hu2U_all(u,:)  = hu2U(:).';
    h_up2_all(u,:) = h_up2(:).';
    h_uw2_all(u,:) = h_uw2(:).';
end
end
function [hu2_lic,hu2_un,h_up2,h_uw2] = gen_uav_channels_only_scalar(...
    q,WS,WP,WW,H,f_lic,f_un,c,A2G,Shadow,useRayleigh,useShadowA2G)
K = size(WS,1);
J = size(WP,1);
M = size(WW,1);
hu2_lic = zeros(K,1);
hu2_un  = zeros(K,1);
for k=1:K
    gL = a2g_gain_alhourani(q,WS(k,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
    gU = a2g_gain_alhourani(q,WS(k,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
    if useRayleigh
        z1 = (randn + 1j*randn)/sqrt(2);
        z2 = (randn + 1j*randn)/sqrt(2);
        hu2_lic(k) = gL * abs(z1)^2;
        hu2_un(k)  = gU * abs(z2)^2;
    else
        hu2_lic(k) = gL;
        hu2_un(k)  = gU;
    end
end
h_up2 = zeros(J,1);
for j=1:J
    g = a2g_gain_alhourani(q,WP(j,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_pu_z(j));
    if useRayleigh
        z = (randn + 1j*randn)/sqrt(2);
        h_up2(j) = g * abs(z)^2;
    else
        h_up2(j) = g;
    end
end
h_uw2 = zeros(M,1);
for m=1:M
    g = a2g_gain_alhourani(q,WW(m,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_wu_z(m));
    if useRayleigh
        z = (randn + 1j*randn)/sqrt(2);
        h_uw2(m) = g * abs(z)^2;
    else
        h_uw2(m) = g;
    end
end
end
function [hp2,hw2] = gen_terrestrial_interference_channels(WS,w_b,w_wifi,f_pbs,f_wifi,c,...
    n_pbs,n_wifi,Shadow,useRayleigh,useShadow)
K = size(WS,1);
hp2 = zeros(K,1);
hw2 = zeros(K,1);
PL1_pbs_dB  = 20*log10(4*pi*f_pbs/c);
PL1_wifi_dB = 20*log10(4*pi*f_wifi/c);
for k=1:K
    d = max(norm(w_b - WS(k,:)),1.0);
    Xp = 0;
    if useShadow,Xp = Shadow.pbs_su_dB(k); end
    PL = PL1_pbs_dB + 10*n_pbs*log10(d) + Xp;
    g  = 10^(-PL/10);
    if useRayleigh
        z = (randn + 1j*randn)/sqrt(2);
        hp2(k) = g * abs(z)^2;
    else
        hp2(k) = g;
    end
    d = max(norm(w_wifi - WS(k,:)),1.0);
    Xw = 0;
    if useShadow,Xw = Shadow.wifi_su_dB(k); end
    PL = PL1_wifi_dB + 10*n_wifi*log10(d) + Xw;
    g  = 10^(-PL/10);
    if useRayleigh
        z = (randn + 1j*randn)/sqrt(2);
        hw2(k) = g * abs(z)^2;
    else
        hw2(k) = g;
    end
end
end
%% ============================================================
%                  VECTOR BF SLOT ALGORITHMS
%% ============================================================
function sumRate = eval_slot_aodss_mrt_vec(...
        HsuL,HsuU,HpuL,HwuU,hp2,hw2,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,allocMode)
[idxLic,pLic,vLic,idxUn,pUn,vUn] = alloc_mrt_qos_vec(...
    HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
    sigma2_lic,sigma2_un,GammaLic,GammaUn,Pmax_lic,Pmax_un,...
    QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,allocMode);
sumRate = compute_sum_rate_vec(idxLic,pLic,vLic,idxUn,pUn,vUn,...
    HsuL,HsuU,hp2,hw2,P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);
end
function sumRate = eval_slot_proposed_pb_mm_vec(...
        HsuL,HsuU,HpuL,HwuU,hp2,hw2,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,...
        MM_iter,MM_leakFrac,MM_reg_alpha)
[idxLic,pLic,vLic,idxUn,pUn,vUn] = alloc_mm_pb_qos_vec(...
    HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
    sigma2_lic,sigma2_un,GammaLic,GammaUn,Pmax_lic,Pmax_un,...
    QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,MM_iter,MM_leakFrac,MM_reg_alpha);
sumRate = compute_sum_rate_vec(idxLic,pLic,vLic,idxUn,pUn,vUn,...
    HsuL,HsuU,hp2,hw2,P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);
end
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
function [idxLic,pLic,vLic,idxUn,pUn,vUn] = alloc_mm_pb_qos_vec(...
        HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
        sigma2_lic,sigma2_un,GammaLic,GammaUn,Pmax_lic,Pmax_un,...
        QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,MM_iter,MM_leakFrac,MM_reg_alpha)
K = numel(HsuL);
J = numel(HpuL);
M = numel(HwuU);
idxLic = zeros(0,2); pLic = []; vLic = {};
idxUn  = zeros(0,2); pUn  = []; vUn  = {};
for it = 1:AO_AP_iter %#ok<NASGU>
    rhoL = assign_licensed_vec(HsuL,HpuL,hp2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,Bsc_lic,K,J,...
        'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,Pmax_lic/max(J,1));
    [pLic,idxLic,vLic] = solve_band_mm_given_rho(rhoL,HsuL,HpuL,hp2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,...
        MM_iter,MM_leakFrac,MM_reg_alpha);
    needUn = true(K,1); % All SUs eligible in both bands
    if any(needUn)
        rhoU = assign_unlicensed_vec(HsuU,HwuU,hw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,Bsc_un,K,M,needUn,...
            'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,Pmax_un/max(M,1));
        [pUn,idxUn,vUn] = solve_band_mm_given_rho(rhoU,HsuU,HwuU,hw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,...
            MM_iter,MM_leakFrac,MM_reg_alpha);
    else
        idxUn = zeros(0,2); pUn = []; vUn = {};
    end
end
end
function rho = assign_licensed_iac_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,alphaFallback,pRef)
% Interference-aware licensed channel selection (new proactive idea).
% The metric rewards useful SU gain/rate and penalizes:
%   1) received licensed interference at the SU from PBS,%   2) leakage caused by the UAV toward the PU on that licensed subchannel.
% Power is still allocated later by the same CBF-MM/KKT solver.
Rmat = zeros(K,S);
lambdaLeak = 1.00;
lambdaRx   = 0.10;
for k = 1:K
    for s = 1:S
        denomRx = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
        else
            v = mrt_unit(Hsu{k});
        end
        gSig  = abs(Hsu{k}'*v)^2;
        gLeak = abs(Hprot{s}'*v)^2;
        p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
        rateEst = Bsc*log2(1 + max(p_app*gSig/denomRx,0));
        leakRatio = (p_app*gLeak) / max(GammaVec(s),1e-30);
        rxRatio   = (hI(k)*P_I_vec(s)) / max(sigma2,1e-30);
        iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio);
        Rmat(k,s) = rateEst / iacPenalty;
    end
end
if strcmpi(allocMode,'greedy')
    pairs = assign_greedy_max(Rmat);
else
    pairs = assign_hungarian_max(Rmat);
end
rho = zeros(K,S);
for ii = 1:size(pairs,1)
    rho(pairs(ii,1),pairs(ii,2)) = 1;
end
end
function rho = assign_unlicensed_iac_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,alphaFallback,pRef)
% Interference-aware unlicensed channel selection (new proactive idea).
% Active users are all associated users only. The metric rewards useful SU
% rate and penalizes WiFi interference plus leakage toward WUs.
rho = zeros(K,S);
activeUsers = find(needUn(:));
if isempty(activeUsers) || Pmax <= 0
    return;
end
Ka = numel(activeUsers);
Rmat = zeros(Ka,S);
lambdaLeak = 1.00;
lambdaRx   = 0.10;
for aa = 1:Ka
    k = activeUsers(aa);
    for s = 1:S
        denomRx = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
        else
            v = mrt_unit(Hsu{k});
        end
        gSig  = abs(Hsu{k}'*v)^2;
        gLeak = abs(Hprot{s}'*v)^2;
        p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
        rateEst = Bsc*log2(1 + max(p_app*gSig/denomRx,0));
        leakRatio = (p_app*gLeak) / max(GammaVec(s),1e-30);
        rxRatio   = (hI(k)*P_I_vec(s)) / max(sigma2,1e-30);
        iacPenalty = 1 + lambdaLeak*leakRatio + lambdaRx*log1p(rxRatio);
        Rmat(aa,s) = rateEst / iacPenalty;
    end
end
if strcmpi(allocMode,'greedy')
    pairsLocal = assign_greedy_max(Rmat);
else
    pairsLocal = assign_hungarian_max(Rmat);
end
for ii = 1:size(pairsLocal,1)
    aa = pairsLocal(ii,1);
    s  = pairsLocal(ii,2);
    rho(activeUsers(aa),s) = 1;
end
end
function rho = assign_licensed_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
Rmat = zeros(K,S);
for k = 1:K
    for s = 1:S
        denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            % Do NOT call CVX inside the assignment matrix. Calling CVX for every
            % candidate (k,s) creates nested/non-empty CVX-scope warnings and is
            % extremely slow. Use a leakage-aware RZF metric for assignment;
            % true MM/WMMSE is applied later only to the selected pairs in
            % solve_band_mm_given_rho() or solve_band_wmmse_given_rho().
            v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
            gSig = abs(Hsu{k}'*v)^2;
            gLeak = abs(Hprot{s}'*v)^2;
        else
            v = mrt_unit(Hsu{k});
            gSig = abs(Hsu{k}'*v)^2;
            gLeak = abs(Hprot{s}'*v)^2;
        end
        p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
        SINR = p_app * gSig / denom;
        Rmat(k,s) = Bsc*log2(1 + max(SINR,0));
    end
end
if strcmpi(allocMode,'greedy')
    pairs = assign_greedy_max(Rmat);
else
    pairs = assign_hungarian_max(Rmat);
end
rho = zeros(K,S);
for ii = 1:size(pairs,1)
    rho(pairs(ii,1),pairs(ii,2)) = 1;
end
end
function rho = assign_unlicensed_vec(Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,Bsc,K,S,needUn,allocMode,bfMode,MM_iter,leakFrac,alphaFallback,pRef)
rho = zeros(K,S);
activeUsers = find(needUn(:));
if isempty(activeUsers) || Pmax <= 0
    return;
end
Ka = numel(activeUsers);
Rmat = zeros(Ka,S);
for aa = 1:Ka
    k = activeUsers(aa);
    for s = 1:S
        denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
        if strcmpi(bfMode,'mm') || strcmpi(bfMode,'wmmse') || strcmpi(bfMode,'fp') || strcmpi(bfMode,'safe-drl')
            % Do NOT call CVX inside the assignment matrix. Calling CVX for every
            % candidate (k,s) creates nested/non-empty CVX-scope warnings and is
            % extremely slow. Use a leakage-aware RZF metric for assignment;
            % true MM/WMMSE is applied later only to the selected pairs in
            % solve_band_mm_given_rho() or solve_band_wmmse_given_rho().
            v = leakage_rzf_unit(Hsu{k},Hprot{s},alphaFallback);
            gSig = abs(Hsu{k}'*v)^2;
            gLeak = abs(Hprot{s}'*v)^2;
        else
            v = mrt_unit(Hsu{k});
            gSig = abs(Hsu{k}'*v)^2;
            gLeak = abs(Hprot{s}'*v)^2;
        end
        p_app = min(Pmax/max(S,1),GammaVec(s)/max(gLeak,1e-30));
        SINR = p_app * gSig / denom;
        Rmat(aa,s) = Bsc*log2(1 + max(SINR,0));
    end
end
if strcmpi(allocMode,'greedy')
    pairsLocal = assign_greedy_max(Rmat);
else
    pairsLocal = assign_hungarian_max(Rmat);
end
for ii = 1:size(pairsLocal,1)
    aa = pairsLocal(ii,1);
    s  = pairsLocal(ii,2);
    rho(activeUsers(aa),s) = 1;
end
end
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
function [p_assigned,idxList,vList] = solve_band_mm_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,MM_iter,leakFrac,alphaFallback)
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
        [v,gSig,gLeak] = mm_unit_beam_unprotected(Hsu{k},denom,MM_iter,alphaFallback);
        a = max(gSig/denom,1e-30);
        pcap = Pmax; % No PU/WU cap in initial KKT
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
function [p_assigned,idxList,vList] = solve_band_fp_cbf_given_rho(rho,Hsu,Hprot,hI,P_I_vec,sigma2,GammaVec,Pmax,FP_iter,leakFrac,alphaFallback)
% FP-based CBF for selected user-subchannel pairs.
% CVX-free quadratic-transform style update with leakage regularization.
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
        [v,gSig,gLeak] = fp_cbf_unit_beam(Hsu{k},Hprot{s},denom,GammaVec(s),pRef,FP_iter,leakFrac,alphaFallback);
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
function v = mrt_unit(h)
nh = norm(h);
if nh < 1e-30
    v = ones(numel(h),1)/sqrt(numel(h));
else
    v = h/nh;
end
end
function [v,gSig,gLeak] = mm_unit_beam(hSU,hProt,denom,Gamma,pRef,MM_iter,leakFrac,alphaFallback)
global AIIBPC_BEAM_CALLS;
if ~isempty(AIIBPC_BEAM_CALLS), AIIBPC_BEAM_CALLS=AIIBPC_BEAM_CALLS+1; end
% MM-based leakage-aware unit-norm beam design.
% ------------------------------------------------------------
% This replaces the previous FP/SDR CVX-based beamformer.
% It solves a surrogate sequence for the useful-signal/leakage tradeoff:
%
%   maximize_v   v^H (Hs/denom - mu*Hp) v
%   subject to   ||v||_2 = 1,%
% where Hs = hSU*hSU^H and Hp = hProt*hProt^H. The leakage constraint is
% finally polished by a leakage-aware projection/RZF step. Actual PU/WU
% interference protection is still enforced later by the proposed 4-case
% per-band power clipping function.
%
% Important: this is an MM-style iterative beam update,not FP/SDR and not CVX.
Nt = numel(hSU);
Hs = hSU*hSU';
Hp = hProt*hProt';
leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
leakCapUnit = max(leakCapUnit,0);
% Initial feasible/stable beam: leakage-aware RZF.
v = leakage_rzf_unit(hSU,hProt,alphaFallback);
% If the RZF beam still violates the unit leakage cap,try null-space beam.
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-9)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
% Penalty weight for leakage. Larger pRef/Gamma means stronger protection.
mu = pRef / max(Gamma,1e-30);
% Hermitian quadratic matrix for MM surrogate/power-iteration update.
C = Hs/max(denom,1e-30) - mu*Hp;
C = (C + C')/2;
% Shifted positive-semidefinite surrogate matrix.
lamMin = min(real(eig(C)));
tau = max(0,-lamMin) + alphaFallback;
B = C + tau*eye(Nt);
B = (B + B')/2;
for it = 1:max(MM_iter,1)
    vNew = B*v;
    if norm(vNew) < 1e-30
        vNew = mrt_unit(hSU);
    else
        vNew = vNew/norm(vNew);
    end
    % Leakage polishing after each MM step.
    if leakCapUnit > 0 && abs(hProt'*vNew)^2 > leakCapUnit*(1+1e-9)
        vNew = polish_leakage_beam(vNew,hSU,hProt,leakCapUnit,alphaFallback);
    end
    if norm(vNew - v) < 1e-6
        v = vNew;
        break;
    end
    v = vNew;
end
% Final normalization and leakage polishing.
v = v / max(norm(v),1e-30);
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-6)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
gSig  = abs(hSU'*v)^2;
gLeak = abs(hProt'*v)^2;
end
function [v,gSig,gLeak] = wmmse_unit_beam(hSU,hProt,denom,Gamma,pRef,WMMSE_iter,leakFrac,alphaFallback)
% WMMSE-based leakage-aware unit-norm beam design.
% ------------------------------------------------------------
% For one selected stream,the scalar WMMSE updates are:
%   u = sqrt(pRef) h^H v / (pRef |h^H v|^2 + denom)
%   e = 1 - 2 Re{u^* sqrt(pRef) h^H v} + |u|^2(pRef |h^H v|^2 + denom)
%   w = 1/e
% Then the beam is updated from a regularized least-squares/WMMSE surrogate
% with leakage penalty to protect PU/WU receivers:
%   v <- (w|u|^2 pRef hh^H + mu Hp + alpha I)^(-1) (w u sqrt(pRef) h)
% followed by normalization and leakage polishing.
Nt = numel(hSU);
Hs = hSU*hSU';
Hp = hProt*hProt';
leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
leakCapUnit = max(leakCapUnit,0);
% Stable initial beam.
v = leakage_rzf_unit(hSU,hProt,alphaFallback);
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-9)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
mu = pRef / max(Gamma,1e-30);
for it = 1:max(WMMSE_iter,1)
    hv = hSU' * v;
    rxDen = max(pRef*abs(hv)^2 + denom,1e-30);
    u = sqrt(max(pRef,0)) * hv / rxDen;
    mse = 1 - 2*real(conj(u)*sqrt(max(pRef,0))*hv) + abs(u)^2 * rxDen;
    mse = max(real(mse),1e-12);
    w = 1/mse;
    A = w*abs(u)^2*pRef*Hs + mu*Hp + alphaFallback*eye(Nt);
    b = w*conj(u)*sqrt(max(pRef,0))*hSU;
    vNew = A \ b;
    if norm(vNew) < 1e-30
        vNew = mrt_unit(hSU);
    else
        vNew = vNew/norm(vNew);
    end
    if leakCapUnit > 0 && abs(hProt'*vNew)^2 > leakCapUnit*(1+1e-9)
        vNew = polish_leakage_beam(vNew,hSU,hProt,leakCapUnit,alphaFallback);
    end
    if norm(vNew - v) < 1e-6
        v = vNew;
        break;
    end
    v = vNew;
end
v = v/max(norm(v),1e-30);
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-6)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
gSig  = abs(hSU'*v)^2;
gLeak = abs(hProt'*v)^2;
end
function v = polish_leakage_beam(vIn,hSU,hProt,leakCapUnit,alphaFallback)
% Reduce leakage while retaining useful signal as much as possible.
Nt = numel(hSU);
% Candidate 1: leakage-aware RZF.
v1 = leakage_rzf_unit(hSU,hProt,alphaFallback);
% Candidate 2: null-space beam if possible.
Pnull = eye(Nt) - (hProt*hProt')/max(norm(hProt)^2,1e-30);
v2 = Pnull*hSU;
if norm(v2) > 1e-12
    v2 = v2/norm(v2);
else
    v2 = v1;
end
% Candidate 3: blend current beam and null-space beam.
v3 = vIn / max(norm(vIn),1e-30);
if abs(hProt'*v3)^2 > leakCapUnit && abs(hProt'*v2)^2 <= leakCapUnit*(1+1e-9)
    lo = 0; hi = 1;
    for bb = 1:40
        aa = 0.5*(lo+hi);
        vb = (1-aa)*v3 + aa*v2;
        vb = vb/max(norm(vb),1e-30);
        if abs(hProt'*vb)^2 <= leakCapUnit
            hi = aa;
        else
            lo = aa;
        end
    end
    v3 = (1-hi)*v3 + hi*v2;
    v3 = v3/max(norm(v3),1e-30);
end
cands = {v1,v2,v3};
best = cands{1};
bestScore = -inf;
for ii = 1:numel(cands)
    vc = cands{ii};
    vc = vc/max(norm(vc),1e-30);
    leak = abs(hProt'*vc)^2;
    sig  = abs(hSU'*vc)^2;
    if leak <= leakCapUnit*(1+1e-6)
        score = sig;
    else
        score = sig/(1 + 1e6*(leak/leakCapUnit - 1));
    end
    if score > bestScore
        bestScore = score;
        best = vc;
    end
end
v = best/max(norm(best),1e-30);
end
function [v,gSig,gLeak] = fp_cbf_unit_beam(hSU,hProt,denom,Gamma,pRef,FP_iter,leakFrac,alphaFallback)
% FP-based CBF unit-norm beam design.
% ------------------------------------------------------------
% Uses a quadratic-transform style update for the SINR objective:
%   log(1 + p|h^H v|^2/denom)
% combined with a leakage regularization term for PU/WU protection.
% This is CVX-free and suited for repeated Monte-Carlo simulation.
Nt = numel(hSU);
Hs = hSU*hSU';
Hp = hProt*hProt';
leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
leakCapUnit = max(leakCapUnit,0);
v = leakage_rzf_unit(hSU,hProt,alphaFallback);
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-9)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
mu = pRef / max(Gamma,1e-30);
for it = 1:max(FP_iter,1)
    hv = hSU' * v;
    y = sqrt(max(pRef,0)) * hv / max(pRef*abs(hv)^2 + denom,1e-30);
    A = abs(y)^2*pRef*Hs + mu*Hp + alphaFallback*eye(Nt);
    b = conj(y)*sqrt(max(pRef,0))*hSU;
    vNew = A \ b;
    if norm(vNew) < 1e-30
        vNew = mrt_unit(hSU);
    else
        vNew = vNew/norm(vNew);
    end
    if leakCapUnit > 0 && abs(hProt'*vNew)^2 > leakCapUnit*(1+1e-9)
        vNew = polish_leakage_beam(vNew,hSU,hProt,leakCapUnit,alphaFallback);
    end
    if norm(vNew-v) < 1e-6
        v = vNew;
        break;
    end
    v = vNew;
end
v = v/max(norm(v),1e-30);
if leakCapUnit > 0 && abs(hProt'*v)^2 > leakCapUnit*(1+1e-6)
    v = polish_leakage_beam(v,hSU,hProt,leakCapUnit,alphaFallback);
end
gSig  = abs(hSU'*v)^2;
gLeak = abs(hProt'*v)^2;
end
function [v,gSig,gLeak] = safe_drl_cbf_unit_beam(hSU,hProt,denom,Gamma,pRef,SafeIter,leakFrac,alphaFallback)
% Safe DRL-based CBF lightweight baseline.
% ------------------------------------------------------------
% It emulates a safe policy-improvement layer: generate candidate beam actions,% project unsafe candidates through the safety layer,evaluate reward with an
% interference-temperature cost,and select the safest/highest-reward action.
% This avoids heavy DNN training while preserving the safety idea in simulation.
Nt = numel(hSU);
leakCapUnit = leakFrac * Gamma / max(pRef,1e-15);
leakCapUnit = max(leakCapUnit,0);
cand = {};
cand{end+1} = mrt_unit(hSU);
cand{end+1} = leakage_rzf_unit(hSU,hProt,alphaFallback);
[c1,~,~] = mm_unit_beam(hSU,hProt,denom,Gamma,pRef,max(2,ceil(SafeIter/2)),leakFrac,alphaFallback); cand{end+1} = c1;
[c2,~,~] = wmmse_unit_beam(hSU,hProt,denom,Gamma,pRef,max(2,ceil(SafeIter/2)),leakFrac,alphaFallback); cand{end+1} = c2;
[c3,~,~] = fp_cbf_unit_beam(hSU,hProt,denom,Gamma,pRef,max(2,ceil(SafeIter/2)),leakFrac,alphaFallback); cand{end+1} = c3;
% Deterministic exploration actions around the useful-channel direction.
base = leakage_rzf_unit(hSU,hProt,alphaFallback);
Pnull = eye(Nt) - (hProt*hProt')/max(norm(hProt)^2,1e-30);
nullDir = Pnull*hSU;
if norm(nullDir) > 1e-12
    nullDir = nullDir/norm(nullDir);
else
    nullDir = base;
end
for rr = 1:max(SafeIter,1)
    beta = rr/(max(SafeIter,1)+1);
    vb = (1-beta)*base + beta*nullDir;
    if norm(vb) > 1e-30
        cand{end+1} = vb/norm(vb); %#ok<AGROW>
    end
end
bestReward = -inf;
v = cand{1};
for ii = 1:numel(cand)
    vc = cand{ii};
    vc = vc/max(norm(vc),1e-30);
    if leakCapUnit > 0 && abs(hProt'*vc)^2 > leakCapUnit*(1+1e-9)
        vc = polish_leakage_beam(vc,hSU,hProt,leakCapUnit,alphaFallback);
    end
    sig = abs(hSU'*vc)^2;
    leak = abs(hProt'*vc)^2;
    rateReward = log2(1 + max(pRef*sig/max(denom,1e-30),0));
    safetyCost = max(0,pRef*leak - Gamma)/max(Gamma,1e-30);
    reward = rateReward - 20*safetyCost^2;
    if reward > bestReward
        bestReward = reward;
        v = vc;
    end
end
v = v/max(norm(v),1e-30);
gSig  = abs(hSU'*v)^2;
gLeak = abs(hProt'*v)^2;
end
function v = leakage_rzf_unit(hSU,hProt,alpha)
Nt = numel(hSU);
A = hProt*hProt' + alpha*eye(Nt);
v = A \ hSU;
if norm(v) < 1e-30
    v = mrt_unit(hSU);
else
    v = v/norm(v);
end
end
function Ruser = compute_user_rate_band_vec(idx,p,vList,Hsu,hI,P_I_vec,sigma2,Bsc,K)
Ruser = zeros(K,1);
for ell = 1:size(idx,1)
    k = idx(ell,1);
    s = idx(ell,2);
    v = vList{ell};
    denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
    gSig = abs(Hsu{k}'*v)^2;
    SINR = p(ell)*gSig/denom;
    Ruser(k) = Ruser(k) + Bsc*log2(1 + max(SINR,0));
end
end
function sumRate = compute_sum_rate_vec(idxLic,pLic,vLic,idxUn,pUn,vUn,...
                                    HsuL,HsuU,hp2,hw2,P_PBS_j,P_WIFI_m,...
                                    sigma2_lic,sigma2_un,Bsc_lic,Bsc_un)
sumRate = 0;
for ell=1:size(idxLic,1)
    k=idxLic(ell,1); j=idxLic(ell,2);
    v = vLic{ell};
    denom = max(hp2(k)*P_PBS_j(j) + sigma2_lic,1e-30);
    gSig = abs(HsuL{k}'*v)^2;
    SINR = pLic(ell)*gSig/denom;
    sumRate = sumRate + Bsc_lic*log2(1 + max(SINR,0));
end
for ell=1:size(idxUn,1)
    k=idxUn(ell,1); m=idxUn(ell,2);
    v = vUn{ell};
    denom = max(hw2(k)*P_WIFI_m(m) + sigma2_un,1e-30);
    gSig = abs(HsuU{k}'*v)^2;
    SINR = pUn(ell)*gSig/denom;
    sumRate = sumRate + Bsc_un*log2(1 + max(SINR,0));
end
end
%% ============================================================
%                  SCALAR TRAJECTORY ALLOCATION HELPERS
%% ============================================================
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
function rhoL = assign_licensed_original(hu2L,hp2,h_up2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,Bsc_lic,K,J,allocMode)
if nargin < 11 || isempty(allocMode)
    allocMode = 'hungarian';
end
RmatL = zeros(K,J);
for k = 1:K
    for j = 1:J
        p_app = min(Pmax_lic/max(J,1),GammaLic(j)/max(h_up2(j),1e-30));
        denom = max(hp2(k)*P_PBS_j(j) + sigma2_lic,1e-30);
        SINR  = (hu2L(k)*p_app)/denom;
        RmatL(k,j) = Bsc_lic*log2(1 + max(SINR,0));
    end
end
if strcmpi(allocMode,'greedy')
    pairsL = assign_greedy_max(RmatL);
else
    pairsL = assign_hungarian_max(RmatL);
end
rhoL = zeros(K,J);
for ii = 1:size(pairsL,1)
    rhoL(pairsL(ii,1),pairsL(ii,2)) = 1;
end
end
function rhoU = assign_unlicensed_for_users(hu2U,hw2,h_uw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,Bsc_un,K,M,needUn,allocMode)
if nargin < 12 || isempty(allocMode)
    allocMode = 'hungarian';
end
rhoU = zeros(K,M);
activeUsers = find(needUn(:));
if isempty(activeUsers) || Pmax_un <= 0
    return;
end
Ka = numel(activeUsers);
RmatU = zeros(Ka,M);
for aa = 1:Ka
    k = activeUsers(aa);
    for m = 1:M
        p_app = min(Pmax_un/max(M,1),GammaUn(m)/max(h_uw2(m),1e-30));
        denom = max(hw2(k)*P_WIFI_m(m) + sigma2_un,1e-30);
        SINR  = (hu2U(k)*p_app)/denom;
        RmatU(aa,m) = Bsc_un*log2(1 + max(SINR,0));
    end
end
if strcmpi(allocMode,'greedy')
    pairsLocal = assign_greedy_max(RmatU);
else
    pairsLocal = assign_hungarian_max(RmatU);
end
for ii = 1:size(pairsLocal,1)
    aa = pairsLocal(ii,1);
    m  = pairsLocal(ii,2);
    rhoU(activeUsers(aa),m) = 1;
end
end
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
function p = kkt_waterfill_capped_original(a,Ptot,pcap)
L = numel(a);
p = zeros(L,1);
if L==0 || Ptot<=0,return; end
a = max(a(:),1e-30);
pcap = max(pcap(:),0);
if isfinite(sum(pcap)) && sum(pcap) <= Ptot
    p = pcap;
    return;
end
lam_lo = 1e-12;
lam_hi = 1e12;
for it = 1:100
    lam = 0.5*(lam_lo + lam_hi);
    p_try = max(0,1./lam - 1./a);
    p_try = min(p_try,pcap);
    if sum(p_try) > Ptot
        lam_lo = lam;
    else
        lam_hi = lam;
    end
end
p = max(0,1./lam_hi - 1./a);
p = min(p,pcap);
s = sum(p);
if s > Ptot && s > 0
    p = p * (Ptot/s);
end
end
function Ruser = compute_user_rate_band(idx,p,hu2,hI,P_I_vec,sigma2,Bsc,K)
Ruser = zeros(K,1);
for ell = 1:size(idx,1)
    k = idx(ell,1);
    s = idx(ell,2);
    denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
    SINR = hu2(k)*p(ell)/denom;
    Ruser(k) = Ruser(k) + Bsc*log2(1 + max(SINR,0));
end
end
function c_k = build_c_per_user(idx,p_assigned,hI,P_I_vec,sigma2,K)
c_k = zeros(K,1);
for ell = 1:size(idx,1)
    k = idx(ell,1);
    s = idx(ell,2);
    denom = max(hI(k)*P_I_vec(s) + sigma2,1e-30);
    c_k(k) = c_k(k) + p_assigned(ell) / denom;
end
end
function B_k = build_B_per_user(idx,Bsc,K)
B_k = zeros(K,1);
for ell = 1:size(idx,1)
    k = idx(ell,1);
    B_k(k) = B_k(k) + Bsc;
end
end
%% ============================================================
%                  ASSIGNMENT HELPERS
%% ============================================================
function pairs = assign_greedy_max(score)
[R,C] = size(score);
L = min(R,C);
pairs = zeros(0,2);
if L == 0
    return;
end
usedR = false(R,1);
usedC = false(C,1);
for ell = 1:L
    bestVal = -inf;
    bestR = 0;
    bestC = 0;
    for r = 1:R
        if usedR(r),continue; end
        for cc = 1:C
            if usedC(cc),continue; end
            if score(r,cc) > bestVal
                bestVal = score(r,cc);
                bestR = r;
                bestC = cc;
            end
        end
    end
    if bestR == 0
        break;
    end
    pairs = [pairs; bestR,bestC]; %#ok<AGROW>
    usedR(bestR) = true;
    usedC(bestC) = true;
end
end
function pairs = assign_hungarian_max(score)
if isempty(score)
    pairs = zeros(0,2);
    return;
end
if exist('matchpairs','file') == 2
    cost = -score;
    bigM = 1e12;
    pairs = matchpairs(cost,bigM,'min');
    pairs = sortrows(pairs,1);
else
    pairs = assign_bruteforce(-score);
end
end
function pairs = assign_bruteforce(cost)
[R,C] = size(cost);
L = min(R,C);
pairs = zeros(L,2);
if L==0,return; end
bestVal = inf;
if R <= C
    P = perms(1:C);
    for i=1:size(P,1)
        sel = P(i,1:R);
        val = 0;
        for r=1:R
            val = val + cost(r,sel(r));
        end
        if val < bestVal
            bestVal = val;
            pairs = [(1:R)',sel(:)];
        end
    end
else
    P = perms(1:R);
    for i=1:size(P,1)
        sel = P(i,1:C);
        val = 0;
        for cc=1:C
            val = val + cost(sel(cc),cc);
        end
        if val < bestVal
            bestVal = val;
            pairs = [sel(:),(1:C)'];
        end
    end
    pairs = pairs(1:L,:);
end
pairs = pairs(1:L,:);
end
%% ============================================================
%                  TRAJECTORY UPDATE HELPERS
%% ============================================================
function [q_next,solved] = traj_step_cvx_sca_from_gradient(q0,qPrev,qNext,...
    gradF,L0,gradL,stepMax,area,radius)
% Affine numerical first-order models of objective and normalized leakage.
% Fixed neighboring positions preserve the entire trajectory's mobility.
q_next=q0; solved=false;
if exist('cvx_begin','file')~=2
    error('CVX is required.');
end
try
    cvx_begin quiet
        variable q(2)
        maximize(gradF(:).'*(q-q0(:)))
        subject to
            norm(q-q0(:),2) <= radius;
            norm(q-qPrev(:),2) <= stepMax;
            norm(q-qNext(:),2) <= stepMax;
            q>=0;
            q<=area;
            L0(:)+gradL*(q-q0(:))<=1;
    cvx_end
    solved=contains(lower(cvx_status),'solved') && all(isfinite(q));
    if solved
        q_next=q(:).';
    end
catch ME
    warning('CVX trajectory subproblem failed: %s',ME.message);
    try,cvx_clear;catch,end
end
end
%% Old unused helper retained below; main trajectory update now uses CVX/SCA.
function q_next = traj_step_projected_gradient_mean(q0,q_prev,q_end,n,N,WS,H,f_lic,f_un,c,A2G,...
        cLic_k,Blic_k,cUn_k,Bun_k,stepMax,area,epsFD)
Kloc = size(WS,1);
grad = [0,0];
for k = 1:Kloc
    wk = WS(k,:);
    if cLic_k(k) > 0 && Blic_k(k) > 0
        [dgdx,dgdy,g0] = fd_gain_gradient(q0,wk,H,f_lic,c,A2G,epsFD);
        coeff = (Blic_k(k)/log(2)) * cLic_k(k) / max(1 + cLic_k(k)*g0,1e-12);
        grad = grad + coeff * [dgdx,dgdy];
    end
    if cUn_k(k) > 0 && Bun_k(k) > 0
        [dgdx,dgdy,g0] = fd_gain_gradient(q0,wk,H,f_un,c,A2G,epsFD);
        coeff = (Bun_k(k)/log(2)) * cUn_k(k) / max(1 + cUn_k(k)*g0,1e-12);
        grad = grad + coeff * [dgdx,dgdy];
    end
end
if norm(grad) < 1e-12
    q_candidate = q0;
else
    q_candidate = q_prev + stepMax * grad/norm(grad);
end
q_next = project_single_step(q_candidate,q_prev,q_end,n,N,stepMax,area);
end
function [dgdx,dgdy,g0] = fd_gain_gradient(q0,wk,H,f,c,A2G,epsFD)
g0  = a2g_gain_alhourani_mean(q0,wk,H,f,c,A2G);
gx1 = a2g_gain_alhourani_mean(q0+[epsFD,0],wk,H,f,c,A2G);
gx2 = a2g_gain_alhourani_mean(q0-[epsFD,0],wk,H,f,c,A2G);
gy1 = a2g_gain_alhourani_mean(q0+[0,epsFD],wk,H,f,c,A2G);
gy2 = a2g_gain_alhourani_mean(q0-[0,epsFD],wk,H,f,c,A2G);
dgdx = (gx1-gx2)/(2*epsFD);
dgdy = (gy1-gy2)/(2*epsFD);
end
function q_projected = project_single_step(q,q_prev,q_end,n,N,stepMax,area)
q_projected = min(max(q,[0,0]),[area,area]);
d = q_projected - q_prev;
nd = norm(d);
if nd > stepMax
    q_projected = q_prev + stepMax * d / max(nd,1e-12);
end
q_projected = min(max(q_projected,[0,0]),[area,area]);
remSteps = N - n;
rReach = remSteps * stepMax;
v = q_projected - q_end;
nv = norm(v);
if nv > rReach
    q_projected = q_end + rReach * v / max(nv,1e-12);
end
q_projected = min(max(q_projected,[0,0]),[area,area]);
d = q_projected - q_prev;
nd = norm(d);
if nd > stepMax
    q_projected = q_prev + stepMax * d / max(nd,1e-12);
end
q_projected = min(max(q_projected,[0,0]),[area,area]);
end
function Q = project_full_trajectory(Q,area,stepMax,q_start,q_end,projIter)
N = size(Q,1);
Q(:,1) = min(max(Q(:,1),0),area);
Q(:,2) = min(max(Q(:,2),0),area);
Q(1,:) = q_start;
Q(N,:) = q_end;
for it = 1:projIter
    Q(1,:) = q_start;
    for n = 2:N
        d = Q(n,:) - Q(n-1,:);
        nd = norm(d);
        if nd > stepMax
            Q(n,:) = Q(n-1,:) + stepMax * d / max(nd,1e-12);
        end
        Q(n,1) = min(max(Q(n,1),0),area);
        Q(n,2) = min(max(Q(n,2),0),area);
    end
    Q(N,:) = q_end;
    for n = N-1:-1:1
        d = Q(n,:) - Q(n+1,:);
        nd = norm(d);
        if nd > stepMax
            Q(n,:) = Q(n+1,:) + stepMax * d / max(nd,1e-12);
        end
        Q(n,1) = min(max(Q(n,1),0),area);
        Q(n,2) = min(max(Q(n,2),0),area);
    end
    Q(1,:) = q_start;
    Q(N,:) = q_end;
end
end
%% ============================================================
%                  A2G GAIN
%% ============================================================
function g = a2g_gain_alhourani(q,w,H,f,c,A2G,useShadowA2G,z)
d2 = norm(q - w);
d3 = sqrt(d2^2 + H^2);
theta = atan2d(H,max(d2,1e-9));
P_LoS = 1 / (1 + A2G.a * exp(-A2G.b * (theta - A2G.a)));
P_NLoS = 1 - P_LoS;
FSPL_dB = 20*log10(4*pi*f*d3/c);
if useShadowA2G
    sig_dB = sqrt(P_LoS*A2G.sigmaLoS_dB^2 + P_NLoS*A2G.sigmaNLoS_dB^2);
    sh_dB  = sig_dB * z;
else
    sh_dB = 0;
end
eta_dB = P_LoS*A2G.etaLoS_dB + P_NLoS*A2G.etaNLoS_dB;
PL_dB = FSPL_dB + eta_dB + sh_dB;
g = 10^(-PL_dB/10);
end
function g = a2g_gain_alhourani_mean(q,w,H,f,c,A2G)
d2 = norm(q - w);
d3 = sqrt(d2^2 + H^2);
theta = atan2d(H,max(d2,1e-9));
P_LoS = 1 / (1 + A2G.a * exp(-A2G.b * (theta - A2G.a)));
P_NLoS = 1 - P_LoS;
FSPL_dB = 20*log10(4*pi*f*d3/c);
eta_dB = P_LoS*A2G.etaLoS_dB + P_NLoS*A2G.etaNLoS_dB;
PL_dB = FSPL_dB + eta_dB;
g = 10^(-PL_dB/10);
end
function [v,gSig,gLeak] = mm_unit_beam_unprotected(hSU,denom,MM_iter,alphaFallback)
global AIIBPC_BEAM_CALLS;
if ~isempty(AIIBPC_BEAM_CALLS), AIIBPC_BEAM_CALLS=AIIBPC_BEAM_CALLS+1; end
% Initial CBF-MM surrogate WITHOUT PU/WU interference penalties.
% The rank-one desired-signal surrogate converges to MRT.
Nt=numel(hSU); B=(hSU*hSU')/max(denom,1e-30)+alphaFallback*eye(Nt);
v=ones(Nt,1)/sqrt(Nt);
for it=1:max(1,MM_iter)
    w=B*v; vNew=w/max(norm(w),1e-30);
    if norm(vNew-v)<1e-8, v=vNew; break; end
    v=vNew;
end
gSig=abs(hSU'*v)^2;
gLeak=0; % Protection-channel leakage is measured only after allocation.
end


function [tx,transferLog] = optionB_same_uav_rate_reallocation( ...
    txRaw,tx,HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m, ...
    sigma2L,sigma2U,BscL,BscU,GammaL,GammaU,PmaxL,PmaxU,verbose)
if nargin<19,verbose=true;end
% Rate-aware Option B: redistribute existing power on the same UAV,
% never transfer watts across bands. Only clipped streams qualify as
% recipients. For each, test donors on the opposite band of the SAME UAV.
% Each candidate is checked against the actual network-wide sum rate and
% aggregate PU/WU interference. Accept strictly positive gains only.
transferLog = zeros(0,6);
for u = 1:numel(tx)
    for srcBandNum = 1:2
        if srcBandNum == 1
            sourceBand = 'lic'; destBand = 'un'; Hdest = HwuU; Gdest = GammaU;
        else
            sourceBand = 'un'; destBand = 'lic'; Hdest = HpuL; Gdest = GammaL;
        end
        [srcIdx,srcNow] = get_band_fields(tx,u,sourceBand);
        [srcIdxRaw,srcRaw] = get_band_fields(txRaw,u,sourceBand);
        [dstIdx,dstP] = get_band_fields(tx,u,destBand);
        if isempty(srcIdx) || isempty(dstIdx), continue; end
        for r = 1:size(srcIdx,1)
            originalRow = find(srcIdxRaw(:,1)==srcIdx(r,1) & ...
                               srcIdxRaw(:,2)==srcIdx(r,2),1);
            if isempty(originalRow) || srcRaw(originalRow)-srcNow(r)<1e-9
                continue;
            end
            recipient = find(dstIdx(:,1)==srcIdx(r,1),1);
            if isempty(recipient), continue; end
            % Donor stream must belong to another SU of this UAV.
            for donor = 1:numel(dstP)
                if donor==recipient || dstP(donor)<1e-7, continue; end
                baseline = optionB_total_rate(tx,HsuL,HsuU,hp2,hw2, ...
                    P_PBS_j,P_WIFI_m,sigma2L,sigma2U,BscL,BscU);
                bestGain = 0; bestTx = tx; bestDelta = 0;
                % Conservative transfer fractions; preserve the full budget.
                for frac = [0.05 0.10 0.20 0.35 0.50 0.75 1.00]
                    delta = dstP(donor)*frac;
                    trial = tx;
                    pTry = dstP;
                    pTry(donor) = pTry(donor)-delta;
                    pTry(recipient) = pTry(recipient)+delta;
                    trial = set_band_power(trial,u,destBand,pTry);
                    Itest = aggregate_interference_by_channel(trial,Hdest,Gdest,destBand);
                    if any(Itest(:)>Gdest(:).*(1+1e-8)),continue;end
                    % Source band remains untouched and already protected.
                    rateTry = optionB_total_rate(trial,HsuL,HsuU,hp2,hw2, ...
                        P_PBS_j,P_WIFI_m,sigma2L,sigma2U,BscL,BscU);
                    if isfinite(rateTry) && rateTry-baseline>bestGain+1e-7
                        bestGain=rateTry-baseline;
                        bestTx=trial;
                        bestDelta=delta;
                    end
                end
                if bestGain>0
                    recipientSU=tx(u).users(srcIdx(r,1));
                    donorSU=tx(u).users(dstIdx(donor,1));
                    tx=bestTx;
                    transferLog(end+1,:)=[u recipientSU donorSU 3-srcBandNum bestDelta bestGain/1e6]; %#ok<AGROW>
                    if verbose, fprintf('OPTION B: UAV %d, SU %d receives %.6g W from SU %d on %s; network gain %.6g Mbps\n', ...
                        u,recipientSU,bestDelta,donorSU,destBand,bestGain/1e6); end
                    [~,dstP] = get_band_fields(tx,u,destBand);
                end
            end
        end
    end
end
% Independent per-UAV power budgets and aggregate interference verification.
for u=1:numel(tx)
    assert(sum(tx(u).pLic(:))<=PmaxL*(1+1e-7));
    assert(sum(tx(u).pUn(:))<=PmaxU*(1+1e-7));
end
assert(all(aggregate_interference_by_channel(tx,HpuL,GammaL,'lic')<=GammaL(:).*(1+1e-7)));
assert(all(aggregate_interference_by_channel(tx,HwuU,GammaU,'un')<=GammaU(:).*(1+1e-7)));
end

function total = optionB_total_rate(tx,HsuL,HsuU,hp2,hw2,P_PBS_j,P_WIFI_m, ...
    sigma2L,sigma2U,BscL,BscU)
total=compute_global_sum_rate_band_interuav(tx,HsuL,hp2,P_PBS_j,sigma2L,BscL,'lic')+ ...
      compute_global_sum_rate_band_interuav(tx,HsuU,hw2,P_WIFI_m,sigma2U,BscU,'un');
end

function r=diagnostic_tx_rate(tx,HL,HU,hp2,hw2,Ppbs,Pwifi,nL,nU,bL,bU)
global AIIBPC_RATE_CALLS;
if ~isempty(AIIBPC_RATE_CALLS), AIIBPC_RATE_CALLS=AIIBPC_RATE_CALLS+1; end
r=compute_global_sum_rate_band_interuav(tx,HL,hp2,Ppbs,nL,bL,'lic')+ ...
  compute_global_sum_rate_band_interuav(tx,HU,hw2,Pwifi,nU,bU,'un');
end
function name=diagnostic_mode_name(mode)
if mode==0,name='safety-only';else,name='four-case';end
end

function [tx,stats]=ai_ibpc_integrated_four_case_rateaware(raw,HP,HW,GL,GU,PL,PU,rateFn)
% Integrated four-case AI-IBPC controller.
% ------------------------------------------------------------
% INPUT raw is the solution AFTER association, subchannel allocation,
% CBF-MM beamforming and interference-unaware KKT power allocation.
%
% The four cases are determined from RAW aggregate interference:
%   Case 1: licensed high, unlicensed safe.
%   Case 2: unlicensed high, licensed safe.
%   Case 3: neither high.
%   Case 4: both high.
%
% Every accepted action must satisfy the corresponding PU/WU protection
% threshold and the independent per-UAV power budget. The objective used
% to select among tested feasible power actions is the TRUE network sum rate
% supplied by rateFn. This is a rate-aware coordinate/candidate search; it is
% not a proof of the global non-convex optimum.
%
% IMPORTANT: protection is integrated here. The caller must NOT apply a
% separate safety-clipping/protection block after this function.

raw=enforce_per_uav_band_budget(raw,PL,'lic');
raw=enforce_per_uav_band_budget(raw,PU,'un');
IL0=aggregate_interference_by_channel(raw,HP,GL,'lic');
IU0=aggregate_interference_by_channel(raw,HW,GU,'un');
highL=any(IL0(:)>GL(:).*(1+1e-9));
highU=any(IU0(:)>GU(:).*(1+1e-9));

stats=struct('caseId',0,'caseName','','rawRate',rateFn(raw),...
    'finalRate',NaN,'licensedHigh',highL,'unlicensedHigh',highU,...
    'licensedReduced_W',0,'unlicensedReduced_W',0,...
    'licensedIncreased_W',0,'unlicensedIncreased_W',0);

pL0=arrayfun(@(z)sum(z.pLic(:)),raw);
pU0=arrayfun(@(z)sum(z.pUn(:)),raw);

if highL && ~highU
    % CASE 1: protect licensed side by rate-aware reduction. Then use only
    % unused UNLICENSED-band budget/headroom when that increases sum rate.
    stats.caseId=1; stats.caseName='licensed-high-only';
    tx=rate_aware_band_projection_ratefn(raw,HP,GL,PL,'lic',rateFn);
    tx=rate_aware_increase_band_only(tx,HW,GU,PU,'un',rateFn);

elseif ~highL && highU
    % CASE 2: mirror of Case 1.
    stats.caseId=2; stats.caseName='unlicensed-high-only';
    tx=rate_aware_band_projection_ratefn(raw,HW,GU,PU,'un',rateFn);
    tx=rate_aware_increase_band_only(tx,HP,GL,PL,'lic',rateFn);

elseif ~highL && ~highU
    % CASE 3: already protected. Keep the KKT power solution unchanged.
    stats.caseId=3; stats.caseName='neither-high';
    tx=raw;

else
    % CASE 4: both bands violate protection. Reduce BOTH bands using the
    % same global sum-rate objective. Alternate the order and retain the
    % better feasible protected result so the result is not tied to one
    % arbitrary licensed-first/unlicensed-first ordering.
    stats.caseId=4; stats.caseName='both-high';

    candLU=rate_aware_band_projection_ratefn(raw,HP,GL,PL,'lic',rateFn);
    candLU=rate_aware_band_projection_ratefn(candLU,HW,GU,PU,'un',rateFn);

    candUL=rate_aware_band_projection_ratefn(raw,HW,GU,PU,'un',rateFn);
    candUL=rate_aware_band_projection_ratefn(candUL,HP,GL,PL,'lic',rateFn);

    if ai_ibpc_is_feasible(candLU,HP,HW,GL,GU,PL,PU) && ...
            (~ai_ibpc_is_feasible(candUL,HP,HW,GL,GU,PL,PU) || ...
             rateFn(candLU)>=rateFn(candUL))
        tx=candLU;
    else
        tx=candUL;
    end
end

% This is verification, NOT another protection operation.
assert(ai_ibpc_is_feasible(tx,HP,HW,GL,GU,PL,PU),...
    'Integrated AI-IBPC failed to produce a protected feasible solution.');

pLf=arrayfun(@(z)sum(z.pLic(:)),tx);
pUf=arrayfun(@(z)sum(z.pUn(:)),tx);
stats.licensedReduced_W=sum(max(0,pL0-pLf));
stats.unlicensedReduced_W=sum(max(0,pU0-pUf));
stats.licensedIncreased_W=sum(max(0,pLf-pL0));
stats.unlicensedIncreased_W=sum(max(0,pUf-pU0));
stats.finalRate=rateFn(tx);
end

function tx=rate_aware_band_projection_ratefn(tx,HP,Gamma,Pmax,band,rateFn)
% Protect one band using rate-aware power REDUCTIONS only.
% For each unsafe protected subchannel, test proportional reduction and
% deterministic single-stream reduction orders. Keep the feasible candidate
% with the largest TRUE network sum rate.
tx=enforce_per_uav_band_budget(tx,Pmax,band);
for sc=1:numel(Gamma)
    I=aggregate_interference_by_channel(tx,HP,Gamma,band);
    if I(sc)<=Gamma(sc)*(1+1e-10),continue;end

    entries=zeros(0,4); % [u,ell,p,leakGain]
    for u=1:numel(tx)
        [idx,p,v]=get_band_fields(tx,u,band);
        for ell=1:size(idx,1)
            if idx(ell,2)==sc && p(ell)>0
                entries(end+1,:)=[u,ell,p(ell),...
                    abs(HP{u,sc}'*v{ell})^2]; %#ok<AGROW>
            end
        end
    end
    if isempty(entries)
        error('Unsafe protected channel has no active stream to reduce.');
    end

    % Candidate A: proportional reduction of all streams on this SC.
    scale=min(1,Gamma(sc)/max(I(sc),realmin));
    best=tx;
    for z=1:size(entries,1)
        u=entries(z,1); ell=entries(z,2);
        [~,p,~]=get_band_fields(best,u,band);
        p(ell)=p(ell)*scale;
        best=set_band_power(best,u,band,p);
    end
    bestRate=rateFn(best);

    % Candidate B: different deterministic stream-reduction orders.
    for first=1:size(entries,1)
        cand=tx;
        order=[first,setdiff(1:size(entries,1),first,'stable')];
        for oi=order
            Icur=aggregate_interference_by_channel(cand,HP,Gamma,band);
            if Icur(sc)<=Gamma(sc)*(1+1e-10),break;end
            u=entries(oi,1); ell=entries(oi,2);
            [~,p,~]=get_band_fields(cand,u,band);
            gain=entries(oi,4);
            dp=min(p(ell),(Icur(sc)-Gamma(sc))/max(gain,realmin));
            p(ell)=max(0,p(ell)-dp);
            cand=set_band_power(cand,u,band,p);
        end
        Ic=aggregate_interference_by_channel(cand,HP,Gamma,band);
        if Ic(sc)>Gamma(sc)*(1+1e-7),continue;end
        r=rateFn(cand);
        if r>bestRate+1e-9
            best=cand;
            bestRate=r;
        end
    end
    tx=best;
end

% Verification only. The search above must itself have protected the band.
I=aggregate_interference_by_channel(tx,HP,Gamma,band);
assert(all(I(:)<=Gamma(:).*(1+1e-7)),...
    'AI-IBPC rate-aware reduction did not satisfy aggregate protection.');
end

function tx=rate_aware_increase_band_only(tx,HP,Gamma,Pmax,band,rateFn)
% Increase powers ONLY on the safe opposite band and ONLY when the TRUE
% network sum rate improves. Independent per-band budgets are preserved;
% no watts are transferred from the violating band.
for pass=1:2
    improved=false;
    for u=1:numel(tx)
        [idx,p,v]=get_band_fields(tx,u,band);
        for e=1:numel(p)
            sc=idx(e,2);
            I=aggregate_interference_by_channel(tx,HP,Gamma,band);
            leak=abs(HP{u,sc}'*v{e})^2;
            current=p(e);
            budgetLeft=max(0,Pmax-sum(p));
            headroom=max(0,Gamma(sc)-I(sc));
            if leak<=1e-30
                addMax=budgetLeft;
            else
                addMax=min(budgetLeft,headroom/leak);
            end
            if addMax<=1e-14,continue;end
            upper=current+addMax;
            baseRate=rateFn(tx);
            trialP=p;
            fun=@(power)ai_ibpc_trial_rate(tx,u,band,trialP,e,power,rateFn);
            opts=optimset('TolX',max(1e-11,upper*1e-5),'Display','off');
            pStar=fminbnd(@(power)-fun(power),current,upper,opts);
            tests=unique([current;upper;pStar]);
            scores=zeros(numel(tests),1);
            for tt=1:numel(tests)
                scores(tt)=fun(tests(tt));
            end
            [bestRate,kbest]=max(scores);
            if bestRate>baseRate+1e-9
                p(e)=tests(kbest);
                tx=set_band_power(tx,u,band,p);
                improved=true;
            end
        end
    end
    if ~improved,break;end
end

I=aggregate_interference_by_channel(tx,HP,Gamma,band);
assert(all(I(:)<=Gamma(:).*(1+1e-7)),...
    'AI-IBPC opposite-band increase violated aggregate protection.');
for u=1:numel(tx)
    [~,p,~]=get_band_fields(tx,u,band);
    assert(sum(p(:))<=Pmax*(1+1e-7),...
        'AI-IBPC opposite-band increase violated per-UAV power budget.');
end
end

function r=ai_ibpc_trial_rate(tx,u,band,p,e,power,rateFn)
p(e)=power;
tx=set_band_power(tx,u,band,p);
r=rateFn(tx);
end

function ok=ai_ibpc_is_feasible(tx,HP,HW,GL,GU,PL,PU)
IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
IU=aggregate_interference_by_channel(tx,HW,GU,'un');
ok=all(IL(:)<=GL(:).*(1+1e-7)) && ...
   all(IU(:)<=GU(:).*(1+1e-7));
for u=1:numel(tx)
    ok=ok && sum(tx(u).pLic(:))<=PL*(1+1e-7) && ...
             sum(tx(u).pUn(:))<=PU*(1+1e-7);
end
end

function [tx,stats]=rate_aware_protect_both(tx,HP,HW,GL,GU,PL,PU,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU)
% Same protection for baseline and proposed modes; fixed beamformers.
stats=struct();
tx=rate_aware_band_projection(tx,HP,GL,PL,'lic',HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
tx=rate_aware_band_projection(tx,HW,GU,PU,'un',HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
end
function tx=rate_aware_band_projection(tx,HP,Gamma,Pmax,band,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU)
% For each unsafe protected channel, try feasible single-stream reductions
% and proportional reduction, choose the candidate with highest TRUE rate.
% Each candidate protects the CURRENT channel; later channel processing
% does not increase power, so previous channels remain protected.
tx=enforce_per_uav_band_budget(tx,Pmax,band);
for sc=1:numel(Gamma)
    I=aggregate_interference_by_channel(tx,HP,Gamma,band);
    if I(sc)<=Gamma(sc)*(1+1e-10),continue;end
    entries=zeros(0,4); % [u,ell,p,leakGain]
    for u=1:numel(tx)
        [idx,p,v]=get_band_fields(tx,u,band);
        for ell=1:size(idx,1)
            if idx(ell,2)==sc && p(ell)>0
                entries(end+1,:)=[u,ell,p(ell),abs(HP{u,sc}'*v{ell})^2]; %#ok<AGROW>
            end
        end
    end
    if isempty(entries),continue;end
    scale=Gamma(sc)/max(I(sc),realmin);
    best=tx;
    for z=1:size(entries,1)
        u=entries(z,1);ell=entries(z,2);
        [~,p,~]=get_band_fields(best,u,band);
        p(ell)=p(ell)*scale;
        best=set_band_power(best,u,band,p);
    end
    bestRate=diagnostic_tx_rate(best,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
    % Greedy order search: each iteration removes enough interference from
    % one stream to meet threshold, or zeroes it if that is insufficient.
    for first=1:size(entries,1)
        cand=tx;
        order=[first,setdiff(1:size(entries,1),first,'stable')];
        for oi=order
            Icur=aggregate_interference_by_channel(cand,HP,Gamma,band);
            if Icur(sc)<=Gamma(sc)*(1+1e-10),break;end
            u=entries(oi,1);ell=entries(oi,2);
            [~,p,~]=get_band_fields(cand,u,band);
            gain=entries(oi,4);
            dp=min(p(ell),(Icur(sc)-Gamma(sc))/max(gain,realmin));
            p(ell)=max(0,p(ell)-dp);
            cand=set_band_power(cand,u,band,p);
        end
        Ic=aggregate_interference_by_channel(cand,HP,Gamma,band);
        if Ic(sc)>Gamma(sc)*(1+1e-7),continue;end
        r=diagnostic_tx_rate(cand,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
        if r>bestRate+1e-9
            best=cand;bestRate=r;
        end
    end
    tx=best;
end
% Final guard against numerical errors.
tx=project_aggregate_band_interference(tx,HP,Gamma,Pmax,band);
end
function tx=compensate_original_violators(tx,raw,HP,HW,GL,GU,PL,PU,unused)
% Four-case flags from PRE-protection raw aggregate interference. Only
% opposite-band increases from UNUSED SAME-BAND budget are allowed.
IL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IU=aggregate_interference_by_channel(raw,HW,GU,'un');
for u=1:numel(tx)
    for loc=1:numel(tx(u).users)
        [idxL,~,~]=get_band_fields(raw,u,'lic');
        [idxU,~,~]=get_band_fields(raw,u,'un');
        eL=find(idxL(:,1)==loc,1);
        eU=find(idxU(:,1)==loc,1);
        highL=~isempty(eL)&&IL(idxL(eL,2))>GL(idxL(eL,2))*(1+1e-9);
        highU=~isempty(eU)&&IU(idxU(eU,2))>GU(idxU(eU,2))*(1+1e-9);
        if highL && ~highU && ~isempty(eU)
            [tx,~]=increase_su_stream_if_safe(tx,u,eU,HW,GU,PU,'un');
        elseif highU && ~highL && ~isempty(eL)
            [tx,~]=increase_su_stream_if_safe(tx,u,eL,HP,GL,PL,'lic');
        end
    end
end
tx=apply_final_safety_clipping(tx,HP,HW,GL,GU,PL,PU,'both');
end

function [best,info]=fixed_order_alternating_power(raw,safe,HP,HW,GL,GU,PL,PU,rateFn,opts,maxSweeps,order)
% Fixed-order AO control: exactly the same optimizer and stopping rule.
% Both bands initially assigned. Cases are defined from RAW aggregate
% interference; they determine the first block and whether both blocks
% are activated. No power is transferred across bands or between SUs.
% Optimizes the TRUE coupled sum rate at each accepted block step.
caseCounts=zeros(1,4);
best=safe; bestRate=rateFn(best);accepted=0;history=bestRate/1e6;
solverCalls=0;totalIterations=0;totalEvaluations=0;
for sweep=1:maxSweeps
    previous=bestRate;
    for b=order
        [candidate,flag,solverStat]=case_optimize_one_band(raw,best,b,HP,HW,GL,GU,PL,PU,rateFn,opts);
        solverCalls=solverCalls+solverStat.calls;
        totalIterations=totalIterations+solverStat.iterations;
        totalEvaluations=totalEvaluations+solverStat.evaluations;
        r=rateFn(candidate);
        if flag>=0 && isfinite(r) && r>bestRate+1e-5
            best=candidate;bestRate=r;accepted=accepted+1;
        end
    end
    history(end+1)=bestRate/1e6; %#ok<AGROW>
    if bestRate-previous<1e-4,break;end
end
IL=aggregate_interference_by_channel(best,HP,GL,'lic');
IU=aggregate_interference_by_channel(best,HW,GU,'un');
assert(all(IL(:)<=GL(:)*(1+1e-5))&&all(IU(:)<=GU(:)*(1+1e-5)), ...
    'Case-conditioned optimizer violated aggregate interference');
for u=1:numel(best)
    assert(sum(best(u).pLic)<=PL*(1+1e-5));
    assert(sum(best(u).pUn)<=PU*(1+1e-5));
end
info=struct('nCase1',caseCounts(1),'nCase2',caseCounts(2), ...
    'nCase3',caseCounts(3),'nCase4',caseCounts(4), ...
    'acceptedSteps',accepted,'history_Mbps',history,'order',order, ...
    'solverCalls',solverCalls,'totalIterations',totalIterations, ...
    'totalEvaluations',totalEvaluations);
end

function plotOrderAblation(x,Y,names,ylabelText,filename)
figure('Color','w','Position',[100 100 620 470]);
styles={'s-','o--','^-','d-.'};
for i=1:size(Y,2)
    plot(x,Y(:,i),styles{i},'LineWidth',2,'MarkerSize',7);hold on;
end
ax=gca;set(ax,'FontName','Times New Roman','FontSize',16, ...
    'FontWeight','normal','LineWidth',1.0,'Position',[0.13 0.15 0.84 0.66]);
grid on;ax.GridLineStyle=':';ax.GridAlpha=0.25;
xlabel('Paired channel realization');ylabel(ylabelText);
legend(names,'FontName','Times New Roman','FontSize',11, ...
    'NumColumns',2,'Location','northoutside');
saveas(gcf,filename);
end

function [best,info]=case_conditioned_alternating_power(raw,safe,HP,HW,GL,GU,PL,PU,rateFn,opts,maxSweeps)
% A separate, feasible case-conditioned block-coordinate optimizer.
% Both bands initially assigned. Cases are defined from RAW aggregate
% interference; they determine the first block and whether both blocks
% are activated. No power is transferred across bands or between SUs.
% Optimizes the TRUE coupled sum rate at each accepted block step.
IL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IU=aggregate_interference_by_channel(raw,HW,GU,'un');
caseCounts=zeros(1,4);
for u=1:numel(raw)
    for loc=1:numel(raw(u).users)
        [iL,~,~]=get_band_fields(raw,u,'lic');
        [iU,~,~]=get_band_fields(raw,u,'un');
        eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
        highL=~isempty(eL)&&IL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
        highU=~isempty(eU)&&IU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
        if highL&&highU,caseCounts(3)=caseCounts(3)+1;
        elseif highL,caseCounts(1)=caseCounts(1)+1;
        elseif highU,caseCounts(2)=caseCounts(2)+1;
        else,caseCounts(4)=caseCounts(4)+1;end
    end
end
% Begin at safe powers. The order is determined by the active case flags.
% Even safe bands can be optimized because coupled SINR may benefit.
if caseCounts(2)>caseCounts(1),order=[2 1];else,order=[1 2];end
best=safe; bestRate=rateFn(best);accepted=0;history=bestRate/1e6;
solverCalls=0;totalIterations=0;totalEvaluations=0;
for sweep=1:maxSweeps
    previous=bestRate;
    for b=order
        [candidate,flag,solverStat]=case_optimize_one_band(raw,best,b,HP,HW,GL,GU,PL,PU,rateFn,opts);
        solverCalls=solverCalls+solverStat.calls;
        totalIterations=totalIterations+solverStat.iterations;
        totalEvaluations=totalEvaluations+solverStat.evaluations;
        r=rateFn(candidate);
        if flag>=0 && isfinite(r) && r>bestRate+1e-5
            best=candidate;bestRate=r;accepted=accepted+1;
        end
    end
    history(end+1)=bestRate/1e6; %#ok<AGROW>
    if bestRate-previous<1e-4,break;end
end
IL=aggregate_interference_by_channel(best,HP,GL,'lic');
IU=aggregate_interference_by_channel(best,HW,GU,'un');
assert(all(IL(:)<=GL(:)*(1+1e-5))&&all(IU(:)<=GU(:)*(1+1e-5)), ...
    'Case-conditioned optimizer violated aggregate interference');
for u=1:numel(best)
    assert(sum(best(u).pLic)<=PL*(1+1e-5));
    assert(sum(best(u).pUn)<=PU*(1+1e-5));
end
info=struct('nCase1',caseCounts(1),'nCase2',caseCounts(2), ...
    'nCase3',caseCounts(3),'nCase4',caseCounts(4), ...
    'acceptedSteps',accepted,'history_Mbps',history,'order',order, ...
    'solverCalls',solverCalls,'totalIterations',totalIterations, ...
    'totalEvaluations',totalEvaluations);
end

function [best,flag,solverStat]=case_optimize_one_band(raw,current,b,HP,HW,GL,GU,PL,PU,rateFn,opts)
% Optimize one band, fixing the other. Linear per-UAV and per-SC
% constraints; full nonlinear inter-user sum rate in objective.
map=zeros(0,2); x0=[];
for u=1:numel(current)
    if b==1,p=current(u).pLic;else,p=current(u).pUn;end
    for ell=1:numel(p)
        map(end+1,:)=[u,ell]; %#ok<AGROW>
        x0(end+1,1)=p(ell); %#ok<AGROW>
    end
end
best=current;flag=0;solverStat=struct('calls',0,'iterations',0,'evaluations',0);
if isempty(map),return;end
if b==1, Gamma=GL; HPband=HP; Pmax=PL;band='lic';
else,Gamma=GU;HPband=HW;Pmax=PU;band='un';end
A=zeros(numel(current)+numel(Gamma),numel(x0));
rhs=[Pmax*ones(numel(current),1);Gamma(:)];
for z=1:size(map,1)
    u=map(z,1);ell=map(z,2);
    if b==1,sc=raw(u).idxLic(ell,2);v=raw(u).vLic{ell};
    else,sc=raw(u).idxUn(ell,2);v=raw(u).vUn{ell};end
    A(u,z)=1;
    A(numel(current)+sc,z)=abs(HPband{u,sc}'*v)^2;
end
for row=numel(current)+1:size(A,1)
    sca=max(rhs(row),realmin);
    A(row,:)=A(row,:)/sca;rhs(row)=rhs(row)/sca;
end
makeTx=@(p) case_set_band_vector(current,map,p,b);
objective=@(p) -rateFn(makeTx(p))/1e6;
try
    [p,~,ef,solverOutput]=fmincon(objective,x0,A,rhs,[],[],zeros(size(x0)), ...
        Pmax*ones(size(x0)),[],opts);
    solverStat.calls=1;
    solverStat.iterations=solverOutput.iterations;
    solverStat.evaluations=solverOutput.funcCount;
    if all(A*p<=rhs+1e-7)&&all(p>=-1e-9)
        cand=makeTx(p);
        if rateFn(cand)>rateFn(current)+1e-5
            best=cand;flag=ef;
        end
    end
catch ME
    warning('Case-conditioned band %s failed: %s',band,ME.message);
end
end

function tx=case_set_band_vector(tx,map,p,b)
for z=1:numel(p)
    u=map(z,1);ell=map(z,2);
    if b==1,tx(u).pLic(ell)=p(z);
    else,tx(u).pUn(ell)=p(z);end
end
end

function [best,info]=joint_power_fixed_beams(raw,safe,HP,HW,GL,GU,PL,PU,rateFn,opts)
% Joint licensed+unlicensed power optimization for FIXED assignments/beams.
% fmincon handles true global SINR objective; all constraints are linear.
% SQP is local, not a guarantee of global optimality.
map=zeros(0,3); p0=[]; ps=[];
for u=1:numel(raw)
    for b=1:2
        if b==1, p=raw(u).pLic; q=safe(u).pLic;
        else, p=raw(u).pUn; q=safe(u).pUn; end
        for ell=1:numel(p)
            map(end+1,:)=[u,b,ell]; %#ok<AGROW>
            p0(end+1,1)=p(ell); %#ok<AGROW>
            ps(end+1,1)=q(ell); %#ok<AGROW>
        end
    end
end
if isempty(map),best=safe;info=struct('exitflag',0,'maxViolation',0, ...
    'solverCalls',0,'totalIterations',0,'totalEvaluations',0);return;end
rows=2*numel(raw)+numel(GL)+numel(GU);
A=zeros(rows,numel(p0)); rhs=[repmat([PL;PU],numel(raw),1);GL(:);GU(:)];
for z=1:size(map,1)
    u=map(z,1);b=map(z,2);ell=map(z,3);
    A(2*(u-1)+b,z)=1;
    if b==1
        sc=raw(u).idxLic(ell,2);v=raw(u).vLic{ell};
        A(2*numel(raw)+sc,z)=abs(HP{u,sc}'*v)^2;
    else
        sc=raw(u).idxUn(ell,2);v=raw(u).vUn{ell};
        A(2*numel(raw)+numel(GL)+sc,z)=abs(HW{u,sc}'*v)^2;
    end
end
% Normalize tiny interference constraints to improve SQP conditioning.
for row=2*numel(raw)+1:rows
    sca=max(rhs(row),realmin);A(row,:)=A(row,:)/sca;rhs(row)=rhs(row)/sca;
end
% Safety allocation is a guaranteed feasible warm start.
objective=@(p) -rateFn(joint_tx_from_vector(raw,map,p))/1e6;
starts=[ps,zeros(size(ps))];
% Use raw as additional start ONLY if already feasible.
if all(A*p0<=rhs+1e-8),starts=[starts,p0];end
best=safe; bestRate=rateFn(best);bestFlag=0;
solverCalls=0;totalIterations=0;totalEvaluations=0;
for k=1:size(starts,2)
    x=starts(:,k);
    try
        [p,~,ef,solverOutput]=fmincon(objective,x,A,rhs,[],[], ...
            zeros(size(p0)),repmat(max(PL,PU),size(p0)),[],opts);
        solverCalls=solverCalls+1;
        totalIterations=totalIterations+solverOutput.iterations;
        totalEvaluations=totalEvaluations+solverOutput.funcCount;
        viol=max([A*p-rhs;-p]);
        if viol<=1e-6
            cand=joint_tx_from_vector(raw,map,p); r=rateFn(cand);
            if isfinite(r)&&r>bestRate+1e-5
                best=cand;bestRate=r;bestFlag=ef;
            end
        end
    catch ME
        warning('Joint power start %d failed: %s',k,ME.message);
    end
end
% Final exact power/leakage check in physical units.
IL=aggregate_interference_by_channel(best,HP,GL,'lic');
IU=aggregate_interference_by_channel(best,HW,GU,'un');
assert(all(IL(:)<=GL(:)*(1+1e-5))&&all(IU(:)<=GU(:)*(1+1e-5)));
for u=1:numel(best)
    assert(sum(best(u).pLic)<=PL*(1+1e-5));
    assert(sum(best(u).pUn)<=PU*(1+1e-5));
end
info=struct('exitflag',bestFlag,'maxViolation', ...
    max([max(IL(:)./GL(:))-1,max(IU(:)./GU(:))-1,0]), ...
    'solverCalls',solverCalls,'totalIterations',totalIterations, ...
    'totalEvaluations',totalEvaluations);
end
function tx=joint_tx_from_vector(raw,map,p)
tx=raw;
for z=1:numel(p)
    u=map(z,1);ell=map(z,3);
    if map(z,2)==1,tx(u).pLic(ell)=p(z);
    else,tx(u).pUn(ell)=p(z);end
end
end

function [best,info]=enhance_case_ao_power(raw,safe,original,originalInfo,HP,HW,GL,GU,PL,PU,rateFn,opts)
% Feasible, rate-monotone refinement of original four-case alternating AO.
% 1. Cheap all-UAV band shutdown candidates (both licensed and unlicensed).
% 2. For promising shutdowns, rerun ORIGINAL case-conditioned AO from the
%    feasible shutdown state, and retain only a strictly better solution.
% 3. If original AO was below a simple leakage-aware start, refine that start.
% No cross-band transfer, no change in association/scheduling/beamforming.
best=original;bestRate=rateFn(best);
info=struct('solverCalls',0,'totalIterations',0,'totalEvaluations',0, ...
    'acceptedShutdowns',0,'testedShutdowns',0,'acceptedRestarts',0, ...
    'history_Mbps',bestRate/1e6);
% Test every active UAV-band shutdown; one quick objective evaluation each.
% Restart only from candidates that are competitive before reoptimization.
for u=1:numel(best)
    for b=1:2
        trial=best;
        if b==1
            if ~any(trial(u).pLic>1e-12),continue;end
            trial(u).pLic(:)=0;
        else
            if ~any(trial(u).pUn>1e-12),continue;end
            trial(u).pUn(:)=0;
        end
        info.testedShutdowns=info.testedShutdowns+1;
        r0=rateFn(trial);
        % A shutdown is always feasible when the original solution is.
        if r0>bestRate+1e-5
            best=trial;bestRate=r0;info.acceptedShutdowns=info.acceptedShutdowns+1;
            info.history_Mbps(end+1)=bestRate/1e6;
        end
        % Refinement from a shutdown can uncover a different AO basin.
        % Use a bounded number of iterations to preserve the time advantage.
        if r0>=bestRate-0.25e6
            [candidate,ci]=case_conditioned_alternating_power( ...
                raw,trial,HP,HW,GL,GU,PL,PU,rateFn,opts,3);
            info.solverCalls=info.solverCalls+ci.solverCalls;
            info.totalIterations=info.totalIterations+ci.totalIterations;
            info.totalEvaluations=info.totalEvaluations+ci.totalEvaluations;
            rc=rateFn(candidate);
            if rc>bestRate+1e-5
                best=candidate;bestRate=rc;info.acceptedRestarts=info.acceptedRestarts+1;
                info.history_Mbps(end+1)=bestRate/1e6;
            end
        end
    end
end
% A low-cost leakage-aware feasible multistart, constructed by reducing
% powers on the highest leakage-weighted streams before safety projection.
% Only use this extra start when original AO had a limited initial gain.
if rateFn(original)-rateFn(safe)<0.25e6
    trial=safe;
    for u=1:numel(trial)
        for b=1:2
            if b==1,idx=trial(u).idxLic; pp=trial(u).pLic; HH=HP; gg=GL;
            else,idx=trial(u).idxUn; pp=trial(u).pUn; HH=HW; gg=GU;end
            for e=1:numel(pp)
                sc=idx(e,2);
                if b==1,v=trial(u).vLic{e};else,v=trial(u).vUn{e};end
                leakage=pp(e)*abs(HH{u,sc}'*v)^2/max(gg(sc),realmin);
                if leakage>0.15,pp(e)=0.5*pp(e);end
            end
            if b==1,trial(u).pLic=pp;else,trial(u).pUn=pp;end
        end
    end
    [candidate,ci]=case_conditioned_alternating_power( ...
        raw,trial,HP,HW,GL,GU,PL,PU,rateFn,opts,3);
    info.solverCalls=info.solverCalls+ci.solverCalls;
    info.totalIterations=info.totalIterations+ci.totalIterations;
    info.totalEvaluations=info.totalEvaluations+ci.totalEvaluations;
    if rateFn(candidate)>bestRate+1e-5
        best=candidate;bestRate=rateFn(candidate);
        info.acceptedRestarts=info.acceptedRestarts+1;
        info.history_Mbps(end+1)=bestRate/1e6;
    end
end
IL=aggregate_interference_by_channel(best,HP,GL,'lic');
IU=aggregate_interference_by_channel(best,HW,GU,'un');
assert(all(IL(:)<=GL(:)*(1+1e-5))&&all(IU(:)<=GU(:)*(1+1e-5)), ...
    'Enhanced AI-IBPC violates aggregate protection');
for u=1:numel(best)
    assert(sum(best(u).pLic)<=PL*(1+1e-5) && ...
        sum(best(u).pUn)<=PU*(1+1e-5), ...
        'Enhanced AI-IBPC violates an independent per-band budget');
end
assert(bestRate>=rateFn(original)-1e-5,'Enhanced result reduced original rate');
info.originalIterations=originalInfo.totalIterations;
end


function [best,info]=refine_association_full_rebuild( ...
    incumbent,assoc0,maxCandidates,Qslot,WS,WP,WW,w_b,w_wifi,H, ...
    f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow, ...
    P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn, ...
    Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un, ...
    seedNow,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter, ...
    MM_leakFrac,MM_reg_alpha,methodMode,HP,HW,HL,HU,hp,hw,rateFn,opts)
% One-user relocation. Full rebuilding ensures the moved user is not
% evaluated with stale Hungarian subchannels or stale beamforming vectors.
% Candidate channel realizations, geometry and random seed are held fixed.
% This is a discrete local search; it does not guarantee global optimality.
U=size(Qslot,1); K=numel(assoc0);
info=struct('tested',0,'accepted',0,'moves',zeros(0,3), ...
    'history_Mbps',rateFn(incumbent)/1e6);
best=incumbent;bestRate=rateFn(best);
% Use one-move neighborhood around the ORIGINAL association, so results
% remain comparable and no multi-move geometry-induced effects are hidden.
for k=1:K
    for newU=1:U
        if newU==assoc0(k),continue;end
        if info.tested>=maxCandidates,return;end
        assocTry=assoc0;assocTry(k)=newU;
        % The helper runs one full resource-allocation sweep with a forced
        % association. The stage diagnostic returns its pre-control RAW tx.
        [~,~,~,~,~,dd]=eval_slot_multi_qos_bf( ...
            Qslot,WS,WP,WW,w_b,w_wifi,H, ...
            f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow, ...
            P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn, ...
            Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,1, ...
            seedNow,useRayleigh,useShadow,useShadowA2G,Nt,MM_iter, ...
            MM_leakFrac,MM_reg_alpha,methodMode,1,true,assocTry);
        info.tested=info.tested+1;
        if ~isfield(dd,'rawTx') || isempty(dd.rawTx)
            error('Candidate raw allocation unavailable for reassociation');
        end
        candidateRaw=dd.rawTx;
        [candidateSafe,~]=rate_aware_protect_both( ...
            candidateRaw,HP,HW,GammaLic,GammaUn,Pmax_lic,Pmax_un, ...
            HL,HU,hp,hw,P_PBS_j,P_WIFI_m, ...
            sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);
        [candidateCase,ci]=case_conditioned_alternating_power( ...
            candidateRaw,candidateSafe,HP,HW,GammaLic,GammaUn, ...
            Pmax_lic,Pmax_un,rateFn,opts,5);
        % Only run the more expensive shutdown/multistart stage if this
        % candidate is within 0.25 Mbps of the current best.
        if rateFn(candidateCase)>=bestRate-0.25e6
            [candidate,~]=enhance_case_ao_power( ...
                candidateRaw,candidateSafe,candidateCase,ci, ...
                HP,HW,GammaLic,GammaUn,Pmax_lic,Pmax_un,rateFn,opts);
        else
            candidate=candidateCase;
        end
        IL=aggregate_interference_by_channel(candidate,HP,GammaLic,'lic');
        IU=aggregate_interference_by_channel(candidate,HW,GammaUn,'un');
        feasible=all(IL(:)<=GammaLic(:)*(1+1e-5)) && ...
            all(IU(:)<=GammaUn(:)*(1+1e-5));
        for uu=1:U
            feasible=feasible && ...
                sum(candidate(uu).pLic(:))<=Pmax_lic*(1+1e-5) && ...
                sum(candidate(uu).pUn(:))<=Pmax_un*(1+1e-5);
        end
        r=rateFn(candidate);
        if feasible && isfinite(r) && r>bestRate+1e-5
            best=candidate;bestRate=r;
            info.accepted=info.accepted+1;
            info.moves(end+1,:)=[k,assoc0(k),newU];
            info.history_Mbps(end+1)=bestRate/1e6;
        end
    end
end
end


function [tx,st]=pure_refined_fourcase(raw,HP,HW,GL,GU,PL,PU,rateFn,allowComp)
if nargin<9,allowComp=true;end
% Case-specific feasible power reduction. Only affected SU can gain power
% on the OPPOSITE band, and only when that band's budget/headroom permits.
% Case 4 is left unchanged unless another SU's necessary reductions affect it.
% All reductions are selected by evaluating the ACTUAL coupled network rate.
tx=enforce_per_uav_band_budget(raw,PL,'lic');
tx=enforce_per_uav_band_budget(tx,PU,'un');
IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
IU=aggregate_interference_by_channel(tx,HW,GU,'un');
st=struct('cases',zeros(1,4),'compL',0,'compU',0, ...
    'blockedBudget',0,'tested',0,'accepted',0);
% Freeze the original case membership before any control changes.
for u=1:numel(tx)
    for loc=1:numel(tx(u).users)
        [iL,~,~]=get_band_fields(tx,u,'lic');
        [iU,~,~]=get_band_fields(tx,u,'un');
        eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
        highL=~isempty(eL)&&IL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
        highU=~isempty(eU)&&IU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
        if highL&&highU,st.cases(3)=st.cases(3)+1;
        elseif highL,st.cases(1)=st.cases(1)+1;
        elseif highU,st.cases(2)=st.cases(2)+1;
        else,st.cases(4)=st.cases(4)+1;end
    end
end
% Fix overloaded channels. Each trial removes ONLY necessary power;
% enumerate which stream bears reduction, then try same-SU compensation.
% A full aggregate projection is used as the initial feasible fallback.
% To avoid comparing a feasible result to infeasible intermediate points,
% construct complete candidates from RAW and rank their final rates.
% Coordinate descent over the order of violating streams (fixed association).
for pass=1:3
    changed=false;
    for b=1:2
        if b==1,band='lic';other='un';HPb=HP;G=GL;Po=PU;Hother=HW;Go=GU;
        else,band='un';other='lic';HPb=HW;G=GU;Po=PL;Hother=HP;Go=GL;end
        for sc=1:numel(G)
            I=aggregate_interference_by_channel(tx,HPb,G,band);
            if I(sc)<=G(sc)*(1+1e-9),continue;end
            % Include proportional clipping as feasible fallback for this SC.
            fallback=tx;
            scale=G(sc)/max(I(sc),realmin);
            for u=1:numel(tx)
                [idx,p,~]=get_band_fields(fallback,u,band);
                hit=find(idx(:,2)==sc);
                if ~isempty(hit)
                    p(hit)=p(hit)*scale;
                    fallback=set_band_power(fallback,u,band,p);
                end
            end
            best=fallback;bestRate=rateFn(best);
            % Test each offending stream as first reduction target.
            for firstU=1:numel(tx)
                [idx,p,~]=get_band_fields(tx,firstU,band);
                for firstE=find(idx(:,2)==sc).'
                    trial=tx;
                    [trial,dp]=reduce_su_stream_to_aggregate_limit( ...
                        trial,firstU,firstE,HPb,G,band);
                    if dp<=0,continue;end
                    % Complete any residual protection on this SC only.
                    It=aggregate_interference_by_channel(trial,HPb,G,band);
                    if It(sc)>G(sc)*(1+1e-9)
                        remScale=G(sc)/max(It(sc),realmin);
                        for uu=1:numel(trial)
                            [ii,pp,~]=get_band_fields(trial,uu,band);
                            ee=find(ii(:,2)==sc);
                            if ~isempty(ee)
                                pp(ee)=pp(ee)*remScale;
                                trial=set_band_power(trial,uu,band,pp);
                            end
                        end
                    end
                    % Compensation is permitted only for this affected SU
                    % and only when it was initially in case 1 or case 2.
                    loc=idx(firstE,1);
                    [ii,pp,~]=get_band_fields(trial,firstU,other);
                    oppositeE=find(ii(:,1)==loc,1);
                    IrawL=aggregate_interference_by_channel(raw,HP,GL,'lic');
                    IrawU=aggregate_interference_by_channel(raw,HW,GU,'un');
                    [rawL,~,~]=get_band_fields(raw,firstU,'lic');
                    [rawU,~,~]=get_band_fields(raw,firstU,'un');
                    el=find(rawL(:,1)==loc,1);eu=find(rawU(:,1)==loc,1);
                    hL=~isempty(el)&&IrawL(rawL(el,2))>GL(rawL(el,2))*(1+1e-9);
                    hU=~isempty(eu)&&IrawU(rawU(eu,2))>GU(rawU(eu,2))*(1+1e-9);
                    canComp=(b==1 && hL && ~hU)||(b==2 && hU && ~hL);
                    if allowComp && canComp && ~isempty(oppositeE)
                        [trial,added]=pure_marginal_compensate(trial,firstU, ...
                            oppositeE,Hother,Go,Po,other,rateFn);
                        if added==0,st.blockedBudget=st.blockedBudget+1;end
                    end
                    st.tested=st.tested+1;
                    trialRate=rateFn(trial);
                    if trialRate>bestRate+1e-6
                        best=trial;bestRate=trialRate;
                    end
                end
            end
            % Every accepted change eliminates violation on this channel.
            tx=best;changed=true;st.accepted=st.accepted+1;
        end
    end
    if ~changed,break;end
end
% Optional compensation AFTER protection for original case-1/2 SUs:
% evaluate actual rate and reject any harmful compensation.
IrawL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IrawU=aggregate_interference_by_channel(raw,HW,GU,'un');
for u=1:numel(tx)
    for loc=1:numel(tx(u).users)
        [iL,~,~]=get_band_fields(raw,u,'lic');
        [iU,~,~]=get_band_fields(raw,u,'un');
        eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
        hL=~isempty(eL)&&IrawL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
        hU=~isempty(eU)&&IrawU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
        if allowComp && hL && ~hU && ~isempty(eU)
            [t,dp]=pure_marginal_compensate(tx,u,eU,HW,GU,PU,'un',rateFn);
            if rateFn(t)>rateFn(tx)+1e-6,tx=t;st.compU=st.compU+dp;end
        elseif allowComp && hU && ~hL && ~isempty(eL)
            [t,dp]=pure_marginal_compensate(tx,u,eL,HP,GL,PL,'lic',rateFn);
            if rateFn(t)>rateFn(tx)+1e-6,tx=t;st.compL=st.compL+dp;end
        end
    end
end
tx=apply_final_safety_clipping(tx,HP,HW,GL,GU,PL,PU,'both');
end

function [tx,st]=pure_post_compensate(base,raw,HP,HW,GL,GU,PL,PU,rateFn,mode)
% Both policies start from exactly the same reduction-only base.
% Generic: same-user opposite-band compensation whenever OWN stream was
% reduced, regardless of original interference case. Case: only original
% case 1 or case 2, on its safe opposite band. Same marginal solver.
% No donor-user power, no cross-band budget transfer, no reserve.
tx=base;
IrawL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IrawU=aggregate_interference_by_channel(raw,HW,GU,'un');
st=struct('cases',zeros(1,4),'compL',0,'compU',0, ...
    'eligible',0,'blocked',0,'accepted',0);
for u=1:numel(raw)
    for loc=1:numel(raw(u).users)
        [iL,pL,~]=get_band_fields(raw,u,'lic');
        [iU,pU,~]=get_band_fields(raw,u,'un');
        [bL,qL,~]=get_band_fields(base,u,'lic');
        [bU,qU,~]=get_band_fields(base,u,'un');
        eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
        fL=find(bL(:,1)==loc,1);fU=find(bU(:,1)==loc,1);
        highL=~isempty(eL)&&IrawL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
        highU=~isempty(eU)&&IrawU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
        if highL&&highU,st.cases(3)=st.cases(3)+1;
        elseif highL,st.cases(1)=st.cases(1)+1;
        elseif highU,st.cases(2)=st.cases(2)+1;
        else,st.cases(4)=st.cases(4)+1;end
        reducedL=~isempty(eL)&&~isempty(fL)&&pL(eL)>qL(fL)+1e-12;
        reducedU=~isempty(eU)&&~isempty(fU)&&pU(eU)>qU(fU)+1e-12;
        if strcmp(mode,'case')
            wantU=highL && ~highU && reducedL;
            wantL=highU && ~highL && reducedU;
        else
            wantU=reducedL;
            wantL=reducedU;
        end
        % One user may qualify for both directions in generic mode.
        % Fixed order in both methods; only eligibility differs.
        if wantU && ~isempty(fU)
            st.eligible=st.eligible+1;
            [trial,dp]=pure_marginal_compensate(tx,u,fU,HW,GU,PU,'un',rateFn);
            if dp>1e-12
                tx=trial;st.compU=st.compU+dp;st.accepted=st.accepted+1;
            else
                st.blocked=st.blocked+1;
            end
        end
        if wantL && ~isempty(fL)
            st.eligible=st.eligible+1;
            [trial,dp]=pure_marginal_compensate(tx,u,fL,HP,GL,PL,'lic',rateFn);
            if dp>1e-12
                tx=trial;st.compL=st.compL+dp;st.accepted=st.accepted+1;
            else
                st.blocked=st.blocked+1;
            end
        end
    end
end
end

function [best,added]=pure_marginal_compensate(tx,u,ell,Hprot,Gamma,Pmax,band,rateFn)
best=tx;added=0;
[idx,p,v]=get_band_fields(tx,u,band);
if ell>numel(p),return;end
s=idx(ell,2);I=aggregate_interference_by_channel(tx,Hprot,Gamma,band);
g=abs(Hprot{u,s}'*v{ell})^2;
unused=max(0,Pmax-sum(p));
head=max(0,Gamma(s)-I(s));
if I(s)>Gamma(s)*(1+1e-9),return;end
if g>1e-30,cap=min(unused,head/g);else,cap=unused;end
if cap<=1e-12,return;end
% Maximize actual total rate along SAME-SU feasible compensation dimension.
objective=@(dp) pure_negative_rate_step(tx,u,ell,band,p,dp,rateFn);
options=optimset('Display','off','TolX',1e-8);
[dp,~]=fminbnd(objective,0,cap,options);
for x=[dp,cap]
    candidate=tx;pp=p;pp(ell)=pp(ell)+x;
    candidate=set_band_power(candidate,u,band,pp);
    if rateFn(candidate)>rateFn(best)+1e-6
        best=candidate;added=x;
    end
end
end
function val=pure_negative_rate_step(tx,u,ell,band,p,dp,rateFn)
p(ell)=p(ell)+dp;
tx=set_band_power(tx,u,band,p);
val=-rateFn(tx);
end
function [rL,rU]=pure_check(tx,HP,HW,GL,GU,PL,PU)
IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
IU=aggregate_interference_by_channel(tx,HW,GU,'un');
rL=max(IL(:)./GL(:));rU=max(IU(:)./GU(:));
assert(rL<=1+1e-5 && rU<=1+1e-5,'Aggregate interference violated');
for u=1:numel(tx)
    assert(sum(tx(u).pLic(:))<=PL*(1+1e-5) && ...
        sum(tx(u).pUn(:))<=PU*(1+1e-5),'Independent power budget violated');
end
end

%% ===== Case-triggered same-user opposite-band subchannel SWAP helpers =====
function out=swap_classify_cases(raw,HP,HW,GL,GU)
IL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IU=aggregate_interference_by_channel(raw,HW,GU,'un');
out.byUser=cell(numel(raw),1);out.counts=zeros(1,4);
for u=1:numel(raw)
    nu=numel(raw(u).users);out.byUser{u}=4*ones(nu,1);
    for loc=1:nu
        iL=find(raw(u).idxLic(:,1)==loc,1);
        iU=find(raw(u).idxUn(:,1)==loc,1);
        highL=~isempty(iL)&&IL(raw(u).idxLic(iL,2))> ...
            GL(raw(u).idxLic(iL,2))*(1+1e-9);
        highU=~isempty(iU)&&IU(raw(u).idxUn(iU,2))> ...
            GU(raw(u).idxUn(iU,2))*(1+1e-9);
        if highL&&highU,caseId=3;
        elseif highL,caseId=1;
        elseif highU,caseId=2;
        else,caseId=4;end
        out.byUser{u}(loc)=caseId;
        out.counts(caseId)=out.counts(caseId)+1;
    end
end
end

function [best,info]=swap_search_same_user(start,cases,caseTriggered, ...
        HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
        MM_iter,leakFrac,alpha)
% One accepted swap maximum per affected user; same-UAV one-to-one swaps.
% Candidate evaluation identical for generic and case-triggered variants.
rateFn=@(tx) diagnostic_tx_rate(tx,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
best=start;bestRate=rateFn(best);
info=struct('tested',0,'accepted',0,'compL',0,'compU',0, ...
    'moves',zeros(0,5));
for u=1:numel(start)
    for loc=1:numel(start(u).users)
        caseId=cases.byUser{u}(loc);
        if caseTriggered
            if caseId==1,bands=2; % Licensed excessive: unlicensed swap.
            elseif caseId==2,bands=1; % Unlicensed excessive: licensed swap.
            elseif caseId==3,bands=[1 2];
            else,bands=[];end
        else
            bands=[1 2];
        end
        % Only one accepted swap per SU, as in the proposed experiment.
        selected=best;selectedRate=bestRate;selectedMeta=[];
        for b=bands
            if b==1,idx=best(u).idxLic;
            else,idx=best(u).idxUn;end
            e=find(idx(:,1)==loc,1);
            if isempty(e),continue;end
            for partner=1:size(idx,1)
                if partner==e,continue;end
                info.tested=info.tested+1;
                [trial,delta]=swap_trial(best,u,e,partner,b, ...
                    HL,HU,HP,HW,GL,GU,PL,PU,rateFn, ...
                    hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
                    MM_iter,leakFrac,alpha);
                r=rateFn(trial);
                if isfinite(r)&&r>selectedRate+1e-5
                    selected=trial;selectedRate=r;
                    selectedMeta=[u loc b idx(e,2) idx(partner,2) delta];
                end
            end
        end
        if ~isempty(selectedMeta)
            best=selected;bestRate=selectedRate;
            info.accepted=info.accepted+1;
            info.moves(end+1,:)=selectedMeta(1:5); %#ok<AGROW>
            if selectedMeta(3)==1,info.compL=info.compL+selectedMeta(6);
            else,info.compU=info.compU+selectedMeta(6);end
        end
    end
end
end

function [trial,delta]=swap_trial(base,u,e,partner,b, ...
        HL,HU,HP,HW,GL,GU,PL,PU,rateFn, ...
        hp,hw,Ppbs,Pwifi,nL,nU,BL,BU,MM_iter,leakFrac,alpha)
trial=base;delta=0;
if b==1
    idx=trial(u).idxLic; p=trial(u).pLic;
    Hs=HL;Hprot=HP;G=GL;Pmax=PL;vList=trial(u).vLic;
    denomInterf=hp;Pint=Ppbs;noise=nL;
else
    idx=trial(u).idxUn;p=trial(u).pUn;
    Hs=HU;Hprot=HW;G=GU;Pmax=PU;vList=trial(u).vUn;
    denomInterf=hw;Pint=Pwifi;noise=nU;
end
% Swap subchannel labels while preserving each stream's SU and its power.
% Recalculate BOTH affected users' leakage-aware beams for their NEW SCs.
scA=idx(e,2);scB=idx(partner,2);
idx(e,2)=scB;idx(partner,2)=scA;
for ell=[e partner]
    loc=idx(ell,1);globalK=trial(u).users(loc);sc=idx(ell,2);
    den=max(denomInterf(globalK)*Pint(sc)+noise,1e-30);
    [v,~,~]=mm_unit_beam(Hs{u,globalK},Hprot{u,sc}, ...
        den,G(sc),max(p(ell),1e-12),MM_iter,leakFrac,alpha);
    vList{ell}=v;
end
if b==1
    trial(u).idxLic=idx;trial(u).vLic=vList;
else
    trial(u).idxUn=idx;trial(u).vUn=vList;
end
% Protect all shared channels after changing beams/subchannel occupancy.
trial=apply_final_safety_clipping(trial,HP,HW,GL,GU,PL,PU,'both');
% SAME-SU compensation only on the newly selected subchannel, using
% leftover power of the same band and aggregate interference headroom.
% No donor-user power changes and no cross-band budget transfer.
if b==1
    band='lic';G=GL;Hprot=HP;
else
    band='un';G=GU;Hprot=HW;
end
[~,pNow,vNow]=get_band_fields(trial,u,band);
remaining=max(0,Pmax-sum(pNow));
sc=idx(e,2);leak=abs(Hprot{u,sc}'*vNow{e})^2;
I=aggregate_interference_by_channel(trial,Hprot,G,band);
headroom=max(0,(G(sc)-I(sc))/max(leak,realmin));
upper=min(remaining,headroom);
if upper>1e-10
    originalRate=rateFn(trial);
    obj=@(extra) -rateFn(swap_add_stream_power(trial,u,e,b,extra));
    [x,~]=fminbnd(obj,0,upper,optimset('TolX',1e-9,'Display','off'));
    % Also evaluate the boundary; fminbnd need not find global optimum.
    xs=[0 x upper];
    baseComp=trial;
    for t=1:numel(xs)
        cand=swap_add_stream_power(baseComp,u,e,b,xs(t));
        r=rateFn(cand);
        if r>originalRate+1e-5
            trial=cand;originalRate=r;delta=xs(t);
        end
    end
end
end

function out=swap_add_stream_power(base,u,e,b,extra)
out=base;
if b==1,out(u).pLic(e)=out(u).pLic(e)+extra;
else,out(u).pUn(e)=out(u).pUn(e)+extra;end
end

function swap_verify_assignment(tx,raw)
% All initial SU associations, per-band user coverage and per-UAV SC sets
% remain unchanged. Swaps are a permutation of existing SC assignments.
for u=1:numel(raw)
    assert(isequal(tx(u).users,raw(u).users),'Association changed');
    assert(isequal(tx(u).idxLic(:,1),raw(u).idxLic(:,1)), ...
        'Licensed user assignments changed');
    assert(isequal(tx(u).idxUn(:,1),raw(u).idxUn(:,1)), ...
        'Unlicensed user assignments changed');
    assert(isequal(sort(tx(u).idxLic(:,2)),sort(raw(u).idxLic(:,2))), ...
        'Licensed channel set changed');
    assert(isequal(sort(tx(u).idxUn(:,2)),sort(raw(u).idxUn(:,2))), ...
        'Unlicensed channel set changed');
end
end


%% =========== FOUR-CASE RESERVED SUBCHANNEL CONTROLLERS ===========
function [best,info]=reserved_sc_search(start,policy,replacement, ...
    cases,HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
    mmIter,leakFrac,alpha,resL,resU)
% Legacy unlimited version is intentionally not used in this experiment.
error('Use budgeted_reserved_search instead');
end
function [best,info]=budgeted_reserved_search(start,policy,maxCandidates, ...
    cases,HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
    mmIter,leakFrac,alpha,resL,resU)
% Both arms call exactly the same candidate evaluator. Budget counts
% FULL candidate evaluations, including rejected/infeasible trials.
rateFn=@(tx) diagnostic_tx_rate(tx,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
best=start;bestRate=rateFn(best);
info=struct('tested',0,'accepted',0,'testedByCase',zeros(1,4), ...
    'acceptedByCase',zeros(1,4),'gainByCase',zeros(1,4));
for u=1:numel(start)
    for loc=1:numel(start(u).users)
        caseId=cases.byUser{u}(loc);
        if policy==0
            bands=[1 2];
        elseif policy==2
            if caseId==1,bands=2;
            elseif caseId==2,bands=1;
            else,bands=[1 2];end
        elseif policy==3
            if caseId==4,continue;end
            if caseId==1,bands=2;
            elseif caseId==2,bands=1;
            else,bands=[1 2];end
        elseif policy==4
            if caseId~=4,continue;end
            bands=[1 2];
        else
            error('Unknown replacement policy');
        end
        selected=best;selectedRate=bestRate;
        for b=bands
            if b==1,idx=best(u).idxLic;reserve=resL;
            else,idx=best(u).idxUn;reserve=resU;end
            e=find(idx(:,1)==loc,1);
            if isempty(e),continue;end
            for sc=reserve
                if reserved_used_anywhere(best,sc,b),continue;end
                if info.tested>=maxCandidates,break;end
                info.tested=info.tested+1;
                info.testedByCase(caseId)=info.testedByCase(caseId)+1;
                [trial,~]=reserved_sc_trial(best,u,e,sc,b,true, ...
                    HL,HU,HP,HW,GL,GU,PL,PU,rateFn,hp,hw, ...
                    Ppbs,Pwifi,nL,nU,mmIter,leakFrac,alpha);
                r=rateFn(trial);
                if isfinite(r)&&r>selectedRate+1e-5
                    selected=trial;selectedRate=r;
                end
            end
            if info.tested>=maxCandidates,break;end
        end
        if selectedRate>bestRate+1e-5
            info.accepted=info.accepted+1;
            info.acceptedByCase(caseId)=info.acceptedByCase(caseId)+1;
            info.gainByCase(caseId)=info.gainByCase(caseId)+ ...
                (selectedRate-bestRate);
            best=selected;bestRate=selectedRate;
        end
        if info.tested>=maxCandidates,return;end
    end
end
end

function tf=reserved_used_anywhere(tx,sc,b)
tf=false;
for u=1:numel(tx)
    if b==1,idx=tx(u).idxLic;else,idx=tx(u).idxUn;end
    if any(idx(:,2)==sc),tf=true;return;end
end
end
function [trial,delta]=reserved_sc_trial(base,u,e,sc,b,replacement, ...
    HL,HU,HP,HW,GL,GU,PL,PU,rateFn,hp,hw, ...
    Ppbs,Pwifi,nL,nU,mmIter,leakFrac,alpha)
trial=base;delta=0;
if b==1
    idx=base(u).idxLic;p=base(u).pLic(:);v=base(u).vLic;
    HS=HL;Hprot=HP;G=GL;Pmax=PL;hi=hp;Pi=Ppbs;noise=nL;
    band='lic';
else
    idx=base(u).idxUn;p=base(u).pUn(:);v=base(u).vUn;
    HS=HU;Hprot=HW;G=GU;Pmax=PU;hi=hw;Pi=Pwifi;noise=nU;
    band='un';
end
loc=idx(e,1);k=base(u).users(loc);
den=max(hi(k)*Pi(sc)+noise,1e-30);
[vNew,~,~]=mm_unit_beam(HS{u,k},Hprot{u,sc}, ...
    den,G(sc),max(p(e),1e-12),mmIter,leakFrac,alpha);
if replacement
    % Move existing power/stream to a RESERVED channel. No extra bandwidth
    % used by this SU, and no additional power is created.
    idx(e,2)=sc;v{e}=vNew;
else
    % Add a new independent OFDMA stream for the SAME SU, only if its
    % corresponding UAV/band has unused power and the reserved SC is safe.
    remaining=max(0,Pmax-sum(p));
    if remaining<=1e-12,return;end
    idx(end+1,:)=[loc sc];p(end+1,1)=0;v{end+1,1}=vNew;
end
if b==1
    trial(u).idxLic=idx;trial(u).pLic=p;trial(u).vLic=v;
else
    trial(u).idxUn=idx;trial(u).pUn=p;trial(u).vUn=v;
end
% Replacement may alter aggregate protection on a reserved channel.
% Do NOT clip: protection is a feasibility test of the reserved candidate.
[preRatioL,preRatioU]=pure_check(trial,HP,HW,GL,GU,PL,PU);
if preRatioL>1+1e-5 || preRatioU>1+1e-5
    trial=base;
    return;
end
if replacement
    % Optimize extra power only if unused power exists after moving.
    [~,pNow,vNow]=get_band_fields(trial,u,band);
    stream=e;
else
    [~,pNow,vNow]=get_band_fields(trial,u,band);
    stream=numel(pNow);
end
remaining=max(0,Pmax-sum(pNow));
I=aggregate_interference_by_channel(trial,Hprot,G,band);
leak=abs(Hprot{u,sc}'*vNow{stream})^2;
head=max(0,G(sc)-I(sc));
if leak>1e-30,upper=min(remaining,head/leak);
else,upper=remaining;end
if upper>1e-12
    % Same scalar optimization for generic and case-triggered arms.
    obj=@(x) -rateFn(reserved_add_power(trial,u,stream,b,x));
    [x,~]=fminbnd(obj,0,upper,optimset('TolX',1e-9,'Display','off'));
    choices=[0 x upper];
    baseRate=rateFn(trial);bestExtra=0;
    for ii=1:numel(choices)
        candidate=reserved_add_power(trial,u,stream,b,choices(ii));
        r=rateFn(candidate);
        if isfinite(r)&&r>baseRate+1e-5
            baseRate=r;bestExtra=choices(ii);
        end
    end
    trial=reserved_add_power(trial,u,stream,b,bestExtra);
    delta=bestExtra;
end
% Do not retain a zero-power added stream. This also prevents an empty
% reservation from being counted as an accepted channel activation.
if ~replacement && delta<=1e-12
    trial=base;return;
end
% All accepted changes must satisfy aggregate interference and budgets.
[ratioL,ratioU]=pure_check(trial,HP,HW,GL,GU,PL,PU);
assert(ratioL<=1+1e-5&&ratioU<=1+1e-5,'Reserved trial infeasible');
end
function out=reserved_add_power(base,u,e,b,extra)
out=base;
if b==1,out(u).pLic(e)=out(u).pLic(e)+extra;
else,out(u).pUn(e)=out(u).pUn(e)+extra;end
end
function reserved_verify(tx,raw,resL,resU,allowReplacement)
for u=1:numel(tx)
    assert(isequal(tx(u).users,raw(u).users), ...
        'User association changed unexpectedly');
    for b=1:2
        if b==1,idx=tx(u).idxLic;original=raw(u).idxLic;reserve=resL;
        else,idx=tx(u).idxUn;original=raw(u).idxUn;reserve=resU;end
        assert(numel(unique(idx(:,2)))==size(idx,1), ...
            'One UAV has two streams on one subchannel');
        for ell=1:size(idx,1)
            assert(any(original(:,1)==idx(ell,1)), ...
                'An unassociated user received a stream');
        end
        if ~allowReplacement
            assert(isequal(idx(1:size(original,1),:),original), ...
                'Initial assignment changed during addition-only test');
            assert(all(ismember(idx(size(original,1)+1:end,2),reserve)), ...
                'Added stream is not on a reserved subchannel');
        else
            assert(isequal(idx(:,1),original(:,1))&& ...
                size(idx,1)==size(original,1), ...
                'Replacement changed the number of original streams');
            changed=idx(:,2)~=original(:,2);
            assert(all(ismember(idx(changed,2),reserve)), ...
                'Replacement used a non-reserved channel');
        end
    end
end
for b=1:2
    if b==1,reserve=resL;else,reserve=resU;end
    for sc=reserve
        n=0;
        for u=1:numel(tx)
            if b==1,idx=tx(u).idxLic;else,idx=tx(u).idxUn;end
            n=n+sum(idx(:,2)==sc);
        end
        assert(n<=1,'Reserved SC assigned to multiple UAVs');
    end
end
end
function reserved_plot(x,Y,names,ylab,filename)
figure('Position',[100 100 620 470]);
styles={'o-','s--','d-.','^-','v:'};hold on;
for ii=1:size(Y,2)
    plot(x,Y(:,ii),styles{ii},'LineWidth',2,'MarkerSize',7);
end
ax=gca;ax.FontName='Times New Roman';ax.FontSize=16;
ax.FontWeight='normal';ax.LineWidth=1.2;
ax.Position=[0.13 0.13 0.84 0.68];grid on;
ax.GridLineStyle=':';ax.GridAlpha=0.30;
xlabel('Paired channel realization');ylabel(ylab);
legend(names,'FontName','Times New Roman','FontSize',12, ...
    'NumColumns',2,'Location','northoutside');
saveas(gcf,filename);
end


%% ===== FINAL RAW-INTERFERENCE RESERVED-SUBCHANNEL CONTROLLER =====
function [best,stats]=fourcase_crossband_reserved_controller(base,cases,...
    HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU,...
    mmIter,leakFrac,alpha,resL,resU)
% RESERVED ALGORITHM USED IN PROPOSED II
% ------------------------------------------------------------
% 1) BASE is the RAW allocation after ordinary power allocation.
% 2) No protection/clipping is applied before the reserved search.
% 3) For every interference-affected SU, candidate replacements are tested
%    on the licensed band, unlicensed band, and both bands jointly.
% 4) A candidate is preferred first by aggregate-protection feasibility;
%    while the current allocation is still infeasible, the search selects
%    the candidate that most reduces the worst normalized violation.
% 5) Once protection is achieved, only protected candidates are eligible
%    and the candidate with the largest actual network sum rate is retained.
% 6) No post-search safety clipping is used. The returned allocation must
%    itself satisfy PU/WU aggregate-interference and per-UAV power budgets.
%
% This is a coordinated sequential candidate search (local search), not a
% claim of globally optimal mixed-integer enumeration.
rateFn=@(tx) diagnostic_tx_rate(tx,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
best=base;
stats=struct('tested',0,'accepted',0,'caseCounts',zeros(1,4),...
    'testedByCase',zeros(1,4),'acceptedByCase',zeros(1,4));

% Count raw interference cases.
for u=1:numel(base)
    for loc=1:numel(base(u).users)
        cid=cases.byUser{u}(loc);
        stats.caseCounts(cid)=stats.caseCounts(cid)+1;
    end
end

% Multiple coordinated passes allow replacements for several affected SUs
% to collectively remove a network-wide aggregate-interference violation.
maxPass=max(1,2*sum(stats.caseCounts(1:3)));
for pass=1:maxPass
    changed=false;
    for u=1:numel(best)
        for loc=1:numel(best(u).users)
            caseId=cases.byUser{u}(loc);
            if caseId==4
                continue; % raw allocation is safe for this SU on both bands
            end

            incumbent=best;
            incumbentRate=rateFn(incumbent);
            incumbentViolation=reserved_violation_metric(incumbent,HP,HW,GL,GU,PL,PU);
            chosen=incumbent;
            chosenRate=incumbentRate;
            chosenViolation=incumbentViolation;

            % Check BOTH bands. NaN means keep the original SC on that band.
            candidateL=[NaN resL];
            candidateU=[NaN resU];
            for il=1:numel(candidateL)
                for iu=1:numel(candidateU)
                    newL=candidateL(il); newU=candidateU(iu);
                    if isnan(newL) && isnan(newU)
                        continue;
                    end
                    if ~isnan(newL) && reserved_used_anywhere(incumbent,newL,1)
                        continue;
                    end
                    if ~isnan(newU) && reserved_used_anywhere(incumbent,newU,2)
                        continue;
                    end

                    trial=incumbent;
                    if ~isnan(newL)
                        e=find(trial(u).idxLic(:,1)==loc,1);
                        if isempty(e),continue;end
                        trial=reserved_replace_no_clip(trial,u,e,newL,1,...
                            HL,HU,HP,HW,GL,GU,hp,hw,Ppbs,Pwifi,nL,nU,...
                            mmIter,leakFrac,alpha);
                    end
                    if ~isnan(newU)
                        e=find(trial(u).idxUn(:,1)==loc,1);
                        if isempty(e),continue;end
                        trial=reserved_replace_no_clip(trial,u,e,newU,2,...
                            HL,HU,HP,HW,GL,GU,hp,hw,Ppbs,Pwifi,nL,nU,...
                            mmIter,leakFrac,alpha);
                    end

                    % Re-optimize powers under the actual aggregate protection
                    % headroom. This routine uses capped KKT only; no clipping.
                    trial=fourcase_kkt_refine_candidate(trial,u,...
                        HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,...
                        nL,nU,BL,BU);

                    stats.tested=stats.tested+1;
                    stats.testedByCase(caseId)=stats.testedByCase(caseId)+1;
                    v=reserved_violation_metric(trial,HP,HW,GL,GU,PL,PU);
                    r=rateFn(trial);
                    if ~isfinite(r),continue;end

                    if incumbentViolation>1+1e-6
                        % Before feasibility is reached, reduce violation first;
                        % use rate only as the tie-breaker.
                        better=(v<chosenViolation-1e-9) || ...
                            (abs(v-chosenViolation)<=1e-9 && r>chosenRate+1e-5);
                    else
                        % Once protected, maximize rate over protected candidates.
                        better=(v<=1+1e-6) && r>chosenRate+1e-5;
                    end
                    if better
                        chosen=trial;
                        chosenRate=r;
                        chosenViolation=v;
                    end
                end
            end

            if chosenViolation<incumbentViolation-1e-9 || ...
                    (chosenViolation<=1+1e-6 && incumbentViolation<=1+1e-6 && ...
                     chosenRate>incumbentRate+1e-5)
                best=chosen;
                changed=true;
                stats.accepted=stats.accepted+1;
                stats.acceptedByCase(caseId)=stats.acceptedByCase(caseId)+1;
            end
        end
    end
    if reserved_violation_metric(best,HP,HW,GL,GU,PL,PU)<=1+1e-6
        % One additional protected rate-improvement pass is enough; if this
        % pass made no change, the coordinated sequential search has converged.
        if ~changed || pass>1
            break;
        end
    elseif ~changed
        break;
    end
end

% Final verification only. Candidate evaluation above is intentionally
% non-asserting because the RAW allocation may violate protection.
ILfinal=aggregate_interference_by_channel(best,HP,GL,'lic');
IUfinal=aggregate_interference_by_channel(best,HW,GU,'un');
ratioL=max(ILfinal(:)./max(GL(:),realmin));
ratioU=max(IUfinal(:)./max(GU(:),realmin));
powerOK=true;
for uu=1:numel(best)
    powerOK=powerOK && sum(best(uu).pLic(:))<=PL*(1+1e-5) && ...
        sum(best(uu).pUn(:))<=PU*(1+1e-5);
end
assert(ratioL<=1+1e-5 && ratioU<=1+1e-5 && powerOK,...
    ['No feasible reserved-subchannel combination was found without ' ...
     'power clipping. Increase the reserved pool or revise the raw allocation.']);
end

function trial=reserved_replace_no_clip(base,u,e,sc,b,...
    HL,HU,HP,HW,GL,GU,hp,hw,Ppbs,Pwifi,nL,nU,...
    mmIter,leakFrac,alpha)
% Replace one existing stream by a reserved subchannel while preserving its
% current power. Recompute its leakage-aware beam on the new subchannel.
% NO clipping or safety scaling is performed here.
trial=base;
if b==1
    idx=trial(u).idxLic; p=trial(u).pLic(:); v=trial(u).vLic;
    HS=HL; Hprot=HP; G=GL; hi=hp; Pi=Ppbs; noise=nL;
else
    idx=trial(u).idxUn; p=trial(u).pUn(:); v=trial(u).vUn;
    HS=HU; Hprot=HW; G=GU; hi=hw; Pi=Pwifi; noise=nU;
end
loc=idx(e,1);
k=trial(u).users(loc);
den=max(hi(k)*Pi(sc)+noise,1e-30);
[vNew,~,~]=mm_unit_beam(HS{u,k},Hprot{u,sc},...
    den,G(sc),max(p(e),1e-12),mmIter,leakFrac,alpha);
idx(e,2)=sc;
v{e}=vNew;
if b==1
    trial(u).idxLic=idx;
    trial(u).vLic=v;
else
    trial(u).idxUn=idx;
    trial(u).vUn=v;
end
end

function metric=reserved_violation_metric(tx,HP,HW,GL,GU,PL,PU)
% Worst normalized aggregate-interference or per-UAV power-budget ratio.
% IMPORTANT: this is a NON-ASSERTING candidate metric. During the reserved
% search, infeasible raw/trial allocations are expected and must be scored,
% not rejected by pure_check(). Final protection is asserted only after the
% reserved-subchannel search has selected its output.
IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
IU=aggregate_interference_by_channel(tx,HW,GU,'un');
ratioL=max(IL(:)./max(GL(:),realmin));
ratioU=max(IU(:)./max(GU(:),realmin));
ratioPL=0; ratioPU=0;
for uu=1:numel(tx)
    ratioPL=max(ratioPL,sum(tx(uu).pLic(:))/max(PL,realmin));
    ratioPU=max(ratioPU,sum(tx(uu).pUn(:))/max(PU,realmin));
end
metric=max([ratioL,ratioU,ratioPL,ratioPU]);
end

function trial=fourcase_kkt_refine_candidate(base,u,...
    HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU)
% Same-UAV per-band capped KKT refinement. Other UAVs are held fixed.
% Protection is imposed through residual aggregate-interference caps inside
% the KKT allocation itself. There is NO safety clipping after this step.
trial=base;
for b=1:2
    if b==1
        band='lic';HS=HL;Hprot=HP;G=GL;Pmax=PL;
        hI=hp;Pi=Ppbs;noise=nL;
        idx=trial(u).idxLic;vs=trial(u).vLic;
    else
        band='un';HS=HU;Hprot=HW;G=GU;Pmax=PU;
        hI=hw;Pi=Pwifi;noise=nU;
        idx=trial(u).idxUn;vs=trial(u).vUn;
    end
    if isempty(idx),continue;end
    a=zeros(size(idx,1),1);
    caps=zeros(size(idx,1),1);
    Iall=aggregate_interference_by_channel(trial,Hprot,G,band);
    for e=1:size(idx,1)
        sc=idx(e,2);
        k=trial(u).users(idx(e,1));
        if b==1
            pcur=trial(u).pLic(e);
        else
            pcur=trial(u).pUn(e);
        end
        leak=abs(Hprot{u,sc}'*vs{e})^2;
        otherLeak=max(0,Iall(sc)-pcur*leak);
        caps(e)=min(Pmax,max(0,G(sc)-otherLeak)/max(leak,realmin));
        den=max(hI(k)*Pi(sc)+noise+...
            inter_uav_power_same_sc(trial,HS,u,k,sc,band),1e-30);
        a(e)=max(abs(HS{u,k}'*vs{e})^2/den,1e-30);
    end
    pNew=kkt_waterfill_capped_original(a,Pmax,caps);
    trial=set_band_power(trial,u,band,pNew);
end
end

function [txBest,stats] = original_aiibpc_rateaware_diagnostic(raw,txConv,txFour, ...
    HP,HW,GL,GU,PL,PU,rateFn)
    % EXPERIMENTAL rate-aware, fallback-enhanced variant, NOT the
    % original AI-IBPC definition. Both protected references are polished.
    % Protection was applied in the original four-case block (txFour),
    % or conventional reference protection (txConv), never in raw allocation.
    %#ok<INUSD> raw is retained in interface for auditability.
    [txConv,convStats] = rateaware_polish_existing( ...
        txConv,HP,HW,GL,GU,PL,PU,rateFn);
    [txFour,fourStats] = rateaware_polish_existing( ...
        txFour,HP,HW,GL,GU,PL,PU,rateFn);
    rConv=rateFn(txConv);rFour=rateFn(txFour);
    if rFour > rConv+1e-9
        txBest=txFour;stats.source='fourcase';
    else
        txBest=txConv;stats.source='protected_reference';
    end
    [rL,rU]=pure_check(txBest,HP,HW,GL,GU,PL,PU);
    assert(max(rL,rU)<=1+1e-5,'Rate-aware original violates protection');
    stats.rate=rateFn(txBest);
    stats.objectiveCalls=convStats.objectiveCalls+fourStats.objectiveCalls+3;
    stats.scalarSearches=convStats.scalarSearches+fourStats.scalarSearches;
    stats.acceptedUpdates=convStats.acceptedUpdates+fourStats.acceptedUpdates;
    stats.conventionalPolish=convStats;
    stats.fourcasePolish=fourStats;
end

function [tx,stats]=rateaware_polish_existing(tx,HP,HW,GL,GU,PL,PU,rateFn)
    % Count all rate objective calls including fminbnd internal trials.
    stats=struct('objectiveCalls',0,'scalarSearches',0,'acceptedUpdates',0);
    bestRate=counted_rate(tx);
    for pass=1:2
        improved=false;
        for u=1:numel(tx)
            for bb=1:2
                if bb==1
                    band='lic';Hprot=HP;Gamma=GL;Pmax=PL;
                else
                    band='un';Hprot=HW;Gamma=GU;Pmax=PU;
                end
                [idx,p,v]=get_band_fields(tx,u,band);
                for e=1:numel(p)
                    sc=idx(e,2);
                    leak=abs(Hprot{u,sc}'*v{e})^2;
                    I=aggregate_interference_by_channel(tx,Hprot,Gamma,band);
                    other=max(0,I(sc)-p(e)*leak);
                    budget=max(0,Pmax-(sum(p)-p(e)));
                    if leak>1e-30
                        protect=max(0,(Gamma(sc)-other)/leak);
                    else
                        protect=inf;
                    end
                    upper=min(budget,protect);
                    if upper<=0,continue;end
                    current=min(max(0,p(e)),upper);
                    fun=@(power)counted_trial(tx,u,band,p,e,power);
                    opts=optimset('TolX',max(1e-11,upper*1e-5),'Display','off');
                    stats.scalarSearches=stats.scalarSearches+1;
                    pStar=fminbnd(@(power)-fun(power),0,upper,opts);
                    tests=unique([0;current;upper;pStar]);
                    scores=zeros(numel(tests),1);
                    for tt=1:numel(tests)
                        scores(tt)=fun(tests(tt));
                    end
                    [candidateRate,k]=max(scores);
                    if candidateRate>bestRate+1e-6
                        p(e)=tests(k);
                        tx=set_band_power(tx,u,band,p);
                        bestRate=candidateRate;
                        stats.acceptedUpdates=stats.acceptedUpdates+1;
                        improved=true;
                    end
                end
            end
        end
        if ~improved,break;end
    end
    function r=counted_rate(t)
        stats.objectiveCalls=stats.objectiveCalls+1;
        r=rateFn(t);
    end
    function r=counted_trial(t,uu,bandName,powers,ee,power)
        stats.objectiveCalls=stats.objectiveCalls+1;
        r=rateaware_trial_rate(t,uu,bandName,powers,ee,power,rateFn);
    end
end

function r=rateaware_trial_rate(tx,u,band,p,e,power,rateFn)
    p(e)=power;
    tx=set_band_power(tx,u,band,p);
    r=rateFn(tx);
end




