clc; clear; close all;
% Modularized from the original repository: function bodies unchanged.
projectRoot=fileparts(fileparts(mfilename('fullpath')));
overrideDir=fullfile(projectRoot,'Convergence_Overrides');
if contains([path pathsep],[overrideDir pathsep]),rmpath(overrideDir);end
addpath(fullfile(projectRoot,'Common_Functions'),'-begin');
addpath(fullfile(projectRoot,'Cases_Functions'),'-begin');

%% ============================================================
% JOURNAL FOUR-CASE INTERFERENCE STUDY: AI-IBPC versus IA-RSS
% ------------------------------------------------------------
% Panel (a): raw four-case distribution AFTER association, Hungarian
% allocation, CBF-MM, and ordinary KKT power allocation, BEFORE the
% proposed interference-management controller.
%
% Panel (b): retained total power-budget utilization after applying the
% ACTUAL journal AI-IBPC and IA-RSS controllers to the same raw sample.
%
% IMPORTANT:
%   - AI-IBPC logic is unchanged.
%   - IA-RSS logic is unchanged.
%   - No LPRUPA / shared cross-band budget logic remains.
%   - No artificial equal-power raw state is used.
%   - Figure cases are classified ONCE per whole-network realization.
%   - IA-RSS retains its original per-SU classifier internally.
%% ============================================================

%% -------------------- System parameters --------------------
area=1000; H=100; U=3; K=6; Ninitial=6; J=10; M=10; Nt=8;
Bsc_lic=180e3; Bsc_un=180e3; N0_dBmHz=-169;
sigma2_lic=10.^((N0_dBmHz+10*log10(Bsc_lic)-30)/10);
sigma2_un =10.^((N0_dBmHz+10*log10(Bsc_un )-30)/10);
Pmax_lic=0.5; Pmax_un=0.5; P_PBS=2.5; P_WIFI=1;
P_PBS_j=P_PBS*ones(J,1); P_WIFI_m=P_WIFI*ones(M,1);
c=3e8; f_lic=2e9; f_un=5e9; f_pbs=f_lic; f_wifi=f_un;
n_pbs=3.7; n_wifi=3; sigmaSh_pbs_dB=8; sigmaSh_wifi_dB=6;
A2G=struct('a',9.61,'b',0.16,'etaLoS_dB',1,'etaNLoS_dB',20,...
    'sigmaLoS_dB',4,'sigmaNLoS_dB',6);
useRayleigh=true; useShadow=true; useShadowA2G=true;
MM_iter=10; MM_leakFrac=1; MM_reg_alpha=1e-3; QoS_target=0;
GammaLic_dBm=-80; GammaUn_dBm=-80;
GammaLic=10.^((GammaLic_dBm-30)/10)*ones(J,1);
GammaUn =10.^((GammaUn_dBm -30)/10)*ones(M,1);
w_b=[area/2 area/2]; w_wifi=[area/2 area/2+200];
AO_AP_iter=3;
reserveLic=(Ninitial+1):J;
reserveUn =(Ninitial+1):M;

%% -------------------- Simulation settings --------------------
NDrops=30;
MC=300;
Tflight=220; dt_fixed=5; Vmax_UAV=5;
N=round(Tflight/dt_fixed)+1;
STEP_MAX_FIXED=Vmax_UAV*dt_fixed;
Dmin_UAV=100;
dropSeedBase=9000;
channelSeedBase=12345;
q_start_all=[120 850;260 180;430 620];
q_end_all=q_start_all;

caseNames={'Licensed high only','Unlicensed high only',...
           'Both bands high','Both bands normal'};
methodNames={'AI-IBPC','IA-RSS'};
nCases=4; nMethods=2;

% Statistics are accumulated per drop so confidence intervals remain valid.
dropRawCasePct=zeros(NDrops,nCases);
dropConditionalPower=nan(NDrops,nCases,nMethods);

fprintf('\n============================================================\n');
fprintf('Journal four-case study: AI-IBPC versus IA-RSS\n');
fprintf('NDrops=%d, MC=%d, N=%d\n',NDrops,MC,N);
fprintf('GammaLic=%.1f dBm, GammaUn=%.1f dBm\n',GammaLic_dBm,GammaUn_dBm);
fprintf('Active SCs=%d per band, reserved SCs=%d per band\n',Ninitial,numel(reserveLic));
fprintf('============================================================\n');

%% -------------------- Main simulation --------------------
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

    % Common feasible trajectory for this controller-focused diagnostic.
    q_hover_all=make_multi_hover_points_closed(q_start_all,WS,area);
    Q=build_multi_fly_hover_trajectory(q_start_all,q_hover_all,q_end_all,...
        N,STEP_MAX_FIXED,area);
    if ~check_multi_traj_feasibility(Q,q_start_all,q_end_all,...
            STEP_MAX_FIXED,area,Dmin_UAV)
        Q=repair_multi_trajectory_cvx(Q,area,STEP_MAX_FIXED,...
            q_start_all,q_end_all,Dmin_UAV);
    end

    rawCounts=zeros(1,nCases);
    powerSums=zeros(nCases,nMethods);
    conditionalCounts=zeros(1,nCases);

    for mc=1:MC
        for n=1:N
            seedNow=channelSeedBase+100000*drop+1000*mc+n;
            Qslot=squeeze(Q(:,n,:));

            % Ordinary communication solution on ACTIVE non-reserved SCs.
            [~,~,~,~,~,dd]=eval_slot_multi_qos_bf(...
                Qslot,WS,WP(1:Ninitial,:),WW(1:Ninitial,:),...
                w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,...
                n_pbs,n_wifi,Sinit,P_PBS_j(1:Ninitial),...
                P_WIFI_m(1:Ninitial),sigma2_lic,sigma2_un,...
                GammaLic(1:Ninitial),GammaUn(1:Ninitial),...
                Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,...
                AO_AP_iter,seedNow,useRayleigh,useShadow,useShadowA2G,...
                Nt,MM_iter,MM_leakFrac,MM_reg_alpha,'prop-pb-mm',0,true);
            raw=dd.rawTx;

            % Generate the SAME channels used by the two journal controllers.
            [HL,HU,HP0,HW0,hp,hw]=gen_multi_channels_vec(...
                Qslot,WS,WP(1:Ninitial,:),WW(1:Ninitial,:),...
                w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,A2G,...
                n_pbs,n_wifi,Sinit,seedNow,useRayleigh,useShadow,...
                useShadowA2G,Nt);
            [~,~,HPextra,HWextra]=gen_multi_channels_vec(...
                Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,...
                c,A2G,n_pbs,n_wifi,Shadow,seedNow+250000,...
                useRayleigh,useShadow,useShadowA2G,Nt);
            HP=cell(U,J); HW=cell(U,M);
            HP(:,1:Ninitial)=HP0; HW(:,1:Ninitial)=HW0;
            HP(:,reserveLic)=HPextra(:,reserveLic);
            HW(:,reserveUn)=HWextra(:,reserveUn);

            % Raw per-SU four-case classification BEFORE either controller.
            cases=swap_classify_cases(raw,HP,HW,GammaLic,GammaUn);
            networkCase=classify_raw_network_case(raw,HP,HW,GammaLic,GammaUn);
            rawCounts(networkCase)=rawCounts(networkCase)+1;

            rateFn=@(tx)diagnostic_tx_rate(tx,HL,HU,hp,hw,...
                P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);

            % Proposed I: ACTUAL integrated four-case rate-aware AI-IBPC.
            [txAI,~]=ai_ibpc_integrated_four_case_rateaware(...
                raw,HP,HW,GammaLic,GammaUn,Pmax_lic,Pmax_un,rateFn);
            [chkL,chkU]=pure_check(txAI,HP,HW,GammaLic,GammaUn,...
                Pmax_lic,Pmax_un);
            assert(max(chkL,chkU)<=1+1e-5,'AI-IBPC protection violation.');

            % Proposed II: ACTUAL four-case IA-RSS reserved-subchannel search.
            [txRSS,~]=fourcase_crossband_reserved_controller(...
                raw,cases,HL,HU,HP,HW,GammaLic,GammaUn,...
                Pmax_lic,Pmax_un,hp,hw,P_PBS_j,P_WIFI_m,...
                sigma2_lic,sigma2_un,Bsc_lic,Bsc_un,...
                MM_iter,MM_leakFrac,MM_reg_alpha,reserveLic,reserveUn);
            [chkL,chkU]=pure_check(txRSS,HP,HW,GammaLic,GammaUn,...
                Pmax_lic,Pmax_un);
            assert(max(chkL,chkU)<=1+1e-5,'IA-RSS protection violation.');

            utilAI=total_power_utilization_pct(txAI,Pmax_lic,Pmax_un);
            utilRSS=total_power_utilization_pct(txRSS,Pmax_lic,Pmax_un);

            % Each complete network realization contributes exactly once.
            % Condition network-wide utilization on its single raw case.
            conditionalCounts(networkCase)=conditionalCounts(networkCase)+1;
            powerSums(networkCase,1)=powerSums(networkCase,1)+utilAI;
            powerSums(networkCase,2)=powerSums(networkCase,2)+utilRSS;
        end
    end

    totalClassified=sum(rawCounts);
    assert(totalClassified==MC*N,'Unexpected number of network realizations.');
    dropRawCasePct(drop,:)=100*rawCounts/totalClassified;
    for cc=1:nCases
        if conditionalCounts(cc)>0
            dropConditionalPower(drop,cc,:)=powerSums(cc,:)/conditionalCounts(cc);
        end
    end

    fprintf('Drop %2d/%2d | raw cases [%5.1f %5.1f %5.1f %5.1f]%%\n',...
        drop,NDrops,dropRawCasePct(drop,1),dropRawCasePct(drop,2),...
        dropRawCasePct(drop,3),dropRawCasePct(drop,4));
end

%% -------------------- Aggregate results --------------------
rawCaseMean=mean(dropRawCasePct,1);
rawCaseCI95=1.96*std(dropRawCasePct,0,1)/sqrt(NDrops);
conditionalPowerMean=nan(nCases,nMethods);
conditionalPowerCI95=nan(nCases,nMethods);
for cc=1:nCases
    for mm=1:nMethods
        x=squeeze(dropConditionalPower(:,cc,mm));
        x=x(isfinite(x));
        if ~isempty(x)
            conditionalPowerMean(cc,mm)=mean(x);
            if numel(x)>1
                conditionalPowerCI95(cc,mm)=1.96*std(x,0)/sqrt(numel(x));
            else
                conditionalPowerCI95(cc,mm)=0;
            end
        end
    end
end

assert(abs(sum(rawCaseMean)-100)<1e-8,...
    'Raw case percentages do not sum to 100%%.');
assert(all(conditionalPowerMean(:)<=100+1e-6 | isnan(conditionalPowerMean(:))),...
    'Power-budget utilization exceeds 100%%.');

ResultOccurrence=table(caseNames(:),rawCaseMean(:),rawCaseCI95(:),...
    'VariableNames',{'RawCase','Occurrence_pct','CI95_pct'});
ResultConditionalPower=table(caseNames(:),...
    conditionalPowerMean(:,1),conditionalPowerCI95(:,1),...
    conditionalPowerMean(:,2),conditionalPowerCI95(:,2),...
    'VariableNames',{'RawCase','AI_IBPC_Utilization_pct','AI_IBPC_CI95_pct',...
    'IA_RSS_Utilization_pct','IA_RSS_CI95_pct'});

disp(' '); disp('=== Network-wide raw four-case distribution ==='); disp(ResultOccurrence);
disp('=== Power-budget utilization conditioned on raw case ===');
disp(ResultConditionalPower);

%% -------------------- Two-panel journal figure --------------------
showErrorBars=false;
fig=figure('Color','w','Units','inches','Position',[1 1 7.2 3.65]);
t=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
caseColors=[0.0000 0.4470 0.7410;0.8500 0.3250 0.0980;...
            0.9290 0.6940 0.1250;0.4940 0.1840 0.5560];
methodColors=[0.0000 0.5000 0.5000;0.6350 0.0780 0.1840];
shortCaseLabels={'Lic. high','Unlic. high','Both high','Both normal'};

ax1=nexttile(t,1);
b1=bar(ax1,1:nCases,rawCaseMean,0.62,'FaceColor','flat','EdgeColor','none');
b1.CData=caseColors; hold(ax1,'on'); box(ax1,'on'); grid(ax1,'on');
if showErrorBars
    errorbar(ax1,1:nCases,rawCaseMean,rawCaseCI95,'k','LineStyle','none',...
        'LineWidth',0.8,'CapSize',4);
end
set(ax1,'XTick',1:nCases,'XTickLabel',shortCaseLabels,...
    'FontName','Times New Roman','FontSize',8.5,'LineWidth',0.8,...
    'TickDir','out','YLim',[0 max(60,ceil((max(rawCaseMean+rawCaseCI95)+5)/10)*10)],...
    'GridAlpha',0.14,'XTickLabelRotation',18);
ylabel(ax1,'Percentage of network realizations (\%)','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',10);
title(ax1,'(a) Network-wide raw interference cases','FontName','Times New Roman',...
    'FontSize',10,'FontWeight','normal');
for cc=1:nCases
    yy=rawCaseMean(cc)+1.5;
    if showErrorBars, yy=yy+rawCaseCI95(cc); end
    text(ax1,cc,yy,sprintf('%.1f',rawCaseMean(cc)),...
        'HorizontalAlignment','center','FontName','Times New Roman','FontSize',8);
end

ax2=nexttile(t,2);
b2=bar(ax2,1:nCases,conditionalPowerMean,'grouped','BarWidth',0.76);
hold(ax2,'on'); box(ax2,'on'); grid(ax2,'on');
for mm=1:nMethods
    b2(mm).FaceColor=methodColors(mm,:); b2(mm).EdgeColor='none';
end
if showErrorBars
    for mm=1:nMethods
        errorbar(ax2,b2(mm).XEndPoints,conditionalPowerMean(:,mm),...
            conditionalPowerCI95(:,mm),'k','LineStyle','none',...
            'LineWidth',0.75,'CapSize',3);
    end
end
set(ax2,'XTick',1:nCases,'XTickLabel',shortCaseLabels,...
    'FontName','Times New Roman','FontSize',8.5,'LineWidth',0.8,...
    'TickDir','out','YLim',[0 105],'YTick',0:20:100,...
    'GridAlpha',0.14,'XTickLabelRotation',18);
ylabel(ax2,'Power-budget utilization (\%)','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',10);
title(ax2,'(b) Network utilization by raw case',...
    'FontName','Times New Roman','FontSize',10,'FontWeight','normal');
for mm=1:nMethods
    xx=b2(mm).XEndPoints;
    for cc=1:nCases
        val=conditionalPowerMean(cc,mm);
        if isfinite(val)
            yy=val+1.5; if showErrorBars, yy=yy+conditionalPowerCI95(cc,mm); end
            text(ax2,xx(cc),yy,sprintf('%.1f',val),...
                'HorizontalAlignment','center','FontName','Times New Roman','FontSize',7.5);
        end
    end
end
lgd=legend(ax2,methodNames,'Location','southoutside','Orientation','horizontal',...
    'NumColumns',2,'Box','off','FontName','Times New Roman','FontSize',14);
lgd.Layout.Tile='south';

%% -------------------- Export --------------------
exportgraphics(fig,'FourCase_AI_IBPC_IA_RSS_Journal.pdf','ContentType','vector');
exportgraphics(fig,'FourCase_AI_IBPC_IA_RSS_Journal.png','Resolution',600);
writetable(ResultOccurrence,'FourCase_RawOccurrence_AI_IBPC_IA_RSS.csv');
writetable(ResultConditionalPower,'FourCase_PowerUtilization_AI_IBPC_IA_RSS.csv');
save('FourCase_AI_IBPC_IA_RSS_Journal.mat','rawCaseMean','rawCaseCI95',...
    'dropRawCasePct','conditionalPowerMean','conditionalPowerCI95',...
    'dropConditionalPower','GammaLic_dBm','GammaUn_dBm','NDrops','MC','N');

fprintf('\nSaved journal four-case outputs.\n');

