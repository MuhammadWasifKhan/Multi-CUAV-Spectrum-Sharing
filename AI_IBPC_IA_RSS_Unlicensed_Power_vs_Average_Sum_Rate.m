clc; clear; close all;
% Modularized from the original repository: function bodies unchanged.
projectRoot=fileparts(fileparts(mfilename('fullpath')));
overrideDir=fullfile(projectRoot,'Convergence_Overrides');
if contains([path pathsep],[overrideDir pathsep]),rmpath(overrideDir);end
addpath(fullfile(projectRoot,'Unlicensed_Power_Functions'),'-begin');
% Proposed I: AI-IBPC; Proposed II: IA-RSS (reserved-subchannel selection).
% Standalone MATLAB R2016b+ script. CVX 2.2 required for trajectory SCA.
% Both algorithms use identical geometry, random seeds, initial paths and
% resource-allocation routines; trajectories are optimized independently.
% Run a small test with NDrops=1, MC=2 before the production run.
area=1000; H=100; U=3; K=6; J0=6; J=10; M=10; Nt=8;
BL=180e3; BU=180e3; N0=-169;
nL=10^((N0+10*log10(BL)-30)/10); nU=10^((N0+10*log10(BU)-30)/10);
PL=0.5; PUlist=0.5:0.5:5.5; PP=2.5*ones(J,1); PW=ones(M,1);
fl=2e9; fu=5e9; c=3e8;
A=struct('a',9.61,'b',0.16,'etaL',1,'etaN',20,'sigL',4,'sigN',6);
GL=10^((-80-30)/10)*ones(J,1); GU=10^((-80-30)/10)*ones(M,1);
bs=[500,500]; ap=[500,700]; starts=[120,850;260,180;430,620];
T=220; dt=5; N=round(T/dt)+1; step=25; Dmin=100;
AO=3; trajIter=5; mmIter=10; NDrops=1; MC=20;
seedBase=9000; active=1:J0; reserve=J0+1:J;
R1=zeros(NDrops,numel(PUlist)); R2=R1; Qchange=zeros(NDrops,numel(PUlist),2);
assert(exist('cvx_begin','file')==2,'Install and initialize CVX 2.2 before running.');
for drop=1:NDrops
    rng(seedBase+drop,'twister');
    W=area*rand(K,2); P=area*rand(J,2); V=area*rand(M,2);
    sh=struct('su',randn(K,1),'pu',randn(J,1),'wu',randn(M,1), ...
        'pbs',8*randn(K,1),'wifi',6*randn(K,1));
    hover=zeros(U,2);
    for u=1:U
        ids=[];
        for k=1:K
            [~,closest]=min(sum((starts-W(k,:)).^2,2));
            if closest==u,ids(end+1)=k;end %#ok<AGROW>
        end
        if isempty(ids),hover(u,:)=starts(u,:);
        else,hover(u,:)=mean(W(ids,:),1);end
    end
    Q0=initial_path(starts,hover,N,step,area,Dmin);
    for t=1:numel(PUlist)
        PU=PUlist(t);
        seed=77777+10000*drop+1000*t;
        % Identical initial trajectory, distinct optimized trajectories.
        Q1=trajectory_ao(Q0,'AI',W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
            PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO, ...
            active,reserve,starts,step,Dmin,area,trajIter,seed);
        Q2=trajectory_ao(Q0,'RSS',W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
            PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO, ...
            active,reserve,starts,step,Dmin,area,trajIter,seed);
        Qchange(drop,t,1)=norm(Q1(:)-Q0(:));
        Qchange(drop,t,2)=norm(Q2(:)-Q0(:));
        rates=zeros(MC,N,2);
        for mc=1:MC
            for n=1:N
                s=12345+100000*drop+1000*mc+n;
                rates(mc,n,1)=slot_solver(squeeze(Q1(:,n,:)),'AI',W,P,V,sh,bs,ap, ...
                    H,fl,fu,c,A,PP,PW,nL,nU,GL,GU,PL,PU,BL,BU, ...
                    Nt,mmIter,AO,active,reserve,s);
                rates(mc,n,2)=slot_solver(squeeze(Q2(:,n,:)),'RSS',W,P,V,sh,bs,ap, ...
                    H,fl,fu,c,A,PP,PW,nL,nU,GL,GU,PL,PU,BL,BU, ...
                    Nt,mmIter,AO,active,reserve,s);
            end
        end
        R1(drop,t)=mean(reshape(rates(:,:,1),[],1))/1e6;
        R2(drop,t)=mean(reshape(rates(:,:,2),[],1))/1e6;
        fprintf('Drop %2d/%d | PU %.1f W | AI-IBPC %.5f | IA-RSS %.5f Mbps | dQ %.2f / %.2f m\n', ...
            drop,NDrops,PU,R1(drop,t),R2(drop,t),Qchange(drop,t,1),Qchange(drop,t,2));
        save('Proposed_I_II_Checkpoint.mat','R1','R2','Qchange','PUlist','drop','t','NDrops','MC');
    end
end
Results=table(PUlist(:),mean(R1,1)',mean(R2,1)', ...
    'VariableNames',{'Pmax_un_W','AI_IBPC_Mbps','IA_RSS_Mbps'});
disp(Results);
writetable(Results,'Proposed_I_II_UnlicensedPower_Comparison.csv');
save('Proposed_I_II_UnlicensedPower_Comparison.mat','Results','R1','R2','Qchange','PUlist','NDrops','MC');
figure('Color','w','Position',[100 100 620 470]); hold on;
plot(PUlist,Results.AI_IBPC_Mbps,'-o','LineWidth',2,'MarkerSize',8);
plot(PUlist,Results.IA_RSS_Mbps,'-d','LineWidth',2,'MarkerSize',8);
ax=gca; set(ax,'FontName','Times New Roman','FontSize',16,'LineWidth',1, ...
    'Position',[0.13 0.13 0.84 0.72],'GridLineStyle',':','GridAlpha',0.25);
grid on; box on; xticks(PUlist); xlim([PUlist(1),PUlist(end)]);
xlabel('Maximum unlicensed power (W)'); ylabel('Average sum rate (Mbps)');
legend({'Proposed I (AI-IBPC)','Proposed II (IA-RSS)'}, ...
    'Location','northoutside','Orientation','horizontal', ...
    'FontName','Times New Roman','FontSize',11);
exportgraphics(gcf,'Proposed_I_II_UnlicensedPower_Comparison.png','Resolution',300);

