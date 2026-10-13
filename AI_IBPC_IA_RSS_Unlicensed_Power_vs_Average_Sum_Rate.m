clc; clear; close all;
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

function Q=initial_path(starts,hover,N,step,area,Dmin)
U=size(starts,1); Q=zeros(U,N,2);
for u=1:U
    a=starts(u,:); d=hover(u,:)-a;
    maxOut=0.45*(N-1)*step; d=d*min(1,maxOut/max(norm(d),1e-12));
    for n=1:N
        v=(n-1)/(N-1); scale=min([1,2*v,2*(1-v)]);
        Q(u,n,:)=a+scale*d;
    end
end
Q=repair_path(Q,starts,step,Dmin,area);
end

function Q=repair_path(target,starts,step,Dmin,area)
[U,N,~]=size(target); Q=zeros(size(target));
for u=1:U
    for n=1:N,Q(u,n,:)=starts(u,:);end
end
if ~path_ok(Q,starts,step,Dmin,area)
    error('The specified stationary starting positions violate UAV separation.');
end
Xraw=reshape(permute(target,[2 3 1]),N,2*U);
X0=reshape(permute(Q,[2 3 1]),N,2*U);
try
    cvx_begin quiet
        variable X(N,2*U)
        minimize(sum_square(reshape(X-Xraw,[],1)))
        subject to
            X(:)>=0; X(:)<=area;
            for u=1:U
                cc=2*u-1:2*u;
                X(1,cc)==starts(u,:);X(N,cc)==starts(u,:);
                for n=1:N-1
                    norm(X(n+1,cc)-X(n,cc),2)<=step;
                end
            end
            for n=1:N
                for u=1:U-1
                    for v=u+1:U
                        cu=2*u-1:2*u;cv=2*v-1:2*v;
                        d=X0(n,cu)-X0(n,cv);
                        2*d*(X(n,cu)-X(n,cv)).'-sum(d.^2)>=Dmin^2;
                    end
                end
            end
    cvx_end
    if contains(lower(cvx_status),'solved')
        proposed=permute(reshape(X,N,2,U),[3 1 2]);
        if path_ok(proposed,starts,step,Dmin,area),Q=proposed;end
    end
catch ME
    warning('Initial trajectory repair: %s',ME.message);
    try,cvx_clear;catch,end
end
end

function ok=path_ok(Q,starts,step,Dmin,area)
[U,N,~]=size(Q); tol=1e-3;ok=all(isfinite(Q(:))) && all(Q(:)>=-tol) && all(Q(:)<=area+tol);
for u=1:U
    p=squeeze(Q(u,:,:));
    ok=ok && norm(p(1,:)-starts(u,:))<tol && norm(p(end,:)-starts(u,:))<tol;
    ok=ok && all(sqrt(sum(diff(p).^2,2))<=step+tol);
end
for n=1:N
    for u=1:U-1
        for v=u+1:U
            ok=ok && norm(squeeze(Q(u,n,:)-Q(v,n,:)))>=Dmin-tol;
        end
    end
end
end

function Q=trajectory_ao(Q,mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
    PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO, ...
    active,reserve,starts,step,Dmin,area,maxIter,seed)
% Joint all-waypoint CVX trust-region SCA; common random numbers for trials.
[U,N,~]=size(Q);
f=@(Z)trajectory_rate(Z,mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
    PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed);
current=f(Q);
for it=1:maxIter
    grad=zeros(U,N,2); epsFD=1;
    % Local finite-difference surrogate of REOPTIMIZED sum-rate.
    % This replaces unused analytical leakage derivatives from the old file.
    for n=2:N-1
        for u=1:U
            for xy=1:2
                qa=Q; qb=Q;
                qa(u,n,xy)=qa(u,n,xy)+epsFD;
                qb(u,n,xy)=qb(u,n,xy)-epsFD;
                sa=slot_solver(squeeze(qa(:,n,:)),mode,W,P,V,sh,bs,ap, ...
                    H,fl,fu,c,A,PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed+n);
                sb=slot_solver(squeeze(qb(:,n,:)),mode,W,P,V,sh,bs,ap, ...
                    H,fl,fu,c,A,PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed+n);
                grad(u,n,xy)=(sa-sb)/(2*epsFD);
            end
        end
    end
    accepted=false; radius=7.5;
    for bt=1:3
        [candidate,solved]=sca_path(Q,grad,starts,step,Dmin,area,radius);
        if solved && path_ok(candidate,starts,step,Dmin,area)
            trial=f(candidate);
            if isfinite(trial) && trial>=current-1e-8*max(1,abs(current))
                if abs(trial-current)<=1e-5*max(1,abs(current))
                    return;
                end
                Q=candidate;current=trial;accepted=true;break;
            end
        end
        radius=radius/2;
    end
    if ~accepted,break;end
end
end

function [Qout,ok]=sca_path(Q,grad,starts,step,Dmin,area,radius)
[U,N,~]=size(Q);X0=reshape(permute(Q,[2 3 1]),N,2*U);
G=reshape(permute(grad,[2 3 1]),N,2*U);scale=max(1,max(abs(G(:))));
Qout=Q;ok=false;
try
    cvx_begin quiet
        variable X(N,2*U)
        maximize(sum(sum((G/scale).*(X-X0))) - ...
            1e-4/max(radius^2,1e-9)*sum_square(reshape(X-X0,[],1)))
        subject to
            X(:)>=0;X(:)<=area;
            for u=1:U
                cc=2*u-1:2*u;
                X(1,cc)==starts(u,:);X(N,cc)==starts(u,:);
                for n=1:N-1
                    norm(X(n+1,cc)-X(n,cc),2)<=step;
                end
                for n=2:N-1
                    norm(X(n,cc)-X0(n,cc),2)<=radius;
                end
            end
            for n=1:N
                for u=1:U-1
                    for v=u+1:U
                        cu=2*u-1:2*u;cv=2*v-1:2*v;
                        d=X0(n,cu)-X0(n,cv);
                        2*d*(X(n,cu)-X(n,cv)).'-sum(d.^2)>=Dmin^2;
                    end
                end
            end
    cvx_end
    ok=contains(lower(cvx_status),'solved') && all(isfinite(X(:)));
    if ok,Qout=permute(reshape(X,N,2,U),[3 1 2]);end
catch ME
    warning('Trajectory SCA: %s',ME.message);
    try,cvx_clear;catch,end
end
end

function r=trajectory_rate(Q,mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
    PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed)
N=size(Q,2);r=0;
for n=1:N
    r=r+slot_solver(squeeze(Q(:,n,:)),mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
        PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed+n)/N;
end
end

function r=slot_solver(Q,mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
    PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed)
[HL,HU,HP,HW,hp,hw]=make_channels(Q,W,P,V,sh,bs,ap,H,fl,fu,c,A,Nt,seed);
U=size(Q,1);K=size(W,1);
score=zeros(U,K);
for u=1:U
    for k=1:K,score(u,k)=norm(HL{u,k})^2;end
end
[~,assoc]=max(score,[],1);
tx=repmat(struct('users',[],'idxLic',zeros(0,2),'pLic',[], ...
    'vLic',{{}},'idxUn',zeros(0,2),'pUn',[],'vUn',{{}}),U,1);
for u=1:U,tx(u).users=find(assoc==u);end
best=tx;bestRate=0;previous=tx;
for it=1:max(1,AO)
    tx=previous;
    for u=1:U
        us=tx(u).users;
        if isempty(us),continue;end
        for b=1:2
            if b==1,HS=HL;HI=HP;hh=hp;pi=PP;noise=nL;power=PL;band='lic';bw=BL;
            else,HS=HU;HI=HW;hh=hw;pi=PW;noise=nU;power=PU;band='un';bw=BU;end
            C=zeros(numel(us),numel(active));
            for k=1:numel(us)
                for s=1:numel(active)
                    sc=active(s);v=mrt(HS{u,us(k)});
                    den=noise+hh(us(k))*pi(sc)+other_interference(previous,HS,u,us(k),sc,band);
                    C(k,s)=bw*log2(1+(power/max(numel(us),1))*abs(HS{u,us(k)}'*v)^2/max(den,realmin));
                end
            end
            pairs=hungarian_max(C);idx=zeros(size(pairs,1),2); beams=cell(size(pairs,1),1);g=zeros(size(pairs,1),1);
            for e=1:size(pairs,1)
                k=pairs(e,1);sc=active(pairs(e,2));idx(e,:)=[k,sc];
                % Initial unprotected MM surrogate converges to MRT.
                beams{e}=mm_unprotected(HS{u,us(k)},mmIter);
                den=noise+hh(us(k))*pi(sc)+other_interference(previous,HS,u,us(k),sc,band);
                g(e)=abs(HS{u,us(k)}'*beams{e})^2/max(den,realmin);
            end
            pow=waterfill(g,power,power*ones(numel(g),1));
            if b==1,tx(u).idxLic=idx;tx(u).vLic=beams;tx(u).pLic=pow;
            else,tx(u).idxUn=idx;tx(u).vUn=beams;tx(u).pUn=pow;end
        end
    end
    rateFn=@(z)network_rate(z,HL,HU,hp,hw,PP,PW,nL,nU,BL,BU);
    if strcmp(mode,'AI')
        controlled=ai_fourcase(tx,HP,HW,GL,GU,PL,PU,rateFn);
    else
        controlled=rss_fourcase(tx,HL,HU,HP,HW,GL,GU,PL,PU, ...
            hp,hw,PP,PW,nL,nU,reserve,mmIter,rateFn);
    end
    rNew=rateFn(controlled);
    if ~isfinite(rNew) || rNew<bestRate-1e-8*max(1,bestRate),break;end
    best=controlled;previous=controlled;bestRate=rNew;
end
r=bestRate;
end

function [HL,HU,HP,HW,hp,hw]=make_channels(Q,W,P,V,sh,bs,ap,H,fl,fu,c,A,Nt,seed)
U=size(Q,1);K=size(W,1);J=size(P,1);M=size(V,1);
HL=cell(U,K);HU=cell(U,K);HP=cell(U,J);HW=cell(U,M);
rng(seed+555,'twister');hp=zeros(K,1);hw=zeros(K,1);
for k=1:K
    d=max(norm(W(k,:)-bs),1);pl=20*log10(4*pi*fl/c)+37*log10(d)+sh.pbs(k);
    hp(k)=10^(-pl/10)*abs((randn+1i*randn)/sqrt(2))^2;
    d=max(norm(W(k,:)-ap),1);pl=20*log10(4*pi*fu/c)+30*log10(d)+sh.wifi(k);
    hw(k)=10^(-pl/10)*abs((randn+1i*randn)/sqrt(2))^2;
end
for u=1:U
    rng(seed+10000*u,'twister');q=Q(u,:);
    for k=1:K
        HL{u,k}=channel(gain(q,W(k,:),H,fl,c,A,sh.su(k)),Nt);
        HU{u,k}=channel(gain(q,W(k,:),H,fu,c,A,sh.su(k)),Nt);
    end
    for j=1:J,HP{u,j}=channel(gain(q,P(j,:),H,fl,c,A,sh.pu(j)),Nt);end
    for m=1:M,HW{u,m}=channel(gain(q,V(m,:),H,fu,c,A,sh.wu(m)),Nt);end
end
end

function g=gain(q,w,H,f,c,A,z)
r=norm(q-w);d=sqrt(r^2+H^2);theta=atan2d(H,max(r,1e-9));
p=1/(1+A.a*exp(-A.b*(theta-A.a)));
loss=20*log10(4*pi*f*d/c)+p*A.etaL+(1-p)*A.etaN;
loss=loss+sqrt(p*A.sigL^2+(1-p)*A.sigN^2)*z;
g=10^(-loss/10);
end
function h=channel(g,Nt)
h=sqrt(g/2)*(randn(Nt,1)+1i*randn(Nt,1));
end
function v=mrt(h)
v=h/max(norm(h),realmin);
end
function v=mm_unprotected(h,iter)
v=ones(size(h))/sqrt(numel(h));B=h*h';
for k=1:max(1,iter)
    vNext=B*v; vNext=vNext/max(norm(vNext),realmin);
    if norm(vNext-v)<1e-8,v=vNext;break;end
    v=vNext;
end
end

function pairs=hungarian_max(score)
% Exact rectangular assignment by dynamic programming; K<=6, S<=10.
[R,S]=size(score);pairs=zeros(0,2);
if R==0||S==0,return;end
if R>S,error('More users than active subchannels on one UAV');end
cost=-score;numStates=2^S;D=inf(R+1,numStates);D(1,1)=0;
prev=zeros(R+1,numStates);choose=zeros(R+1,numStates);
for k=1:R
    for mask=0:numStates-1
        if ~isfinite(D(k,mask+1)),continue;end
        for j=1:S
            if bitget(mask,j),continue;end
            newMask=bitset(mask,j);
            x=D(k,mask+1)+cost(k,j);
            if x<D(k+1,newMask+1)
                D(k+1,newMask+1)=x;prev(k+1,newMask+1)=mask;
                choose(k+1,newMask+1)=j;
            end
        end
    end
end
[~,mask]=min(D(R+1,:));mask=mask-1;pairs=zeros(R,2);
for k=R:-1:1
    j=choose(k+1,mask+1);pairs(k,:)=[k,j];
    mask=prev(k+1,mask+1);
end
end

function p=waterfill(g,Pmax,caps)
% Correctly bracketed KKT solution for sum log2(1+g_i*p_i).
g=max(g(:),0);caps=max(caps(:),0);p=zeros(size(g));
if isempty(g)||Pmax<=0,return;end
if sum(caps)<=Pmax,p=caps;return;end
if ~any(g>0),return;end
hi=max(1,max(1./max(g,realmin)))+Pmax;lo=0;
for it=1:100
    mu=(lo+hi)/2;p=min(caps,max(0,mu-1./max(g,realmin)));
    if sum(p)>Pmax,hi=mu;else,lo=mu;end
end
p=min(caps,max(0,lo-1./max(g,realmin)));
if sum(p)>Pmax,p=p*(Pmax/sum(p));end
end

function [idx,p,vs]=band_data(tx,u,band)
if strcmp(band,'lic'),idx=tx(u).idxLic;p=tx(u).pLic(:);vs=tx(u).vLic;
else,idx=tx(u).idxUn;p=tx(u).pUn(:);vs=tx(u).vUn;end
end
function tx=set_p(tx,u,band,p)
if strcmp(band,'lic'),tx(u).pLic=p(:);else,tx(u).pUn=p(:);end
end
function I=leakage(tx,H,G,band)
I=zeros(numel(G),1);
for u=1:numel(tx)
    [idx,p,vs]=band_data(tx,u,band);
    for e=1:numel(p)
        sc=idx(e,2);I(sc)=I(sc)+p(e)*abs(H{u,sc}'*vs{e})^2;
    end
end
end
function v=other_interference(tx,H,u,k,sc,band)
v=0;
for t=1:numel(tx)
    if t==u,continue;end
    [idx,p,vs]=band_data(tx,t,band);
    for e=1:numel(p)
        if idx(e,2)==sc,v=v+p(e)*abs(H{t,k}'*vs{e})^2;end
    end
end
end
function r=network_rate(tx,HL,HU,hp,hw,PP,PW,nL,nU,BL,BU)
r=0;
for u=1:numel(tx)
    for b=1:2
        if b==1,H=HL;hh=hp;Pi=PP;noise=nL;B=BL;band='lic';
        else,H=HU;hh=hw;Pi=PW;noise=nU;B=BU;band='un';end
        [idx,p,vs]=band_data(tx,u,band);
        for e=1:numel(p)
            k=tx(u).users(idx(e,1));sc=idx(e,2);
            sig=p(e)*abs(H{u,k}'*vs{e})^2;
            den=noise+hh(k)*Pi(sc)+other_interference(tx,H,u,k,sc,band);
            r=r+B*log2(1+sig/max(den,realmin));
        end
    end
end
end
function ok=feasible(tx,HP,HW,GL,GU,PL,PU)
ok=all(leakage(tx,HP,GL,'lic')<=GL*(1+1e-6)) && ...
    all(leakage(tx,HW,GU,'un')<=GU*(1+1e-6));
for u=1:numel(tx)
    ok=ok && all(tx(u).pLic>=-1e-12) && all(tx(u).pUn>=-1e-12) && ...
        sum(tx(u).pLic)<=PL*(1+1e-6) && sum(tx(u).pUn)<=PU*(1+1e-6);
end
end

function tx=ai_fourcase(raw,HP,HW,GL,GU,PL,PU,rateFn)
IL=leakage(raw,HP,GL,'lic');IU=leakage(raw,HW,GU,'un');
hL=any(IL>GL*(1+1e-9));hU=any(IU>GU*(1+1e-9));tx=raw;
if hL,tx=protect_band(tx,HP,GL,'lic',rateFn);end
if hU,tx=protect_band(tx,HW,GU,'un',rateFn);end
if hL && ~hU,tx=increase_band(tx,HW,GU,PU,'un',rateFn);end
if hU && ~hL,tx=increase_band(tx,HP,GL,PL,'lic',rateFn);end
assert(feasible(tx,HP,HW,GL,GU,PL,PU),'AI-IBPC infeasible');
end
function tx=protect_band(tx,H,G,band,rateFn)
% Rate-aware candidate power reductions on each violating protected channel.
for sc=1:numel(G)
    I=leakage(tx,H,G,band);
    if I(sc)<=G(sc)*(1+1e-10),continue;end
    entries=zeros(0,3);
    for u=1:numel(tx)
        [idx,p,vs]=band_data(tx,u,band);
        for e=1:numel(p)
            if idx(e,2)==sc && p(e)>0
                entries(end+1,:)=[u,e,abs(H{u,sc}'*vs{e})^2]; %#ok<AGROW>
            end
        end
    end
    best=tx;scale=G(sc)/max(I(sc),realmin);
    for z=1:size(entries,1)
        u=entries(z,1);e=entries(z,2);[~,p]=band_data(best,u,band);
        p(e)=p(e)*scale;best=set_p(best,u,band,p);
    end
    bestR=rateFn(best);
    for first=1:size(entries,1)
        trial=tx;order=[first,setdiff(1:size(entries,1),first,'stable')];
        for z=order
            I=leakage(trial,H,G,band);
            if I(sc)<=G(sc)*(1+1e-10),break;end
            u=entries(z,1);e=entries(z,2);g=entries(z,3);
            [~,p]=band_data(trial,u,band);
            p(e)=max(0,p(e)-min(p(e),(I(sc)-G(sc))/max(g,realmin)));
            trial=set_p(trial,u,band,p);
        end
        if leakage(trial,H,G,band)<=G*(1+1e-7)
            r=rateFn(trial);
            if r>bestR,best=trial;bestR=r;end
        end
    end
    tx=best;
end
end
function tx=increase_band(tx,H,G,Pmax,band,rateFn)
for pass=1:2
    changed=false;
    for u=1:numel(tx)
        [idx,p,vs]=band_data(tx,u,band);
        for e=1:numel(p)
            sc=idx(e,2);I=leakage(tx,H,G,band);g=abs(H{u,sc}'*vs{e})^2;
            extra=min(max(0,Pmax-sum(p)),max(0,G(sc)-I(sc))/max(g,realmin));
            if extra<=1e-14,continue;end
            base=rateFn(tx);bestP=p(e);
            for x=[0,0.25,0.5,0.75,1]*extra
                cand=tx;pp=p;pp(e)=pp(e)+x;cand=set_p(cand,u,band,pp);
                r=rateFn(cand);
                if r>base+1e-7,base=r;bestP=pp(e);end
            end
            if bestP>p(e),p(e)=bestP;tx=set_p(tx,u,band,p);changed=true;end
        end
    end
    if ~changed,break;end
end
end

function best=rss_fourcase(raw,HL,HU,HP,HW,GL,GU,PL,PU, ...
    hp,hw,PP,PW,nL,nU,reserve,mmIter,rateFn)
% Four-case reserved-channel search. No post-search clipping.
% If no feasible reserved-only allocation exists, keep feasible shutdown
% fallback, rather than reporting an infeasible allocation as successful.
IL=leakage(raw,HP,GL,'lic');IU=leakage(raw,HW,GU,'un');
best=raw;bestV=violation(best,HP,HW,GL,GU,PL,PU);
maxPass=2*sum(arrayfun(@(z)numel(z.users),raw));
for pass=1:maxPass
    updated=false;
    for u=1:numel(best)
        for loc=1:numel(best(u).users)
            [idxL]=band_data(raw,u,'lic');[idxU]=band_data(raw,u,'un');
            eL=find(idxL(:,1)==loc,1);eU=find(idxU(:,1)==loc,1);
            hL=~isempty(eL) && IL(idxL(eL,2))>GL(idxL(eL,2))*(1+1e-9);
            hU=~isempty(eU) && IU(idxU(eU,2))>GU(idxU(eU,2))*(1+1e-9);
            if ~hL && ~hU,continue;end
            candidateL=[NaN,reserve];candidateU=candidateL;
            chosen=best;chosenV=bestV;chosenR=rateFn(best);
            for a=1:numel(candidateL)
                for b=1:numel(candidateU)
                    scL=candidateL(a);scU=candidateU(b);
                    if isnan(scL)&&isnan(scU),continue;end
                    trial=best;
                    if ~isnan(scL)
                        e=find(trial(u).idxLic(:,1)==loc,1);
                        if isempty(e)||reserved_used(trial,scL,'lic'),continue;end
                        trial=replace_reserve(trial,u,e,scL,'lic',HL,HP, ...
                            hp,PP,nL,GL,mmIter);
                    end
                    if ~isnan(scU)
                        e=find(trial(u).idxUn(:,1)==loc,1);
                        if isempty(e)||reserved_used(trial,scU,'un'),continue;end
                        trial=replace_reserve(trial,u,e,scU,'un',HU,HW, ...
                            hw,PW,nU,GU,mmIter);
                    end
                    trial=refine_rss_power(trial,u,HL,HU,HP,HW,GL,GU,PL,PU, ...
                        hp,hw,PP,PW,nL,nU);
                    v=violation(trial,HP,HW,GL,GU,PL,PU);r=rateFn(trial);
                    if (chosenV>1+1e-6 && v<chosenV-1e-9) || ...
                       (v<=1+1e-6 && chosenV<=1+1e-6 && r>chosenR+1e-7)
                        chosen=trial;chosenV=v;chosenR=r;
                    end
                end
            end
            if chosenV<bestV-1e-9 || ...
                    (chosenV<=1+1e-6 && bestV<=1+1e-6 && chosenR>rateFn(best)+1e-7)
                best=chosen;bestV=chosenV;updated=true;
            end
        end
    end
    if ~updated,break;end
end
if ~feasible(best,HP,HW,GL,GU,PL,PU)
    % Explicit zero-power emergency fallback. This preserves PU/WU safety
    % but means this realization was not solved by reserved replacement.
    warning('IA-RSS: no feasible reserved-only selection, using silent fallback.');
    for u=1:numel(best),best(u).pLic(:)=0;best(u).pUn(:)=0;end
end
assert(feasible(best,HP,HW,GL,GU,PL,PU),'IA-RSS infeasible');
end
function used=reserved_used(tx,sc,band)
used=false;
for u=1:numel(tx)
    [idx]=band_data(tx,u,band);
    if any(idx(:,2)==sc),used=true;return;end
end
end
function trial=replace_reserve(trial,u,e,sc,band,HS,HP,hI,PI,noise,G,iter)
[idx,p,vs]=band_data(trial,u,band);loc=idx(e,1);k=trial(u).users(loc);
% Leakage-aware RZF initial beam on the new protected channel.
h=HS{u,k};g=HP{u,sc};alpha=1e-3;
v=(g*g'+alpha*eye(numel(h)))\h;v=v/max(norm(v),realmin);
% Iterative surrogate polishing with fixed channel dimensions.
for it=1:max(1,iter)
    newV=(h*h'/max(noise+hI(k)*PI(sc),realmin))*v;
    newV=newV/max(norm(newV),realmin);
    if abs(g'*newV)^2<=G(sc)/max(p(e),1e-15),v=newV;end
end
idx(e,2)=sc;vs{e}=v;
if strcmp(band,'lic'),trial(u).idxLic=idx;trial(u).vLic=vs;
else,trial(u).idxUn=idx;trial(u).vUn=vs;end
end
function trial=refine_rss_power(trial,u,HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,PP,PW,nL,nU)
% Other UAV streams fixed; KKT with residual aggregate interference caps.
for b=1:2
    if b==1,band='lic';HS=HL;HI=HP;G=GL;Pmax=PL;hI=hp;PI=PP;noise=nL;
    else,band='un';HS=HU;HI=HW;G=GU;Pmax=PU;hI=hw;PI=PW;noise=nU;end
    [idx,p,vs]=band_data(trial,u,band);
    if isempty(p),continue;end
    I=leakage(trial,HI,G,band);a=zeros(numel(p),1);cap=a;
    for e=1:numel(p)
        sc=idx(e,2);k=trial(u).users(idx(e,1));
        le=abs(HI{u,sc}'*vs{e})^2;
        other=max(0,I(sc)-p(e)*le);
        cap(e)=min(Pmax,max(0,G(sc)-other)/max(le,realmin));
        den=noise+hI(k)*PI(sc)+other_interference(trial,HS,u,k,sc,band);
        a(e)=abs(HS{u,k}'*vs{e})^2/max(den,realmin);
    end
    trial=set_p(trial,u,band,waterfill(a,Pmax,cap));
end
end
function v=violation(tx,HP,HW,GL,GU,PL,PU)
v=max([leakage(tx,HP,GL,'lic')./GL; leakage(tx,HW,GU,'un')./GU]);
for u=1:numel(tx)
    v=max([v,sum(tx(u).pLic)/PL,sum(tx(u).pUn)/PU]);
end
end
