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

