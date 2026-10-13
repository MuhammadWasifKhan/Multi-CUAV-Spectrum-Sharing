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
