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


