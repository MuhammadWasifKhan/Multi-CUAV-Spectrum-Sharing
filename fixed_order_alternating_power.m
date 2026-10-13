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

