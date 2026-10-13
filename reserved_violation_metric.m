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

