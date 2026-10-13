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
