function util=total_power_utilization_pct(tx,PL,PU)
% Total utilized power relative to the two independent per-UAV budgets.
U=numel(tx); vals=zeros(U,1);
for u=1:U
    vals(u)=(sum(tx(u).pLic(:))+sum(tx(u).pUn(:)))/(PL+PU)*100;
end
util=mean(vals);
end
