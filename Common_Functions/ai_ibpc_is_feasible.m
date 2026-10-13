function ok=ai_ibpc_is_feasible(tx,HP,HW,GL,GU,PL,PU)
IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
IU=aggregate_interference_by_channel(tx,HW,GU,'un');
ok=all(IL(:)<=GL(:).*(1+1e-7)) && ...
   all(IU(:)<=GU(:).*(1+1e-7));
for u=1:numel(tx)
    ok=ok && sum(tx(u).pLic(:))<=PL*(1+1e-7) && ...
             sum(tx(u).pUn(:))<=PU*(1+1e-7);
end
end

