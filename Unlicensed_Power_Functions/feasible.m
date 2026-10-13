function ok=feasible(tx,HP,HW,GL,GU,PL,PU)
ok=all(leakage(tx,HP,GL,'lic')<=GL*(1+1e-6)) && ...
    all(leakage(tx,HW,GU,'un')<=GU*(1+1e-6));
for u=1:numel(tx)
    ok=ok && all(tx(u).pLic>=-1e-12) && all(tx(u).pUn>=-1e-12) && ...
        sum(tx(u).pLic)<=PL*(1+1e-6) && sum(tx(u).pUn)<=PU*(1+1e-6);
end
end

