function v=violation(tx,HP,HW,GL,GU,PL,PU)
v=max([leakage(tx,HP,GL,'lic')./GL; leakage(tx,HW,GU,'un')./GU]);
for u=1:numel(tx)
    v=max([v,sum(tx(u).pLic)/PL,sum(tx(u).pUn)/PU]);
end
end
