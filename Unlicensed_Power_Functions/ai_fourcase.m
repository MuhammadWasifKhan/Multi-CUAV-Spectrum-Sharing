function tx=ai_fourcase(raw,HP,HW,GL,GU,PL,PU,rateFn)
IL=leakage(raw,HP,GL,'lic');IU=leakage(raw,HW,GU,'un');
hL=any(IL>GL*(1+1e-9));hU=any(IU>GU*(1+1e-9));tx=raw;
if hL,tx=protect_band(tx,HP,GL,'lic',rateFn);end
if hU,tx=protect_band(tx,HW,GU,'un',rateFn);end
if hL && ~hU,tx=increase_band(tx,HW,GU,PU,'un',rateFn);end
if hU && ~hL,tx=increase_band(tx,HP,GL,PL,'lic',rateFn);end
assert(feasible(tx,HP,HW,GL,GU,PL,PU),'AI-IBPC infeasible');
end
