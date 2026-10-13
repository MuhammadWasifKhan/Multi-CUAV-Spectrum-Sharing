function [tx,added] = increase_su_stream_if_safe(tx,u,ell,Hprot,Gamma,Pmax,band)
[idx,p,v] = get_band_fields(tx,u,band);
s=idx(ell,2);
I=aggregate_interference_by_channel(tx,Hprot,Gamma,band);
g=abs(Hprot{u,s}'*v{ell})^2;
unusedBudget=max(0,Pmax-sum(p));
headroom=max(0,Gamma(s)-I(s));
% An already unsafe shared channel cannot receive compensating power.
if I(s)>Gamma(s)*(1+1e-9), added=0; return; end
if g<=1e-30
    added=unusedBudget;
else
    added=min(unusedBudget,headroom/max(g,1e-30));
end
added=max(0,added);
p(ell)=p(ell)+added;
tx=set_band_power(tx,u,band,p);
end

