function [best,added]=pure_marginal_compensate(tx,u,ell,Hprot,Gamma,Pmax,band,rateFn)
best=tx;added=0;
[idx,p,v]=get_band_fields(tx,u,band);
if ell>numel(p),return;end
s=idx(ell,2);I=aggregate_interference_by_channel(tx,Hprot,Gamma,band);
g=abs(Hprot{u,s}'*v{ell})^2;
unused=max(0,Pmax-sum(p));
head=max(0,Gamma(s)-I(s));
if I(s)>Gamma(s)*(1+1e-9),return;end
if g>1e-30,cap=min(unused,head/g);else,cap=unused;end
if cap<=1e-12,return;end
% Maximize actual total rate along SAME-SU feasible compensation dimension.
objective=@(dp) pure_negative_rate_step(tx,u,ell,band,p,dp,rateFn);
options=optimset('Display','off','TolX',1e-8);
[dp,~]=fminbnd(objective,0,cap,options);
for x=[dp,cap]
    candidate=tx;pp=p;pp(ell)=pp(ell)+x;
    candidate=set_band_power(candidate,u,band,pp);
    if rateFn(candidate)>rateFn(best)+1e-6
        best=candidate;added=x;
    end
end
end
