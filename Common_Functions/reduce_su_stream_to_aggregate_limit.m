function [tx,reduced] = reduce_su_stream_to_aggregate_limit(tx,u,ell,Hprot,Gamma,band)
[idx,p,v] = get_band_fields(tx,u,band);
s = idx(ell,2); g = abs(Hprot{u,s}'*v{ell})^2;
I = aggregate_interference_by_channel(tx,Hprot,Gamma,band);
% Hold all OTHER streams fixed. The SU can only remove its own contribution.
otherI = max(0,I(s)-p(ell)*g);
newP = min(p(ell),max(0,(Gamma(s)-otherI)/max(g,1e-30)));
reduced = max(0,p(ell)-newP);
p(ell)=newP;
tx=set_band_power(tx,u,band,p);
end

