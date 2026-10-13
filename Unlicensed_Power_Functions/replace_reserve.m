function trial=replace_reserve(trial,u,e,sc,band,HS,HP,hI,PI,noise,G,iter)
[idx,p,vs]=band_data(trial,u,band);loc=idx(e,1);k=trial(u).users(loc);
% Leakage-aware RZF initial beam on the new protected channel.
h=HS{u,k};g=HP{u,sc};alpha=1e-3;
v=(g*g'+alpha*eye(numel(h)))\h;v=v/max(norm(v),realmin);
% Iterative surrogate polishing with fixed channel dimensions.
for it=1:max(1,iter)
    newV=(h*h'/max(noise+hI(k)*PI(sc),realmin))*v;
    newV=newV/max(norm(newV),realmin);
    if abs(g'*newV)^2<=G(sc)/max(p(e),1e-15),v=newV;end
end
idx(e,2)=sc;vs{e}=v;
if strcmp(band,'lic'),trial(u).idxLic=idx;trial(u).vLic=vs;
else,trial(u).idxUn=idx;trial(u).vUn=vs;end
end
