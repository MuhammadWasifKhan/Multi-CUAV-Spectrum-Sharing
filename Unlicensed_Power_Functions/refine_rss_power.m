function trial=refine_rss_power(trial,u,HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,PP,PW,nL,nU)
% Other UAV streams fixed; KKT with residual aggregate interference caps.
for b=1:2
    if b==1,band='lic';HS=HL;HI=HP;G=GL;Pmax=PL;hI=hp;PI=PP;noise=nL;
    else,band='un';HS=HU;HI=HW;G=GU;Pmax=PU;hI=hw;PI=PW;noise=nU;end
    [idx,p,vs]=band_data(trial,u,band);
    if isempty(p),continue;end
    I=leakage(trial,HI,G,band);a=zeros(numel(p),1);cap=a;
    for e=1:numel(p)
        sc=idx(e,2);k=trial(u).users(idx(e,1));
        le=abs(HI{u,sc}'*vs{e})^2;
        other=max(0,I(sc)-p(e)*le);
        cap(e)=min(Pmax,max(0,G(sc)-other)/max(le,realmin));
        den=noise+hI(k)*PI(sc)+other_interference(trial,HS,u,k,sc,band);
        a(e)=abs(HS{u,k}'*vs{e})^2/max(den,realmin);
    end
    trial=set_p(trial,u,band,waterfill(a,Pmax,cap));
end
end
