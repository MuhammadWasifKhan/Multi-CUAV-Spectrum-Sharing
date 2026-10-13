function trial=fourcase_kkt_refine_candidate(base,u,...
    HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU)
% Same-UAV per-band capped KKT refinement. Other UAVs are held fixed.
% Protection is imposed through residual aggregate-interference caps inside
% the KKT allocation itself. There is NO safety clipping after this step.
trial=base;
for b=1:2
    if b==1
        band='lic';HS=HL;Hprot=HP;G=GL;Pmax=PL;
        hI=hp;Pi=Ppbs;noise=nL;
        idx=trial(u).idxLic;vs=trial(u).vLic;
    else
        band='un';HS=HU;Hprot=HW;G=GU;Pmax=PU;
        hI=hw;Pi=Pwifi;noise=nU;
        idx=trial(u).idxUn;vs=trial(u).vUn;
    end
    if isempty(idx),continue;end
    a=zeros(size(idx,1),1);
    caps=zeros(size(idx,1),1);
    Iall=aggregate_interference_by_channel(trial,Hprot,G,band);
    for e=1:size(idx,1)
        sc=idx(e,2);
        k=trial(u).users(idx(e,1));
        if b==1
            pcur=trial(u).pLic(e);
        else
            pcur=trial(u).pUn(e);
        end
        leak=abs(Hprot{u,sc}'*vs{e})^2;
        otherLeak=max(0,Iall(sc)-pcur*leak);
        caps(e)=min(Pmax,max(0,G(sc)-otherLeak)/max(leak,realmin));
        den=max(hI(k)*Pi(sc)+noise+...
            inter_uav_power_same_sc(trial,HS,u,k,sc,band),1e-30);
        a(e)=max(abs(HS{u,k}'*vs{e})^2/den,1e-30);
    end
    pNew=kkt_waterfill_capped_original(a,Pmax,caps);
    trial=set_band_power(trial,u,band,pNew);
end
end

