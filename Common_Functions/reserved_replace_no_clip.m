function trial=reserved_replace_no_clip(base,u,e,sc,b,...
    HL,HU,HP,HW,GL,GU,hp,hw,Ppbs,Pwifi,nL,nU,...
    mmIter,leakFrac,alpha)
% Replace one existing stream by a reserved subchannel while preserving its
% current power. Recompute its leakage-aware beam on the new subchannel.
% NO clipping or safety scaling is performed here.
trial=base;
if b==1
    idx=trial(u).idxLic; p=trial(u).pLic(:); v=trial(u).vLic;
    HS=HL; Hprot=HP; G=GL; hi=hp; Pi=Ppbs; noise=nL;
else
    idx=trial(u).idxUn; p=trial(u).pUn(:); v=trial(u).vUn;
    HS=HU; Hprot=HW; G=GU; hi=hw; Pi=Pwifi; noise=nU;
end
loc=idx(e,1);
k=trial(u).users(loc);
den=max(hi(k)*Pi(sc)+noise,1e-30);
[vNew,~,~]=mm_unit_beam(HS{u,k},Hprot{u,sc},...
    den,G(sc),max(p(e),1e-12),mmIter,leakFrac,alpha);
idx(e,2)=sc;
v{e}=vNew;
if b==1
    trial(u).idxLic=idx;
    trial(u).vLic=v;
else
    trial(u).idxUn=idx;
    trial(u).vUn=v;
end
end

