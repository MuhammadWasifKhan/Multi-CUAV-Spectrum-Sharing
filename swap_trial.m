function [trial,delta]=swap_trial(base,u,e,partner,b, ...
        HL,HU,HP,HW,GL,GU,PL,PU,rateFn, ...
        hp,hw,Ppbs,Pwifi,nL,nU,BL,BU,MM_iter,leakFrac,alpha)
trial=base;delta=0;
if b==1
    idx=trial(u).idxLic; p=trial(u).pLic;
    Hs=HL;Hprot=HP;G=GL;Pmax=PL;vList=trial(u).vLic;
    denomInterf=hp;Pint=Ppbs;noise=nL;
else
    idx=trial(u).idxUn;p=trial(u).pUn;
    Hs=HU;Hprot=HW;G=GU;Pmax=PU;vList=trial(u).vUn;
    denomInterf=hw;Pint=Pwifi;noise=nU;
end
% Swap subchannel labels while preserving each stream's SU and its power.
% Recalculate BOTH affected users' leakage-aware beams for their NEW SCs.
scA=idx(e,2);scB=idx(partner,2);
idx(e,2)=scB;idx(partner,2)=scA;
for ell=[e partner]
    loc=idx(ell,1);globalK=trial(u).users(loc);sc=idx(ell,2);
    den=max(denomInterf(globalK)*Pint(sc)+noise,1e-30);
    [v,~,~]=mm_unit_beam(Hs{u,globalK},Hprot{u,sc}, ...
        den,G(sc),max(p(ell),1e-12),MM_iter,leakFrac,alpha);
    vList{ell}=v;
end
if b==1
    trial(u).idxLic=idx;trial(u).vLic=vList;
else
    trial(u).idxUn=idx;trial(u).vUn=vList;
end
% Protect all shared channels after changing beams/subchannel occupancy.
trial=apply_final_safety_clipping(trial,HP,HW,GL,GU,PL,PU,'both');
% SAME-SU compensation only on the newly selected subchannel, using
% leftover power of the same band and aggregate interference headroom.
% No donor-user power changes and no cross-band budget transfer.
if b==1
    band='lic';G=GL;Hprot=HP;
else
    band='un';G=GU;Hprot=HW;
end
[~,pNow,vNow]=get_band_fields(trial,u,band);
remaining=max(0,Pmax-sum(pNow));
sc=idx(e,2);leak=abs(Hprot{u,sc}'*vNow{e})^2;
I=aggregate_interference_by_channel(trial,Hprot,G,band);
headroom=max(0,(G(sc)-I(sc))/max(leak,realmin));
upper=min(remaining,headroom);
if upper>1e-10
    originalRate=rateFn(trial);
    obj=@(extra) -rateFn(swap_add_stream_power(trial,u,e,b,extra));
    [x,~]=fminbnd(obj,0,upper,optimset('TolX',1e-9,'Display','off'));
    % Also evaluate the boundary; fminbnd need not find global optimum.
    xs=[0 x upper];
    baseComp=trial;
    for t=1:numel(xs)
        cand=swap_add_stream_power(baseComp,u,e,b,xs(t));
        r=rateFn(cand);
        if r>originalRate+1e-5
            trial=cand;originalRate=r;delta=xs(t);
        end
    end
end
end

