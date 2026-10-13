function [trial,delta]=reserved_sc_trial(base,u,e,sc,b,replacement, ...
    HL,HU,HP,HW,GL,GU,PL,PU,rateFn,hp,hw, ...
    Ppbs,Pwifi,nL,nU,mmIter,leakFrac,alpha)
trial=base;delta=0;
if b==1
    idx=base(u).idxLic;p=base(u).pLic(:);v=base(u).vLic;
    HS=HL;Hprot=HP;G=GL;Pmax=PL;hi=hp;Pi=Ppbs;noise=nL;
    band='lic';
else
    idx=base(u).idxUn;p=base(u).pUn(:);v=base(u).vUn;
    HS=HU;Hprot=HW;G=GU;Pmax=PU;hi=hw;Pi=Pwifi;noise=nU;
    band='un';
end
loc=idx(e,1);k=base(u).users(loc);
den=max(hi(k)*Pi(sc)+noise,1e-30);
[vNew,~,~]=mm_unit_beam(HS{u,k},Hprot{u,sc}, ...
    den,G(sc),max(p(e),1e-12),mmIter,leakFrac,alpha);
if replacement
    % Move existing power/stream to a RESERVED channel. No extra bandwidth
    % used by this SU, and no additional power is created.
    idx(e,2)=sc;v{e}=vNew;
else
    % Add a new independent OFDMA stream for the SAME SU, only if its
    % corresponding UAV/band has unused power and the reserved SC is safe.
    remaining=max(0,Pmax-sum(p));
    if remaining<=1e-12,return;end
    idx(end+1,:)=[loc sc];p(end+1,1)=0;v{end+1,1}=vNew;
end
if b==1
    trial(u).idxLic=idx;trial(u).pLic=p;trial(u).vLic=v;
else
    trial(u).idxUn=idx;trial(u).pUn=p;trial(u).vUn=v;
end
% Replacement may alter aggregate protection on a reserved channel.
% Do NOT clip: protection is a feasibility test of the reserved candidate.
[preRatioL,preRatioU]=pure_check(trial,HP,HW,GL,GU,PL,PU);
if preRatioL>1+1e-5 || preRatioU>1+1e-5
    trial=base;
    return;
end
if replacement
    % Optimize extra power only if unused power exists after moving.
    [~,pNow,vNow]=get_band_fields(trial,u,band);
    stream=e;
else
    [~,pNow,vNow]=get_band_fields(trial,u,band);
    stream=numel(pNow);
end
remaining=max(0,Pmax-sum(pNow));
I=aggregate_interference_by_channel(trial,Hprot,G,band);
leak=abs(Hprot{u,sc}'*vNow{stream})^2;
head=max(0,G(sc)-I(sc));
if leak>1e-30,upper=min(remaining,head/leak);
else,upper=remaining;end
if upper>1e-12
    % Same scalar optimization for generic and case-triggered arms.
    obj=@(x) -rateFn(reserved_add_power(trial,u,stream,b,x));
    [x,~]=fminbnd(obj,0,upper,optimset('TolX',1e-9,'Display','off'));
    choices=[0 x upper];
    baseRate=rateFn(trial);bestExtra=0;
    for ii=1:numel(choices)
        candidate=reserved_add_power(trial,u,stream,b,choices(ii));
        r=rateFn(candidate);
        if isfinite(r)&&r>baseRate+1e-5
            baseRate=r;bestExtra=choices(ii);
        end
    end
    trial=reserved_add_power(trial,u,stream,b,bestExtra);
    delta=bestExtra;
end
% Do not retain a zero-power added stream. This also prevents an empty
% reservation from being counted as an accepted channel activation.
if ~replacement && delta<=1e-12
    trial=base;return;
end
% All accepted changes must satisfy aggregate interference and budgets.
[ratioL,ratioU]=pure_check(trial,HP,HW,GL,GU,PL,PU);
assert(ratioL<=1+1e-5&&ratioU<=1+1e-5,'Reserved trial infeasible');
end
