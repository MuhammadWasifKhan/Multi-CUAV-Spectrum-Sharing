function tx=rate_aware_increase_band_only(tx,HP,Gamma,Pmax,band,rateFn)
% Increase powers ONLY on the safe opposite band and ONLY when the TRUE
% network sum rate improves. Independent per-band budgets are preserved;
% no watts are transferred from the violating band.
for pass=1:2
    improved=false;
    for u=1:numel(tx)
        [idx,p,v]=get_band_fields(tx,u,band);
        for e=1:numel(p)
            sc=idx(e,2);
            I=aggregate_interference_by_channel(tx,HP,Gamma,band);
            leak=abs(HP{u,sc}'*v{e})^2;
            current=p(e);
            budgetLeft=max(0,Pmax-sum(p));
            headroom=max(0,Gamma(sc)-I(sc));
            if leak<=1e-30
                addMax=budgetLeft;
            else
                addMax=min(budgetLeft,headroom/leak);
            end
            if addMax<=1e-14,continue;end
            upper=current+addMax;
            baseRate=rateFn(tx);
            trialP=p;
            fun=@(power)ai_ibpc_trial_rate(tx,u,band,trialP,e,power,rateFn);
            opts=optimset('TolX',max(1e-11,upper*1e-5),'Display','off');
            pStar=fminbnd(@(power)-fun(power),current,upper,opts);
            tests=unique([current;upper;pStar]);
            scores=zeros(numel(tests),1);
            for tt=1:numel(tests)
                scores(tt)=fun(tests(tt));
            end
            [bestRate,kbest]=max(scores);
            if bestRate>baseRate+1e-9
                p(e)=tests(kbest);
                tx=set_band_power(tx,u,band,p);
                improved=true;
            end
        end
    end
    if ~improved,break;end
end

I=aggregate_interference_by_channel(tx,HP,Gamma,band);
assert(all(I(:)<=Gamma(:).*(1+1e-7)),...
    'AI-IBPC opposite-band increase violated aggregate protection.');
for u=1:numel(tx)
    [~,p,~]=get_band_fields(tx,u,band);
    assert(sum(p(:))<=Pmax*(1+1e-7),...
        'AI-IBPC opposite-band increase violated per-UAV power budget.');
end
end

