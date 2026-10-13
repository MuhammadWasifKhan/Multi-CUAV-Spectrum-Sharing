function [tx,stats]=rateaware_polish_existing(tx,HP,HW,GL,GU,PL,PU,rateFn)
    % Count all rate objective calls including fminbnd internal trials.
    stats=struct('objectiveCalls',0,'scalarSearches',0,'acceptedUpdates',0);
    bestRate=counted_rate(tx);
    for pass=1:2
        improved=false;
        for u=1:numel(tx)
            for bb=1:2
                if bb==1
                    band='lic';Hprot=HP;Gamma=GL;Pmax=PL;
                else
                    band='un';Hprot=HW;Gamma=GU;Pmax=PU;
                end
                [idx,p,v]=get_band_fields(tx,u,band);
                for e=1:numel(p)
                    sc=idx(e,2);
                    leak=abs(Hprot{u,sc}'*v{e})^2;
                    I=aggregate_interference_by_channel(tx,Hprot,Gamma,band);
                    other=max(0,I(sc)-p(e)*leak);
                    budget=max(0,Pmax-(sum(p)-p(e)));
                    if leak>1e-30
                        protect=max(0,(Gamma(sc)-other)/leak);
                    else
                        protect=inf;
                    end
                    upper=min(budget,protect);
                    if upper<=0,continue;end
                    current=min(max(0,p(e)),upper);
                    fun=@(power)counted_trial(tx,u,band,p,e,power);
                    opts=optimset('TolX',max(1e-11,upper*1e-5),'Display','off');
                    stats.scalarSearches=stats.scalarSearches+1;
                    pStar=fminbnd(@(power)-fun(power),0,upper,opts);
                    tests=unique([0;current;upper;pStar]);
                    scores=zeros(numel(tests),1);
                    for tt=1:numel(tests)
                        scores(tt)=fun(tests(tt));
                    end
                    [candidateRate,k]=max(scores);
                    if candidateRate>bestRate+1e-6
                        p(e)=tests(k);
                        tx=set_band_power(tx,u,band,p);
                        bestRate=candidateRate;
                        stats.acceptedUpdates=stats.acceptedUpdates+1;
                        improved=true;
                    end
                end
            end
        end
        if ~improved,break;end
    end
    function r=counted_rate(t)
        stats.objectiveCalls=stats.objectiveCalls+1;
        r=rateFn(t);
    end
    function r=counted_trial(t,uu,bandName,powers,ee,power)
        stats.objectiveCalls=stats.objectiveCalls+1;
        r=rateaware_trial_rate(t,uu,bandName,powers,ee,power,rateFn);
    end
end

