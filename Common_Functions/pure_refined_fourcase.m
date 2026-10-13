function [tx,st]=pure_refined_fourcase(raw,HP,HW,GL,GU,PL,PU,rateFn,allowComp)
if nargin<9,allowComp=true;end
% Case-specific feasible power reduction. Only affected SU can gain power
% on the OPPOSITE band, and only when that band's budget/headroom permits.
% Case 4 is left unchanged unless another SU's necessary reductions affect it.
% All reductions are selected by evaluating the ACTUAL coupled network rate.
tx=enforce_per_uav_band_budget(raw,PL,'lic');
tx=enforce_per_uav_band_budget(tx,PU,'un');
IL=aggregate_interference_by_channel(tx,HP,GL,'lic');
IU=aggregate_interference_by_channel(tx,HW,GU,'un');
st=struct('cases',zeros(1,4),'compL',0,'compU',0, ...
    'blockedBudget',0,'tested',0,'accepted',0);
% Freeze the original case membership before any control changes.
for u=1:numel(tx)
    for loc=1:numel(tx(u).users)
        [iL,~,~]=get_band_fields(tx,u,'lic');
        [iU,~,~]=get_band_fields(tx,u,'un');
        eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
        highL=~isempty(eL)&&IL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
        highU=~isempty(eU)&&IU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
        if highL&&highU,st.cases(3)=st.cases(3)+1;
        elseif highL,st.cases(1)=st.cases(1)+1;
        elseif highU,st.cases(2)=st.cases(2)+1;
        else,st.cases(4)=st.cases(4)+1;end
    end
end
% Fix overloaded channels. Each trial removes ONLY necessary power;
% enumerate which stream bears reduction, then try same-SU compensation.
% A full aggregate projection is used as the initial feasible fallback.
% To avoid comparing a feasible result to infeasible intermediate points,
% construct complete candidates from RAW and rank their final rates.
% Coordinate descent over the order of violating streams (fixed association).
for pass=1:3
    changed=false;
    for b=1:2
        if b==1,band='lic';other='un';HPb=HP;G=GL;Po=PU;Hother=HW;Go=GU;
        else,band='un';other='lic';HPb=HW;G=GU;Po=PL;Hother=HP;Go=GL;end
        for sc=1:numel(G)
            I=aggregate_interference_by_channel(tx,HPb,G,band);
            if I(sc)<=G(sc)*(1+1e-9),continue;end
            % Include proportional clipping as feasible fallback for this SC.
            fallback=tx;
            scale=G(sc)/max(I(sc),realmin);
            for u=1:numel(tx)
                [idx,p,~]=get_band_fields(fallback,u,band);
                hit=find(idx(:,2)==sc);
                if ~isempty(hit)
                    p(hit)=p(hit)*scale;
                    fallback=set_band_power(fallback,u,band,p);
                end
            end
            best=fallback;bestRate=rateFn(best);
            % Test each offending stream as first reduction target.
            for firstU=1:numel(tx)
                [idx,p,~]=get_band_fields(tx,firstU,band);
                for firstE=find(idx(:,2)==sc).'
                    trial=tx;
                    [trial,dp]=reduce_su_stream_to_aggregate_limit( ...
                        trial,firstU,firstE,HPb,G,band);
                    if dp<=0,continue;end
                    % Complete any residual protection on this SC only.
                    It=aggregate_interference_by_channel(trial,HPb,G,band);
                    if It(sc)>G(sc)*(1+1e-9)
                        remScale=G(sc)/max(It(sc),realmin);
                        for uu=1:numel(trial)
                            [ii,pp,~]=get_band_fields(trial,uu,band);
                            ee=find(ii(:,2)==sc);
                            if ~isempty(ee)
                                pp(ee)=pp(ee)*remScale;
                                trial=set_band_power(trial,uu,band,pp);
                            end
                        end
                    end
                    % Compensation is permitted only for this affected SU
                    % and only when it was initially in case 1 or case 2.
                    loc=idx(firstE,1);
                    [ii,pp,~]=get_band_fields(trial,firstU,other);
                    oppositeE=find(ii(:,1)==loc,1);
                    IrawL=aggregate_interference_by_channel(raw,HP,GL,'lic');
                    IrawU=aggregate_interference_by_channel(raw,HW,GU,'un');
                    [rawL,~,~]=get_band_fields(raw,firstU,'lic');
                    [rawU,~,~]=get_band_fields(raw,firstU,'un');
                    el=find(rawL(:,1)==loc,1);eu=find(rawU(:,1)==loc,1);
                    hL=~isempty(el)&&IrawL(rawL(el,2))>GL(rawL(el,2))*(1+1e-9);
                    hU=~isempty(eu)&&IrawU(rawU(eu,2))>GU(rawU(eu,2))*(1+1e-9);
                    canComp=(b==1 && hL && ~hU)||(b==2 && hU && ~hL);
                    if allowComp && canComp && ~isempty(oppositeE)
                        [trial,added]=pure_marginal_compensate(trial,firstU, ...
                            oppositeE,Hother,Go,Po,other,rateFn);
                        if added==0,st.blockedBudget=st.blockedBudget+1;end
                    end
                    st.tested=st.tested+1;
                    trialRate=rateFn(trial);
                    if trialRate>bestRate+1e-6
                        best=trial;bestRate=trialRate;
                    end
                end
            end
            % Every accepted change eliminates violation on this channel.
            tx=best;changed=true;st.accepted=st.accepted+1;
        end
    end
    if ~changed,break;end
end
% Optional compensation AFTER protection for original case-1/2 SUs:
% evaluate actual rate and reject any harmful compensation.
IrawL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IrawU=aggregate_interference_by_channel(raw,HW,GU,'un');
for u=1:numel(tx)
    for loc=1:numel(tx(u).users)
        [iL,~,~]=get_band_fields(raw,u,'lic');
        [iU,~,~]=get_band_fields(raw,u,'un');
        eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
        hL=~isempty(eL)&&IrawL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
        hU=~isempty(eU)&&IrawU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
        if allowComp && hL && ~hU && ~isempty(eU)
            [t,dp]=pure_marginal_compensate(tx,u,eU,HW,GU,PU,'un',rateFn);
            if rateFn(t)>rateFn(tx)+1e-6,tx=t;st.compU=st.compU+dp;end
        elseif allowComp && hU && ~hL && ~isempty(eL)
            [t,dp]=pure_marginal_compensate(tx,u,eL,HP,GL,PL,'lic',rateFn);
            if rateFn(t)>rateFn(tx)+1e-6,tx=t;st.compL=st.compL+dp;end
        end
    end
end
tx=apply_final_safety_clipping(tx,HP,HW,GL,GU,PL,PU,'both');
end

