function [tx,st]=pure_post_compensate(base,raw,HP,HW,GL,GU,PL,PU,rateFn,mode)
% Both policies start from exactly the same reduction-only base.
% Generic: same-user opposite-band compensation whenever OWN stream was
% reduced, regardless of original interference case. Case: only original
% case 1 or case 2, on its safe opposite band. Same marginal solver.
% No donor-user power, no cross-band budget transfer, no reserve.
tx=base;
IrawL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IrawU=aggregate_interference_by_channel(raw,HW,GU,'un');
st=struct('cases',zeros(1,4),'compL',0,'compU',0, ...
    'eligible',0,'blocked',0,'accepted',0);
for u=1:numel(raw)
    for loc=1:numel(raw(u).users)
        [iL,pL,~]=get_band_fields(raw,u,'lic');
        [iU,pU,~]=get_band_fields(raw,u,'un');
        [bL,qL,~]=get_band_fields(base,u,'lic');
        [bU,qU,~]=get_band_fields(base,u,'un');
        eL=find(iL(:,1)==loc,1);eU=find(iU(:,1)==loc,1);
        fL=find(bL(:,1)==loc,1);fU=find(bU(:,1)==loc,1);
        highL=~isempty(eL)&&IrawL(iL(eL,2))>GL(iL(eL,2))*(1+1e-9);
        highU=~isempty(eU)&&IrawU(iU(eU,2))>GU(iU(eU,2))*(1+1e-9);
        if highL&&highU,st.cases(3)=st.cases(3)+1;
        elseif highL,st.cases(1)=st.cases(1)+1;
        elseif highU,st.cases(2)=st.cases(2)+1;
        else,st.cases(4)=st.cases(4)+1;end
        reducedL=~isempty(eL)&&~isempty(fL)&&pL(eL)>qL(fL)+1e-12;
        reducedU=~isempty(eU)&&~isempty(fU)&&pU(eU)>qU(fU)+1e-12;
        if strcmp(mode,'case')
            wantU=highL && ~highU && reducedL;
            wantL=highU && ~highL && reducedU;
        else
            wantU=reducedL;
            wantL=reducedU;
        end
        % One user may qualify for both directions in generic mode.
        % Fixed order in both methods; only eligibility differs.
        if wantU && ~isempty(fU)
            st.eligible=st.eligible+1;
            [trial,dp]=pure_marginal_compensate(tx,u,fU,HW,GU,PU,'un',rateFn);
            if dp>1e-12
                tx=trial;st.compU=st.compU+dp;st.accepted=st.accepted+1;
            else
                st.blocked=st.blocked+1;
            end
        end
        if wantL && ~isempty(fL)
            st.eligible=st.eligible+1;
            [trial,dp]=pure_marginal_compensate(tx,u,fL,HP,GL,PL,'lic',rateFn);
            if dp>1e-12
                tx=trial;st.compL=st.compL+dp;st.accepted=st.accepted+1;
            else
                st.blocked=st.blocked+1;
            end
        end
    end
end
end

