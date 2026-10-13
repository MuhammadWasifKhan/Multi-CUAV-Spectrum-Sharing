function tx=compensate_original_violators(tx,raw,HP,HW,GL,GU,PL,PU,unused)
% Four-case flags from PRE-protection raw aggregate interference. Only
% opposite-band increases from UNUSED SAME-BAND budget are allowed.
IL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IU=aggregate_interference_by_channel(raw,HW,GU,'un');
for u=1:numel(tx)
    for loc=1:numel(tx(u).users)
        [idxL,~,~]=get_band_fields(raw,u,'lic');
        [idxU,~,~]=get_band_fields(raw,u,'un');
        eL=find(idxL(:,1)==loc,1);
        eU=find(idxU(:,1)==loc,1);
        highL=~isempty(eL)&&IL(idxL(eL,2))>GL(idxL(eL,2))*(1+1e-9);
        highU=~isempty(eU)&&IU(idxU(eU,2))>GU(idxU(eU,2))*(1+1e-9);
        if highL && ~highU && ~isempty(eU)
            [tx,~]=increase_su_stream_if_safe(tx,u,eU,HW,GU,PU,'un');
        elseif highU && ~highL && ~isempty(eL)
            [tx,~]=increase_su_stream_if_safe(tx,u,eL,HP,GL,PL,'lic');
        end
    end
end
tx=apply_final_safety_clipping(tx,HP,HW,GL,GU,PL,PU,'both');
end

