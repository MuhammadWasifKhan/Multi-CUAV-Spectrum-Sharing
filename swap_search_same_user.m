function [best,info]=swap_search_same_user(start,cases,caseTriggered, ...
        HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
        MM_iter,leakFrac,alpha)
% One accepted swap maximum per affected user; same-UAV one-to-one swaps.
% Candidate evaluation identical for generic and case-triggered variants.
rateFn=@(tx) diagnostic_tx_rate(tx,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
best=start;bestRate=rateFn(best);
info=struct('tested',0,'accepted',0,'compL',0,'compU',0, ...
    'moves',zeros(0,5));
for u=1:numel(start)
    for loc=1:numel(start(u).users)
        caseId=cases.byUser{u}(loc);
        if caseTriggered
            if caseId==1,bands=2; % Licensed excessive: unlicensed swap.
            elseif caseId==2,bands=1; % Unlicensed excessive: licensed swap.
            elseif caseId==3,bands=[1 2];
            else,bands=[];end
        else
            bands=[1 2];
        end
        % Only one accepted swap per SU, as in the proposed experiment.
        selected=best;selectedRate=bestRate;selectedMeta=[];
        for b=bands
            if b==1,idx=best(u).idxLic;
            else,idx=best(u).idxUn;end
            e=find(idx(:,1)==loc,1);
            if isempty(e),continue;end
            for partner=1:size(idx,1)
                if partner==e,continue;end
                info.tested=info.tested+1;
                [trial,delta]=swap_trial(best,u,e,partner,b, ...
                    HL,HU,HP,HW,GL,GU,PL,PU,rateFn, ...
                    hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
                    MM_iter,leakFrac,alpha);
                r=rateFn(trial);
                if isfinite(r)&&r>selectedRate+1e-5
                    selected=trial;selectedRate=r;
                    selectedMeta=[u loc b idx(e,2) idx(partner,2) delta];
                end
            end
        end
        if ~isempty(selectedMeta)
            best=selected;bestRate=selectedRate;
            info.accepted=info.accepted+1;
            info.moves(end+1,:)=selectedMeta(1:5); %#ok<AGROW>
            if selectedMeta(3)==1,info.compL=info.compL+selectedMeta(6);
            else,info.compU=info.compU+selectedMeta(6);end
        end
    end
end
end

