function [best,info]=budgeted_reserved_search(start,policy,maxCandidates, ...
    cases,HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU, ...
    mmIter,leakFrac,alpha,resL,resU)
% Both arms call exactly the same candidate evaluator. Budget counts
% FULL candidate evaluations, including rejected/infeasible trials.
rateFn=@(tx) diagnostic_tx_rate(tx,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
best=start;bestRate=rateFn(best);
info=struct('tested',0,'accepted',0,'testedByCase',zeros(1,4), ...
    'acceptedByCase',zeros(1,4),'gainByCase',zeros(1,4));
for u=1:numel(start)
    for loc=1:numel(start(u).users)
        caseId=cases.byUser{u}(loc);
        if policy==0
            bands=[1 2];
        elseif policy==2
            if caseId==1,bands=2;
            elseif caseId==2,bands=1;
            else,bands=[1 2];end
        elseif policy==3
            if caseId==4,continue;end
            if caseId==1,bands=2;
            elseif caseId==2,bands=1;
            else,bands=[1 2];end
        elseif policy==4
            if caseId~=4,continue;end
            bands=[1 2];
        else
            error('Unknown replacement policy');
        end
        selected=best;selectedRate=bestRate;
        for b=bands
            if b==1,idx=best(u).idxLic;reserve=resL;
            else,idx=best(u).idxUn;reserve=resU;end
            e=find(idx(:,1)==loc,1);
            if isempty(e),continue;end
            for sc=reserve
                if reserved_used_anywhere(best,sc,b),continue;end
                if info.tested>=maxCandidates,break;end
                info.tested=info.tested+1;
                info.testedByCase(caseId)=info.testedByCase(caseId)+1;
                [trial,~]=reserved_sc_trial(best,u,e,sc,b,true, ...
                    HL,HU,HP,HW,GL,GU,PL,PU,rateFn,hp,hw, ...
                    Ppbs,Pwifi,nL,nU,mmIter,leakFrac,alpha);
                r=rateFn(trial);
                if isfinite(r)&&r>selectedRate+1e-5
                    selected=trial;selectedRate=r;
                end
            end
            if info.tested>=maxCandidates,break;end
        end
        if selectedRate>bestRate+1e-5
            info.accepted=info.accepted+1;
            info.acceptedByCase(caseId)=info.acceptedByCase(caseId)+1;
            info.gainByCase(caseId)=info.gainByCase(caseId)+ ...
                (selectedRate-bestRate);
            best=selected;bestRate=selectedRate;
        end
        if info.tested>=maxCandidates,return;end
    end
end
end

