function best=rss_fourcase(raw,HL,HU,HP,HW,GL,GU,PL,PU, ...
    hp,hw,PP,PW,nL,nU,reserve,mmIter,rateFn)
% Four-case reserved-channel search. No post-search clipping.
% If no feasible reserved-only allocation exists, keep feasible shutdown
% fallback, rather than reporting an infeasible allocation as successful.
IL=leakage(raw,HP,GL,'lic');IU=leakage(raw,HW,GU,'un');
best=raw;bestV=violation(best,HP,HW,GL,GU,PL,PU);
maxPass=2*sum(arrayfun(@(z)numel(z.users),raw));
for pass=1:maxPass
    updated=false;
    for u=1:numel(best)
        for loc=1:numel(best(u).users)
            [idxL]=band_data(raw,u,'lic');[idxU]=band_data(raw,u,'un');
            eL=find(idxL(:,1)==loc,1);eU=find(idxU(:,1)==loc,1);
            hL=~isempty(eL) && IL(idxL(eL,2))>GL(idxL(eL,2))*(1+1e-9);
            hU=~isempty(eU) && IU(idxU(eU,2))>GU(idxU(eU,2))*(1+1e-9);
            if ~hL && ~hU,continue;end
            candidateL=[NaN,reserve];candidateU=candidateL;
            chosen=best;chosenV=bestV;chosenR=rateFn(best);
            for a=1:numel(candidateL)
                for b=1:numel(candidateU)
                    scL=candidateL(a);scU=candidateU(b);
                    if isnan(scL)&&isnan(scU),continue;end
                    trial=best;
                    if ~isnan(scL)
                        e=find(trial(u).idxLic(:,1)==loc,1);
                        if isempty(e)||reserved_used(trial,scL,'lic'),continue;end
                        trial=replace_reserve(trial,u,e,scL,'lic',HL,HP, ...
                            hp,PP,nL,GL,mmIter);
                    end
                    if ~isnan(scU)
                        e=find(trial(u).idxUn(:,1)==loc,1);
                        if isempty(e)||reserved_used(trial,scU,'un'),continue;end
                        trial=replace_reserve(trial,u,e,scU,'un',HU,HW, ...
                            hw,PW,nU,GU,mmIter);
                    end
                    trial=refine_rss_power(trial,u,HL,HU,HP,HW,GL,GU,PL,PU, ...
                        hp,hw,PP,PW,nL,nU);
                    v=violation(trial,HP,HW,GL,GU,PL,PU);r=rateFn(trial);
                    if (chosenV>1+1e-6 && v<chosenV-1e-9) || ...
                       (v<=1+1e-6 && chosenV<=1+1e-6 && r>chosenR+1e-7)
                        chosen=trial;chosenV=v;chosenR=r;
                    end
                end
            end
            if chosenV<bestV-1e-9 || ...
                    (chosenV<=1+1e-6 && bestV<=1+1e-6 && chosenR>rateFn(best)+1e-7)
                best=chosen;bestV=chosenV;updated=true;
            end
        end
    end
    if ~updated,break;end
end
if ~feasible(best,HP,HW,GL,GU,PL,PU)
    % Explicit zero-power emergency fallback. This preserves PU/WU safety
    % but means this realization was not solved by reserved replacement.
    warning('IA-RSS: no feasible reserved-only selection, using silent fallback.');
    for u=1:numel(best),best(u).pLic(:)=0;best(u).pUn(:)=0;end
end
assert(feasible(best,HP,HW,GL,GU,PL,PU),'IA-RSS infeasible');
end
