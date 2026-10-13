function [best,stats]=fourcase_crossband_reserved_controller(base,cases,...
    HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU,...
    mmIter,leakFrac,alpha,resL,resU)
% RESERVED ALGORITHM USED IN PROPOSED II
% ------------------------------------------------------------
% 1) BASE is the RAW allocation after ordinary power allocation.
% 2) No protection/clipping is applied before the reserved search.
% 3) For every interference-affected SU, candidate replacements are tested
%    on the licensed band, unlicensed band, and both bands jointly.
% 4) A candidate is preferred first by aggregate-protection feasibility;
%    while the current allocation is still infeasible, the search selects
%    the candidate that most reduces the worst normalized violation.
% 5) Once protection is achieved, only protected candidates are eligible
%    and the candidate with the largest actual network sum rate is retained.
% 6) No post-search safety clipping is used. The returned allocation must
%    itself satisfy PU/WU aggregate-interference and per-UAV power budgets.
%
% This is a coordinated sequential candidate search (local search), not a
% claim of globally optimal mixed-integer enumeration.
rateFn=@(tx) diagnostic_tx_rate(tx,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
best=base;
stats=struct('tested',0,'accepted',0,'caseCounts',zeros(1,4),...
    'testedByCase',zeros(1,4),'acceptedByCase',zeros(1,4));

% Count raw interference cases.
for u=1:numel(base)
    for loc=1:numel(base(u).users)
        cid=cases.byUser{u}(loc);
        stats.caseCounts(cid)=stats.caseCounts(cid)+1;
    end
end

% Multiple coordinated passes allow replacements for several affected SUs
% to collectively remove a network-wide aggregate-interference violation.
maxPass=max(1,2*sum(stats.caseCounts(1:3)));
for pass=1:maxPass
    changed=false;
    for u=1:numel(best)
        for loc=1:numel(best(u).users)
            caseId=cases.byUser{u}(loc);
            if caseId==4
                continue; % raw allocation is safe for this SU on both bands
            end

            incumbent=best;
            incumbentRate=rateFn(incumbent);
            incumbentViolation=reserved_violation_metric(incumbent,HP,HW,GL,GU,PL,PU);
            chosen=incumbent;
            chosenRate=incumbentRate;
            chosenViolation=incumbentViolation;

            % Check BOTH bands. NaN means keep the original SC on that band.
            candidateL=[NaN resL];
            candidateU=[NaN resU];
            for il=1:numel(candidateL)
                for iu=1:numel(candidateU)
                    newL=candidateL(il); newU=candidateU(iu);
                    if isnan(newL) && isnan(newU)
                        continue;
                    end
                    if ~isnan(newL) && reserved_used_anywhere(incumbent,newL,1)
                        continue;
                    end
                    if ~isnan(newU) && reserved_used_anywhere(incumbent,newU,2)
                        continue;
                    end

                    trial=incumbent;
                    if ~isnan(newL)
                        e=find(trial(u).idxLic(:,1)==loc,1);
                        if isempty(e),continue;end
                        trial=reserved_replace_no_clip(trial,u,e,newL,1,...
                            HL,HU,HP,HW,GL,GU,hp,hw,Ppbs,Pwifi,nL,nU,...
                            mmIter,leakFrac,alpha);
                    end
                    if ~isnan(newU)
                        e=find(trial(u).idxUn(:,1)==loc,1);
                        if isempty(e),continue;end
                        trial=reserved_replace_no_clip(trial,u,e,newU,2,...
                            HL,HU,HP,HW,GL,GU,hp,hw,Ppbs,Pwifi,nL,nU,...
                            mmIter,leakFrac,alpha);
                    end

                    % Re-optimize powers under the actual aggregate protection
                    % headroom. This routine uses capped KKT only; no clipping.
                    trial=fourcase_kkt_refine_candidate(trial,u,...
                        HL,HU,HP,HW,GL,GU,PL,PU,hp,hw,Ppbs,Pwifi,...
                        nL,nU,BL,BU);

                    stats.tested=stats.tested+1;
                    stats.testedByCase(caseId)=stats.testedByCase(caseId)+1;
                    v=reserved_violation_metric(trial,HP,HW,GL,GU,PL,PU);
                    r=rateFn(trial);
                    if ~isfinite(r),continue;end

                    if incumbentViolation>1+1e-6
                        % Before feasibility is reached, reduce violation first;
                        % use rate only as the tie-breaker.
                        better=(v<chosenViolation-1e-9) || ...
                            (abs(v-chosenViolation)<=1e-9 && r>chosenRate+1e-5);
                    else
                        % Once protected, maximize rate over protected candidates.
                        better=(v<=1+1e-6) && r>chosenRate+1e-5;
                    end
                    if better
                        chosen=trial;
                        chosenRate=r;
                        chosenViolation=v;
                    end
                end
            end

            if chosenViolation<incumbentViolation-1e-9 || ...
                    (chosenViolation<=1+1e-6 && incumbentViolation<=1+1e-6 && ...
                     chosenRate>incumbentRate+1e-5)
                best=chosen;
                changed=true;
                stats.accepted=stats.accepted+1;
                stats.acceptedByCase(caseId)=stats.acceptedByCase(caseId)+1;
            end
        end
    end
    if reserved_violation_metric(best,HP,HW,GL,GU,PL,PU)<=1+1e-6
        % One additional protected rate-improvement pass is enough; if this
        % pass made no change, the coordinated sequential search has converged.
        if ~changed || pass>1
            break;
        end
    elseif ~changed
        break;
    end
end

% Final verification only. Candidate evaluation above is intentionally
% non-asserting because the RAW allocation may violate protection.
ILfinal=aggregate_interference_by_channel(best,HP,GL,'lic');
IUfinal=aggregate_interference_by_channel(best,HW,GU,'un');
ratioL=max(ILfinal(:)./max(GL(:),realmin));
ratioU=max(IUfinal(:)./max(GU(:),realmin));
powerOK=true;
for uu=1:numel(best)
    powerOK=powerOK && sum(best(uu).pLic(:))<=PL*(1+1e-5) && ...
        sum(best(uu).pUn(:))<=PU*(1+1e-5);
end
assert(ratioL<=1+1e-5 && ratioU<=1+1e-5 && powerOK,...
    ['No feasible reserved-subchannel combination was found without ' ...
     'power clipping. Increase the reserved pool or revise the raw allocation.']);
end

