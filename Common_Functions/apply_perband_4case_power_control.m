function [tx,caseStats] = apply_perband_4case_power_control(tx,HpuL,HwuU,GammaLic,GammaUn,PmaxLic,PmaxUn,enableCompensation,detailedDebug)
if nargin < 9, detailedDebug = false; end
% USER-WISE AI-IBPC, performed only AFTER both bands have been allocated.
% Every associated SU is examined using the aggregate PU/WU interference on
% ITS selected licensed/unlicensed subchannels (all UAVs sharing each SC).
% Compensation adds power to the SAME SU's opposite-band stream only, from
% that opposite band's OWN remaining UAV budget and protected headroom.
% There is no cross-band power transfer. SU order is deterministic.
% Sequential changes update aggregate leakage before the next SU is checked.
% The final projection protects shared channels after all user adjustments.
caseStats = struct('case1_licHighOnly',0,'case2_unHighOnly',0,...
    'case3_bothHigh',0,'case4_normal',0,'licensedReduced_W',0,...
    'unlicensedReduced_W',0,'licensedCompensated_W',0,...
    'unlicensedCompensated_W',0,'usersProcessed',0);
tx = enforce_per_uav_band_budget(tx,PmaxLic,'lic');
tx = enforce_per_uav_band_budget(tx,PmaxUn,'un');
for u = 1:numel(tx)
    for loc = 1:numel(tx(u).users)
        [idxL,pL,~] = get_band_fields(tx,u,'lic');
        [idxU,pU,~] = get_band_fields(tx,u,'un');
        eL = find(idxL(:,1)==loc,1);
        eU = find(idxU(:,1)==loc,1);
        if isempty(eL) && isempty(eU), continue; end
        caseStats.usersProcessed = caseStats.usersProcessed+1;
        IL = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
        IU = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un');
        highL = ~isempty(eL) && IL(idxL(eL,2)) > GammaLic(idxL(eL,2))*(1+1e-9);
        highU = ~isempty(eU) && IU(idxU(eU,2)) > GammaUn(idxU(eU,2))*(1+1e-9);
        if false && detailedDebug
            suGlobal = tx(u).users(loc);
            fprintf('\nSU %d | UAV %d | L stream %d | U stream %d | case Lhigh=%d Uhigh=%d\n', ...
                suGlobal,u,~isempty(eL),~isempty(eU),highL,highU);
            fprintf('  UAV budget: L used %.8g / %.8g W; U used %.8g / %.8g W\n', ...
                sum(tx(u).pLic(:)),PmaxLic,sum(tx(u).pUn(:)),PmaxUn);
            if ~isempty(eL)
                fprintf('  Selected licensed SC %d: aggregate ratio %.6g\n', ...
                    idxL(eL,2),IL(idxL(eL,2))/GammaLic(idxL(eL,2)));
            end
            if ~isempty(eU)
                fprintf('  Selected unlicensed SC %d: aggregate ratio %.6g\n', ...
                    idxU(eU,2),IU(idxU(eU,2))/GammaUn(idxU(eU,2)));
            end
        end
        if highL && ~highU
            caseStats.case1_licHighOnly = caseStats.case1_licHighOnly+1;
            [tx,dp] = reduce_su_stream_to_aggregate_limit(tx,u,eL,HpuL,GammaLic,'lic');
            caseStats.licensedReduced_W = caseStats.licensedReduced_W+dp;
            if enableCompensation && ~isempty(eU)
                if false && detailedDebug
                    [ii,pp,vv] = get_band_fields(tx,u,'un');
                    ss = ii(eU,2);
                    allI = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un');
                    gg = abs(HwuU{u,ss}'*vv{eU})^2;
                    budgetLeft = max(0,PmaxUn-sum(pp(:)));
                    headLeft = max(0,GammaUn(ss)-allI(ss));
                    if gg > 1e-30
                        interferencePowerLeft = headLeft/gg;
                    else
                        interferencePowerLeft = Inf;
                    end
                    fprintf('  -> COMPENSATE UN: pre-power %.9g W | UAV budget left %.9g W | WU/PU headroom %.9g W | equivalent power headroom %.9g W\n', ...
                        pp(eU),budgetLeft,headLeft,interferencePowerLeft);
                end
                [tx,dpU] = increase_su_stream_if_safe(tx,u,eU,HwuU,GammaUn,PmaxUn,'un');
                if false && detailedDebug
                    fprintf('  -> ACTUAL UN COMPENSATION: %.9g W\n',dpU);
                    if dpU <= 1e-12
                        fprintf('  -> BLOCKED: inspect zero UAV budget or zero aggregate interference headroom above.\n');
                    end
                end
                caseStats.unlicensedCompensated_W = caseStats.unlicensedCompensated_W+dpU;
            end
        elseif ~highL && highU
            caseStats.case2_unHighOnly = caseStats.case2_unHighOnly+1;
            [tx,dp] = reduce_su_stream_to_aggregate_limit(tx,u,eU,HwuU,GammaUn,'un');
            caseStats.unlicensedReduced_W = caseStats.unlicensedReduced_W+dp;
            if enableCompensation && ~isempty(eL)
                if false && detailedDebug
                    [ii,pp,vv] = get_band_fields(tx,u,'lic');
                    ss = ii(eL,2);
                    allI = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
                    gg = abs(HpuL{u,ss}'*vv{eL})^2;
                    budgetLeft = max(0,PmaxLic-sum(pp(:)));
                    headLeft = max(0,GammaLic(ss)-allI(ss));
                    if gg > 1e-30
                        interferencePowerLeft = headLeft/gg;
                    else
                        interferencePowerLeft = Inf;
                    end
                    fprintf('  -> COMPENSATE LIC: pre-power %.9g W | UAV budget left %.9g W | WU/PU headroom %.9g W | equivalent power headroom %.9g W\n', ...
                        pp(eL),budgetLeft,headLeft,interferencePowerLeft);
                end
                [tx,dpL] = increase_su_stream_if_safe(tx,u,eL,HpuL,GammaLic,PmaxLic,'lic');
                if false && detailedDebug
                    fprintf('  -> ACTUAL LIC COMPENSATION: %.9g W\n',dpL);
                    if dpL <= 1e-12
                        fprintf('  -> BLOCKED: inspect zero UAV budget or zero aggregate interference headroom above.\n');
                    end
                end
                caseStats.licensedCompensated_W = caseStats.licensedCompensated_W+dpL;
            end
        elseif highL && highU
            caseStats.case3_bothHigh = caseStats.case3_bothHigh+1;
            [tx,dpL] = reduce_su_stream_to_aggregate_limit(tx,u,eL,HpuL,GammaLic,'lic');
            [tx,dpU] = reduce_su_stream_to_aggregate_limit(tx,u,eU,HwuU,GammaUn,'un');
            caseStats.licensedReduced_W = caseStats.licensedReduced_W+dpL;
            caseStats.unlicensedReduced_W = caseStats.unlicensedReduced_W+dpU;
        else
            caseStats.case4_normal = caseStats.case4_normal+1;
        end
    end
end
% Aggregate multi-UAV safety guard; another UAV can use the same SC.
tx = project_aggregate_band_interference(tx,HpuL,GammaLic,PmaxLic,'lic');
tx = project_aggregate_band_interference(tx,HwuU,GammaUn,PmaxUn,'un');
end

