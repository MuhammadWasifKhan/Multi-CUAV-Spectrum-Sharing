function [slotRate,leakLic,leakUn,txBest,userRate,diagnostics] = eval_slot_multi_qos_bf(Qslot,WS,WP,WW,w_b,w_wifi,H,...
        f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,QoS_target,Bsc_lic,Bsc_un,AO_AP_iter,seedNow,...
        useRayleigh,useShadow,useShadowA2G,Nt,MM_iter,MM_leakFrac,MM_reg_alpha,methodMode,compareMode,stageDebug,forcedAssoc)
if ~exist('compareMode','var') || isempty(compareMode), compareMode=0; end
if ~exist('stageDebug','var') || isempty(stageDebug),stageDebug=false;end
if ~exist('forcedAssoc','var'),forcedAssoc=[];end
% MATLAB requires persistent declarations at function scope before executable
% statements, not inside an AO iteration loop.
persistent aoRejectCount aoRejectInfeasible aoRejectRate aoRejectNonfinite
if isempty(aoRejectCount)
    aoRejectCount = 0;
    aoRejectInfeasible = 0;
    aoRejectRate = 0;
    aoRejectNonfinite = 0;
end
% ============================================================
% INNER AO: 3 C-UAV DUAL-BAND INDEPENDENT-BUDGET CBF-MM
% ------------------------------------------------------------
% Fixed Qslot and fixed instantaneous channels:
%   Block 1: User association.
%   Block 2: Licensed subchannel allocation.
%   Block 3: Licensed CBF-MM beamforming + KKT/water-filling power.
%   Block 4: Unlicensed allocation for all associated users.
%   Block 5: Unlicensed CBF-MM beamforming + KKT/water-filling power.
%   Block 6: Measure BOTH initial aggregate PU and WU interference.
%   Block 7: Integrated four-case AI-IBPC:
%            protection + rate-aware power adjustment.
% There is NO separate post-AI-IBPC protection/clipping block.
%
% The unlicensed band uses its own fixed per-UAV budget Pmax_un; no cross-band transfer is allowed.
%
% These blocks are repeated for AO_AP_iter or until the slot sum-rate
% converges. The previous AO iterate is used in the allocation metric through
% inter-UAV co-channel interference terms,so the loop is not a dummy repeat.
% ============================================================
[HsuL,HsuU,HpuL,HwuU,hp2,hw2,hu2L_all] = gen_multi_channels_vec(...
    Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
    A2G,n_pbs,n_wifi,Shadow,seedNow,useRayleigh,useShadow,useShadowA2G,Nt);
U = size(Qslot,1);
Kglob = size(WS,1);
J = numel(P_PBS_j);
M = numel(P_WIFI_m);
AO_AP_iter = max(1,round(AO_AP_iter));
AO_tol = 1e-4;
txPrev = repmat(struct('users',[],'idxLic',zeros(0,2),'pLic',[],'vLic',{{}},...
                       'idxUn',zeros(0,2),'pUn',[],'vUn',{{}}),U,1);
% The all-silent configuration is a feasible initial accepted AO iterate.
% Every subsequent accepted communication update is checked against the
% SAME realized channels and SAME global two-band sum-rate objective.
slotRateOld = 0;
txBest = txPrev;
slotRateBest = 0;
acceptedAOHistory = nan(AO_AP_iter+1,1);
acceptedAOHistory(1) = 0;
acceptedCount = 0;
diagnostics=struct('transferCount',0,'donorLossMbps',0);
for itAO = 1:AO_AP_iter
    % ===== Block 1: user association for fixed Qslot =====
    % Highest licensed-band channel-gain association.
    % Each SU is associated with the C-UAV providing the largest
    % instantaneous licensed-band channel gain ||h_{u,k}^{lic}[n]||^2.
    assoc = associate_users_highest_channel_gain(HsuL);
    if ~isempty(forcedAssoc)
        assert(numel(forcedAssoc)==Kglob && all(forcedAssoc>=1) && ...
            all(forcedAssoc<=U),'Invalid forced association');
        assoc=forcedAssoc(:);
    end
    tx = repmat(struct('users',[],'idxLic',zeros(0,2),'pLic',[],'vLic',{{}},...
                       'idxUn',zeros(0,2),'pUn',[],'vUn',{{}}),U,1);
    for u = 1:U
        tx(u).users = find(assoc == u).';
    end
    %% ===== Block 2: licensed Hungarian + CBF-MM + KKT =====
    for u = 1:U
        users_u = tx(u).users;
        if isempty(users_u)
            continue;
        end
        HsuL_u = HsuL(u,users_u);
        HpuL_u = HpuL(u,:);
        hp2_u  = hp2(users_u);
        extraNoiseL = previous_interuav_noise_matrix(...
            txPrev,HsuL,u,users_u,J,'lic');
        rhoL = assign_licensed_vec_extra_noise(...
            HsuL_u,HpuL_u,hp2_u,P_PBS_j,sigma2_lic,extraNoiseL,...
            GammaLic,Pmax_lic,Bsc_lic,numel(users_u),J,...
            'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,...
            Pmax_lic/max(J,1));
        [tx(u).pLic,tx(u).idxLic,tx(u).vLic] = solve_band_mm_given_rho(...
            rhoL,HsuL_u,HpuL_u,hp2_u,P_PBS_j,sigma2_lic,...
            GammaLic,Pmax_lic,MM_iter,MM_leakFrac,MM_reg_alpha);
    end
    % Do not clip yet: complete both interference-unaware allocations first.
    %% ===== Block 3: all-user unlicensed Hungarian + CBF-MM + KKT =====
    % No QoS gating: every associated SU is eligible on both bands.
    for u = 1:U
        users_u = tx(u).users;
        Pmax_un_u=Pmax_un;
        if compareMode>=2 && compareMode<=4
            % Estimate interference condition from the already-allocated licensed band.
            % Reserve ONLY for UAVs contributing to an unsafe licensed SC.
            licI=aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
            unsafeLic=licI(:)>GammaLic(:)*(1+1e-9);
            localIdx=tx(u).idxLic;
            if ~isempty(localIdx) && any(unsafeLic(localIdx(:,2)))
                Pmax_un_u=Pmax_un*(1-0.1*(compareMode-1));
            end
        end
        if isempty(users_u) || Pmax_un_u <= 1e-15
            continue;
        end
        needUn_local = true(numel(users_u),1); % Every associated SU is eligible
        HsuU_u = HsuU(u,users_u);
        HwuU_u = HwuU(u,:);
        hw2_u  = hw2(users_u);
        extraNoiseU = previous_interuav_noise_matrix(...
            txPrev,HsuU,u,users_u,M,'un');
        rhoU = assign_unlicensed_vec_extra_noise(...
            HsuU_u,HwuU_u,hw2_u,P_WIFI_m,sigma2_un,extraNoiseU,...
            GammaUn,Pmax_un_u,Bsc_un,numel(users_u),M,needUn_local(:),...
            'hungarian','mm',MM_iter,MM_leakFrac,MM_reg_alpha,...
            Pmax_un_u/max(M,1));
        [tx(u).pUn,tx(u).idxUn,tx(u).vUn] = solve_band_mm_given_rho(...
            rhoU,HsuU_u,HwuU_u,hw2_u,P_WIFI_m,sigma2_un,...
            GammaUn,Pmax_un_u,MM_iter,MM_leakFrac,MM_reg_alpha);
    end
    %% ===== Block 4: measure both PRE-CONTROL aggregate interferences =====
    % Aggregate across ALL C-UAVs on each corresponding protected channel.
    % These are the raw initial allocations, before any aggregate clipping.
    Ilic_initial = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic'); %#ok<NASGU>
    Iun_initial  = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un'); %#ok<NASGU>
    %% ===== Block 5: integrated AI-IBPC four-case protection =====
    % Interference is measured AFTER association, subchannel allocation,
    % CBF-MM beamforming and initial KKT power allocation.
    %
    % Case 1: licensed high, unlicensed safe -> rate-aware licensed reduction
    %         plus rate-aware unlicensed increase using only unlicensed budget
    %         and WU-interference headroom.
    % Case 2: unlicensed high, licensed safe -> mirror of Case 1.
    % Case 3: neither high -> keep the original power allocation unchanged.
    % Case 4: both high -> rate-aware reductions on both bands until both
    %         protection constraints are satisfied.
    %
    % Protection is part of AI-IBPC itself. There is NO separate final
    % safety-clipping/protection block after this controller.
    txRaw=tx;
    rateFnLocal=@(z)diagnostic_tx_rate(z,HsuL,HsuU,hp2,hw2,...
        P_PBS_j,P_WIFI_m,sigma2_lic,sigma2_un,Bsc_lic,Bsc_un);
    [tx,caseStats]=ai_ibpc_integrated_four_case_rateaware(...
        txRaw,HpuL,HwuU,GammaLic,GammaUn,...
        Pmax_lic,Pmax_un,rateFnLocal);
    diagnostics=struct('transferCount',0,'donorLossMbps',0);
    if stageDebug
        diagnostics.rawTx=txRaw;
    end
    % The controller processes each SU and updates shared-channel interference.
    %% ===== Block 6: rate evaluation and AO acceptance =====
    %% ===== Convergence check for fixed-Qslot AO =====
    slotRateLic = compute_global_sum_rate_band_interuav(tx,HsuL,hp2,P_PBS_j,...
        sigma2_lic,Bsc_lic,'lic');
    slotRateUn  = compute_global_sum_rate_band_interuav(tx,HsuU,hw2,P_WIFI_m,...
        sigma2_un,Bsc_un,'un');
    slotRateNew = slotRateLic + slotRateUn;
    % The preceding power-protection routines ensure per-band feasibility.
    % Reject any candidate that fails an explicit feasibility check.
    IlicNow = aggregate_interference_by_channel(tx,HpuL,GammaLic,'lic');
    IunNow = aggregate_interference_by_channel(tx,HwuU,GammaUn,'un');
    candidateFeasible = all(IlicNow(:) <= GammaLic(:)*(1+1e-6)) && ...
        all(IunNow(:) <= GammaUn(:)*(1+1e-6));
    for uu = 1:U
        candidateFeasible = candidateFeasible && ...
            sum(tx(uu).pLic(:)) <= Pmax_lic*(1+1e-6) && ...
            sum(tx(uu).pUn(:)) <= Pmax_un*(1+1e-6);
    end
    % Only accept a complete resource-allocation sweep, never assume that
    % Hungarian, MM, or KKT individually guarantee global improvement.
    % A rejected AO proposal is not a solver error. Hungarian, MM, and
    % clipped KKT blocks do not jointly form a guaranteed ascent step.
    % Retain the incumbent rather than accepting a lower-rate proposal.
    % Terminating immediately avoids expensive, repeatedly rejected sweeps.
    % Avoid printing on every finite-difference/channel-sample evaluation.
    rateDecreased = isfinite(slotRateNew) && ...
        slotRateNew < slotRateBest - 1e-10*max(1,abs(slotRateBest));
    if ~candidateFeasible || ~isfinite(slotRateNew) || rateDecreased
        aoRejectCount = aoRejectCount + 1;
        if ~candidateFeasible
            aoRejectInfeasible=aoRejectInfeasible+1;
            cause='constraint violation';
        elseif ~isfinite(slotRateNew)
            aoRejectNonfinite=aoRejectNonfinite+1;
            cause='nonfinite rate';
        else
            aoRejectRate=aoRejectRate+1;
            cause='rate decrease';
        end
        % Show two representative failures and occasional aggregate statistics.
        % This avoids flooding the console during finite-difference SCA.
        if false % VTC-style concise output; safety checks still enforced
            licRatio=max(IlicNow(:)./max(GammaLic(:),realmin));
            unRatio=max(IunNow(:)./max(GammaUn(:),realmin));
            licBudget=max(arrayfun(@(z)sum(z.pLic(:)),tx))/max(Pmax_lic,realmin);
            unBudget=max(arrayfun(@(z)sum(z.pUn(:)),tx))/max(Pmax_un,realmin);
            fprintf(['Inner AO stopped (%s): candidate %.6g Mbps, ' ...
                'incumbent %.6g Mbps; Ilic %.4g, Iun %.4g, ' ...
                'Plic %.4g, Pun %.4g [ratios; limit 1].\n'], ...
                cause,slotRateNew/1e6,slotRateBest/1e6, ...
                licRatio,unRatio,licBudget,unBudget);
            if mod(aoRejectCount,500)==0
                fprintf('AO rejection totals: infeasible=%d, rate=%d, nonfinite=%d.\n', ...
                    aoRejectInfeasible,aoRejectRate,aoRejectNonfinite);
            end
        end
        if ~stageDebug,diagnostics=struct('transferCount',0,'donorLossMbps',0);end
        break;
    end
    previousAcceptedRate = slotRateBest;
    txBest = tx;
    slotRateBest = max(slotRateBest,slotRateNew);
    acceptedCount = acceptedCount + 1;
    acceptedAOHistory(acceptedCount+1) = slotRateBest;
    txPrev = txBest;
    slotRateOld = slotRateBest;
    if acceptedCount > 1 && ...
            (slotRateBest-previousAcceptedRate)/max(1,abs(previousAcceptedRate)) <= AO_tol
        break;
    end
end
% Return best safe AO iterate,not necessarily the last one.
% ------------------------------------------------------------------
% Final verification checks after each slot. These assertions do not change
% the algorithm. They only stop the simulation if the selected safe AO iterate
% violates any aggregate interference or per-UAV power-budget constraint.
Ilic_best = aggregate_interference_by_channel(txBest,HpuL,GammaLic,'lic');
Iun_best  = aggregate_interference_by_channel(txBest,HwuU,GammaUn,'un');
assert(all(Ilic_best(:) <= GammaLic(:).*(1+1e-6)),...
    'Licensed aggregate interference violation after final protection.');
assert(all(Iun_best(:) <= GammaUn(:).*(1+1e-6)),...
    'Unlicensed aggregate interference violation after final protection.');
for uu = 1:U
    pLicSum = 0;
    pUnSum  = 0;
    if isfield(txBest(uu),'pLic') && ~isempty(txBest(uu).pLic)
        pLicSum = sum(txBest(uu).pLic(:));
    end
    if isfield(txBest(uu),'pUn') && ~isempty(txBest(uu).pUn)
        pUnSum = sum(txBest(uu).pUn(:));
    end
    assert(pLicSum <= Pmax_lic*(1+1e-6), ...
        'Licensed AI-IBPC power-budget violation.');
    assert(pUnSum <= Pmax_un*(1+1e-6), ...
        'Unlicensed AI-IBPC power-budget violation.');
end
slotRateLic = compute_global_sum_rate_band_interuav(txBest,HsuL,hp2,P_PBS_j,...
    sigma2_lic,Bsc_lic,'lic');
slotRateUn  = compute_global_sum_rate_band_interuav(txBest,HsuU,hw2,P_WIFI_m,...
    sigma2_un,Bsc_un,'un');
slotRate = slotRateLic + slotRateUn;
userRate=compute_global_user_rate_band_interuav(txBest,HsuL,hp2,P_PBS_j,sigma2_lic,Bsc_lic,Kglob,'lic')+ ...
         compute_global_user_rate_band_interuav(txBest,HsuU,hw2,P_WIFI_m,sigma2_un,Bsc_un,Kglob,'un');
% Actual aggregate post-protection leakage, available for trajectory SCA.
leakLic = aggregate_interference_by_channel(txBest,HpuL,GammaLic,'lic');
leakUn  = aggregate_interference_by_channel(txBest,HwuU,GammaUn,'un');
end
