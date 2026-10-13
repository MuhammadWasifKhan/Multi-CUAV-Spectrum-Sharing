function [tx,transferLog] = optionB_same_uav_rate_reallocation( ...
    txRaw,tx,HsuL,HsuU,HpuL,HwuU,hp2,hw2,P_PBS_j,P_WIFI_m, ...
    sigma2L,sigma2U,BscL,BscU,GammaL,GammaU,PmaxL,PmaxU,verbose)
if nargin<19,verbose=true;end
% Rate-aware Option B: redistribute existing power on the same UAV,
% never transfer watts across bands. Only clipped streams qualify as
% recipients. For each, test donors on the opposite band of the SAME UAV.
% Each candidate is checked against the actual network-wide sum rate and
% aggregate PU/WU interference. Accept strictly positive gains only.
transferLog = zeros(0,6);
for u = 1:numel(tx)
    for srcBandNum = 1:2
        if srcBandNum == 1
            sourceBand = 'lic'; destBand = 'un'; Hdest = HwuU; Gdest = GammaU;
        else
            sourceBand = 'un'; destBand = 'lic'; Hdest = HpuL; Gdest = GammaL;
        end
        [srcIdx,srcNow] = get_band_fields(tx,u,sourceBand);
        [srcIdxRaw,srcRaw] = get_band_fields(txRaw,u,sourceBand);
        [dstIdx,dstP] = get_band_fields(tx,u,destBand);
        if isempty(srcIdx) || isempty(dstIdx), continue; end
        for r = 1:size(srcIdx,1)
            originalRow = find(srcIdxRaw(:,1)==srcIdx(r,1) & ...
                               srcIdxRaw(:,2)==srcIdx(r,2),1);
            if isempty(originalRow) || srcRaw(originalRow)-srcNow(r)<1e-9
                continue;
            end
            recipient = find(dstIdx(:,1)==srcIdx(r,1),1);
            if isempty(recipient), continue; end
            % Donor stream must belong to another SU of this UAV.
            for donor = 1:numel(dstP)
                if donor==recipient || dstP(donor)<1e-7, continue; end
                baseline = optionB_total_rate(tx,HsuL,HsuU,hp2,hw2, ...
                    P_PBS_j,P_WIFI_m,sigma2L,sigma2U,BscL,BscU);
                bestGain = 0; bestTx = tx; bestDelta = 0;
                % Conservative transfer fractions; preserve the full budget.
                for frac = [0.05 0.10 0.20 0.35 0.50 0.75 1.00]
                    delta = dstP(donor)*frac;
                    trial = tx;
                    pTry = dstP;
                    pTry(donor) = pTry(donor)-delta;
                    pTry(recipient) = pTry(recipient)+delta;
                    trial = set_band_power(trial,u,destBand,pTry);
                    Itest = aggregate_interference_by_channel(trial,Hdest,Gdest,destBand);
                    if any(Itest(:)>Gdest(:).*(1+1e-8)),continue;end
                    % Source band remains untouched and already protected.
                    rateTry = optionB_total_rate(trial,HsuL,HsuU,hp2,hw2, ...
                        P_PBS_j,P_WIFI_m,sigma2L,sigma2U,BscL,BscU);
                    if isfinite(rateTry) && rateTry-baseline>bestGain+1e-7
                        bestGain=rateTry-baseline;
                        bestTx=trial;
                        bestDelta=delta;
                    end
                end
                if bestGain>0
                    recipientSU=tx(u).users(srcIdx(r,1));
                    donorSU=tx(u).users(dstIdx(donor,1));
                    tx=bestTx;
                    transferLog(end+1,:)=[u recipientSU donorSU 3-srcBandNum bestDelta bestGain/1e6]; %#ok<AGROW>
                    if verbose, fprintf('OPTION B: UAV %d, SU %d receives %.6g W from SU %d on %s; network gain %.6g Mbps\n', ...
                        u,recipientSU,bestDelta,donorSU,destBand,bestGain/1e6); end
                    [~,dstP] = get_band_fields(tx,u,destBand);
                end
            end
        end
    end
end
% Independent per-UAV power budgets and aggregate interference verification.
for u=1:numel(tx)
    assert(sum(tx(u).pLic(:))<=PmaxL*(1+1e-7));
    assert(sum(tx(u).pUn(:))<=PmaxU*(1+1e-7));
end
assert(all(aggregate_interference_by_channel(tx,HpuL,GammaL,'lic')<=GammaL(:).*(1+1e-7)));
assert(all(aggregate_interference_by_channel(tx,HwuU,GammaU,'un')<=GammaU(:).*(1+1e-7)));
end

