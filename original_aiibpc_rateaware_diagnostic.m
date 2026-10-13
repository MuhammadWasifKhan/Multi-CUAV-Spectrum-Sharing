function [txBest,stats] = original_aiibpc_rateaware_diagnostic(raw,txConv,txFour, ...
    HP,HW,GL,GU,PL,PU,rateFn)
    % EXPERIMENTAL rate-aware, fallback-enhanced variant, NOT the
    % original AI-IBPC definition. Both protected references are polished.
    % Protection was applied in the original four-case block (txFour),
    % or conventional reference protection (txConv), never in raw allocation.
    %#ok<INUSD> raw is retained in interface for auditability.
    [txConv,convStats] = rateaware_polish_existing( ...
        txConv,HP,HW,GL,GU,PL,PU,rateFn);
    [txFour,fourStats] = rateaware_polish_existing( ...
        txFour,HP,HW,GL,GU,PL,PU,rateFn);
    rConv=rateFn(txConv);rFour=rateFn(txFour);
    if rFour > rConv+1e-9
        txBest=txFour;stats.source='fourcase';
    else
        txBest=txConv;stats.source='protected_reference';
    end
    [rL,rU]=pure_check(txBest,HP,HW,GL,GU,PL,PU);
    assert(max(rL,rU)<=1+1e-5,'Rate-aware original violates protection');
    stats.rate=rateFn(txBest);
    stats.objectiveCalls=convStats.objectiveCalls+fourStats.objectiveCalls+3;
    stats.scalarSearches=convStats.scalarSearches+fourStats.scalarSearches;
    stats.acceptedUpdates=convStats.acceptedUpdates+fourStats.acceptedUpdates;
    stats.conventionalPolish=convStats;
    stats.fourcasePolish=fourStats;
end

