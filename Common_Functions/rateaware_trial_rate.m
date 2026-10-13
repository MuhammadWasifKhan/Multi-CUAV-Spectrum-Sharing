function r=rateaware_trial_rate(tx,u,band,p,e,power,rateFn)
    p(e)=power;
    tx=set_band_power(tx,u,band,p);
    r=rateFn(tx);
end




