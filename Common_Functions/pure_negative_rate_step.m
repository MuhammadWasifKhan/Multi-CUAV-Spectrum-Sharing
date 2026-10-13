function val=pure_negative_rate_step(tx,u,ell,band,p,dp,rateFn)
p(ell)=p(ell)+dp;
tx=set_band_power(tx,u,band,p);
val=-rateFn(tx);
end
