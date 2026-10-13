function r=diagnostic_tx_rate(tx,HL,HU,hp2,hw2,Ppbs,Pwifi,nL,nU,bL,bU)
global AIIBPC_RATE_CALLS;
if ~isempty(AIIBPC_RATE_CALLS), AIIBPC_RATE_CALLS=AIIBPC_RATE_CALLS+1; end
r=compute_global_sum_rate_band_interuav(tx,HL,hp2,Ppbs,nL,bL,'lic')+ ...
  compute_global_sum_rate_band_interuav(tx,HU,hw2,Pwifi,nU,bU,'un');
end
