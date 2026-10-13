function [tx,stats]=rate_aware_protect_both(tx,HP,HW,GL,GU,PL,PU,HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU)
% Same protection for baseline and proposed modes; fixed beamformers.
stats=struct();
tx=rate_aware_band_projection(tx,HP,GL,PL,'lic',HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
tx=rate_aware_band_projection(tx,HW,GU,PU,'un',HL,HU,hp,hw,Ppbs,Pwifi,nL,nU,BL,BU);
end
