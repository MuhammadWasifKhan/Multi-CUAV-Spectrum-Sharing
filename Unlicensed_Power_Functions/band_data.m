function [idx,p,vs]=band_data(tx,u,band)
if strcmp(band,'lic'),idx=tx(u).idxLic;p=tx(u).pLic(:);vs=tx(u).vLic;
else,idx=tx(u).idxUn;p=tx(u).pUn(:);vs=tx(u).vUn;end
end
