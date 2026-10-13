function tx=set_p(tx,u,band,p)
if strcmp(band,'lic'),tx(u).pLic=p(:);else,tx(u).pUn=p(:);end
end
