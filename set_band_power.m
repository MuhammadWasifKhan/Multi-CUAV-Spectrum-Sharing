function tx = set_band_power(tx,u,bandName,p)
if strcmpi(bandName,'lic')
    tx(u).pLic = p(:);
else
    tx(u).pUn = p(:);
end
end
