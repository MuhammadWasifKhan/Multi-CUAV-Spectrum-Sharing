function [idx,p,vList] = get_band_fields(tx,u,bandName)
if strcmpi(bandName,'lic')
    idx = tx(u).idxLic;
    p = tx(u).pLic(:);
    vList = tx(u).vLic;
else
    idx = tx(u).idxUn;
    p = tx(u).pUn(:);
    vList = tx(u).vUn;
end
end
