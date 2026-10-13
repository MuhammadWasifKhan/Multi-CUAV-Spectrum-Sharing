function tx=case_set_band_vector(tx,map,p,b)
for z=1:numel(p)
    u=map(z,1);ell=map(z,2);
    if b==1,tx(u).pLic(ell)=p(z);
    else,tx(u).pUn(ell)=p(z);end
end
end

