function tx=joint_tx_from_vector(raw,map,p)
tx=raw;
for z=1:numel(p)
    u=map(z,1);ell=map(z,3);
    if map(z,2)==1,tx(u).pLic(ell)=p(z);
    else,tx(u).pUn(ell)=p(z);end
end
end

