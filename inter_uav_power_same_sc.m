function Iuav = inter_uav_power_same_sc(tx,HsuFull,servingUAV,victimUserGlobal,sc,bandName)
% Interference at victimUserGlobal caused by all other UAVs using the same subchannel.
Iuav = 0;
U = numel(tx);
for u2 = 1:U
    if u2 == servingUAV
        continue;
    end
    if strcmpi(bandName,'lic')
        idx2 = tx(u2).idxLic; p2 = tx(u2).pLic; vList2 = tx(u2).vLic;
    else
        idx2 = tx(u2).idxUn;  p2 = tx(u2).pUn;  vList2 = tx(u2).vUn;
    end
    for ell2 = 1:size(idx2,1)
        sc2 = idx2(ell2,2);
        if sc2 == sc
            v2 = vList2{ell2};
            hInterf = HsuFull{u2,victimUserGlobal};
            Iuav = Iuav + p2(ell2) * abs(hInterf' * v2)^2;
        end
    end
end
end
