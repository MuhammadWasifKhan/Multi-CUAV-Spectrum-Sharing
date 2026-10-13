function [hu2_lic,hu2_un,h_up2,h_uw2] = gen_uav_channels_only_scalar(...
    q,WS,WP,WW,H,f_lic,f_un,c,A2G,Shadow,useRayleigh,useShadowA2G)
K = size(WS,1);
J = size(WP,1);
M = size(WW,1);
hu2_lic = zeros(K,1);
hu2_un  = zeros(K,1);
for k=1:K
    gL = a2g_gain_alhourani(q,WS(k,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
    gU = a2g_gain_alhourani(q,WS(k,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
    if useRayleigh
        z1 = (randn + 1j*randn)/sqrt(2);
        z2 = (randn + 1j*randn)/sqrt(2);
        hu2_lic(k) = gL * abs(z1)^2;
        hu2_un(k)  = gU * abs(z2)^2;
    else
        hu2_lic(k) = gL;
        hu2_un(k)  = gU;
    end
end
h_up2 = zeros(J,1);
for j=1:J
    g = a2g_gain_alhourani(q,WP(j,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_pu_z(j));
    if useRayleigh
        z = (randn + 1j*randn)/sqrt(2);
        h_up2(j) = g * abs(z)^2;
    else
        h_up2(j) = g;
    end
end
h_uw2 = zeros(M,1);
for m=1:M
    g = a2g_gain_alhourani(q,WW(m,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_wu_z(m));
    if useRayleigh
        z = (randn + 1j*randn)/sqrt(2);
        h_uw2(m) = g * abs(z)^2;
    else
        h_uw2(m) = g;
    end
end
end
