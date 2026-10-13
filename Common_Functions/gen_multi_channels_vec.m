function [HsuL,HsuU,HpuL,HwuU,hp2,hw2,hu2L_all] = gen_multi_channels_vec(...
    Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
    A2G,n_pbs,n_wifi,Shadow,seedNow,useRayleigh,useShadow,useShadowA2G,Nt)
U = size(Qslot,1);
K = size(WS,1);
J = size(WP,1);
M = size(WW,1);
HsuL = cell(U,K);
HsuU = cell(U,K);
HpuL = cell(U,J);
HwuU = cell(U,M);
hu2L_all = zeros(U,K);
rng(seedNow + 555,'twister');
[hp2,hw2] = gen_terrestrial_interference_channels(WS,w_b,w_wifi,f_pbs,f_wifi,c,...
    n_pbs,n_wifi,Shadow,useRayleigh,useShadow);
for u = 1:U
    rng(seedNow + 10000*u,'twister');
    q = Qslot(u,:);
    for k = 1:K
        gL = a2g_gain_alhourani(q,WS(k,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
        gU = a2g_gain_alhourani(q,WS(k,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_su_z(k));
        HsuL{u,k} = gen_miso_vector(gL,Nt,useRayleigh);
        HsuU{u,k} = gen_miso_vector(gU,Nt,useRayleigh);
        hu2L_all(u,k) = norm(HsuL{u,k})^2;
    end
    for j = 1:J
        g = a2g_gain_alhourani(q,WP(j,:),H,f_lic,c,A2G,useShadowA2G,Shadow.a2g_pu_z(j));
        HpuL{u,j} = gen_miso_vector(g,Nt,useRayleigh);
    end
    for m = 1:M
        g = a2g_gain_alhourani(q,WW(m,:),H,f_un,c,A2G,useShadowA2G,Shadow.a2g_wu_z(m));
        HwuU{u,m} = gen_miso_vector(g,Nt,useRayleigh);
    end
end
end
