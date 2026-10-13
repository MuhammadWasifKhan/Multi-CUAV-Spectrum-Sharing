function [hu2L_all,hu2U_all,hp2,hw2,h_up2_all,h_uw2_all] = gen_multi_channels_scalar(...
    Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
    A2G,n_pbs,n_wifi,Shadow,seedNow,useRayleigh,useShadow,useShadowA2G)
U = size(Qslot,1);
K = size(WS,1);
J = size(WP,1);
M = size(WW,1);
hu2L_all  = zeros(U,K);
hu2U_all  = zeros(U,K);
h_up2_all = zeros(U,J);
h_uw2_all = zeros(U,M);
rng(seedNow + 555,'twister');
[hp2,hw2] = gen_terrestrial_interference_channels(WS,w_b,w_wifi,f_pbs,f_wifi,c,...
    n_pbs,n_wifi,Shadow,useRayleigh,useShadow);
for u = 1:U
    rng(seedNow + 10000*u,'twister');
    q = Qslot(u,:);
    [hu2L,hu2U,h_up2,h_uw2] = gen_uav_channels_only_scalar(...
        q,WS,WP,WW,H,f_lic,f_un,c,A2G,Shadow,useRayleigh,useShadowA2G);
    hu2L_all(u,:)  = hu2L(:).';
    hu2U_all(u,:)  = hu2U(:).';
    h_up2_all(u,:) = h_up2(:).';
    h_uw2_all(u,:) = h_uw2(:).';
end
end
