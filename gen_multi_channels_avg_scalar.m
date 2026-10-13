function [hu2L_all,hu2U_all,hp2,hw2,h_up2_all,h_uw2_all] = gen_multi_channels_avg_scalar(...
    Qslot,WS,WP,WW,w_b,w_wifi,H,f_lic,f_un,f_pbs,f_wifi,c,...
    A2G,n_pbs,n_wifi,Shadow,Nsamp,seed,useRayleigh,useShadow,useShadowA2G)
U = size(Qslot,1);
K = size(WS,1);
J = size(WP,1);
M = size(WW,1);
hu2L_all  = zeros(U,K);
hu2U_all  = zeros(U,K);
hp2       = zeros(K,1);
hw2       = zeros(K,1);
h_up2_all = zeros(U,J);
h_uw2_all = zeros(U,M);
for s = 1:Nsamp
    [a,b,c1,d1,e,f] = gen_multi_channels_scalar(Qslot,WS,WP,WW,w_b,w_wifi,H,...
        f_lic,f_un,f_pbs,f_wifi,c,A2G,n_pbs,n_wifi,Shadow,...
        seed + s,useRayleigh,useShadow,useShadowA2G);
    hu2L_all  = hu2L_all  + a;
    hu2U_all  = hu2U_all  + b;
    hp2       = hp2       + c1;
    hw2       = hw2       + d1;
    h_up2_all = h_up2_all + e;
    h_uw2_all = h_uw2_all + f;
end
hu2L_all  = hu2L_all  / Nsamp;
hu2U_all  = hu2U_all  / Nsamp;
hp2       = hp2       / Nsamp;
hw2       = hw2       / Nsamp;
h_up2_all = h_up2_all / Nsamp;
h_uw2_all = h_uw2_all / Nsamp;
end
