function [hp2,hw2] = gen_terrestrial_interference_channels(WS,w_b,w_wifi,f_pbs,f_wifi,c,...
    n_pbs,n_wifi,Shadow,useRayleigh,useShadow)
K = size(WS,1);
hp2 = zeros(K,1);
hw2 = zeros(K,1);
PL1_pbs_dB  = 20*log10(4*pi*f_pbs/c);
PL1_wifi_dB = 20*log10(4*pi*f_wifi/c);
for k=1:K
    d = max(norm(w_b - WS(k,:)),1.0);
    Xp = 0;
    if useShadow,Xp = Shadow.pbs_su_dB(k); end
    PL = PL1_pbs_dB + 10*n_pbs*log10(d) + Xp;
    g  = 10^(-PL/10);
    if useRayleigh
        z = (randn + 1j*randn)/sqrt(2);
        hp2(k) = g * abs(z)^2;
    else
        hp2(k) = g;
    end
    d = max(norm(w_wifi - WS(k,:)),1.0);
    Xw = 0;
    if useShadow,Xw = Shadow.wifi_su_dB(k); end
    PL = PL1_wifi_dB + 10*n_wifi*log10(d) + Xw;
    g  = 10^(-PL/10);
    if useRayleigh
        z = (randn + 1j*randn)/sqrt(2);
        hw2(k) = g * abs(z)^2;
    else
        hw2(k) = g;
    end
end
end
%% ============================================================
%                  VECTOR BF SLOT ALGORITHMS
%% ============================================================
