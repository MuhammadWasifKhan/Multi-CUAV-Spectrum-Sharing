function g = a2g_gain_alhourani_mean(q,w,H,f,c,A2G)
d2 = norm(q - w);
d3 = sqrt(d2^2 + H^2);
theta = atan2d(H,max(d2,1e-9));
P_LoS = 1 / (1 + A2G.a * exp(-A2G.b * (theta - A2G.a)));
P_NLoS = 1 - P_LoS;
FSPL_dB = 20*log10(4*pi*f*d3/c);
eta_dB = P_LoS*A2G.etaLoS_dB + P_NLoS*A2G.etaNLoS_dB;
PL_dB = FSPL_dB + eta_dB;
g = 10^(-PL_dB/10);
end
