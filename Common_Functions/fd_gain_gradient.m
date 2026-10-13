function [dgdx,dgdy,g0] = fd_gain_gradient(q0,wk,H,f,c,A2G,epsFD)
g0  = a2g_gain_alhourani_mean(q0,wk,H,f,c,A2G);
gx1 = a2g_gain_alhourani_mean(q0+[epsFD,0],wk,H,f,c,A2G);
gx2 = a2g_gain_alhourani_mean(q0-[epsFD,0],wk,H,f,c,A2G);
gy1 = a2g_gain_alhourani_mean(q0+[0,epsFD],wk,H,f,c,A2G);
gy2 = a2g_gain_alhourani_mean(q0-[0,epsFD],wk,H,f,c,A2G);
dgdx = (gx1-gx2)/(2*epsFD);
dgdy = (gy1-gy2)/(2*epsFD);
end
