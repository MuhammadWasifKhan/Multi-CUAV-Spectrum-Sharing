function [g,gradLog]=a2g_gain_and_gradient(q,w,H,f,c,A,useShadow,z)
% gradLog = gradient of log(g), not gradient of g.
d=q-w; r2=sum(d.^2); r=sqrt(r2); D=r2+H^2;
theta=atan2d(H,max(r,1e-9));
p=1/(1+A.a*exp(-A.b*(theta-A.a)));
if r<1e-9
    gradTheta=[0,0];
else
    gradTheta=-(180/pi)*H/D*d/r;
end
gradP=A.b*p*(1-p)*gradTheta;
eta=p*A.etaLoS_dB+(1-p)*A.etaNLoS_dB;
gradEta=(A.etaLoS_dB-A.etaNLoS_dB)*gradP;
if useShadow
    variance=p*A.sigmaLoS_dB^2+(1-p)*A.sigmaNLoS_dB^2;
    sd=sqrt(max(variance,1e-30));
    sh=sd*z;
    gradSh=z*(A.sigmaLoS_dB^2-A.sigmaNLoS_dB^2)/(2*sd)*gradP;
else
    sh=0; gradSh=[0,0];
end
pl=20*log10(4*pi*f*sqrt(D)/c)+eta+sh;
g=10^(-pl/10);
gradLog=-d/D-(log(10)/10)*(gradEta+gradSh);
% FSPL is 20log10(distance), so its log-gain derivative is -2*d/D.
gradLog=gradLog-d/D;
end
