function g=gain(q,w,H,f,c,A,z)
r=norm(q-w);d=sqrt(r^2+H^2);theta=atan2d(H,max(r,1e-9));
p=1/(1+A.a*exp(-A.b*(theta-A.a)));
loss=20*log10(4*pi*f*d/c)+p*A.etaL+(1-p)*A.etaN;
loss=loss+sqrt(p*A.sigL^2+(1-p)*A.sigN^2)*z;
g=10^(-loss/10);
end
