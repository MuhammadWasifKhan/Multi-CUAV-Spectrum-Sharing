function out=reserved_add_power(base,u,e,b,extra)
out=base;
if b==1,out(u).pLic(e)=out(u).pLic(e)+extra;
else,out(u).pUn(e)=out(u).pUn(e)+extra;end
end
