function out=swap_classify_cases(raw,HP,HW,GL,GU)
IL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IU=aggregate_interference_by_channel(raw,HW,GU,'un');
out.byUser=cell(numel(raw),1);out.counts=zeros(1,4);
for u=1:numel(raw)
    nu=numel(raw(u).users);out.byUser{u}=4*ones(nu,1);
    for loc=1:nu
        iL=find(raw(u).idxLic(:,1)==loc,1);
        iU=find(raw(u).idxUn(:,1)==loc,1);
        highL=~isempty(iL)&&IL(raw(u).idxLic(iL,2))> ...
            GL(raw(u).idxLic(iL,2))*(1+1e-9);
        highU=~isempty(iU)&&IU(raw(u).idxUn(iU,2))> ...
            GU(raw(u).idxUn(iU,2))*(1+1e-9);
        if highL&&highU,caseId=3;
        elseif highL,caseId=1;
        elseif highU,caseId=2;
        else,caseId=4;end
        out.byUser{u}(loc)=caseId;
        out.counts(caseId)=out.counts(caseId)+1;
    end
end
end

