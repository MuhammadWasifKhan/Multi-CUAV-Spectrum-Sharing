function caseId=classify_raw_network_case(raw,HP,HW,GL,GU)
% One raw case per complete time-slot/channel realization.
% A band is high when ANY of its protected channel constraints is violated.
IL=aggregate_interference_by_channel(raw,HP,GL,'lic');
IU=aggregate_interference_by_channel(raw,HW,GU,'un');
assert(numel(IL)==numel(GL) && numel(IU)==numel(GU), ...
    'Interference and threshold dimensions must agree.');
highL=any(IL(:)>GL(:).*(1+1e-9));
highU=any(IU(:)>GU(:).*(1+1e-9));
if highL && ~highU
    caseId=1;
elseif ~highL && highU
    caseId=2;
elseif highL && highU
    caseId=3;
else
    caseId=4;
end
end

