function pairs=hungarian_max(score)
% Exact rectangular assignment by dynamic programming; K<=6, S<=10.
[R,S]=size(score);pairs=zeros(0,2);
if R==0||S==0,return;end
if R>S,error('More users than active subchannels on one UAV');end
cost=-score;numStates=2^S;D=inf(R+1,numStates);D(1,1)=0;
prev=zeros(R+1,numStates);choose=zeros(R+1,numStates);
for k=1:R
    for mask=0:numStates-1
        if ~isfinite(D(k,mask+1)),continue;end
        for j=1:S
            if bitget(mask,j),continue;end
            newMask=bitset(mask,j);
            x=D(k,mask+1)+cost(k,j);
            if x<D(k+1,newMask+1)
                D(k+1,newMask+1)=x;prev(k+1,newMask+1)=mask;
                choose(k+1,newMask+1)=j;
            end
        end
    end
end
[~,mask]=min(D(R+1,:));mask=mask-1;pairs=zeros(R,2);
for k=R:-1:1
    j=choose(k+1,mask+1);pairs(k,:)=[k,j];
    mask=prev(k+1,mask+1);
end
end

