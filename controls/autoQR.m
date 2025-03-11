function [Qp,Rp]=autoQR(A,B,C,maxIt)
%%
%This script automate the selection of Q and R selection for LQR design.
%Input:  A=state matrix
        %B=input Matrix
        %C=output matrix
        %maxIt= Set desired number of iteration
%This concept is published in this paper: https://ieeexplore.ieee.org/document/10380869
%% 
Mcr=ctrb(A,B);
Mob=obsv(A,C);
if rank(Mcr)==size(A)
    'System is Controlable'
else
    'System is NOT Controlable'
end
% CONTR_RANK=rank(Mcr) 
if rank(Mob)==size(A)
    'System is Observable'
else
    'System is NOT Observable'
end
[aa,~]=size(A);
[b1,b2]=size(B);
eigVal=eye(aa);
% initilize eigVal position
for i=1:aa
    realEigen=-1*rand;
    imEigen=randi(aa,1)*rand*((min(min(A))+max(max(A))*(rand*max(max(A))*100)^0.125)*max(max(B)))^0.5;
    eigVal(i,i)=realEigen+1i*(-1)^i*imEigen;
end
eigVal=real(eigVal);
b=[];
for j=1:b1
    if B(j)==0
        b(j,1)=1;
    else 
        b(j,1)=B(j);
    end
end
y=0;
% Initialize Q and R
r=zeros(b2);
q=zeros(aa);
while y~=maxIt
    y=y+1;
    t=2*y*randi(2);
    r=r+eye(b2).*rand(b2).*randi(2,b2,b2);
    % set treshhold for positive definatness
    ppthres=0.0001;
    r=(r+rand*(t-y+1)*r/t)/2;
    Rp=r;    
    q=(q+(t*eye(aa)+(q+(sum(sum(r))*(round(rand(aa)).*(randi(100,aa,1)*ones(1,aa))+(eye(aa).*(randi(100,aa,1)*ones(1,aa))))/2))/2)/50)/2;
    for m=1:(aa)^2
        if abs(q(m))<ppthres
            q(m)=0;
        end
    end
    %make sure q is symmetric
    q=(tril(q)+(tril(q))')/2;
    for m=1:(aa)^2
        if abs(q(m))<ppthres
            q(m)=0;
        end
    end
    q=rand*(t-y+1)*q/t;
    for m=1:(aa)^2
        if abs(q(m))<ppthres
            q(m)=0;
        end
    end
    Qp=q;
    [~,P,~]=lqr(A,B,Qp,Rp); %find P
    % update Q using P
    q=(abs(A'*P+P*eigVal)+(abs(A'*P+P*eigVal))')/2;
    q=q.*Qp/max(max(q));
    for m=1:(aa)^2
        if abs(q(m))<ppthres
        q(m)=0;
        end
    end
end