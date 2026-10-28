function a=alpha(t,f)
a=ones(6,numel(t)); r=max(0,min(1,(t-f(1))/max(eps,f(2)-f(1)))); a(2,:)=1-(1-f(3))*r;
end
