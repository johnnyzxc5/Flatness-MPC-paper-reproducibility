function p=prepare(kind,tr,m,c)
tm=tic;
if strcmp(kind,'LTV'), base='LTV'; else, base='Flatness'; end
p=ftmpc.prepare(base,tr,m,c); p.variant=kind; p.has_warm=false;
if strcmp(kind,'HoldCorrected')
    nk=numel(tr.t)-1; p.K=zeros(6,6,nk);
    for k=1:nk
        [~,B]=ftmpc.linearize([tr.q(:,k);tr.dq(:,k)],tr.tau(:,k),m,c.Ts);
        assert(rcond(B(7:12,:))>1e-10,'Singular velocity-to-torque map');
        p.K(:,:,k)=(c.Ts*eye(6))/B(7:12,:);
    end
end
p.prep_seconds=toc(tm);
end
