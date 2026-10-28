function v=variant(name)
assert(ismember(name,{'Flatness','StateMap','Observer','StateObserver','FastStateObserver','LTV','LTVObserver'}));
v=struct('name',name,'ltv',startsWith(name,'LTV'),'observer',contains(name,'Observer'), ...
    'state_map',ismember(name,{'StateMap','StateObserver','FastStateObserver'}),'internal_h',.001, ...
    'observer_tc',.12,'observer_fraction',.15,'health_gate',.015,'q_on',.02,'q_off',.01,'v_on',.08,'v_off',.04);
if strcmp(name,'FastStateObserver'), v.internal_h=.002; end
end
