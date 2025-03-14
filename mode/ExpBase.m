for i = 1:length(agent)
  agent(i).reference.takeoff = TAKEOFF_REFERENCE(agent(i),[]);
  agent(i).landing = LANDING_REFERENCE(agent(i),dt,0.1);
  if isfield(agent,"cha_allocation")
  agent.cha_allocation = struct("reference",["time_varying"],"t",struct("reference",["takeoff"]),"l",struct("reference","landing"));
  end
end