function StopProp(app)
app.fStart = 0;
app.isReady = false;
for i = 1:app.N
app.agent(i).plant.stop;
end
end