function StopProp(app)
for i = 1:app.N
app.agent(i).plant.stop;
end
end