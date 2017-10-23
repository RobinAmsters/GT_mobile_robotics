function startup_dxform(map, start, goal)
startCompute=tic;
dx = DXform(map);
dx.plot()
dx.plan(goal)
timeToCompute=toc(startCompute);
dx.path(start);
p = dx.path(start);
dx.plot(p)
figure()
dx.plot3d(p)
disp(['Planning took ', num2str(timeToCompute), ' sec'])
disp(['Path length : ',num2str(length(p)), ' squares'])
end