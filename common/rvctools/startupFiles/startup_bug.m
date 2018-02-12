function startup_bug(map,start,goal)
startCompute=tic;
bug = Bug2(map);
bug.plot()
bug.goal = goal;
timeToCompute=toc(startCompute);
bug.path(start);
p=bug.path(start);
disp(['Planning took ', num2str(timeToCompute), ' sec'])
disp(['Path length : ',num2str(length(p)), ' squares'])
end