function startup_voronoi(map, start, goal)
startCompute=tic;
free = 1 - map;
free(1,:) = 0; free(100,:) = 0;
free(:,1) = 0; free(:,100) = 0;
skeleton = ithin(free);

[x,y]=size(skeleton);

sumOnes=find(skeleton==1);

skeletonPath=zeros(length(sumOnes),2);
i=1;

for xi=1:x
    for yi=1:y
        if skeleton(xi,yi) == 1
            skeletonPath(i,:)=[yi,xi];
            i=i+1;
        end
    end
end

% find closest start position on Voronoi Path
distStart=zeros(length(skeletonPath),1);
for i=1:length(skeletonPath)
    distStart(i)=norm(skeletonPath(i,:)-start);
end
[~,index]=min(distStart);
startOnPath=skeletonPath(index,:);

% find closest goal position on Voronoi Path
distGoal=zeros(length(skeletonPath),1);
for i=1:length(skeletonPath)
    distGoal(i)=norm(skeletonPath(i,:)-goal);
end
[~,index]=min(distGoal);
goalOnPath=skeletonPath(index,:);

% plan the shortest path, on the Voronoi roadmap
% voronoi path = 0
% rest is 1 (obstacle)
% this is forcing the shortest path method to use the voronoi roadmap
modmap=1-skeleton;
voronoi = DXform(modmap);
voronoi.plan(goalOnPath)
voronoiPath=voronoi.path(startOnPath);


% add shortest path to user start point, using regular map
dx1 = DXform(map);
dx1.plan(startOnPath)
startPath = dx1.path(start);
% add shortest path to user end point, using regular map
dx2 = DXform(map);
dx2.plan(goal)
endPath = dx2.path(goalOnPath);
% plot the voronoi path on the "regular" map
voronoi = DXform(map);
voronoi.plot()
hold on
voronoi.plan(goal)
% combine all
totalPath=[startPath;voronoiPath;endPath];
timeToCompute=toc(startCompute);
% plot it
for i=1:length(totalPath)
    plot(totalPath(i,1), totalPath(i,2), 'g.', 'MarkerSize', 12)
    drawnow
end
hold off
disp(['Planning took ', num2str(timeToCompute), ' sec'])
disp(['Path length : ',num2str(length(totalPath)), ' squares'])
end