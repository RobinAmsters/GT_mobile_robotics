function startup_dstar(map, start, goal)
% TODO introductuion terrain change
startCompute=tic;
ds = Dstar(map,'quiet');
c=ds.costmap_get;
ds.plan(goal)
timeToCompute=toc(startCompute);
ds.path(start);
p=ds.path(start);

% Conclusion
disp(['Planning took ', num2str(timeToCompute), ' sec'])
disp(['Path length : ',num2str(length(p)), ' squares'])
disp('Now, you will increase/decrease the cost in traversability of a small region.')
disp('As indication, for example, that this part of terrain is rougher/smoother than the rest')
disp('The default cost of traveling threw a cell is 1')
disp('Press any key in the command window to continue')
pause()
%% Terrain modification
bored_terrainChange=false;
while ~bored_terrainChange
    modCost = input('Please enter your modified cost (cost>0): ');
    while (modCost <= 0) || ~isnumeric(modCost)
        modCost = input('You have entered a wrong cost, please try again (cost>0): ');
    end
    msgbox('Caution, this can also change the values of the wall');
    pause(2)
    terrainMod=round(ginput(2));
    close
    pause(0.1)
    for y=min(terrainMod(:,2)):max(terrainMod(:,2))
        for x=min(terrainMod(:,1)):max(terrainMod(:,1))
            ds.modify_cost([x,y], modCost);
        end
    end
    startReCompute=tic;
    ds.plan()
    timeToReCompute=toc(startReCompute);
    ds.path(start); hold on;
    x=[terrainMod(1,1),terrainMod(2,1), terrainMod(2,1),terrainMod(1,1),terrainMod(1,1)];
    y=[terrainMod(1,2),terrainMod(1,2), terrainMod(2,2),terrainMod(2,2),terrainMod(1,2)];
    plot(x,y,'w'); hold off;
    p=ds.path(start);
    disp(['Planning took ', num2str(timeToCompute), ' sec'])
    disp(['Replanning took ', num2str(timeToReCompute), ' sec'])
    disp(['Path length : ',num2str(length(p)), ' squares'])
    
    choice = questdlg('Would you like to change another part of the terrain cost ?', ...
    'Terrain cost change', ...
    'Yes','No','Yes');
    % Handle response
    switch choice
        case 'Yes'
            disp(' ')
            disp('-------------------------------')
            disp(' ')
            disp('Ok ! Lets change a new part of the terrain')
        case 'No'
            bored_terrainChange=true;
    end
end
end