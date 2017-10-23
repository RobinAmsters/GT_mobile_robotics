% Clean up matlab
clearvars -except start goal
close all
clc

startup_rvc

load map1
bored_planning=false;
while ~bored_planning
    if (exist('start', 'var') == 1)  
        % Construct a questdlg with three options
        choice = questdlg('Would you like to keep the same start and goal position?', ...
            'Start and goal position', ...
            'Yes','No','Yes');
        % Handle response
        switch choice
            case 'Yes'
                disp('Your previous start and goal position will be used')
            case 'No'
                bug = Bug2(map);
                bug.plot()
                disp('Please choose your new start and goal position')
                title('Select start position')
                start=round(ginput(1));
                title('Select goal position')
                goal=round(ginput(1));
                close
                pause(0.5)
        end
    else
        bug = Bug2(map);
        bug.plot()
        disp('Please chose your new start and goal position')
        title('Select start position')
        start=round(ginput(1));
        title('Select destination position')
        goal=round(ginput(1));   
    end
    disp('Please choose your Path Planning method')
    choice = menuMod('Which pathplanning method would you like to use?', ...
    'Bug2','DXform','D*','Voronoi');
    if     choice == 1
        choice = 'Bug2';
    elseif choice == 2
        choice = 'DXform';
    elseif choice == 3
        choice = 'D*';
    elseif choice == 4
        choice = 'Voronoi';
    end
    
    % Handle response
    dispMsg=['Starting ',choice,' path planning method'];
    switch choice  
        case 'Bug2'
            disp(dispMsg)
            startup_bug(map,start,goal)
        case 'DXform'
            disp(dispMsg)
            startup_dxform(map, start, goal)
        case 'D*'
            disp(dispMsg)
            disp('Sit tight, D* planning is quite slow !')
            pause(3)
            close
            startup_dstar(map, start, goal)
        case 'Voronoi'
            disp(dispMsg)
            startup_voronoi(map, start, goal)
    end
    disp('Press any key in the Command Window to continue')
    pause()
    choice = questdlg('Would you like to try out another path planning method ?', ...
    'Path planning method change', ...
    'Yes','No','Yes');
    % Handle response
    switch choice
        case 'Yes'
            close
            disp(' ')
            disp('-------------------------------')
            disp(' ')
            disp('Ok ! Lets change the path planning method')
            close
            clearvars -except start goal bored_planning map
        case 'No'
            bored_planning=true;
    end
end

clear all
close all
clc