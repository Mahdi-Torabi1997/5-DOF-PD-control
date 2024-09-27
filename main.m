% This file shows a manipulator flipping pancakes on a griddle and serving
% the pancakes onto a plate.
% 
% Ensure that the following files are in the working directory.
%  - manipulator.mat
%  - environment.mat
%  - update_pancake.m
%  - path_generation.m
%  - via_points.m
%  - plot_trajectory.m

%#ok<*SAGROW,*AGROW,*UNRCH>

%% 1. INITIALIZATION
clc; clear; close all;
% ======================================================================= %
save_animation = false;    % SET TO TRUE TO SAVE THE ANIMATION AS A VIDEO
show_comet = true;         % SET TO TRUE TO SHOW COMET DURING ANIMATION
show_traj_plots = false;   % SET TO TRUE TO SHOW TRAJECTORY PLOTS
% ======================================================================= %

if ~exist('output', 'dir'), mkdir('output'); end

% 1.1 Define size of figure and create figure handle (DO NOT MODIFY)
set(0,'Units','pixels');
tbh = 40;  % taskbar height
dim = get(0,'ScreenSize');
main_fig = figure('doublebuffer', 'on', ...
                  'OuterPosition', [dim(3)/2,tbh,dim(3)/2,(dim(4)-tbh)/2], ...
                  'Name', '3D Object', 'NumberTitle', 'off');
set(main_fig,'color', [1 1 1])  % Background Colour

% 1.2 Define the figure buffer, initial lighting, and aspect ratio
set(main_fig,'Renderer','zbuffer','doublebuffer','off')
light('color',[.5,.5,.5],'position',[0,1,3],'Style','infinite')
daspect([1 1 1]);

% 1.3 Axes (TO MODIFY Make sure your axes fit within the region) 
axis([-150 550 -100 1000 -60 200]);  %To be changed to include workspace
view(50,30);                         %To be changed to view from other angle
zoom(1)                              %To be changed to zoom in/out 
xlabel('X'); ylabel('Y'); zlabel('Z');
axh = get(main_fig, 'CurrentAxes');
set(axh, 'OuterPosition', [0,0, 1, 1.3])


%% 2.1 LOAD ENVIRONMENT OBJECTS
load('environment.mat', 'rod', 'post', 'griddle', 'plate', 'pancake')
num_pancakes = 3*5;  % this is the maximum number of pancakes the 
                     % manipulator can stack on the plate (in sets of 3).

% Patching all faces
rod_patch     = create_patch(rod,     "#808080");
post_patch(1) = create_patch(post,    "#808080");
post_patch(2) = create_patch(post,    "#808080");
griddle_patch = create_patch(griddle, "#353535");
plate_patch   = create_patch(plate,   "#A9A9A9");
for i = 1:num_pancakes
    pancake_patch(i) = create_patch(pancake, "#F1C095"); 
end

% Position environmental objects
post(2).V = post(1).V + [0 900 0];
post(1).V = post(1).V + [0 -60 0];
griddle.V = griddle.V + [255 230 -60];
plate.V   = plate.V   + [380 80 -60];
for i = 1:num_pancakes
    x = 380 + 400*floor((i-1)/3);
    y = 355+200*mod(i-1,3);
    pancake_V{i} = pancake.V + [x y -10];
end

% Display objects on figure
set(rod_patch,     'Vertices', rod.V);
set(post_patch(1), 'Vertices', post(1).V);
set(post_patch(2), 'Vertices', post(2).V);
set(griddle_patch, 'Vertices', griddle.V);
set(plate_patch,   'Vertices', plate.V);
for i = 1:num_pancakes
    set(pancake_patch(i), 'Vertices', pancake_V{i}); 
end

% Set initial pancake conditions
pancake = update_pancake(pancake, 'reset');

% Clear unneeded variables
clear x y rod post griddle plate
clear rod_patch post_patch griddle_patch plate_patch


%% 2.2 LOAD MANIPULATOR PARTS
load('manipulator.mat', 'UpperArm', 'LowerArm', 'Hand', 'EndEffector')
link(1) = UpperArm;
link(2) = LowerArm;
link(3) = Hand;
link(4) = EndEffector;

% Patching all faces
link_patch(1) = create_patch(link(1), "#0072BD");  % Upper Arm
link_patch(2) = create_patch(link(2), "#D95319");  % Lower Arm
link_patch(3) = create_patch(link(3), "#EDB120");  % Hand
link_patch(4) = create_patch(link(4), "#C0C0C0");  % End-Effector

% Length of the links
L1 = 110;  % Upper Arm
L2 = 110;  % Lower Arm
L3 = 70;   % Hand
L4 = 130;  % End-Effector

% Clear unneeded link variables
clear UpperArm LowerArm Hand EndEffector


%% LIGHTING
% (Change the position vector if the figure is too bright or dark)
light('color',[.99,.99,.99],'position',[1,-1,1],'Style','infinite')
lighting gouraud


%% 3. PATH GENERATION
% 3.1 Single Pose (Project Phase II)
%          x   y   z  alpha beta gamma (Euler Z-Y-Z)
% P_ee = [265 355 175  180   -90   0   ];  % Ready position 2
% P_ee = [265 355 -10  180   -90   0   ];  % Pickup position 1
% P_ee = [380 355 -10  180   -90   0   ];  % Pickup position 2
% P_ee = [380  80 -40  180   -90   0   ];  % Plate position

% 3.2 Path Generation 
% Spatial Coordinates of the end-effector 
P_ee = path_generation(pancake, num_pancakes);


%% 4. INVERSE KINEMATICS
% Variables for Inverse Kinematics
% Row vectors
x = P_ee(:,1)';
y = P_ee(:,2)';
z = P_ee(:,3)';
alpha = P_ee(:,6)';

% 4.1 Inverse Kinematics
d1 = y;
c3 = (-L1^2-L2^2+L3^2+2*L3*L4-2*L3*x+L4^2-2*L4*x+x.^2+z.^2)/(2*L1*L2);
s3 = sqrt(1-c3.^2);
theta3 = atan2d(s3, c3);
theta2 = atan2d(x-L4-L3, z) - atan2d(L2*s3, L1+L2*c3);
theta4 = -theta3-theta2;
theta5 = alpha;

% Compile Joints
% Rows:    joints
% Columns: points
joints = [d1; theta2; theta3; theta4; theta5];
num_joints = size(joints, 1);
clear x y z alpha c3 s3


%% 5. TRAJECTORY GENERATION
% 5.1 Trajectory Generation
[D, V, A, T] = deal([]);
stepsize = 0.05;  % delta t
t_offset = 0;
stop_points = find(P_ee(:,7)==0);
for i = 1:length(stop_points)-1
    section = stop_points(i):stop_points(i+1);
    duration = P_ee(section,8);
    points = joints(:,section);
    [pos,vel,acc,time] = via_points(points, duration, stepsize, t_offset);
    t_offset = time(end);
    D = [D, pos]; V = [V, vel]; A = [A, acc]; T = [T, time];
end

% Plot Trajectory
animate_trajectory = false;
single_figure = true;
fig_position = [0,tbh,dim(3),dim(4)-tbh];
trajectory = plot_trajectory(D, V, A, T, show_traj_plots, single_figure, fig_position, animate_trajectory);
if trajectory.save_animation
    trajectory.animation.FrameRate = 1/stepsize;
    open(trajectory.animation);
end

% Plot comet
comet = comet3(axh, '#1E90FF', 100);
if show_comet
    comet_fig = figure('OuterPosition', [dim(3)/2,(dim(4)+tbh)/2,dim(3)/2,(dim(4)-tbh)/2], ...
                       'Name', 'End-Effector History', 'NumberTitle', 'off');
    set(comet_fig,'color', [1 1 1])
    axis([-150 550 -100 1000 -60 200]); view(50,30);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    daspect([1 1 1]);
    grid on;
    comet_axh = get(comet_fig, 'CurrentAxes');
    history = comet3(comet_axh, '#1E90FF', 10000);
    trajectory.history.FrameRate = 1/stepsize;
end

% Clear unneed variables
clear joints pos vel acc time t_offset section duration points stop_points
clear tbh dim fig_position single_figure

% Show main fig up front.
figure(main_fig)

% EXAMPLES FOR PARTS II 
% 5.2 Single Pose (Part II example)
% D = [d1; theta2; theta3; theta4; theta5];
% D = [  0;   0;   0;  -90; 0];  % Erect position
% D = [355; -45;  90;  -45; 0];  % Ready position 1
% D = [355;  25; 145; -170; 0];  % Pickup position 1
% D = [355;  61;  64; -125; 0];  % Pickup position 2


%% 6. FORWARD KINEMATICS / DISPLACEMENT AND ROTATION OF HANDLE OBJECTS

% Joint Variables
d1     = D(1,:);
theta2 = D(2,:);
theta3 = D(3,:);
theta4 = D(4,:);
theta5 = D(5,:);

ee = 6; % == Link Parameters of DH table (angles in degrees) ==
        %         i-1  i   alpha   a    d    theta
DHtable   =   {    0   1    -90    0    d1    -90    ;   % Linear Position
                   1   2     0     0    0    theta2  ;   % Upper Arm
                   2   3     0     L1   0    theta3  ;   % Lower Arm
                   3   4     0     L2   0    theta4  ;   % Hand
                   4   5    -90    0    L3   theta5  ;   % Handle End
                   5   ee    0     0    L4     0    };   % End-Effector
DHtable = DHtable(:,3:6);  % Remove index columns
[ alpha0, a0, d1,  theta1  ] = DHtable{1,:};
[ alpha1, a1, d2,  theta2  ] = DHtable{2,:};
[ alpha2, a2, d3,  theta3  ] = DHtable{3,:};
[ alpha3, a3, d4,  theta4  ] = DHtable{4,:};
[ alpha4, a4, d5,  theta5  ] = DHtable{5,:};
[ alpha5, a5, dee, thetaee ] = DHtable{ee,:};
clear DHtable ee

% Animation
max_loops = 2;
if save_animation
    video = VideoWriter('output.mp4', 'MPEG-4'); 
    video.FrameRate = 1/stepsize;
    open(video);
end
disp('<strong>ENTERING LOOP.</strong> Close main figure to exit loop.')
for loop = 1:max_loops, if ~ishandle(axh), break; end
    clearpoints(comet)
    pancake = update_pancake(pancake, 'reset');
    for i = 1:num_pancakes
        set(pancake_patch(i), 'Vertices', pancake_V{i});
    end

    for i = 1:size(D,2), if ~ishandle(axh), break; end
        % DH parameters to transforms
        T_01  = tmat( alpha0, a0, d1(i), theta1    );
        T_12  = tmat( alpha1, a1, d2,    theta2(i) );
        T_23  = tmat( alpha2, a2, d3,    theta3(i) );
        T_34  = tmat( alpha3, a3, d4,    theta4(i) );
        T_45  = tmat( alpha4, a4, d5,    theta5(i) );
        T_5ee = tmat( alpha5, a5, dee,   thetaee   );

        % Forward Kinematics (Homogeneous Tranforms)
        T_02  = T_01*T_12;
        T_03  = T_02*T_23;
        T_04  = T_03*T_34;
        T_05  = T_04*T_45;
        T_0ee = T_05*T_5ee;

        % Move Links of Manipulator.
        newV{1} = transform(T_02,  link(1).V');  % Upper Arm
        newV{2} = transform(T_03,  link(2).V');  % Lower Arm
        newV{3} = transform(T_04,  link(3).V');  % Hand
        newV{4} = transform(T_0ee, link(4).V');  % End-Effector
        
        % Show End-Effector Position History
        if show_comet
            addpoints(comet, T_0ee(1,4), T_0ee(2,4), T_0ee(3,4))
            addpoints(history, T_0ee(1,4), T_0ee(2,4), T_0ee(3,4))
        end
        
        % Update graphical positions of manipulator links 
        set(link_patch(1), 'Vertices', newV{1}');  % Upper Arm
        set(link_patch(2), 'Vertices', newV{2}');  % Lower Arm
        set(link_patch(3), 'Vertices', newV{3}');  % Hand
        set(link_patch(4), 'Vertices', newV{4}');  % End-Effector
        
        % Update trajectory plots
        if trajectory.animate == true
            for j = 1:num_joints
                addpoints(trajectory.D(j), T(i), D(j,i))
                addpoints(trajectory.V(j), T(i), V(j,i))
                addpoints(trajectory.A(j), T(i), A(j,i))
            end
        end
        if trajectory.save_animation
            writeVideo(trajectory.animation, getframe(trajectory.fig));
            writeVideo(trajectory.history, getframe(comet_fig));
        end
        
        % Moving pancake
        if isequal(round(T_0ee(1:3,4)), pancake.endPos)
            newV{5} = move_pancake(pancake.V, T_0ee);
            set(pancake_patch(pancake.num), 'Vertices', newV{5}')
            pancake = update_pancake(pancake);
        elseif pancake.move || isequal(round(T_0ee(1:3,4)), pancake.startPos)
            pancake.move = true;
            newV{5} = move_pancake(pancake.V, T_0ee);
            set(pancake_patch(pancake.num), 'Vertices', newV{5}')
        end
        
        drawnow  % Draw objects to their new poisitons
        
        % Save frame for animation 
        if save_animation && ishandle(axh)
            writeVideo(video, getframe(main_fig));
        end
        
        % Move next set of pancakes
        if pancake.nextSet ...
                && isequal(round(T_0ee(1:3,4)), [265; 230; 180]) ...
                && mod(pancake.num,3) == 1 ...
                && pancake.num < num_pancakes ...
                && pancake.task == "flip"
            for s = 0:40:400
                for pnck = pancake.num:pancake.num+2
                    x = 380 + 400 - s;
                    y = 355+200*mod(pnck-1,3);
                    newV{5} = pancake.V + [x y -10];
                    set(pancake_patch(pnck), 'Vertices', newV{5})
                end
                drawnow
                if save_animation && ishandle(axh)
                    writeVideo(video,getframe(main_fig));
                end
            end
            pancake.nextSet = false;
            trajectory.animate = false;
            if trajectory.save_animation
                trajectory.save_animation = false;
                close(trajectory.animation);
            end
        end
    end
    if save_animation && ishandle(axh)
        close(video);
        save_animation = false;
    end
end
disp('Exited loop.')
if ~ishandle(axh), close all; end
if save_animation, close(video); end
if trajectory.save_animation, close(trajectory.animation); end
disp('<strong>Project successfully ran</strong>')


%% Functions
function p = create_patch(obj, color)
    p = patch(               ...
        'Faces', obj.F,      ...
        'Vertices', obj.V,   ...
        'EdgeColor', 'none', ...
        'FaceColor', color   ...
    );
end

function comet = comet3(axes_handle, line_color, max_points)
    comet = animatedline(              ...
        axes_handle,                   ...
        'Color', line_color,           ...
        'MaximumNumPoints', max_points ...
    );
end

function T = tmat(alpha, a, d, theta)
    c  = cosd(theta); s  = sind(theta);
    ca = cosd(alpha); sa = sind(alpha);
    T = [  c     -s    0     a    ;
          s*ca  c*ca  -sa  -sa*d  ;
          s*sa  c*sa   ca   ca*d  ;
           0     0     0     1   ];
end

function newV = transform(T, V)
    newV = T(1:3,:)*[V; ones(1, size(V,2))];
end

function newV = move_pancake(pancake_V, T_0ee)
    % Transform from end-effector to pancake
    T_eepancake = [ 0  0  1  0 ;
                    0 -1  0  0 ;
                    1  0  0  0 ;
                    0  0  0  1 ];
    
	% Transform from base frame to pancake
	T_0pancake = T_0ee*T_eepancake;
    
    V = pancake_V;
 	newV = transform(T_0pancake, V');
end
