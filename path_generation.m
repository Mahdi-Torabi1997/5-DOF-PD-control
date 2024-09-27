% PATH GENERATION
%
% OUTPUT COLUMNS
% column 1: x position
% column 2: y position
% column 3: z position
% column 4: alpha, Euler Z angle
% column 5: beta,  Euler Y angle
% column 6: gamma, Euler Z angle
% column 7: scalar speed at position (NaN if unspecified)
% column 8: amount of time to get to next point (aka duration)

function P_ee = path_generation(pancake, num_pancakes)

    unit_time = 0.25;  % in seconds

    num_sets = num_pancakes/3;

    P_ee(1,:) = ready_position;
    for set = 1:num_sets
        [P_ee, pancake] = flip_pancake(P_ee, pancake);  % left pancake
        [P_ee, pancake] = flip_pancake(P_ee, pancake);  % middle pancake
        [P_ee, pancake] = flip_pancake(P_ee, pancake);  % right pancake
        [P_ee, pancake] = serve_pancake(P_ee, pancake);  % left pancake
        [P_ee, pancake] = serve_pancake(P_ee, pancake);  % middle pancake
        [P_ee, pancake] = serve_pancake(P_ee, pancake);  % right pancake
    end
    P_ee(end+1,:) = ready_position;
    
    P_ee(:,8) = unit_time*P_ee(:,8);

    function [x, y, z] = get_pos(pancake, position)
        if position == "start"
            x = pancake.startPos(1);
            y = pancake.startPos(2);
            z = pancake.startPos(3);
        elseif position == "end"
            x = pancake.endPos(1);
            y = pancake.endPos(2);
            z = pancake.endPos(3);
        end
    end

    function P = ready_position
        % output: 1 point
        P = [265 230 110 180 -90 0 0 4];
    end

    function P = retrieve_pancake(pancake)
        % output: 3 points
        [x,y,z] = get_pos(pancake, "start");
        P(1,:) = [x-115 y  z+5 180 -90 0  0   2 ];  % pancake pickup start
        P(2,:) = [  x   y   z  180 -90 0  0   4 ];  % under pancake
        P(3,:) = [  x   y  90  180 -90 0 NaN  4 ];  % pickup pancake
    end

    function P = intermediate_point(direction)
        % output: 1 point
        if direction == "serving"
            P = [380 230 120 180 -90 0 NaN 4];
        elseif direction == "returning"
            P = [265 230 180 180 -90 0 NaN 4];
        end
    end

    function [P_ee, pancake] = flip_pancake(P_ee, pancake)
        P(1:3,:) = retrieve_pancake(pancake);
        [x,y,z] = get_pos(pancake, "end");
        P(4,:) = [ x y+80 100 180 -90  0  NaN 2 ];  % start flip
        P(5,:) = [ x y+40 120 180 -90  90 NaN 1 ];  % mid flip
        P(6,:) = [ x   y   z  180 -90 180  0  4  ];  % onto griddle
        P(7,:) = [265  y  100 180 -90  0   0  4  ];  % release pancake
        [P_ee, pancake] = update_points(P_ee, P, pancake);
    end

    function [P_ee, pancake] = serve_pancake(P_ee, pancake)
        P(1:3,:) = retrieve_pancake(pancake);
        P(4,:) = intermediate_point('serving');
        [x,y,z] = get_pos(pancake, "end");
        P(5,:) = [  x   y  z   180 -90 0  0  2];  % pancake on plate
        P(6,:) = [ x-75 y  z   180 -90 0 NaN 2];
        P(7,:) = [x-115 y z+10 180 -90 0  0  4];  % release pancake
        P(8,:) = intermediate_point('returning');
        [P_ee, pancake] = update_points(P_ee, P, pancake);
    end

    function [P_ee, pancake] = update_points(P_ee, P_new, pancake)
        pancake = update_pancake(pancake);
        P_ee = [P_ee; P_new];
    end
end

