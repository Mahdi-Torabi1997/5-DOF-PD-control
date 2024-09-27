%  Continuous Path Trajectory Generation (Via Points Scheme)
%  with Matching Velocity and Acceleration
%
%  This code will generate a trajectory using cubic polynomials with a path
%  that contains n points and k joints in cartesian coordinates.
%  The velocity and acceleration are matched between swegments.
%
%  INPUT: The file requires an k x n matrix, i.e., where k is the number of 
%  joints or cartesian coordinates and n are all the points (initial, 
%  intermidiate and final), a vector of size n-1 that specifies the 
%  duration of each segment, and the increment of time within each segment.
%
%  OUTPUT: The file will output three matrices [disp,vel,acc] that will 
%  describe the  trajectory of the joint or cartesian coordinate - 
%  displacement, velocity and acceleration, respectively. 

function [pos, vel, acc, time] = via_points(points, segment_duration, step, t_initial_offset)
    
    [num_joints, num_points] = size(points);
    num_segments = num_points - 1;
    num_via_points = num_points - 2;
    m = 4*num_segments;  % number of unknowns or number of constants
    A = zeros(m);

    % Displacement Constraints
    for s = (1:num_segments) - 1
        tf = segment_duration(s+1);
        i = 2*s+1;  j = 4*s+1;
        A(  i, j:j+3) = [ 1,  0,    0,    0 ]; 
        A(i+1, j:j+3) = [ 1, tf, tf^2, tf^3 ];
    end

    % Velocity and Acceleration Constraints
    for s = (1:num_via_points) - 1
        tf = segment_duration(s+1);
        i = 2*s+m/2+1;  j = 4*s+1;
        A(  i, j:j+7) = [ 0, 1, 2*tf, 3*tf^2, 0, -1,  0, 0 ]; 
        A(i+1, j:j+7) = [ 0, 0,    2,   6*tf, 0,  0, -2, 0 ]; 
    end

    % Initial Velocity Conditions
    A(m-1,1:4) = [0, 1, 0, 0]; 
    
    % Final Velocity Conditions
    tfn = segment_duration(end-1);
    A(m,m-3:m) = [0, 1, 2*tfn, 3*tfn^2];

    % Tragectory Generation for Every Joint
    [pos, vel, acc] = deal(zeros(num_joints, num_points));
    time = zeros(1, num_points);
    bk = zeros(m,1);
    for k = 1:num_joints 
        for s = (1:num_segments) - 1
            bk(2*s+1) = points(k,s+1);
            bk(2*s+2) = points(k,s+2);
        end
        
        a = reshape( A\bk, 4, [] )';

        i = 1;
        t_offset = t_initial_offset;
        for s = 1:num_segments
            for t = 0:step:segment_duration(s)
                pos(k,i)  =    a(s,1) +   a(s,2)*t +   a(s,3)*t^2 + a(s,4)*t^3;
                vel(k,i)  =    a(s,2) + 2*a(s,3)*t + 3*a(s,4)*t^2;
                acc(k,i)  =  2*a(s,3) + 6*a(s,4)*t;
                time(1,i) =  t + t_offset;
                i = i + 1;
            end
            t_offset = t_offset + segment_duration(s);
        end
    end

return

