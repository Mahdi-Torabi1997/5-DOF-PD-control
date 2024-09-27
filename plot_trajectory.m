function trajectory = plot_trajectory(D, V, A, T, show_plots, single_figure, fig_position, animate)
    t_max = 35;
    num_joints = size(D, 1);
    if single_figure || not(show_plots)
        trajectory.fig = figure('OuterPosition', fig_position);
        tiledlayout(3, num_joints, 'Padding', 'tight');
        for j = 1:num_joints
            tile_axh = nexttile; title(sprintf("Joint %d Displacement", j))
            xlim([0, t_max]); ylim([min(D(j,:)), max(D(j,:))]);
            trajectory.D(j) = animatedline(tile_axh,"Color","r");
        end
        for j = 1:num_joints
            tile_axh = nexttile; title(sprintf("Joint %d Velocity", j))
            xlim([0, t_max]); ylim([min(V(j,:)), max(V(j,:))]);
            trajectory.V(j) = animatedline(tile_axh,"Color","g");
        end
        for j = 1:num_joints
            tile_axh = nexttile; title(sprintf("Joint %d Acceleration", j))
            xlim([0, t_max]); ylim([min(A(j,:)), max(A(j,:))]); xlabel('time (s)');
            trajectory.A(j) = animatedline(tile_axh,"Color","b");
        end
        if not(show_plots)
            trajectory.fig.Visible = 'off';
        end
    else
        x = fig_position(1);
        y = fig_position(2);
        w = fig_position(3)/2;
        h = fig_position(4)/4;
        for j = num_joints:-1:1
            trajectory.fig(j) = figure('Position', [x, y, w, h], ...
                                       'Name', sprintf("Joint %d", j), ...
                                       'NumberTitle', 'off');
            tiledlayout(1, 3, 'Padding', 'compact');
            
            if j==1, units="mm"; else, units="deg"; end
            
            tile_axh = nexttile;
            xlim([0, t_max]); ylim([min(D(j,:)), max(D(j,:))]);
            xlabel('time (s)'); ylabel(sprintf("Joint %d Displacement (%s)", j, units));
            trajectory.D(j) = animatedline(tile_axh,"Color","r");
            
            tile_axh = nexttile;
            xlim([0, t_max]); ylim([min(V(j,:)), max(V(j,:))]);
            xlabel('time (s)'); ylabel(sprintf("Joint %d Velocity (%s/s)", j, units));
            trajectory.V(j) = animatedline(tile_axh,"Color","g");
            
            tile_axh = nexttile;
            xlim([0, t_max]); ylim([min(A(j,:)), max(A(j,:))]); 
            xlabel('time (s)'); ylabel(sprintf("Joint %d Acceleration (%s/s^2)", j, units));
            trajectory.A(j) = animatedline(tile_axh,"Color","b");
        end
    end
    
    if not(animate) || not(show_plots)
        for j = 1:num_joints
            addpoints(trajectory.D(j), T(:), D(j,:))
            addpoints(trajectory.V(j), T(:), V(j,:))
            addpoints(trajectory.A(j), T(:), A(j,:))
        end
    end
    
    trajectory.animate = animate;
    trajectory.save_animation = false;
    trajectory.animation = VideoWriter('output.mp4', 'MPEG-4');
    trajectory.history = VideoWriter('output.mp4', 'MPEG-4');
end

