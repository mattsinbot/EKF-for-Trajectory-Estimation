clc
close all
clear all

% Grid definition
x = 0:.1:10;
y = 0:.1:10;

% Maximum nunber of grid points
maxind = x(1,end)/0.1;

% Start point
x_agent = [1];
y_agent = [1];

Act = zeros(length(x),length(y));
V = zeros(length(x),length(y));

% Plot the grid as per definition
[xx,yy] = meshgrid(x,y);

figure;
plot(xx,yy,'k.')
hold on;

% Goal coordinate
x_goal = 9;
y_goal = 9;

x_start = 1;
y_start = 1;

[i_x,i_y]= xy_to_indices(x_goal,y_goal);
ix_goal = i_x;
iy_goal = i_y;

i_do_nth = [i_x,i_y];

Act(i_x,i_y) = 0;
plot(x_goal,y_goal,'ro')

% Non buffered obstacles to plot
obs_non_buffered = [6 1.5 10 2.5;
                    6.5 5.5 10 6.5;
                    0 8.5 3.5 10;
                    0 3 1 7;
                    1 3 2 4;
                    5.2 1.5 6 8.5;
                    3 5 4 6.5;
                    7 9 8 10;
                    3 2 4 4];

% Need to solve the problem
% obs_locs = [6 1.5 10 2.5;
%             6.5 5.5 10 7;
%             0 8 4 10;
%             0 3 1.5 7;
%             1 3 2.5 4;
%             5 1.5 6 9;
%             3 5 4 7;
%             7 8.5 8 10;
%             3 2 4 4];

obs_locs = [6 1.5 10 2.5;
                    6.5 5.5 10 6.5;
                    0 8.5 3.5 10;
                    0 3 1 7;
                    1 3 2 4;
                    5.2 1.5 6 8.5;
                    3 5 4 6.5;
                    7 9 8 10;
                    3 2 4 4];

for i = 1:size(obs_locs,1)
     patch( [obs_locs(i,1) obs_locs(i,1)  obs_locs(i,3) obs_locs(i,3)  ], ...
        [obs_locs(i,2) obs_locs(i,4)  obs_locs(i,4) obs_locs(i,2)  ],'r' );    
    
    [ix_obs_st,iy_obs_st]= xy_to_indices( obs_locs(i,1), obs_locs(i,2));
    [ix_obs_en,iy_obs_en]= xy_to_indices( obs_locs(i,3), obs_locs(i,4));
    
    Act(ix_obs_st:ix_obs_en,iy_obs_st:iy_obs_en) =  200;
end

close all
figure;
% Costs at the obstacle grids
V = 4000*ones(size(Act));
gam = .9;
filename = ['Value_growth' num2str(i) '.gif'];
changed = 1;
i_V = 1;
while changed == 1
    changed = 0;
    V_old = V;
    for i_x = 1:length(x)
        for i_y = 1:length(y)
            
            if (i_x == ix_goal) &&(i_y == iy_goal)
                if V(i_x,i_y) > 0
                    changed = 1;
                    V(i_x,i_y) = 0;
                end
            end
            
            if Act(i_x,i_y) ~= 200;
                iv_x = [1 -1 0 0 1 -1 1  -1];
                iv_y = [0 0 -1 1  1 -1 -1 1];
                V_new = [];
                for i_v = 1:8,
                    val = check_ind(i_x+iv_x(i_v),i_y+iv_y(i_v),maxind);
                    if val == 1
                        V_new  = V(i_x+iv_x(i_v),i_y+iv_y(i_v)) + 10*sqrt(iv_x(i_v)^2+iv_y(i_v)^2);
                        
                        if V_new< V(i_x,i_y)
                            V(i_x,i_y) = V_new;
                            changed = 1;
                            
                        end
                    end
                end
                
                
            else
                V(i_x,i_y) = 4000;
            end
        end
    end
    
    
    surf(yy,xx,V_old);xlabel('X');ylabel('Y');zlabel('Value')
    title(['Value after ' num2str(i_V) ' iterations.'])
    % axes of cost plot
    axis([0 10 0 10 -200 5100])
    view(i_V*3,30);
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i_V == 1;
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.1);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
    end
    i_V = i_V+1;
end

for i = i_V:i_V+60
    surf(yy,xx,V);xlabel('X');ylabel('Y');zlabel('Value')
    title(['Value after ' num2str(i) ' iterations (Converged).'])
    % axes of cost plot
    axis([0 10 0 10 -200 5100])
    view(i*3,30);
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
    
end

filename = ['Obs_Avoidance' num2str(i) '.gif'];
close all

figure;
plot(xx,yy,'k.')
hold on;
for i = 1:size(obs_locs,1)
    patch( [obs_non_buffered(i,1) obs_non_buffered(i,1)  obs_non_buffered(i,3) obs_non_buffered(i,3)  ], ...
        [obs_non_buffered(i,2) obs_non_buffered(i,4)  obs_non_buffered(i,4) obs_non_buffered(i,2)  ],'g' );
end

% for i = 1:size(obs_locs,1)
%     patch( [obs_locs(i,1)-0.2 obs_locs(i,1)-0.2  obs_locs(i,3)-0.2 obs_locs(i,3)-0.2], ...
%         [obs_locs(i,2)-0.2 obs_locs(i,4)-0.2  obs_locs(i,4)-0.2 obs_locs(i,2)-0.2],'g' );
% end

Va = [];
hold on;
i_move = 1;
k = 1;

for i = 1:length(x_agent)
    [i_x,i_y]= xy_to_indices(x_agent(i),y_agent(i));
    stop_mov = 0;
    while stop_mov == 0
        iv_x = [1 -1 0 0 1 -1 1  -1];
                iv_y = [0 0 -1 1  1 -1 -1 1];
        for i_v = 1:8,
            Va(i_v) = V(i_x+iv_x(i_v),i_y+iv_y(i_v)) + 10*sqrt(iv_x(i_v)^2+iv_y(i_v)^2);
        end
        
        [V_min , i_vmin]= min(Va);
        x_agent(i) = x( i_x+iv_x(i_vmin));
        y_agent(i) = y( i_y+iv_y(i_vmin));
        path_cord(1,k) = x_agent(i);
        path_cord(2,k) = y_agent(i);
        plot(x_agent(i),y_agent(i),'b*')
        
        if (i_x==ix_goal)&&(i_y==iy_goal)
            stop_mov = 1;
        end
        
         frame = getframe(1);
         im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i_move == 1;
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.1);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
        end
        i_move = i_move+1;
        [i_x,i_y]= xy_to_indices(x_agent(i),y_agent(i));
        pause(0.01);
        k = k + 1;
    end
end

% Gradient descent Method to smooth the path
newPath = smoothPath(path_cord');

% Get the trajectory of the smoothed path
[timeInter, xInter, yInter] = pointToTrajectory(newPath);

% get a text data file
% print the trajectory into a text file
traj = [timeInter; xInter; yInter];
fileID = fopen('smooth_way_points.txt','w');
fprintf(fileID,'%6.2f %12.8f %12.8f\n',traj);
fclose(fileID);