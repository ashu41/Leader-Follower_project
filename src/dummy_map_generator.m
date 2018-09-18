function [map] = dummy_map_generator()

%defining map size, initially everything is a free space
map = robotics.BinaryOccupancyGrid(10,10);

% inserting obstacles
x = [2:7 3:5 1:8 5:10];
y= zeros(length(x),1);
y(1:6) = 1;
y(7:9) = 3;
y(10:17) = 6;
y(18:23) = 8;
setOccupancy(map,[x' y],1);

%visualise map
show(map);
hold on;
plot(x,y,'xr','MarkerSize', 20)
grid on
% set(gca,'XTick',0:0.2:10,'YTick',0:0.2:10)
% xlim([4 6])
% ylim([4 6])
end