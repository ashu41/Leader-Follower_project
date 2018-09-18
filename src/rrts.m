function P = rrts(pose,obst,param,p_start,p_goal)

global iterations
map = dummy_map_generator();
P = [];
rrt = {};
rrt = AddNode(rrt,p_start,0);
iter = 1;

while iter <= param.maxiters
    
    if mod(iter,50) == 0
        tf=map;
        
    end
    
    p = rand(2,1); % random p
%     p(1,1) = floor(p(1,1)*11-5);
%     p(2,1) = floor(p(2,1)*16-4);
    p(1,1) = p(1,1)*10-5;
    p(2,1) = p(2,1)*15-4;
    pose.p = p;
    col = InCollision_Node(pose,obst);
    if col == 1 % skip to next iteration
%         circle(p(1,1),p(2,1),0.1,'r');
        iter = iter + 1;
        continue
    end
    % do something if valid coordinate
    for i=1:length(rrt)
        dist = norm(rrt{i}.p - p);
        if (i==1) || (dist < mindist)
            mindist = dist;
            imin = i;
            l = rrt{i}.p;
        end
    end
    col = InCollision_Edge(pose,obst,p,l,param.res); %check for valid edge
    if col == 1 % skip to next iteration if not valid edge
%         circle(p(1,1),p(2,1),0.1,'r');
        iter = iter + 1;
        continue 
    end
    rrt = AddNode(rrt,p,imin); % add p to T with parent l
    dist = norm(p-p_goal);
    %display(iter,dist,length(rrt))
    fprintf('Nodes:   %d, Distance: %.1f, Iterations: %d/1000\n',length(rrt),dist,iter)
%     circle(p(1,1),p(2,1),rob.ballradius,'b');
%     circle(p(1,1),p(2,1),0.1,'b');
    plot([p(1,1);rrt{imin}.p(1,1)],[p(2,1);rrt{imin}.p(2,1)],'m','LineWidth',3);
    if (dist < param.thresh)
        col = InCollision_Edge(pose,obst,p,p_goal,param.res); %check for valid edge
        if col == 1 % skip to next iteration if not valid edge
            iter = iter + 1;
            continue 
        end
        iterations = iter
        % add qgoal to T with parent q and exit with success
        rrt = AddNode(rrt,p_goal,length(rrt));
        % construct Q here:
        i = length(rrt);
        P(:,1) = rrt{i}.p;
        while 1
            i = rrt{i}.iPrev;
            if i == 0
                return
            end
            P = [rrt{i}.p P];
        end
    end

    iter = iter + 1;
end
iterations = iter - 1

function col = InCollision_Node(rob,obst)

global checkcount;
checkcount = checkcount + 1;
col = 0;
numobst = length(obst.ball);
    for j=1:numobst % check for robot-obstacle collision
        % calculate distance between ith robot ball center and jth obstacle
        % ball center
        %dist = sqrt((rob.ball{i}.p(1)-obst.ball{j}.p(1))^2+(rob.ball{i}.p(2)-obst.ball{j}.p(2))^2+(rob.ball{i}.p(3)-obst.ball{j}.p(3))^2);
        dist = norm(rob.p-obst.ball{j}.p);
        if dist < (rob.ballradius + obst.ball{j}.r)
            col = 1;
            return;
        end 
    end
%end

function col = InCollision_Edge(rob,obst,p1,p2,res)

col = 0;
d = norm(p1 - p2);
m = ceil(d/res);
t = linspace(0,1,m);
for i=2:(m-1)
    p = (1-t(i))*p1 + t(i)*p2; %calculate configuration
    rob.p = p;
    col = InCollision_Node(rob,obst); 
    if col == 1
        return;
    end
end

function P = SmoothPath(rob,obst,param,P) %was SmoothPath(rob,obst,param,m,Q)

%
P = P;
[n,m] = size(P);
clearvars n;
l = zeros(m,1);
for k=2:m
    l(k)=norm(P(:,k)-P(:,k-1)) + l(k-1); % find all of the straight-line distances
end
l_init = l(m);
iter = 1;
while iter <= param.smoothiters
    s1 = rand(1,1)*l(m); 
    s2 = rand(1,1)*l(m); 
    if s2 < s1
        temps = s1;
        s1 = s2;
        s2 = temps;
    end
    for k=2:m
        if s1 < l(k)
            i = k - 1;
            break;
        end
    end
    for k=(i+1):m
        if s2 < l(k)
            j = k - 1;
            break;
        end
    end
    if (j <= i)
        iter = iter + 1;
        continue;
    end
    t1 = (s1 - l(i))/(l(i+1)-l(i));
    gamma1 = (1 - t1)*P(:,i) + t1*P(:,i+1);
    t2 = (s2 - l(j))/(l(j+1)-l(j));
    gamma2 = (1 - t2)*P(:,j) + t2*P(:,j+1);
    col = InCollision_Edge(rob,obst,gamma1,gamma2,param.res); %check for valid edge
    if col == 1
        iter = iter + 1;
        continue;
    end
    newP = [P(:,1:i) gamma1 gamma2 P(:,j+1:m)];
    clearvars P;
    P = newP;
    [n,m] = size(P);
    clearvars n;
    l = zeros(m,1);
    for k=2:m
        l(k)=norm(P(:,k)-P(:,k-1)) + l(k-1);
    end
    iter = iter + 1;
end
