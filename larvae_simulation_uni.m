%% Description
% Simulates the behavior of larvae.
%Nicolette Prevost
%10/29/2018

%% Experiment Constants
%Run the simulation for a specific number of iterations.
iterations = 3000;
iCounter = 0;
% Defines radius where bots cannot enter (4 bots should fit around it).
food_radius = .16;
% Defines how many iterations a bot should stay at the center for.
eating_time = 60;

%% Variable Creation
% Get Robotarium object used to communicate with the robots/simulator.
rb = RobotariumBuilder();
% boundaries = [-x x -y y] limit.
boundaries = rb.boundaries;
% Get the number of available agents from the Robotarium. (N =
% rb.get_available_agents();)...
% ...Or, fix the number of robots.
N = 20;
% Initializes change_direction to false for all bots.
change_direction = zeros(1,N);
% Initializes array to track bot distance from center.
distance_from_center = zeros(1,N);
% Initializes array to track how long a bot has been near the food.
time_at_center = zeros(1,N);
% Initializes changing_direction to false for all bots.
changing_direction = zeros(1,N);
% Initializes set_still to false for all bots.
set_still = zeros(1,N);
% Build the Robotarium simulator object.
r = rb.set_number_of_agents(N).set_save_data(true).build();        
% Sets orange picture as backdrop.
img = imread('Orange At Center.jpg');
image(img,'XData',[-.16 .16],'YData',[-.16 .16]);
% Initialize x so that we don't run into problems later.  This isn't always
% necessary.
x = r.get_poses();
% Possible function to fix starting position.
%x = fix_poses(x, food_radius, boundaries);
r.step();
iCounter = iCounter + 1;

%% Set some parameters for use with the barrier certificates.  
% Create a barrier certificate for use with the above parameters
unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', 0.06, ...
    'ProjectionDistance', 0.03);
%Initiates random initial position in the robotarium arena
target = generate_initial_conditions(N, 'Width',...
    r.boundaries(2), 'Height', r.boundaries(4), 'Spacing', 0.2);
args = {'PositionError', 0.01, 'RotationError', 0.1};
init_checker = create_is_initialized(args{:});
automatic_parker = create_automatic_parking_controller2(args{:});

%  Generate array to determine which bots will start rotating left, and
%  which will start rotating right.
rot_r = logical(randi([0 1], 1,N));
rot_l = ~rot_r;
% Logical array for indexing later must be same size as dxu, but we don't
% want to edit the top rows [linear velocities], so make it all 0's.
top_row = logical(zeros(1,N));
rot_r = [top_row;rot_r];
rot_l = [top_row;rot_l];

%% Iterate over time 
for iCounter = iCounter:iterations
    x = r.get_poses();
    dxu = automatic_parker(x, target);
    dxu = unicycle_barrier_certificate(dxu, x);
    
    % Let's incorporate the "wiggling".
    % It is randomized which bots start rotating left, and which start
    % right.
    % They will toggle every 50 iterations.
   if (mod(iCounter,100) <= 50)
        %rot_r([top_row;dxu(1,:) < 0.1]) = 0;
        dxu(rot_r) = (20/360)*(2*pi);
        dxu(rot_l) = (-20/360)*(2*pi);
    elseif (mod(iCounter,150) > 50) 
        dxu(rot_r) = (-20/360)*(2*pi);
        dxu(rot_l) = (20/360)*(2*pi);
    end
    % Could manually set angular velocity with formula below.
    %dxu(2,:) = (180/360)*(2*pi);
    r.set_velocities(1:N, dxu);
    r.step();
    
    % This for loop tracks the location/movement of each robot.
    % For each robot, calculate its distance from the center.
    % If it's within the food radius and the width of the bot, then set its
    % target location to where it currently is.
    for c = 1:N
        % Calculate bots' distance from center
        distance_from_center(c) = sqrt((abs(x(1,c))^2) + (abs(x(2,c))^2));
        
        % If the bot is a certain distance from center 
        % (radius of food + 1 bot diamater), they stop there if they're not
        % changing direction.
        if (distance_from_center(c) <= (food_radius + ( .08 )) && ~changing_direction(c))
            if (~set_still(c))
                target(1,c) = x(1,c);
                target(2,c) = x(2,c);
                %r.set_velocities(c,0);
                set_still(c) = true;
            else
            time_at_center(c) = time_at_center(c) + 1;
            end
        end
        
        % When change_direction is true, a new random target outside of 
        % the food radius will be assigned to the bot.
        if (change_direction(c))
            
            % Reset x to be between lower x boundary of arena, -food_radius
            % or from food_radius, upper x boundary (+1.6)
            if randi([0, 1]) == 0 
                target(1,c) = randi([food_radius * 100,...
                    (boundaries(2)-.1) * 100]) * (-1 / 100);
            else
                target(1,c) = randi([food_radius * 100,...
                    (boundaries(2)-.1) * 100]) * (1 / 100);
            end
        
            
            % Reset y to be between lower y boundary of arena, -food_radius
            % or from food_radius, upper y boundary (+1.0)
            if randi([0, 1]) == 0 
                target(2,c) = randi([food_radius * 100,...
                    (boundaries(4)-.1) * 100]) * (-1 / 100);
            else
                target(2,c) = randi([food_radius * 100,...
                    (boundaries(4)-.1) * 100]) * (1 / 100);
            end 
            
            change_direction(c) = false;
            changing_direction(c) = true;
        end
        
        % If done eaitng and not already changing direction,
        % change_direction will be set to true to set the robot to a new
        % random location.
        if (time_at_center(c) > eating_time && ~changing_direction(c))
            change_direction(c) = true;
            %disp("time at center for robot " + c + " exceeds eating_time.");
            %disp("change direction set to true.");
        end
        
        %Let's try to keep bots moving.
        %if (~set_still(c) && (abs(x(1,c) - target(1,c)) <= 0.1 && abs(x(2,c) - target(2,c)) <= 0.1))
            %change_direction(c) = true;
            %disp("this line was reached at some point by bot " + c);
        %end   
    end
end
% Call this at the end of en experiment whether or not data is saved.
r.call_at_scripts_end();