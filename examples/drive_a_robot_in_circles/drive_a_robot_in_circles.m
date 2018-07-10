%%%% This example code is for the Robotarium simulator version 1.1 %%%%%%

% Drive one robot in a circle of specified center and radius using the
% 'SingleIntegrator' setting in the new simulator
% Siddharth Mayya
% 10/17/2017

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% For this experiment, we only need one robot
N = 1; 

% Initialize the robotarium object, and specify number of robots, how we
% want to control the robots, whether we want collision avoidance etc.
r = rb.build('NumberOfAgents', N, 'Dynamics', 'SingleIntegrator', ...
    'CollisionAvoidance', true, 'SaveData', true, 'ShowFigure', true);

% max number of iterations for the simulation
iterations = 10000;

% specify the center (X,Y) coordinates of the circle.  
center = [0;0];

% specify the radius of the circle
radius = 0.5;

% specify the acceptable error margin between points on the circle and position
% of the robot.
error_margin = 0.02;

% Given the circle parameters, we create a pre-specified number of equally spaced waypoint locations on
% the circle. Once a robot reaches one waypoint, it is assigned the next one as a goal. 
num_waypoints = 100;
th_vec = linspace(0,2*pi,num_waypoints); 
waypoints = [radius.*cos(th_vec);radius.*sin(th_vec)]; % 2 X 100 matrix

% indicates which waypoint is currently being used. 
current_goal_index = 1;

% create a position controller function
controller = create_si_position_controller();

for i = 1:iterations
    
    % obtain the current pose of the robot.
    % x - 3 X 1 vector - contains x, y, and orientation (angle) of the robot.
    x = r.get_states();
    
    % if the robot reaches a waypoint, change the goal location to the 
    % next point on the circle.
    if norm(x(1:2) - waypoints(:,current_goal_index)) <= error_margin
        current_goal_index = mod(current_goal_index,num_waypoints)+1;
    end
    
    % decide how the robot should move (compute a cartesian velocity vector), based
    % on where the robot is, and where the goal is
    velocity = controller(x(1:2),waypoints(:,current_goal_index)); 
    
    % send these velocities to the simulated/real robot (which has index 1)
    r.set_inputs(1,velocity);
    
    % simulate one step! 
    r.step();
    
end

% always call at the end of an experiment
r.call_at_scripts_end();