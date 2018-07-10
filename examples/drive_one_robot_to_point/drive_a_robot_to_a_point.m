%%%% This example code is for the Robotarium simulator version 1.1 %%%%%%

% Drive one robot to a point on the Robotarium arena
% Demonstrates the use of the 'PointControlled' setting in the new
% simulator
% Siddharth Mayya
% 10/17/2017

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% For this experiment, we need only one robot
N = 1; 

% Initialize the robotarium object, and specify number of robots, how we
% want to control the robots, whether we want collision avoidance etc.
r = rb.build('NumberOfAgents', N, 'Dynamics', 'PointControlled', ...
    'CollisionAvoidance', true, 'SaveData', true, 'ShowFigure', true);

% specify the goal (X,Y) coordinates that you want the robot to end up at.  
goal = [0;0];

% specify the acceptable error margin between the goal location and final
% position of the robot. If this number is too small, the experiment may never
% end! 
error_margin = 0.02;

% obtain the current pose of the robot. 
% 'Point Controlled' and 'SingleIntegrator':
% x - 2 X 1 vector - contains x, y position of the robot.
% 'PoseControlled':
% x - 3 X 1 vector - contains x, y position, and orientation (angle with respect to a horizontal axxis) of the robot
x = r.get_states();
r.step();

% while the distance between the robot and the goal is larger than the
% margin....
while norm(x(1:2) - goal) >=  error_margin
    
    % get the current position of the robot
    x = r.get_states();
    
    % command the robot (which has index 1) to drive to the goal point
    r.set_inputs(1,goal);
    
    % simulate one step! 
    r.step();
    
end

% always call at the end of an experiment
r.call_at_scripts_end();