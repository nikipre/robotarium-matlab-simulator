%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular pose
%Paul Glotfelter 
%3/24/2016

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents(); 

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
% Since we've chosen pose-controlled dynamics, the inputs will be poses.
r = rb.build('NumberOfAgents', N, 'Dynamics', 'PoseControlled', ...
'CollisionAvoidance', true, 'ShowFigure', true, 'SaveData', true);

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_states();
r.step();
        
%Get randomized initial conditions in the robotarium arena
initial_conditions = generate_initial_conditions(N, 'Width', r.boundaries(2), 'Height', r.boundaries(4), 'Spacing', 0.3);

args = {'PositionError', 0.02, 'RotationError', 0.3};
init_checker = create_is_initialized(args{:});
automatic_parker = create_automatic_parking_controller(args{:});

while(~init_checker(x, initial_conditions))

    % Since we've chosen pose-controlled dynamics, the states will be (x,
    % y, theta) poses.
    x = r.get_states();    

    % Since we've chosen pose-controlled dynamics, the inputs will be (x, y, theta) poses
    % (i.e., the desired initial conditions)
    r.set_inputs(1:N, initial_conditions);
    r.step();   
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

