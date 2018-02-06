%This script shows how to initialize robots to a particular point
%Paul Glotfelter 
%3/24/2016

%% Initialize objects

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents(); 

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.build('NumberOfAgents', N, 'Dynamics', 'PointControlled', ...
    'CollisionAvoidance', true, 'SaveData', true, 'ShowFigure', true);

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_states();
r.step();

        
%Get randomized initial conditions in the robotarium arena
initial_conditions = generate_initial_conditions(N, 'Width', r.boundaries(2)-r.boundaries(1)-0.1, 'Height', r.boundaries(4)-r.boundaries(3)-0.1, 'Spacing', 0.3);

% We'll make the rotation error huge so that the initialization checker
% doesn't care about it
init_checker = create_is_initialized('PositionError', 0.01, 'RotationError', 50);

%% Algorithm

% We add on the zeros, because the state from a point-controlled agent is 2
% dimensional.  The init-checker function needs a 3-dimensional state
while(~init_checker([x ; zeros(1, N)], initial_conditions))

    x = r.get_states();

    r.set_inputs(1:N, initial_conditions(1:2, :));
    r.step();   
end

%% Misc

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

