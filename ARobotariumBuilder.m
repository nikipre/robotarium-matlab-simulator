classdef ARobotariumBuilder < handle
    %ARobotariumBuilder This is an abstract class for the RobotariumBuilder class
    %that models the manner in which a Robotarium object is created
    % This file should never be modified.  Otherwise, your code will not
    % execute properly on the Robotarium
    
    properties (GetAccess = public, SetAccess = protected)
        available_agents
        number_of_agents
        save_data = true
        show_figure = true
        dynamics_transform
        state_transform
    end
    
    methods (Abstract)
        % Builds the Robotarium object.  Definitely backend/sim dependent.
        get_available_agents(this);
        build(this);
    end
    
    methods  
        function [dynamics_transform, state_transform] = get_transforms(this, varargin)
            % This can be done in the abstract class because it's
            % independent of the network implementation.
            
            % Validation stuff for incoming data
            isbool = @(x) x == true || x == false;
            possible_dynamics = {'Unicycle',...
                'SingleIntegrator', 'PointControlled', 'PoseControlled'};
            
            p = inputParser;
            % Required parameters
            p.addParameter('NumberOfAgents', this.available_agents, @(x) isscalar(x) && x > 0);
            p.addParameter('Dynamics', 'Unicycle', @(x) any(validatestring(x, possible_dynamics)));
            
            % Optional parameters
            p.addParameter('SaveData', true, isbool);
            p.addParameter('ShowFigure', false, isbool);          
            p.addParameter('CollisionAvoidance', true, isbool);
                        
            parse(p, varargin{:});
            
            % Set vanilla parameters
            this.number_of_agents = p.Results.NumberOfAgents;
            this.save_data = p.Results.SaveData;
            this.show_figure = p.Results.ShowFigure;
            
            % There are three levels happening here 
            % -> High-level controller (e.g., point controlled)
            % -> Collision avoidance (e.g., for single integrator) 
            % -> Dynamical mapping (e.g., si to uni)

            % Each should be a function of (input, state).  But the actual
            % contents of state could differ, depending on the dynamical
            % model chosen
            mapping = [];
            high_level = [];         
            state_transform = [];
            switch(p.Results.Dynamics)
                case 'Unicycle'
                    % Don't have to do anything  
                    high_level = @(input, state) input;
                    mapping = @(input, state) input;
                    state_transform = @(state) state;
                case 'SingleIntegrator'
                    high_level = @(input, state) input;
                    mapping = create_si_to_uni_mapping2();
                    state_transform = @(state) state(1:2, :);
                case 'PoseControlled'
                    high_level_ = create_automatic_parking_controller2();
                    high_level = @(input, state) high_level_(state, input);
                    mapping = @(input, state) input; % Already unicycle at this point
                    state_transform = @(state) state;
                case 'PointControlled'
                    high_level_ = create_si_position_controller();
                    state_transform = @(state) state(1:2, :);
                    high_level = @(input, state) high_level_(state, input);
                    mapping = create_si_to_uni_mapping2();               
            end             
            
            % Figure out which collision avoidance method to choose based
            % on the parser results.
            collision_avoidance = [];
            switch(p.Results.CollisionAvoidance)
                case true 
                    switch(p.Results.Dynamics)
                        case 'Unicycle' 
                            collision_avoidance = create_uni_barrier_certificate();
                        case 'SingleIntegrator'
                            collision_avoidance_ = create_si_barrier_certificate();
                            collision_avoidance = @(input, state) collision_avoidance_(input, state);
                        case 'PointControlled'
                            % Still single integrator
                            collision_avoidance_ = create_si_barrier_certificate();
                            collision_avoidance = @(input, state) collision_avoidance_(input, state);
                        case 'PoseControlled'
                            % Unicycle dynamics basically
                            collision_avoidance = create_uni_barrier_certificate();
                    end
                case false
                    % Apply nothing.  Just pass the dynamics through
                    collision_avoidance = @(input, state) input;
            end                       
            
            % Note that the state transformation may be applied at the
            % high level and the collision avoidance.  However, the mapping
            % should be a function of the true state.
            
            % Compose the three levels high_level -> avoidance -> mapping
            % -> $$$
            dynamics_transform = @(input, state) mapping(collision_avoidance(high_level(input, state_transform(state)), state_transform(state)), state);
        end
    end   
end

