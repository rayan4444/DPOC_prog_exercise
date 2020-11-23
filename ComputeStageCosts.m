function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    G = ones(K,5);

    [xmax,ymax]=size(map);
    
    for i=1:K
        for j=1:5
            x_i = stateSpace(i,1);
            y_i = stateSpace(i,2);
            if (i == TERMINAL_STATE_INDEX)
                G(i,j)=0;
            else
            %Scenario 1 : Go north 
            if(j == NORTH)
                G(i,j) = input_not_allowed(x_i,y_i,map,NORTH) + ...
                0.25*P_WIND*cost_to_go_here(x_i,y_i,map) + ...
                0.25*P_WIND*cost_to_go_here(x_i-1,y_i+1,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i,y_i+2,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i+1,y_i+1,map)+ ...
                (1-P_WIND)*cost_to_go_here(x_i,y_i+1,map);

            end
            if(j == SOUTH)
                G(i,j) = input_not_allowed(x_i,y_i,map,SOUTH)+ 0.25*P_WIND*cost_to_go_here(x_i+1,y_i-1,map) + ...
                0.25*P_WIND*cost_to_go_here(x_i-1,y_i-1,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i,y_i-2,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i,y_i,map)+ ...
                (1-P_WIND)*cost_to_go_here(x_i,y_i-1,map);

            end
            if(j == WEST)

                G(i,j) =  input_not_allowed(x_i,y_i,map,WEST) + 0.25*P_WIND*cost_to_go_here(x_i-1,y_i+1,map) + ...
                0.25*P_WIND*cost_to_go_here(x_i-2,y_i,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i-1,y_i-1,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i,y_i,map)+ ...
                (1-P_WIND)*cost_to_go_here(x_i-1,y_i,map);
                
            end
            if(j == EAST)
                G(i,j) = input_not_allowed(x_i,y_i,map,EAST) + 0.25*P_WIND*cost_to_go_here(x_i+1,y_i+1,map) + ...
                0.25*P_WIND*cost_to_go_here(x_i+2,y_i,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i+1,y_i-1,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i,y_i,map)+ ...
                (1-P_WIND)*cost_to_go_here(x_i+1,y_i,map);
            end
            if(j == HOVER)
                G(i,j) = 0.25*P_WIND*cost_to_go_here(x_i,y_i-1,map) + ...
                0.25*P_WIND*cost_to_go_here(x_i,y_i+1,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i-1,y_i,map)+ ...
                0.25*P_WIND*cost_to_go_here(x_i+1,y_i,map)+ ...
                (1-P_WIND)*cost_to_go_here(x_i,y_i,map);
            end
            end
        end
    end

end

function p = p_no_shoot(mx, nx,map)

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
%determine the probablility of not getting shot given a position x

% Assumption: getting shot by one shooter is independednt from getting shot
% by another one. however, we can be shot by multiple shooters: not
% mutually exclusive

%method: find all the shooters on the map, compute their probability of
%failure. The probability they all fail is the product of the individual
%failure probabilities
p=1;
[m,n]=size(map);
for x=1:m
    for y=1:n
        if map(x,y)== SHOOTER
            %calculate the distance between the shooter and the drone 
            dist= (abs(mx-x)+abs(nx-y));
            if dist <= R
                p=p*(1-(GAMMA/(dist+1)));
            end 
        end
    end 
end 
end 

function q1 = cost_to_go_here(x,y,map)
global GAMMA R P_WIND Nc
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
q1 = 1;
[xmax,ymax]=size(map);
%test if wind push us out of the boundaries
if(x<1 || x > xmax || y < 1 || y > ymax)
    q1 = Nc;
else
    if(map(x,y) == TREE)
        q1 = Nc;
    else
        if(p_no_shoot(x,y,map)<1)
        q1 = 1+(1-p_no_shoot(x,y,map))*Nc;
        else 
        q1 = 1;
        end
    end

end

end

function q2 = input_not_allowed(x,y,map,input)
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
q2 = 0;
[xmax,ymax]=size(map);
%test if we want to go in a tree or out of the boundaries
    
            if ((y == 1)  && (input == SOUTH))
                q2 = inf;
            end
            if((y+1<=ymax) && (map(x,y+1)==TREE) && (input == NORTH))
                q2 = inf;
            end

            if((y==ymax) && (input == NORTH))
                q2 = inf;
            end
            if((y-1>=1 && map(x,y-1)==TREE) && (input == SOUTH))
                q2 = inf;
            end
            if((x==1) && (input == WEST))
                q2 = inf;
            end
            if((x-1>=1) && (map(x-1,y)==TREE) && (input == WEST))
               q2 = inf;
            end
            if((x == xmax) && (input == EAST))
                q2 = inf;
            end
            if((x+1<=xmax) && (map(x+1,y)==TREE) && (input == EAST))
                q2=inf;
            end

end
