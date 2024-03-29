function P = ComputeTransitionProbabilities_old(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

%% Initialisation
%By design, the drone can only access a few next states given an initial state. 
%it makes sense then to initialize the matrix P with zero probablilities
%for all transitions.
%in cases where the transition is possible, a non-zero probability is ADDED
%to the exisiting value in the cell 

P=zeros(K,K,5);

%% Loop going through all the possible start-finish possibilities
for i=1:K
    m_i= stateSpace(i,1);
    n_i= stateSpace(i,2);
    p_i= stateSpace(i,3); 
    for j=1:K
            if i== TERMINAL_STATE_INDEX
                %if is is ther terminal state, the drone doesn't move anywhere, so
                % the transition probability to any j that is different than itself
                % is 0. 

                P(i,i,:)=ones(5,1);
%                 
            else
                m_j=stateSpace(j,1);
                n_j=stateSpace(j,2);
                p_j=stateSpace(j,3);

                %check if the transition is possible given the package status
                %the change in package status between two consicutive stages only
                %happens when 
                %(1)the drone gets shot while having a package: i=1, j=0, j is base
                %(2)the drone successfully picks up a package; i=0, j=1, j is pickup   

                if p_i==0 && p_j==1 %drone picked up the package and didn't crash
                    if map(m_j, n_j) == PICK_UP
                        %calculate probability of nearby cells
                        P=P+transition_probabilities_no_crash(i, j, m_i, n_i, m_j, n_j,map);
                    end

                elseif p_i==1 && p_j==0 %drone crashed
                   if map(m_j, n_j) == BASE 
                      P=P+base_transition_probability(i,j,m_i,n_i,map);
                   end

                elseif p_i==0 && p_j==0 
                    if map(m_j, n_j) ~= PICK_UP %j cannot be the pickup location and have no package)
                     %calculate probability of nearby cells
                     P=P+transition_probabilities_no_crash(i, j, m_i, n_i, m_j, n_j,map);
                    end 
                    if map(m_j, n_j) == BASE %if j is the base, we need to include the probability of crashing
                        P=P+base_transition_probability(i,j,m_i,n_i,map);
                    end 
                    
             

                elseif p_i==1 && p_j==1 % drone moved and didn't crash
                    %calculate probability of nearby cells
                    P=P+transition_probabilities_no_crash(i, j, m_i, n_i, m_j, n_j,map);
                end 
            end 
    end
end
disp('Finsished calculating transition probabilities');
end

%% Determining the probability of not getting shot

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

%% Determining the transtition probability in case of crash

function P=base_transition_probability(i,j,m_i,n_i,map)

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

%determine map bounds
[xmax,ymax]=size(map);
P=zeros(K,K,5);
 %scenario 1: Hover position is always allowed
   P=P+transition_probability_crash(i,j,HOVER, m_i, n_i,map);

   %scenario 2: NORTH input is allowed
   if (n_i+1)<= ymax && map(m_i,n_i+1) ~= TREE
        P=P+transition_probability_crash(i,j,NORTH,m_i,n_i+1,map);
   end

   %scenario 3: SOUTH input is allowed
   if (n_i-1)>=1 && map(m_i,n_i-1) ~= TREE
        P=P+transition_probability_crash(i,j,SOUTH,m_i,n_i-1,map);
   end

   %scenario 4: EAST input is allowed
   if (m_i+1)<= xmax && map(m_i+1,n_i)~= TREE
        P=P+transition_probability_crash(i,j,EAST,m_i+1,n_i,map);
   end

   %scenario 5: WEST input is allowed
   if (m_i-1)>= 1 && map(m_i-1,n_i) ~= TREE
        P=P+transition_probability_crash(i,j,WEST,m_i-1,n_i,map);
   end
   return
end 

function P=transition_probability_crash(i, j,input, mi, ni,map)

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX 

%determine map bounds
[xmax,ymax]=size(map);
P=zeros(K,K,5);
%reminder: we know for sure that in this case j is BASE
% here mi and ni are the coordinates of the temporary position (between
% i and the crash)

   % scenario 1: no wind, get shot 
    P(i,j,input)= P(i,j,input) + (1-P_WIND)*(1-p_no_shoot(mi,ni,map));
    
    %scenario 2: wind North
    if (ni+1)>ymax % blown off map
        P(i,j,input)=P(i,j,input)+0.25*P_WIND;
    else
        temp= map(mi,ni+1);
        if (temp ~= TREE) %get shot in new spot
            % if the temporary position is not an obstacle then you got
            % shot
            P(i,j,input)= P(i,j,input)+0.25*P_WIND*(1-p_no_shoot(mi,ni+1,map));
        else % run into a tree
            P(i,j,input)=P(i,j,input)+0.25*P_WIND;
        end
    end 
    
    % scenario 3: wind South
    if (ni-1)<1 %blown off map
        P(i,j,input)=P(i,j,input)+0.25*P_WIND;
    else
        temp= map(mi,ni-1);
        if (temp ~= TREE) %get shot in new spot
            P(i,j,input)= P(i,j,input)+0.25*P_WIND*(1-p_no_shoot(mi,ni-1,map));
        else % run into a tree
            P(i,j,input)=P(i,j,input)+0.25*P_WIND;
        end
    end
    
    % scenario 4: wind East
    if (mi+1)> xmax %blown off map
        P(i,j,input)=P(i,j,input)+0.25*P_WIND;
    else
        temp= map(mi+1,ni);
        if (temp ~= TREE)% get shot in new spot
            P(i,j,input)= P(i,j,input)+0.25*P_WIND*(1-p_no_shoot(mi+1,ni,map));
        else % run into a tree
            P(i,j,input)=P(i,j,input)+0.25*P_WIND;
        end
    end
    
    % scenario 5: wind West
    if (mi-1)<1 %blown off map
        P(i,j,input)=P(i,j,input)+0.25*P_WIND;
    else
        temp= map(mi-1,ni);
        if (temp ~= TREE)% get shot in new spot
            P(i,j,input)= P(i,j,input)+0.25*P_WIND*(1-p_no_shoot(mi-1,ni,map));
        else % run into a tree
            P(i,j,input)=P(i,j,input)+0.25*P_WIND;
        end
    end
    return
end 

%% Determining the transition probability in the case of no crashes.

function P=transition_probabilities_no_crash(i, j, mi, ni, mj, nj,map)

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

P=zeros(K,K,5);
%determine the distance between i and j to see if j is reacheable
d =(abs(mi-mj)+abs(ni-nj));
pns = p_no_shoot(mj, nj,map);

%determine map bounds
[xmax,ymax]=size(map);

switch d
    case 0 % staying in place
        %scenario 1: input = hover, given no wind and no getting shot 
        P(i, j, HOVER) = P(i, j, HOVER)+(1-P_WIND)*(pns);
        
        %scenario 2: input = north, wind sounth, no getting shot
            if (ni+1)<=ymax
                temp= map(mi,ni+1);
                if (temp ~=TREE)
                    P(i,j,NORTH)=  0.25*P_WIND*(pns);
                end 
            end
        
        %scenario 3: input = south, wind north, no getting shot
        if (ni-1)>=1
            temp= map(mi,ni-1);
            if (temp ~= TREE)
                P(i,j,SOUTH)= 0.25*P_WIND*(pns);
            end
        end
        
        %scenario 4: input = east, wind west, no getting shot
        if (mi+1)<= xmax
            temp= map(mi+1,ni);
            if (temp ~= TREE) 
                P(i,j,EAST)= 0.25*P_WIND*(pns);
            end
        end 
        
        %scenario 5: input = west, wind east, no getting shot 
        if( mi-1)>=1
            temp= map(mi-1,ni);
            if (temp ~= TREE)
                P(i,j,WEST)= 0.25*P_WIND*(pns);
            end
        end 
        
    case 1 % moving by one:
        
        %since we go directly to j without intermediate steps and j is a
        %possible output, there is no need to check for out of bounds or
        %trees 
        
        % scenario 1: input = hover, wind in any direction, no getting shot
        P(i, j, HOVER) = 0.25*P_WIND*(pns);
        % scenario 2: input = direction, no wind, no getting shot.
        switch (mi-mj)
            case 0
                if nj>ni
                    %j is to the north of i
                    P(i,j,NORTH)= (1-P_WIND)*(pns);
                else 
                    %j is to the south of i 
                    P(i,j,SOUTH)= (1-P_WIND)*(pns);
                end
            case 1
                %j is to the west of i 
                P(i,j,WEST)= (1-P_WIND)*(pns);
            case -1
                %j is to the east of i
                P(i,j,EAST)= (1-P_WIND)*(pns);
        end 
        
    case 2 % moving by 2
        if mi==mj 
            % since j is given we know it is not out of bounds. But we need
            % to make sure there is no tree in the intermediate step
            
            if nj>ni % j is to the north of i 
                %scenario: input = north, wind north, no getting shot
                if (map(mi, ni+1)~= TREE)
                    P(i,j,NORTH)= 0.25*P_WIND*(pns);
                end
            else  % j is to the south of i 
                %scenario: input = south, wind south, no getting shot
                if (map(mi, ni-1)~= TREE)
                    P(i,j,SOUTH)= 0.25*P_WIND*(pns);
                end
            end 
        end 
        
        if ni==nj
            % since j is given we know it is not out of bounds. But we need
            % to make sure there is no tree in the intermediate step
            
            if mj>mi % j is to the east of i 
                %scenario: input = east, wind east, no getting shot 
                if (map(mi+1, ni)~= TREE)
                    P(i,j,EAST)= 0.25*P_WIND*(pns);
                end
            else  % j is to the west of i 
                %scenario: input = south, wind south, no getting shot
                if (map(mi-1, ni)~= TREE)
                    P(i,j,WEST)= 0.25*P_WIND*(pns);
                end
            end 
        end 
        
        if mj==(mi+1)
            % as long as the desitnation exists, we know that the
            % intermediate step is within map bounds, so we only need to
            % make sure there are no trees in the way 
            
            if nj>ni %j is north-east of i 
                %scenario 1: input = north, wind east, no getting shot
                temp= map(mi,ni+1);
                if (temp ~= TREE)
                    P(i,j,NORTH)= 0.25*P_WIND*(pns);
                end 
                
                %scenario 2: input = east, wind north, no getting shot
                temp= map(mi+1,ni);
                if (temp ~= TREE)
                    P(i,j,EAST)= 0.25*P_WIND*(pns);
                end
            else %j is south-east of i
                 %scenario 1: input = south, wind east, no getting shot
                 temp= map(mi,ni-1);
                if (temp ~= TREE)
                    P(i,j,SOUTH)= 0.25*P_WIND*(pns);
                end
                %scenario 2: input = east, wind south, no getting shot
                 temp= map(mi+1,ni);
                if (temp ~= TREE)
                    P(i,j,EAST)=  0.25*P_WIND*(pns);
                end
            end 
        end
        
        if mj==(mi-1)
            % as long as the desitnation exists, we know that the
            % intermediate step is within map bounds, so we only need to
            % make sure there are no trees in the way 
            
            if nj>ni %j is north-west of i 
                %scenario 1: input = north, wind west, no getting shot
                temp= map(mi,ni+1);
                if (temp ~= TREE)
                    P(i,j,NORTH)= 0.25*P_WIND*(pns);
                end 
                
                %scenario 2: input = est, wind north, no getting shot
                temp= map(mi-1,ni);
                if (temp ~= TREE)
                    P(i,j,WEST)= 0.25*P_WIND*(pns);
                end
            else %j is south-east of i
                 %scenario 1: input = south, wind west, no getting shot
                 temp= map(mi,ni-1);
                if (temp ~= TREE)
                    P(i,j,SOUTH)= 0.25*P_WIND*(pns);
                end
                %scenario 2: input = west, wind south, no getting shot
                 temp= map(mi-1,ni);
                if (temp ~= TREE)
                    P(i,j,WEST)= 0.25*P_WIND*(pns);
                end
            end 
        end
end 
return
end 
