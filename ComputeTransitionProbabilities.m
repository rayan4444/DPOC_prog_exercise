function P = ComputeTransitionProbabilities(stateSpace, map)
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
global xmax ymax 
%% Initialisation
%By design, the drone can only access a few next states given an initial state. 
%it makes sense then to initialize the matrix P with zero probablilities
%for all transitions.
%in cases where the transition is possible, a non-zero probability is ADDED
%to the exisiting value in the cell 

P=zeros(K,K,5);
[xmax, ymax]= size(map);

%% Find the index of the base and the pickup point
%State space index of the base after a crash
%ref: https://ch.mathworks.com/matlabcentral/answers/454859-how-to-find-the-row-index-of-specific-vector-in-a-matrix
[a,b]=find(map==BASE);
BASE_INDEX=find(ismember(stateSpace, [a,b,0],'rows'));

%state spce index of pickup location wiht package
[a,b]=find(map==PICK_UP);
PICKUP_INDEX=find(ismember(stateSpace, [a,b,1],'rows'));

%% Find the probability of not getting shot for every position in the map 
shooters =[];
pns=ones(xmax,ymax);

%find all the shooters
for m=1:xmax
    for n=1:ymax
        if map(m,n)== SHOOTER
            shooters=[shooters;m,n];
        end 
    end 
end

%calculate the probability of getting not getting shot ofr  every point in
%the map
for m=1:xmax
    for n=1:ymax
        for i=1:size(shooters,1)
            dist= (abs(shooters(i,1)-m)+abs(shooters(i,2)-n));
            if dist <= R
                pns(m,n)=pns(m,n)*(1-(GAMMA/(dist+1)));
            end 
        end
    end
end

%% Establish a mapping between map and the state space indexes 
% to easily go from one to the other. we put the index of the state with 0,
% knowing that according to how the state space is defined, the index of
% the same position ut wiht the package would be the result +1

index_of_pos=zeros(xmax,ymax); %the entries that stay 0 correspond to trees
for i=1:2:K %we skip the indexes of drone with package
     index_of_pos(stateSpace(i,1), stateSpace(i,2))=i;
end 


%% Building the transition probability matrix
for i=1:2:K
   % find i on the map 
    m= stateSpace(i,1);
    n= stateSpace(i,2);
    
    % Method: for every starting position on the map, we find the
    % probability of transitioning to the base (in other words, the
    % probability of crashing) and the probabilities of transitioning to
    % neighbouring cells, depending on which inputs can be applied. 
    
    % Knowing how the stateSpace matrix is built (for every possible position we
    % have first the state without the package and then with the package),
    % we can iterate with step 2.
    
    % the only special cases we need to take into consideration are: the
    % Terminal stage, for which the probability of staying is 1 and the
    % case in which the drone lands on the pickup location without crashing
    %(change of package staus from 0 to 1) 
    
    
    
    
    
    %if going NORTH is allowed
    if (n+1)<=ymax 
       jn=index_of_pos(m,n+1); 
        if jn~=0
            %not crashing after input north but staying in place
            P(i,i,NORTH)= 0.25*P_WIND*pns(m,n);% wind back to i
            P(i+1,i+1,NORTH)= P(i,i,NORTH);
            if i+1==PICKUP_INDEX
                P(i,i+1,NORTH)= 0.25*P_WIND*pns(m,n);
            end
            %not crashing after north wind
            P(i,jn, HOVER) = 0.25*P_WIND*pns(m,n+1);
            P(i+1,jn+1,HOVER) = P(i,jn, HOVER);
            
            %not crashing after North input and no wind
            P(i,jn,NORTH)= (1-P_WIND)*pns(m,n+1);
            P(i+1,jn+1,NORTH)= P(i,jn,NORTH);
            
            %reaching the pickup location and not crashing
            if jn+1 == PICKUP_INDEX
                P(i,jn+1,HOVER) = 0.25*P_WIND*pns(m,n+1);
                P(i,jn+1,NORTH)= (1-P_WIND)*pns(m,n+1);
                P(i,jn,NORTH)=0;
                P(i,jn,HOVER)=0;
            end 
        
        end
    end
    %if going SOUTH is within bounds
    if (n-1)>=1 
        js=index_of_pos(m,n-1);
        if js~=0
            %not crashing after input South but staying in place
            P(i,i,SOUTH)= 0.25*P_WIND*pns(m,n);
            P(i+1,i+1,SOUTH)=P(i,i,SOUTH);
            
            if i+1==PICKUP_INDEX
                P(i,i+1,SOUTH)= 0.25*P_WIND*pns(m,n);
            end
            %not crashing after north wind
            P(i,js,HOVER) = 0.25*P_WIND*pns(m,n-1);
            P(i+1,js+1,HOVER)=P(i,js,HOVER);
            
            %not crashing after SOUTH input and no wind
            P(i,js,SOUTH)= (1-P_WIND)*pns(m,n-1);
            P(i+1,js+1,SOUTH)=P(i,js,SOUTH);
            
            %reaching the pickup location and not crashing
            if js+1 == PICKUP_INDEX
              P(i,js+1,HOVER) = 0.25*P_WIND*pns(m,n-1);
              P(i,js+1,SOUTH)= (1-P_WIND)*pns(m,n-1);  
              P(i,js,SOUTH)=0;
              P(i,js,HOVER)=0;
            end
            
        end
    end

    %if going EAST is allowed
    if (m+1)<= xmax 
        je=index_of_pos(m+1,n);
        if je~=0
            P(i,i,EAST)= 0.25*P_WIND*pns(m,n);
            P(i+1,i+1,EAST)=P(i,i,EAST);
            if i+1==PICKUP_INDEX
                P(i,i+1,EAST)= 0.25*P_WIND*pns(m,n);
            end

            P(i,je,HOVER) = 0.25*P_WIND*pns(m+1,n);
            P(i+1,je+1,HOVER) =P(i,je,HOVER);

            P(i,je,EAST)= (1-P_WIND)*pns(m+1,n);
            P(i+1,je+1,EAST)=P(i,je,EAST);

            if je+1 == PICKUP_INDEX
                P(i,je+1,HOVER) = 0.25*P_WIND*pns(m+1,n);
                P(i,je+1,EAST)= (1-P_WIND)*pns(m+1,n);
                P(i,je,EAST)=0;
                P(i,je,HOVER)=0;
            end
        end                           
    end 
    %if going WEST is allowed 
    if(m-1)>=1 
        jw=index_of_pos(m-1,n);
        if jw~=0

            P(i,i,WEST)= 0.25*P_WIND*pns(m,n);
            P(i+1,i+1,WEST)= P(i,i,WEST);
            if i+1==PICKUP_INDEX
                P(i,i+1,WEST)= 0.25*P_WIND*pns(m,n);
            end

            P(i,jw,HOVER) = 0.25*P_WIND*pns(m-1,n);
            P(i+1,jw+1,HOVER) =P(i,jw,HOVER);

            P(i,jw,WEST)= (1-P_WIND)*pns(m-1,n);
            P(i+1,jw+1,WEST)= P(i,jw,WEST);

            if jw+1 == PICKUP_INDEX
                P(i,jw+1,HOVER) = 0.25*P_WIND*pns(m-1,n); 
                P(i,jw+1,WEST)=(1-P_WIND)*pns(m-1,n);
                P(i,jw,WEST)=0;
                P(i,jw,HOVER)=0;
            end
        end   
    end  

    % two North
    if (n+2)<=ymax 
        j=index_of_pos(m,n+2);                 
        if j~=0 && jn~=0
            P(i,j,NORTH)= 0.25*P_WIND*pns(m,n+2);
            P(i+1,j+1,NORTH)=P(i,j,NORTH);

            if j+1 == PICKUP_INDEX
                P(i,j+1,NORTH)= 0.25*P_WIND*pns(m,n+2);
                P(i,j,NORTH)=0;
            end
        end
    end 
    % two south
    if (n-2)>=1
        j=index_of_pos(m,n-2);
        if j~=0 && js~=0
            P(i,j,SOUTH)= 0.25*P_WIND*pns(m,n-2);
            P(i+1,j+1,SOUTH)=P(i,j,SOUTH);

            if j+1 == PICKUP_INDEX
                P(i,j+1,SOUTH)= 0.25*P_WIND*pns(m,n-2);
                P(i,j,SOUTH)=0;
            end
        end
    end

    % two East
    if(m+2)<=xmax 
        j=index_of_pos(m+2,n);
        if j~=0 && je~=0
            P(i,j,EAST)= 0.25*P_WIND*pns(m+2,n);
            P(i+1,j+1,EAST)= P(i,j,EAST);

            if j+1 == PICKUP_INDEX
                P(i,j,EAST)= 0.25*P_WIND*pns(m+2,n);
                P(i,j,EAST)=0;
            end
        end
    end
    % two West
    if(m-2)>=1
        j=index_of_pos(m-2,n);
        if j~=0 && jw~=0
            P(i,j,WEST)= 0.25*P_WIND*pns(m-2,n);
            P(i+1,j+1,WEST)=P(i,j,WEST);
            if j+1 == PICKUP_INDEX
                P(i,j+1,WEST)= 0.25*P_WIND*pns(m-2,n);
                P(i,j,WEST)=0;
            end 
        end
    end

    % NorthEast
    if (n+1)<=ymax &&(m+1)<= xmax 
        j=index_of_pos(m+1,n+1);
        if j~=0
            if jn~=0
                P(i,j,NORTH)= 0.25*P_WIND*pns(m+1,n+1);
                P(i+1,j+1,NORTH)=P(i,j,NORTH);
                if j+1 == PICKUP_INDEX
                    P(i,j+1,NORTH)= 0.25*P_WIND*pns(m+1,n+1);
                    P(i,j,NORTH)=0;
                end
            end 
            if je~=0
                P(i,j,EAST)= 0.25*P_WIND*pns(m+1,n+1);
                P(i+1,j+1,EAST)= P(i,j,EAST);

                if j+1 == PICKUP_INDEX
                    P(i,j+1,EAST)= 0.25*P_WIND*pns(m+1,n+1);
                    P(i,j,EAST)=0;
                end
            end
        end 
    end
    % NorthWest
    if (n+1)<=ymax &&(m-1)>= 1
        j=index_of_pos(m-1,n+1);
        if j~=0
            if jn~=0
                P(i,j,NORTH)= 0.25*P_WIND*pns(m-1,n+1);
                P(i+1,j+1,NORTH)= P(i,j,NORTH);
                if j+1 == PICKUP_INDEX
                    P(i,j+1,NORTH)= 0.25*P_WIND*pns(m-1,n+1);
                    P(i,j,NORTH)=0;
                end 
            end 
            if jw~=0
                P(i,j,WEST)= 0.25*P_WIND*pns(m-1,n+1);
                P(i+1,j+1,WEST)= P(i,j,WEST);
                if j+1 == PICKUP_INDEX
                    P(i,j+1,WEST)= 0.25*P_WIND*pns(m-1,n+1);
                    P(i,j,WEST)=0;
                end
            end
        end
    end

    % SouthEast
    if (n-1)>= 1 &&(m+1)<= xmax
        j=index_of_pos(m+1,n-1);
        if j~=0
            if js~=0
                P(i,j,SOUTH)= 0.25*P_WIND*pns(m+1,n-1);
                P(i+1,j+1,SOUTH)= P(i,j,SOUTH);
                if j+1 == PICKUP_INDEX
                    P(i,j+1,SOUTH)= 0.25*P_WIND*pns(m+1,n-1);
                end
            end 
            if je~=0
                P(i,j,EAST)= 0.25*P_WIND*pns(m+1,n-1);
                P(i+1,j+1,EAST)=P(i,j,EAST);
                if j+1 == PICKUP_INDEX
                    P(i,j+1,EAST)= 0.25*P_WIND*pns(m+1,n-1);
                    P(i,j,EAST)=0;
                end
            end
        end
    end

    % SouthWest
    if (n-1)>= 1 &&(m-1)>= 1
        j=index_of_pos(m+-1,n-1);
        if j~=0
            if js~=0
                P(i,j,SOUTH)= 0.25*P_WIND*pns(m-1,n-1);
                P(i+1,j+1,SOUTH)=P(i,j,SOUTH);

                if j+1 == PICKUP_INDEX
                    P(i,j+1,SOUTH)= 0.25*P_WIND*pns(m-1,n-1);
                    P(i,j,SOUTH)=0;
                end
            end 
            if jw~=0
                P(i,j,WEST)= 0.25*P_WIND*pns(m-1,n-1);
                P(i+1,j+1,WEST)=P(i,j,WEST);
                if j+1 == PICKUP_INDEX
                    P(i,j+1,WEST)= 0.25*P_WIND*pns(m-1,n-1);
                    P(i,j,WEST)=0;
                end
            end
        end
    end
    
    % HOVER is always allowed:
    %not crashing, in place
    P(i,i,HOVER) = (1-P_WIND)*pns(m,n);
    P(i+1,i+1,HOVER) = (1-P_WIND)*pns(m,n);
    
    %HANDLING  CRASH TO BASE
    %crashing in place
    P(i,BASE_INDEX,HOVER)=P(i,BASE_INDEX,HOVER)+p_base(m,n,pns,map);
    P(i+1,BASE_INDEX,HOVER)=p_base(m,n,pns,map);
    
    %crashing after input north (For the case where there is no package
    %we must add the crash probability to the displacement probability
    %in case we end up there by pure displacement and not crash)
    if (n+1)<=ymax && jn~=0
        P(i,BASE_INDEX,NORTH)=P(i,BASE_INDEX,NORTH)+p_base(m,n+1,pns,map);
        P(i+1,BASE_INDEX,NORTH)=p_base(m,n+1,pns,map);
    end
    if (n-1)>=1 && js~=0
        P(i,BASE_INDEX,SOUTH)=P(i,BASE_INDEX,SOUTH)+p_base(m,n-1,pns,map);
        P(i+1,BASE_INDEX,SOUTH)=p_base(m,n-1,pns,map);
    end
    if (m+1)<=xmax && je~=0
        P(i,BASE_INDEX,EAST)=P(i,BASE_INDEX,EAST)+p_base(m+1,n,pns,map);
        P(i+1,BASE_INDEX,EAST)=p_base(m+1,n,pns,map);
    end
    if (m-1)>=1 && jw~=0
        P(i,BASE_INDEX,WEST)=P(i,BASE_INDEX,WEST)+p_base(m-1,n,pns,map);
        P(i+1,BASE_INDEX,WEST)=p_base(m-1,n,pns,map);
    end
    
    if i+1 == PICKUP_INDEX
        %being a the pickup place without collecting a package and without
        %crashing is not possible
        P(i,i,:)=zeros(5,1);
        %being at the pickup
        P(i,i+1, HOVER)=(1-P_WIND)*pns(m,n);
    end 

end
% Correct for terminal state
P(TERMINAL_STATE_INDEX,:,:)=zeros(K,5);
P(TERMINAL_STATE_INDEX,TERMINAL_STATE_INDEX,:)=ones(5,1);

end 




%%
function p_crash=p_base(mi,ni,pns, map)
global xmax ymax TREE P_WIND
p_crash=0;
%reminder: we know for sure that in this case j is BASE
% here mi and ni are the coordinates of the temporary position (between
% i and the crash)

   % scenario 1: no wind, get shot 
     p_crash=  p_crash + (1-P_WIND)*(1-pns(mi,ni));
    
    %scenario 2: wind North
    if (ni+1)>ymax % blown off map
         p_crash= p_crash+0.25*P_WIND;
    else
        if (map(mi,ni+1)~= TREE) %get shot in new spot
            % if the temporary position is not an obstacle then you got
            % shot
             p_crash=  p_crash+0.25*P_WIND*(1-pns(mi,ni+1));
        else % run into a tree
             p_crash= p_crash+0.25*P_WIND;
        end
    end 
    
    % scenario 3: wind South
    if (ni-1)<1 %blown off map
         p_crash= p_crash+0.25*P_WIND;
    else
        if (map(mi,ni-1) ~= TREE) %get shot in new spot
             p_crash=  p_crash+0.25*P_WIND*(1-pns(mi,ni-1));
        else % run into a tree
             p_crash= p_crash+0.25*P_WIND;
        end
    end
    
    % scenario 4: wind East
    if (mi+1)> xmax %blown off map
         p_crash= p_crash+0.25*P_WIND;
    else
        if (map(mi+1,ni)~= TREE)% get shot in new spot
             p_crash=  p_crash+0.25*P_WIND*(1-pns(mi+1,ni));
        else % run into a tree
             p_crash= p_crash +0.25*P_WIND;
        end
    end
    
    % scenario 5: wind West
    if (mi-1)<1 %blown off map
         p_crash= p_crash+0.25*P_WIND;
    else
        if (map(mi-1,ni)~= TREE)% get shot in new spot
             p_crash=  p_crash+0.25*P_WIND*(1-pns(mi-1,ni));
        else % run into a tree
             p_crash= p_crash+0.25*P_WIND;
        end
    end
end 
