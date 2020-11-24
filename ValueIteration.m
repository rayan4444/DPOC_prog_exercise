function [ J_opt, u_opt_ind] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER
global ERR
ERR = 10e-5;

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
    P(TERMINAL_STATE_INDEX,:,:)= [];
    P(:,TERMINAL_STATE_INDEX,:)= [];
    G(TERMINAL_STATE_INDEX,:)= [];

    u_opt_ind = ones(K-1,1);
    J_opt = ones(K-1,1);
    J_temp = ones(5,1);
    J_old = ones(K-1,1);
    
 while(1)
        for i = 1:K-1
            for u = 1:5
                J_temp(u) = G(i,u)+ P(i,:,u)*J_old;
            end
           [J_opt(i),u_opt_ind(i)]= min(J_temp);
        end
        
        if(max(abs(J_opt-J_old))< ERR)
            break;
        else
            J_old = J_opt;
        end
 end
 %% Adding back the terminal stage considerations
% if we are at the terminal stage we stay there so policy would be HOVER    
% so we need to insert in HOVER in the array at the terminal index position    
u_opt_ind = [u_opt_ind(1:TERMINAL_STATE_INDEX-1);...
             HOVER;...
             u_opt_ind(TERMINAL_STATE_INDEX:end)];
% same for the cost to go, the cost to go a the terminal stage is zero, so
% we need to insert zero at the terminal index posiiton in the array
J_opt    = [J_opt(1:TERMINAL_STATE_INDEX-1);...
            0;...
            J_opt(TERMINAL_STATE_INDEX:end)];
 
end