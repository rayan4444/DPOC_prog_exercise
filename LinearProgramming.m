function [ J_opt, u_opt_ind, A, b, cost_action] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

global K HOVER;

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX;
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%Terminal state index should be removed from the P and G matrices. For P,
%for both the initial and transition state: 

P(TERMINAL_STATE_INDEX,:,:)=[];
P(:,TERMINAL_STATE_INDEX,:)=[];
G(TERMINAL_STATE_INDEX,:)=[];


%We cannot have infinity in the G matrix for the computations. We can
%either choose to remove it or set it to a very large number as below. 
G(isinf(G))=10^6;

%Identity matriz
I=eye(K-1); 


f=-ones(K-1,1);

%Matrices to be implemented in linprog function 
A=[];
b=[];

for l=1:5 % iterate for every input/action

        %Defining left entry:
        cost_vector = [];
        
        prob_matrix = P(:,:,l);
        
        for i = 1:K-1
            cost_entry = G(i,l);
            cost_vector = [cost_vector;G(i,l)];
        end
        
        
        % here we get the A and b parts for a single output, in for of
        % cells that are then stacked. like this we have a giant stack of
        % all the possibilities, stacked for all the inputs
        
    %stacked A
   A=[A; I-P(:,:,l)];
    
    %stacked b    
    
   b=[b;cost_vector];
   
    
end
size(A);
size(b);

%Lower bound, upper bound and equalities are all empty
l_bd = [];
u_bd=[];
A_eq = [];
b_eq = [];

%Initialising J_opt and u_opt 

J_opt = ones(K-1,1)
u_opt = ones(K-1,1)
u_opt_ind=ones(K-1,1)


%Using linprog function to compute J_opt
J_opt = linprog(f,A,b,A_eq,b_eq,l_bd,u_bd);


%Now looping first through every state as the optimal action for every
%state should be determined we have: 

cost_action=ones(5,1)

for i = 1:K-1
    for l=1:5
        cost_to_go = G(i,l); %Cost for every input at this state
        trans_prob = P(i,:,l); %Transition prob for all other states given this input
        cost_action(l) = cost_to_go + trans_prob*J_opt;
    end

    [u_opt(i),u_opt_ind(i)] = min(cost_action);
end


%Adding what happens at the terminal state(Placeholders for now) 

u_opt_ind = [u_opt_ind; TERMINAL_STATE_INDEX]

J_opt = [J_opt; 30]
end


