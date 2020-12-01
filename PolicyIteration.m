function [ J_opt, u_opt_ind] = PolicyIteration( P, G )

global K HOVER
global TERMINAL_STATE_INDEX

%% Handling the terminal cost
% During policy iteration we need to remove the terminal state from the
% array because we do not want to iterate on it. After finding the optimal
% policy and hte optimal cost with policy iteration we can add back the
% entries for the terminal state in the correspoinding matrices. 

%ref: matlab delete matrix line 
P(TERMINAL_STATE_INDEX,:,:) = [ ];
P(:, TERMINAL_STATE_INDEX,:) = [ ];
G(TERMINAL_STATE_INDEX,:) = [ ];

%We cannot have infinity in the G matrix for the computations. We can
%either choose to remove it or set it to a very large number as below. 
G(isinf(G))=10^12;

%% Initialization
% initialize matrices to hold results or/and temporary values
J_opt = zeros(K-1,1); %matrix to hold the cost to go 
P_u_h = zeros(K-1, K-1);
G_u_h = zeros(K-1, 1);
I = eye(K-1);
Cost = zeros(5,1);
counter=0;

%Step 0: we initialise policy iteration with any proper policy.
%We start with all HOVER as our initial guess for the proper policy. If it
%is proper (eigenvalues of P for this policy are not equal to 1) then we
%can proceed. otherwise we generate another policy at random until we get a
%proper one
u_opt_ind = HOVER * ones(K-1,1);  
P_u_test = zeros(K-1, K-1);
a=0;
while ~a
    for i=1:K-1
            P_u_test(i,:) = P(i,:,u_opt_ind(i));
    end
    problem =find(eig(P_u_test) >(1-1e-10),1);
    if isempty(problem)
       a=1;
       disp('This is a proper policy');
     else
        u_opt_ind=randi([1,5],[K-1,1]);
        disp('Trying another policy');
    end
end 


% Matrices used in the recursion algorithm 
J_opt_previous = -1*ones(K-1,1); % matrix to hold hte cost to go of the previous step. 
u_previous = zeros(K-1,1);% matrix to hold the index of the optimal policy for the previous step

%% Policy Iteration
% We need to let the recursion run until the optimal policy between two
% iterations doesn't change anymore. 
%However when testing it would sometimes get caught alternating bewteen 2
%alternating policies and never stop the loop, so we added the condition
%that if the optimal cost to go has converged, then we can assume the
%policy iteration has reached its objective

 while ~(isequal(u_previous,u_opt_ind))|| max(abs(J_opt_previous-J_opt)) < 1e-20
    u_previous = u_opt_ind;
    J_opt_previous = J_opt;
    counter=counter+1;
%step 1: policy evaluation
        for i=1:K-1
            G_u_h(i) = G(i,u_previous(i));
            P_u_h(i,:) = P(i,:,u_previous(i));
        end

    %ref: matlab operator \ instead of inv(A)*b
    J_opt = (I - P_u_h)\ G_u_h;
    
    for i = 1:K-1
        %for every position, get the cost for a given control input 
        for l = 1:5
            Cost(l) =  P(i, :, l)*J_opt + G(i, l);
        end
        %step 2: policy update 
        %ref: matlab operator ~ to ignore function output: 
        %https://ch.mathworks.com/help/matlab/matlab_prog/ignore-function-outputs.html
        %ref: matlab min 
        %https://ch.mathworks.com/help/matlab/ref/min.html#d122e829674
        [~, u_opt_ind(i)] = min(Cost);
    end
end

% Add back the policy and cost-to-go of terminal state
J_opt = [J_opt(1:TERMINAL_STATE_INDEX-1,:);...
        0;...
        J_opt(TERMINAL_STATE_INDEX:end,:)];
    
u_opt_ind = [u_opt_ind(1:TERMINAL_STATE_INDEX-1);...
            HOVER;...
            u_opt_ind(TERMINAL_STATE_INDEX:end)];
        disp('number of iterations:');
        disp(counter);

end



