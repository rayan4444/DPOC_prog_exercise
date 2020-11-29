%This now just compares the outputs since we are pretty much sure for the
%example worlds

function[Jp_check_vi_lp,Jp_check_pi_lp,Jp_check_vi_pi,delta_Jp_vi_lp,delta_Jp_pi_lp,delta_Jp_vi_pi] = CheckMatrix(J_opt_vi,J_opt_pi,J_opt_lp,u_opt_ind_vi,u_opt_ind_pi,u_opt_ind_lp)

% P_ex= load('example_P');
% P_array = struct2array(P_ex);
% 
% G_ex = load('example_G');
% G_array =  struct2array(G_ex);

%Self-Computed P and G:
%P=ComputeTransitionProbabilities(stateSpace, map);
%G = ComputeStageCosts(stateSpace, map);

%Checks if both matrices are the same (output 0 if no, 1 if yes)
%P_check = isequal(P, P_array);
%G_check = isequal(G, G_array);


%Check for equality of the outputs
delta_Jp_vi_lp = J_opt_vi - J_opt_lp;
delta_Jp_pi_lp = J_opt_vi - J_opt_lp;
delta_Jp_vi_pi = J_opt_vi - J_opt_pi;

Jp_check_vi_lp = isequal(u_opt_ind_vi, u_opt_ind_lp);
Jp_check_pi_lp = isequal(u_opt_ind_pi,u_opt_ind_lp);
Jp_check_vi_pi = isequal(u_opt_ind_vi,u_opt_ind_pi);
end