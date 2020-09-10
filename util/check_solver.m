function [solver] = check_solver()
%% Function useful to check 'gurobi' solver 
% If not present, default solver is 'bmibnb' 

%%%%%%% ATTENTION: This function does NOT check gurobi's license validity 
%%% It only checks the presence of the solver 

index = -1;
exists = false ;
solvername='bmibnb';

[solvers,found] = getavailablesolvers(0);

for i = 1:length(solvers)
    if (strcmpi(solvers(i).tag,'GUROBI')==1 && strcmpi(solvers(i).version,'GUROBI')==1 )
        index  = i ;
    end
end

if index ~= -1
    exists = found(index);
else
    exists = false;
end

if exists
    solvername = 'gurobi';
else
    solvername = 'bmibnb';
end

solver=solvername;

end

