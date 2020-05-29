function x = model_function(x,u,function_name)
    %% MODEL FUNCTION - function of the model
    %  Call the matlab function 'function_name' of the system model
    %  starting with initial states x and applying a constant input u.
    x = feval(function_name,x,u);
end

