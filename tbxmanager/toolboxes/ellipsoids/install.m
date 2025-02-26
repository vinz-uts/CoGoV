function install(root)
%
% Install Ellipsoidal Toolbox.
%

  fprintf('Installing Ellipsoidal Toolbox version 1.1.2 ...\n\n');

  if ~exist('root', 'var')
    root = pwd;
  end

  adddir([root]);
  adddir([root '/auxiliary']);
  adddir([root '/control']);
  adddir([root '/control/auxiliary']);
  adddir([root '/demo']);
  adddir([root '/graphics']);
  adddir([root '/solvers']);
  adddir([root '/solvers/gradient']);
  
  fprintf('To finish the installation, go to ''File'' --> ''Set Path...'' and click ''Save''.\n\n');

  return;



  

%%%%%%%%

function adddir(directory)

  if isempty(dir(directory))
    error(['Directory ' directory ' not found!']);
  else
    addpath(directory);
  end

  return;
