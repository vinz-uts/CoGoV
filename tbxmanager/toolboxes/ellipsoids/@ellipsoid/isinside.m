function [res, status] = isinside(E1, E2, s)
%
% ISINSIDE - checks if the intersection of ellipsoids contains the union or
%            intersection of given ellipsoids or polytopes.
%
%
% Description:
% ------------
%
% RES = ISINSIDE(E1, E2, s) Checks if the union (s = 'u') or intersection (s = 'i')
%                           of ellipsoids in E2 lies inside the intersection of
%                           ellipsoids in E1.
%                           Ellipsoids in E1 and E2 must be of the same dimension.
%                           s = 'u' (default) - union of ellipsoids in E2.
%                           s = 'i' - intersection.
%   RES = ISINSIDE(E, P, s) Checks if the union (s = 'u') or intersection (s = 'i')
%                           of polytopes in P lies inside the intersection of
%                           ellipsoids in E.
%                           Ellipsoids in E and polytopes in P must be
%                           of the same dimension.
%                           s = 'u' (default) - union of polytopes in P.
%                           s = 'i' - intersection.
%
%    To check if the union of ellipsoids E2 belongs to the intersection of
%    ellipsoids E1, it is enough to check that every ellipsoid of E2 is 
%    contained in every ellipsoid of E1.
%    Checking if the intersection of ellipsoids in E2 is inside intersection E1
%    can be formulated as quadratically constrained quadratic programming (QCQP)
%    problem.
%    Let E(q, Q) be an ellipsoid with center q and shape matrix Q.
%    To check if this ellipsoid contains the intersection of ellipsoids
%    E(q1, Q1), E(q2, Q2), ..., E(qn, Qn), we define the QCQP problem:
%                      J(x) = <(x - q), Q^(-1)(x - q)> --> max
%    with constraints:
%                       <(x - q1), Q1^(-1)(x - q1)> <= 1   (1)
%                       <(x - q2), Q2^(-1)(x - q2)> <= 1   (2)
%                       ................................
%                       <(x - qn), Qn^(-1)(x - qn)> <= 1   (n)
%
%    If this problem is feasible, i.e. inequalities (1)-(n) do not contradict,
%    or, in other words, intersection of ellipsoids E(q1, Q1), E(q2, Q2), ..., E(qn, Qn)
%    is nonempty, then we can find vector y such that it satisfies inequalities (1)-(n)
%    and maximizes function J. If J(y) <= 1, then ellipsoid E(q, Q) contains
%    the given intersection, otherwise, it does not.
%
%    The intersection of polytopes is a polytope, which is computed
%    by the standard routine of MPT. If the vertices of this polytope
%    belong to the intersection of ellipsoids, then the polytope itself
%    belongs to this intersection.
%    Checking if the union of polytopes belongs to the intersection
%    of ellipsoids is the same as checking if its convex hull belongs 
%    to this intersection.
%
%    We use YALMIP as interface to the optimization tools.
%    (http://control.ee.ethz.ch/~joloef/yalmip.php)
%
%
% Output:
% -------
%
%    RES - result:
%           -1 - problem is infeasible,
%                for example, if s = 'i', but the intersection of ellipsoids in E2
%                is an empty set;
%            0 - intersection is empty;
%            1 - if intersection is nonempty.
%    S   - (optional) status variable returned by YALMIP.
%
%
% See also:
% ---------
%
%    ELLIPSOID/ELLIPSOID, INTERSECT, ISINTERNAL,
%    POLYTOPE/POLYTOPE.
%

%
% Author:
% -------
%
%    Alex Kurzhanskiy <akurzhan@eecs.berkeley.edu>
%

  global ellOptions;

  if ~isstruct(ellOptions)
    evalin('base', 'ellipsoids_init;');
  end

  if ~(isa(E1, 'ellipsoid'))
    error('ISINSIDE: first input argument must be ellipsoid.');
  end

  if ~(isa(E2, 'ellipsoid')) & ~(isa(E2, 'polytope'))
    error('ISINSIDE: second input arguments must be ellipsoids or polytope.');
  end

  if (nargin < 3) | ~(ischar(s))
    s = 'u';
  end

  status = [];

  if isa(E2, 'polytope')
    [m, n] = size(E2);
    if s == 'i'
      PP = E2(1);
      for j = 1:n
        PP = PP & E2(j);
      end
      X = extreme(PP);
    else
      X = [];
      for j = 1:n
        X = [X; extreme(E2(j))];
      end
    end
    if isempty(X)
      res = -1;
    else
      res = min(isinternal(E1, X', 'i'));
    end

    if nargout < 2
      clear status;
    end

    return;
  end

  if s == 'u'
    [m, n] = size(E1);
    res    = 1;
    for i = 1:m
      for j = 1:n
        if min(min(contains(E1(i, j), E2))) < 1
          res = 0;
          if nargout < 2
            clear status;
          end
          return;
        end
      end
    end
  elseif min(size(E2) == [1 1]) == 1
    res = 1;
    if min(min(contains(E1, E2))) < 1
      res = 0;
    end
  else
    dims = dimension(E1);
    m    = min(min(dims));
    n    = max(max(dims));
    dims = dimension(E2);
    k    = min(min(dims));
    l    = max(max(dims));
    if (m ~= n) | (k ~= l) | (k ~= m)
      error('ISINSIDE: ellipsoids must be of the same dimension.');
    end
    if ellOptions.verbose > 0
      fprintf('Invoking YALMIP...\n');
    end
    [m, n] = size(E1);
    res    = 1;
    for i = 1:m
      for j = 1:n
        [res, status] = qcqp(E2, E1(i, j));
        if res < 1
          if nargout < 2
            clear status;
          end
          return;
	end
      end
    end
  end

  if nargout < 2
    clear status;
  end

  return;





%%%%%%%%

function [res, status] = qcqp(EA, E)
%
% QCQP - formulate quadratically constrained quadratic programming problem
%        and invoke external solver.
%

  global ellOptions;

  [q, Q] = parameters(E(1, 1));
  if size(Q, 2) > rank(Q)
    if ellOptions.verbose > 0
      fprintf('QCQP: Warning! Degenerate ellipsoid.\n');
      fprintf('      Regularizing...\n');
    end
    Q = regularize(Q);
  end
  Q = ell_inv(Q);
  Q = 0.5*(Q + Q');

  % YALMIP
  x         = sdpvar(length(Q), 1);
  objective = x'*Q*x + 2*(-Q*q)'*x + (q'*Q*q - 1);
  F         = set([]);
  
  [m, n] = size(EA);
  for i = 1:m
    for j = 1:n
      [q, Q] = parameters(EA(i, j));
      if size(Q, 2) > rank(Q)
        Q = regularize(Q);
      end
      Q = ell_inv(Q);
      Q = 0.5*(Q + Q');

      % YALMIP
      F = F + set(x'*Q*x + 2*(-Q*q)'*x + (q'*Q*q - 1) <= 0);
    end
  end

  % YALMIP 
  status = solvesdp(F, -objective, ellOptions.sdpsettings);
  
  if status.problem ~= 0
    % problem is infeasible, or global minimum cannot be found
    res = -1;
    return;
  end

  if double(objective) < ellOptions.abs_tol
    res = 1;
  else
    res = 0;
  end

  return;
