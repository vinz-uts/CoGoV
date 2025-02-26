function T = ell_simdiag(A, B)
%
% ELL_SIMDIAG - computes the transformation matrix that simultaneously
%               diagonalizes two symmetric matrices.
%
%
% Description:
% ------------
%
%    T = ELL_SIMDIAG(A, B)  Given two symmetric matrices, A and B, with A being
%                           positive definite, find nonsingular transformation
%                           matrix T such that
%                                       T A T' = I
%                                       T B T' = D
%                           where I is identity matrix, and D is diagonal. 
%
%    General info.
%    Two matrices are said to be simultaneously diagonalizable if they are
%    diagonalized by a same invertible matrix. That is, they share full rank
%    of linearly independent eigenvectors. Two square matrices of the same
%    dimension are simultaneously diagonalizable if and only if they are
%    diagonalizable and commutative, or these matrices are symmetric and
%    one of them is positive definite.
%
%
% Output:
% -------
%
%    T - tranformation matrix.
%
%
% See also:
% ---------
%
%    SVD, GSVD.
%

%
% Author:
% -------
%
%    Alex Kurzhanskiy <akurzhan@eecs.berkeley.edu>
%

  if ~(isa(A, 'double')) | ~(isa(B, 'double'))
    error('ELL_SIMDIAG: both arguments must be symmetric matrices of the same dimension.');
  end
  if (A ~= A') | (min(eig(A)) <= 0)
    error('ELL_SIMDIAG: first argument must be symmetric positive definite matrix.');
  end
  if (B ~= B')
    error('ELL_SIMDIAG: second argument must be symmetric matrix.');
  end

  m = size(A, 1);
  n = size(B, 1);
  if m ~= n
    error('ELL_SIMDIAG: both matrices must be of the same dimension.');
  end
  if m > rank(A)
    error('ELL_SIMDIAG: first argument must be strictly positive definite matrix.');
  end

  [U1 S V] = svd(A);
  U        = U1 * inv(sqrtm(S));
  [U2 D V] = svd(U'*B*U);
  T        = U2' * U';

  return;
