function dXdt = ell_iesm_ode(t, X, xl0, l0, mydata, n, back)
%
% ELL_IESM_ODE - ODE for the shape matrix of the internal ellipsoid.
%

  global ellOptions;

  if back > 0
    t = -t;
    F = ell_value_extract(mydata.Phi, t, [n n]);
    s = -1;
  else
    F = ell_value_extract(mydata.Phinv, t, [n n]);	  
    s = 1;
  end

  A     = ell_value_extract(mydata.A, t, [n n]);
  BPBsr = ell_value_extract(mydata.BPBsr, t, [n n]);
  X     = reshape(X, n, n);
  Y     = sqrtm(X);

  l = BPBsr * F' * l0;
  xl0 = Y * F' * l0;
  if norm(l) < ellOptions.abs_tol
    S = eye(n);
  else
    S = ell_valign(xl0, l);
  end

  dXdt = s*A*X + s*X*A' + Y*S*BPBsr + BPBsr*S'*Y;
  dXdt = reshape(0.5*(dXdt + dXdt'), n*n, 1);

  return;
