function R = Rotaion_Matrix(Q, alpha);
  R = [cos(Q) -sin(Q)*cos(alpha) sin(Q)*sin(alpha);
     sin(Q) cos(Q)*cos(alpha) -sin(alpha)*cos(Q);
     0 sin(alpha)  cos(alpha)];
end