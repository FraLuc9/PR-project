1; 



function S = skew(t)
    S = [0    -t(3)  t(2);
         t(3)  0    -t(1);
        -t(2)  t(1)  0];
endfunction

function r = flatten4(X)
    r = [X(1:3, 1); X(1:3,2); X(1:3,3); X(1:3,4)];
endfunction

function r = flatten3(R)
    r = [R(1:3, 1); R(1:3,2); R(1:3,3)];
endfunction

function [e, Ji, Jj] = poseErrorAndJacobian(Xi, Xj, Z)
    Ri = Xi(1:3,1:3);
    Rj = Xj(1:3,1:3);
    ti = Xi(1:3,4);
    tj = Xj(1:3,4);

    # prediction
    g = inv(Xi)*Xj;

    # CHORDAL ERROR
    Zhat = flatten4(g);
    e = Zhat - flatten4(Z);
    
    dRx0 = [0 0 0; 0 0 -1; 0 1 0];
    dRy0 = [0 0 1; 0 0 0; -1 0 0];
    dRz0 = [0 -1 0; 1 0 0; 0 0 0];
    rx = flatten3(transpose(Ri)*dRx0*Rj);
    ry = flatten3(transpose(Ri)*dRy0*Rj);
    rz = flatten3(transpose(Ri)*dRz0*Rj);
    #CHORDAL JACOBIANS
    Jj = zeros(12,6);
    Jj(10:12,1:3) = transpose(Ri);
    Jj(1:9,4:6) = [rx ry rz];
    Jj(10:12,4:6) = -transpose(Ri)*skew(tj);
    Ji = -Jj;

endfunction


# testino

X1 = [1 0 0 1;
      0 1 0 1;
      0 0 1 0;
      0 0 0 1];
X2 = [1 0 0 2;
      0 1 0 2;
      0 0 1 0;
      0 0 0 1];

Z = inv(X1)*X2;

[e, j1, j2] = poseErrorAndJacobian(X1, X2, Z);
j1
j2