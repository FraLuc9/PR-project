1;


#2D stuff

function T = v2t(p)
    T = eye(3);
    T(1:2,1:2) = getRotMat(p(3));
    T(1:2,3) = p(1:2);
endfunction

function p = t2v(T)
    p = zeros(3,1);
    p(1:2) = T(1:2,3);
    p(3) = atan2(T(2,1),T(1,1));
endfunction

function R = getRotMat(a)
    R = [cos(a) -sin(a);
         sin(a) cos(a)];
endfunction


#3D
#robot in SE(3) flattened on the ground
#transforms a SE(2)->R3 point into a SE(3) homogeneous matrix
#z axis is left untouched
function T = v2t3D(p)
    T = eye(4);
    T(1:2,1:2) = getRotMat(p(3));
    T(1:2,4) = p(1:2);
endfunction

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

