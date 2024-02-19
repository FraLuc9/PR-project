1;


#2D stuff

function T = v2t(p)
    T = eye(3);
    T(1:2,1:2) = getRotMat(p(3));
    T(1:2,3) = p(1:2);
end

function p = t2v(T)
    p = zeros(3,1);
    p(1:2) = T(1:2,3);
    p(3) = atan2(T(2,1),T(1,1));
end

function R = getRotMat(a)
    R = [cos(a) -sin(a);
         sin(a) cos(a)];
end