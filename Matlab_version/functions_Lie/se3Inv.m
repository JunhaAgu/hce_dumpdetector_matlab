function [ T_inv ] = se3Inv( T )
R=T(1:3,1:3); t=T(1:3,4);
T_inv = [R.',-R.'*t;0,0,0,1];
end


