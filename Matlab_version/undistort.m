function [u_u, v_u] = undistort(u,v,K, distort)
k1 =distort(1,1);
k2 = distort(1,2);
k3 = distort(1,3);
x = (u-K(1,3))/K(1,1);
y = (v-K(2,3))/K(2,2);
r = sqrt(x.^2 + y.^2);
r_radial = ones(size(r)) + k1*r.^2 + k2*r.^4 + k3*r.^6;
x_u = x.*r_radial; y_u = y.*r_radial;
u_u = K(1,1)*x_u+K(1,3);
v_u = K(2,2)*y_u+K(2,3);
end