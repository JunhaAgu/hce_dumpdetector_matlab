function w = so3Log(A)
w_hat = logm(A); % 3x3
w = [-w_hat(2,3);w_hat(1,3);-w_hat(1,2)];

end