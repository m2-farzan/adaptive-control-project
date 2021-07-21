% Updates everything in the world
% S = [X; W]
function Sdot = Sdot(t, S)
    X = S(1:7);
    W = S(8:18);
    [Wdot_, u] = Wdot(X, W, z_r(t));
    Xdot_ = Xdot(X, u, t);
    Sdot = [Xdot_; Wdot_];
end