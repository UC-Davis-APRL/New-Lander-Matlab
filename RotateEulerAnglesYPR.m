function object = RotateEulerAnglesYPR(object, y, p, r)
% y about X, p about Y, r about Z
% object: N x 3 row vectors

arguments
    object double
    y double
    p double
    r double
end

N = size(object,1);

% allow scalars or N-vectors
if isscalar(y), y = repmat(y,N,1); end
if isscalar(p), p = repmat(p,N,1); end
if isscalar(r), r = repmat(r,N,1); end

% --- Rx(y) ---
cy = cos(y); sy = sin(y);
Rx = zeros(3,3,N);
Rx(1,1,:) = 1;
Rx(2,2,:) = cy;   Rx(2,3,:) = -sy;
Rx(3,2,:) = sy;   Rx(3,3,:) = cy;

% --- Ry(p) ---
cp = cos(p); sp = sin(p);
Ry = zeros(3,3,N);
Ry(2,2,:) = 1;
Ry(1,1,:) = cp;   Ry(1,3,:) = sp;
Ry(3,1,:) = -sp;  Ry(3,3,:) = cp;

% --- Rz(r) ---
cr = cos(r); sr = sin(r);
Rz = zeros(3,3,N);
Rz(3,3,:) = 1;
Rz(1,1,:) = cr;   Rz(1,2,:) = -sr;
Rz(2,1,:) = sr;   Rz(2,2,:) = cr;

% vectors as 3x1xN
v = permute(object, [2 3 1]);   % 3 x 1 x N

% apply in order: X then Y then Z
v = pagemtimes(Rx, v);
v = pagemtimes(Ry, v);
v = pagemtimes(Rz, v);

% back to N x 3
object = squeeze(permute(v, [3 1 2]));
end