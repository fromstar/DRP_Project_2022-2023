function [map_hat, D] = s_mds(D, sz, dim)

% Double centering operation
H = eye(sz) - ones(sz,1)*ones(1,sz)./sz;
B = -0.5*H*D.^2*H;

% Eigenvalue problem of B

[V,Lambda]  = eig(B);
lambda      = diag(Lambda);
[lambda,IX] = sort(lambda,'descend');
V           = V(:,IX);

%Representation in R^r: first r principal components
lambda_r     = lambda([1,2,3]);
V_r          = V(:,[1,2,3]);

% Projection of centroids onto principal components
map_hat = zeros(sz, dim);
for i = 1:dim
    map_hat(:,i) = sqrt(lambda_r(i)).*V_r(:,i);
end

map_hat = map_hat';