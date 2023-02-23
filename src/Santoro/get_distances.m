function [D, sz] = get_distances(map, mu_r, sigma_r)

sz = size(map,2);

D = sqrt(bsxfun(@minus,map(1,:),map(1,:)').^2 + bsxfun(@minus,map(2,:),map(2,:)').^2) + normrnd(mu_r,sigma_r,sz,sz);
D = D - diag(diag(D));

for i = 1:sz
    for j = 1:sz
        if j <= i
            D(i,j) = (D(i,j) + D(j,i)) / 2;
            D(j,i) =  D(i,j);
        end
    end
end

    