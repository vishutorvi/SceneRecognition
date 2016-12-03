% Starter code prepared by James Hays for Computer Vision

%This function will predict the category for every test image by finding
%the training image with most similar features. Instead of 1 nearest
%neighbor, you can vote based on k nearest neighbors which will increase
%performance (although you need to pick a reasonable value for k).

function predicted_categories = nearest_neighbor_classify(train_image_feats, train_labels, test_image_feats)
% image_feats is an N x d matrix, where d is the dimensionality of the
%  feature representation.
% train_labels is an N x 1 cell array, where each entry is a string
%  indicating the ground truth category for each training image.
% test_image_feats is an M x d matrix, where d is the dimensionality of the
%  feature representation. You can assume M = N unless you've modified the
%  starter code.
% predicted_categories is an M x 1 cell array, where each entry is a string
%  indicating the predicted category for each test image.

%{
Useful functions:
 matching_indices = strcmp(string, cell_array_of_strings)
   This can tell you which indices in train_labels match a particular
   category. Not necessary for simple one nearest neighbor classifier.

 D = vl_alldist2(X,Y) 
    http://www.vlfeat.org/matlab/vl_alldist2.html
    returns the pairwise distance matrix D of the columns of X and Y. 
    D(i,j) = sum (X(:,i) - Y(:,j)).^2
    Note that vl_feat represents points as columns vs this code (and Matlab
    in general) represents points as rows. So you probably want to use the
    transpose operator ' 
   vl_alldist2 supports different distance metrics which can influence
   performance significantly. The default distance, L2, is fine for images.
   CHI2 tends to work well for histograms.
 
  [Y,I] = MIN(X) if you're only doing 1 nearest neighbor, or
  [Y,I] = SORT(X) if you're going to be reasoning about many nearest
  neighbors 

%}
categories = {'Kitchen', 'Store', 'Bedroom', 'LivingRoom', 'Office', ...
       'Industrial', 'Suburb', 'InsideCity', 'TallBuilding', 'Street', ...
       'Highway', 'OpenCountry', 'Coast', 'Mountain', 'Forest'};
len = length(test_image_feats);
predicted_categories = cell(len,1);
nearest_neighbor_labels = cell(len,2);
train_image_feats_t = train_image_feats';
test_image_feats_t = test_image_feats';
%categories = strings(10,1);
count = zeros(10,1);
D = vl_alldist2(train_image_feats_t,test_image_feats_t);
[row,col] = size(D);
for i=1:row
   testedValue = D(:,i);
   for j=1:len
       nearest_neighbor_labels{j,1} = train_labels{j,1};
       nearest_neighbor_labels{j,2} = testedValue(j,1);
   end
   sorted_nearest_neighbor_labels = sortcell(nearest_neighbor_labels,2);
   for k = 1:10
       category{k} = sorted_nearest_neighbor_labels{k,1};
   end
   [str, ~, lab] = unique(category);
   cnt = sum(bsxfun(@eq, lab(:), 1:max(lab))).';
   highestCnt = find(cnt==max(cnt));
   uniqueCategories = str(highestCnt);
   if length(uniqueCategories) > 1
        uniqueCategories = uniqueCategories(1);
   end
   %category = uniqueCategories(max(count));
   match_indices = strcmp(categories,uniqueCategories);
   predicted_categories{i,1} = categories{find(match_indices==1)};% categories{find(match_indices)};
end
end