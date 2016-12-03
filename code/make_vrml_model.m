function make_vrml_model(model_filename, X1_3D, x_rect_ic, tri_pts, plane_names);
% construct textured vrml model
% 
% Input parameters:
%   - model_filename = name of output VRML file
%   - X1_3D = 3xn matrix of n 3D points
%   - x_rect_ic = cell array of 2x3 triplets of 2D points, this determines the texture coordinate
%           note that you need to specify the texture coordinates according to the specifications given
%           in VRML, where (0,0) is lower left corner and (1,1) corresponds to the upper left corner
%           The image is scaled; if it is not a square image, the aspect ratio will be changed
%   - tri_pts = 3xn array of point triplets that form planes
%   - plane_names = images of the planes given by URLs as in an html file
% 
% convert to vrml texture coords
% Source: http://users.rsise.anu.edu.au/~luke/cvcourse_files/Assignment2/make_vrml_model.m
% Modified by Xiuwen Liu on 9-14-2006
%   to output triangle indices correctly
for n = 1:length(x_rect_ic)
    x_tex_coords{n} = x_rect_ic{n};
           % x_rect_ic{n}(1,:) / (max(x_rect_ic{n}(1,:)) - min(x_rect_ic{n}(1,:))); ...
           % 1 - x_rect_ic{n}(2,:) / (max(x_rect_ic{n}(2,:)) - min(x_rect_ic{n}(2,:)))]; 
end; %for

fid = fopen(model_filename,'w');
fprintf(fid,'#VRML V2.0 utf8\n\n');
fprintf(fid,'Background {\n    skyColor    [0.9, 0.95, 1]\n}\n\n');
fprintf(fid,'DEF MYPOINTS Coordinate {\n  point [ ');
for i = 1:size(X1_3D,2)
    if i>1
        fprintf(fid,','); 
    end; %if
    point_3D = X1_3D(1:3,i)';
    fprintf(fid,'\n        %6.2f  %6.2f %6.2f', point_3D); 
end; %for
fprintf(fid,' ]\n}\n\n');

% for each polygon do:-
for n = 1:size(tri_pts,2)
    url = sprintf('      url "%s"', plane_names{n});
    fprintf(fid, 'Shape {\n  appearance Appearance {\n    texture ImageTexture {\n');
    fprintf(fid, '%s', url);
    fprintf(fid, '\n    }\n  }\n'); 
    fprintf(fid, '  geometry IndexedFaceSet { \n');
    fprintf(fid, '    solid FALSE\n');                  
    fprintf(fid, '    coord USE MYPOINTS\n'); 
    fprintf(fid, '    coordIndex [ ');
    fprintf(fid, '\n        %d, %d, %d, -1 ]\n', tri_pts(:,n)-1); 
    fprintf(fid, '    texCoord  TextureCoordinate {\n      point [');
    for i = 1:3
        if i>1
            fprintf(fid,','); 
        end; %if
        texture_coord = x_tex_coords{n}(1:2,i)';
        fprintf(fid,'\n        %6.2f  %6.2f', texture_coord); 
    end; %for
    fprintf(fid,'] \n    }\n');                            
    fprintf(fid,'    texCoordIndex [ 0, 1, 2, -1 ]\n');
    fprintf(fid,'  }\n}\n\n');
end; %for
fclose(fid);