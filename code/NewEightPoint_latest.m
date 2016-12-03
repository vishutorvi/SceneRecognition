function NewEightPoint_latest
tic
clear all;
%image1 = imread('rgb01.jpg');
%image2 = imread('rgb02.jpg');
load xypoint_second_set.txt
load xypoint_second_set_1.txt
%load('synthetic_model.mat');
%4 pair of correspondence point
xypoint1 = xypoint_second_set;%userInputOfCorrespondencePoint(image1);
xypoint2 = xypoint_second_set_1;%userInputOfCorrespondencePoint(image2);
figure(3);

subplot(1,2,1);
axis('ij');
hold on;
for i=1:size(xypoint1,1),
    hh=plot(xypoint1(i,1),xypoint1(i,2),'r+');
end
title('Left Image Points');

subplot(1,2,2);
axis('ij');
hold on;
for i=1:size(xypoint2,1),
    plot(xypoint2(i,1),xypoint2(i,2),'r+');
end
title('Right Image Points');
%8-point algorithm
% First we need to normalize the inputs
mean1 = mean(xypoint1(:,1:2));
mean2 = mean(xypoint2(:,1:2));
T_trans_1=[1 0 -mean1(1);0 1 -mean1(2); 0 0 1];
T_trans_2=[1 0 -mean2(1);0 1 -mean2(2); 0 0 1];
xypoint1 = [xypoint1(:,1) xypoint1(:,2) ones(8,1)];
xypoint11=T_trans_1*xypoint1';
rms_image1=sqrt(sum(sum(xypoint11(1:2,:).^2))/size(xypoint2,2));
T1_scale=[sqrt(2)/rms_image1 0 0; 0 sqrt(2)/rms_image1 0; 0 0 1];
norm_points_1=T1_scale*xypoint11;
T_norm1=T1_scale*T_trans_1;

xypoint2 = [xypoint2(:,1) xypoint2(:,2) ones(8,1)];
xypoint22=T_trans_2*xypoint2';
rms_image2=sqrt(sum(sum(xypoint22(1:2,:).^2))/size(xypoint2,2));
T_scale_2=[sqrt(2)/rms_image2 0 0; 0 sqrt(2)/rms_image2 0; 0 0 1];
norm_points_2=T_scale_2*xypoint22;
T_norm_2=T_scale_2*T_trans_2;

%We need to construct the A matrix
A=zeros([size(norm_points_1,2) 9]);
for i=1:size(norm_points_1,2)
    A(i,:)=[norm_points_1(:,i)'*norm_points_2(1,i) norm_points_1(:,i)'*norm_points_2(2,i) norm_points_1(:,i)'];
end
[U,S,V]=svd(A);
f_norm=V(:,end);
F_norm=reshape(f_norm,[3 3])';
[U1,D,V1]=svd(F_norm);
D(3,3)=0.0; %We need to enforce the rank of F to be 2
F_norm=U1*D*V1';
%We need to denormalize to get fundamental matrix F
F=T_norm_2'*F_norm*T_norm1;

%Given intrinsix parameter
K1_cam = [-525 0 320;
    0 -525 240;
    0 0 1];

%We need to get the essential matrix
E=K1_cam'*F*K1_cam;
W=[0 -1 0;1 0 0; 0 0 1];
[UE,SE,VE]=svd(E);
if det(UE) < 0
    UE(:,3)=-UE(:,3);
end

if det(VE) < 0
    VE(:,3)=-VE(:,3);
end

R_1=UE*W*VE';
R_2=UE*W'*VE';
T_1=UE(:,3);
T_2=0-UE(:,3);
norm_points_1=inv(K1_cam)*xypoint1';
norm_points_2=inv(K1_cam')*xypoint2';
z_pos_max=0;
R_final = 0;
T_final = 0;
XYZs_1E = 0;
for i=1:2
    if i==1
        R_sel=R_1;
    else
        R_sel=R_2;
    end
    for j=1:2
        if j==1
            T_sel=T_1;
        else
            T_sel=T_2;
        end
        XYZs_1=xypoint1'*0;
        XYZs_2=XYZs_1;
        for kk=1:size(xypoint1,1)
            A=[-1 0 norm_points_1(1,kk) 0
                0  -1 norm_points_1(2,kk) 0
                -R_sel(1,1)+norm_points_2(1,kk)*R_sel(3,1) -R_sel(1,2)+norm_points_2(1,kk)*R_sel(3,2) -R_sel(1,3)+norm_points_2(1,kk)*R_sel(3,3)  -T_sel(1)+norm_points_2(1,kk)*T_sel(3)
                -R_sel(2,1)+norm_points_2(2,kk)*R_sel(3,1) -R_sel(2,2)+norm_points_2(2,kk)*R_sel(3,2) -R_sel(2,3)+norm_points_2(2,kk)*R_sel(3,3)  -T_sel(2)+norm_points_2(2,kk)*T_sel(3)];
            [U, S, V]=svd(A);
            XYZ_1=V(1:3,end)./V(4,end);
            XYZs_1(:,kk)=XYZ_1;
            XYZ_2=R_sel*XYZ_1+T_sel;
            XYZs_2(:,kk)=XYZ_2;
        end
        fprintf(1,'Depth of Z from Left camera = %2.5f\n', XYZs_1(3,:));
        fprintf(1,'Depth of Z from Right camera = %2.5f\n', XYZs_2(3,:));
        z_depth_1 = XYZs_1(:,3);
        z_depth_2 = XYZs_2(:,3);
        if ((z_depth_1 > 0) & (z_depth_2 > 0))
            R_final=R_sel;
            T_final=T_sel;
            XYZs_1E=XYZs_1;
            XYZs_2E=XYZs_2;
        end
        fprintf(1,'For R = %d and T = %d\n', i, j);
        XYZs_1
        XYZs_2
    end
end
%printing final values of R and T 
R_final
T_final

%Assume we know the true scale;
% True_scale=norm(T_true);
% TE=TE/norm(TE);
XYZs_1=zeros([4 size(xypoint1',2)]);
XYZs_2=XYZs_1E;
for kk=1:size(xypoint1,1)
    A=[-1 0 norm_points_1(1,kk) 0
        0  -1 norm_points_1(2,kk) 0
        -R_final(1,1)+norm_points_2(1,kk)*R_final(3,1) -R_final(1,2)+norm_points_2(1,kk)*R_final(3,2) -R_final(1,3)+norm_points_2(1,kk)*R_final(3,3)  -T_final(1)+norm_points_2(1,kk)*T_final(3)
        -R_final(2,1)+norm_points_2(2,kk)*R_final(3,1) -R_final(2,2)+norm_points_2(2,kk)*R_final(3,2) -R_final(2,3)+norm_points_2(2,kk)*R_final(3,3)  -T_final(2)+norm_points_2(2,kk)*T_final(3)];
     [U, S, V]=svd(A);
     
    XYZ_1=V(:,end)/V(4,end);
    XYZs_1(:,kk)=XYZ_1;
    XYZ_2=[R_final, T_final;0 0 0 1]*XYZ_1; % 3D points in other camera.
    XYZs_2(:,kk)=XYZ_2;
end

xx = XYZs_1(1,:);
yy = XYZs_1(2,:);
zz = XYZs_1(3,:);

figure(1),
plot3(xx, yy, zz, 'r+');title('3D plots');

load triangle_values.txt
selected_3d_points = userSelectionfForInputOfCorrespondencePoint(xx,yy,XYZs_1);
xpoints = [selected_3d_points(1:4,1);selected_3d_points(1,1);selected_3d_points(3,1)];
ypoints = [selected_3d_points(1:4,2);selected_3d_points(1,2);selected_3d_points(3,2)];
zpoints = [selected_3d_points(1:4,3);selected_3d_points(1,3);selected_3d_points(3,3)];
triangle = [xpoints,ypoints,zpoints];

figure,
hold on
plot3(triangle(:,1),triangle(:,2),triangle(:,3),'r-');
plot3(selected_3d_points(:,1),selected_3d_points(:,2),selected_3d_points(:,3),'b+');title('Triangle Points');
hold off
view(3)

load quadrilateral.txt
xpoints = [selected_3d_points(1:4,1);selected_3d_points(1,1)];
ypoints = [selected_3d_points(1:4,2);selected_3d_points(1,2)];
zpoints = [selected_3d_points(1:4,3);selected_3d_points(1,3)];
quadrilateralPoints = [xpoints, ypoints, zpoints];

figure,
hold on
plot3(selected_3d_points(:,1),selected_3d_points(:,2),selected_3d_points(:,3),'r+');title('quadrilateral Points');
plot3(quadrilateralPoints(:,1),quadrilateralPoints(:,2),quadrilateralPoints(:,3),'b-');
view(3);
toc
end