function NewEightPoint
tic
clear all;
image1 = imread('rgb10.jpg');
image2 = imread('rgb11.jpg');
load xypoint_second_set.txt
load xypoint_second_set_1.txt
%load('synthetic_model.mat');
%xypoint1 = userInputOfCorrespondencePoint(image1);
%xypoint2 = userInputOfCorrespondencePoint(image2);
%4 pair of correspondence point
xypoint1 = xypoint_second_set;%userInputOfCorrespondencePoint(image1);
xypoint2 = xypoint_second_set_1;%userInputOfCorrespondencePoint(image2);
figure(3);
clf;

subplot(2,2,1);
axis('ij');
hold on;
for i=1:size(xypoint1,1),
    hh=plot(xypoint1(i,1),xypoint1(i,2),'r+');
    set(hh,'MarkerSize',[12]);
    text(xypoint1(i,1),xypoint1(i,2),[sprintf('%d',i)],'FontSize',[20]);
end
title('Left Image Points');

subplot(2,2,2);
axis('ij');
hold on;
for i=1:size(xypoint2,1),
    hh=plot(xypoint2(i,1),xypoint2(i,2),'r+');
    set(hh,'MarkerSize',[12]);
    text(xypoint2(i,1),xypoint2(i,2),[sprintf('%d',i)],'FontSize',[20]);
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
T_norm_1=T1_scale*T_trans_1;

xypoint2 = [xypoint2(:,1) xypoint2(:,2) ones(8,1)];
xypoint22=T_trans_2*xypoint2';
rms_image2=sqrt(sum(sum(xypoint22(1:2,:).^2))/size(xypoint2,2));
T2_scale=[sqrt(2)/rms_image2 0 0; 0 sqrt(2)/rms_image2 0; 0 0 1];
norm_points_2=T2_scale*xypoint22;
T_norm_2=T2_scale*T_trans_2;

%We need to construct the A matrix
A=zeros([size(norm_points_1,2) 9]);
for i=1:size(norm_points_1,2),
    A(i,:)=[norm_points_1(:,i)'*norm_points_2(1,i) norm_points_1(:,i)'*norm_points_2(2,i) norm_points_1(:,i)'];
end
[U,S,V]=svd(A);
f_norm=V(:,end);
F_norm=reshape(f_norm,[3 3])';
[U1,D,V1]=svd(F_norm);
fprintf(1,'Changing %15.14f to zero to enforce the rank of F to be 2.\n', D(3,3));
D(3,3)=0.0; %We need to enforce the rank of F to be 2
F_norm=U1*D*V1';
%We need to denormalize to get fundamental matrix F
F=T_norm_2'*F_norm*T_norm_1;
K1_cam = [-525 0 320;
    0 -525 240;
    0 0 1];
%We need to get the essential matrix
E=K1_cam'*F*K1_cam;
W=[0 -1 0;1 0 0; 0 0 1];
[UE,SE,VE]=svd(E);
if det(UE) < 0,
    UE(:,3)=-UE(:,3);
end

if det(VE) < 0,
    VE(:,3)=-VE(:,3);
end

R_1=UE*W*VE';
R_2=UE*W'*VE';
T_1=UE(:,3);
T_2=0-UE(:,3);
norm_points_1=inv(K1_cam)*xypoint1';
norm_points_2=inv(K1_cam')*xypoint2';
z_pos_max=0;
for i=1:2,
    if i==1,
        Rcan=R_1;
    else
        Rcan=R_2;
    end
    for j=1:2,
        if j==1,
            Tcan=T_1;
        else
            Tcan=T_2;
        end
        XYZs_1=xypoint1'*0;
        XYZs_2=XYZs_1;
        for kk=1:size(xypoint1,1),
            AP=[-1 0 norm_points_1(1,kk) 0
                0  -1 norm_points_1(2,kk) 0
                -Rcan(1,1)+norm_points_2(1,kk)*Rcan(3,1) -Rcan(1,2)+norm_points_2(1,kk)*Rcan(3,2) -Rcan(1,3)+norm_points_2(1,kk)*Rcan(3,3)  -Tcan(1)+norm_points_2(1,kk)*Tcan(3)
                -Rcan(2,1)+norm_points_2(2,kk)*Rcan(3,1) -Rcan(2,2)+norm_points_2(2,kk)*Rcan(3,2) -Rcan(2,3)+norm_points_2(2,kk)*Rcan(3,3)  -Tcan(2)+norm_points_2(2,kk)*Tcan(3)];
            [UAP, SAP, VAP]=svd(AP);
            XYZ_1=VAP(1:3,end)./VAP(4,end);
            XYZs_1(:,kk)=XYZ_1;
            XYZ_2=Rcan*XYZ_1+Tcan;
            XYZs_2(:,kk)=XYZ_2;
        end
        fprintf(1,'Depth of Z from Left camera = %2.5f\n', XYZs_1(3,:));
        fprintf(1,'Depth of Z from Right camera = %2.5f\n', XYZs_2(3,:));
        zpos=sum(XYZs_1(3,:)>0)+sum(XYZs_2(3,:)>0);
        if zpos > z_pos_max;
            R_final=Rcan;
            T_final=Tcan;
            z_pos_max=zpos;
            XYZs_1E=XYZs_1;
            XYZs_2E=XYZs_2;
        end
        fprintf(1,'For R = %d and T = %d\n', i, j);
        XYZs_1
        XYZs_2
    end
end

fprintf(1,'The estimated R and T are the follow, which gives %d positive depths.\n', z_pos_max);
R_final
T_final

%Assume we know the true scale;
% True_scale=norm(T_true);
% TE=TE/norm(TE);
XYZs_1=zeros([4 size(xypoint1,1)]);
XYZs_2=XYZs_1;
for kk=1:size(xypoint1,1),
    AP=[-1 0 norm_points_1(1,kk) 0
        0  -1 norm_points_1(2,kk) 0
        -R_final(1,1)+norm_points_2(1,kk)*R_final(3,1) -R_final(1,2)+norm_points_2(1,kk)*R_final(3,2) -R_final(1,3)+norm_points_2(1,kk)*R_final(3,3)  -T_final(1)+norm_points_2(1,kk)*T_final(3)
        -R_final(2,1)+norm_points_2(2,kk)*R_final(3,1) -R_final(2,2)+norm_points_2(2,kk)*R_final(3,2) -R_final(2,3)+norm_points_2(2,kk)*R_final(3,3)  -T_final(2)+norm_points_2(2,kk)*T_final(3)];
     [UAP, SAP, VAP]=svd(AP);
     
    XYZ_1=VAP(:,end)/VAP(4,end);
    XYZs_1(:,kk)=XYZ_1;
    XYZ_2=[R_final, T_final;0 0 0 1]*XYZ_1;
    XYZs_2(:,kk)=XYZ_2;
end

xx = XYZs_1(1,:);
yy = XYZs_1(2,:);
zz = XYZs_1(3,:);

figure(1),
plot3(xx, yy, zz, 'r+');title('3D plots');

%plot for getting user input
figure,
plot(xx, yy, 'r+');title('select points to mark triangle/quadrilateral');

load triangle_pair_2.txt
triangle_values1 = triangle_pair_2*K1_cam;%shortestDistanceOfUserSelectedPoints(XYZs_1,triangle_pair_2);
xpoints = [triangle_values1(1:4,1);triangle_values1(1,1);triangle_values1(3,1)];
ypoints = [triangle_values1(1:4,2);triangle_values1(1,2);triangle_values1(3,2)];
zpoints = [triangle_values1(1:4,3);triangle_values1(1,3);triangle_values1(3,3)];
triangle = [xpoints,ypoints,zpoints];

figure,
hold on
plot3(triangle(:,1),triangle(:,2),triangle(:,3),'r-');
plot3(triangle_values1(:,1),triangle_values1(:,2),triangle_values1(:,3),'b+');title('Triangle Points');
hold off
view(3)

load quadrilateral_pair_2.txt
xpoints = [quadrilateral_pair_2(1:4,1);quadrilateral_pair_2(1,1)];
ypoints = [quadrilateral_pair_2(1:4,2);quadrilateral_pair_2(1,2)];
zpoints = [quadrilateral_pair_2(1:4,3);quadrilateral_pair_2(1,3)];
quadrilateralPoints = [xpoints, ypoints, zpoints];

figure,
hold on
plot3(quadrilateral_pair_2(:,1),quadrilateral_pair_2(:,2),quadrilateral_pair_2(:,3),'r+');title('quadrilateral Points');
plot3(quadrilateralPoints(:,1),quadrilateralPoints(:,2),quadrilateralPoints(:,3),'b-');
view(3);

toc