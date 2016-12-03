function selectedPoints = userSelectionfForInputOfCorrespondencePoint(xx,yy,XYZs_1)

figure,
plot(xx, yy, 'r+');title('select points from the plots');

xypoints = ginput(4);

selectedPoints = get3DPointsForSelectedPoints(XYZs_1,xypoints);
end