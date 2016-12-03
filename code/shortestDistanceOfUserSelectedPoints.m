%This method is used to calculate shortestdistance between the selected
%points and input pair of points.
function finalizedPoints = shortestDistanceOfUserSelectedPoints(xypoints,selected_points)
selected_points = selected_points(1:4,:);
xypoints = xypoints';
for j=1:size(selected_points,1)
    count = 0;
    mindis = 0;
    for i=1:size(xypoints,1)
        dis = (xypoints(i,1) - selected_points(j,1))^2 + (xypoints(i,2) - selected_points(j,2))^2;
        if mindis == 0
           mindis = dis; 
        end
        if dis <= mindis
            count = count + 1;
            mindis = dis;
        end
    end
    finalizedPoints(j,:) = xypoints(count,:);
end
end