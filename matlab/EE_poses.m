function [x] = EE_poses(x0)
    qi(1,:) = [53.592,84.806,-12.214,118.513,-98.423,-14.968]; % needs to be in radians
    qi(2,:) = [37.891,74.118,5.620,127.551,-25.084,-25.272];
    qi(3,:) = [27.607,85.683,-7.830,35.993,78.551,114.925];
    qi(4,:) = [33.179,76.313,7.969,-42.291,-56.897,34.168];
    % qi(5,:) = [435.624,399.842,101.409,177.368,-70.399,82.854];
    qi = deg2rad(qi);

    n = height(qi);
    x = zeros(1,n);
    EE_Pos = zeros(n,3);
    for i=1:n
        EE_Pos(i,:) = ToolInBase(x0,qi(i,:));
    end
    EE_Pos
    % centroid = mean(EE_Pos, 1);
    % EE_offset = EE_Pos-centroid;
    % for i = 1:n
    %     x(i) = norm(EE_offset(i,:));
    % end
    % x = mean(x);
    x = norm(std(EE_Pos, 0, 1));
end