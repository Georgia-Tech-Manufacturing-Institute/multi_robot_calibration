function [robot] = buildRobot(init)
    [l1z,l2x,l3y,l3z,l4z,l5x,l5y,l6x] = deal([init(1)],[init(2)],[init(3)],[init(4)],[init(5)],[init(6)],[init(7)],[init(8)]);
    robot = rigidBodyTree;
    
    link_1 = rigidBody('link_1');
    jnt1 = rigidBodyJoint('jnt1','revolute');
    jnt1.HomePosition = 0;
    jnt1.JointAxis = [0, 0, 1];
    tform1 = trvec2tform([0, 0, l1z]); % User defined
    setFixedTransform(jnt1,tform1);
    link_1.Joint = jnt1;
    addBody(robot,link_1,'base') % Add link_1 to base
    
    link_2 = rigidBody('link_2');
    jnt2 = rigidBodyJoint('jnt2','revolute');
    jnt2.HomePosition = 0; % User defined
    jnt2.JointAxis = [0, 1, 0];
    tform2 = trvec2tform([l2x, 0, 0]); % User defined
    setFixedTransform(jnt2,tform2);
    link_2.Joint = jnt2;
    addBody(robot,link_2,'link_1'); % Add link_2 to link_1
    
    link_3 = rigidBody('link_3');
    jnt3 = rigidBodyJoint('jnt3','revolute');
    jnt3.HomePosition = 0; % User defined
    jnt3.JointAxis = [0, 1, 0];
    tform3 = trvec2tform([0, l3y, l3z]); % User defined
    setFixedTransform(jnt3,tform3);
    link_3.Joint = jnt3;
    addBody(robot,link_3,'link_2'); % Add link_3 to link_2
    
    link_4 = rigidBody('link_4');
    jnt4 = rigidBodyJoint('jnt4','revolute');
    jnt4.HomePosition = 0; % User defined
    jnt4.JointAxis = [1, 0, 0];
    tform4 = trvec2tform([0, 0, l4z]); % User defined
    setFixedTransform(jnt4,tform4);
    link_4.Joint = jnt4;
    addBody(robot,link_4,'link_3'); % Add link_4 to link_3
    
    link_5 = rigidBody('link_5');
    jnt5 = rigidBodyJoint('jnt5','revolute');
    jnt5.HomePosition = 0; % User defined
    jnt5.JointAxis = [0, 1, 0];
    tform5 = trvec2tform([l5x, l5y, 0]); % User defined
    setFixedTransform(jnt5,tform5);
    link_5.Joint = jnt5;
    addBody(robot,link_5,'link_4'); % Add link_5 to link_4
    
    link_6 = rigidBody('link_6');
    jnt6 = rigidBodyJoint('jnt6','revolute');
    jnt6.HomePosition = 0; % User defined
    jnt6.JointAxis = [1, 0, 0];
    tform6 = trvec2tform([l6x, 0, 0]); % User defined
    setFixedTransform(jnt6,tform6);
    link_6.Joint = jnt6;
    addBody(robot,link_6,'link_5'); % Add link_6 to link_5
end