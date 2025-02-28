function [P] = ToolInBase(x0,q)
    t1 = [0, 0, .45];
    t2 = [.025, 0, 0];
    t3 = [0, -.001, .454];
    t4 = [0, 0, .035];
    t5 = [.4195, .001, 0];
    t6 = [.1175, 0, 0];
    t7 = [x0(1),x0(2),x0(3)];
    
    eul1 = [0, 0, q(1)];
    eul2 = [0, q(2), 0];
    eul3 = [0, q(3), 0];
    eul4 = [q(4), 0, 0];
    eul5 = [0, q(5), 0];
    eul6 = [q(6), 0, 0];
    eul7 = [0, 0, 0];
    
    H01 = se3(eul1,"eul","XYZ",t1);
    H12 = se3(eul2,"eul","XYZ",t2);
    H23 = se3(eul3,"eul","XYZ",t3);
    H34 = se3(eul4,"eul","XYZ",t4);
    H45 = se3(eul5,"eul","XYZ",t5);
    H56 = se3(eul6,"eul","XYZ",t6);
    H67 = se3(eul7,"eul","XYZ",t7);
    
    H07 = H01*H12*H23*H34*H45*H56*H67;

    P = trvec(H07);
end