function [W,t] = rigid_link(link_geometry)

d_skew=[0,-link_geometry(3),link_geometry(2);
        link_geometry(3),0,-link_geometry(1);
       -link_geometry(2),link_geometry(1),0];
D=[eye(3) , d_skew;
    zeros(3), eye(3)];
W=[zeros(6), zeros(6);
    eye(6) , D'];
t=[D        ,  -eye(6);
   zeros(6) , zeros(6)];

