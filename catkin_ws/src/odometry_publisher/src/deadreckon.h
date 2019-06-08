#include <eigen3/Eigen/Dense>
#include <iostream>
using Eigen::MatrixXd;
struct odomIntegral{
    double wl,wr,totalTravel,x,y,theta,xdot,ydot,thetadot;
    MatrixXd rthinv,xsi_R_dot;
    odomIntegral(double wheelBase = 0.3113298120556685, double wheelRadius = 0.0508){
        wl = wheelBase/2;
        wr = wheelRadius;
        theta = 0;
        xsi_R_dot = MatrixXd(3,1);
        xsi_R_dot(1) = 0;
        rthinv = MatrixXd(3,3);
        std::cout << rthinv.size() << " " << xsi_R_dot.size() << std::endl;
        rthinv(0,2) = 0;
        rthinv(1,2) = 0;
        rthinv(2,0) = 0;
        rthinv(2,1) = 0;
        rthinv(2,2) = 1;

        totalTravel = 0;
        x = 0;
        y = 0;
    };
    void proc(double l, double r, double tinc){//l and r should be instant wheel velocities in radians per second
        rthinv(0,0) = cos(theta);
        rthinv(0,1) = -sin(theta);
        rthinv(1,0) = sin(theta);
        rthinv(1,1) = cos(theta);
        xsi_R_dot(0,0) = wr*r/2 + wr*l/2; 
        xsi_R_dot(2,0) = wr*r/(2*wl) - wr*l/(2*wl);
        totalTravel += xsi_R_dot(1,0)*tinc;
        auto xsi_I_dot = rthinv * xsi_R_dot;
        xdot = xsi_I_dot(0,0);
        ydot = xsi_I_dot(1,0);
        thetadot = xsi_I_dot(2,0);
        x += xdot*tinc;
        y += ydot*tinc;
        theta += thetadot*tinc;
    }

};
