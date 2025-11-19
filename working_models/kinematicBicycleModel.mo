model KinematicBicycle
    import Modelica.Units.SI.*;
    
    // we are keeping accel constant and that is our input
    parameter Acceleration accel = 15; // input, m/s^2
    constant Real length = 1.5; // m
    constant Real lR = length/2;
    constant Real lF = length/2;
    constant Real R = 20; // m, radius of corner
    Velocity v(start=0);
    Velocity vx(start=0);
    Velocity vy(start=0);
    constant Angle steeringAngle = 0.26; // rad, input, also known as delta
    Angle sideSlipAngle(start=0); // beta
    AngularVelocity omega(start=0);
    Angle yaw(start=0); // input
    
equation
    der(v) = accel;
    omega = v/R;
    sideSlipAngle = atan((lR/(lF + lR))*tan(steeringAngle));
    der(vx) = v*cos(yaw + sideSlipAngle);
    der(vy) = v*sin(yaw + sideSlipAngle);
    der(yaw) = (v/(lF + lR))*cos(sideSlipAngle)*tan(steeringAngle);

end KinematicBicycle;
