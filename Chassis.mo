model Chassis
    import Modelica.Units.SI.*;
    import SteeringAngleOutput;
    import AccelerationOutput; // TODO: need to add AccelerationInput to the wheels?
    import VelocityOutput;
    import YawRateOutput;
    import TireForceInput;
    import Modelica.Constants.g_n;

    // car constants
    constant Mass car_mass = ModelConstants.carMass;
    constant Length a1 = ModelConstants.a1;
    constant Length a2 = ModelConstants.a2;
    constant Height cg_height = ModelConstants.cg_height;
    constant Length t = ModelConstants.trackwidth;
    constant Length l = ModelConstants.length;
    constant Radius wheel_radius = ModelConstants.wheel_radius;

    // aero constants
    constant Density rho = ModelConstants.air_density;
    constant Area S = ModelConstants.frontal_area;
    constant Real Cz = ModelConstants.downforce_coef;
    constant Real Cx = ModelConstants.drag_coef;

    Velocity vx(start=0); // longitudinal velocity
    Velocity vy(start=0); // lateral velocity
    Velocity v(start=0);
    Acceleration ax(start=0);
    Acceleration ay(start=0);
    constant Angle delta = ModelConstants.steering_angle_input; // steering angle
    Angle yaw(start=0);
    AngularVelocity yaw_rate(start=0); // der of yaw
    SurfaceDensity Jz(start=0); // moment of inertia units = kg/m2 = surface density for some reason... TODO
    // AngularAcceleration yaw_accel(start=0); TODO: not sure if I need because I don't use it elsewhere...

    // tire forces
    Force Fx11(start=0); // N
    Force Fx12(start=0);
    Force Fx21(start=0);
    Force Fx22(start=0);

    Force Fy11(start=0);
    Force Fy12(start=0);
    Force Fy21(start=0);
    Force Fy22(start=0);

    Force Fz11(start=0); // N
    Force Fz12(start=0);
    Force Fz21(start=0);
    Force Fz22(start=0);

    Force X1(start=0);
    Force X2(start=0);
    Force Y1(start = 0);
    Force Y2(start=0);

    Force X_aero(start=0);

    Force Z10(start=0);
    Force Z20(start=0);
    Force Z1a(start=0);
    Force Z2a(start=0);
    Force delta_Z(start=0);
    Force Z1(start=0);
    Force Z2(start=0);
    Force delta_Z1(start=0);
    Force delta_Z2(start=0);

    TireForceInput tire_forces_11, tire_forces_12, tire_forces_21, tire_forces_22;

    // TODO: I don't think we need to send accel to wheels
    // AccelerationOutput accel_11, accel_12, accel_21, accel_22;
    VelocityOutput velocity_11, velocity_12, velocity_21, velocity_22;
    YawRateOutput yr_11, yr_12, yr_21, yr_22;
    SteeringAngleOutput steer_angle_11, steer_angle_12, steer_angle_21, steer_angle_22;
    
initial equation
    yaw = 2;
    //vx = 0.001; // we are going to get division by 0 error if it starts at 0
    //vy = 0.001;
    vx = 0; // m/s
    vy = 0;

equation
    der(yaw) = yaw_rate;
    //der(vx) = ax;
    //der(vy) = ay;
    vx = 0.5*time;
    vy = 0.5*time;
    v = sqrt(vx^2 + vy^2);
    // v = 0.5*t;
    Jz = 0.5*car_mass*((t^2) + (l^2)); // big estimation/oversimplification - Izz of a rectangle...
    X1 = (Fx11*cos(delta) + Fx12*cos(delta)) - (Fy11*sin(delta) + Fy12*sin(delta));
    X2 = Fx21 + Fx22;
    Y1 = (Fy11*cos(delta) + Fy12*cos(delta)) + (Fx11*sin(delta) + Fx12*sin(delta));
    Y2 = Fy21 + Fy22;
    X_aero = 0.5*rho*S*Cx*(vx^2);

    Z10 = (car_mass*g_n*a2)/l;
    Z20 = (car_mass*g_n*a1)/l;
    Z1a = 0.5*rho*(v^2)*S*Cz;
    Z2a = 0.5*rho*(v^2)*S*Cz; // simplifying so that front downforce coef = rear downforce coef
    delta_Z = -((car_mass*ax*cg_height)/l);
    Z1 = Z10 + Z1a + delta_Z;
    Z2 = Z20 + Z2a - delta_Z;

    delta_Z1 = (Fz12 - Fz11)/2;
    delta_Z2 = (Fz22 - Fz21)/2;

    // in-plane equations of motion
    car_mass*ax = X1 + X2 - X_aero; // solves ax
    car_mass*ay = Y1 + Y2; // solves ay
    Jz*yaw_rate = Y1*a1 - Y2*a2 + (((Fx12 - Fx11)*cos(delta) - (Fy12 - Fy11)*sin(delta))/2)*t + ((Fx22 - Fx21)/2); // might be where overdeterminedness is coming from

    // out of plane equations of motion
    // car_mass*g_n + Z1a + Z2a = Z1 + Z2; // overdeterminedness?
    // -(car_mass*ax*cg_height) + (Z1a*a1 - Z2a*a2) = Z1*a1 - Z2*a2;
    // car_mass*ay*cg_height = delta_Z1*t + delta_Z2*t; // already solved for these variables

    // info transfer - inputs
    tire_forces_11.fx = Fx11;
    tire_forces_11.fy = Fy11;
    tire_forces_11.fz = Fz11;

    tire_forces_12.fx = Fx12;
    tire_forces_12.fy = Fy12;
    tire_forces_12.fz = Fz12;

    tire_forces_21.fx = Fx21;
    tire_forces_21.fy = Fy21;
    tire_forces_21.fz = Fz21;

    tire_forces_22.fx = Fx22;
    tire_forces_22.fy = Fy22;
    tire_forces_22.fz = Fz22;

    // info transfer - outputs
    // accel_11.ax = ax;
    // accel_11.ay = ay;
    // accel_12.ax = ax;
    // accel_12.ay = ay;
    // accel_21.ax = ax;
    // accel_21.ay = ay;
    // accel_22.ax = ax;
    // accel_22.ay = ay;

    velocity_11.vx = vx;
    velocity_11.vy = vy;
    velocity_12.vx = vx;
    velocity_12.vy = vy;
    velocity_21.vx = vx;
    velocity_21.vy = vy;
    velocity_22.vx = vx;
    velocity_22.vy = vy;

    yr_11.yaw_rate = yaw_rate;
    yr_12.yaw_rate = yaw_rate;
    yr_21.yaw_rate = yaw_rate;
    yr_22.yaw_rate = yaw_rate;

    steer_angle_11.delta = delta;
    steer_angle_12.delta = delta;
    steer_angle_21.delta = delta;
    steer_angle_22.delta = delta;

end Chassis;
