model Wheel11
    // Tire 11 = Front Left
    import Modelica.Units.SI.*;
    import connectors.SteeringAngleInput;
    import Modelica.Constants.g_n;
    import functions.MagicFormula;
    // model will calculate Fx, Fy, Fz for a tire

    Force Fx11(start=0);
    Force Fy11(start=0);
    Force Fz11(start=0);

    constant Mass car_mass = ModelConstants.carMass;
    constant Mass tire_static_mass = car_mass*ModelConstants.weight_dist_front*ModelConstants.weight_dist_left;
    Angle alpha(start=0); // slip angle for this wheel
    Real k(start=0); // slip ratio

    constant Length a1 = ModelConstants.a1;
    constant Length a2 = ModelConstants.a2;
    constant Height cg_height = ModelConstants.cg_height;
    constant Length t = ModelConstants.trackwidth;
    constant Length car_length = ModelConstants.length;
    constant Radius wheel_radius = ModelConstants.wheel_radius;

    // aero stuff
    constant Density rho = ModelConstants.air_density;
    constant Area S = ModelConstants.frontal_area;
    constant Real Cz = ModelConstants.downforce_coef;

    // magic formula coefs
    constant Real d0_fx = ModelConstants.d0_fx;
    constant Real d1_fx = ModelConstants.d1_fx;
    constant Real c_fx = ModelConstants.c_fx;
    constant Real b0_fx = ModelConstants.b0_fx;
    constant Real b1_fx = ModelConstants.b1_fx;
    constant Real d0_fy = ModelConstants.d0_fy;
    constant Real d1_fy = ModelConstants.d1_fy;
    constant Real c_fy = ModelConstants.c_fy;
    constant Real b0_fy = ModelConstants.b0_fy;
    constant Real b1_fy = ModelConstants.b1_fy;

    Velocity vx(start=0);
    Velocity vy(start=0);
    Velocity v(start=0);
    Acceleration ax(start=0);
    Acceleration ay(start=0);
    Angle delta(start=0); // steering angle
    AngularVelocity yaw_rate(start=0);
    AngularVelocity omega(start=0); // angular velocity of the wheel
    Real LLT(start=0);
    
    // get steering angle, Vy, Vx, yaw rate from chassis using a connector flange
    // INPUTS FROM THE CHASSIS
    SteeringAngleInput steering_angle; // input flange for getting steering angle from the chassis
    VelocityInput velocity;
    YawRateInput yr;

    // OUTPUTS TO THE CHASSIS
    TireForceOutput tire_forces;

equation
    ay = der(vy);
    ax = der(vx);
    v = sqrt(vx^2 + vy^2);
    omega = v/wheel_radius;
    alpha = atan(-vy/vx); // slip angle
    k = -(v*(vx - omega*wheel_radius)/vx); // slip ratio
    LLT = (ay*cg_height)/t;
    Fz11 = 0.5*(((car_mass*g_n*a2)/car_length) + (0.5*rho*S*Cz)*(vx^2) - ((car_mass*ax*cg_height)/car_length)) - ((car_mass*ay*cg_height)/car_length); // last term is simplified LLT, no susp
    Fy11 = MagicFormula(d0=d0_fy, d1=d1_fy, c=c_fy, b0=b0_fy, b1=b1_fy, x=alpha, y=Fz11);
    Fx11 = MagicFormula(d0=d0_fx, d1=d1_fx, c=c_fx, b0=b0_fx, b1=b1_fx, x=k, y=Fz11);

    // getting info from chassis
    steering_angle.delta = delta;
    velocity.vx = vx;
    velocity.vy = vy;
    yr.yaw_rate = yaw_rate;

    // sending info to chassis
    tire_forces.Fx = Fx11;
    tire_forces.Fy = Fy11;
    tire_forces.Fz = Fz11;
end Wheel11;