model Car
    // Connecting everything together
    import Modelica.Units.SI.*;
    import Chassis;
    import Wheel11;
    import Wheel12;
    import Wheel21;
    import Wheel22;

    // wheels and chassis import
    Wheel11 FL; // front left
    Wheel12 FR;
    Wheel21 RL; // rear left
    Wheel22 RR;
    Chassis chassis;

equation
    // connect all the flanges together
    // chassis TO wheel
    connect(FL.steering_angle, chassis.steer_angle_11);
    connect(FL.velocity, chassis.velocity_11);
    connect(FL.yr, chassis.yr_11);

    connect(FR.steering_angle, chassis.steer_angle_12);
    connect(FR.velocity, chassis.velocity_12);
    connect(FR.yr, chassis.yr_12);

    connect(RL.steering_angle, chassis.steer_angle_21);
    connect(RL.velocity, chassis.velocity_21);
    connect(RL.yr, chassis.yr_21);

    connect(RR.steering_angle, chassis.steer_angle_22);
    connect(RR.velocity, chassis.velocity_22);
    connect(RR.yr, chassis.yr_22);

    // wheel TO chassis (tire forces)
    connect(chassis.tire_forces_11, FL.tire_forces);
    connect(chassis.tire_forces_12, FR.tire_forces);
    connect(chassis.tire_forces_21, RL.tire_forces);
    connect(chassis.tire_forces_22, RR.tire_forces);

end Car;
