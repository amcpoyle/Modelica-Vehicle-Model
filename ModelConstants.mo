package ModelConstants
    import Modelica.Units.SI.*;

    // Constants for the car
    constant Length length = 1.5; // m
    constant Length a1 = length/2; // m, lon. distance from G to front axle
    constant Length a2 = length/2; // m, lon. distance from G to rear axle
    constant Length trackwidth = 1.2; // m, front
    constant Mass carMass = 190; // kg
    constant Height cg_height = 0.254; // m, cg height
    constant Real weight_dist_front = 0.525;
    constant Real weight_dist_rear = 1 - weight_dist_front;
    constant Real weight_dist_right = 0.5;
    constant Real weight_dist_left = 1 - weight_dist_right;
    constant Radius wheel_radius = 0.2032; // need to change to a more accurate number

    // Aero constants
    constant Density air_density = 1.225; // kg/m^3, density of air
    constant Area frontal_area = 1; // m^2, frontal area of car, guess
    constant Real downforce_coef = 3.03; // downforce coef (Cz), an avg value I found on reddit, unscientific
    constant Real drag_coef = 0.9; // drag coef (Cx), guess

    // magic formula coefficients
    // Fx is a function of slip ratio and Fz
    // Fy is a function of slip angle and Fz
    constant Real d0_fx = -2.76779886;
    constant Real d1_fx = -0.66793554;
    constant Real c_fx = -1.35183433;
    constant Real b0_fx = -14.12668056;
    constant Real b1_fx = -1.49722406;

    constant Real d0_fy = 2.66356359;
    constant Real d1_fy = 0.22192692;
    constant Real c_fy = 1.55870603;
    constant Real b0_fy = 14.0650621;
    constant Real b1_fy = 4.2838454;
    
    constant Angle steering_angle_input = 0.2; // rad

end ModelConstants;
