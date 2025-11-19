<<<<<<< HEAD
model DynamicBicycle
    import Modelica.Units.SI.*;
    // input variables

    constant Real slope_Fy = 1;
    constant Angle alpha_F = 0.26; // rad, front slip angles
    constant Angle alpha_R = 0.26; // rad, rear slip angles
    constant Real C_af = slope_Fy/alpha_F;
    constant Real C_ar = slope_Fy/alpha_R;
    constant Mass carMass = 190; // kg
    constant Real length = 1.5; // m
    constant Real lR = length/2;
    constant Real lF = length/2;
    constant Inertia Iz = 0.1; // kg/m2, not accurate, just to make the model work
    constant Angle steeringAngle = 0.15; // rad

    Velocity vlon(start=1);
    Velocity vlat(start=1);
    AngularVelocity omega(start=0);
    AnglularVelocity yawRate(start=0);
    
equation
  omega = 0.5*time;
  der(vlat) = -(((C_ar + C_af)/(carMass*vlon))*(vlat)) + (((C_ar*lR - C_af*lF)/(carMass*vlon)) - vlon)*omega + (C_af/carMass)*steeringAngle;
  der(yawRate) = omega;
  der(omega) = (((lR*C_ar) - (lF*C_af))/(Iz*vlon))*vlat - ((((lF^2)*C_af) + ((lR^2)*C_ar))/(Iz*vlon))*omega + (C_af/Iz)*(lF)*steeringAngle;

 


end DynamicBicycle;
=======
model DynamicBicycle
    import Modelica.Units.SI.*;
    // input variables

    constant Real slope_Fy = 1;
    constant Angle alpha_F = 0.26; // rad, front slip angles
    constant Angle alpha_R = 0.26; // rad, rear slip angles
    constant Real C_af = slope_Fy/alpha_F;
    constant Real C_ar = slope_Fy/alpha_R;
    constant Mass carMass = 190; // kg
    constant Real length = 1.5; // m
    constant Real lR = length/2;
    constant Real lF = length/2;
    constant Inertia Iz = 0.1; // kg/m2, not accurate, just to make the model work
    constant Angle steeringAngle = 0.15; // rad

    Velocity vlon(start=0);
    Velocity vlat(start=0);
    AngularVelocity omega(start=0);
    AngularVelocity yawRate(start=0);
    
equation
  der(vlat) = -(((C_ar + C_af)/(carMass*vlon))*(vlat)) + (((C_ar*lR - C_af*lF)/(carMass*vlon)) - vlon)*omega + (C_af/carMass)*steeringAngle;
  der(yawRate) = omega;
  der(omega) = (((lR*C_ar) - (lF*C_af))/(Iz*vlon))*vlat - ((((lF^2)*C_af) + ((lR^2)*C_ar))/(Iz*vlon))*omega + (C_af/Iz)*(lF)*steeringAngle;

 


end DynamicBicycle;
>>>>>>> 02e9ae8969767b38ff80b27a3ea7ae8cf5398ec9
