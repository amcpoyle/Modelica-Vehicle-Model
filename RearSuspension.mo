model Suspension
    import Modelica.Mechanics.Translational.Interfaces.Flange_a;
    import Modelica.Mechanics.Translational.Components.Spring;
    import Modelica.Mechanics.Translational.Components.Damper;
    import Modelica.Mechanics.Translational.Components.SpringDamper;
    import Modelica.Mechanics.Translational.Components.Mass; 
    import Modelica.Mechanics.MultiBody.Joints.Revolute; 


    Flange_a rocker_fix;
    Spring spring();
    Damper damper();
    Mass unsprung_mass;

    // mirror this part
    Revolute spring_to_rocker;
    Flange_a rocker_to_arm;
    // push rod suspension arm
    // insert what push rod suspension connects to

end RearSuspension;