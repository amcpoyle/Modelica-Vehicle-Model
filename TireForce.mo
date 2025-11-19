function TireForce "Implement Pacejka magic formula tire model with 4 terms"
    input Real slip "slip angle (if lateral force) OR slip ratio (if longitudinal force)";
    input Real Fz "vertical force";
    input Real d1;
    input Real d2;
    input Real b;
    input Real c;
    input Real sv;
    input Real sh;
algorithm
    f := (d1 + d2/1000*Fz)*Fz*sin(c*atan(b*(slip-sh)))+sv;
end TireForce;