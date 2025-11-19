function MagicFormula
    input Real d0;
    input Real d1;
    input Real c;
    input Real b0;
    input Real b1;
    input Real x; // slip ratio (Fx) or slip angle (Fy)
    input Real y; // fz
    output Real target_value; // Fx or Fy
algorithm
    target_value := (d0+(d1*y)/1000)*y*sin(c*atan(((b0+(b1*y))/1000)*x))
end MagicFormula