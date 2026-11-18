% Simscape(TM) Multibody(TM) version: 25.2

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(27).translation = [0.0 0.0 0.0];
smiData.RigidTransform(27).angle = 0.0;
smiData.RigidTransform(27).axis = [0.0 0.0 0.0];
smiData.RigidTransform(27).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [-14.999999999999902 125.00000000000003 0];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = "B[motor-1:-:motor wheel shaft-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-1.2621774483536189e-29 -2.2737367544323206e-13 1.1813980916589873e-26];  % mm
smiData.RigidTransform(2).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(2).axis = [-0.57735026918962584 0.57735026918962584 0.57735026918962573];
smiData.RigidTransform(2).ID = "F[motor-1:-:motor wheel shaft-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [-14.999999999999902 144.09188309203731 19.091883092037023];  % mm
smiData.RigidTransform(3).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(3).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(3).ID = "B[motor-1:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [19.091883091992884 115.90811690796389 121.99999999999579];  % mm
smiData.RigidTransform(4).angle = 5.1958437552457326e-14;  % rad
smiData.RigidTransform(4).axis = [-2.3200259742606088e-15 -1 6.0272462351849411e-29];
smiData.RigidTransform(4).ID = "F[motor-1:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [-15.000000000000014 123.50000000000009 -6.5000000000000613];  % mm
smiData.RigidTransform(5).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(5).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(5).ID = "B[motor-2:-:motor wheel shaft-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-6.5000000000039044 59.999999999997236 -1.5000000000000071];  % mm
smiData.RigidTransform(6).angle = 6.787813141621145e-15;  % rad
smiData.RigidTransform(6).axis = [0.016356122383769687 -0.065424489535078748 0.99772346540995094];
smiData.RigidTransform(6).ID = "F[motor-2:-:motor wheel shaft-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [-15.000000000000014 124.99999999999962 -26.999999999999357];  % mm
smiData.RigidTransform(7).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(7).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(7).ID = "B[motor-2:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [6.040597923245257e-12 107.99999999999926 -122.00000000000011];  % mm
smiData.RigidTransform(8).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(8).axis = [1 1.5407439555096176e-32 2.7755575615629042e-16];
smiData.RigidTransform(8).ID = "F[motor-2:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [241.31853763547412 -15.749000000000000 28.284271247461689];  % mm  (adjusted -36.749 mm in Y to place caster contact at floor)
smiData.RigidTransform(9).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(9).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(9).ID = "B[Chasis-1:-:caster wheel base-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [28.284271247473683 -9.000000000004917 -28.284271247477982];  % mm
smiData.RigidTransform(10).angle = 1.5707963267948966;  % rad
smiData.RigidTransform(10).axis = [1 -1.6485970816179639e-15 -5.2784487014600583e-15];
smiData.RigidTransform(10).ID = "F[Chasis-1:-:caster wheel base-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [-174.99999999999986 179.00000000000006 0];  % mm
smiData.RigidTransform(11).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(11).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(11).ID = "B[Chasis-1:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [-3.1773469377063564e-09 -9.9999999999993179 -3.1441231840290129e-09];  % mm
smiData.RigidTransform(12).angle = 2.0943951023931997;  % rad
smiData.RigidTransform(12).axis = [-0.57735026918962706 -0.57735026918962307 -0.57735026918962706];
smiData.RigidTransform(12).ID = "F[Chasis-1:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [-25.000000000000021 49.25090600715648 0];  % mm
smiData.RigidTransform(13).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(13).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(13).ID = "B[caster wheel base-1:-:caster wheel2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(14).translation = [1.1664003096711895e-12 -9.9999999999978364 -5.1230131248303223e-12];  % mm
smiData.RigidTransform(14).angle = 2.0943951023932006;  % rad
smiData.RigidTransform(14).axis = [-0.5773502691896274 -0.57735026918962629 -0.57735026918962362];
smiData.RigidTransform(14).ID = "F[caster wheel base-1:-:caster wheel2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(15).translation = [-60.000000000000057 49.25090600715648 0];  % mm
smiData.RigidTransform(15).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(15).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(15).ID = "B[caster wheel base-1:-:caster wheel shaft-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(16).translation = [-3.4674485505092889e-12 -34.999999999998984 6.8922645368729718e-13];  % mm
smiData.RigidTransform(16).angle = 2.0943951023931935;  % rad
smiData.RigidTransform(16).axis = [-0.57735026918962518 -0.57735026918962717 -0.57735026918962506];
smiData.RigidTransform(16).ID = "F[caster wheel base-1:-:caster wheel shaft-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(17).translation = [0 10.000000000000009 0];  % mm
smiData.RigidTransform(17).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(17).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(17).ID = "B[caster wheel shaft-1:-:caster wheel2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(18).translation = [7.4808510948183373e-13 1.1368683772161595e-12 -1.5492334550553881e-12];  % mm
smiData.RigidTransform(18).angle = 2.0943951023931979;  % rad
smiData.RigidTransform(18).axis = [-0.57735026918962651 -0.57735026918962584 -0.57735026918962484];
smiData.RigidTransform(18).ID = "F[caster wheel shaft-1:-:caster wheel2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(19).translation = [0 59.999999999999943 0];  % mm
smiData.RigidTransform(19).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(19).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(19).ID = "B[motor wheel shaft-1:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(20).translation = [-4.7573158036211987e-11 135.00000000000091 181.999999999995];  % mm
smiData.RigidTransform(20).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(20).axis = [1 7.2106817118014697e-31 2.5979218776228663e-14];
smiData.RigidTransform(20).ID = "F[motor wheel shaft-1:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(21).translation = [0 59.999999999999943 0];  % mm
smiData.RigidTransform(21).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(21).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(21).ID = "B[motor wheel shaft-1:-:motor wheel-3]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(22).translation = [-2.2648549702353193e-13 15.000000000000114 8.6508578078792198e-13];  % mm
smiData.RigidTransform(22).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(22).axis = [0.57735026918962562 -0.57735026918962584 0.57735026918962595];
smiData.RigidTransform(22).ID = "F[motor wheel shaft-1:-:motor wheel-3]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(23).translation = [0 60 0];  % mm
smiData.RigidTransform(23).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(23).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(23).ID = "B[motor wheel shaft-2:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(24).translation = [5.0638478288962441e-12 135.00000000000239 -121.9999999999975];  % mm
smiData.RigidTransform(24).angle = 3.1415926535897865;  % rad
smiData.RigidTransform(24).axis = [1 1.1463135028995548e-30 3.3306690738762698e-16];
smiData.RigidTransform(24).ID = "F[motor wheel shaft-2:-:motor mount-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(25).translation = [0 60 0];  % mm
smiData.RigidTransform(25).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(25).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(25).ID = "B[motor wheel shaft-2:-:motor wheel-4]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(26).translation = [3.1974423109204508e-14 -45 5.6843418860808015e-14];  % mm
smiData.RigidTransform(26).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(26).axis = [-0.57735026918962573 0.57735026918962573 0.57735026918962584];
smiData.RigidTransform(26).ID = "F[motor wheel shaft-2:-:motor wheel-4]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(27).translation = [269.58856007729798 337.80935070022656 664.54850073235866];  % mm
smiData.RigidTransform(27).angle = 0;  % rad
smiData.RigidTransform(27).axis = [0 0 0];
smiData.RigidTransform(27).ID = "RootGround[Chasis-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(8).mass = 0.0;
smiData.Solid(8).CoM = [0.0 0.0 0.0];
smiData.Solid(8).MoI = [0.0 0.0 0.0];
smiData.Solid(8).PoI = [0.0 0.0 0.0];
smiData.Solid(8).color = [0.0 0.0 0.0];
smiData.Solid(8).opacity = 0.0;
smiData.Solid(8).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.54220521972684543;  % kg
smiData.Solid(1).CoM = [0.72094973493387404 92.905926614508374 0.0093096947556517602];  % mm
smiData.Solid(1).MoI = [1314.6531747419388 347.88233985651999 1304.9266429396728];  % kg*mm^2
smiData.Solid(1).PoI = [0.39711053317428396 0.013201684903981712 -12.545660243413673];  % kg*mm^2
smiData.Solid(1).color = [0.29411764705882354 0.32156862745098036 1];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "motor*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.083142249577253857;  % kg
smiData.Solid(2).CoM = [0 15.000000000000002 0];  % mm
smiData.Solid(2).MoI = [36.538745288769256 62.659707538846966 36.53874528876927];  % kg*mm^2
smiData.Solid(2).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(2).color = [0 0.50196078431372548 0.50196078431372548];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "caster wheel2*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.0018535396656179789;  % kg
smiData.Solid(3).CoM = [0 25 0];  % mm
smiData.Solid(3).MoI = [0.48004844743791047 0.011859512267301477 0.48004844743791047];  % kg*mm^2
smiData.Solid(3).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(3).color = [1 1 0.75294117647058822];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "caster wheel shaft*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 5.8878405052584455;  % kg
smiData.Solid(4).CoM = [30.086006878778548 145.62793784498209 -2.5478411072665862e-12];  % mm
smiData.Solid(4).MoI = [233120.75655456731 638850.00038306799 502312.87086709688];  % kg*mm^2
smiData.Solid(4).PoI = [0 5.962505678978297e-10 28378.03370131613];  % kg*mm^2
smiData.Solid(4).color = [0.66666666666666663 0.69803921568627447 0.76862745098039209];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Chasis*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.1334995377759802;  % kg
smiData.Solid(5).CoM = [0 6.0543058980772999 0];  % mm
smiData.Solid(5).MoI = [100.80779832138398 118.48469962416134 111.47595748704114];  % kg*mm^2
smiData.Solid(5).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(5).color = [1 1 0.50196078431372548];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "caster wheel base*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 0.0058944040954379264;  % kg
smiData.Solid(6).CoM = [-0.19467616767030388 29.999999999999993 0];  % mm
smiData.Solid(6).MoI = [1.811594327907009 0.093033083243685366 1.8180812125994301];  % kg*mm^2
smiData.Solid(6).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(6).color = [0 0.50196078431372548 0.50196078431372548];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = "motor wheel shaft*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 1.5084367504319454;  % kg
smiData.Solid(7).CoM = [0.00012678690105604804 29.992849441532869 0];  % mm
smiData.Solid(7).MoI = [4851.2759587914898 8825.3150470836426 4851.2748403877285];  % kg*mm^2
smiData.Solid(7).PoI = [0 0 0.0038236324828678837];  % kg*mm^2
smiData.Solid(7).color = [0.90980392156862744 0.44313725490196076 0.031372549019607843];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = "motor wheel*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(8).mass = 1.5673621253065919;  % kg
smiData.Solid(8).CoM = [0 54.531435327824468 0];  % mm
smiData.Solid(8).MoI = [29196.018418586169 35370.42340633709 17580.59134814352];  % kg*mm^2
smiData.Solid(8).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(8).color = [0.50196078431372548 0.50196078431372548 1];
smiData.Solid(8).opacity = 1;
smiData.Solid(8).ID = "motor mount*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the CylindricalJoint structure array by filling in null values.
smiData.CylindricalJoint(3).Rz.Pos = 0.0;
smiData.CylindricalJoint(3).Pz.Pos = 0.0;
smiData.CylindricalJoint(3).ID = "";

smiData.CylindricalJoint(1).Rz.Pos = -161.81993809692236;  % deg
smiData.CylindricalJoint(1).Pz.Pos = 0;  % mm
smiData.CylindricalJoint(1).ID = "[caster wheel base-1:-:caster wheel2-1]";

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.CylindricalJoint(2).Rz.Pos = -179.99999999999986;  % deg
smiData.CylindricalJoint(2).Pz.Pos = 0;  % mm
smiData.CylindricalJoint(2).ID = "[motor wheel shaft-1:-:motor mount-2]";

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.CylindricalJoint(3).Rz.Pos = 3.0730714222293638e-12;  % deg
smiData.CylindricalJoint(3).Pz.Pos = 0;  % mm
smiData.CylindricalJoint(3).ID = "[motor wheel shaft-2:-:motor mount-2]";


%Initialize the PlanarJoint structure array by filling in null values.
smiData.PlanarJoint(1).Rz.Pos = 0.0;
smiData.PlanarJoint(1).Px.Pos = 0.0;
smiData.PlanarJoint(1).Py.Pos = 0.0;
smiData.PlanarJoint(1).ID = "";

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.PlanarJoint(1).Rz.Pos = 108.1800619030776;  % deg
smiData.PlanarJoint(1).Px.Pos = 0;  % mm
smiData.PlanarJoint(1).Py.Pos = 0;  % mm
smiData.PlanarJoint(1).ID = "[caster wheel shaft-1:-:caster wheel2-1]";


%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(3).Rz.Pos = 0.0;
smiData.RevoluteJoint(3).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = -1.3423889655707551e-13;  % deg
smiData.RevoluteJoint(1).ID = "[motor-1:-:motor mount-2]";

smiData.RevoluteJoint(2).Rz.Pos = 3.0539880941405834e-12;  % deg
smiData.RevoluteJoint(2).ID = "[motor-2:-:motor mount-2]";

smiData.RevoluteJoint(3).Rz.Pos = 90;  % deg
smiData.RevoluteJoint(3).ID = "[caster wheel base-1:-:caster wheel shaft-1]";

