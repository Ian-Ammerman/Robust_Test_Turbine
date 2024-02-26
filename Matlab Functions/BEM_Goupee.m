%Blade Element Momentum Theory Code

%Andrew J. Goupee
%University of Maine
%Last modified:  3/18/2017
%
%Description:  This script will estimate the performance coefficient and
%thrust coefficient over a range of tip-speed ratios using the blade
%element momentum theory formulation in the 'AeroDyn Theory Manual'
%authored by P.H. Moriarty and A.C. Hansen.  This simple code requires a
%table of relevant blade information in addition to three airfoil tables.
%A description of the contents of these tables, which are loaded from text
%files, can be found below.

%Clear workspace
clear all; close all; clc;

%Set inputs (These inputs are for the NREL 5 MW reference wind turbine)
R = 1.432068311; %Blade radius (m)
RH = 0.028462998; %Hub radius (m)
Uinf = 5; %Wind speed (m/s)
B = 3; %Number of blades (-)
rho = 1.225; %Density of air (kg/m^3)
RPMList = [65:5:425]; %Rotor speed (rpm)
pitch_offset = [-2:0.5:15];

%Load blade node data
%Columns: 1- radial position (m), 2 - twist (deg), 3 - dr (m)
%4 - chord (m), %5 - %airfoil # (-)
b = load('../Lab Notes & Documentation/stiffer_blade.txt'); %Blade specification is modified NREL 5 MW

%Load airfoil data
%Columns: 1 - alpha in deg, 2 - lift coefficient, 3 - drag coefficient,
%4 - pitching moment coefficient (not used).
a1 = load('../Lab Notes & Documentation/Cylinder_airfoil.txt');
a2 = load('../Lab Notes & Documentation/AG04_MOD_airfoil.txt'); %Airfoil 1 is a cylinder with CD = 0.5

%Separate out blade information
r = b(:,1); %Node radial position
beta = (b(:,2)+0)*pi/180; %Node twist (converted to radians)
dr = b(:,3); %Element width dr
c = b(:,4); %Node chord
at = b(:,5); %Airfoil #

%Determine number of elements
ne = length(r);

%Compute local solidity
sp = (1/(2*pi))*B.*c./r;

%Set BEM solver solution criteria
tol = 0.005;
maxiter = 1000;

%Determine number of rotor speed (TSR) values to compute Cp and Ct values
NTSR = length(RPMList);

%Initialize TSR, Cp and Ct
TSR = zeros(NTSR,length(pitch_offset)); %Tip-speed ratio
Cp = zeros(NTSR,length(pitch_offset)); %Performance coefficient
Ct = zeros(NTSR,length(pitch_offset)); %Thrust coefficient

% Loop through pitch angles
for p = 1:length(pitch_offset)
    %Loop through rotor speed values
    for j = 1:NTSR
        
        %Set RPM
        RPM = RPMList(j);
        
        %Compute local tip speed ratio for all elements
        tsr = (RPM*2*pi/60)*(1/Uinf)*r;
        
        %Initialize total thrust and total torque
        Ttotal = 0; %Thrust (N)
        Qtotal = 0; %Torque (N-m)
        Ftotal = 0; %Rotor plane blade shear force (N)
        
        %Loop through blade elements to compute torque and thrust increment dT
        %and dQ
        for i = ne:-1:1
            
            %Set initial guess for induction factors
            a = 0.25*(2+pi*tsr(i)*sp(i)...
                -sqrt(4-4*pi*tsr(i)*sp(i)+pi*(tsr(i)^2)*sp(i)...
                *(8*beta(i)+pi*sp(i))));  %Axial induction factor
            ap = 0;  %Tangential induction factor
            
            %Set iteration counter
            ct = 0;
            
            %Set initial error
            err = 100;
            
            %Iterate to find final BEM induction factors
            while ((err > tol) && (ct < maxiter));
                
                %Update counter
                ct = ct+1;
                
                %Compute inflow angle (rad)
                phi = atan(Uinf*(1-a)/((RPM*2*pi/60)*r(i)*(1+ap)));
                
                %Estimate angle of attack in degrees
                alpha = (180/pi)*(phi-beta(i)) - pitch_offset(p);
                
                %Obtain lift and drag coefficients
                if (at(i) == 1);
                    CL = interp1(a1(:,1),a1(:,2),alpha);
                    CD = interp1(a1(:,1),a1(:,3),alpha);
                elseif (at(i) == 2);
                    CL = interp1(a2(:,1),a2(:,2),alpha);
                    CD = interp1(a2(:,1),a2(:,3),alpha);
                else
                    CL = interp1(a3(:,1),a3(:,2),alpha);
                    CD = interp1(a3(:,1),a3(:,3),alpha);                
                end
                
                %Estimate the thrust coefficient
                CT = (sp(i)*((1-a)^2)*(CL*cos(phi)+CD*sin(phi)))/(sin(phi)^2);
                
                %Compute tip loss
                Ft = (2/pi)*acos(exp(-(B*(R-r(i))/(2*r(i)*sin(phi)))));
                
                %Compute hub loss
                Fh = (2/pi)*acos(exp(-(B*(r(i)-RH)/(2*RH*sin(phi)))));
                
                %Compute total loss
                F = Ft*Fh;
                
                %Compute updated axial induction factor
                if (CT <= 0.96*F);  %Blade is not highly loaded
                    an = (1+(4*F*(sin(phi)^2))...
                        /(sp(i)*(CL*cos(phi)+CD*sin(phi))))^-1;
                else  %Blade is highly loaded
                    an = (18*F-20-3*sqrt(CT*(50-36*F)+12*F*(3*F-4)))/(36*F-50);
                end
                
                %Compute updated tangential induction factor
                apn = (-1+(4*F*sin(phi)*cos(phi))...
                    /(sp(i)*(CL*sin(phi)-CD*cos(phi))))^-1;
                
                %Compute error
                err = norm([a-an ap-apn]);
                
                %Update induction factors
                a = an;
                ap = apn;
            end;
            
            %Compute Ustar
            Ustar = sqrt((Uinf*(1-a))^2+((RPM*2*pi/60)*r(i)*(1+ap))^2); 
            
            %Update thrust
            dT = B*0.5*rho*(Ustar^2)*...
                (CL*cos(phi)+CD*sin(phi))*c(i)*dr(i); %Thrust increment
            Ttotal = Ttotal+dT; 
            
            %Update torque
            dQ = B*0.5*rho*(Ustar^2)*...
                (CL*sin(phi)-CD*cos(phi))*c(i)*r(i)*dr(i); %Torque increment
            Qtotal = Qtotal+dQ;
            
            %Obtain values need to compute blade edge moment
            dF = dQ/r(i);
            Ftotal = Ftotal+dF;
            
            %Store out-of-rotor-plane moment My
            if (i == ne);
                My(j,i) = (dT/B)*dr(i)/2;
            else;
                My(j,i) = My(j,i+1)+(dT/B)*dr(i)/2+((Ttotal-dT)/B)*dr(i);
            end;
            
            %Store in-rotor-plane moment Mx
            if (i == ne);
                Mx(j,i) = (dF/B)*dr(i)/2;
            else;
                Mx(j,i) = Mx(j,i+1)+(dF/B)*dr(i)/2+((Ftotal-dF)/B)*dr(i);
            end;    
                 
        end;
        
        %Compute total power
        Ptotal = Qtotal*RPM*2*pi/60;
        
        %Compute TSR for rotor speed j
        TSR(j,p) = (RPM*2*pi/60)*R/Uinf;
        
        %Compute Cp for rotor speed j
        Cp(j,p) = Ptotal/(0.5*rho*pi*(R^2)*(Uinf^3));
        
        %Compute Ct for rotor speed j
        Ct(j,p) = Ttotal/(0.5*rho*pi*(R^2)*(Uinf^2));
        
    end
end



% %Plot thrust and performance coefficients
% figure(1)
% clf
% hold on
% box on
% subplot(1,2,1), plot(TSR,Cp,'-o')
% subplot(1,2,1), xlabel('TSR')
% subplot(1,2,1), ylabel('Predicted C_p')
% subplot(1,2,2), plot(TSR,Ct,'-o')
% subplot(1,2,2), xlabel('TSR')
% subplot(1,2,2), ylabel('Predicted C_t')

%% Plot thrust & performance coefficients
% load("Stiffer_Blade_Aero.mat");
% Cp = Aero.Cp;
% Ct = Aero.Ct;
% TSR = repmat(Aero.TSR(:,1),1,size(Cp,2));
% pitch_offset = [0:0.5:10];

figure
gca; hold on; box on;
title('C_p vs TSR')
xlabel('TSR')
ylabel('Predicted C_p')
for ploti = 1:length(pitch_offset)
    plot(TSR(:,ploti),Cp(:,ploti),'DisplayName',sprintf('%0.2g',pitch_offset(ploti)))
end
cp_leg = legend('location','eastoutside');
title(cp_leg,'Blade Pitch [deg]')

figure
gca; hold on; box on;
title('C_t vs TSR')
xlabel('TSR')
ylabel('Predicted C_t')
for ploti = 1:length(pitch_offset)
    plot(TSR(:,ploti),Ct(:,ploti),'DisplayName',sprintf('%0.2g',pitch_offset(ploti)))
end
ct_leg = legend('location','eastoutside');
title(ct_leg,'Blade Pitch [deg]')

Aero.TSR = TSR;
Aero.Cp = Cp;
Aero.Ct = Ct;
save('Stiffer_Blade_Aero.mat','Aero');

% %Plot blade flap moments
% rmoment = r-dr/2; %Locations of moment calculations
% figure(2)
% clf
% surf(rmoment,TSR,My/1000)
% xlabel('Radius (m)')
% ylabel('TSR')
% zlabel('Blade Moment M_y (kN-m)')
% 
% %Plot blade edge moments
% figure(3)
% clf
% surf(rmoment,TSR,Mx/1000)
% xlabel('Radius (m)')
% ylabel('TSR')
% zlabel('Blade Moment M_x (kN-m)')



