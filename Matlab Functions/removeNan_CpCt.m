% Example x and y vectors with NaN values
x = Aero.TSR(:,1);
cols_to_fix = [8,9]

for i = cols_to_fix

    y = Aero.Cp(:,i);

    % Find indices of non-NaN values
    non_nan_indices = ~isnan(y);
    
    % Interpolate missing values using PCHIP
    filled_y = interp1(x(non_nan_indices), y(non_nan_indices), x, 'pchip');

    Aero.Cp(:,i) = filled_y;
end

for i = cols_to_fix

    y = Aero.Ct(:,i);

    % Find indices of non-NaN values
    non_nan_indices = ~isnan(y);
    
    % Interpolate missing values using PCHIP
    filled_y = interp1(x(non_nan_indices), y(non_nan_indices), x, 'pchip');

    Aero.Ct(:,i) = filled_y;
end

save('Stiffer_Blade_Aero.mat','Aero')