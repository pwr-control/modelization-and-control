function im = im_calculus()
    
    pwr_nom = 250e3;
    u_nom = 400;
    i_nom = 432;
    freq_nom = 50;
    rpm_load = 992;
    number_poles = 6;
    eta = 0.95;
    cosphi = 0.88;
    Rs = 2e-3;
    u_no_load = u_nom;
    i_no_load = 135;
    f_no_load = freq_nom;
    u_cc = u_nom;
    i_cc = 3084;
    f_cc = freq_nom;
    Jm = 5;
    load_friction_factor = 0.75;
    im = im_setup('GM355L6-250kW', pwr_nom, u_nom, i_nom, freq_nom, rpm_load, number_poles, eta, cosphi, ...
                            Rs, u_no_load, i_no_load, f_no_load, u_cc, i_cc, f_cc, Jm, load_friction_factor);
    displayInfo(im);
end