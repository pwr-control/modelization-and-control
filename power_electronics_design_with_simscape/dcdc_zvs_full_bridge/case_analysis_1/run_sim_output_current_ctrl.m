clear all
close all
clc

init_model;

scenario = 1;
Iout_dab_sim = Iout_dab_nom;
    number_of_cells_1_sim = floor(Uin_nom/u_cell);
    number_of_cells_2_sim = floor((Uout_nom-delta_load)/u_cell);
    i_out_dab_ref_1 = Iout_dab_sim;
    i_out_dab_pu_ref_1 = i_out_dab_ref_1 / Idc_FS;
sim("dcdc_zvs_full_bridge.slx");
save sim_results_1;

scenario = 2;
Iout_dab_sim = Iout_dab_nom;
    number_of_cells_1_sim = floor(Uin_min/u_cell);
    number_of_cells_2_sim = floor((Uout_min-delta_load)/u_cell);
    i_out_dab_ref_1 = Iout_dab_sim;
    i_out_dab_pu_ref_1 = i_out_dab_ref_1 / Idc_FS;
sim("dcdc_zvs_full_bridge.slx");
save sim_results_2;

scenario = 3;
Iout_dab_sim = Iout_dab_nom;
    number_of_cells_1_sim = floor(Uin_max/u_cell);
    number_of_cells_2_sim = floor((Uout_max-delta_load)/u_cell);
    i_out_dab_ref_1 = Iout_dab_sim * Uout_nom/Uout_max;
    i_out_dab_pu_ref_1 = i_out_dab_ref_1 / Idc_FS;
sim("dcdc_zvs_full_bridge.slx");
save sim_results_3;

scenario = 1;
Iout_dab_sim = Iout_dab_max;
    number_of_cells_1_sim = floor(Uin_nom/u_cell);
    number_of_cells_2_sim = floor((Uout_nom-delta_load)/u_cell);
    i_out_dab_ref_1 = Iout_dab_sim;
    i_out_dab_pu_ref_1 = i_out_dab_ref_1 / Idc_FS;
sim("dcdc_zvs_full_bridge.slx");
save sim_results_4;

scenario = 2;
Iout_dab_sim = Iout_dab_max;
    number_of_cells_1_sim = floor(Uin_min/u_cell);
    number_of_cells_2_sim = floor((Uout_min-delta_load)/u_cell);
    i_out_dab_ref_1 = Iout_dab_sim;
    i_out_dab_pu_ref_1 = i_out_dab_ref_1 / Idc_FS;
sim("dcdc_zvs_full_bridge.slx");
save sim_results_5;

scenario = 3;
Iout_dab_sim = Iout_dab_max;
    number_of_cells_1_sim = floor(Uin_max/u_cell);
    number_of_cells_2_sim = floor((Uout_max-delta_load)/u_cell);
    i_out_dab_ref_1 = Iout_dab_sim * Uout_nom/Uout_max;
    i_out_dab_pu_ref_1 = i_out_dab_ref_1 / Idc_FS;
sim("dcdc_zvs_full_bridge.slx");
save sim_results_6;
clear all
clc