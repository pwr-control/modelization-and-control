clear all
close all
clc

init_model;

scenario = 1;
    number_of_cells_1_sim = floor(750/3.6);
    number_of_cells_2_sim = floor(560/3.6);
    i_in_dab_pu_ref = 0.65;
    i_in_dab_ref = i_in_dab_pu_ref * Idc_FS;
sim("dcdc_zvs_full_bridge.slx");
save sim_results_1;

scenario = 2;
    number_of_cells_1_sim = floor(500/3.6);
    number_of_cells_2_sim = floor(860/3.6);
    i_in_dab_pu_ref = 0.65;
    i_in_dab_ref = i_in_dab_pu_ref * Idc_FS;
sim("dcdc_zvs_full_bridge.slx");
save sim_results_2;

scenario = 3;
    number_of_cells_1_sim = floor(900/3.6);
    number_of_cells_2_sim = floor(360/3.6);
    i_in_dab_pu_ref = 0.54;
    i_in_dab_ref = i_in_dab_pu_ref * Idc_FS;
sim("dcdc_zvs_full_bridge.slx");
save sim_results_3;

clear all