clear all
close all
clc

ADC_RESOLUTION = 4096;
i_inv_out_max = 150;
i_inv_out_nom = 89;
u_zvs_out_max = 1150;
u_zvs_out_nom = 780;

ADC_INPUT = (ADC_RESOLUTION - ADC_RESOLUTION/2) * 0.1*i_inv_out_nom / i_inv_out_max 
i_inv_out_scale  =  ((i_inv_out_max / i_inv_out_nom) * 2.0) / ADC_RESOLUTION;
i_inv_out_offset = -i_inv_out_scale * ADC_RESOLUTION / 2.0;
i_inv_out = (ADC_INPUT * i_inv_out_scale)

ADC_INPUT = ADC_RESOLUTION * 0.1 * u_zvs_out_nom / u_zvs_out_max 
u_zvs_out_scale = ((u_zvs_out_max / u_zvs_out_nom) * 1.0) / ADC_RESOLUTION;
u_zvs_out = ADC_INPUT * param.u_zvs_out_scale
