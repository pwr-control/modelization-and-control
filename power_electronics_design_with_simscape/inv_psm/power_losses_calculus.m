clc



ploss_igbtQ1 = mean(igbt_ploss_invQ1_sim)
ploss_diodeQ1 = mean(diode_ploss_invQ1_sim)
deltaJH_igbtQ1 = mean(igbt_JH_temp_invQ1_sim)
deltaJH_diodeQ1 = mean(diode_JH_temp_invQ1_sim)

rippleJH_igbtQ1 = max(igbt_JH_temp_invQ1_sim) - mean(igbt_JH_temp_invQ1_sim)
rippleJH_diodeQ1 = max(diode_JH_temp_invQ1_sim) - mean(diode_JH_temp_invQ1_sim)

TjMAX_igbt = 65 + rippleJH_igbtQ1 + deltaJH_igbtQ1
TjMAX_diode = 65 + rippleJH_diodeQ1 + deltaJH_diodeQ1

Tj_igbt_design = 65 + deltaJH_igbtQ1
