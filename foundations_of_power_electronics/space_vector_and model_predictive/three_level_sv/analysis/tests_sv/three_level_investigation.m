clear all
close all
clc

idx = 1;
state_table_sa = zeros(1,3);
state_table_sb = zeros(1,3);
state_table_sc = zeros(1,3);
state_table_alpha = zeros(1,3);
state_table_beta = zeros(1,3);
angles  = zeros(1,27);
for sa = -1:1
    for sb = -1:1
        for sc = -1:1
            state_table_sa(:,idx) = sa;            
            state_table_sb(:,idx) = sb;            
            state_table_sc(:,idx) = sc;
            ua = sa;
            ub = sb;
            uc = sc;
            state_table_alpha(:,idx) = 2/3*(ua -1/2*ub -1/2*uc);
            state_table_beta(:,idx) = 2/3*(sqrt(3)/2*(ub - uc));
            idx = idx + 1;
       end
    end
end

for i = 1:27
    a = state_table_alpha(i);
    b = state_table_beta(i);
    if (abs(a) < 1e-3 && abs(b) < 1e-3)
        angles(:,i) = -10;
    else
        angle = atan2(b,a);
            if (angle < 0)
                angle = angle + 2*pi; 
            end
        angles(:,i) = angle;
    end
end

figure; scatter(state_table_alpha,state_table_beta,"filled"); axis("equal"); grid on
figure; scatter(cos(angles),sin(angles),"filled"); axis("equal"); grid on

alpha = 0.13;
beta = 0.23;
theta = atan2(beta,alpha);

idx_ge = -1;
min_ge_diff = 1e6;
for i = 1:27
    if (angles(i) < 0) 
        continue;
    else
        diff = angles(i) - theta;
        if (diff < 0) 
            diff = diff + 2*pi;
        end
        if (diff >= 0 && diff < min_ge_diff)
            min_ge_diff = diff;
            idx_ge = i;
        end
    end
end

idx_prev = -1;
if (idx_ge >= 0) 
    best_diff = 1e9;
    for i = 1:27
        if (angles(i) < 0) 
            continue;
        else
            diff = angles(idx_ge) - angles(i);
            if (diff < 0) 
                diff = diff + 2*pi;
            end
            if (diff > 0 && diff < best_diff)
                best_diff = diff;
                idx_prev = i;
            end
        end
    end
    if (idx_prev == -1)
            best_abs = 1e9;
            for i = 1:27
                if (angles(i) < 0) 
                    continue;
                else
                    d = abs(angles(i) - angles(idx_ge));
                    if (d < best_abs)
                        best_abs = d;
                        idx_prev = i;
                    end
                end
            end
    end
else
    idx_ge = 1;
    idx_prev = 2;
end

idx1 = idx_prev
idx2 = idx_ge

idx_all_neg = -1;
idx_all_pos = -1;
for i = 1:27
    if (state_table_sa(i) == -1 && state_table_sb(i) == -1 && state_table_sc(i) == -1)
        idx_all_neg = i;
    end
    if (state_table_sa(i) == 1 && state_table_sb(i) == 1 && state_table_sc(i) == 1)
        idx_all_pos = i;
    end
end



if (idx_all_neg >= 0)
    idx_zero = idx_all_neg;
else
    idx_zero = idx_all_pos;
end


idx_zero

v1x = state_table_alpha(idx1);
v1y = state_table_beta(idx1);
v2x = state_table_alpha(idx2);
v2y = state_table_beta(idx2);

det = v1x*v2y - v1y * v2x;

if (abs(det) < 1e-3)
    T1 = 0.5;
    T2 = 0.5;
else
    rhs_x = alpha;
    rhs_y = beta;
    T1 = ( rhs_x * v2y - rhs_y * v2x ) / det;
    T2 = (-rhs_x * v1y + rhs_y * v1x ) / det;
end

if (T1 < 0) 
   T1 = 0;
end
if (T2 < 0) 
   T2 = 0;
end
T0 = 1 - (T1 + T2);
if (T0 < 0) 
   T0 = 0;
end

T1
T2
T0

T1+T2+T0

   t1_out = T1;
   t2_out = T2;
   t0_out = T0;

   t0h = T0/2;
   seg_dur = [t0h, T1, T2, t0h];
   idx_seq = [idx_zero, idx1, idx2, idx_zero];
    
   Ta = 0; Tb = 0; Tc = 0;
    
   for s = 1:4
       id = idx_seq(s);
       dur = seg_dur(s);
   
       if (state_table_sa(id) == 1) 
           Ta = Ta + dur;
       end
       if (state_table_sb(id) == 1) 
           Tb = Tb + dur;
       end
       if (state_table_sc(id) == 1) 
           Tc = Tc + dur;
       end
   end
   
   d1 = Ta
   d2 = Tb
   d3 = Tc