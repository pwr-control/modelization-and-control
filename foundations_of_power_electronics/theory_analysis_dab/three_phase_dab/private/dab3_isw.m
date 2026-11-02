function sw = dab3_isw(ed, i_in)
%DAB3_ISW  Switching currents of a six-step bridge at its logged commutations.
%
%   sw = dab3_isw(ed, i_in)
%
%   ed    edges from dab3_edges
%   i_in  [A] current into the leg nodes from the transformer, one column per leg (samples x 3)
%
%   The switching current is the current that charges the leg capacitances: into the node at a rising
%   edge, out of it at a falling edge (> 0: ZVS possible, transition time 2 Coss V / i_sw). Value at an
%   edge: mean of the samples before and after it.
%   sw.rise, sw.fall, sw.leg  [A] mean per leg (1 x 3) on the rising edges, the falling edges, all edges
%   sw.all, sw.min            [A] mean and minimum over all the edges

    x = zeros(size(ed.n));
    for q = 1:numel(ed.n)
        c = 0.5*(i_in(ed.n(q) - 1, ed.leg(q)) + i_in(ed.n(q), ed.leg(q)));
        x(q) = c*ed.dir(q);
    end
    for k = 1:3
        sw.rise(k) = mean(x(ed.leg == k & ed.dir > 0));
        sw.fall(k) = mean(x(ed.leg == k & ed.dir < 0));
        sw.leg(k) = mean(x(ed.leg == k));
    end
    sw.all = mean(x);
    sw.min = min(x);
end
