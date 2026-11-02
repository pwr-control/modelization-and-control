function ed = dab3_edges(u, V)
%DAB3_EDGES  Commutations of a six-step bridge, found on its logged phase voltages.
%
%   ed = dab3_edges(u, V)
%
%   u   [V] phase-to-star voltages of the bridge, one column per phase (samples x 3)
%   V   [V] DC voltage of the bridge
%
%   When leg k switches, phase k jumps by 2V/3 and the other two phases by -V/3: an edge is a sample
%   where the largest jump exceeds V/2.
%   ed.n    index of the first sample at the new level
%   ed.leg  leg (1, 2, 3)
%   ed.dir  +1 rising (upper switch turns on), -1 falling

    du = diff(u);
    [m, leg] = max(abs(du), [], 2);
    n = find(m > 0.5*V);
    ed.n = n + 1;
    ed.leg = leg(n);
    ed.dir = sign(du(sub2ind(size(du), n, leg(n))));
end
