%% ALBERI NELLA TEORIA DEI GRAFI — Bullo, "Lectures on Network Systems" ed. 1.7
%  Riferimenti ai capitoli: Cap. 2 (Grafi e matrici associate),
%                            Cap. 3 (Algoritmi di averaging).
%
%  Autore: Claude
%  -----------------------------------------------------------------------
%  CONTENUTO:
%   1. Definizione di un grafo orientato (digrafo) G = (V, E)
%   2. Matrice di adiacenza  A  e matrice dei gradi  Delta
%   3. Laplaciano  L = Delta - A   (Bullo Def. 2.25)
%   4. Matrice di incidenza orientata  B  (Bullo Def. 2.7)
%      Relazione:  L = B * B'   (grafo non pesato, orientato)
%   5. Spanning tree: verifica algebrica tramite rango di L
%      (Bullo Thm. 2.29: G contiene uno spanning tree  <=>  rank(L) = n-1)
%   6. Costruzione esplicita dello spanning tree BFS
%   7. Visualizzazione: grafo completo vs spanning tree
%   8. Dimostrazione del Consensus su spanning tree  x_dot = -L*x
% -----------------------------------------------------------------------

clear; clc; close all;

%% -----------------------------------------------------------------------
%  1. GRAFO ORIENTATO  G = (V, E)
%     n = 5 nodi,  archi orientati (i -> j)
% -----------------------------------------------------------------------
n = 5;   % |V|

% Lista archi: [sorgente, destinazione]
% Grafo bilanciato con uno spanning tree radice = 1
edges = [1 2;
         1 3;
         2 4;
         3 4;
         3 5;
         4 5;
         5 1];   % arco di chiusura del ciclo

m = size(edges,1);  % |E|

fprintf('Grafo: n = %d nodi,  m = %d archi\n', n, m);

%% -----------------------------------------------------------------------
%  2. MATRICE DI ADIACENZA  A  e matrice dei gradi  Delta
%     A(i,j) = 1  se esiste arco  i -> j
%     Delta = diag(d_out)   dove d_out(i) = out-degree del nodo i
% -----------------------------------------------------------------------
A = zeros(n);
for k = 1:m
    A(edges(k,1), edges(k,2)) = 1;
end

d_out = sum(A, 2);          % out-degree
Delta = diag(d_out);        % matrice dei gradi

fprintf('\nMatrice di adiacenza A:\n'); disp(A)
fprintf('Out-degrees: '); disp(d_out')

%% -----------------------------------------------------------------------
%  3. LAPLACIANO  L = Delta - A   (Bullo Def. 2.25)
%     Proprietà: L*1_n = 0  (sempre),  1_n'*L = 0  se bilanciato
% -----------------------------------------------------------------------
L = Delta - A;

fprintf('Laplaciano L = Delta - A:\n'); disp(L)

% Verifica L*1 = 0
residual_right = norm(L * ones(n,1));
fprintf('Verifica  L*1 = 0:   ||L*1|| = %.2e\n', residual_right)

% Verifica bilanciamento (1'*L = 0)
residual_left = norm(ones(1,n) * L);
fprintf('Verifica bilanciamento  1''*L = 0:   ||1''*L|| = %.2e\n', residual_left)

%% -----------------------------------------------------------------------
%  4. MATRICE DI INCIDENZA ORIENTATA  B  (Bullo Def. 2.7)
%     B è  n x m,   B(i,k) = +1 se l'arco k esce da i
%                             -1 se l'arco k entra in i
%                              0 altrimenti
%     Relazione:   L = B * B'
% -----------------------------------------------------------------------
B = zeros(n, m);
for k = 1:m
    src = edges(k,1);
    dst = edges(k,2);
    B(src, k) =  1;
    B(dst, k) = -1;
end

L_check = B * B';
fprintf('\nVerifica  L = B*B'':   max|L - B*B''| = %.2e\n', max(abs(L(:) - L_check(:))))

%% -----------------------------------------------------------------------
%  5. SPANNING TREE: verifica algebrica
%     Bullo Thm. 2.29:
%       G contiene uno spanning tree (in-branching da una radice)
%       <==>  rank(L) = n-1
%       <==>  L ha esattamente un autovalore nullo
% -----------------------------------------------------------------------
eigvals = sort(real(eig(L)));

fprintf('\nAutovalori del Laplaciano L (ordinati):\n')
fprintf('  lambda_%d = %.4f\n', [(1:n); eigvals'])

rk = rank(L);
fprintf('\nRango di L: %d  (su n=%d nodi)\n', rk, n)

if rk == n-1
    fprintf('--> G contiene uno spanning tree  (Bullo Thm. 2.29)\n')
else
    fprintf('--> G NON contiene spanning tree\n')
end

%% -----------------------------------------------------------------------
%  6. COSTRUZIONE DELLO SPANNING TREE (BFS da nodo radice = 1)
%     Identificazione degli archi dell'albero
% -----------------------------------------------------------------------
root = 1;
visited = false(1, n);
visited(root) = true;
queue = root;
tree_edges = [];     % [src, dst]

while ~isempty(queue)
    u = queue(1);  queue(1) = [];
    neighbors = find(A(u,:));   % out-neighbors di u
    for v = neighbors
        if ~visited(v)
            visited(v) = true;
            tree_edges = [tree_edges; u v]; %#ok<AGROW>
            queue = [queue v]; %#ok<AGROW>
        end
    end
end

fprintf('\nSpanning tree BFS (radice = nodo %d):\n', root)
fprintf('  Arco: %d -> %d\n', tree_edges')

%% -----------------------------------------------------------------------
%  7. VISUALIZZAZIONE
% -----------------------------------------------------------------------
figure('Name','Bullo – Grafo e Spanning Tree','NumberTitle','off',...
       'Position',[100 100 1100 480])

% Layout circolare fisso
theta = linspace(0, 2*pi*(1-1/n), n);
pos = [cos(theta); sin(theta)]';   % n x 2

% --- Subplot 1: grafo completo ---
ax1 = subplot(1,2,1);
title('Grafo orientato G = (V, E)', 'FontWeight','bold')
hold on; axis equal; axis off
xlim([-1.5 1.5]); ylim([-1.5 1.5])

% Archi
for k = 1:m
    s = edges(k,1); d = edges(k,2);
    draw_arrow(pos(s,:), pos(d,:), [0.4 0.6 1.0], 2)
end
% Nodi
for i = 1:n
    plot(pos(i,1), pos(i,2), 'o', 'MarkerSize',22, ...
        'MarkerFaceColor',[0.2 0.4 0.8], 'MarkerEdgeColor','k', 'LineWidth',1.5)
    text(pos(i,1), pos(i,2), num2str(i), 'HorizontalAlignment','center',...
        'Color','w', 'FontWeight','bold', 'FontSize',11)
end

% --- Subplot 2: spanning tree ---
ax2 = subplot(1,2,2);
title(sprintf('Spanning tree BFS (radice = %d)', root), 'FontWeight','bold')
hold on; axis equal; axis off
xlim([-1.5 1.5]); ylim([-1.5 1.5])

% Archi NON-tree in grigio chiaro
for k = 1:m
    s = edges(k,1); d = edges(k,2);
    in_tree = any(tree_edges(:,1)==s & tree_edges(:,2)==d);
    if ~in_tree
        draw_arrow(pos(s,:), pos(d,:), [0.8 0.8 0.8], 1)
    end
end
% Archi dell'albero in rosso
for k = 1:size(tree_edges,1)
    draw_arrow(pos(tree_edges(k,1),:), pos(tree_edges(k,2),:), [0.9 0.2 0.2], 2.5)
end
% Nodi
for i = 1:n
    col = [0.2 0.4 0.8];
    if i == root; col = [0.1 0.6 0.2]; end
    plot(pos(i,1), pos(i,2), 'o', 'MarkerSize',22, ...
        'MarkerFaceColor', col, 'MarkerEdgeColor','k','LineWidth',1.5)
    text(pos(i,1), pos(i,2), num2str(i), 'HorizontalAlignment','center',...
        'Color','w', 'FontWeight','bold', 'FontSize',11)
end
text(0, -1.4, 'Archi albero: rosso  |  Archi rimanenti: grigio', ...
    'HorizontalAlignment','center','FontSize',9,'Color',[0.4 0.4 0.4])

sgtitle('Teoria dei Grafi — Bullo, Lectures on Network Systems', ...
    'FontSize',13,'FontWeight','bold')

%% -----------------------------------------------------------------------
%  8. CONSENSUS SU SPANNING TREE   dx/dt = -L_tree * x
%     Si costruisce il Laplaciano del solo spanning tree e si simula
%     la convergenza al valor medio (per grafo bilanciato su albero
%     questo richiede il grafo in entrambe le direzioni; qui usiamo
%     L simmetrico dell'albero non orientato per mostrare il consensus)
% -----------------------------------------------------------------------
% Laplaciano simmetrico (non orientato) dello spanning tree
A_tree = zeros(n);
for k = 1:size(tree_edges,1)
    s = tree_edges(k,1); d = tree_edges(k,2);
    A_tree(s,d) = 1;
    A_tree(d,s) = 1;   % non orientato per garantire consensus
end
D_tree = diag(sum(A_tree,2));
L_tree = D_tree - A_tree;

% Stato iniziale casuale
rng(42)
x0 = 10 * randn(n,1);
x_mean = mean(x0);

% Integrazione  dx/dt = -L_tree * x   con ode45
t_span = [0 5];
[t, X] = ode45(@(t,x) -L_tree*x, t_span, x0);

figure('Name','Consensus su spanning tree','NumberTitle','off',...
       'Position',[100 600 700 380])
hold on; grid on
colors = lines(n);
for i = 1:n
    plot(t, X(:,i), 'LineWidth', 1.8, 'Color', colors(i,:), ...
        'DisplayName', sprintf('x_%d(t)', i))
end
yline(x_mean, 'k--', 'LineWidth', 2, 'DisplayName', sprintf('Media = %.2f', x_mean))
xlabel('t  [s]'); ylabel('x_i(t)')
title('Consensus lineare:  \dot{x} = -L_{tree} x', 'FontWeight','bold')
legend('Location','eastoutside','FontSize',9)
xlim([0 5])

fprintf('\nConsensus: stato finale (t=5 s):\n')
fprintf('  x_%d = %.4f\n', [(1:n); X(end,:)])
fprintf('  valor medio teorico = %.4f\n', x_mean)

%% -----------------------------------------------------------------------
%  FUNZIONE LOCALE: disegna freccia tra due punti 2D
% -----------------------------------------------------------------------
function draw_arrow(p1, p2, col, lw)
    offset = 0.13;   % raggio del nodo
    d = p2 - p1;
    dn = d / norm(d);
    p1s = p1 + dn * offset;
    p2s = p2 - dn * offset;
    annotation_arrow = false;
    if annotation_arrow
        % (non usato)
    end
    quiver(p1s(1), p1s(2), p2s(1)-p1s(1), p2s(2)-p1s(2), 0, ...
        'Color', col, 'LineWidth', lw, ...
        'MaxHeadSize', 0.6, 'AutoScale', 'off')
end
