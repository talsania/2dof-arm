function plot_workspace_2DOF(L1, L2)
% PLOT_WORKSPACE_2DOF  Sweep all joint angles and plot reachable workspace.

    t1r = linspace(-pi, pi, 200);
    t2r = linspace(-pi, pi, 200);
    [T1, T2] = meshgrid(t1r, t2r);
    X = L1*cos(T1) + L2*cos(T1+T2);
    Y = L1*sin(T1) + L2*sin(T1+T2);

    plot(X(:), Y(:), '.', 'Color', [0.7 0.85 1], 'MarkerSize', 1); hold on;

    th = linspace(0, 2*pi, 400);
    plot((L1+L2)*cos(th), (L1+L2)*sin(th), 'b-', 'LineWidth', 2);
    r_min = abs(L1-L2);
    if r_min > 0.001
        plot(r_min*cos(th), r_min*sin(th), 'r-', 'LineWidth', 2);
    end
    plot(0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('Reachable Workspace  L1=%.2f m  L2=%.2f m', L1, L2));
    legend('Reachable','Outer boundary','Inner boundary','Base','Location','best');
    hold off;
end
