function imprimeEvento(nome, t_val, x_val, mach_val, q_val)
    alt_km = x_val(2) / 1000;
    vel_ms = x_val(3);
    path_deg = x_val(4) * (180/pi);
    massa = x_val(5);
    q_kpa = q_val / 1000;

    fprintf('%-20s | %8.2f | %10.3f | %10.2f | %6.2f | %8.2f | %12.2f | %10.1f\n', ...
        nome, t_val, alt_km, vel_ms, mach_val, q_kpa, path_deg, massa);
end