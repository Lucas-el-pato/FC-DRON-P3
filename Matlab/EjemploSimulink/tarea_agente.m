init_ejemplo;
s = tf('s');
pt1 = @(f) 1/(s/(2*pi*f) + 1);
Fg = pt1(150)*pt1(300);
Fd = pt1(75)*pt1(150);
Dl = pade(exp(-Tm*s), 2);
C  = Kp + Ki/s;
for esc = [1 0.75 0.5 0.4 0.3 0.2]
    G = esc*200*Gm_num*l/J / (s*(s^2 + Gm_den(2)*s + Gm_den(3)));
    L1 = (C + Kd*s*Fd)*G*Fg*Dl;
    m = allmargin(L1);
    [pm, i] = min(m.PhaseMargin);
    fprintf('K=%.3f por %%: estable=%d PM=%.1f deg a %.1f rad/s\n', esc*Gm_num/Gm_den(3), ...
        m.Stable, pm, m.PMFrequency(i));
end
