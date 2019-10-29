function [z_n] = simulate_radar(state, R)
x = state(1);
y = state(3);
z = sqrt(x^2 + y^2);

b = sqrt(R);
a = -sqrt(R);
noise = ((b-a).*rand(1,1) + a);
z_n = z + noise;
end