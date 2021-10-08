open_system('picontroller');

out = sim('picontroller');

figure(1)
plot(out.Position)
axis([1,1.45,0,1.2])
figure(2)
plot(out.Position2)
axis([1,1.4,0,0.8])