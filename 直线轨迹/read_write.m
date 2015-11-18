load Line_20121129_2.dat
a=Line_20121129_2;
q=zeros(14, 2048);
for n=1 :14
    for m=1:2048
        s=(n-1)*2048+m;
        q(n,m)=a(s);
    end
end
N = 1566;
t = 0:0.005:(N - 1)*0.005;
figure (1);
subplot(1,3,1);
plot(t, q(1, 1:N), t, q(8, 1:N));
xlabel ('time / s');
ylabel ('joint / rad');
legend ('q1');
hold on; 
grid on;
subplot(1,3,2);
plot(t, q(2, 1:N), t, q(9, 1:N)); 
xlabel ('time / s');
ylabel ('joint / rad');
legend ('q2');
hold on; 
grid on;
subplot(1,3,3);
plot(t, q(3, 1:N), t, q(10, 1:N));
xlabel ('time / s');
ylabel ('joint / rad');
legend ('q3');
gtext ('Tracking trajectory of shoulder joint angles');
hold on; 
grid on;

figure (2);
plot(t, q(4, 1:N), t, q(11, 1:N));
xlabel ('time / s');
ylabel ('joint / rad');
legend ('q4');
gtext ('Tracking trajectory of elbow joint angles');
hold on;
grid on;

figure (3);
subplot(1,3,1);
plot(t, q(5, 1:N), t, q(12, 1:N)); 
xlabel ('time / s');
ylabel ('joint / rad');
legend ('q5');
hold on; 
grid on;
subplot(1,3,2);
plot(t, q(6, 1:N), t, q(13, 1:N)); 
xlabel ('time / s');
ylabel ('joint / rad');
legend ('q6');
hold on;
grid on;
subplot(1,3,3);
plot(t, q(7, 1:N), t, q(14, 1:N)); 
xlabel ('time / s');
ylabel ('joint / rad');
legend ('q7');
hold on;
grid on;
gtext ('Tracking trajectory of wrist joint angles');

