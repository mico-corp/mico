ff1 = load("/home/bardo91/Desktop/bm_flow_singleInt_10.txt");
ff2 = load("/home/bardo91/Desktop/bm_flow_singleInt_100.txt");
ff3 = load("/home/bardo91/Desktop/bm_flow_singleInt_1000.txt");
ff4 = load("/home/bardo91/Desktop/bm_flow_singleInt_10000.txt");
ff5 = load("/home/bardo91/Desktop/bm_flow_singleInt_100000.txt");

fr1 = load("/home/bardo91/Desktop/bm_ros_singleInt_10.txt");
fr2 = load("/home/bardo91/Desktop/bm_ros_singleInt_100.txt");
fr3 = load("/home/bardo91/Desktop/bm_ros_singleInt_1000.txt");
fr4 = load("/home/bardo91/Desktop/bm_ros_singleInt_10000.txt");
fr5 = load("/home/bardo91/Desktop/bm_ros_singleInt_100000.txt");


figure;
subplot(2,3,1);
hold on;
plot(ff1, 'b');
plot(fr1, 'r');
ylim ([0, 100000]);
subplot(2,3,2);
hold on;
plot(ff2, 'b');
plot(fr2, 'r');
ylim ([0, 100000]);
subplot(2,3,3);
hold on;
plot(ff3, 'b');
plot(fr3, 'r');
ylim ([0, 100000]);
subplot(2,3,4);
hold on;
plot(ff4, 'b');
plot(fr4, 'r');
ylim ([0, 100000]);
subplot(2,3,5);
hold on;
plot(ff5, 'b');
plot(fr5, 'r');
ylim ([0, 100000]);

x = [10, 100, 1000, 10000, 100000];
yf = [mean(ff1), mean(ff2), mean(ff3), mean(ff4), mean(ff5)];
yr = [mean(fr1), mean(fr2), mean(fr3), mean(fr4), mean(fr5)];

subplot(2,3,6);
hold on;
plot(x,yf, 'b');
plot(x,yr, 'r');
legend('flow', 'ros');
ylim ([0, 100000]);