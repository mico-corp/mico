clear all;
close all;
clc;

root = "/home/bardo91/programming/mico/build";

ff1 = load(strcat(root, "/bm_flow_singleInt_10.txt"));
ff2 = load(strcat(root, "/bm_flow_singleInt_100.txt"));
ff3 = load(strcat(root, "/bm_flow_singleInt_1000.txt"));
ff4 = load(strcat(root, "/bm_flow_singleInt_10000.txt"));
ff5 = load(strcat(root, "/bm_flow_singleInt_100000.txt"));

fr1 = load(strcat(root, "/bm_ros_singleInt_10.txt"));
fr2 = load(strcat(root, "/bm_ros_singleInt_100.txt"));
fr3 = load(strcat(root, "/bm_ros_singleInt_1000.txt"));
fr4 = load(strcat(root, "/bm_ros_singleInt_10000.txt"));
fr5 = load(strcat(root, "/bm_ros_singleInt_100000.txt"));


figure;
subplot(2,3,1);
hold on;
plot(ff1, 'b');
plot(fr1, 'r');
subplot(2,3,2);
hold on;
plot(ff2, 'b');
plot(fr2, 'r');
subplot(2,3,3);
hold on;
plot(ff3, 'b');
plot(fr3, 'r');
subplot(2,3,4);
hold on;
plot(ff4, 'b');
plot(fr4, 'r');
subplot(2,3,5);
hold on;
plot(ff5, 'b');
plot(fr5, 'r');

x = [10, 100, 1000, 10000, 100000];
yf = [mean(ff1), mean(ff2), mean(ff3), mean(ff4), mean(ff5)];
yr = [mean(fr1), mean(fr2), mean(fr3), mean(fr4), mean(fr5)];

subplot(2,3,6);
hold on;
plot(x,yf, 'b');
plot(x,yr, 'r');
legend('flow', 'ros');
ylim ([0, 100000]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5555%%

ffm1 = load(strcat(root, "/bm_flow_multiInt_5_1000.txt"));
ffm1 = reshape(ffm1,1,[]);
ffm2 = load(strcat(root, "/bm_flow_multiInt_10_1000.txt"));
ffm2 = reshape(ffm2,1,[]);
ffm3 = load(strcat(root, "/bm_flow_multiInt_20_1000.txt"));
ffm3 = reshape(ffm3,1,[]);
ffm4 = load(strcat(root, "/bm_flow_multiInt_50_1000.txt"));
ffm4 = reshape(ffm4,1,[]);


frm1 = load(strcat(root, "/bm_ros_multiInt_5_1000.txt"));
frm1 = reshape(frm1,1,[]);
frm2 = load(strcat(root, "/bm_ros_multiInt_10_1000.txt"));
frm2 = reshape(frm2,1,[]);
frm3 = load(strcat(root, "/bm_ros_multiInt_20_1000.txt"));
frm3 = reshape(frm3,1,[]);
frm4 = load(strcat(root, "/bm_ros_multiInt_50_1000.txt"));
frm4 = reshape(frm4,1,[]);


figure;
subplot(2,3,1);
hold on;
plot(ffm1, 'b');
plot(frm1, 'r');
subplot(2,3,2);
hold on;
plot(ffm2, 'b');
plot(frm2, 'r');
subplot(2,3,3);
hold on;
plot(ffm3, 'b');
plot(frm3, 'r');
subplot(2,3,4);
hold on;
plot(ffm4, 'b');
plot(frm4, 'r');

xm = [5, 10, 20, 50];
yfm = [mean(ffm1), mean(ffm2), mean(ffm3), mean(ffm4)];
yrm = [mean(frm1), mean(frm2), mean(frm3), mean(frm4)];

subplot(2,3,6);
hold on;
plot(xm,yfm, 'b');
plot(xm,yrm, 'r');
legend('flow', 'ros');
