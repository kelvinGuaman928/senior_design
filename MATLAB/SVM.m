clear all
load Exp1
load Exp2
load Exp3
% Exp1 (:,2:3)input  are the features, Exp1(:,4) output  is the label
% outlier fraction 0.05. ##presentation : dec 16 ##
% train 
Mdl = fitcsvm(Exp1(:,2:3),Exp1(:,4),'Standardize',true,'KernelFunction','RBF','KernelScale','auto','OutlierFraction',0.05);

[Test, score] = predict(Mdl, Exp2(:,1:2)); % live data from serialport   

PerformanceExp2=100-norm([Test-Exp2(:,3)]).^2/length(Exp2(:,3))*100 % Performance of SVM on Exp2 .5s 

[Test, score] = predict(Mdl, Exp3(:,1:2));
PerformanceExp3=100-norm([Test-Exp3(:,3)]).^2/length(Exp3(:,3))*100 % Performance of SVM on Exp3
