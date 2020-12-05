%device = serialport('COM5',115200);
% code not working
  clear all
   clc
   arduino=serial('COM5','BaudRate',115200);
   fopen(arduino); 
     x=linspace(1,100);
     numcols = 1; 
     y = {};
     for i=1:length(x)
      data =fscanf(arduino,'%f'); 
      
     end 

fclose(arduino);
