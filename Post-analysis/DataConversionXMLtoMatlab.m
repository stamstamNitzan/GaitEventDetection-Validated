%{
Copyright 2022 Nitzan Breitman 
https://github.com/stamstamNitzan/GaitEventDetection-Validated

Licensed under the Educational Community License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License. 
You may obtain a copy of the License at https://opensource.org/licenses/ECL-2.0
Unless required by applicable law or agreed to in writing, software distributed under
 the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 either express or implied.
See the License for the specific language governing permissions and limitations under the License.
%}

close all
clear all


XML = readstruct("test.xml");

for i=1:size(XML.wait_for_notification,2)
    Value(i,1) =  XML.wait_for_notification(i).assert_value.valueAttribute;
end

g = textscan(Value(1),'%8s');
FirstMilis = hex2dec(g{1,1}{2});
j=1;
for i = 1:size(Value,1)
    if ismissing(Value(i)) || isequal(Value(i), 'Inf') || sum(str2num(Value(i))) >= 1e+13 %|| isequal(Value(i), '0') %The equal to 0 might be bad
        h=1;
    else
        g = textscan(Value(i),'%8s');
        Time(j) = (hex2dec(g{1,1}{2}) - FirstMilis) / 1000; %In sec
        
        if (hex2dec(g{1,1}{1}) >= hex2dec('7FFFFFFF'))
            
            GyroY(j) = (hex2dec(g{1,1}{1}) - hex2dec('FFFFFFFF') - 1) / 100000000;
            
        else
            
            GyroY(j) = hex2dec(g{1,1}{1}) / 100000000;
            
        end
        j = j + 1;
    end
    
end

GyroYnoOutliers = filloutliers(GyroY,'linear','movmean',11);
TimenoOutliers = filloutliers(Time,'linear','movmean',11);
hold on
% plot(TimenoOutliers,GyroY)
plot(TimenoOutliers,GyroYnoOutliers)
% A = filter(0.2,1,GyroYnoOutliers) %Moving avg filter
% plot(TimenoOutliers,A)
title('Gyroscope Data')
xlabel('Time [sec]')
ylabel('Gyro [deg/sec]')
axis tight
% legend('Original Data','Filled Data')
hold off
LoggingRateHZ = size(TimenoOutliers,2) / TimenoOutliers(size(TimenoOutliers,2))