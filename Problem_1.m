%% DOCUMENTATION !!-- Date April 16, 2011 --!!
% Program Documentation:
% Developer: Nishant Joshi
% Roll No.: 0800201
% Dept.: Electrical Engineering

%%
clc
clear all
close all
%%
number_cars = 100;
car_vel = 10;
car_speed = car_vel*ones(number_cars,1);
car_acceleration = zeros(number_cars,1);
cow_speed = 0.75;
number_cows = 10;
lane_length = 3;
total_time = 1:100;
time = size(total_time,2);
location = zeros(number_cars,time);
min_dist = 5;

cow_start_time = 22;
cow_location = 100;

%% Free Flow of cars
for i = 1:number_cars
     location(i,:) = car_speed(i)*(total_time(:)-2*i);
     hold on;
     plot(location(i,:));
end

location_temp = location;
%% Modelling Car flow with herd of cows

for i = 1:number_cars
    if location(i,cow_start_time)==cow_location
        car_index = i+1;
        break
    end
end

avg_dist = location(1,1)-location(2,1);
stop_dist = 0;

for i = car_index:number_cars
    stop_dist = stop_dist + avg_dist - min_dist;
    car_acceleration(i) = -(car_vel^2)/(2*stop_dist); 
end

for i = car_index:number_cars
    for j = cow_start_time : time
        if (car_vel + car_acceleration(i)*(total_time(j)-cow_start_time))> 0
            location_temp(i,j) = location_temp(i,cow_start_time) + (car_vel*(total_time(j)-cow_start_time)+(0.5*car_acceleration(i)*((total_time(j)-cow_start_time)^2)));
            car_speed(i) = car_vel + car_acceleration(i)*(total_time(j)-cow_start_time);
        else
            car_speed(i) = 0; 
            location_temp(i,j) = location_temp(i,j-1);  
        end
    end
    hold on;
%     plot(location_temp(i,:));
end

cow_stop_time = ((lane_length/cow_speed)*number_cows) + cow_start_time;

for i = car_index:number_cars
    car_acceleration(i) = 4;
end
figure(2);
for i = car_index:number_cars
    for j = cow_stop_time : time
        if(car_speed(i) + car_acceleration(i)*(total_time(j)-cow_start_time))<10
            location_temp(i,j) = location_temp(i,cow_stop_time) + (car_speed(i)*(total_time(j)-cow_stop_time)+(0.5*car_acceleration(i)*((total_time(j)-cow_stop_time)^2)));
        else
            location_temp(i,j) = car_vel + location_temp(i,j-1);
        end
    end
    hold on;
    plot(location_temp(i,:));
end