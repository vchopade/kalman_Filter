%%NAME: Vishal Chopade
%%ADEX Technologies
%%DATE: 04/02/2018
%%Matlab V 9.4 2018version
%%Used the free versiom fro students.

%the terminology of the equations is explained in the papers attached with
%the email.

% I have used the following video and article links-
% https://www.mathworks.com/videos/understanding-kalman-filters-part-1-why-use-kalman-filters--1485813028675.html
% https://www.youtube.com/watch?v=bm3cwEP2nUo
% https://www.youtube.com/watch?v=ul3u2yLPwU0





clear all   %clear removes all variables from the current workspace, releasing them from system memory.
%% defining the smapling duration
duration = 10  
dt = .1; 

%% (Coefficent matrices): [state transition (state + velocity)] + [input control (acceleration)]
A = [1 dt; 0 1] ;   %state transition matrix state prediction
B = [dt^2/2; dt];   %input control matrix acceleration.
C = [1 0];          % measurement matrix


%% main variable and constants declaration heading
u = 1.5;                                                    %acelerometer
V= [0; 0];                                                  %position and velocity of vehicle
V_estimate = V;                                             %x_estimate of initial location estimation of where the vehicle is
vehicle_noise_mag = 0.05;                                   %process noise: how fast the vehicle is speeding up (stdv of acceleration: meters/sec^2)
Signal_noise_mag = 50;                                      %measurement noise (in meters) as per textbook
Ez = Signal_noise_mag^2;                                    % Ez convert the measurement noise into covariance matrix
Ex = vehicle_noise_mag^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2];    % same for Ex
P = Ex;                                                     % initial vehicle position estimate(covariance matrix)

V_loc = [];             % ACTUAL vehicle path
vel = [];               % ACTUAL vehicle velocity
V_loc_meas = [];        % vehicle path that signal can reach


%% simulate what the Signal gets over time
figure(2);clf          %clf is used because it deletes from the current figure all graphics objects whose handles are not hidden 
figure(1);clf          %same as above
for t = 0 : dt: duration

    
    vehicle_noise = vehicle_noise_mag * [(dt^2/2)*randn; dt*randn];     % Generate the vehicle path random number generate
    V= A * V+ B * u + vehicle_noise;
    
    
    Signal_noise = Signal_noise_mag * randn;                            % Generate what the Signal sees
    y = C * V+ Signal_noise;
    V_loc = [V_loc; V(1)];
    V_loc_meas = [V_loc_meas; y];
    vel = [vel; V(2)];
    
    plot(0:dt:t, V_loc, 'r')                                            %iteratively plot what the Signal sees
    plot(0:dt:t, V_loc_meas, '-k.')
    axis([0 10 -30 80])
    hold on  %explained
    
end

%plotting the theoretical path of Signal that doesn't use kalman. the red
%line will be missing in the graph
%velocity, its constantly increasing, due to constant acceleration
plot(0:dt:t, V_loc_meas, 'g')       % every alphabet used here is for colour




%% Kalman filtering Heading
%initizing estimation variables
V_loc_estimate = [];        %  vehicle position estimate
vel_estimate = [];          % vehicle velocity estimate
V= [0; 0];                  % re-initizing state of the vehicle
P_estimate = P;
P_mag_estimate = [];        
predic_state = [];          %predicting state matrix
predic_var = [];
for t = 1:length(V_loc)
    % Predict next state of the vehicle with the last state (feedback) and predicted motion.
    V_estimate = A * V_estimate + B * u; %u is controlled motion
    predic_state = [predic_state; V_estimate(1)] ;
    %predict next covariance
    P = A * P * A' + Ex;3    %A' is transpose of A
    predic_var = [predic_var; P] ;
    K = P*C'*inv(C*P*C'+Ez); % Kalman Gain
    V_estimate = V_estimate + K * (V_loc_meas(t) - C * V_estimate);   % Update the state estimate.
    % update covariance estimation.
    P =  (eye(2)-K*C)*P;
    %Store for plotting
    V_loc_estimate = [V_loc_estimate; V_estimate(1)];
    vel_estimate = [vel_estimate; V_estimate(2)];
    P_mag_estimate = [P_mag_estimate; P(1)]
end
 
% Plot the results on graph, the red line is so into the green that the
% observer will have to zoom the plot.
figure(2);
tt = 0 : dt : duration;
plot(tt,V_loc,'r',tt,V_loc_meas,'k', tt,V_loc_estimate,'g'); %red black green
axis([0 10 -30 80])   %the x to x and y to y resp.
%tt- To write a timetable out to a text or spreadsheet file, first convert it to a table with timetable2table.
%Then write the table to a file with the writetable function.

%plot the evolution of the distributions
figure(3);clf
for T = 1:length(V_loc_estimate)
clf % not using the clf will give the graph verious garbage graphs
    x = V_loc_estimate(T)-5:.01:V_loc_estimate(T)+5; % range on x axis
     
    %predicted next position of the vehicle    
    hold on   %hold on retains plots in the current axes so that new plots added to the axes do not delete existing plots
    mu = predic_state(T); % mean
    sigma = predic_var(T); % standard deviation
    y = normpdf(x,mu,sigma); % pdf
    y = y/(max(y));
    hl = line(x,y,'Color','m'); % or use hold on and normal plot
      
    %data measured by the Signal in graph 1
    mu = V_loc_meas(T); % mean
    sigma = Signal_noise_mag; % standard deviation
    y = normpdf(x,mu,sigma); % pdf
    y = y/(max(y));
    hl = line(x,y,'Color','k'); % or use hold on and normal plot
   
    %combined position estimate in graph 2
    mu = V_loc_estimate(T); % mean
    sigma = P_mag_estimate(T); % standard deviation
    y = normpdf(x,mu,sigma); % pdf
    y = y/(max(y));
    hl = line(x,y, 'Color','g'); % or use hold on and normal plot
    axis([V_loc_estimate(T)-5 V_loc_estimate(T)+5 0 1]);    

   
    %actual position of the vehicle is given in the 3rd graph
    plot(V_loc(T));
    ylim=get(gca,'ylim');
    line([V_loc(T);V_loc(T)],ylim.','linewidth',2,'color','b');
    legend('state predicted','measurement','state estimate','actual vehicle position') %notations on the graph
    %pause
end