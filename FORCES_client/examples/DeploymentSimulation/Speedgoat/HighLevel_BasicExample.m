% initialize values
solvername = 'FORCESNLPsolver/p1/s1'; % name of the solver output
statename = 'Discrete State-Space'; % name of the state values output
modelname = 'speedgoat'; % simulink model name
output = zeros(1,30);
state = zeros(2,30);

tg = slrt; % get target handler

tg.load(modelname); %load the project
scope_idx = tg.getsignalid(solvername); % find the signal we need
state_idx1 = tg.getsignalid([statename,'/s1']); % find the signal we need
state_idx2 = tg.getsignalid([statename,'/s2']); % find the signal we need

% run simulation
tg.start(); % start the simulation
pause(0.5); % wait half simulation step to get accurate reading
for i = 1:30
    output(1,i) = tg.getsignal(scope_idx);
    state(1,i) = tg.getsignal(state_idx1);
    state(2,i) = tg.getsignal(state_idx2);
    fprintf('Step %d: U = %16.18f, X1 = %16.18f, X2 = %16.18f\n', ...
        i, output(1,i), state(1,i), state(2,i));
    pause(1); % wait a full simulation step to get next reading
end
tg.unload();

% plot results
figure(1); clf;
subplot(2,1,1); grid on; title('states'); hold on;
ylim(1.1*[-0.5,0.5]); stairs(1:30,state');
subplot(2,1,2);  grid on; title('input'); hold on;
ylim(1.1*[-0.5,0.5]); stairs(1:30,output');

