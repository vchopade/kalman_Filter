%% Name: Vishal Gajanan Chopade
%  Project: One stage kalman filter.
%  ADEX Technologies

%  Please refer the page 632 for equations
%  Kindly give me the Feedback on the results, This helps me learning.



%% Initialization of the variables
ORDER = 1;
XLOSE=99999999999;
W=.1;
PHIS=10.;
TS=1.;
BIASH=0.;
BIAS=1.;
SIGNOISE=50;
M=(1:1);
P=(1:1);
K=(1:1);
PHI=(1:1);
H=(1:1);
R=(1:1);
PHIT=(1:1);
XNOISE = 0;
%% initializing the matrices
PHIP=(1:1);HT=(1:1);KH=(1:1);IKH=(1:14);
MHT=(1:1);HMHT=(1:1);HMHTR=(1:1);HMHTRINV=(1:1);IDN=(1:1);
Q=(1:1);PHIPPHIT=(1:1);
PHI(1:1)=0.;
P(1:1)=0.;
IDN(1:1)=0.;
Q(1:1)=0.;
IDN(1:1)=1.;
P(1:1)=99999999999999.;
PHI(1:1)=1;
H(1:1)=0;

%% PHIT in PHI transpose and Q and R are defined
R(1:1)=SIGNOISE*2;
PHIT = transpose(PHI);
Q(1:1)=PHIS*TS;

%% For loop is used as a replacement of DO loop in Fortran from T=0 to 200 to get 200 values for an event

for T = [1:200]
    if(T>XLOSE) % this condition is not actually affecting the loop as XLOSE is way greater than T
        {
            R(1:1)==999999999999999;
            }
    end
    
H(1:1)= 5*T^2; %T square
HT = transpose(H); % traqnspose is denoted by T, so HT
PHIP = PHIP*P;
PHIPPHIT = PHIP*PHIT; % the vriables are taken from the sample example in the Textbook to make the understanding simple
M = PHIPPHIT + Q;
MHT = M * HT;
HMHT = H * MHT;

HMHTR(1:1)= HMHT(1:1)+R(1:1);
HMHTRINV(1:1) = 1/HMHTR(1:1);

K = MHT * HMHTRINV;
KH = K * H;
IKH = IDN - KH;
P = IKH * M;

SUM=0;
%% Gauss function
for J = [1:200]
  IRAN = randi(32768);
  SUM=SUM+IRAN;
end
%% continue of For loop
XNOISE = SUM/65536;
XNOISE = 1.414 * XNOISE * SIGNOISE;


X = (100 * T) - (20 * cos(W*T) /W)+ (20 /W);
XD=100 + (20 * sin(W*T)); %derivative of the X
XDD=20 * W * cos(W*T); % derivative of the XD
XGPSS = X + XNOISE;
XINSS = X + ((0.5)* BIAS * T * T); % Zins as explained in the textbook
XS = XINSS - XGPSS;
RES = XS-(.5*BIASH*T*T);
BIASH = BIASH + K(1:1)*RES;
SP11 = sqrt(P(1,1));
BIASERR = BIAS - BIASH;
XH = XINSS-(0.5*BIASH*T*T);

%% fprintf is using the values to print to the console
% the program requests the Write statementin fortran refers to print in Matlab 
fprintf('%d \n',T)
fprintf('Value of BIAS= %f \n',BIAS)
fprintf('Value of BIASH= %f \n',BIASH)
fprintf('Value of BIASERR= %f \n',BIASERR)
fprintf('Value of SP11= %f \n',SP11)
fprintf('Value of -SP11= %f \n',-SP11)
fprintf('Value of X= %f \n',X)
fprintf('Value of XD= %f \n',XD)
fprintf('Value of XGPSS or Zgps= %f \n',XGPSS)
fprintf('Value of XINSS or Zins= %f \n',XINSS)
fprintf('_____________________________________ \n')
end
%% printing the outputs







