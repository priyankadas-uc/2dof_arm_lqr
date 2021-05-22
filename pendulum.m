function [sys,x0,str,ts,simStateCompliance] = pendulum(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,P);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = P.x0;

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,P)
F=zeros(2,1);
F=u;
xdot=zeros(4,1);
theta1=x(1);
theta2=x(2);
theta1dot=x(3);
theta2dot=x(4);
m1=P.m1;
m2=P.m2;
l1=P.l1;
l2=P.l2;
lg1=P.lg1;
lg2=P.lg2;
j1=P.j1;
j2=P.j2;
g=P.g;
% Bq=[(m1+m2)*l1^2+(m2*l2^2)+2*m2*l1*l2*cos(theta2) m2*l2^2+m2*l1*l2*cos(theta2);
%     m2*l2^2+m2*l1*l2*cos(theta2) m2*l2^2];
% xdot(1)=x(3);
% xdot(2)=x(4);
% Cq=[-m2*l1*l2*sin(theta2)*(2*theta1dot*theta2dot+theta2dot^2);
%     -2*l1*l2*sin(theta2)*theta1dot*theta2dot];
% Gq=[-(1+m2)*g*l1*sin(theta1)-m2*g*l2*sin(theta1+theta2);
%     -2*g*l2*sin(theta1+theta2)];
% xdot(3:4)=Bq^(-1)*(F-Gq-Cq)
Bq=[m1*(lg1^2)+m2*(l1^2)+j1 m2*l1*lg2*cos(theta1-theta2);
    m2*l1*lg2*cos(theta1-theta2) m2*lg2^2+2];
xdot(1)=x(3);
xdot(2)=x(4);
Cq=m2*l1*lg2*g*sin(theta1-theta2)*[theta1dot;theta2dot];
Gq=[(m1*lg1+m2*l1)*cos(theta1);
    m2*lg2*g*cos(theta2)];
xdot(3:4)=Bq^(-1)*(F-Gq-Cq);
sys = xdot;

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,P)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
