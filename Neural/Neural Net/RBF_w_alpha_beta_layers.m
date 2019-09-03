%radial basis function network emulation using multilayer perceptrons

%fit two inputs to one output
%
clear all
load 'arm_x.txt' %data file containing columns of feature 1, feature 2 and targets
theta1=arm_x(:,1);
theta2=arm_x(:,2);
target_vals=arm_x(:,3);
temp=size(theta1);
npatterns = temp(1); %there are this many training patterns in the file
bias_inputs = ones(npatterns,1); %fake input node for bias--always outputs 1 for every pattern

figure(1)
plot3(theta1,theta2,target_vals,'*') %take a look at the training data
title('display of training data')

ninputs=2; %theta1 and theta2, also weights from bias

%use this many alpha-layer perceptrons 
nalpha = 1000;  %EXPERIMENT WITH THIS VALUE
%use tansig() activation function for alpha nodes

%initialize weights from inputs to alpha layer to random values:
%weights below range from -1 to 1
W_to_alpha_from_inputs = 100*(rand([nalpha,ninputs+1])-0.5); %ones(nalpha,ninputs+1);

%beta layer should behave equivalent to radial basis funcs
%number of beta nodes must be less than number of training patterns
% use logsig() activation fnc for beta nodes

ngamma=1; %single output--use linear activation function, sigma=u



%initialize alpha-to-beta weights all to zero; include room for virtual
%bias alpha node

nbeta=80;
W_to_beta_from_alpha = zeros(nbeta,nalpha+1);

%initialize weights from beta layer to gamma (output) node:
%include room for virtual beta node bias input
W_to_gamma_from_beta = 2*rand([ngamma,nbeta+1])-1;


%pick training inputs to create beta nodes:
%nbeta=121;
%W_to_beta_from_alpha = zeros(nbeta,nalpha+1);

%initialize weights from beta layer to gamma (output) node:
%include room for virtual beta node bias input
%W_to_gamma_from_beta = 2*rand([ngamma,nbeta+1])-1;
% ibeta=0;
% xvals=[0:0.1:1]*0.1 +0.157;
% yvals=[0:0.1:1]*0.16 +0.22;
% xtrain=[];
% ytrain=[];
% for (i=1:11)
%     for(j=1:11)
%         ibeta=ibeta+1;
%         xtrain=[xtrain;xvals(i)];
%         ytrain=[ytrain;yvals(j)];
%         stim = [1;xvals(i);yvals(j)]; %stimulate network at this set of inputs, including bias
%         u_alphas = W_to_alpha_from_inputs*stim; %vector of alpha-node inputs
%         sig_alpha=tansig(u_alphas); %outputs of alpha layer--not including bias node
%         %sig_alpha=[1;sig_alpha]; % don't insert virtual bias node yet--
%         %observe effect of non-bias terms...
%        mag = sig_alpha'*sig_alpha %net input to each node, excluding bias effect
%        %wvec=2*sig_alpha/mag;
%        wvec = [1-mag;sig_alpha]; %define bias s.t. inputs are negative except near this stimulus
%        W_to_beta_from_alpha(ibeta,:)=wvec'; %install these weights for the next beta node      
%     end
% end
xtrain=[];
ytrain=[];
chosen_patterns=zeros(npatterns,1); %keep track of which patterns chosen
%alt--choose beta-training points selected at random from training patterns
for ibeta=1:nbeta
    jpick=1;
    while (chosen_patterns(jpick)==1)
      jpick=ceil(npatterns*rand());
    end
    chosen_patterns(jpick)=1;
    xval=theta1(jpick);
    yval=theta2(jpick);
     xtrain=[xtrain;xval];
     ytrain=[ytrain;yval];
     stim = [1;xval;yval]; %stimulate network at this set of inputs, including bias
        u_alphas = W_to_alpha_from_inputs*stim; %vector of alpha-node inputs
        sig_alpha=tansig(u_alphas); %outputs of alpha layer--not including bias node
        %sig_alpha=[1;sig_alpha]; % don't insert virtual bias node
        %yet--observe effect of non-bias terms...
       mag = sig_alpha'*sig_alpha %net input to each node, excluding bias effect
       %wvec=2*sig_alpha/mag;
       wvec = [1-mag;sig_alpha]; %define bias s.t. inputs are negative except near this stimulus
       W_to_beta_from_alpha(ibeta,:)=wvec'; %install these weights for the next beta node     
end

%debug--look at the inputs and outputs of each beta node
for ibeta=1:nbeta
    ibeta
    ffwd_beta_surfplot(W_to_alpha_from_inputs,W_to_beta_from_alpha,ibeta)
    title('trained beta node response')
    pause
end
%these all look good--localized gaussian-like functions centered on
%training inputs


%now utilize all training data...
%compute outputs of alpha layer:
%u_alphas has nalpha rows (one for each node) and npatterns columns 
u_alphas = W_to_alpha_from_inputs*[bias_inputs';theta1';theta2'];
sig_alphas=tansig(u_alphas);
sig_alphas=[ones(1,npatterns );sig_alphas]; %virtual bias nodes in first row
       
%compute beta-node responses to all stimuli:

%put beta-layer sigmas in columns
u_betas = W_to_beta_from_alpha*sig_alphas;
sig_betas=logsig(u_betas);
%sig_betas is nbeta rows of npat cols
%expand sig_betas to add a virtual extra node for bias:
sig_betas = [ones(1,npatterns);sig_betas]; %virtual beta node output = 1 for every stim
%call this F; want to find row vector w_vec from beta to alpha
%want   target_vals' = w_vec*sig_betas
%need to match: target_vals' = w_vec*sig_betas = w_vec*F
% or, target_vals = F' * w_vec'
%or, w_vec' = F'\target_vals

%use of pseudoinverse for min-squared error solution for w_vec:
F = sig_betas;
w_vec =F'\target_vals; %row vector of weights from beta to gamma
w_vec = w_vec';


%simulate:
x_sim = w_vec*sig_betas; %row vector of outputs for each stimulus

%compute the sum squared error for simulation of all training data:
errvec = target_vals - x_sim';
Esqd_avg = norm(errvec)/npatterns;
rms_err = sqrt(Esqd_avg)
pause

%same for training samples: nbeta = number of training samples
u_alphas = W_to_alpha_from_inputs*[ones(1,nbeta);xtrain';ytrain'];
sig_alphas=tansig(u_alphas);
sig_alphas=[ones(1,nbeta );sig_alphas]; %virtual bias nodes in first row
u_betas = W_to_beta_from_alpha*sig_alphas;
sig_betas=logsig(u_betas);
sig_betas = [ones(1,nbeta);sig_betas]; %virtual beta node output = 1 for every stim
z_train=w_vec*sig_betas;

%same for uniform scan over rectangular range:
xpts=[];
ypts=[];
zpts=[];
xvals=[0:0.1:1]*0.1 +0.157;
yvals=[0:0.1:1]*0.16 +0.22;
Z=zeros(11,11); %holder for 11x11 grid of outputsn
nsamps=0;
for (i=1:11)
    for(j=1:11)
        nsamps=nsamps+1;
        xpt=xvals(i);
        ypt=yvals(j);
        xpts=[xpts;xpt];
        ypts=[ypts;ypt];
         stim = [1;xpt;ypt]; %stimulate network at this set of inputs, including bias
         u_alphas = W_to_alpha_from_inputs*[1;xpt;ypt];
         sig_alphas=tansig(u_alphas); %outputs of alpha layer
         sig_alphas=[1;sig_alphas];
         u_betas = W_to_beta_from_alpha*sig_alphas;
         sig_betas=logsig(u_betas); %outputs of beta nodes
         %add extra virtual beta node = bias...
         sig_betas=[1;sig_betas];
         gamma = w_vec*sig_betas;
         Z(j,i)= gamma;  %note: surf() demands swap indices!
         zpts=[zpts;gamma];
    end
end


figure(1)
%plot3(theta1,theta2,target_vals,'*',theta1,theta2,x_sim,'x')
%plot3(theta1,theta2,target_vals,'*',theta1,theta2,x_sim,'x',xtrain,ytrain,z_train','o',xpts,ypts,zpts,'.')
plot3(theta1,theta2,target_vals,'*',xtrain,ytrain,z_train','o')

title('training points and beta-training subset response')

figure(2)
plot3(theta1,theta2,target_vals,'*',theta1,theta2,x_sim,'x',xtrain,ytrain,z_train','o',xpts,ypts,zpts,'.')
title('sample points and simulated function (surface)')
hold on;
surf(xvals,yvals,Z)
hold off;









