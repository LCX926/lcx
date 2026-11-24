function [Fcylinder,Mreaction] = fcn(xun,vun,q,qdot,qddot,Fxmag,Fzmag,Mdrum,bodyPitch,bodyPitchRate,bodyPitchAcc,ax,an,grade,sideSign,P)
%#codegen
% Arm inverse dynamics based on the mirrored left/right equations.
% sideSign=-1 left, +1 right. qR is negative when the right arm is down.
mCut = P(4);
Larm = P(5);
Kcut = P(6);
Icut = P(7);
pivotSpan = P(9);
Qdist = P(23);
L0 = P(16);
L1 = P(17);
L2 = P(18);
minLever = P(19);
g = P(20);
rotDamping = P(21);
cylDamping = P(22);
theta = grade + bodyPitch;
Pdist = 0.5*pivotSpan;
% Convert the right input angle to the positive-down beta convention.
beta = -sideSign*q;
betaDot = -sideSign*qdot;
betaAcc = -sideSign*qddot;
phi = beta + sideSign*theta;
phiDot = betaDot + sideSign*bodyPitchRate;
% Terms are arranged from equations (3-30)/(3-32): the left cutting
% force moment increases the required reaction; the mirrored right
% cutting force moment decreases it. Drum resistance torque increases
% the reaction-moment magnitude on both sides.
translationTerm = -sideSign*mCut*Kcut*sin(phi)*ax + mCut*Kcut*cos(phi)*an;
pitchCoupling = sideSign*mCut*(Pdist*Kcut*cos(beta) + Qdist*Kcut*sin(beta))*bodyPitchAcc;
armInertia = Icut*(betaAcc + sideSign*bodyPitchAcc);
centrifugal = -mCut*(Pdist*Kcut*sin(beta) - Qdist*Kcut*cos(beta))*bodyPitchRate*bodyPitchRate;
forceMoment = abs(Fxmag)*Larm*sin(beta) + abs(Fzmag)*Larm*cos(beta);
gravityMoment = mCut*Kcut*g*cos(phi);
Mmag = translationTerm + pitchCoupling + armInertia - centrifugal;
Mmag = Mmag + abs(Mdrum) - sideSign*forceMoment + gravityMoment;
Mmag = Mmag + rotDamping*phiDot;
% Signed convention kept for the existing output: left positive, right negative.
Mreaction = -sideSign*Mmag;
linkLength = max(L0 + xun,1.0e-3);
cosBeta = (L2*L2 + linkLength*linkLength - L1*L1)/(2.0*L2*linkLength);
cosBeta = min(max(cosBeta,-1.0),1.0);
lever = max(L2*sqrt(max(1.0-cosBeta*cosBeta,0.0)),minLever);
Fcylinder = abs(Mreaction)/lever + cylDamping*abs(vun);
end