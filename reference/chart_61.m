function [TloadL,TloadR,FtracL,FtracR,NL,NR,marginL,marginR,pitchResidual,longitudinalResidual] = fcn(vL,vR,TcmdL,TcmdR,FxLmag,FzLmag,FxRmag,FzRmag,qL,qdotL,qddotL,qR,qdotR,qddotR,ax,bodyPitch,bodyPitchRate,bodyPitchAcc,an,grade,MreactionL,MreactionR,P)
%#codegen
% MG1000 reduced whole-machine inverse dynamics.
% Force convention: Fx inputs are resistance magnitudes. Positive Fz
% means upward at the left drum and downward at the right drum, matching
% the mirrored drum convention in the governing free-body diagram.
M = P(1);
mCut = P(4);
Larm = P(5);
Kcut = P(6);
B = P(8);
pivotSpan = P(9);
Rh = P(10);
Rs = P(11);
wheelRadius = P(12);
tractionLimit = P(13);
rollingCoeff = P(14);
Ipitch = P(15);
g = P(20);
Qdist = P(23);
theta = grade + bodyPitch;
vMean = 0.5*(vL + vR);
if abs(vMean) > 1.0e-4
    direction = sign(vMean);
elseif abs(TcmdL + TcmdR) > 1.0
    direction = sign(TcmdL + TcmdR);
else
    direction = 0.0;
end
% Mirrored arm coordinates: qL is positive upward; qR is negative
% when the right arm is below the body. betaR=-qR is the positive
% downward angle used by the right-arm governing equation.
betaL = qL;
betaR = -qR;
phiL = betaL - theta;
phiR = betaR + theta;
phiDotL = qdotL - bodyPitchRate;
phiDotR = -qdotR + bodyPitchRate;
phiAccL = qddotL - bodyPitchAcc;
phiAccR = -qddotR + bodyPitchAcc;
xddRelL = Kcut*(cos(phiL)*phiDotL*phiDotL + sin(phiL)*phiAccL);
xddRelR = -Kcut*(cos(phiR)*phiDotR*phiDotR + sin(phiR)*phiAccR);
zddRelL = Kcut*(-sin(phiL)*phiDotL*phiDotL + cos(phiL)*phiAccL);
zddRelR = Kcut*(sin(phiR)*phiDotR*phiDotR - cos(phiR)*phiAccR);
% Left normal cutting force is upward; right is downward.
FzL = abs(FzLmag);
FzR = -abs(FzRmag);
Ntotal = M*(g*cos(theta) + an) + mCut*(zddRelL + zddRelR) - FzL - FzR;
Ntotal = max(Ntotal,0.0);
% Per-side longitudinal demand follows Ft_i=f_i+Fx_i+Ft_i(gravity/inertia).
baseL = 0.5*M*ax + mCut*xddRelL + 0.5*M*g*sin(theta) + direction*abs(FxLmag);
baseR = 0.5*M*ax + mCut*xddRelR + 0.5*M*g*sin(theta) + direction*abs(FxRmag);
% Forces at the two arm pivots contribute with the mirrored signs shown
% in the body pitch equation. The arm-length contribution is already in
% MreactionL/MreactionR and must not be counted a second time here.
Pdist = 0.5*pivotSpan;
FxL = -direction*abs(FxLmag);
FxR = -direction*abs(FxRmag);
McutAtPivots = Qdist*(FxL + FxR) + Pdist*(FzL - FzR);
Mheight = MreactionL + MreactionR;
Mtarget = Ipitch*bodyPitchAcc;
NL = 0.5*Ntotal;
NR = 0.5*Ntotal;
FtracL = 0.0;
FtracR = 0.0;
fshoeL = 0.0;
fshoeR = 0.0;
Mtraction = 0.0;
% Fixed-count iteration resolves the support/friction coupling.
% Twelve passes keep the algebraic residual below the validation tolerance
% for the thesis-based coefficient without introducing an algebraic loop.
for iteration = 1:12
    fshoeL = direction*rollingCoeff*max(NL,0.0);
    fshoeR = direction*rollingCoeff*max(NR,0.0);
    FreqL = baseL + fshoeL;
    FreqR = baseR + fshoeR;
    FtracL = min(max(FreqL,-tractionLimit),tractionLimit);
    FtracR = min(max(FreqR,-tractionLimit),tractionLimit);
    Mtraction = Rh*(FtracL + FtracR) - Rs*(fshoeL + fshoeR);
    normalDifference = 2.0*(Mtarget - Mtraction - McutAtPivots - Mheight)/B; % NR-NL
    NLraw = 0.5*(Ntotal - normalDifference);
    NRraw = 0.5*(Ntotal + normalDifference);
    if Ntotal <= 0.0
        NL = 0.0;
        NR = 0.0;
    elseif NLraw < 0.0
        NL = 0.0;
        NR = Ntotal;
    elseif NRraw < 0.0
        NL = Ntotal;
        NR = 0.0;
    else
        NL = NLraw;
        NR = NRraw;
    end
end
FreqL = baseL + direction*rollingCoeff*max(NL,0.0);
FreqR = baseR + direction*rollingCoeff*max(NR,0.0);
FtracL = min(max(FreqL,-tractionLimit),tractionLimit);
FtracR = min(max(FreqR,-tractionLimit),tractionLimit);
fshoeL = direction*rollingCoeff*max(NL,0.0);
fshoeR = direction*rollingCoeff*max(NR,0.0);
Mtraction = Rh*(FtracL + FtracR) - Rs*(fshoeL + fshoeR);
TloadL = wheelRadius*FtracL;
TloadR = wheelRadius*FtracR;
marginL = tractionLimit - abs(FtracL);
marginR = tractionLimit - abs(FtracR);
longitudinalResidual = (FtracL + FtracR) - (FreqL + FreqR);
Msupport = 0.5*B*(NR - NL);
pitchResidual = Msupport + Mtraction + McutAtPivots + Mheight - Mtarget;
end