#include "mg1000.h"
#include <math.h>
#include <stddef.h>

static double clamp(double x, double lo, double hi)
{
    return fmin(fmax(x, lo), hi);
}

static double sign_of(double x)
{
    return (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0);
}

void mg1000_default_parameters(double p[MG1000_PARAM_COUNT])
{
    static const double defaults[MG1000_PARAM_COUNT] = {
        166500, 25418, 23000, 48418, 3.535, 2.652, 419164.524,
        7.530, 9.820, 0.984, 1.783, 0.2574, 750000, 0.35,
        858409.0965, 1.55, 2.00, 0.78, 0.05, 9.81, 0, 0, 0.145
    };
    size_t k;
    if (p != NULL) {
        for (k = 0; k < MG1000_PARAM_COUNT; ++k) p[k] = defaults[k];
    }
}

/* Source: chart_38 / chart_99, identical equations with sideSign=-1/+1. */
static int arm(double xun, double vun, double q, double qdot, double qddot,
                double Fxmag, double Fzmag, double Mdrum,
                double bodyPitch, double bodyPitchRate, double bodyPitchAcc,
                double ax, double an, double grade, double sideSign,
                const double *p, double *Fcylinder, double *Mreaction)
{
    const double mCut = p[3], Larm = p[4], Kcut = p[5], Icut = p[6];
    const double Pdist = 0.5*p[8], Qdist = p[22];
    const double L0 = p[15], L1 = p[16], L2 = p[17], minLever = p[18];
    const double g = p[19], rotDamping = p[20], cylDamping = p[21];
    const double theta = grade + bodyPitch;
    const double beta = -sideSign*q;
    const double betaAcc = -sideSign*qddot;
    const double phi = beta + sideSign*theta;
    const double phiDot = -sideSign*qdot + sideSign*bodyPitchRate;
    const double translationTerm = -sideSign*mCut*Kcut*sin(phi)*ax
                                  + mCut*Kcut*cos(phi)*an;
    const double pitchCoupling = sideSign*mCut*(Pdist*Kcut*cos(beta)
                                 + Qdist*Kcut*sin(beta))*bodyPitchAcc;
    const double armInertia = Icut*(betaAcc + sideSign*bodyPitchAcc);
    const double centrifugal = -mCut*(Pdist*Kcut*sin(beta)
                                 - Qdist*Kcut*cos(beta))*bodyPitchRate*bodyPitchRate;
    const double forceMoment = fabs(Fxmag)*Larm*sin(beta)
                               + fabs(Fzmag)*Larm*cos(beta);
    const double gravityMoment = mCut*Kcut*g*cos(phi);
    double Mmag = translationTerm + pitchCoupling + armInertia - centrifugal;
    double linkLength, cosBeta, lever;
    Mmag = Mmag + fabs(Mdrum) - sideSign*forceMoment + gravityMoment;
    Mmag = Mmag + rotDamping*phiDot;
    *Mreaction = -sideSign*Mmag;
    linkLength = fmax(L0 + xun, 1.0e-3);
    cosBeta = (L2*L2 + linkLength*linkLength - L1*L1)/(2.0*L2*linkLength);
    if (!isfinite(Mmag) || !isfinite(linkLength) || !isfinite(cosBeta)) return 0;
    cosBeta = clamp(cosBeta, -1.0, 1.0);
    lever = fmax(L2*sqrt(fmax(1.0-cosBeta*cosBeta, 0.0)), minLever);
    *Fcylinder = fabs(*Mreaction)/lever + cylDamping*fabs(vun);
    return isfinite(*Fcylinder);
}

/* Source: WholeMachineDynamicsComplete/chart_61. */
static int machine(const double *u, const double *p, double *y)
{
    const double M=p[0], mCut=p[3], Kcut=p[5], B=p[7], Pdist=0.5*p[8];
    const double Rh=p[9], Rs=p[10], wheelRadius=p[11], tractionLimit=p[12];
    const double rollingCoeff=p[13], Ipitch=p[14], g=p[19], Qdist=p[22];
    const double ax=u[18], bodyPitchRate=u[20], bodyPitchAcc=u[21], an=u[22];
    const double theta=u[23]+u[19], vMean=0.5*(u[0]+u[1]);
    double direction;
    double phiL, phiR, phiDotL, phiDotR, phiAccL, phiAccR;
    double xddRelL, xddRelR, zddRelL, zddRelR;
    double FzL, FzR, Ntotal, baseL, baseR, FxL, FxR;
    double McutAtPivots, Mheight, Mtarget, NL, NR;
    double FtracL=0.0, FtracR=0.0, fshoeL=0.0, fshoeR=0.0, Mtraction=0.0;
    double FreqL, FreqR, normalDifference, NLraw, NRraw, Msupport;
    int iteration;
    if (!isfinite(vMean) || !isfinite(u[2]+u[3])) return 0;
    if (fabs(vMean)>1.0e-4) direction=sign_of(vMean);
    else if (fabs(u[2]+u[3])>1.0) direction=sign_of(u[2]+u[3]);
    else direction=0.0;
    phiL=u[10]-theta; phiR=-u[15]+theta;
    phiDotL=u[11]-bodyPitchRate; phiDotR=-u[16]+bodyPitchRate;
    phiAccL=u[12]-bodyPitchAcc; phiAccR=-u[17]+bodyPitchAcc;
    xddRelL=Kcut*(cos(phiL)*phiDotL*phiDotL + sin(phiL)*phiAccL);
    xddRelR=-Kcut*(cos(phiR)*phiDotR*phiDotR + sin(phiR)*phiAccR);
    zddRelL=Kcut*(-sin(phiL)*phiDotL*phiDotL + cos(phiL)*phiAccL);
    zddRelR=Kcut*(sin(phiR)*phiDotR*phiDotR - cos(phiR)*phiAccR);
    FzL=fabs(u[5]); FzR=-fabs(u[7]);
    Ntotal=M*(g*cos(theta)+an) + mCut*(zddRelL+zddRelR) - FzL - FzR;
    if (!isfinite(Ntotal)) return 0;
    Ntotal=fmax(Ntotal,0.0);
    baseL=0.5*M*ax + mCut*xddRelL + 0.5*M*g*sin(theta) + direction*fabs(u[4]);
    baseR=0.5*M*ax + mCut*xddRelR + 0.5*M*g*sin(theta) + direction*fabs(u[6]);
    FxL=-direction*fabs(u[4]); FxR=-direction*fabs(u[6]);
    McutAtPivots=Qdist*(FxL+FxR) + Pdist*(FzL-FzR);
    Mheight=y[11]+y[12]; Mtarget=Ipitch*bodyPitchAcc;
    NL=0.5*Ntotal; NR=0.5*Ntotal;
    for (iteration=0; iteration<12; ++iteration) {
        fshoeL=direction*rollingCoeff*fmax(NL,0.0);
        fshoeR=direction*rollingCoeff*fmax(NR,0.0);
        FreqL=baseL+fshoeL; FreqR=baseR+fshoeR;
        if (!isfinite(FreqL) || !isfinite(FreqR)) return 0;
        FtracL=clamp(FreqL,-tractionLimit,tractionLimit);
        FtracR=clamp(FreqR,-tractionLimit,tractionLimit);
        Mtraction=Rh*(FtracL+FtracR)-Rs*(fshoeL+fshoeR);
        normalDifference=2.0*(Mtarget-Mtraction-McutAtPivots-Mheight)/B;
        NLraw=0.5*(Ntotal-normalDifference); NRraw=0.5*(Ntotal+normalDifference);
        if (!isfinite(NLraw) || !isfinite(NRraw)) return 0;
        if (Ntotal<=0.0) { NL=0.0; NR=0.0; }
        else if (NLraw<0.0) { NL=0.0; NR=Ntotal; }
        else if (NRraw<0.0) { NL=Ntotal; NR=0.0; }
        else { NL=NLraw; NR=NRraw; }
    }
    FreqL=baseL+direction*rollingCoeff*fmax(NL,0.0);
    FreqR=baseR+direction*rollingCoeff*fmax(NR,0.0);
    if (!isfinite(FreqL) || !isfinite(FreqR)) return 0;
    FtracL=clamp(FreqL,-tractionLimit,tractionLimit);
    FtracR=clamp(FreqR,-tractionLimit,tractionLimit);
    fshoeL=direction*rollingCoeff*fmax(NL,0.0);
    fshoeR=direction*rollingCoeff*fmax(NR,0.0);
    Mtraction=Rh*(FtracL+FtracR)-Rs*(fshoeL+fshoeR);
    y[0]=wheelRadius*FtracL; y[1]=wheelRadius*FtracR;
    y[2]=FtracL; y[3]=FtracR; y[9]=NL; y[10]=NR;
    y[13]=tractionLimit-fabs(FtracL); y[14]=tractionLimit-fabs(FtracR);
    y[16]=(FtracL+FtracR)-(FreqL+FreqR);
    Msupport=0.5*B*(NR-NL);
    y[15]=Msupport+Mtraction+McutAtPivots+Mheight-Mtarget;
    return 1;
}

Mg1000Status mg1000_step(const double u[MG1000_INPUT_COUNT],
                         const double p[MG1000_PARAM_COUNT],
                         double output[MG1000_OUTPUT_COUNT])
{
    double y[MG1000_OUTPUT_COUNT];
    size_t k;
    if (u==NULL || p==NULL || output==NULL) return MG1000_BAD_ARGUMENT;
    for (k=0; k<MG1000_INPUT_COUNT; ++k)
        if (!isfinite(u[k])) return MG1000_BAD_ARGUMENT;
    for (k=0; k<MG1000_PARAM_COUNT; ++k)
        if (!isfinite(p[k]) || p[k]<0.0) return MG1000_BAD_PARAMETER;
    if (p[7]<=0.0 || p[17]<=0.0 || p[18]<=0.0) return MG1000_BAD_PARAMETER;
    if (!arm(u[8],u[9],u[10],u[11],u[12],u[4],u[5],u[24],
        u[19],u[20],u[21],u[18],u[22],u[23],-1.0,p,&y[4],&y[11]))
        return MG1000_NUMERIC_ERROR;
    if (!arm(u[13],u[14],u[15],u[16],u[17],u[6],u[7],u[25],
        u[19],u[20],u[21],u[18],u[22],u[23],1.0,p,&y[5],&y[12]))
        return MG1000_NUMERIC_ERROR;
    y[6]=u[19]; y[7]=u[10]; y[8]=u[15];
    if (!machine(u,p,y)) return MG1000_NUMERIC_ERROR;
    for (k=0; k<MG1000_OUTPUT_COUNT; ++k)
        if (!isfinite(y[k])) return MG1000_NUMERIC_ERROR;
    for (k=0; k<MG1000_OUTPUT_COUNT; ++k) output[k]=y[k];
    return MG1000_OK;
}
