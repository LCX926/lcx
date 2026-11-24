#include "mg1000.h"
#include <stdio.h>
int main(void)
{
    double p[MG1000_PARAM_COUNT], u[MG1000_INPUT_COUNT]={0}, y[MG1000_OUTPUT_COUNT];
    mg1000_default_parameters(p);
    u[MG_IN_VL]=0.05; u[MG_IN_VR]=0.05;
    u[MG_IN_FX_L]=100000; u[MG_IN_FX_R]=100000;
    u[MG_IN_FZ_L]=50000; u[MG_IN_FZ_R]=50000;
    u[MG_IN_Q_L]=0.2; u[MG_IN_Q_R]=-0.2;
    u[MG_IN_MDRUM_L]=200000; u[MG_IN_MDRUM_R]=200000;
    if (mg1000_step(u,p,y)!=MG1000_OK) return 1;
    printf("Tload L/R: %.6f %.6f Nm\n", y[0],y[1]);
    printf("Fcylinder L/R: %.6f %.6f N\n", y[4],y[5]);
    printf("Residual pitch/longitudinal: %.9g Nm / %.9g N\n",y[15],y[16]);
    return 0;
}
