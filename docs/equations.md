# 原模型逻辑、方程和 C 对应

以下方程按原函数重写，没有重新推导或修正模型。原函数注释提到的论文式号并不随文件提供，因此不能据此验证论文原式。

## 1. 调用图与端口

`mg1000_step()` → 左侧 `arm(sideSign=-1)` 和右侧 `arm(sideSign=+1)` → `machine()` → 整理 17 个输出。

| 原模型 | 归档源码 | C 函数 |
|---|---|---|
| HeightReactionLeftMG1000 | reference/chart_38.m | arm，s=-1 |
| HeightReactionRightMG1000 | reference/chart_99.m | arm，s=+1 |
| WholeMachineDynamicsComplete | reference/chart_61.m | machine |
| MG1000_Parameter_Config/P_MG1000 | reference/parameters.json | mg1000_default_parameters |

顶层端口完整顺序见 `reference/ports.json`，其数组索引与头文件枚举顺序一致。
输入 1～8 是左右牵引速度、牵引轮转矩、滚筒 Fx/Fz；9～18 是左右油缸位移/速度和摇臂角度/角速度/角加速度；
19～24 是纵向加速度、相对机身俯仰角/角速度/角加速度、法向加速度、坡度；25～26 是左右滚筒阻力矩。

输出 1～4 是左右牵引负载转矩与牵引阻力；5～6 是油缸负载；7～9 是机身和摇臂角度直通；
10～13 是左右支撑力和摇臂反作用力矩；14～17 是牵引余量与两个平衡残差。
所有编号在这里从 1 开始，C 数组索引减 1。

## 2. 参数对应

名称依据原函数局部变量解释；高度、跨度的具体测量基准还需机械图纸确认。

| MATLAB 参数 | C 枚举（MG_P_前缀） | 默认值 | 含义/单位 |
|---|---|---:|---|
| P(1) | MASS | 166500 | 整机质量 M，kg |
| P(2) | UNUSED_2 | 25418 | 本次三个函数未使用 |
| P(3) | UNUSED_3 | 23000 | 本次三个函数未使用 |
| P(4) | ARM_MASS | 48418 | mCut，kg |
| P(5) | ARM_LENGTH | 3.535 | Larm，m |
| P(6) | ARM_COM_RADIUS | 2.652 | Kcut，m |
| P(7) | ARM_INERTIA | 419164.524 | Icut，kg·m² |
| P(8) | SUPPORT_SPAN | 7.530 | B，m |
| P(9) | PIVOT_SPAN | 9.820 | pivotSpan，m |
| P(10) | TRACTION_HEIGHT | 0.984 | Rh，m |
| P(11) | SHOE_HEIGHT | 1.783 | Rs，m |
| P(12) | WHEEL_RADIUS | 0.2574 | 牵引轮半径，m |
| P(13) | TRACTION_LIMIT | 750000 | 每侧牵引力限幅，N |
| P(14) | FRICTION | 0.35 | rollingCoeff，原代码用于滑靴摩擦 |
| P(15) | PITCH_INERTIA | 858409.0965 | 俯仰惯量，kg·m² |
| P(16) | LINK_L0 | 1.55 | L0，m |
| P(17) | LINK_L1 | 2.00 | L1，m |
| P(18) | LINK_L2 | 0.78 | L2，m |
| P(19) | MIN_LEVER | 0.05 | 最小力臂，m |
| P(20) | GRAVITY | 9.81 | g，m/s² |
| P(21) | ROT_DAMPING | 0 | 转动阻尼，N·m·s/rad |
| P(22) | CYL_DAMPING | 0 | 油缸阻尼，N·s/m |
| P(23) | PIVOT_OFFSET | 0.145 | Qdist，m |

## 3. 左右摇臂逆动力学

记坡度为 γ，机身相对俯仰角为 ψ，s=-1 表示左侧、s=+1 表示右侧。
用 aₓ、aₙ 表示机身纵向和法向加速度，P₀=pivotSpan/2，Q=Qdist，m=mCut，K=Kcut，L=Larm。

$$
\theta=\gamma+\psi,\qquad
\beta=-sq,\quad \dot\beta=-s\dot q,\quad \ddot\beta=-s\ddot q,
\qquad \phi=\beta+s\theta,\quad \dot\phi=\dot\beta+s\dot\psi.
$$

左右镜像关系由这些式子固定：左侧 φ=qL−θ，右侧 φ=−qR+θ。不要在接口外再任意反转符号。

原代码各项对应为：

$$
\begin{aligned}
T_{trans}&=-smK\sin\phi\,a_x+mK\cos\phi\,a_n,\\
T_{pitch}&=sm(P_0K\cos\beta+QK\sin\beta)\ddot\psi,\\
T_{inertia}&=I_{cut}(\ddot\beta+s\ddot\psi),\\
C&=-m(P_0K\sin\beta-QK\cos\beta)\dot\psi^2,\\
T_F&=|F_x|L\sin\beta+|F_z|L\cos\beta,\\
T_g&=mKg\cos\phi.
\end{aligned}
$$

在 C `arm()` 中依次对应 `translationTerm`、`pitchCoupling`、`armInertia`、`centrifugal`、`forceMoment`、`gravityMoment`。
注意代码中 C 项本身带负号，组合时又减去 C；不能凭名称改写符号。

$$
M_{mag}=T_{trans}+T_{pitch}+T_{inertia}-C+|M_{drum}|-sT_F+T_g+c_r\dot\phi,
\qquad M_{reaction}=-sM_{mag}.
$$

`Mmag` 是原变量名，并不意味着数学上始终非负。`Mreaction` 保留符号；油缸负载输出取其幅值。

油缸几何与力矩转换：

$$
\ell=\max(L_0+x,10^{-3}),\qquad
c=\operatorname{clip}\left(\frac{L_2^2+\ell^2-L_1^2}{2L_2\ell},-1,1\right),
$$
$$
h=\max\left(L_2\sqrt{\max(1-c^2,0)},h_{min}\right),\qquad
F_{cyl}=\frac{|M_{reaction}|}{h}+c_{cyl}|v_{cyl}|.
$$

`xun/vun` 是油缸位移与速度，`q/qdot/qddot` 是摇臂运动。二者都由外部输入；该函数不检查其运动学一致性。
`Fcyl` 是幅值，不能直接当作任意坐标系下有符号的液压缸外力。

## 4. 整机受力计算

### 4.1 运动方向

$$
d=\begin{cases}
\operatorname{sign}((v_L+v_R)/2),&|(v_L+v_R)/2|>10^{-4},\\
\operatorname{sign}(T_{cmdL}+T_{cmdR}),&|T_{cmdL}+T_{cmdR}|>1,\\
0,&\text{其他情况}.
\end{cases}
$$

`Tcmd` 输入在本模型中仅用于近零速方向判断，不直接决定牵引力大小。左右速度相反且平均为零时也执行这条规则。

### 4.2 摇臂质心相对加速度

$$
\begin{aligned}
\phi_L&=q_L-\theta,&\phi_R&=-q_R+\theta,\\
\dot\phi_L&=\dot q_L-\dot\psi,&\dot\phi_R&=-\dot q_R+\dot\psi,\\
\ddot\phi_L&=\ddot q_L-\ddot\psi,&\ddot\phi_R&=-\ddot q_R+\ddot\psi.
\end{aligned}
$$
$$
\begin{aligned}
a_{xL}^{rel}&=K(\cos\phi_L\dot\phi_L^2+\sin\phi_L\ddot\phi_L),\\
a_{xR}^{rel}&=-K(\cos\phi_R\dot\phi_R^2+\sin\phi_R\ddot\phi_R),\\
a_{zL}^{rel}&=K(-\sin\phi_L\dot\phi_L^2+\cos\phi_L\ddot\phi_L),\\
a_{zR}^{rel}&=K(\sin\phi_R\dot\phi_R^2-\cos\phi_R\ddot\phi_R).
\end{aligned}
$$

对应 `xddRelL/R`、`zddRelL/R`。

### 4.3 总支撑力与纵向基础需求

原式将滚筒法向力转换为左侧向上、右侧向下：

$$F_{zL}=|F_{zL,in}|,\qquad F_{zR}=-|F_{zR,in}|.$$
$$N_\Sigma=\max\{M(g\cos\theta+a_n)+m(a_{zL}^{rel}+a_{zR}^{rel})-F_{zL}-F_{zR},0\}.$$
$$b_i=\tfrac12Ma_x+ma_{xi}^{rel}+\tfrac12Mg\sin\theta+d|F_{xi,in}|,\quad i=L,R.$$

对应 `Ntotal` 和 `baseL/R`。代码有意对 Fx/Fz 输入取绝对值；带符号的外部载荷会因此失去原符号。

### 4.4 作用在机身上的力矩

$$
F_{xL}=-d|F_{xL,in}|,\quad F_{xR}=-d|F_{xR,in}|,
$$
$$
M_{cut}=Q(F_{xL}+F_{xR})+P_0(F_{zL}-F_{zR}),\quad
M_h=M_{reactionL}+M_{reactionR},\quad M_{target}=I_{pitch}\ddot\psi.
$$

`McutAtPivots` 只使用铰点位置；摇臂长度引起的力矩已在 `Mreaction` 中，不能再重复相加。

### 4.5 支撑力—摩擦耦合的固定 12 次迭代

初始化 NL=NR=NΣ/2。每次执行：

$$
f_i=d\mu\max(N_i,0),\quad F_{req,i}=b_i+f_i,\quad
F_{trac,i}=\operatorname{clip}(F_{req,i},-F_{lim},F_{lim}),
$$
$$
M_t=R_h(F_{trac,L}+F_{trac,R})-R_s(f_L+f_R),\qquad
\Delta N=\frac{2(M_{target}-M_t-M_{cut}-M_h)}{B},
$$
$$N_L^{raw}=(N_\Sigma-\Delta N)/2,\qquad N_R^{raw}=(N_\Sigma+\Delta N)/2.$$

若 NΣ≤0，左右支撑力都置零；若某侧原始支撑力为负，将该侧置零，另一侧承受 NΣ；否则使用原始值。
12 次后，用最终 NL/NR **再次计算**摩擦、需求牵引力、限幅牵引力和牵引力矩。这一步也保留在 C 中。

### 4.6 输出

$$
T_{load,i}=r_wF_{trac,i},\qquad margin_i=F_{lim}-|F_{trac,i}|,
$$
$$
r_x=(F_{trac,L}+F_{trac,R})-(F_{req,L}+F_{req,R}),
$$
$$
r_M=\tfrac12B(N_R-N_L)+M_t+M_{cut}+M_h-M_{target}.
$$

对应 `Tload`、`margin`、`longitudinalResidual`、`pitchResidual`。
margin 使用限幅后的牵引力，因此超载时只会到零，不会变负；要发现需求无法满足，应同时观察纵向残差。
支撑力截断或牵引饱和时平衡残差可能不为零；12 次迭代不是任意工况收敛的保证。

机身俯仰角、左右摇臂角输出直接复制输入，**不是这组方程计算出的运动响应**。
