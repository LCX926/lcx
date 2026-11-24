# MG1000 整机与左右摇臂逆动力学 C 模块

将 `shearer_machine_dynamics_complete_MG1000.slx` 的三个 MATLAB Function
及顶层端口接线手工移植为 C99，保留原式、绝对值、左右符号、几何保护、牵引限幅和 12 次迭代。

这是**给定运动求受力**的无状态计算模块，不积分机身或摇臂运动；不包含电机、齿轮、截煤载荷发生器或液压求解器。

## 文件

- `include/mg1000.h`：26 输入、23 参数、17 输出的命名编号和状态码。
- `src/mg1000.c`：左右摇臂 `arm()`、整机 `machine()`、总入口 `mg1000_step()`。
- `examples/demo.c`：完整可编译调用例。
- `docs/equations.md`：方程、符号、参数及原模型逐项对应。
- `reference/chart_38.m`、`chart_61.m`、`chart_99.m`：直接从 SLX XML 提取的原函数。
- `reference/parameters.json`：从原模型常量块提取的参数。
- `reference/ports.json`：从原模型提取的输入/输出端口顺序。
- `tests/verify.py`：编译与数值对照、解析静力学及边界测试。
- `docs/review.md`：审查结论、验证范围及继承的模型限制。

## 编译和使用

```sh
cc -std=c99 -O2 -Wall -Wextra -Werror -pedantic -Iinclude \
  src/mg1000.c examples/demo.c -lm -o demo
./demo
python tests/verify.py --cc gcc
```

Windows 可使用 GCC/Clang 或 `python tests/verify.py --cc 'path/to/zig.exe cc'`。
测试只需要 Python 标准库和 C99 编译器；运行时只需要 C 数学库。

```c
double parameters[MG1000_PARAM_COUNT];
double input[MG1000_INPUT_COUNT] = {0};
double output[MG1000_OUTPUT_COUNT];
mg1000_default_parameters(parameters);
input[MG_IN_VL] = 0.05; /* m/s, not m/min */
input[MG_IN_VR] = 0.05;
Mg1000Status status = mg1000_step(input, parameters, output);
/* Use output only when status == MG1000_OK. */
```

所有角度使用 rad；力 N、转矩 N·m、长度 m、时间 s。数组下标从 0 开始，参数 `p[k-1]` 对应 MATLAB 的 `P(k)`。
输入、输出数组长度由调用方保证；C 指针接口不能检查缓冲区长度。

## 嵌入式调用约定

每个仿真周期提供同一时刻的输入并调用一次。原模型整机步长为 1 ms，但本模块没有内部时钟，调用周期由外部调度决定。
没有堆分配、文件操作、全局可变状态或随机数；可重入。使用 double 保持与原模型双精度计算的对照基础。
只需集成 `.h` 和 `.c`；示例中的 `printf` 不在模型内核中。
禁止 `-ffast-math`，因为接口使用 `isfinite` 检查非有限输入/输出。

`MG1000_BAD_ARGUMENT` 表示空指针或非有限输入；`MG1000_BAD_PARAMETER` 表示非法参数；
`MG1000_NUMERIC_ERROR` 表示数值异常。错误时输出保持调用前的内容，调用方必须检查状态码，不能把旧值视为新结果。
参数要求全部有限且非负，支撑跨度、L2、最小力臂必须严格为正；这比原 MATLAB 函数增加了显式输入保护。
几何越界仍按原模型截断，不作为错误：因此 `OK` 只说明数值计算成功，不代表工况物理可行。

## 来源与验证边界

源 SLX SHA-256：`319896723965A4724CC3C8C9E7E2172D860E81963BECB0DD584B0FFB0A69C5A7`。
原 SLX 和 AMESim 大文件未纳入本代码包。
测试将归档 MATLAB 源函数中的标量表达式和分支转换为 Python 求值，与编译后的 C 逐输出比较。
该过程验证移植一致性，不等同于 MATLAB/Simulink 实际运行对照或采煤机实测验证。
嵌入式交叉编译、目标板最坏执行时间和闭环联合仿真尚需在选定硬件后进行。
