"""Compile C and compare every output with scalar equations read from source .m.

The limited translator below accepts only the constructs present in the three
archived charts. It does not validate MATLAB/Simulink scheduling or physics.
Run: python tests/verify.py [--cc 'zig cc'] [--cases 10000]
"""
import argparse
import ctypes as ct
import hashlib
import json
import math
import os
from pathlib import Path
import random
import re
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if not __debug__:
    raise RuntimeError("Run tests without Python -O; assertions must be enabled")
PROVENANCE = json.loads((ROOT / "reference/provenance.json").read_text(encoding="utf-8-sig"))
for filename, expected_hash in PROVENANCE["charts"].items():
    actual_hash = hashlib.sha256((ROOT / "reference" / filename).read_bytes()).hexdigest()
    if actual_hash != expected_hash:
        raise RuntimeError("Archived source hash mismatch: " + filename)


def translate_chart(path):
    source = path.read_text(encoding="utf-8")
    header = re.search(r"function\s+\[([^]]+)\]\s*=\s*fcn\(([^)]+)\)", source)
    outputs = header[1].split(",")
    args = header[2].split(",")
    lines = ["def reference(" + ",".join(args) + "):"]
    indent = 1
    body = source[header.end():]
    for raw in body.splitlines():
        line = raw.split("%", 1)[0].strip().rstrip(";")
        if not line:
            continue
        if line == "end":
            indent -= 1
            continue
        if line.startswith("elseif "):
            indent -= 1
            line = "elif " + line[7:] + ":"
        elif line == "else":
            indent -= 1
            line = "else:"
        elif line.startswith("if "):
            line += ":"
        elif line.startswith("for "):
            match = re.fullmatch(r"for (\w+) = (\d+):(\d+)", line)
            if not match:
                raise ValueError("unsupported loop: " + line)
            line = f"for {match[1]} in range({match[2]}, {int(match[3])+1}):"
        line = re.sub(r"P\((\d+)\)", lambda m: f"P[{int(m[1])-1}]", line)
        lines.append("    "*indent + line)
        if line.endswith(":"):
            indent += 1
    if indent != 0:
        raise ValueError("unbalanced source blocks")
    lines.append("    return " + ",".join(outputs))
    namespace = {"sin": math.sin, "cos": math.cos, "sqrt": math.sqrt,
                 "abs": abs, "min": min, "max": max,
                 "sign": lambda x: (x > 0)-(x < 0), "range": range}
    exec(compile("\n".join(lines), str(path), "exec"), namespace)
    return namespace["reference"]


LEFT = translate_chart(ROOT / "reference/chart_38.m")
RIGHT = translate_chart(ROOT / "reference/chart_99.m")
MACHINE = translate_chart(ROOT / "reference/chart_61.m")


def oracle(u, p):
    fl, ml = LEFT(*u[8:13], u[4], u[5], u[24], *u[19:22],
                  u[18], u[22], u[23], -1, p)
    fr, mr = RIGHT(*u[13:18], u[6], u[7], u[25], *u[19:22],
                   u[18], u[22], u[23], 1, p)
    tL, tR, fL, fR, nL, nR, mL, mR, pitch, longitudinal = MACHINE(
        *u[:8], *u[10:13], *u[15:18], u[18], *u[19:24], ml, mr, p)
    return [tL, tR, fL, fR, fl, fr, u[19], u[10], u[15], nL, nR,
            ml, mr, mL, mR, pitch, longitudinal]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default=os.environ.get("CC", "cc"))
    parser.add_argument("--cases", type=int, default=10000)
    opts = parser.parse_args()
    build = ROOT / "build"
    build.mkdir(exist_ok=True)
    # Keep Zig's first-run compiler cache inside the project workspace.
    os.environ.setdefault("ZIG_GLOBAL_CACHE_DIR", str(build / "zig-global"))
    os.environ.setdefault("ZIG_LOCAL_CACHE_DIR", str(build / "zig-local"))
    compiler = shlex.split(opts.cc, posix=(os.name != "nt"))
    library = build / ("mg1000.dll" if os.name == "nt" else "libmg1000.so")
    common = ["-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
              "-Iinclude", "src/mg1000.c"]
    cmd = compiler + common + ["-shared"]
    if os.name == "nt":
        cmd += ["-Wl,--export-all-symbols"]
    else:
        cmd += ["-fPIC"]
    cmd += ["-o", str(library), "-lm"]
    subprocess.run(cmd, cwd=ROOT, check=True)
    executable = build / ("demo.exe" if os.name == "nt" else "demo")
    subprocess.run(compiler + common + ["examples/demo.c", "-o", str(executable), "-lm"],
                   cwd=ROOT, check=True)
    subprocess.run([str(executable)], check=True)
    lib = ct.CDLL(str(library))
    pointer = ct.POINTER(ct.c_double)
    lib.mg1000_step.argtypes = [pointer, pointer, pointer]
    lib.mg1000_step.restype = ct.c_int
    lib.mg1000_default_parameters.argtypes = [pointer]
    lib.mg1000_default_parameters.restype = None
    A, P, Y = ct.c_double*26, ct.c_double*23, ct.c_double*17
    defaults = P()
    lib.mg1000_default_parameters(defaults)
    source_p = json.loads((ROOT / "reference/parameters.json").read_text(encoding="utf-8-sig"))
    assert list(defaults) == source_p
    rng = random.Random(20261124)
    cases = []
    for speed, torque in [(0, 0), (0, 0.5), (0, 0.500001), (0, -1),
                          (1e-4, 0), (1.00001e-4, 0), (-1.00001e-4, 0),
                          (0.1, 0), (-0.1, 0)]:
        u = [0.0]*26
        u[:4] = [speed, speed, torque, torque]
        cases.append((u, source_p[:]))
    for x, an, acc in [(-2, 0, 0), (10, 0, 0), (0, -20, 0),
                        (0, 0, 100), (0, 0, -100)]:
        u = [0.0]*26
        u[8] = u[13] = x
        u[22], u[21] = an, acc
        cases.append((u, source_p[:]))
    for _ in range(opts.cases):
        u = [rng.uniform(-0.25, 0.25) for _ in range(26)]
        u[2:4] = [rng.uniform(-2e5, 2e5) for _ in range(2)]
        u[4:8] = [rng.uniform(-1.5e6, 1.5e6) for _ in range(4)]
        u[8], u[13] = rng.uniform(-2, 3), rng.uniform(-2, 3)
        u[10], u[15] = rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)
        u[18], u[22] = rng.uniform(-15, 15), rng.uniform(-15, 15)
        u[21] = rng.uniform(-10, 10)
        u[24:26] = [rng.uniform(-1e6, 1e6) for _ in range(2)]
        p = [value*rng.uniform(0.5, 1.5) if value else 0.0 for value in source_p]
        p[20], p[21] = rng.uniform(0, 1e4), rng.uniform(0, 1e4)
        cases.append((u, p))
    # Additional normal-operation range, separate from overload stress cases.
    for _ in range(1000):
        u = [0.0]*26
        u[0] = u[1] = rng.uniform(-0.2, 0.2)
        u[4:8] = [rng.uniform(0, 100000) for _ in range(4)]
        u[8] = u[13] = rng.uniform(0, 1.0)
        u[10], u[15] = rng.uniform(-0.3, 0.5), rng.uniform(-0.5, 0.3)
        u[11:13] = [rng.uniform(-0.02, 0.02) for _ in range(2)]
        u[16:18] = [rng.uniform(-0.02, 0.02) for _ in range(2)]
        u[18:24] = [rng.uniform(-0.02, 0.02) for _ in range(6)]
        u[24:26] = [rng.uniform(0, 200000) for _ in range(2)]
        cases.append((u, source_p[:]))
    max_abs = [0.0]*17
    max_scaled = 0.0
    branch_counts = {"traction_saturated": 0, "support_lifted": 0,
                     "normal_zero": 0}
    for u, p in cases:
        result = Y()
        status = lib.mg1000_step(A(*u), P(*p), result)
        assert status == 0, status
        expected = oracle(u, p)
        for i, (actual, ref) in enumerate(zip(result, expected)):
            error = abs(actual-ref)
            max_abs[i] = max(max_abs[i], error)
            max_scaled = max(max_scaled, error/(1+abs(ref)))
            assert math.isclose(actual, ref, rel_tol=2e-12, abs_tol=1e-6), (i, actual, ref)
        assert result[4] >= 0 and result[5] >= 0
        assert result[9] >= 0 and result[10] >= 0
        assert abs(result[2]) <= p[12] and abs(result[3]) <= p[12]
        assert result[13] >= 0 and result[14] >= 0
        assert result[6:9] == [u[19], u[10], u[15]]
        assert math.isclose(result[0], p[11]*result[2])
        branch_counts["traction_saturated"] += int(result[13] == 0 or result[14] == 0)
        branch_counts["support_lifted"] += int(result[9] == 0 or result[10] == 0)
        branch_counts["normal_zero"] += int(result[9] + result[10] == 0)
    # Independent analytical equilibrium: horizontal, still, no cutting load.
    y = Y()
    assert lib.mg1000_step(A(), defaults, y) == 0
    assert math.isclose(y[9], source_p[0]*source_p[19]/2)
    assert y[0] == y[1] == y[15] == y[16] == 0
    assert math.isclose(y[11], source_p[3]*source_p[5]*source_p[19])
    assert y[11] == -y[12] and y[4] == y[5]
    # Signed force inputs are converted to magnitudes in the source charts.
    u, p = cases[-1]
    flipped = u[:]
    for i in [4, 5, 6, 7, 24, 25]: flipped[i] = -flipped[i]
    a, b = Y(), Y()
    assert lib.mg1000_step(A(*u), P(*p), a) == 0
    assert lib.mg1000_step(A(*flipped), P(*p), b) == 0
    assert list(a) == list(b)
    # Errors leave all outputs unchanged.
    for location in range(26):
        for bad in [math.nan, math.inf, -math.inf]:
            u = A(); u[location] = bad; y = Y(*([123.0]*17))
            assert lib.mg1000_step(u, defaults, y) == 1
            assert list(y) == [123.0]*17
    for location in range(23):
        for bad in [math.nan, math.inf, -1.0]:
            p = P(*source_p); p[location] = bad; y = Y(*([123.0]*17))
            assert lib.mg1000_step(A(), p, y) == 2
            assert list(y) == [123.0]*17
    for location in [7, 17, 18]:
        p = P(*source_p); p[location] = 0
        assert lib.mg1000_step(A(), p, Y()) == 2
    for args in [(None, defaults, Y()), (A(), None, Y()), (A(), defaults, None)]:
        assert lib.mg1000_step(*args) == 1
    u = A(); u[12] = sys.float_info.max; y = Y(*([123.0]*17))
    assert lib.mg1000_step(u, defaults, y) == 3
    assert list(y) == [123.0]*17
    for location in [16, 17]:
        p = P(*source_p); p[location] = sys.float_info.max
        y = Y(*([123.0]*17))
        assert lib.mg1000_step(A(), p, y) == 3
        assert list(y) == [123.0]*17
    # Aliasing input/output is supported by the temporary result buffer.
    u, p = cases[-1]; shared = A(*u)
    assert lib.mg1000_step(shared, P(*p), shared) == 0
    assert all(math.isclose(x, z, rel_tol=2e-12, abs_tol=1e-6)
               for x, z in zip(shared[:17], oracle(u, p)))
    report = {"cases_compared": len(cases), "seed": 20261124,
              "max_absolute_error_per_output": max_abs,
              "max_error_over_1_plus_abs_reference": max_scaled,
              "branch_cases": branch_counts,
              "compiler": compiler,
              "compiler_version": subprocess.check_output(compiler+["--version"], text=True).splitlines()[0],
              "c_sha256": hashlib.sha256((ROOT/"src/mg1000.c").read_bytes()).hexdigest(),
              "scope": "C vs scalar source-equation translation; no MATLAB/Simulink execution or target MCU timing"}
    (build / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
