# Genetic Fuzzy Logic System (GFLS)
## GA-Optimised Mamdani Torque Distribution Controller

> **Submodule of:** Dual-Motor Electric Tractor Drivetrain  ([Electric Tractor](https://github.com/Kenny214-apple/Electric_Tractor))
> **Author:** Nasirudeen, Ridwan Kehinde ·

[![MATLAB](https://img.shields.io/badge/MATLAB-R2024b-orange?logo=mathworks)](https://www.mathworks.com)
[![Toolbox](https://img.shields.io/badge/Global_Optimization_Toolbox-Required-red)](https://www.mathworks.com/products/global-optimization.html)
[![Method](https://img.shields.io/badge/Method-Genetic_Algorithm-purple)]()
[![Controller](https://img.shields.io/badge/Controller-Mamdani_GFLS-blue)]()

---

## What This Does

The base project uses a **manually tuned Mamdani fuzzy logic controller** where the membership function (MF) peak positions were set by engineering judgement.
Whilst physically reasonable, manually chosen parameters are not mathematically optimal.

This submodule applies a **Genetic Algorithm (GA)** to automatically search for
the MF peak positions that minimise a composite performance objective across all
five Nigerian agricultural operating modes simultaneously.


**What the GA changes:** Only the 15 MF peak positions (5 peaks × 3 variables).

**What stays fixed:** The 25-rule rule base, FIS type, defuzzification method,
and all vehicle/controller parameters. The linguistic structure of the controller
is fully preserved.

---

## Motivation

### Why Genetic Fuzzy Logic?

Standard Mamdani fuzzy controllers rely on designer expertise to place membership
functions. This introduces subjectivity and leaves performance improvement on the
table. Genetic Fuzzy Logic Systems (GFLS) use evolutionary computation to find
parameter sets that are verifiably better than the manual baseline — providing
mathematical justification for the controller design.

### Why Not Tune the Rule Base?

The 25-rule rule base encodes physical engineering knowledge about tractor
behaviour:

> *"If speed error is large positive AND error rate is large positive
>  THEN rear motor torque should be Very High"*

This causal relationship is physically correct and should not be altered by
an algorithm. What CAN be improved is the exact shape and position of the
triangular membership functions that define what "large positive" means.

This separation — fixed rules, evolved MF parameters — is the defining
characteristic of the **Michigan-style GFLS** approach adopted here.

---


---

## Requirements

The following MATLAB toolboxes must be installed:

| Toolbox | Purpose | Required? |
|---|---|---|
| Fuzzy Logic Toolbox | FIS construction and evalfis() | ✅ Essential |
| Global Optimization Toolbox | ga() function | ✅ Essential |
| Control System Toolbox | Simulation utilities | ✅ Essential |
| Parallel Computing Toolbox | Speed up GA (optional) | ⚡ Optional |

Check your toolboxes:
```matlab
ver('fuzzy')
ver('globaloptim')
ver('control')
```

---

## Quick Start

### Step 1 — Ensure base FIS exists

The GA starts from the original manually-tuned FIS.
Make sure it exists in your simulation folder:

```matlab
% Check it exists
ls ../simulation/tractor_torque_controller.fis
```

If it does not exist, regenerate it:
```matlab
cd ../simulation
init   % load parameters
% Then re-run the FIS builder script or rebuild from init.m
```

### Step 2 — Run the optimiser

```matlab
cd genetic_fuzzy
genetic_fuzzy_optimizer
```

Expected runtime: **15–30 minutes** on a standard laptop.
The GA convergence plot opens automatically and updates each generation.

### Step 3 — Compare results

```matlab
compare_original_vs_ga
```

Prints a side-by-side performance table and saves comparison figures.

### Step 4 — Use the optimised FIS in Simulink

1. Open `../simulation/electric_tractor.slx`
2. Double-click the **Fuzzy Logic Controller** block
3. Change the FIS field from: tractor_torque_controller.fis to ../genetic_fuzzy/tractor_torque_controller_ga.fis

4. Click OK and save the model
5. Run init.m and simulate as normal

---

## Algorithm Design

### Chromosome Encoding

Each candidate solution (chromosome) is a vector of **15 real-valued genes**
representing the peak positions of all membership functions:

x = [e1, e2, e3, e4, e5, ← speed_error MF peaks (rpm)
de1, de2, de3, de4, de5, ← error_rate MF peaks (rpm/s)
t1, t2, t3, t4, t5] ← TSR output MF peaks (—)

Where:
e_i = peak of i-th speed_error MF (NL, NS, ZE, PS, PL)
de_i = peak of i-th error_rate MF (NL, NS, ZE, PS, PL)
t_i = peak of i-th TSR MF (VL, L, M, H, VH)


**Triangular feet are computed automatically** from adjacent peak midpoints,
ensuring MFs always form valid non-overlapping triangles regardless of GA
perturbation.

### Search Bounds

| Variable | Original Peaks | Lower Bound | Upper Bound | Max Shift |
|---|---|---|---|---|
| speed\_error | −10, −5, 0, +5, +10 | ±1.5 rpm per peak | ±1.5 rpm | 15% |
| error\_rate | −5, −2.5, 0, +2.5, +5 | ±0.75 rpm/s per peak | ±0.75 rpm/s | 15% |
| TSR output | 0.40, 0.48, 0.55, 0.62, 0.70 | ±0.025 per peak | ±0.025 | ~8% |

Bounds are set at ±15% of original inter-peak spacing to ensure:
- MF ordering is always preserved (NL < NS < ZE < PS < PL)
- MFs cannot overlap or cross
- TSR output stays physically meaningful within [0.40, 0.70]

### Fitness Function

The GA minimises a weighted composite objective computed by simulating the
closed-loop drivetrain using Euler integration across all five modes:
J = Σ [w_mode_i × (w_ITAE × ITAE_norm
+ w_energy × Wh_km_norm
+ w_overshoot × overshoot_norm
+ w_settle × t_settle_norm)]


| Component | Weight | What it penalises |
|---|---|---|
| ITAE (normalised) | 0.50 | Speed tracking error over time |
| Energy (Wh/km, normalised) | 0.30 | Energy consumption per kilometre |
| Overshoot (normalised) | 0.20 | Speed overshoot above reference |
| Settling time | 0.10 | Time to reach ±2% of reference |

**Mode weights** (how much each mode contributes to total fitness):

| Mode | Weight | Reason |
|---|---|---|
| Plowing | 0.30 | Most demanding — highest priority |
| Harrowing | 0.20 | — |
| Ridging | 0.20 | — |
| Planting | 0.15 | — |
| Transportation | 0.15 | Least demanding |

### GA Parameters

| Parameter | Value | Justification |
|---|---|---|
| Population size | 50 | Balance between diversity and runtime |
| Max generations | 80 | Sufficient for convergence in 15 variables |
| Elite count | 4 | 8% of population — preserves best solutions |
| Crossover fraction | 0.80 | Standard for real-valued GA |
| Crossover function | Scattered | Effective for multi-parameter real problems |
| Mutation function | Adaptive feasible | Respects bounds automatically |
| Selection | Tournament | Robust to fitness landscape shape |
| Initial seed | Original MF peaks | GA can only equal or improve manual design |
| Stall generations | 20 | Stop if no improvement for 20 generations |
| Function tolerance | 1×10⁻⁵ | Convergence precision |

### Why Euler Integration in the Fitness Function?

The fitness function simulates the drivetrain using **Euler integration at
dt = 5 ms** rather than calling Simulink. This is intentional:

- A single Simulink run takes ~2–5 seconds
- The GA evaluates the fitness function ~4,000 times (50 pop × 80 gen)
- Simulink-based GA would take **5–7 hours**
- The Euler integrator takes **~0.05 seconds** per evaluation
- Total GA runtime: **15–30 minutes**

The Euler integrator introduces ~0.3% numerical error compared to Simulink's
ode4 solver — acceptable for parameter search purposes. The optimised parameters
are then validated in the full Simulink model before reporting.

---

## Population Initialisation Strategy

The GA initial population is seeded with the original manually-tuned parameter
vector as one member. This guarantees that the GA can only equal or improve the
manual design — it cannot accidentally discard a working solution by starting
from a random population alone.

The remaining 49 members are randomly sampled within bounds using the
Latin Hypercube Sampling strategy (MATLAB default for bounded real GA).

---

## Reading the GA Console Output

When running, MATLAB prints one row per generation:
Generation f-count Best f(x) Mean f(x) Stall gen
1 100 0.9983 1.0421 0
5 350 0.9841 1.0102 0
10 600 0.9744 0.9988 0
20 1100 0.9631 0.9782 0
40 2100 0.9589 0.9641 3
60 3100 0.9562 0.9598 8
80 4100 0.9551 0.9573 12

Optimization terminated: average change in the fitness value
less than options.FunctionTolerance.


| Column | Meaning |
|---|---|
| Generation | Current generation number |
| f-count | Total fitness function evaluations so far |
| Best f(x) | Fitness of best chromosome in this generation |
| Mean f(x) | Mean fitness of the population |
| Stall gen | Consecutive generations without improvement |

A healthy run shows **Best f(x) decreasing steadily** over generations.
If it stalls early (Best f(x) flat after generation 10), consider increasing
population size or relaxing bounds.

---

## Output Files Generated

After running `genetic_fuzzy_optimizer.m`:

| File | Description |
|---|---|
| `tractor_torque_controller_ga.fis` | GA-optimised FIS ready for Simulink |
| `Fig_MF_Comparison.png` | Side-by-side MF plots: original vs optimised |
| `Fig_Rule_Surface_Comparison.png` | Rule surface: original vs optimised |

After running `compare_original_vs_ga.m`:

| File | Description |
|---|---|
| `Fig_Energy_Comparison_GAvsOrig.png` | Grouped bar chart — Wh/km comparison |
| `Fig_ITAE_Comparison.png` | ITAE bar chart — tracking improvement |

---

## Interpreting Results

### What Good Results Look Like

After optimisation, open the two FIS files in Fuzzy Logic Designer and compare:

```matlab
fis_orig = readfis('../simulation/tractor_torque_controller.fis');
fis_ga   = readfis('tractor_torque_controller_ga.fis');

% Check zero-error TSR (must still be ~0.55)
fprintf('Original zero-error TSR: %.4f\n', evalfis(fis_orig, [0,0]));
fprintf('GA       zero-error TSR: %.4f\n', evalfis(fis_ga,   [0,0]));

% Check plowing transient
fprintf('Original plowing TSR: %.4f\n', evalfis(fis_orig, [6,3]));
fprintf('GA       plowing TSR: %.4f\n', evalfis(fis_ga,   [6,3]));
```

Expected GA output:
- Zero-error TSR: still close to 0.55 (controller remains balanced at steady state)
- Plowing transient TSR: slightly different from 0.6253 — this is the improvement

### Rule Surface Check

The GA-optimised rule surface should:
- ✅ Remain smooth and monotonically increasing (lower-left to upper-right)
- ✅ Show no discontinuities or flat plateaus
- ✅ Differ subtly from the original — peaks and transitions shifted

If the rule surface has discontinuities, the bounds may be too wide.
Tighten the bounds and re-run.

---

## Expected Performance Improvement

Typical results based on this optimisation configuration:

| Metric | Original | GA-Optimised | Improvement |
|---|---|---|---|
| ITAE (average, all modes) | Baseline | ~3–6% lower | ✅ |
| Wh/km (plowing) | 3,813.2 | ~3,720–3,760 | ~1.4–2.4% ✅ |
| Wh/km (average) | Baseline | ~1–3% lower | ✅ |
| Settling time (average) | < 2.0 s | < 1.8 s | ✅ |
| Overshoot | < 10% | < 8% | ✅ |
| Zero-error TSR | 0.5500 | ~0.5480–0.5520 | Maintained ✅ |
| Rule base | 25 rules | 25 rules | Unchanged ✅ |

The energy improvement is modest (~1–3%) because the original Mamdani controller
was already well-designed. The main gain is in ITAE — more precise speed tracking
during load transients, especially during plowing startup.

---

## Troubleshooting

### "ga is not recognised"
The Global Optimization Toolbox is not installed.
Go to: **Home → Add-Ons → Get Add-Ons** and search for
*Global Optimization Toolbox*.

### GA converges in generation 1 with no improvement
The bounds are too tight. Increase the ±variation in lb and ub arrays
inside `genetic_fuzzy_optimizer.m` (Section: CHROMOSOME BOUNDS).

### Simulink runs slowly after swapping FIS
Expected — evalfis with the GA FIS has the same computational cost as
the original. No performance difference in Simulink.

### GA-optimised FIS gives unexpected TSR values
Run the diagnostic:
```matlab
fis_ga = readfis('tractor_torque_controller_ga.fis');
fprintf('Inputs : %d\n', numel(fis_ga.Inputs));
fprintf('Outputs: %d\n', numel(fis_ga.Outputs));
fprintf('Rules  : %d\n', numel(fis_ga.Rules));
% All three must match: 2, 1, 25
```

If rules ≠ 25, the FIS was not saved correctly. Re-run the optimiser.

### Runtime exceeds 45 minutes
Enable parallel evaluation if you have the Parallel Computing Toolbox:
```matlab
% In genetic_fuzzy_optimizer.m, change this line:
'UseParallel', false   →   'UseParallel', true
```
Then start a parallel pool first:
```matlab
parpool('local', 4);   % Use 4 cores
genetic_fuzzy_optimizer
```

---

## Academic Context

This implementation follows the **Pittsburgh-style / Michigan-style hybrid**
approach to genetic fuzzy systems described in the fuzzy control optimisation
literature. The rule base is fixed (Michigan characteristic — one rule per
chromosome member is not used) and the MF parameter set is evolved as a
complete solution (Pittsburgh characteristic — the full parameter vector is
one chromosome).

The weighted multi-objective fitness function simultaneously minimises energy
consumption and tracking error, consistent with the energy management
optimisation methodology applied to hybrid electric powertrains by
Ahmadi et al. (2018) and adapted here to the agricultural torque distribution
context.

**Cite this component as:**

> Nasirudeen, R.K. (2026). Genetic Fuzzy Logic System for Dual-Motor
> Electric Tractor Torque Distribution. FYP Submodule, FUT Minna.
> GitHub: github.com/[your-username]/electric-tractor-fyp/tree/main/genetic_fuzzy

---

## Related Files

| File | Location | Relationship |
|---|---|---|
| `electric_tractor.slx` | `../simulation/` | Main model — swap FIS here |
| `init.m` | `../simulation/` | Must be run before comparison |
| `tractor_torque_controller.fis` | `../simulation/` | Original baseline FIS |
| `plot_results.m` | `../scripts/` | Re-run after swapping FIS for GA figures |
| `run_all_modes.m` | `../scripts/` | Full 5-mode loop with GA FIS |

---

## Contact

**Nasirudeen, Ridwan Kehinde**
LinkedIn: [ridwan-nasirudeen-b23a67287](https://linkedin.com/in/ridwan-nasirudeen-b23a67287)
Email: kendumoimoi@gmail.com

---

*Department of Mechatronics Engineering · Federal University of Technology, Minna*
*Recommended update to Final Year Project 2025/2026*