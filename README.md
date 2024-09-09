
# Kino-PAX  
Kinodynamic Parallel Accelerated eXpansion

Sampling-based motion planners (SBMPs) are effective for planning with complex kinodynamic constraints in high-dimensional spaces. However, they still struggle to achieve real-time performance, primarily due to their serial computation design.

We present Kinodynamic Parallel Accelerated eXpansion (Kino-PAX), a novel, highly parallel kinodynamic SBMP designed for parallel devices such as GPUs. Kino-PAX grows a tree of trajectory segments in parallel.

Our key insight is in decomposing the iterative tree growth process into three massively parallel subroutines. Kino-PAX is designed to align with the parallel device execution hierarchies, ensuring that:
- Threads are largely independent
- Workloads are evenly distributed
- Low-latency resources are utilized while minimizing high-latency data transfers and process synchronization

This design results in a highly efficient GPU implementation. We prove that Kino-PAX is probabilistically complete and analyze its scalability as compute hardware improves.

Empirical evaluations demonstrate solutions in the order of 10 ms on a desktop GPU and in the order of 100 ms on an embedded GPU, representing up to a 1000× improvement compared to coarse-grained CPU parallelization of state-of-the-art sequential algorithms over a range of complex environments and systems.

## Disclaimers

The code in this repository is "research-grade", meaning it is largely uncommented and may not be immediately useful without modification. It serves primarily as a reference for future CUDA-related projects.

## What is included

Three example executables are included: `benchMain`, `executionTimeMain`, and `main`. 

- `benchMain` runs N iterations of KPAX and collects data on tree growth, region exploration, and all nodes in the tree. It uses a `planBench` version of KPAX that is **NOT INTENDED FOR RUNTIME PERFORMANCE.**
  
- `executionTimeMain` runs N iterations of the lightweight version of KPAX, where only the solution trajectory is sent back to the CPU. This version of KPAX is used to collect runtime data.
  
- `main` runs a single iteration of `planBench` and is used to collect data for plotting.

To manage the dynamic model used, edit the `config` file.

## Contact

If you are interested in the code and would like to discuss further, please email me at nicolas.perrault@colorado.edu.