# Abstract
## Authors: Yi Wang, Bingxian Mu, Oren Salzman
We introduces Bidirectional Lazy Informed Trees (BLIT*), the first algorithm to incorporate anytime incremental lazy bidirectional heuristic search (Bi-HS) into batch-wise sampling-based motion planning (Bw-SBMP). BLIT* operates on batches of informed states (states that can potentially improve the cost of the incumbent solution) structured as an implicit random geometric graph (RGG). The computational cost of collision detection is mitigated via {\em a new lazy edge-evaluation strategy} by focusing on states near obstacles. Experimental results, especially in high dimensions, show that BLIT* outperforms existing Bw-SBMP planners by efficiently finding an initial solution and effectively improving the quality as more computational resources are available.

<img width="1226" alt="BLIT*" src="https://github.com/user-attachments/assets/07d3e9e8-f574-42e0-9fe2-c8c1e9254724" />

🚀 **[BLIT* has been accepted to the 42th IEEE International Conference on Robotics & Automation (ICRA 2025)](https://2025.ieee-icra.org)!** We will release the C++ implementation for comparison-version after mid-June, followed by publishing the OMPL-compatible version to the official OMPL repository.
### Dr. Eyal Weiss is a co-author in our Journal version
