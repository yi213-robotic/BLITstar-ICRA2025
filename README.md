# Abstract
## Authors: Yi Wang, Bingxian Mu, Oren Salzman
We introduces Bidirectional Lazy Informed Trees (BLIT*), the first algorithm to incorporate anytime incremental lazy bidirectional heuristic search (Bi-HS) into batch-wise sampling-based motion planning (Bw-SBMP). BLIT* operates on batches of informed states (states that can potentially improve the cost of the incumbent solution) structured as an implicit random geometric graph (RGG). The computational cost of collision detection is mitigated via {\em a new lazy edge-evaluation strategy} by focusing on states near obstacles. Experimental results, especially in high dimensions, show that BLIT* outperforms existing Bw-SBMP planners by efficiently finding an initial solution and effectively improving the quality as more computational resources are available.

<img width="1226" alt="BLIT*" src="https://github.com/user-attachments/assets/07d3e9e8-f574-42e0-9fe2-c8c1e9254724" />

:rocket: **[BLIT* has been accepted to the 42th IEEE International Conference on Robotics & Automation (ICRA 2025)](https://2025.ieee-icra.org)!** 

In this repository, we have released C++ implementation for the comparison-version.

The OMPL-compatible version will be released to the official OMPL repository after mid-June.
### [Dr. Eyal Weiss](https://sites.google.com/view/eyal-weiss) is a co-author in our Journal version.

### This tutorial explains how to add the BLIT* planner to OMPL by creating the appropriate directory and copying the source code from this GitHub repository.

If you're already familiar with OMPL, follow these steps:

   (1) cd /yourOwnPath/ompl/src/ompl/geometric/planners/

   (2) mkdir lazyinformedtrees

   (3) cd lazyinformedtrees

   (4) git clone this repos

### :monocle_face: <span style="color:blue">Notably</span>, in contrast to a geometric system, the path cost of a kinodynamic system denotes the cumulative effort or control input required to satisfy dynamical constraints, rather than just the path length.

### When testing the 10D linearized quadrotor, you will need :

(1) For computing edge cost:

        double arrTime_ = estimatedTimeMGLQ(vertex->getState(),neighbor->getState());
        
        auto edgeCost = lqEdgeCostMGLQ(vertex->getState(),neighbor->getState(), arrTime_);

(2) In isValidAtResolution() for checking validity: 
         
         double t = (static_cast<double>(mid) / static_cast<double>(segmentCount))*arriveTime_;
         
         interpolateMGLQ(t, arriveTime_, parent->getState(), child->getState(), detectionState_, false);  

### Please feel free to reach out if you have any questions!
