/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, University of New Hampshire
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the names of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Authors: Yi Wang, Eyal Weiss, Bingxian Mu, Oren Salzman

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_

#include <algorithm>
#include <memory>
#include <unordered_map>
#include "linQuad.h"
#include "ompl/base/Planner.h"
#include <unsupported/Eigen/Polynomials>
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/lazyinformedtrees/blitstar/ImplicitGraph.h"
#include "ompl/geometric/planners/lazyinformedtrees/blitstar/Vertex.h"
#include "ompl/geometric/planners/lazyinformedtrees/blitstar/Queuetypes.h"

#include <chrono>
using namespace std::chrono;
namespace ompl
{
    namespace geometric
    {
        /**
        @anchor gblitstar

        \ref blitstar "BLIT*" (Bidirectional Lazy Informed Trees) is an almost-surely asymptotically optimal motion planner.
   
        */

        class BLITstar : public ompl::base::Planner
        {
        public:
            /** \brief Constructs a BLIT*. */
            explicit BLITstar(const ompl::base::SpaceInformationPtr &spaceInformation);

            /** \brief Destructs a BLIT*. */
            ~BLITstar();

            /** \brief Additional setup that can only be done once a problem definition is set. */
            void setup() override;
            void runTime();  
            /** \brief Checks whether the planner is successfully setup. */
            ompl::base::PlannerStatus::StatusType ensureSetup();

            /** \brief Checks whether the problem is successfully setup. */
            ompl::base::PlannerStatus::StatusType
            ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition);

            /** \brief Clears the algorithm's internal state. */
            void clear() override;

            /** \brief Solves a motion planning problem. */
            ompl::base::PlannerStatus
            solve(const ompl::base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Get the cost of the incumbent solution. */
            ompl::base::Cost bestCost() const;
            ompl::base::Cost costCombine(ompl::base::Cost c1, ompl::base::Cost c2);
            /** \brief Get the planner data. */
            void getPlannerData(base::PlannerData &data) const override;
	    
            /** \brief Set the batch size. */
            void setBatchSize(std::size_t batchSize);

            /** \brief Get the batch size. */
            std::size_t getBatchSize() const;

            /** \brief Set the rewire factor of the RGG graph. */
            void setRewireFactor(double rewireFactor);
            bool isValid(const blitstar::keyEdgePair &edge, double arriveTime_);
            bool couldBeValid(const blitstar::keyEdgePair &edge, double arriveTime_); 
            /** \brief Get the rewire factor of the RGG graph. */
            double getRewireFactor() const;
            std::vector<double> timeSlice(double time_, int num); 
            /** \brief Set whether to track approximate solutions. */
            void trackApproximateSolutions(bool track);

            /** \brief Get whether approximate solutions are tracked. */
            bool areApproximateSolutionsTracked() const;

            /** \brief Set whether pruning is enabled or not. */
            void enablePruning(bool prune);

            /** \brief Get whether pruning is enabled or not. */
            bool isPruningEnabled() const;

            /** \brief Set whether to use a k-nearest RGG connection model. If false, AIT* uses an r-disc model. */
            void setUseKNearest(bool useKNearest);

            /** \brief Get whether to use a k-nearest RGG connection model. If false, AIT* uses an r-disc model. */
            bool getUseKNearest() const;

            /** \brief Set the maximum number of goals AIT* will sample from sampleable goal regions. */
            void setMaxNumberOfGoals(unsigned int numberOfGoals);

            /** \brief Get the maximum number of goals AIT* will sample from sampleable goal regions. */
            unsigned int getMaxNumberOfGoals() const;
       
            /**     */
            void clearReverseVertexQueue();
            void clearForwardVertexQueue();
            
            void resetReverseValue(auto &vertex);
            void resetForwardValue(auto &vertex); 
            
            /** \brief meet in middle*/
            bool terminateSearch(); 
            
            void insertGoalVerticesInReverseVertexQueue();  
            void insertStartVerticesInForWardVertexQueue();
            
            void resetForwardParentInformation(auto & vertex);   
            void resetReverseParentInformation(auto & vertex); 
            
            /** \brief get the priority C for a vertex*/
            bool SelectExpandState(bool & forward);      
            
            bool PathValidity(std::shared_ptr<blitstar::Vertex> &vertex); 
            
            void printForwardStatement(const std::shared_ptr<blitstar::Vertex> &vertex);
            void printReverseStatement(const std::shared_ptr<blitstar::Vertex> &vertex);

            void resetForwardParentAndRemeberTheVertex(auto &child, auto &parent);
            void resetReverseParentAndRemeberTheVertex(auto &child, auto &parent);
             
             
            void lookingForBestNeighbor(ompl::base::Cost curMin_, size_t neighbor); 
            
            void ForwardLazySearch(const std::shared_ptr<blitstar::Vertex> &vertex);// forward preliminary motion tree
            void ReverseLazySearch(const std::shared_ptr<blitstar::Vertex> &vertex);// backward preliminary motion tree
           
            void ForwardPathValidityChecking(std::shared_ptr<blitstar::Vertex> &vertex, bool &validity);
            void ReversePathValidityChecking(std::shared_ptr<blitstar::Vertex> &vertex, bool &validity);
            void bestNeighbor(ompl::base::Cost costToCome, ompl::base::Cost costToGoal, size_t neighbor);
            
            
            bool isValidAtResolution(const blitstar::keyEdgePair &edge, std::size_t numChecks, bool sparseCheck, double arriveTime_);
            void updateReverseCost(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo);
            void updateForwardCost(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo);
                               

            void EvaluateValidityStartAndToGoal(const std::shared_ptr<blitstar::Vertex> &start, const std::shared_ptr<blitstar::Vertex> &goal);
            
            /** \brief Inserts or updates a vertex in the reverse queue. */
            void insertOrUpdateInForwardVertexQueue(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal, bool couldMeet);
            void insertOrUpdateInReverseVertexQueue(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal, bool couldMeet);
            
            void updateCostToGo(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost &costToCome, ompl::base::Cost &costToGo, ompl::base::Cost costFromOriginal,bool meetOnTree); 
            
            void updateBestSolutionFoundSoFar(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost meetCost, ompl::base::Cost costToCome, ompl::base::Cost &costToGo, ompl::base::Cost costFromOri);  


        private:
            /** \brief Performs one iteration of AIT*. */
            void iterate(const ompl::base::PlannerTerminationCondition &terminationCondition);
            ompl::base::SpaceInformationPtr spaceInformation_;

            ompl::base::State *detectionState_; 
            

            /** \brief Prints a message using OMPL_INFORM to let the user know that AIT* found a new solution. */
            void informAboutNewSolution() const;
            /** \brief Prints a message using OMPL_INFORM to let the user know of the planner status. */
            void informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const;

            /** \brief Inserts the goal vertices of the graph into the reverse search queue. */
            void insertGoalVerticesInReverseQueue();


            /** \brief Returns the path a start to the argument. */
            std::shared_ptr<ompl::geometric::PathGeometric>
            getPathToVertex(const std::shared_ptr<blitstar::Vertex> &vertex) const;

            /** \brief Computes the sort key of an edge. */
            std::array<ompl::base::Cost, 3u> computeEstimatedPathCosts(ompl::base::Cost CostToStart, ompl::base::Cost CostToGoal, ompl::base::Cost motionCost) const;
     
            std::array<ompl::base::Cost, 3u> computeSortKey(const std::shared_ptr<blitstar::Vertex> &parent,
                                                            const std::shared_ptr<blitstar::Vertex> &child) const;

            /** \brief Computes the sort key of a vertex. */
            std::array<ompl::base::Cost, 2u> computeSortKey(const std::shared_ptr<blitstar::Vertex> &vertex) const;

            std::array<ompl::base::Cost, 2u> computeEstimatedPathCosts(ompl::base::Cost CostToStart, ompl::base::Cost CostToGoal) const;


            void updateExactSolution();
            /** \brief Checks whether the input vertex is the new best approximate solution and updates the solution in
             * the problem definition if so. **/
            void updateApproximateSolution(const std::shared_ptr<blitstar::Vertex> &vertex);
            /** \brief Checks which vertex is the best approximate solution. */
            void updateApproximateSolution();

            /** \brief Updates the exact solution and if AIT* track approximate solutions, it updates it as well. */
            ompl::base::PlannerStatus::StatusType updateSolution();

            /** \brief Updates the exact solution and if AIT* track approximate solutions, it updates it as well. */
            ompl::base::PlannerStatus::StatusType updateSolution(const std::shared_ptr<blitstar::Vertex> &vertex);

            /** \brief Returns the best cost-to-go-heuristic to any start in the graph. */
            ompl::base::Cost computeCostToGoToStartHeuristic(const std::shared_ptr<blitstar::Vertex> &vertex) const;
            /** \brief Returns the best cost-to-go-heuristic to any goal in the graph. */
            ompl::base::Cost computeCostToGoToGoalHeuristic(const std::shared_ptr<blitstar::Vertex> &vertex) const;

            /** \brief Returns the best cost to go to any goal in the graph. */
            ompl::base::Cost computeCostToGoToGoal(const std::shared_ptr<blitstar::Vertex> &vertex) const;

            /** \brief Returns the best cost to come form the goal of any start. */
            ompl::base::Cost computeBestCostToComeFromGoalOfAnyStart() const;

            /** \brief Counts the number of vertices in the forward tree. */
            std::size_t countNumVerticesInForwardTree() const;

            /** \brief Counts the number of vertices in the reverse tree. */
            std::size_t countNumVerticesInReverseTree() const;


            /** \brief The cost of the incumbent solution. */
            ompl::base::Cost solutionCost_;

            /** \brief The cost of the least edge cost for a given RGG. */
            ompl::base::Cost leastEdge_;

            /** \brief The minimum priority value on Open_B and Open_F. */
            ompl::base::Cost PriorityC;

            /** \brief The optimal solution found so far for a give RGG. */          
            ompl::base::Cost C_curr;
            
            /** \brief f,g,h value in both motion tree. */  
            std::array<ompl::base::Cost, 3u> ForwardEValue;
            std::array<ompl::base::Cost, 3u> BackwardEValue;
            std::array<ompl::base::Cost, 2u> ForwardValue;
            std::array<ompl::base::Cost, 2u> BackwardValue;
            /** \brief The cost to come to the vertex that is closest to the goal (in cost space). */
            ompl::base::Cost approximateSolutionCost_{};

            /** \brief The cost to go to the goal from the current best approximate solution. */
            ompl::base::Cost approximateSolutionCostToGoal_{};
            
            ompl::base::Cost fmin_{0u};
            ompl::base::Cost ForwardCost{0u};
            ompl::base::Cost BackwardCost{0u}; 
            ompl::base::Cost boundedValue_{0u};
            ompl::base::Cost minimalneighbor_{0u};
            /** \brief The increasingly dense sampling-based approximation. */
            blitstar::ImplicitGraph graph_;

            /** \brief The forwardVertexQueue queue. */
            blitstar::VertexQueue forwardVertexQueue_;
	    
	        /** \brief The backwardVertexQueue queue. */
            blitstar::VertexQueue reverseVertexQueue_;
            
            /** \biref the best vertex*/
            std::shared_ptr<blitstar::Vertex> BestVertex_;

            std::shared_ptr<ompl::geometric::PathGeometric> path_;

            /** \biref the middler vertex*/
            blitstar::MiddleVertex MMvertex_;  
            /** \brief Lexicographically compares the keys of two vertices. */
            bool isVertexBetter(const blitstar::KeyVertexPair &lhs, const blitstar::KeyVertexPair &rhs) const;

            /** \brief the number of edges in the forward queue with inconsistent target states. */
            std::size_t numInconsistentOrUnconnectedTargets_{0u};

            /** \brief The number of iterations that have been performed. */
            std::size_t numIterations_{0u};
            std::size_t bestNeighbor_{0u};
            std::size_t numSparseCollisionChecksCurrentLevel_{0u};
            std::size_t forwardSearchVersion_{0u};
            std::size_t reverseSearchVersion_{0u};
            double time_taken{0u};
            /** \brief The number of samples per batch. */
            std::size_t batchSize_{300u};
            /** \brief The option that specifies whether to track approximate solutions. */
            bool trackApproximateSolutions_{true};
            /** \brief The option that specifies whether to prune the graph of useless samples. */
            bool isPruningEnabled_{false};
            bool increProcess_{false};
            bool meeting_{false};
            bool start_scratch_{false};
            void showingRunningtime();
            //ompl::base::Cost TrueEdgeCost(const ompl::base::State *state1, const ompl::base::State *state2) const;
            bool betterThan(const ompl::base::Cost & cost1, const ompl::base::Cost & cost2);

            /** \brief addming more samples*/
            bool NeedMoreSamples();
            bool iSolution_{false};  
            bool need_Prune_{false};        
            bool isVertexEmpty_{true};
            bool found_meeting_{false};
            bool find_solution_{false};
            bool forwardInvalid_{false};
            bool reverseInvalid_{false};  
            bool goalCloseToStart_{false};
            bool foundSolution_or_Exhausted_{false};
            high_resolution_clock::time_point  startT,endT;
            /** \brief Syntactic helper to get at the optimization objective of the planner base class. */
            ompl::base::OptimizationObjectivePtr objective_;

            /** \brief Syntactic helper to get at the motion validator of the planner base class. */
            ompl::base::MotionValidatorPtr motionValidator_;

            /** \brief The number of processed edges. */
            std::size_t numProcessedEdges_{0u};
            std::size_t numbatch_{0u};
            std::shared_ptr<ompl::base::StateSpace> space_;
            /** \brief The number of edge collision checks performed. */
            std::size_t numEdgeCollisionChecks_{0u};
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_blitstar
