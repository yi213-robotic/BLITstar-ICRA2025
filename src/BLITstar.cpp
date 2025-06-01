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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"
#include "ompl/geometric/planners/lazyinformedtrees/BLITstar.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <boost/range/adaptor/reversed.hpp>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/util/Console.h"

using namespace std::string_literals;
using namespace ompl::geometric::blitstar;
namespace ob = ompl::base;
using namespace std;
namespace ompl
{
    namespace geometric
    {
        BLITstar::BLITstar(const ompl::base::SpaceInformationPtr &spaceInformation)
          : ompl::base::Planner(spaceInformation, "BLITstar")
          , solutionCost_()
          , graph_(solutionCost_)
          , detectionState_(spaceInformation->allocState())
          , space_(spaceInformation->getStateSpace())
          , forwardVertexQueue_([this](const auto &lhs, const auto &rhs) { return isVertexBetter(lhs, rhs); })
          , reverseVertexQueue_([this](const auto &lhs, const auto &rhs) { return isVertexBetter(lhs, rhs); })
        {
            // Specify BLIT*'s planner specs.
            specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
            specs_.multithreaded = false;
            specs_.approximateSolutions = true;
            specs_.optimizingPaths = true;
            specs_.directed = true;
            specs_.provingSolutionNonExistence = false;
            specs_.canReportIntermediateSolutions = true;
            spaceInformation_ = spaceInformation;
            // Register the setting callbacks.
            declareParam<bool>("use_k_nearest", this, &BLITstar::setUseKNearest, &BLITstar::getUseKNearest, "0,1");
            declareParam<double>("rewire_factor", this, &BLITstar::setRewireFactor, &BLITstar::getRewireFactor,
                                 "1.0:0.01:3.0");
            declareParam<std::size_t>("samples_per_batch", this, &BLITstar::setBatchSize, &BLITstar::getBatchSize,
                                      "1:1:1000");
            declareParam<bool>("use_graph_pruning", this, &BLITstar::enablePruning, &BLITstar::isPruningEnabled, "0,1");
            declareParam<bool>("find_approximate_solutions", this, &BLITstar::trackApproximateSolutions,
                               &BLITstar::areApproximateSolutionsTracked, "0,1");
            declareParam<std::size_t>("set_max_num_goals", this, &BLITstar::setMaxNumberOfGoals,
                                      &BLITstar::getMaxNumberOfGoals, "1:1:1000");

            // Register the progress properties.
            addPlannerProgressProperty("iterations INTEGER", [this]() { return std::to_string(numIterations_); });
            addPlannerProgressProperty("best cost DOUBLE", [this]() { return std::to_string(solutionCost_.value()); });
            addPlannerProgressProperty("state collision checks INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfStateCollisionChecks()); });
            addPlannerProgressProperty("edge collision checks INTEGER",
                                       [this]() { return std::to_string(numEdgeCollisionChecks_); });
            addPlannerProgressProperty("nearest neighbour calls INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfNearestNeighborCalls()); });
        }
        
        BLITstar::~BLITstar()
        {
            si_->freeState(detectionState_);
        }
        
        
        void BLITstar::setup()
        {
            // Call the base-class setup.
            Planner::setup();

            // Check that a problem definition has been set.
            if (static_cast<bool>(Planner::pdef_))
            {
                // Default to path length optimization objective if none has been specified.
                if (!pdef_->hasOptimizationObjective())
                {
                    OMPL_WARN("%s: No optimization objective has been specified. Defaulting to path length.",
                              Planner::getName().c_str());
                    Planner::pdef_->setOptimizationObjective(
                        std::make_shared<ompl::base::PathLengthOptimizationObjective>(Planner::si_));
                }

                if (static_cast<bool>(pdef_->getGoal()))
                {
                    // If we were given a goal, make sure its of appropriate type.
                    if (!(pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION)))
                    {
                        OMPL_ERROR("BMIT* is currently only implemented for goals that can be cast to "
                                   "ompl::base::GOAL_SAMPLEABLE_GOAL_REGION.");
                        setup_ = false;
                        return;
                    }
                }

                // Pull the optimization objective through the problem definition.
                objective_ = pdef_->getOptimizationObjective();

                // Initialize the solution cost to be infinite.
                solutionCost_ = objective_->infiniteCost();
                C_curr = objective_->infiniteCost();
                leastEdge_ = objective_->identityCost();
                boundedValue_= objective_->infiniteCost();
                approximateSolutionCost_ = objective_->infiniteCost();
                approximateSolutionCostToGoal_ = objective_->infiniteCost();

                // Pull the motion validator through the space information.
                motionValidator_ = si_->getMotionValidator();

                // Setup a graph.
                graph_.setup(si_, pdef_, &pis_); 
            }
            else
            {
                // AIT* can't be setup without a problem definition.
                setup_ = false;
                OMPL_WARN("BMIT*: Unable to setup without a problem definition.");
            }
        }

        ompl::base::PlannerStatus::StatusType BLITstar::ensureSetup()
        {
            // Call the base planners validity check. This checks if the
            // planner is setup if not then it calls setup().
            checkValidity();

            // Ensure the planner is setup.
            if (!setup_)
            {
                OMPL_ERROR("%s: The planner is not setup.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            // Ensure the space is setup.
            if (!si_->isSetup())
            {
                OMPL_ERROR("%s: The space information is not setup.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            return ompl::base::PlannerStatus::StatusType::UNKNOWN;
        }

        ompl::base::PlannerStatus::StatusType
        BLITstar::ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // If the graph currently does not have a start state, try to get one.
            if (!graph_.hasAStartState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);

                // If we could not get a start state, then there's nothing to solve.
                if (!graph_.hasAStartState())
                {
                    OMPL_WARN("%s: No solution can be found as no start states are available", name_.c_str());
                    return ompl::base::PlannerStatus::StatusType::INVALID_START;
                }
            }

            // If the graph currently does not have a goal state, we wait until we get one.
            if (!graph_.hasAGoalState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);

                // If the graph still doesn't have a goal after waiting, then there's nothing to solve.
                if (!graph_.hasAGoalState())
                {
                    OMPL_WARN("%s: No solution can be found as no goal states are available", name_.c_str());
                    return ompl::base::PlannerStatus::StatusType::INVALID_GOAL;
                }
            }

            // Would it be worth implementing a 'setup' or 'checked' status type?
            return ompl::base::PlannerStatus::StatusType::UNKNOWN;
        }

        void BLITstar::clear()
        {
            graph_.clear();
            numInconsistentOrUnconnectedTargets_ = numIterations_ = 0u;
            approximateSolutionCostToGoal_ = approximateSolutionCost_ = solutionCost_ = objective_->infiniteCost();
            Planner::clear();
            setup_ = false;
        }

        ompl::base::PlannerStatus BLITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Ensure that the planner and state space are setup before solving.
            auto status = ensureSetup();

            // Return early if the planner or state space are not setup.
            if (status == ompl::base::PlannerStatus::StatusType::ABORT)
            {
                return status;
            }

            // Ensure that the problem has start and goal states before solving.
            status = ensureStartAndGoalStates(terminationCondition);

            // Return early if the problem cannot be solved.
            if (status == ompl::base::PlannerStatus::StatusType::INVALID_START ||
                status == ompl::base::PlannerStatus::StatusType::INVALID_GOAL)
            {
                return status;
            }
            OMPL_INFORM("%s: Solving the given planning problem. The current best solution cost is %.4f", name_.c_str(),
                        solutionCost_.value());
                
            // Iterate to solve the problem.
            startT = chrono::high_resolution_clock::now();
            while (!terminationCondition && !objective_->isSatisfied(solutionCost_))
            { 
                iterate(terminationCondition);
            }
            // Someone might call ProblemDefinition::clearSolutionPaths() between invocations of Planner::sovle(), in
            // which case previously found solutions are not registered with the problem definition anymore.
            status = updateSolution();

            // Let the caller know the status.
            informAboutPlannerStatus(status);
            return status;
        }

        ompl::base::Cost BLITstar::bestCost() const
        {
            return solutionCost_;
        }

        void BLITstar::getPlannerData(base::PlannerData &data) const
        {
            // base::PlannerDataVertex takes a raw pointer to a state. I want to guarantee, that the state lives as
            // long as the program lives.
            static std::set<std::shared_ptr<Vertex>,
                            std::function<bool(const std::shared_ptr<Vertex> &, const std::shared_ptr<Vertex> &)>>
                liveStates([](const auto &lhs, const auto &rhs) { return lhs->getId() < rhs->getId(); });

            // Fill the planner progress properties.
            Planner::getPlannerData(data);

            // Get the vertices.
            auto vertices = graph_.getVertices();

            // Add the vertices and edges.
            for (const auto &vertex : vertices)
            {
                // Add the vertex to the live states.
                liveStates.insert(vertex);

                // Add the vertex as the right kind of vertex.
                if (graph_.isStart(vertex))
                {
                    data.addStartVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }
                else if (graph_.isGoal(vertex))
                {
                    data.addGoalVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }
                else
                {
                    data.addVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }

                // If it has a parent, add the corresponding edge.
                if (vertex->hasForwardParent())
                {
                    data.addEdge(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()),
                                 ompl::base::PlannerDataVertex(vertex->getForwardParent()->getState(),
                                                               vertex->getForwardParent()->getId()));
                }
            }
        }

        void BLITstar::setBatchSize(std::size_t batchSize)
        {
            batchSize_ = batchSize;
        }

        std::size_t BLITstar::getBatchSize() const
        {
            return batchSize_;
        }

        void BLITstar::setRewireFactor(double rewireFactor)
        {
            graph_.setRewireFactor(rewireFactor);
        }

        double BLITstar::getRewireFactor() const
        {
            return graph_.getRewireFactor();
        }

        void BLITstar::trackApproximateSolutions(bool track)
        {
            trackApproximateSolutions_ = track;
            if (!trackApproximateSolutions_)
            {
                if (static_cast<bool>(objective_))
                {
                    approximateSolutionCost_ = objective_->infiniteCost();
                    approximateSolutionCostToGoal_ = objective_->infiniteCost();
                }
            }
        }

        bool BLITstar::areApproximateSolutionsTracked() const
        {
            return trackApproximateSolutions_;
        }

        void BLITstar::enablePruning(bool prune)
        {
            isPruningEnabled_ = prune;
        }

        bool BLITstar::isPruningEnabled() const
        {
            return isPruningEnabled_;
        }

        void BLITstar::setUseKNearest(bool useKNearest)
        {
            graph_.setUseKNearest(useKNearest);
        }

        bool BLITstar::getUseKNearest() const
        {
            return graph_.getUseKNearest();
        }

       void BLITstar::setMaxNumberOfGoals(unsigned int numberOfGoals)
        {
            graph_.setMaxNumberOfGoals(numberOfGoals);
        }

        unsigned int BLITstar::getMaxNumberOfGoals() const
        {
            return graph_.getMaxNumberOfGoals();
        }

        void BLITstar::clearReverseVertexQueue()
        {
            std::vector<blitstar::KeyVertexPair> reverseQueue;
            reverseVertexQueue_.getContent(reverseQueue);
            for (const auto &element : reverseQueue)
            {
                element.second->resetbackwardVertexQueuePointer();
            }
            reverseVertexQueue_.clear();
        }

        void BLITstar::clearForwardVertexQueue()
        {
            std::vector<blitstar::KeyVertexPair> forwardQueue;
            forwardVertexQueue_.getContent(forwardQueue);
            for (const auto &element : forwardQueue)
            {
                element.second->resetforwardVertexQueuePointer();
            }
            forwardVertexQueue_.clear();
        }

        void BLITstar::informAboutNewSolution() const
        {
            OMPL_INFORM("%s (%u iterations): Found a new exact solution of cost %.4f. Sampled a total of %u states, %u "
                        "of which were valid samples (%.1f \%). Processed %u edges, %u of which were collision checked "
                        "(%.1f \%). The forward search tree has %u vertices, %u of which are start states. The reverse "
                        "search tree has %u vertices, %u of which are goal states.",
                        name_.c_str(), numIterations_, solutionCost_.value(), graph_.getNumberOfSampledStates(),
                        graph_.getNumberOfValidSamples(),
                        graph_.getNumberOfSampledStates() == 0u ?
                            0.0 :
                            100.0 * (static_cast<double>(graph_.getNumberOfValidSamples()) /
                                     static_cast<double>(graph_.getNumberOfSampledStates())),
                        numProcessedEdges_, numEdgeCollisionChecks_,
                        numProcessedEdges_ == 0u ? 0.0 :
                                                   100.0 * (static_cast<float>(numEdgeCollisionChecks_) /
                                                            static_cast<float>(numProcessedEdges_)),
                        countNumVerticesInForwardTree(), graph_.getStartVertices().size(),
                        countNumVerticesInReverseTree(), graph_.getGoalVertices().size());
        }

        void BLITstar::informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const
        {
            switch (status)
            {
                case ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Found an exact solution of cost %.4f.", name_.c_str(),
                                numIterations_, solutionCost_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Did not find an exact solution, but found an approximate "
                                "solution "
                                "of cost %.4f which is %.4f away from a goal (in cost space).",
                                name_.c_str(), numIterations_, approximateSolutionCost_.value(),
                                approximateSolutionCostToGoal_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::TIMEOUT:
                {
                    if (trackApproximateSolutions_)
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find any solution.", name_.c_str(), numIterations_);
                    }
                    else
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find an exact solution, and tracking approximate "
                                    "solutions is disabled.",
                                    name_.c_str(), numIterations_);
                    }
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::INFEASIBLE:
                case ompl::base::PlannerStatus::StatusType::UNKNOWN:
                case ompl::base::PlannerStatus::StatusType::INVALID_START:
                case ompl::base::PlannerStatus::StatusType::INVALID_GOAL:
                case ompl::base::PlannerStatus::StatusType::UNRECOGNIZED_GOAL_TYPE:
                case ompl::base::PlannerStatus::StatusType::CRASH:
                case ompl::base::PlannerStatus::StatusType::ABORT:
                case ompl::base::PlannerStatus::StatusType::TYPE_COUNT:
                {
                    OMPL_INFORM("%s (%u iterations): Unable to solve the given planning problem.", name_.c_str(),
                                numIterations_);
                }
            }

            OMPL_INFORM(
                "%s (%u iterations): Sampled a total of %u states, %u of which were valid samples (%.1f \%). "
                "Processed %u edges, %u of which were collision checked (%.1f \%). The forward search tree "
                "has %u vertices. The reverse search tree has %u vertices.",
                name_.c_str(), numIterations_, graph_.getNumberOfSampledStates(), graph_.getNumberOfValidSamples(),
                graph_.getNumberOfSampledStates() == 0u ?
                    0.0 :
                    100.0 * (static_cast<double>(graph_.getNumberOfValidSamples()) /
                             static_cast<double>(graph_.getNumberOfSampledStates())),
                numProcessedEdges_, numEdgeCollisionChecks_,
                numProcessedEdges_ == 0u ?
                    0.0 :
                    100.0 * (static_cast<float>(numEdgeCollisionChecks_) / static_cast<float>(numProcessedEdges_)),
                countNumVerticesInForwardTree(), countNumVerticesInReverseTree());
        }

        std::size_t BLITstar::countNumVerticesInReverseTree() const
        {
            std::size_t numVerticesInReverseTree = 0u;
            auto vertices = graph_.getVertices();
            for (const auto &vertex : vertices)
            {
                if (graph_.isGoal(vertex) || vertex->hasReverseParent())
                {
                    ++numVerticesInReverseTree;
                }
            }
            return numVerticesInReverseTree;
        }
         
        std::size_t BLITstar::countNumVerticesInForwardTree() const
        {
            std::size_t numVerticesInForwardTree = 0u;
            auto vertices = graph_.getVertices();
            for (const auto &vertex : vertices)
            {
                if (graph_.isStart(vertex) || vertex->hasForwardParent())
                {
                    ++numVerticesInForwardTree;
                }
            }
            return numVerticesInForwardTree;
        } 

        void BLITstar::insertGoalVerticesInReverseVertexQueue()
        {
            for (const auto &goal : graph_.getGoalVertices())
            {
                // Set the cost to come from the goal to identity and the expanded cost to infinity.
                goal->setCostToComeFromGoal(objective_->identityCost());
                goal->setCostToComeFromStart(objective_->infiniteCost());
                goal->setLowerCostBoundToGoal(objective_->identityCost());
                goal->setLowerCostBoundToStart(computeCostToGoToStartHeuristic(goal));
                goal->ResetReverseExpanded();
                goal->setGoalVertex();
                goal->setReverseVersion(reverseSearchVersion_); 
                // Create an element for the queue.
                blitstar::KeyVertexPair element({goal->getLowerCostBoundToStart(), objective_->identityCost()},
                                               goal);
                // Insert the element into the queue and set the corresponding pointer.
                auto reverseQueuePointer = reverseVertexQueue_.insert(element);
                goal->setbackwardVertexQueuePointer(reverseQueuePointer);
            }
        }
        
        void BLITstar::lookingForBestNeighbor(ompl::base::Cost curMin_, size_t neighbor)
        {
              if(objective_->isCostBetterThan(curMin_,minimalneighbor_))
              {
                 minimalneighbor_ = curMin_;
                 bestNeighbor_ = neighbor;
              }
        }
        void BLITstar::bestNeighbor(ompl::base::Cost costToCome, ompl::base::Cost costToGoal, size_t neighbor)
        {
              ompl::base::Cost f_value = objective_->combineCosts(costToCome,costToGoal);
              lookingForBestNeighbor(f_value,neighbor);    
        }
        
        void BLITstar::insertOrUpdateInReverseVertexQueue(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal, bool couldMeet)
        {
          auto element = vertex->getbackwardVertexQueuePointer();
          //Update it if it is in
          if(element) {
                 element->data.first = computeEstimatedPathCosts(CostToCome,CostToGoal);
                 reverseVertexQueue_.update(element);
          }
          else //Insert the pointer into the queue
          {
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> element(computeEstimatedPathCosts(CostToCome,CostToGoal),
                                                                                          vertex);
                // Insert the vertex into the queue, storing the corresponding pointer.
                auto backwardQueuePointer = reverseVertexQueue_.insert(element);
                vertex->setbackwardVertexQueuePointer(backwardQueuePointer);
          }
          if(couldMeet)
             bestNeighbor(CostToCome,CostToGoal,vertex->getId());
        }   

        void BLITstar::insertStartVerticesInForWardVertexQueue()
        {
            for (const auto &start : graph_.getStartVertices())
            {
                // Set the cost to come from the goal to identity and the expanded cost to infinity.
                start->setCostToComeFromStart(objective_->identityCost());
                start->setCostToComeFromGoal(objective_->infiniteCost());
                // Set the lower cost bound for start to go or to come
                start->setLowerCostBoundToStart(objective_->identityCost());
                start->setLowerCostBoundToGoal(computeCostToGoToGoalHeuristic(start));
                // Create an element for the queue.
                blitstar::KeyVertexPair element({start->getLowerCostBoundToGoal(), objective_->identityCost()},
                                               start);
                start->setStartVertex();
                start->ResetForwardExpanded();
                start->setForwardVersion(forwardSearchVersion_);    
                // Insert the element into the queue and set the corresponding pointer.
                auto forwardQueuePointer = forwardVertexQueue_.insert(element);
                start->setforwardVertexQueuePointer(forwardQueuePointer);
            } 
        }    
        
        bool BLITstar::NeedMoreSamples()
        {  return isVertexEmpty_ ? true : false; }
        
        bool BLITstar::PathValidity(std::shared_ptr<Vertex> &vertex) 
        {
            bool found_validity = true;
            forwardInvalid_ = false;
            ForwardPathValidityChecking(vertex,found_validity);
            reverseInvalid_= false;
            ReversePathValidityChecking(vertex,found_validity);            
            if(forwardInvalid_)
            {
                   forwardSearchVersion_++;
                   clearForwardVertexQueue();
                   insertStartVerticesInForWardVertexQueue();
            }
            if(reverseInvalid_)
            {
                   reverseSearchVersion_++; 
                   clearReverseVertexQueue();
                   insertGoalVerticesInReverseVertexQueue();
            }
            return found_validity;          
        }
        void BLITstar::ForwardPathValidityChecking(std::shared_ptr<Vertex> &vertex, bool &validity)
        {
                 std::shared_ptr<Vertex> tar_ = vertex; // target vertex
                 std::vector<std::shared_ptr<Vertex>> reversePath;
                 while (!graph_.isStart(tar_))
                 {
                       std::shared_ptr<Vertex> src_ = tar_->getForwardParent();// source vertex
                       reversePath.emplace_back(tar_);
                       bool valid_edge = false;
                       if(!tar_->hasForwardParent())
		        {
		            forwardInvalid_ = !(validity =  false);
		            break;
		        }
		        if(!src_->isWhitelistedAsChild(tar_) && !(valid_edge = isValid(make_pair(src_,tar_), tar_->getForwardEdgeTime())))
		        {
                                forwardInvalid_ = !(validity =  false);
                                resetForwardParentAndRemeberTheVertex(tar_,src_);
                                tar_->resetReverseEdgeParent();
		        }
		        
		        if(valid_edge)
		        {
		           src_->setReverseValidParent(tar_,tar_->getEdgeCostFromForwardParent());
		        }
		        tar_ = src_;   
                 }
        }
        
        void BLITstar::ReversePathValidityChecking(std::shared_ptr<Vertex> &vertex, bool &validity)
        {
                 std::shared_ptr<Vertex> tar_ = vertex;
                 std::vector<std::shared_ptr<Vertex>> reversePath;
                 while (!graph_.isGoal(tar_))
                 {
                     reversePath.emplace_back(tar_);
                     std::shared_ptr<Vertex> src_ = tar_->getReverseParent();
                     bool valid_edge = false;
                     if(!tar_->hasReverseParent())
	             {   
		                  reverseInvalid_ = !(validity =  false);
		                  break;
		     }
                     if(!tar_->isWhitelistedAsChild(src_) &&!(valid_edge = isValid(make_pair(tar_,src_),tar_->getReverseEdgeTime())))
                     {
                            reverseInvalid_ = !(validity =  false);
                            resetReverseParentAndRemeberTheVertex(tar_,src_);
                            tar_->resetForwardEdgeParent();
                     }
                     if(valid_edge)
                     {
                         src_->setForwardValidParent(tar_,tar_->getEdgeCostFromReverseParent());  
                     }
                     tar_ = src_;                
                 }   
        }
        
        void BLITstar::resetForwardParentInformation(auto & vertex)
        {
                 vertex->resetForwardParent();
                 resetForwardValue(vertex);
        }
        
        void BLITstar::resetForwardValue(auto & vertex)
        {
            vertex->ResetForwardExpanded();
            vertex->resetforwardVertexQueuePointer();
            vertex->setCostToComeFromStart(objective_->infiniteCost());
        }

        void BLITstar::resetForwardParentAndRemeberTheVertex(auto &child, auto &parent)
        {      
                 child->setNearObstacle();
                 parent->setNearObstacle(); 
                 resetForwardParentInformation(child);
                 child->setForwardInvalid();
                 parent->removeFromForwardChildren(child->getId());  
        }
        
        void BLITstar::resetReverseParentInformation(auto &vertex)
        {
                 vertex->resetReverseParent();
                 resetReverseValue(vertex);
        }
        
        void BLITstar::resetReverseValue(auto &vertex)
        {
            vertex->ResetReverseExpanded();
            vertex->resetbackwardVertexQueuePointer(); 
            vertex->setCostToComeFromGoal(objective_->infiniteCost());
        }

        void BLITstar::resetReverseParentAndRemeberTheVertex(auto &child, auto &parent)
        { 
                 child->setNearObstacle();
                 parent->setNearObstacle(); 
                 child->setReverseInvalid();
                 resetReverseParentInformation(child);
                 parent->removeFromReverseChildren(child->getId());
        }                 
        bool BLITstar::terminateSearch()
        {
             if(isVertexEmpty_)
             {  return true; }
             if(!found_meeting_ && objective_->isCostLargerThan(C_curr,fmin_))
             {
                 return false;
             } 
             
             if(objective_->isCostBetterThan(C_curr,solutionCost_) && !PathValidity(MMvertex_.second))
             {
                 C_curr = solutionCost_;
                 BestVertex_->resetMeet();
                 MMvertex_.second->resetMeet();
                 start_scratch_ = true;
                 return (found_meeting_ = false);
             }  
             return (find_solution_ = true);
        }
                     
        void BLITstar::iterate(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            /*Add new samples to the graph, respecting the termination condition, if needed*/
            if (NeedMoreSamples() && graph_.addSamples(batchSize_, terminationCondition,need_Prune_))
            {
                    // Expanding start and goal vertices simultaneously
                    insertStartVerticesInForWardVertexQueue();
                    insertGoalVerticesInReverseVertexQueue();
                    forwardSearchVersion_ = reverseSearchVersion_ = 0u; //count the scratch_
                    isVertexEmpty_ = find_solution_ = false;
            }
            // If the selected state is on the invalid search tree, remove it from the quque. Otherwise, continue the search
            bool forwardDirection_ = false;
            if(!SelectExpandState(forwardDirection_))
            {   return;  }

            bool terminateSearch_ =  terminateSearch();
            if(start_scratch_)
            {return;    }
            // If terminate function has not yet been triggered or either queue is empty, continuing the search in either direction
            if(!terminateSearch_)
            {
                 if(forwardDirection_) 
                 {   
                        BestVertex_->SetForwardExpanded();
                        ForwardLazySearch(BestVertex_);    
                 }
                 else
                 {    
                        BestVertex_->SetReverseExpanded(); 
                        ReverseLazySearch(BestVertex_);    
                 }
            }
            else
            {
                   ompl::base::Cost currentCost(solutionCost_);
                   solutionCost_ = C_curr;
                   ompl::base::Cost ecost(1.26); 
                   if( find_solution_ && objective_->isCostBetterThan(solutionCost_,currentCost))
                   { 
                                  foundSolution_or_Exhausted_ = iSolution_ = true;
                                  cerr << solutionCost_ << "  ";
                                  cout << solutionCost_ << "  ";
                                  showingRunningtime();
                                  path_= getPathToVertex(V_meet.second);
                                  
                                  
                   } 
                   clearReverseVertexQueue();
                   clearForwardVertexQueue();
                   found_meeting_ = !(isVertexEmpty_ = true);  
                   need_Prune_ = foundSolution_or_Exhausted_ ? true:need_Prune_;
                      
            }
            // Keep track of the number of iterations.
            ++numIterations_;
        }
        
        bool BLITstar::betterThan(const ompl::base::Cost & cost1, const ompl::base::Cost & cost2)
        {
            return objective_->isCostBetterThan(cost1,cost2);
        }  

        bool BLITstar::isVertexBetter(const blitstar::KeyVertexPair &lhs, const blitstar::KeyVertexPair &rhs) const
        {
                // it's a regular comparison of the keys.
                return std::lexicographical_compare(
                    lhs.first.cbegin(), lhs.first.cend(), rhs.first.cbegin(), rhs.first.cend(),
                    [this](const auto &a, const auto &b) { return objective_->isCostBetterThan(a, b); });
        }
        void BLITstar::runTime()
        {
            endT = chrono::high_resolution_clock::now();
            time_taken = chrono::duration_cast<chrono::nanoseconds>(endT - startT).count();
            time_taken *= 1e-9;
        }

        void BLITstar::showingRunningtime() 
        {
            cerr  << fixed << time_taken << setprecision(9) << endl;
            cout  << fixed << time_taken << setprecision(9) << endl;
        }
        
        void BLITstar::printForwardStatement(std::shared_ptr<blitstar::Vertex> &vertex)
        {
                 if(vertex->isStart())
                 {
                     cerr << "vertex-> " << vertex->getId() << " is expanding in forward preliminary tree. "   << " " << vertex->getCostToComeFromStart() << "  " << vertex->getLowerCostBoundToGoal() << endl;
                 }
                 else
                 {
                    cerr << "vertex-> " << vertex->getId() << " is expanding in forward preliminary tree. "  << "  "  << vertex->getForwardParent()->getId() << " " << vertex->getCostToComeFromStart() << "  " << vertex->getLowerCostBoundToGoal() << endl;
                 }
                 
        }        
        void BLITstar::ForwardLazySearch(const std::shared_ptr<blitstar::Vertex> &vertex) 
        {   
        
            size_t improvedVertex = 0; 
            minimalneighbor_ = objective_->infiniteCost();
            // Insert current children into forward queue  
            for (const auto &child : vertex->getForwardChildren())
            {      
                    if(iSolution_ && vertex->hasReverseEdgeParent() && vertex->getReverseEdgeParent()->getId() == child->getId())
                    {     continue;    }
                    if((!iSolution_ || vertex->nearObstacle() || child->nearObstacle()) && !couldBeValid(make_pair(vertex,child),child->getForwardEdgeTime()))
                    {
                            resetForwardParentAndRemeberTheVertex(child,vertex);
                            continue;
                    }

                    auto edgeCost = child->getEdgeCostFromForwardParent();
                    // g_hat(x_v) 
                    auto gValue =  objective_->combineCosts(vertex->getCostToComeFromStart(),edgeCost);
                    if(iSolution_ && !child->hasReverseParent() && objective_->isCostLargerThan(objective_->combineCosts(gValue,gValue),solutionCost_))
                    {      continue;   }
                    // h_bar(x_v): lower cost bound to go such as Eculidean distance
                    auto hValue = child->getLowerCostBoundToGoal();     
                    if(child->getForwardVersion() != forwardSearchVersion_ || (child->hasForwardParent() && child->getForwardParent()->getId() == vertex->getId())) 
                    {    
                         child->setCostToComeFromStart(gValue); 
                         gValue = child->getCostToComeFromStart();
                         updateForwardCost(child, gValue, hValue);
                         insertOrUpdateInForwardVertexQueue(child,gValue,hValue,vertex->meetVertex());            
                    }
            }
            
            // Insert new neighbors in to forward queue
            bool Potential_collide_ = false;
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {    
                if(neighbor->getId() == vertex->getId())
                   continue; 
                if(iSolution_ && !vertex->nearObstacle() && neighbor->nearObstacle())
                {
                     Potential_collide_ = true;  
                }     
                if(forwardInvalid_ && neighbor->getForwardVersion() != forwardSearchVersion_)
                {
                     neighbor->setCostToComeFromStart(objective_->infiniteCost());
                     neighbor->ResetForwardExpanded();  
                }
                if (!neighbor->IsForwardExpanded())
                { 
                    // If x_u is the parent of x_v, it will not be added to the Q_F
                    if(iSolution_ && vertex->hasReverseEdgeParent() && vertex->getReverseEdgeParent()->getId() == neighbor->getId())
                    {    continue;  }
                    if(neighbor->hasForwardParent() && neighbor->getForwardParent()->getId() == vertex->getId())
                    {    continue;  }
                    if (neighbor->isBlacklistedAsChild(vertex) || vertex->isBlacklistedAsChild(neighbor))
                    {    continue;  }
                    if(vertex->isStart() &&neighbor->isGoal() )
                    {
                             EvaluateValidityStartAndToGoal(neighbor,vertex); 
                             continue;
                    }
               
                    auto gValue = (neighbor->hasForwardParent())? neighbor->getCostToComeFromStart() : objective_->infiniteCost();
                    // c_hat(x_u,x_v)
                    auto edgeCost = objective_->motionCostHeuristic(neighbor->getState(), vertex->getState());//auto edgeCost = TrueEdgeCost(neighbor->getState(),vertex->getState());
                    double arrTime_ = 0;
                    
                    // g_hat(x_u) + c_hat(x_u,x_v)
                    auto est_gValue = objective_->combineCosts(vertex->getCostToComeFromStart(), edgeCost);
                  
                    // If g_F(x_v) > g_F(x_u) + c_hat(x_u,x_v) 
                    if (objective_->isCostBetterThan(est_gValue, gValue)&& neighbor->getId() != 0)
                    {
                         if(iSolution_ && !neighbor->hasReverseParent() && objective_->isCostLargerThan(objective_->combineCosts(est_gValue,est_gValue),solutionCost_))
                         {      continue;   }
                          
                         neighbor->setForwardVersion(forwardSearchVersion_);
                         bool fValid_ = false, rValid_ = false, supposeMeet = false;
                         if(!iSolution_ && !couldBeValid(make_pair(vertex,neighbor),arrTime_) )//: !isValidAtResolution(make_pair(vertex,neighbor),200,false,arrTime_)
                         {
                           vertex->setNearObstacle();
                           neighbor->setNearObstacle();
                           neighbor->setForwardInvalid();

                           continue;
                         }
                         // Set x_u to be x_v's parent and update g_hat(x_v)!iSolution_ &&
                         bool startCollid_ = goalCloseToStart_ && vertex->isStart();
                         if(startCollid_ || Potential_collide_ || vertex->nearObstacle() || neighbor->nearObstacle())
                         {
                           if(!(fValid_ = couldBeValid(make_pair(vertex,neighbor),arrTime_)) || ((!iSolution_ || neighbor->forwardInvalid()) && !isValid(make_pair(vertex,neighbor),arrTime_))) 
                           {  
                                neighbor->setNearObstacle();
                                vertex->setNearObstacle();
                                neighbor->setForwardInvalid();
                                continue;
                           }
                           if(startCollid_)
                           { neighbor->setNearObstacle(); }
                         }
                         neighbor->setCostToComeFromStart(est_gValue);
                         neighbor->setForwardVertexParent(vertex,edgeCost,arrTime_);  
                         vertex->addToForwardChildren(neighbor);
                         auto hValue = neighbor->getLowerCostBoundToGoal();
                         auto getReversepointer = neighbor->getbackwardVertexQueuePointer(); 
                         
                         bool onReverseTree = !(neighbor->hasReverseParent() && neighbor->getReverseVersion() != reverseSearchVersion_);  
                         if(!onReverseTree)
                         { resetReverseValue(neighbor); }
                         // Push x_v into forward vertex queue
                         // If x_v is in Q_B,  Uv = min(U,g_f(s) + g_b(s))
                         if(onReverseTree && (getReversepointer || (neighbor->hasReverseParent())))//|| 
                         {  
                            keyEdgePair fEdge = make_pair(vertex,neighbor);
                            keyEdgePair rEdge = make_pair(neighbor,neighbor->getReverseParent());
                            if(!iSolution_)
                            {
                                 if(!fValid_) 
                                 {
                                    fValid_ = couldBeValid(fEdge,arrTime_);
                                 }
                                 rValid_ = couldBeValid(rEdge,neighbor->getReverseEdgeTime());
                            }

                            
                            if((fValid_ && rValid_) || iSolution_) 
                            {                                     
                                    auto Totalcost = objective_->combineCosts(est_gValue,neighbor->getCostToComeFromGoal());
                                    lookingForBestNeighbor(Totalcost, neighbor->getId());
                                    supposeMeet = true;//arrTime_,neighbor->getReverseEdgeTime(),pathLengthGL(vertex->getState(),neighbor->getState()).value(),pathLengthGL(neighbor->getState(),neighbor->getReverseParent()->getState()).value() 
                                    if(objective_->isCostBetterThan(Totalcost,C_curr)&& (iSolution_ || ((fValid_ = isValid(fEdge,arrTime_)) && (rValid_=isValid(rEdge, neighbor->getReverseEdgeTime())))) )//
                                    {
                                           improvedVertex = neighbor->getId();
                                           updateBestSolutionFoundSoFar(neighbor,Totalcost,est_gValue,hValue,neighbor->getCostToComeFromGoal());
                                           insertOrUpdateInForwardVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()); 
                                           continue;
                                    }
                            }

                            if(!iSolution_ && (!fValid_ || !rValid_)) 
                            {
                                    neighbor->setNearObstacle(); 
                                    auto backwardParent = neighbor->getReverseParent();
                                    if(!fValid_) 
                                    {
                                        resetForwardParentAndRemeberTheVertex(neighbor, vertex);  
                                    } 
                                    else 
                                    {
                                            updateCostToGo(neighbor, est_gValue, hValue,hValue,false); 
                                            insertOrUpdateInForwardVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()); 
                                    }
                                    if(!rValid_)
                                    {   resetReverseParentAndRemeberTheVertex(neighbor, backwardParent); }
                                    continue;
                            }
                         }
                         updateCostToGo(neighbor, est_gValue, hValue,hValue,false);  
                         insertOrUpdateInForwardVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()&& !supposeMeet);  
                    } 
                }
            }
            found_meeting_ = (vertex->meetVertex() && !objective_->isCostLargerThan(minimalneighbor_,C_curr)) ? true : false;    
            
            //cerr << "\t testing here--------------------------------------> " << minimalneighbor_ << endl;      
            if(iSolution_ && vertex->hasReverseEdgeParent())
            {
                  auto reverseParent = vertex->getReverseEdgeParent();
                  auto edgeCost = vertex->getEdgeCostFromReverseParent();//objective_->motionCostHeuristic(reverseParent->getState(), vertex->getState())
                  auto currentValue = objective_->combineCosts(vertex->getCostToComeFromStart(),edgeCost);
                  auto hValue = reverseParent->getLowerCostBoundToGoal();
                  if(vertex->getId() != reverseParent->getForwardParent()->getId() && objective_->isCostBetterThan(currentValue,reverseParent->getCostToComeFromStart()))
                  {
                         reverseParent->setCostToComeFromStart(currentValue);  
                         auto arriveTime_ = vertex->getReverseEdgeTime();
                         reverseParent->setForwardVertexParent(vertex,reverseParent->getEdgeCostFromForwardParent(),arriveTime_);
                         vertex->addToForwardChildren(reverseParent);
                  }
                  updateForwardCost(reverseParent, reverseParent->getCostToComeFromStart(), hValue);
                  insertOrUpdateInForwardVertexQueue(reverseParent,reverseParent->getCostToComeFromStart(),hValue,vertex->meetVertex()); 
            }
        } 
        
        void BLITstar::updateBestSolutionFoundSoFar(const std::shared_ptr<Vertex> &vertex, ompl::base::Cost meetCost, ompl::base::Cost costToCome, ompl::base::Cost &costToGo, ompl::base::Cost costFromOriginal)
        {
                           if(MMvertex_.second)
                           {  MMvertex_.second->resetMeet(); }
                           vertex->setMeet();
		           runTime();
                           MMvertex_ = make_pair(C_curr=meetCost,vertex);
                           updateCostToGo(vertex, costToCome, costToGo,costFromOriginal,true); 
        }
        void BLITstar::updateCostToGo(const std::shared_ptr<Vertex> &vertex, ompl::base::Cost &costToCome, ompl::base::Cost &costToGo, ompl::base::Cost costFromOriginal,bool meetOnTree)
        {
                    if(objective_->isCostBetterThan(costToGo,costToCome))
                    {
                            costToGo =   costToCome;
                    }
                    if(meetOnTree && objective_->isCostBetterThan(costFromOriginal,costToGo))
                    {
                            costToGo = costFromOriginal;
                    }
           
        }
        
        void BLITstar::updateForwardCost(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo)
        {
                         vertex->ResetForwardExpanded();
                         vertex->setForwardVersion(forwardSearchVersion_); 
                         if(vertex->hasReverseParent() && vertex->getReverseVersion() == reverseSearchVersion_)
                         {
                             auto bettersolution_ = objective_->combineCosts(vertex->getCostToComeFromGoal(),vertex->getCostToComeFromStart());
                             if(objective_->isCostBetterThan(bettersolution_,C_curr))
                             {
                                updateBestSolutionFoundSoFar(vertex,bettersolution_,costToCome,costToGo,vertex->getCostToComeFromGoal());
                             }
                             else
                             {
                                updateCostToGo(vertex, costToCome, costToGo,vertex->getCostToComeFromGoal(),true);
                             }  
                         }
                         else
                         {
                             updateCostToGo(vertex, costToCome, costToGo,costToGo,false); 
                         }   
        }
        
        void BLITstar::updateReverseCost(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo)
        {
                         vertex->ResetReverseExpanded();
                         vertex->setReverseVersion(reverseSearchVersion_);
                         if(vertex->hasForwardParent() && vertex->getForwardVersion() == forwardSearchVersion_)
                         {
                             auto bettersolution_ = objective_->combineCosts(vertex->getCostToComeFromGoal(),vertex->getCostToComeFromStart());
                             if(objective_->isCostBetterThan(bettersolution_,C_curr))
                             {
                                 updateBestSolutionFoundSoFar(vertex,bettersolution_,costToCome,costToGo,vertex->getCostToComeFromStart());
                             } 
                             else
                             {
                                updateCostToGo(vertex,costToCome,costToGo,vertex->getCostToComeFromStart(),true);
                             } 
                         }
                         else
                         {
                             updateCostToGo(vertex,costToCome,costToGo,costToGo,false);
                         }
        }
        
        
        void BLITstar::EvaluateValidityStartAndToGoal(const std::shared_ptr<Vertex> &start, const std::shared_ptr<Vertex> &goal)
        {
                      goalCloseToStart_ = true;
                      double arrTime_ = 0;
                      if(isValid(make_pair(start,goal), arrTime_))
                      {
                            solutionCost_ = objective_->motionCost(start->getState(), goal->getState()); 
                            start->setReverseVertexParent(goal,solutionCost_,arrTime_);    
                            goal->setForwardVertexParent(start,solutionCost_,arrTime_); 
                            
                      }
        }
        
        void BLITstar::printReverseStatement(std::shared_ptr<blitstar::Vertex> &vertex)
        {
                  if(vertex->isGoal())
                  {
                     cerr << "vertex->   " << vertex->getId() << " is expanding in reverse preliminary tree. " << "  " << vertex->getCostToComeFromGoal() << "  " << vertex->getLowerCostBoundToStart() << endl;
                  }
                  else
                  {
                    cerr << "vertex->   " << vertex->getId() << " is expanding in reverse preliminary tree. " << vertex->getReverseParent()->getId() << "  " << vertex->getCostToComeFromGoal() << "  " << vertex->getLowerCostBoundToStart() << endl;
                  }      
        }        
        void BLITstar::ReverseLazySearch(const std::shared_ptr<blitstar::Vertex> &vertex) 
        { 

            size_t improvedVertex = 0; 
            minimalneighbor_ = objective_->infiniteCost();
            // Expanding in reverse tree is analogous as forward tree
            for (const auto &child : vertex->getReverseChildren())
            {      
                    if(iSolution_ && vertex->hasForwardEdgeParent() && child->getId() == vertex->getForwardEdgeParent()->getId())
                       continue;
                    if((!iSolution_ || vertex->nearObstacle()  || child->nearObstacle()) && !couldBeValid(make_pair(child,vertex),child->getReverseEdgeTime()))
                    {
                            resetReverseParentAndRemeberTheVertex(child,vertex);
                            continue;
                    }
                    auto edgeCost = child->getEdgeCostFromReverseParent() ;
                    auto gValue = objective_->combineCosts(vertex->getCostToComeFromGoal(),edgeCost);
                    if(iSolution_ && !child->hasForwardParent() && objective_->isCostLargerThan(objective_->combineCosts(gValue,gValue),solutionCost_))
                    {      continue;   }
                    auto hValue = child->getLowerCostBoundToStart();                   
                    if(child->getReverseVersion() != reverseSearchVersion_ || (child->hasReverseParent() && ( child->getReverseParent()->getId() == vertex->getId())))
                    {
                         child->setCostToComeFromGoal(gValue);                                              
                         gValue = child->getCostToComeFromGoal();
                         updateReverseCost(child,gValue,hValue);
                         insertOrUpdateInReverseVertexQueue(child,gValue,hValue,vertex->meetVertex());  
                    }
            }
            bool Potential_collide_ = false;
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {  
                if(neighbor->getId() == vertex->getId())
                {
                     continue;
                }
                if(iSolution_ && !vertex->nearObstacle() && neighbor->nearObstacle())
                {
                     Potential_collide_ = true;  
                }                    
                if(reverseInvalid_ && neighbor->getReverseVersion() != reverseSearchVersion_)
                {
                     neighbor->ResetReverseExpanded();
                     neighbor->setCostToComeFromGoal(objective_->infiniteCost());    
                } 
                
                if (!neighbor->IsReverseExpanded())
                { 
                    
                    if(iSolution_ && vertex->hasForwardEdgeParent() && neighbor->getId() == vertex->getForwardEdgeParent()->getId())
                    {   
                       continue;
                    }
                    if(vertex->isGoal() && neighbor->isStart())
                    {
                        EvaluateValidityStartAndToGoal(vertex,neighbor);
                        continue;
                    }
                     
                    if(neighbor->hasReverseParent() && neighbor->getReverseParent()->getId() == vertex->getId())
                    {
                        continue;
                    }

                    if (neighbor->isBlacklistedAsChild(vertex) || vertex->isBlacklistedAsChild(neighbor))
                    {
                       continue;
                    }
                    auto gValue = (neighbor->hasReverseParent() ) ? neighbor->getCostToComeFromGoal() :objective_->infiniteCost();                 
                    
                    auto edgeCost = objective_->motionCostHeuristic(neighbor->getState(), vertex->getState());
                    double arrTime_ = 0;
                    auto est_gValue = objective_->combineCosts(vertex->getCostToComeFromGoal(), edgeCost);  

                    if (objective_->isCostBetterThan(est_gValue, gValue) && neighbor->getId() != 1)
                    {
                         if(iSolution_&& !neighbor->hasForwardParent()  && objective_->isCostLargerThan(objective_->combineCosts(est_gValue,est_gValue),solutionCost_))
                         {      continue;   }
                         neighbor->setReverseVersion(reverseSearchVersion_);
                         if(!iSolution_ && !couldBeValid(make_pair(neighbor,vertex),arrTime_))//: !isValidAtResolution(make_pair(neighbor,vertex),200,false,arrTime_)
                         {
                            //cerr << "chcking here " <<  << endl;
                            vertex->setNearObstacle();
                            neighbor->setNearObstacle();
                            neighbor->setReverseInvalid();
                            neighbor->ResetReverseExpanded();
                            continue;
                         }
                         bool fValid_ = false, rValid_ = false, supposeMeet = false;
                         bool goalCollid_ = goalCloseToStart_ && vertex->isGoal();
                         if(goalCollid_ || Potential_collide_ || vertex->nearObstacle() || neighbor->nearObstacle())
                         {
                           if(!(rValid_ = couldBeValid(make_pair(neighbor,vertex),arrTime_)) || ((!iSolution_ || neighbor->reverseInvalid())&& !isValid(make_pair(neighbor,vertex),arrTime_))) 
                           {  
                                neighbor->setNearObstacle();
                                vertex->setNearObstacle();
                                neighbor->setReverseInvalid();
                                neighbor->ResetReverseExpanded();
                                continue;
                           }
                           if(goalCollid_)
                           { neighbor->setNearObstacle();}
                         }
                         neighbor->setCostToComeFromGoal(est_gValue); 
                   
                         neighbor->setReverseVertexParent(vertex,edgeCost,arrTime_);
                         vertex->addToReverseChildren(neighbor);
                         auto hValue = neighbor->getLowerCostBoundToStart();
                         auto getForwardpointer = neighbor->getforwardVertexQueuePointer(); 
                         bool onForwardTree = !(neighbor->hasForwardParent() && neighbor->getForwardVersion() != forwardSearchVersion_);
                         if(!onForwardTree)
                         { resetForwardValue(neighbor);}          
                         if(onForwardTree && (getForwardpointer || neighbor->hasForwardParent()))
                         { 
                                keyEdgePair rEdge = make_pair(neighbor,vertex);
                                keyEdgePair fEdge = make_pair(neighbor->getForwardParent(),neighbor);
                                if(!iSolution_)
                                {                                
                                    if(!rValid_)
                                    {
                                       rValid_ = couldBeValid(rEdge,arrTime_);  
                                    }
                                    fValid_ = couldBeValid(fEdge,neighbor->getForwardEdgeTime());//

                                }                           
                            if(iSolution_ || (fValid_ && rValid_)) 
                            {
                                auto Totalcost = objective_->combineCosts(est_gValue,neighbor->getCostToComeFromStart());
                                supposeMeet = true;
                                lookingForBestNeighbor(Totalcost, neighbor->getId());
                                if(objective_->isCostBetterThan(Totalcost,C_curr)&&(iSolution_|| ((fValid_ = isValid(fEdge,neighbor->getForwardEdgeTime()))&&(rValid_=isValid(rEdge,arrTime_)))))//
                                {
                                     updateBestSolutionFoundSoFar(neighbor,Totalcost,est_gValue,hValue,neighbor->getCostToComeFromStart());
                                     improvedVertex = neighbor->getId();
                                     insertOrUpdateInReverseVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex());
                                     continue;
                                }
                            }
                            
                            if(!iSolution_ && (!fValid_ || !rValid_)) 
                            {
                                      neighbor->setNearObstacle();
                                      auto forwardParent = neighbor->getForwardParent();
                                      if(!fValid_) 
                                      {
                                                resetForwardParentAndRemeberTheVertex(neighbor, forwardParent);
                                      } 
                                        
                                      if(!rValid_)
                                      {
                                                resetReverseParentAndRemeberTheVertex(neighbor, vertex);    
                                      }
                                      else 
                                      {
                                                updateCostToGo(neighbor, est_gValue, hValue,hValue,false); 
                                                insertOrUpdateInReverseVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()); 
                                      }
                                        continue; 
                            }
                         } 
                         updateCostToGo(neighbor, est_gValue, hValue,hValue,false);  
                         insertOrUpdateInReverseVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()&& !supposeMeet); 
                    } 
                }
            }
            
            found_meeting_ = (vertex->meetVertex() && objective_->isCostBetterThan(minimalneighbor_,C_curr)) ? true : false; 
            if(iSolution_ && vertex->hasForwardEdgeParent())
            {
                  auto forwardParent = vertex->getForwardEdgeParent();
                  auto edgeCost = vertex->getEdgeCostFromForwardParent(); //auto edgeCost = objective_->motionCostHeuristic(forwardParent->getState(), vertex->getState());
                  auto currentValue = objective_->combineCosts(vertex->getCostToComeFromGoal(),edgeCost);
                  auto hValue = forwardParent->getLowerCostBoundToStart();
                  if(vertex->getId() != forwardParent->getReverseParent()->getId() && objective_->isCostBetterThan(currentValue,forwardParent->getCostToComeFromGoal()))
                  {
                         forwardParent->setCostToComeFromGoal(currentValue); 
                         double arriveTime_ = vertex->getForwardEdgeTime(); 
                         forwardParent->setReverseVertexParent(vertex,forwardParent->getEdgeCostFromReverseParent(),arriveTime_);
                         vertex->addToReverseChildren(forwardParent);
                  }
                  updateReverseCost(forwardParent,forwardParent->getCostToComeFromGoal(),hValue);
                  insertOrUpdateInReverseVertexQueue(forwardParent,forwardParent->getCostToComeFromGoal(),hValue,vertex->meetVertex()); 
            }
        }
        
          
        void BLITstar::insertOrUpdateInForwardVertexQueue(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal, bool couldMeet)
        {
          //Get the pointer to the element in the queue
          auto element = vertex->getforwardVertexQueuePointer();
          //Update it if it is in
          if(element) {
                 element->data.first = computeEstimatedPathCosts(CostToCome,CostToGoal);
                 forwardVertexQueue_.update(element);
          }
          else //Insert the pointer into the queue
          {
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> element(computeEstimatedPathCosts(CostToCome,CostToGoal),
                                                                                             vertex);
                // Insert the vertex into the queue, storing the corresponding pointer.
                auto forwardQueuePointer = forwardVertexQueue_.insert(element);
                vertex->setforwardVertexQueuePointer(forwardQueuePointer);
          }
          if(couldMeet)
             bestNeighbor(CostToCome,CostToGoal,vertex->getId());
        }
           
        std::shared_ptr<ompl::geometric::PathGeometric> BLITstar::getPathToVertex(const std::shared_ptr<Vertex> &vertex) const
        {
            // Create the reverse path by following the parents in forward tree to the start from the middler.
            std::vector<std::shared_ptr<Vertex>> reversePath;
            auto current = vertex;
            while (!graph_.isStart(current))
            {
                reversePath.emplace_back(current);
                current = current->getForwardParent();
            }
            reversePath.emplace_back(current);

            // Reverse the reverse path to get the forward path.
            auto path = std::make_shared<ompl::geometric::PathGeometric>(Planner::si_);
            for (const auto &vertex_ : boost::adaptors::reverse(reversePath))
            {   
                path->append(vertex_->getState());
            }
            reversePath.clear();
            // Trace back the forward path by following the parents in reverse tree to goal from the middler
            current = vertex; 
            while (!graph_.isGoal(current))
            {
                reversePath.emplace_back(current);
                current = current->getReverseParent();
            }
            reversePath.emplace_back(current);
            for (const auto &vertex_ : reversePath)
            { 
                if(vertex_->getId() != vertex->getId())
                {
                  spaceInformation_->printState(vertex_->getState());
                  path->append(vertex->getState());
                }   
            }
            return path;
        }

        std::array<ompl::base::Cost, 3u> BLITstar::computeEstimatedPathCosts(ompl::base::Cost CostToStart, ompl::base::Cost CostToGoal, ompl::base::Cost motionCost) const
        {
            // f = g(v) + c(v,w)+ h(w); g(x) = g(parent(x))+c(parent(x),x)
            return {objective_->combineCosts(CostToStart,CostToGoal),CostToStart, motionCost};
        }

        std::array<ompl::base::Cost, 2u> BLITstar::computeEstimatedPathCosts(ompl::base::Cost CostToStart, ompl::base::Cost CostToGoal) const
        {
            // f = g(x) + h(x); g(x) = g(parent(x))+c(parent(x),x)
            return {objective_->combineCosts(CostToStart,CostToGoal),CostToStart};
        }
                                
        ompl::base::Cost BLITstar::computeCostToGoToStartHeuristic(const std::shared_ptr<Vertex> &vertex) const
        {
            // We need to loop over all start vertices and see which is the closest one.
            ompl::base::Cost bestCost = objective_->infiniteCost();
            for (const auto &start : graph_.getStartVertices())
            {
                bestCost = objective_->betterCost(
                    bestCost, objective_->motionCostHeuristic(vertex->getState(), start->getState()));
            }
            return bestCost;
        }

        ompl::base::Cost BLITstar::computeCostToGoToGoalHeuristic(const std::shared_ptr<Vertex> &vertex) const
        {
            // We need to loop over all goal vertices and see which is the closest one.
            ompl::base::Cost bestCost = objective_->infiniteCost();
            for (const auto &goal : graph_.getGoalVertices())
            {
                bestCost = objective_->betterCost(
                    bestCost, objective_->motionCostHeuristic(vertex->getState(), goal->getState()));
            }
            return bestCost;
        }

        ompl::base::PlannerStatus::StatusType BLITstar::updateSolution()
        {
            updateExactSolution();
            
            if (objective_->isFinite(solutionCost_))
            {
                return ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
            }
            else if (trackApproximateSolutions_)
            {
                //updateApproximateSolution();
                return ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION;
            }
            else
            {
                return ompl::base::PlannerStatus::StatusType::TIMEOUT;
            }
        }

        ompl::base::Cost BLITstar::computeCostToGoToGoal(const std::shared_ptr<Vertex> &vertex) const
        {
            // We need to loop over all goal vertices and see which is the closest one.
            ompl::base::Cost bestCost = objective_->infiniteCost();
            for (const auto &goal : graph_.getGoalVertices())
            {
                bestCost =
                    objective_->betterCost(bestCost, objective_->motionCost(vertex->getState(), goal->getState()));
            }
            return bestCost;
        }

        ompl::base::Cost BLITstar::computeBestCostToComeFromGoalOfAnyStart() const
        {
            // We need to loop over all start vertices and see which is the closest one.
            ompl::base::Cost bestCost = objective_->infiniteCost();
            for (const auto &start : graph_.getStartVertices())
            {
                bestCost = objective_->betterCost(bestCost, start->getCostToComeFromGoal());
            }
            return bestCost;
        }
        
               
        bool BLITstar::SelectExpandState(bool & forwardDirection_)//
        {
            if(found_meeting_)
            {
                 return true;
            }
            if(forwardVertexQueue_.empty() || reverseVertexQueue_.empty())
            {    
                 C_curr = solutionCost_;
                 return (isVertexEmpty_ = !(need_Prune_ = false));
            }   
            
            start_scratch_ = found_meeting_ = foundSolution_or_Exhausted_ = false;
            auto forwardVertex_ = forwardVertexQueue_.top()->data.second;
            auto backwardVertex_ = reverseVertexQueue_.top()->data.second;
            ompl::base::Cost ForwardCost = forwardVertexQueue_.top()->data.first[0u];
            ompl::base::Cost BackwardCost = reverseVertexQueue_.top()->data.first[0u];
            
            if(forwardInvalid_ && forwardVertex_->getForwardVersion() != forwardSearchVersion_)
            {
                forwardVertexQueue_.pop(); 
                resetForwardValue(forwardVertex_); 
                return false;
            }
            
            if(reverseInvalid_ && backwardVertex_->getReverseVersion() != reverseSearchVersion_)
            {
                reverseVertexQueue_.pop();
                resetReverseValue(backwardVertex_);
                return false;   
            } 
             
            // If the minimum priority is from forward search
            if(objective_->isCostBetterThan(ForwardCost, BackwardCost))
            {
                   BestVertex_ = forwardVertexQueue_.top()->data.second;
                   forwardVertexQueue_.pop();
                   BestVertex_->resetforwardVertexQueuePointer();
                   fmin_ = ForwardCost;
                   if(BestVertex_->IsForwardExpanded())
                   {  return true; }
                   forwardDirection_ = true;
            } 
            else 
            {   
                   BestVertex_ = reverseVertexQueue_.top()->data.second;
                   reverseVertexQueue_.pop();
                   BestVertex_->resetbackwardVertexQueuePointer();
                   fmin_ = BackwardCost;
                   if(BestVertex_->IsReverseExpanded())
                   {return true;}
            }
            return true;
        }
        
        bool BLITstar::couldBeValid(const keyEdgePair &edge, double arriveTime_)
        {
            return isValidAtResolution(edge, numSparseCollisionChecksCurrentLevel_,true,arriveTime_);
        }
        
        bool BLITstar::isValid(const keyEdgePair &edge, double arriveTime_)
        {
            return isValidAtResolution(edge,space_->validSegmentCount(parent->getState(), child->getState()),false,arriveTime_);  
        }
        
        bool BLITstar::isValidAtResolution(const keyEdgePair &edge, std::size_t numChecks, bool sparseCheck, double arriveTime_ )
        {
        
            auto parent = edge.first;
            auto child = edge.second;
            // Check if the edge is whitelisted.
            if (parent->isWhitelistedAsChild(child))
            {
                return true;
            }

            // If the edge is blacklisted.
            if (child->isBlacklistedAsChild(parent))
            {
                return false;
            }

            // Get the segment count for the full resolution.
            const std::size_t fullSegmentCount = space_->validSegmentCount(parent->getState(), child->getState());

            // The segment count is the number of checks on this level plus 1, capped by the full resolution segment
            // count.
            const auto segmentCount = std::min(numChecks + 1u, fullSegmentCount);
           
            // Store the current check number.
            std::size_t currentCheck = 1u;

            // Get the number of checks already performed on this edge.
            const std::size_t performedChecks = child->getIncomingCollisionCheckResolution(parent->getId());
            // Initialize the queue of positions to be tested.
            std::queue<std::pair<std::size_t, std::size_t>> indices;
            indices.emplace(1u, numChecks);
            
            // Test states while there are states to be tested.
            while (!indices.empty())
            {
                // Get the current segment.
                const auto current = indices.front();

                // Get the midpoint of the segment.
                auto mid = (current.first + current.second) / 2;
                // Only do the detection if we haven't tested this state on a previous level.
                if (currentCheck > performedChecks)
                {
                    space_->interpolate(parent->getState(), child->getState(),static_cast<double>(mid) / static_cast<double>(segmentCount), detectionState_);
                    if (!spaceInformation_->isValid(detectionState_))
                    {
                        // Blacklist the edge.
                        parent->blacklistAsChild(child);
                        child->blacklistAsChild(parent);
                        if(!sparseCheck && currentCheck > numSparseCollisionChecksCurrentLevel_)
                        {
                            numSparseCollisionChecksCurrentLevel_ = currentCheck+1u;
                        }  
                        // Register it with the graph.
                        return false;
                    }
                }

                // Remove the current segment from the queue.
                indices.pop();

                // Create the first and second half of the split segment if necessary.
                if (current.first < mid)
                {
                    indices.emplace(current.first, mid - 1u);
                }

                if (current.second > mid)
                {
                    indices.emplace(mid + 1u, current.second);
                }

                // Increase the current check number.
                ++currentCheck;
            }

            // Remember at what resolution this edge was already checked. We're assuming that the number of collision
            // checks is symmetric for each edge.
            parent->setIncomingCollisionCheckResolution(child->getId(), currentCheck - 1u);
            child->setIncomingCollisionCheckResolution(parent->getId(), currentCheck - 1u);

            // Whitelist this edge if it was checked at full resolution.
            if (segmentCount == fullSegmentCount)
            {   
                parent->whitelistAsChild(child);
                child->whitelistAsChild(parent);
            }
            return true;
        }  
    }  // namespace geometric
}  
