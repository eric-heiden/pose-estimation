#pragma once

#include <nlopt.hpp>

#include "../configuration.hpp"
#include "../pipelinemodule.hpp"

namespace PoseEstimation
{
    /**
     * @brief Stores the parseName of a {@link Parameter} together with its numerical value.
     */
    typedef std::map<std::string, double> Assignment;

    struct OptimizationResult
    {
        double bestPerformance;
        Assignment bestAssignment;
        nlopt::result resultCode;
        PipelineStats bestStats;
    };

    /**
     * @brief Optimization of configuration parameters using NLopt.
     */
    template<typename PointT>
    class OPT : public PipelineModule
    {
    public:
        OPT(PC<PointT> source, PC<PointT> target)
            : PipelineModule(PipelineModuleType::Miscellaneous)
        {
            _objective.source = source;
            _objective.target = target;
        }

        /**
         * @brief Performs non-linear optimization over all involved parameters
         * in the given configuration.
         * @param config The configuration.
         * @return The {@link OptimizationResult}.
         */
        OptimizationResult optimize(CF<PointT> config)
        {
            _objective.parameters.clear();
            _objective.bestPerformance = std::numeric_limits<double>::min();
            _objective.iteration = 0;

            #define IFF(skip, parameter, type) \
                bool b_##skip = skip.value<bool>() && parameter->category().moduleType() == type; \
                if (b_##skip) \
                { \
                    Logger::warning(boost::format("Skipping optimization of parameter %s (%s)") \
                            % parameter->parseName() \
                            % PipelineModuleType::str(parameter->category().moduleType())); \
                } \
                if (b_##skip)

            // only consider numerical parameters
            for (Parameter *p : config.involvedParameters())
            {
                if (!p->isNumber())
                    continue;

                IFF(skipDescriptor, p, PipelineModuleType::FeatureDescriptor)
                    continue;
                IFF(skipDownsampler, p, PipelineModuleType::Downsampler)
                    continue;
                IFF(skipFeatureMatcher, p, PipelineModuleType::FeatureMatcher)
                    continue;
                IFF(skipKeypointExtractor, p, PipelineModuleType::KeypointExtractor)
                    continue;
                IFF(skipTransformationEstimator, p, PipelineModuleType::TransformationEstimator)
                    continue;
                IFF(skipPoseRefiner, p, PipelineModuleType::PoseRefiner)
                    continue;
                IFF(skipHypothesisVerifier, p, PipelineModuleType::HypothesisVerifier)
                    continue;
                IFF(skipMiscellaneous, p, PipelineModuleType::Miscellaneous)
                    continue;

                _objective.parameters.push_back(p);
            }

            _objective.configuration = config;

            const size_t dimensions = _objective.parameters.size();

            nlopt::opt opt(nlopt::LN_COBYLA, dimensions); // "LN" means local optimization, no derivatives

            _objective.lowerBounds = std::vector<double>(dimensions);
            _objective.upperBounds = std::vector<double>(dimensions);
            for (size_t i = 0; i < dimensions; ++i)
            {
                // temporarily, non-linear optimization might pass the boundaries
                _objective.lowerBounds[i] = _objective.parameters[i]->lowerBound();
                _objective.upperBounds[i] = _objective.parameters[i]->upperBound();
            }

            opt.set_min_objective(Objective::wrap, &_objective);

            // deactivate relative tolerance stopping criterion
            opt.set_xtol_rel(-1.0);
            // stop when an optimization step changes all parameters by less than this value
            opt.set_xtol_abs(xdelta.value<float>());
            // stop after so many iterations
            opt.set_maxeval(iterations.value<int>());

            std::vector<double> x(dimensions);
            if (skipInitialization.value<bool>())
            {
                for (size_t i = 0; i < dimensions; ++i)
                {
                    // use existing settings
                    x[i] = _objective.parameters[i]->numericalValue();
                }
            }
            else
            {
                Logger::debug("Initializing variables");
                for (size_t i = 0; i < dimensions; ++i)
                {
                    // initialize variables
                    x[i] = _objective.lowerBounds[i]
                            + alpha.value<float>() * (_objective.upperBounds[i] - _objective.lowerBounds[i]);
                }
            }

            double minf;
            nlopt::result result;

            if (enabled.value<bool>())
            {
                result = opt.optimize(x, minf);
                if (result > 0)
                {
                    Logger::log(boost::format("Optimization finished successfully. Best performance: %d. Stopping criterion: %i")
                                % (minf) % result);
                    for (auto &assignment : _objective.bestAssignment)
                    {
                        Logger::debug(boost::format("\t%s = %d") % assignment.first % assignment.second);
                    }
                }
                else
                {
                    Logger::error(boost::format("Optimization failed. Performance: %d. Stopping criterion: %i")
                                  % (minf) % result);
                }
            }
            else
            {
                Logger::warning("Optimization has been skipped (check opt_enabled).");
            }

            OptimizationResult optres;
            optres.bestPerformance = _objective.bestPerformance;
            optres.bestAssignment = _objective.bestAssignment;
            optres.bestStats = _objective.bestStats;
            optres.resultCode = result;

            return optres;
        }

        OPT() = default;
        OPT(OPT<PointT> &) = default;
        OPT(OPT<PointT> &&) = default;

        OPT<PointT>& operator=(const OPT<PointT> &) & = default;
        OPT<PointT>& operator=(OPT<PointT> &&) & = default;

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter enabled;

        static Parameter alpha;
        static Parameter iterations;
        static Parameter xdelta;
        static Parameter skipInitialization;

        static Parameter skipDescriptor;
        static Parameter skipDownsampler;
        static Parameter skipFeatureMatcher;
        static Parameter skipKeypointExtractor;
        static Parameter skipTransformationEstimator;
        static Parameter skipPoseRefiner;
        static Parameter skipMiscellaneous;        
        static Parameter skipHypothesisVerifier;

    private:
        /**
         * @brief Functor implementing the objective function for the non-linear optimization.
         */
        class Objective
        {
            public:
                static const size_t MAX_POINTS = 640 * 480; // Kinect's resolution

                /**
                 * @brief Computes the performance value of the pipeline's outcome.
                 * @param stats The pipeline's outcome statistics.
                 * @return The performance value (the higher the better).
                 */
                double performance(PipelineStats &stats)
                {
                    // actual transformation instances are weighted much higher
                    // so that we don't only get a large number of keypoints
                    return 0.01 * (stats.sourceDownsampledPoints / MAX_POINTS)
                            + 0.01 * (stats.targetDownsampledPoints / MAX_POINTS)
                            //+ 0.03 * (stats.sourceKeypointsExtracted / MAX_POINTS)
                            //+ 0.03 * (stats.targetKeypointsExtracted / MAX_POINTS)
                            + 0.5 * stats.correspondencesFound / MAX_POINTS
                            //- stats.averageCorrespondenceDistance * 0.8
                            //- stats.correspondenceSlopeVariance / 10.0
                            + .01 * stats.transformationInstances.size()
                            + .05 * stats.verifiedTransformationInstances.size();
                }

                /**
                 * @brief Computes the objective function value for the given variables.
                 * @param x The variables.
                 * @param grad The gradients. Not used here (non-linear optimization is used).
                 * @return The value of the objective function (performance).
                 */
                double operator()(const std::vector<double> &x, std::vector<double> &)
                {
                    ++iteration;
                    Logger::log(boost::format("##### Started Optimization Iteration %i #####")
                                % iteration);

                    // assign parameter values as determined by NLopt
                    for (size_t i = 0; i < parameters.size(); ++i)
                    {
                        parameters[i]->setNumericalValue(x[i]);
                        Logger::debug(boost::format("Setting %s = %d\t  (was: %s  min: %d  max: %d)")
                                      % parameters[i]->parseName()
                                      % x[i]
                                      % (previous.empty() ? "N/A" : boost::lexical_cast<std::string>((float)previous[i]))
                                      % lowerBounds[i]
                                      % upperBounds[i]);
                    }

                    previous = x;

                    // copy point clouds
                    PC<PointT> sourceCopy(source);
                    PC<PointT> targetCopy(target);

                    // execute pipeline for the given configuration
                    PipelineStats stats = configuration.run(sourceCopy, targetCopy);

                    // compute performance
                    double p = performance(stats);
                    if (p > bestPerformance)
                    {
                        // store best assignment
                        for (size_t i = 0; i < parameters.size(); ++i)
                        {
                            bestAssignment[parameters[i]->parseName()] = x[i];
                        }

                        bestStats = stats;
                    }

                    Logger::debug("+++ Detailed Performance Report +++");
                    stats.print();
                    Logger::log(boost::format("##### Finished Iteration %i. Performance: %d #####")
                                % iteration % -p);
                    return -p; // maximize performance, i.e. minimize -p
                }

                /**
                 * @brief Delegates the NLopt function call to the objective of a functor instance.
                 * @param x The variables.
                 * @param grad The gradients. Not used here (non-linear optimization is used).
                 * @param data Pointer to the {@see Objective} functor instance.
                 * @return The value of the objective function (performance).
                 */
                static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data)
                {
                    return (*reinterpret_cast<OPT::Objective*>(data))(x, grad);
                }

                PC<PointT> source;
                PC<PointT> target;
                CF<PointT> configuration;
                std::vector<Parameter*> parameters;

                size_t iteration;
                std::vector<double> lowerBounds;
                std::vector<double> upperBounds;
                std::vector<double> previous;
                Assignment bestAssignment;
                double bestPerformance;
                PipelineStats bestStats;
        };

        Objective _objective;
    };

    template<typename PointT>
    ParameterCategory OPT<PointT>::argumentCategory(
            "opt", "Non-Linear Optimization for Pipeline Module Parameters",
            PipelineModuleType::Miscellaneous);

    template<typename PointT>
    Parameter OPT<PointT>::enabled = Parameter(
            "opt",
            "enabled",
            true,
            "Whether to optimize pipeline module parameters");

    template<typename PointT>
    Parameter OPT<PointT>::alpha = Parameter(
            "opt",
            "alpha",
            0.2f,
            "Relative parameter value during initialization in the corresponding value range",
            NUMERICAL_PARAMETER_RANGE(0.0, 1.0));

    template<typename PointT>
    Parameter OPT<PointT>::iterations = Parameter(
            "opt",
            "iterations",
            40,
            "Maximum number of iterations (stopping criterion)");

    template<typename PointT>
    Parameter OPT<PointT>::xdelta = Parameter(
            "opt",
            "xdelta",
            0.5f,
            "Minimum allowed change of all parameter values (stopping criterion)");

    template<typename PointT>
    Parameter OPT<PointT>::skipInitialization = Parameter(
            "opt",
            "skip_init",
            true,
            "Skip automatic parameter initialization using value boundaries and begin with existing parameter settings");

    template<typename PointT>
    Parameter OPT<PointT>::skipDescriptor = Parameter(
            "opt",
            "skip_descriptor",
            false,
            "Skip optimization of feature description parameters");

    template<typename PointT>
    Parameter OPT<PointT>::skipDownsampler = Parameter(
            "opt",
            "skip_downsampler",
            false,
            "Skip optimization of downsampling parameters");

    template<typename PointT>
    Parameter OPT<PointT>::skipFeatureMatcher = Parameter(
            "opt",
            "skip_feature_matcher",
            false,
            "Skip optimization of feature matching parameters");

    template<typename PointT>
    Parameter OPT<PointT>::skipKeypointExtractor = Parameter(
            "opt",
            "skip_keypoint_extractor",
            false,
            "Skip optimization of keypoint extraction parameters");

    template<typename PointT>
    Parameter OPT<PointT>::skipTransformationEstimator = Parameter(
            "opt",
            "skip_transformation_estimator",
            false,
            "Skip optimization of transformation estimation parameters");

    template<typename PointT>
    Parameter OPT<PointT>::skipPoseRefiner = Parameter(
            "opt",
            "skip_pose_refiner",
            false,
            "Skip optimization of pose refinement parameters");

    template<typename PointT>
    Parameter OPT<PointT>::skipMiscellaneous = Parameter(
            "opt",
            "skip_misc",
            false,
            "Skip optimization of miscellaneous parameters");

    template<typename PointT>
    Parameter OPT<PointT>::skipHypothesisVerifier = Parameter(
            "opt",
            "skip_hypothesis_verifier",
            true, //XXX
            "Skip optimization of hypothesis verification parameters");


    typedef OPT<PointType> Optimizer;
}
