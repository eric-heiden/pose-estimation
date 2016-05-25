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
         * @return The
         */
        OptimizationResult optimize(CF<PointT> config)
        {
            _objective.parameters.clear();
            _objective.bestPerformance = 0;
            _objective.iteration = 0;

            // only consider numerical parameters
            for (Parameter *p : config.involvedParameters())
            {
                if (p->isNumber())
                    _objective.parameters.push_back(p);
            }

            _objective.configuration = config;

            const size_t dimensions = _objective.parameters.size();

            nlopt::opt opt(nlopt::LN_BOBYQA, dimensions); // "LN" means local optimization, no derivatives

            _objective.lowerBounds = std::vector<double>(dimensions);
            _objective.upperBounds = std::vector<double>(dimensions);
            for (size_t i = 0; i < dimensions; ++i)
            {
                // temporarily, non-linear optimization might pass the boundaries
                _objective.lowerBounds[i] = _objective.parameters[i]->lowerBound();
                _objective.upperBounds[i] = _objective.parameters[i]->upperBound();
            }

            opt.set_min_objective(Objective::wrap, &_objective);

            opt.set_xtol_rel(-1.0); // deactivate relative tolerance stopping criterion
            opt.set_xtol_abs(xdelta.value<float>());  // stop when an optimization step changes all parameters by less than this value
            opt.set_maxeval(iterations.value<int>());    // stop after so many iterations

            std::vector<double> x(dimensions);
            Logger::debug("Initializing variables");
            for (size_t i = 0; i < dimensions; ++i)
            {
                // initialize variables
                x[i] = _objective.lowerBounds[i]
                        + alpha.value<float>() * (_objective.upperBounds[i] - _objective.lowerBounds[i]);
            }

            double minf;
            nlopt::result result = opt.optimize(x, minf);
            if (result > 0)
            {
                Logger::log(boost::format("Optimization finished successfully. Best performance: %d. Stopping criterion: %i")
                            % (-minf) % result);
                for (auto &assignment : _objective.bestAssignment)
                {
                    Logger::debug(boost::format("\t%s = %d") % assignment.first % assignment.second);
                }
            }
            else
            {
                Logger::warning(boost::format("Optimization failed. Performance: %d. Stopping criterion: %i")
                                % (-minf) % result);
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

        OPT<PointT>& operator=(const OPT<PointT>&) & = default;
        OPT<PointT>& operator=(OPT<PointT>&&) & = default;

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter alpha;
        static Parameter iterations;
        static Parameter xdelta;

    private:
        /**
         * @brief Functor implementing the objective function for the non-linear optimization.
         */
        class Objective
        {
            public:
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
                        Logger::debug(boost::format("Setting %s = %d\t  (min: %d  max: %d)")
                                      % parameters[i]->parseName()
                                      % x[i]
                                      % lowerBounds[i]
                                      % upperBounds[i]);
                    }

                    // copy point clouds
                    PC<PointT> sourceCopy(source);
                    PC<PointT> targetCopy(target);

                    // execute pipeline for the given configuration
                    PipelineStats stats = configuration.run(sourceCopy, targetCopy);

                    // compute performance
                    double p = stats.performance();
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
                                % iteration % p);
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
            "Minimum allowed changed of all parameter values (stopping criterion)");


    typedef OPT<PointType> Optimizer;
}
