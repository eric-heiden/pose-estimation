#pragma once

#include <nlopt.hpp>

#include "../configuration.hpp"

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
    };

    /**
     * @brief Optimization of configuration parameters using NLopt.
     */
    template<typename PointT>
    class OPT
    {
    public:
        OPT(PC<PointT> source, PC<PointT> target)
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
            Objective::iteration = 0;

            // only consider numerical parameters
            for (Parameter *p : config.involvedParameters())
            {
                if (p->isNumber())
                    _objective.parameters.push_back(p);
            }

            _objective.configuration = config;

            const size_t dimensions = _objective.parameters.size();

            nlopt::opt opt(nlopt::LN_COBYLA, dimensions); // "LN" means local optimization, no derivatives

            _objective.lowerBounds = std::vector<double>(dimensions);
            _objective.upperBounds = std::vector<double>(dimensions);
            for (size_t i = 0; i < dimensions; ++i)
            {
                // temporarely, non-linear optimization might pass the boundaries
                _objective.lowerBounds[i] = _objective.parameters[i]->lowerBound();
                _objective.upperBounds[i] = _objective.parameters[i]->upperBound();
            }

            opt.set_min_objective(Objective::wrap, &_objective);

            opt.set_xtol_rel(-1.0); // deactivate relative tolerance stopping criterion
            opt.set_maxeval(25);    // stop after so many iterations

            std::vector<double> x(dimensions);
            Logger::debug("Initializing variables");
            const double alpha = 0.2;
            for (size_t i = 0; i < dimensions; ++i)
            {
                // initialize variables
                x[i] = _objective.lowerBounds[i] + alpha * (_objective.upperBounds[i] - _objective.lowerBounds[i]);
            }

            double minf;
            nlopt::result result = opt.optimize(x, minf);
            if (result > 0)
            {
                Logger::log(boost::format("Optimization finished successfully. Performance: %d. Stopping criterion: %i")
                            % (-minf) % result);
            }
            else
            {
                Logger::warning(boost::format("Optimization failed. Performance: %d. Stopping criterion: %i")
                                % (-minf) % result);
            }

            return -minf;
        }

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
                    Logger::debug("+++ Detailed Performance Report +++");
                    stats.print();
                    Logger::log(boost::format("##### Finished Iteration %i. Performance: %d #####")
                                % iteration % p);
                    return -p; // maximize performance, i.e. minimize -performance
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

                static size_t iteration;
                static std::vector<double> lowerBounds;
                static std::vector<double> upperBounds;
                static Assignment bestAssignment; //TODO implement
                static double bestPerformance;
        };

        Objective _objective;
    };

    template<typename PointT>
    size_t OPT<PointT>::Objective::iteration = 0;

    template<typename PointT>
    std::vector<double> OPT<PointT>::Objective::lowerBounds = std::vector<double>();

    template<typename PointT>
    std::vector<double> OPT<PointT>::Objective::upperBounds = std::vector<double>();

    template<typename PointT>
    Assignment OPT<PointT>::Objective::bestAssignment = Assignment();


    typedef OPT<PointType> Optimizer;
}
