#pragma once

#include <nlopt.hpp>
#include "../configuration.hpp"

namespace PoseEstimation
{
    template<typename PointT>
    class OPT
    {
    public:
        OPT(PC<PointT> source, PC<PointT> target)
        {
            _objective.source = source;
            _objective.target = target;
        }

        double optimize(CF<PointT> config)
        {
            _objective.parameters.clear();
            // only consider numerical parameters
            for (Parameter *p : config.involvedParameters())
            {
                if (p->isNumber())
                    _objective.parameters.push_back(p);
            }

            _objective.configuration = config;

            const size_t dimensions = _objective.parameters.size();

            nlopt::opt opt(nlopt::LN_COBYLA, dimensions); // "LN" means local optimization, no derivatives

            std::vector<double> lowerBounds(dimensions);
            std::vector<double> upperBounds(dimensions);
            for (size_t i = 0; i < dimensions; ++i)
            {
                lowerBounds[i] = _objective.parameters[i]->lowerBound();
                upperBounds[i] = _objective.parameters[i]->upperBound();
            }

            opt.set_min_objective(Objective::wrap, &_objective);

            opt.set_xtol_rel(1e-2);

            std::vector<double> x(dimensions);
            Logger::debug("Initializing variables");
            for (size_t i = 0; i < dimensions; ++i)
            {
                // initialize variables
                x[i] = (lowerBounds[i] + upperBounds[i]) / 2.0;
                Logger::debug(boost::format("%1% = %2% (min: %3%  max: %4%)")
                              % _objective.parameters[i]->parseName()
                              % x[i]
                              % lowerBounds[i]
                              % upperBounds[i]);
            }

            double minf;
            nlopt::result result = opt.optimize(x, minf);
            if (result == nlopt::result::SUCCESS)
            {
                Logger::log(boost::format("Optimization finished successfully. Performance: %d") % (-minf));
            }

            return -minf;
        }

    private:
        class Objective {
            public:
                Objective()
                {}

                double operator()(const std::vector<double> &x, std::vector<double> &grad)
                {
                    for (size_t i = 0; i < parameters.size(); ++i)
                    {
                        parameters[i]->setNumericalValue(x[i]);
                    }

                    PipelineStats stats = configuration.run(source, target);
                    return (-stats.performance());
                }

                static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data)
                {
                    return (*reinterpret_cast<OPT::Objective*>(data))(x, grad);
                }

                PC<PointT> source;
                PC<PointT> target;
                CF<PointT> configuration;
                std::vector<Parameter*> parameters;
        };
        Objective _objective;
    };

    typedef OPT<PointType> Optimizer;
}
