#pragma once

#include <boost/algorithm/string.hpp>

namespace PoseEstimation
{
    //XXX rewrite using macros (cf. configuration.hpp)?
    namespace PipelineModuleType
    {
        /**
         * @brief Enumerates the main components of the Pose Estimation pipeline.
         */
        enum Type
        {
            Downsampler,
            KeypointExtractor,
            FeatureDescriptor,
            FeatureMatcher,
            PoseRefiner,
            TransformationEstimator,
            LocalReferenceFrameEstimator,
            HypothesisVerifier,
            Miscellaneous
        };

        static std::string str(Type t)
        {
            switch (t)
            {
                case Downsampler:
                    return "Downsampler";
                case KeypointExtractor:
                    return "KeypointExtractor";
                case FeatureDescriptor:
                    return "FeatureDescriptor";
                case FeatureMatcher:
                    return "FeatureMatcher";
                case PoseRefiner:
                    return "PoseRefiner";
                case TransformationEstimator:
                    return "TransformationEstimator";
                case LocalReferenceFrameEstimator:
                    return "LocalReferenceFrameEstimator";
                case HypothesisVerifier:
                    return "HypothesisVerifier";
                default:
                    return "Miscellaneous";
            }
        }

        static Type parse(std::string s)
        {
            boost::algorithm::to_lower(s);
            if (s == "downsampler")
                return Type::Downsampler;
            if (s == "keypointextractor")
                return Type::KeypointExtractor;
            if (s == "featuredescriptor")
                return Type::FeatureDescriptor;
            if (s == "featurematcher")
                return Type::FeatureMatcher;
            if (s == "poserefiner")
                return Type::PoseRefiner;
            if (s == "transformationestimator")
                return Type::TransformationEstimator;
            if (s == "localreferenceframeestimator")
                return Type::LocalReferenceFrameEstimator;
            if (s == "hypothesisverifier")
                return Type::HypothesisVerifier;
            return Type::Miscellaneous;
        }
    }
}
