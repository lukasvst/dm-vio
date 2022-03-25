/**
* This file is part of DM-VIO.
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
* for more information see <http://vision.in.tum.de/dm-vio>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DM-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DM-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <gtsam/base/SymmetricBlockMatrix.h>
#include "PoseTransformationFactor.h"
#include "Marginalization.h"
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include "GTSAMUtils.h"
#include "util/TimeMeasurement.h"
#include "ExtUtils.h"

dmvio::PoseTransformationFactor::PoseTransformationFactor(const gtsam::NonlinearFactor::shared_ptr& factor,
                                                          const PoseTransformation& poseTransformationPassed,
                                                          ConversionType conversionType,
                                                          const gtsam::Values& fixedValuesPassed)
        : NonlinearFactor(factor->keys()), factor(factor), conversionType(conversionType)
{
    fixedValues.insert(fixedValuesPassed);
    if(!fixedValues.empty())
    {
        keys_.clear();
        fixedKeys = fixedValues.keys();
        fixedKeySet = std::set<gtsam::Key>(fixedKeys.begin(), fixedKeys.end());

        // Only insert non-fixed child keys!
        for(auto&& childKey : factor->keys())
        {
            if(fixedKeySet.find(childKey) == fixedKeySet.end())
            {
                keys_.push_back(childKey);
            }
        }
    }

    // Clone the pose transformation so we can update it with new values.
    poseTransformation = poseTransformationPassed.clone();

    auto&& optimizedSymbols = poseTransformation->getAllOptimizedSymbols();
    keys_.insert(keys_.end(), optimizedSymbols.begin(), optimizedSymbols.end());

    // assumes that the factor does not optimize any of the symbols optimized by poseTransformation!
    for(auto&& key : optimizedSymbols)
    {
        additionalDim += poseTransformation->getOptimizedDim(key);
    }
}

dmvio::PoseTransformationFactor::PoseTransformationFactor(const dmvio::PoseTransformationFactor& o)
        : gtsam::NonlinearFactor(o), factor(o.factor->clone()), poseTransformation(o.poseTransformation->clone()),
          conversionType(o.conversionType),
          additionalDim(o.additionalDim), fixedValues(o.fixedValues), fixedKeys(o.fixedKeys), fixedKeySet(o.fixedKeySet)
{}

dmvio::PoseTransformationFactor::~PoseTransformationFactor()
= default;

gtsam::Values dmvio::PoseTransformationFactor::convertValues(const gtsam::Values& c) const
{
    // Update the poseTransformation.
    poseTransformation->updateWithValues(c);

    // And convert all values.
    gtsam::Values ret;
    convertAllPosesWithTransform(c, *poseTransformation, keys_, ret);
    convertAllPosesWithTransform(fixedValues, *poseTransformation, fixedKeys, ret);
    return ret;
}

double dmvio::PoseTransformationFactor::error(const gtsam::Values& c) const
{
    return factor->error(convertValues(c));
}

size_t dmvio::PoseTransformationFactor::dim() const
{
    return factor->dim() + additionalDim;
}

void dmvio::PoseTransformationFactor::setFEJValues(std::shared_ptr<FEJValues> fejPassed)
{
    // Called if we are supposed to use FEJ values for derivative computatio.

    // Important: Also forward to child factor if necessary!
    auto* casted = dynamic_cast<FactorHandlingFEJ*>(factor.get());
    if(casted)
    {
        if(fejPassed == nullptr)
        {
            childFej.reset();
        }else
        {
            // We have to forward the transformed fej values though!
            // The transformation might change during the optimization, so we do it during linearize.
            childFej = std::make_shared<FEJValues>();
        }
        casted->setFEJValues(childFej);
    }

    fej = std::move(fejPassed);
}

boost::shared_ptr<gtsam::GaussianFactor> dmvio::PoseTransformationFactor::linearize(const gtsam::Values& c) const
{
    // First convert FEJValues for child factor.
    if(childFej)
    {
        // We need optimized symbols + the child factor non-fixed keys.
        gtsam::Values fejVals = fej->buildValues(keys_, c);
        childFej->fejValues = convertValues(fejVals);
    }

    // Linearize child factor.
    gtsam::GaussianFactor::shared_ptr gaussian = factor->linearize(convertValues(c));
    // Note that convertValues already updates the poseTransformation.

    auto&& optimizedSymbols = poseTransformation->getAllOptimizedSymbols();

    gtsam::JacobianFactor* jacobianFac = nullptr;
    if(conversionType == JACOBIAN_FACTOR)
    {
        jacobianFac = dynamic_cast<gtsam::JacobianFactor*>(gaussian.get());
        if(jacobianFac == nullptr)
        {
            std::cout << "WARNING: Using ConversionType JACOBIAN_FACTOR, but a different GaussianFactor was passed!"
                      << std::endl;
        }
    }
    // We have to multiple the Jacobian (A) with our relative Jacobian.
    std::pair<gtsam::Matrix, gtsam::Vector> Ab = std::pair<gtsam::Matrix, gtsam::Vector>();
    if(jacobianFac == nullptr)
    {
        Ab = gaussian->jacobian();
    }else
    {
        Ab = jacobianFac->jacobianUnweighted();
    }

    // Use FEJ values for derivatives if passed.
    if(fej)
    {
        gtsam::Values fejVals = fej->buildValues(optimizedSymbols, c);
        poseTransformation->updateWithValues(fejVals);
    }

    poseTransformation->precomputeForDerivatives();

    // Computation of terms:
    // We only iterate the child keys, but including fixed keys.
    std::vector<std::pair<gtsam::Key, gtsam::Matrix> > terms(keys_.size());
    int pos = 0;
    int i = 0;
    int rows = Ab.first.rows();
    bool firstPose = true;
    int firstOptPos = keys_.size() - optimizedSymbols.size(); // size of non-fixed child keys
    for(auto it = gaussian->keys().begin(); it != gaussian->keys().end(); ++it)
    {
        int dim = gaussian->getDim(it);
        gtsam::Key key = *it;
        bool fixed = fixedKeySet.find(key) != fixedKeySet.end();

        if(gtsam::Symbol(key).chr() == 'p') // We only need to convert poses.
        {
            gtsam::Pose3 pose;
            if(fixed)
            {
                pose = fixedValues.at<gtsam::Pose3>(key);
            }else if(fej && fej->fejValues.exists(key))
            {
                pose = fej->fejValues.at<gtsam::Pose3>(key);
            }else
            {
                pose = c.at<gtsam::Pose3>(key);
            }
            std::vector<gtsam::Matrix> derivatives = poseTransformation->getAllDerivatives(pose.matrix(),
                                                                                           DerivativeDirection::RIGHT_TO_RIGHT);

            if(!fixed)
            {
                terms[i].first = key;
                terms[i].second =
                        Ab.first.block(0, pos, rows, dim) * derivatives[0]; // multiply with relative pose Jacobian.
                i++;
            }

            int j = 0;
            for(auto&& optKey : optimizedSymbols)
            {
                auto&& deriv = derivatives[j + 1];
                int optPos = j + firstOptPos;

                if(firstPose)
                {
                    terms[optPos].first = optKey;
                    terms[optPos].second = Ab.first.block(0, pos, rows, dim) * deriv;
                }else
                {
                    terms[optPos].second += Ab.first.block(0, pos, rows, dim) * deriv;
                }

                j++;
            }
            firstPose = false;
        }else if(!fixed)
        {
            terms[i].first = key;
            terms[i].second = Ab.first.block(0, pos, rows, dim);
            i++;
        }

        pos += dim;
    }

    if(jacobianFac == nullptr)
    {
        return boost::shared_ptr<gtsam::GaussianFactor>(
                new gtsam::JacobianFactor(terms, Ab.second));
    }else
    {
        return boost::shared_ptr<gtsam::GaussianFactor>(
                new gtsam::JacobianFactor(terms, Ab.second, jacobianFac->get_model()));
    }

}

std::ostream& dmvio::operator<<(std::ostream& os, dmvio::PoseTransformationFactor::ConversionType& conversion)
{
    os << static_cast<std::underlying_type<dmvio::PoseTransformationFactor::ConversionType>::type>(conversion);
    return os;
}

std::istream& dmvio::operator>>(std::istream& is, dmvio::PoseTransformationFactor::ConversionType& conversion)
{
    int num;
    is >> num;
    conversion = static_cast<dmvio::PoseTransformationFactor::ConversionType>(num);
    return is;
}

template<>
void dmvio::defaultYAMLHandler<dmvio::PoseTransformationFactor::ConversionType>(void* pointer, const YAML::Node& node)
{
    auto* typedPointer = static_cast<dmvio::PoseTransformationFactor::ConversionType*>(pointer);
    *typedPointer = static_cast<dmvio::PoseTransformationFactor::ConversionType>(node.as<int>());
}