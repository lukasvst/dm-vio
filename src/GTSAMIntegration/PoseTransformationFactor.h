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

#ifndef DMVIO_POSETRANSFORMATIONFACTOR_H
#define DMVIO_POSETRANSFORMATIONFACTOR_H

#include "PoseTransformation.h"
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "yaml-cpp/yaml.h"
#include "util/SettingsUtil.h"
#include "FEJValues.h"

namespace dmvio
{

// Transform another factor using the PoseTransformation
// Note that the transformation works in the inverse direction:
// To pass a factor which works in coordinate system IMU, to a graph working in coordinate system DSO,
// we have to pass wrap the factor in TransformDSOToIMU.
// The class can also fix variables of the child factor.
// The factor will also apply First-Estimates Jacobians if setFEJMapForGraph has been called for the graph before optimization.
// Note that the child factor must not optimize any of the additional symbols optimized by the PoseTransformation.
// Assumes that all (but only) poses use the symbol 'p'
class PoseTransformationFactor : public gtsam::NonlinearFactor, public FactorHandlingFEJ
{
public:
    // Defines how the factor is converted (use JACOBIAN_FACTOR for JacobianFactor, and JACOBIAN_BAKED_IN for other factor types)
    // It might make a small difference for performance but is not too relevant.
    enum ConversionType
    {
        JACOBIAN_FACTOR, JACOBIAN_BAKED_IN
    };

    // factor is the child factor which is transformed.
    // poseTransformation is the transformation which will be applied to values to to convert from the graph to the child factor.
    // conversionType defines how the linear system is converted (mainly relevant for performance).
    // All symbols in fixedValues will be fixed for the child factor during optimization.
    PoseTransformationFactor(const gtsam::NonlinearFactor::shared_ptr& factor,
                             const PoseTransformation& poseTransformation,
                             ConversionType conversionType,
                             const gtsam::Values& fixedValues = gtsam::Values{});

    PoseTransformationFactor(const PoseTransformationFactor& o);

    virtual ~PoseTransformationFactor();

    // --------------------
    // Methods overriden from NonlinearFactor
    double error(const gtsam::Values& c) const override;

    /** get the dimension of the factor (number of rows on linearization) */
    size_t dim() const override;

    /** linearize to a GaussianFactor */
    boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& c) const override;

    virtual gtsam::NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new PoseTransformationFactor(*this)));
    }

    void setFEJValues(std::shared_ptr<FEJValues> fej) override;

    gtsam::Values fixedValues;
private:
    // convert the values using the PoseTransformation
    // The resulting values will only contain values for the symbols optimized by the child factor, or the TransformationFactor.
    gtsam::Values convertValues(const gtsam::Values& c) const;


    gtsam::NonlinearFactor::shared_ptr factor; // child factor.

    mutable std::unique_ptr<PoseTransformation> poseTransformation;

    ConversionType conversionType;

    int additionalDim = 0; // dimension of symbols optimized by the poseTransformation.

    gtsam::FastVector<gtsam::Key> fixedKeys;
    std::set<gtsam::Key> fixedKeySet;

    std::shared_ptr<FEJValues> fej;
    std::shared_ptr<FEJValues> childFej;
};
std::ostream& operator<<(std::ostream& os, dmvio::PoseTransformationFactor::ConversionType& conversion);
std::istream& operator>>(std::istream& is, dmvio::PoseTransformationFactor::ConversionType& conversion);
}

namespace dmvio
{
template<>
void defaultYAMLHandler<dmvio::PoseTransformationFactor::ConversionType>(void* pointer, const YAML::Node& node);
}

#endif //DMVIO_POSETRANSFORMATIONFACTOR_H
