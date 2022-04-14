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

#ifndef DMVIO_SIM3GTSAM_H
#define DMVIO_SIM3GTSAM_H


#include <sophus/sim3.hpp>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Lie.h>

// In contrast to Sim3GTSAM this contains only the scale.
// Could probably be made faster by not basing it on Sophus.
class ScaleGTSAM : public gtsam::LieGroup<ScaleGTSAM, 1>
{
public:
    ScaleGTSAM(double scale);

    Sophus::Sim3d sim() const;
    double scale = 1.0;

    // For the LieGroup.
    static ScaleGTSAM identity();

    /// Composition
    ScaleGTSAM operator*(const ScaleGTSAM& T) const;

    /// Return the inverse
    ScaleGTSAM inverse() const;

    static gtsam::Vector1 Logmap(const ScaleGTSAM& s,
                                 gtsam::OptionalJacobian<1, 1> Hm = boost::none);

    static ScaleGTSAM Expmap(const gtsam::Vector1& v,
                             gtsam::OptionalJacobian<1, 1> Hm = boost::none);

    /// Chart at the origin
    struct ChartAtOrigin
    {
        static ScaleGTSAM Retract(const gtsam::Vector1& v, ChartJacobian H = boost::none)
        {
            return ScaleGTSAM::Expmap(v, H);
        }

        static gtsam::Vector1 Local(const ScaleGTSAM& other, ChartJacobian H = boost::none)
        {
            return ScaleGTSAM::Logmap(other, H);
        }
    };
    using LieGroup<ScaleGTSAM, 1>::inverse;

    gtsam::Matrix1 AdjointMap() const;


    void print(const std::string& str) const;
    size_t dim() const;
    bool equals(const ScaleGTSAM& other, double tol) const;
};

namespace gtsam
{

template<>
struct traits<ScaleGTSAM> : public internal::LieGroup<ScaleGTSAM>
{
};

template<>
struct traits<const ScaleGTSAM> : public internal::LieGroup<ScaleGTSAM>
{
};

}

#endif //DMVIO_SIM3GTSAM_H
