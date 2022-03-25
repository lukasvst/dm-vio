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
// This code file is based on the file NonlinearFactor.cpp from the project GTSAM, which has been released under the following conditions:
/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#ifndef DMVIO_FEJNOISEMODELFACTOR_H
#define DMVIO_FEJNOISEMODELFACTOR_H

#include "FEJValues.h"
#include <gtsam/navigation/ImuFactor.h>
#include <dso/util/settings.h>

namespace dmvio
{

// Can be used to transform any NoiseModelFactor into a factor handling First-Estimates Jacobians correctly.
// Before optimizing setFEJMapForGraph has to be called for the Factor Graph.
// The class is based on the gtsam::NonlinearFactor.
// For using it can be wrapped around any type of NoiseModelFactor (e.g. IMUFactor, BetweenFactor, etc.) and
// arguments of the constructor will be forwarded.
template<typename T, typename = typename std::enable_if<std::is_base_of<gtsam::NoiseModelFactor, T>::value>::type>
class FEJNoiseModelFactor : public T, public FactorHandlingFEJ
{
public:
    template<typename ... Args> explicit FEJNoiseModelFactor(Args&& ... args)
            : T(std::forward<Args>(args) ...)
    {}

    void setFEJValues(std::shared_ptr<FEJValues> fejValues) override
    {
        fej = std::move(fejValues);
    }

    // Modified version of NoiseModelFactor::linearize of the project GTSAM. For license, see above.
    boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& x) const override
    {
        // TODO: update linearize if I update GTSAM.
        // Only linearize if the factor is active
        if(!T::active(x))
            return boost::shared_ptr<gtsam::JacobianFactor>();

        // Call evaluate error to get Jacobians and RHS vector b
        std::vector<gtsam::Matrix> A(T::size());
        gtsam::Vector b;
        if(fej)
        {
            gtsam::Values fejVals = fej->buildValues(T::keys(), x);
            -T::unwhitenedError(fejVals, A); // compute derivatives with first estimates values.
            b = -T::unwhitenedError(x); // Compute residual with current values.
        }else
        {
            b = -T::unwhitenedError(x, A);
        }

        // Whiten the corresponding system now
        if(T::noiseModel_)
            T::noiseModel_->WhitenSystem(A, b);

        // Fill in terms, needed to create JacobianFactor below
        std::vector<std::pair<gtsam::Key, gtsam::Matrix> > terms(T::size());
        for(size_t j = 0; j < T::size(); ++j)
        {
            terms[j].first = T::keys()[j];
            terms[j].second.swap(A[j]);
        }

        // TODO pass unwhitened + noise model to Gaussian factor
        using gtsam::noiseModel::Constrained;
        if(T::noiseModel_ && T::noiseModel_->isConstrained())
            return gtsam::GaussianFactor::shared_ptr(
                    new gtsam::JacobianFactor(terms, b,
                                              boost::static_pointer_cast<Constrained>(T::noiseModel_)->unit()));
        else
            return gtsam::GaussianFactor::shared_ptr(new gtsam::JacobianFactor(terms, b));
    }

protected:
    std::shared_ptr<FEJValues> fej;
};

}
#endif //DMVIO_FEJNOISEMODELFACTOR_H
