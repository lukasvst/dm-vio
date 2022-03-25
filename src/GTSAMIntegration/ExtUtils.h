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

#ifndef DMVIO_EXTUTILS_H
#define DMVIO_EXTUTILS_H

#include <math.h>
#include <iostream>
#include <assert.h>

#ifdef STACKTRACE
#include <boost/stacktrace.hpp>
#endif

namespace dmvio
{

template<typename T> void assertAlmostEq(const T& first, const T& second, double epsilon = 0.00001)
{
    T diff = std::abs(first - second);
    if(diff > epsilon)
    {
        std::cout << "Error: Not Eq: " << diff << " first: " << first << " second" << second << std::endl;
#ifdef STACKTRACE
        std::cout << boost::stacktrace::stacktrace();
#endif
        assert(0);
    }

}

template<typename T> void assertEqEigen(const T& first, const T& second, double epsilon = 0.00001)
{
    T diff = first - second;
    if(!diff.isZero(epsilon))
    {
        std::cout << "Error: Not Eq:\n" << diff << "\nfirst:\n" << first << "\nsecond\n" << second << std::endl;
        std::cout << "Max diff: " << diff.cwiseAbs().maxCoeff() << std::endl;
#ifdef STACKTRACE
        std::cout << boost::stacktrace::stacktrace();
#endif
        assert(0);
    }
}


template<typename T> class MeanAccumulator
{
public:
    void add(const T& t)
    {
        sum += t;
        num++;
    }

    T getMean()
    {
        if(num == 0)
        {
            return 0;
        }
        return sum / num;
    }

private:
    T sum = 0.0;
    int num = 0;
};
typedef MeanAccumulator<double> MeanAccumulatorD;

}

#endif //DMVIO_EXTUTILS_H
