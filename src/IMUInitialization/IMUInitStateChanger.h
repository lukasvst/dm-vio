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

#ifndef DMVIO_IMUINITSTATECHANGER_H
#define DMVIO_IMUINITSTATECHANGER_H

#include <memory>
#include <shared_mutex>

namespace dmvio
{

class IMUInitializerState;

// Provides methods to change the state.
class IMUInitStateChanger
{
public:
    // Can directly be called to change the state.
    virtual void lockAndSetState(std::unique_ptr<IMUInitializerState>&& newState) = 0;

    // Aquire lock for calling setState.
    virtual std::unique_lock<std::shared_timed_mutex> acquireSetStateLock() = 0;
    // Before calling setState a lock must be acquired with the previous method!
    virtual void setState(std::unique_ptr<IMUInitializerState>&& newState) = 0;
};

}

#endif //DMVIO_IMUINITSTATECHANGER_H
