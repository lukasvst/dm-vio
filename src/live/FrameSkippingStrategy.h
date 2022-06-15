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

#ifndef DMVIO_FRAMESKIPPINGSTRATEGY_H
#define DMVIO_FRAMESKIPPINGSTRATEGY_H

#include "dso/IOWrapper/Output3DWrapper.h"
#include "util/SettingsUtil.h"
#include <mutex>

namespace dmvio
{

// Settings for frame skipping.
class FrameSkippingSettings
{
public:
    void registerArgs(dmvio::SettingsUtil& set);

    // Maximum frames to skip ...
    int maxSkipFramesVisualInit = 0;        // ... during visual initializer phase.
    int maxSkipFramesVisualOnlyMode = 1;    // ... during visual only mode.
    int maxSkipFramesVisualInertial = 2;    // ... during visual-inertial mode.
    int maxSkipFramesFullReset = -1;        // ... when a full reset happens.
    // -1 means that all frames until the newest one are skipped (resembling a value of infinity).

    // After visual initializer wait this amount if frames before switching to the visualOnly threshold.
    int skipFramesVisualOnlyDelay = 30;

    // Don't skip if the queue size is < this value.
    int minQueueSizeForSkipping = 2;
};

// Contains the logic to decide how many frames to skip (depending on the current state of the system).
class FrameSkippingStrategy : public dso::IOWrap::Output3DWrapper
{
public:
    FrameSkippingStrategy(FrameSkippingSettings settings);

    // Get current maxSkipFrames according to strategy.
    // queueSize is the number of frames currently in the image queue.
    int getMaxSkipFrames(int queueSize);

    // Overidden from Output3DWrapper
    void publishSystemStatus(dmvio::SystemStatus systemStatus) override;
    void reset() override;


private:
    FrameSkippingSettings settings;

    std::mutex mutex;
    SystemStatus lastStatus;

    bool resetLast = false;

    int visualOnlyDelay = 0;
};


}

#endif //DMVIO_FRAMESKIPPINGSTRATEGY_H
