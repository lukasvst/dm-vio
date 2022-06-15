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

#include "FrameSkippingStrategy.h"

void dmvio::FrameSkippingSettings::registerArgs(dmvio::SettingsUtil& set)
{
    set.registerArg("maxSkipFramesVisualInit", maxSkipFramesVisualInit);
    set.registerArg("maxSkipFramesVisualOnlyMode", maxSkipFramesVisualOnlyMode);
    set.registerArg("maxSkipFramesVisualInertial", maxSkipFramesVisualInertial);
    set.registerArg("maxSkipFramesFullReset", maxSkipFramesFullReset);
    set.registerArg("skipFramesVisualOnlyDelay", skipFramesVisualOnlyDelay);
    set.registerArg("minQueueSizeForSkipping", minQueueSizeForSkipping);
}

dmvio::FrameSkippingStrategy::FrameSkippingStrategy(dmvio::FrameSkippingSettings settings)
    : settings(std::move(settings))
{}

int dmvio::FrameSkippingStrategy::getMaxSkipFrames(int queueSize)
{
    std::unique_lock<std::mutex> lock(mutex);
    if(resetLast)
    {
        // After full reset skip all frames.
        resetLast = false;
        return settings.maxSkipFramesFullReset;
    }
    if(queueSize < settings.minQueueSizeForSkipping)
    {
        return 0;
    }
    switch(lastStatus)
    {
        case VISUAL_INIT:
            return settings.maxSkipFramesVisualInit;
        case VISUAL_ONLY:
            if(visualOnlyDelay > 0)
            {
                visualOnlyDelay--;
                return settings.maxSkipFramesVisualInit;
            }
            return settings.maxSkipFramesVisualOnlyMode;
        case VISUAL_INERTIAL:
            return settings.maxSkipFramesVisualInertial;
    }
}

void dmvio::FrameSkippingStrategy::publishSystemStatus(dmvio::SystemStatus systemStatus)
{
    std::unique_lock<std::mutex> lock(mutex);
    if(lastStatus == VISUAL_INIT && systemStatus == VISUAL_ONLY)
    {
        visualOnlyDelay = settings.skipFramesVisualOnlyDelay;
    }
    lastStatus = systemStatus;
}

void dmvio::FrameSkippingStrategy::reset()
{
    std::unique_lock<std::mutex> lock(mutex);
    resetLast = true;
}

