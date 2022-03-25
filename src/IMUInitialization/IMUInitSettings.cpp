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

#include "util/SettingsUtil.h"
#include "IMUInitSettings.h"

using namespace dmvio;


void IMUInitSettings::registerArgs(dmvio::SettingsUtil& set, std::string prefix)
{
    set.registerArg(prefix + "transitionModel", transitionModel);
    set.registerArg(prefix + "onlyKFs", onlyKFs);
    set.registerArg(prefix + "coarseScaleUncertaintyThresh", coarseScaleUncertaintyThresh);
    set.registerArg(prefix + "initDSOParams", initDSOParams);
    set.registerArg(prefix + "scalePriorAfterInit", scalePriorAfterInit);
    set.registerArg(prefix + "disableVIOUntilFirstInit", disableVIOUntilFirstInit);
    set.registerArg(prefix + "multithreadedInitDespiteNonRT", multithreadedInitDespiteNonRT);

    transformPriors.registerArgs(set, prefix);
    coarseInitSettings.registerArgs(set, prefix);
    pgbaSettings.registerArgs(set, prefix + "pgba_");

    thresholdSettings.threshScale = 1.02;
    thresholdSettings.registerArgs(set, prefix);
    secondThresholdSettings.threshScale = 10000000.0;
    secondThresholdSettings.registerArgs(set, prefix + "second");
}

void CoarseIMUInitOptimizerSettings::registerArgs(dmvio::SettingsUtil& set, std::string prefix)
{
    set.registerArg(prefix + "maxNumPoses", maxNumPoses);
    set.registerArg(prefix + "fixPoses", fixPoses);
    set.registerArg(prefix + "multipleBiases", multipleBiases);

    set.registerArg(prefix + "priorRotSigma", priorRotSigma);
    set.registerArg(prefix + "priorTransSigma", priorTransSigma);

    set.registerArg(prefix + "lambdaLowerBound", lambdaLowerBound);

    set.registerArg(prefix + "conversionType", conversionType);

    set.registerArg(prefix + "updatePoses", updatePoses);

    set.registerArg(prefix + "requestFullResetErrorThreshold", requestFullResetErrorThreshold);
    set.registerArg(prefix + "requestFullResetNormalizedErrorThreshold", requestFullResetNormalizedErrorThreshold);
}

void PGBASettings::registerArgs(dmvio::SettingsUtil& set, std::string prefix)
{
    set.registerArg(prefix + "delay", delay);
    set.registerArg(prefix + "scaleUncertaintyThresh", scaleUncertaintyThresh);
    set.registerArg(prefix + "reinitScaleUncertaintyThresh", reinitScaleUncertaintyThresh);
    set.registerArg(prefix + "skipFirstKFs", skipFirstKFs);
    set.registerArg(prefix + "conversionType", conversionType);

    set.registerArg(prefix + "prepareGraphAddFactors", prepareGraphAddFactors);
    set.registerArg(prefix + "prepareGraphAddDelValues", prepareGraphAddDelValues);

    transformPriors.registerArgs(set, prefix);
}

void IMUThresholdSettings::registerArgs(dmvio::SettingsUtil& set, std::string prefix)
{
    set.registerArg(prefix + "threshScale", threshScale);
    set.registerArg(prefix + "threshGravdir", threshGravdir);
}
