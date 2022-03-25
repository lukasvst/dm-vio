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

#include "SettingsUtil.h"

dmvio::SettingsUtil::Parameter::Parameter(void* pointer,
                                          const std::function<void(void*, std::string)>& commandLineHandler,
                                          const std::function<void(void*, const YAML::Node&)>& yamlHandler,
                                          const std::function<void(void*, std::ostream&)>& printHandler) : pointer(
        pointer), commandLineHandler(commandLineHandler), yamlHandler(yamlHandler), printHandler(printHandler)
{}

void dmvio::SettingsUtil::tryReadFromYaml(const YAML::Node& node)
{
    // Loop through parameters and check if their name is in the node.
    for(auto& pair : parameters)
    {
        // We give preference to commandline over yaml.
        if(!pair.second.loadedFromCommandLine)
        {
            std::string name = pair.first;
            if(node[name])
            {
                pair.second.yamlHandler(pair.second.pointer, node[name]);
            }
        }
    }
}

bool dmvio::SettingsUtil::tryReadFromCommandLine(const std::string& arg)
{
    // Extract name as part before the = sign and lookup in map
    auto pos = arg.find('=');
    if(pos == std::string::npos)
    {
        return false;
    }
    std::string name = arg.substr(0, pos);

    auto it = parameters.find(name);
    if(it == parameters.end())
    {
        return false;
    }

    it->second.commandLineHandler(it->second.pointer, arg.substr(pos + 1));
    it->second.loadedFromCommandLine = true;

    return true;
}

void dmvio::SettingsUtil::printAllSettings(std::ostream& stream)
{
    for(auto& pair : parameters)
    {
        stream << pair.first << ": ";
        pair.second.printHandler(pair.second.pointer, stream);
        stream << '\n';
    }
}

void dmvio::SettingsUtil::createPangolinSettings()
{
    for(auto&& param : parameters)
    {
        auto* set = param.second.pangolinSetting.get();
        if(set)
        {
            set->createVar();
        }
    }
}

void dmvio::SettingsUtil::updatePangolinSettings()
{
    for(auto&& param : parameters)
    {
        auto* set = param.second.pangolinSetting.get();
        if(set)
        {
            set->updateVar();
        }
    }
}
