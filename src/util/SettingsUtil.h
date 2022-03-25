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

#ifndef DMVIO_SETTINGSUTIL_H
#define DMVIO_SETTINGSUTIL_H

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <functional>
#include <assert.h>
#include <pangolin/var/var.h>

// This file contains utils for settings.
// Most settings should be in (potentially nested) settings classes which.
// These can call SettingsUtil::registerArg to register their members as settings.
// Registered settings can be set with commandline arguments or with yaml-files (see methods of SettingsUtil).
// Can also create settings which can be modified in the Pangolin GUI.
namespace dmvio
{
template<typename T> void defaultCommandLineHandler(void* pointer, std::string arg)
{
    std::stringstream stream(arg);
    T* typedPointer = static_cast<T*>(pointer);
    if(!(stream >> *typedPointer))
    {
        std::cerr << "Could not convert argument: " << arg << std::endl;
        assert(0);
    }
}

template<typename T> void defaultYAMLHandler(void* pointer, const YAML::Node& node)
{
    T* typedPointer = static_cast<T*>(pointer);
    *typedPointer = node.as<T>();
}

template<typename T> void defaultPrintHandler(void* pointer, std::ostream& stream)
{
    T* typedPointer = static_cast<T*>(pointer);
    stream << *typedPointer;
}

// Class for a setting that can be set by the GUI in Pangolin.
class PangolinSettingVar
{
public:
    virtual ~PangolinSettingVar() = default;

    // Called in the Pangolin thread
    virtual void createVar() = 0; // Create Var
    virtual void updateVar() = 0; // Update value of var.
};

template<typename T> class PangolinSetting : public PangolinSettingVar
{
public:
    PangolinSetting(std::string name, T* pointer, bool toggle)
            : name(name), pointer(pointer), toggle(toggle), boolConstr(true)
    {}

    PangolinSetting(std::string name, T* pointer, double min, double max)
            : name(name), pointer(pointer), min(min), max(max), boolConstr(false)
    {}

    void createVar() override
    {
        if(boolConstr)
        {
            var.reset(new pangolin::Var<T>("ui." + name, *pointer, toggle));
        }else
        {
            var.reset(new pangolin::Var<T>("ui." + name, *pointer, min, max));
        }
    }

    void updateVar() override
    {
        *pointer = var->Get();
        assert(var);
    }

private:
    std::string name;
    std::unique_ptr<pangolin::Var<T>> var;
    T* pointer;
    bool boolConstr, toggle;
    double min, max;
};

class SettingsUtil
{
public:
    // Overwrite settings with the ones saved in the yaml file.
    // Note that for this we don't check that every element in the yaml file must be read, so typos are not checked.
    void tryReadFromYaml(const YAML::Node& node);

    // Set a parameter from a (single) commandline argument.
    // Returns true if the setting existed and was set.
    // Settings set from commandline have preference over ones set from yaml.
    bool tryReadFromCommandLine(const std::string& arg);

    // Register argument with the given name.
    template<typename T>
    void registerArg(std::string name, T& arg)
    {
        if(parameters.find(name) != parameters.end())
        {
            std::cerr << "ERROR: Trying to add parameter twice! " << name << std::endl;
            assert(0);
        }
        parameters.emplace(name,
                           Parameter(static_cast<void*>(&arg), defaultCommandLineHandler<T>, defaultYAMLHandler<T>,
                                     defaultPrintHandler<T>));
    }

    // The following 2 methods will also create a GUI item in Pangolin for the setting (either a toggle switch or a slider).
    template<typename T> void registerArg(std::string name, T& arg, bool toggle)
    {
        registerArg(name, arg);
        parameters.at(name).pangolinSetting.reset(new PangolinSetting<T>(name, &arg, toggle));
    }

    template<typename T> void registerArg(std::string name, T& arg, double min, double max)
    {
        registerArg(name, arg);
        parameters.at(name).pangolinSetting.reset(new PangolinSetting<T>(name, &arg, min, max));
    }

    // Dump all settings to file.
    void printAllSettings(std::ostream& stream);

    // Should be called from Pangolin thread.
    void createPangolinSettings();
    void updatePangolinSettings();

private:
    struct Parameter
    {
        Parameter(void* pointer, const std::function<void(void*, std::string)>& commandLineHandler,
                  const std::function<void(void*, const YAML::Node&)>& yamlHandler,
                  const std::function<void(void*, std::ostream&)>& printHandler);

        void* pointer;
        std::function<void(void*, std::string)> commandLineHandler;
        std::function<void(void*, const YAML::Node&)> yamlHandler;
        std::function<void(void*, std::ostream&)> printHandler;
        bool loadedFromCommandLine{
                false}; // Used to make sure that we don't overwrite parameters set using commandline when reading from yaml.

        std::unique_ptr<PangolinSettingVar> pangolinSetting;
    };
    std::map<std::string, Parameter> parameters;
};
}

#endif //DMVIO_SETTINGSUTIL_H
