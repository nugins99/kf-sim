#pragma once

#include <boost/property_tree/ptree.hpp>
#include "FileOutput.h"
#include "MultiOutput.h"
#include "OutputInterface.h"
#include "ProgressReporter.h"

std::shared_ptr<OutputInterface> createOutputFromConfig(const boost::property_tree::ptree& pt,
                                                        int steps)
{
    auto multi = std::make_shared<MultiOutput>();
    bool file_enabled = pt.get<bool>("output.file", true);
    bool progress_enabled = pt.get<bool>("output.progress", true);
    if (file_enabled)
    {
        multi->addOutput(
            std::make_shared<FileOutput>(pt.get<std::string>("output.filename", "output.txt")));
    }
    if (progress_enabled)
    {
        multi->addOutput(std::make_shared<ProgressReporter>(steps));
    }
    // Add more outputs here as needed
    return multi;
}