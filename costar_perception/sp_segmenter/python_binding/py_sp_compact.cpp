/**************
 * This is a python binding for SpCompact
 * Felix Jonathan
 * 11/18/2016
**************/
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "sp_segmenter/sp_compact.h"
#include "py_input_to_cpp_utils.h"

using namespace boost::python;


BOOST_PYTHON_MODULE(SpCompactPy)
{
    iterable_converter()
        // Build-in type.
        .from_python<std::vector<double> >()
        // Each dimension needs to be convertable.
        .from_python<std::vector<std::string> >()
    ;

    class_<SpCompact>("SpCompact")
        .def("startTrainingSVM", &SpCompact::startTrainingSVM)

        .def("setInputPathSIFT", &SpCompact::setInputPathSIFT)
        .def("setInputPathSHOT", &SpCompact::setInputPathSHOT)
        .def("setInputPathFPFH", &SpCompact::setInputPathFPFH)

        .def("setUseSHOT", &SpCompact::setUseSHOT)
        .def("setUseFPFH", &SpCompact::setUseFPFH)
        .def("setUseSIFT", &SpCompact::setUseSIFT)

        .def("setInputTrainingPath", &SpCompact::setInputTrainingPath)

        .def("setObjectNames", &SpCompact::setObjectNames)
        .def("setBackgroundNames", &SpCompact::setBackgroundNames)

        .def("setOutputDirectoryPathFEA", &SpCompact::setOutputDirectoryPathFEA)
        .def("setOutputDirectoryPathSVM", &SpCompact::setOutputDirectoryPathSVM)

        .def("setForegroundCC", &SpCompact::setForegroundCC<double>)
        .def("setForegroundCC", &SpCompact::setForegroundCC<float>)
        .def("setMultiCC", &SpCompact::setMultiCC<double>)
        .def("setMultiCC", &SpCompact::setMultiCC<float>)
        
        .def("setBackgroundSampleNumber", &SpCompact::setBackgroundSampleNumber)
        .def("setObjectSampleNumber", &SpCompact::setObjectSampleNumber)

        .def("setCurOrderMax", &SpCompact::setCurOrderMax)
        .def("setSkipFeaExtraction", &SpCompact::setSkipFeaExtraction)
        .def("setSkipBackgroundSVM", &SpCompact::setSkipBackgroundSVM)
        .def("setSkipMultiSVM", &SpCompact::setSkipMultiSVM)
    ;
}