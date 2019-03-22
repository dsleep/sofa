#ifndef INITSURFLABSPEECHRECOGNITION_H
#define INITSURFLABSPEECHRECOGNITION_H

#include <sofa/helper/system/config.h>
#include <sofa/simulation/Simulation.h>

#ifdef SOFA_BUILD_SURFLABSPEECHRECOGNITION
#define SOFA_SURFLABSPEECHRECOGNITION_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_SURFLABSPEECHRECOGNITION_API  SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

/** \mainpage
Contains components for SPEECH RECOGNITION FUNCTIONALITIES in Sofa
*/

#endif // INITSURFLABSPEECHRECOGNITION_H
