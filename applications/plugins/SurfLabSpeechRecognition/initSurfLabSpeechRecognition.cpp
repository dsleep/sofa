#include "initSurfLabSpeechRecognition.h"

namespace sofa
{
	namespace component
	{
		//Here are just several convenient functions to help user to know what contains the plugin

		extern "C" {
			SOFA_SURFLABSPEECHRECOGNITION_API void initExternalModule();
			SOFA_SURFLABSPEECHRECOGNITION_API const char* getModuleName();
			SOFA_SURFLABSPEECHRECOGNITION_API const char* getModuleVersion();
			SOFA_SURFLABSPEECHRECOGNITION_API const char* getModuleLicense();
			SOFA_SURFLABSPEECHRECOGNITION_API const char* getModuleDescription();
			SOFA_SURFLABSPEECHRECOGNITION_API const char* getModuleComponentList();
		}

		void initExternalModule(){
			// Initialization code
			std::cout << "\nInitializing SpeechRecognition\n" << std::endl;
		}

		const char* getModuleName(){
			return "SurfLab Speech Recognition";
		}

		const char* getModuleVersion(){
			return "0.1";
		}

		const char* getModuleLicense(){
			return "None";
		}

		const char* getModuleDescription(){
			return "Allows voice commands to control camera movement and switch instruments";
		}

		const char* getModuleComponentList(){
			// Comma-separated list of compenents in this plugin
			return "SurfLabSpeechRecognition";
		}
	}
}

SOFA_LINK_CLASS(SpeechToText)