#include "SpeechToText.h"
#include <SphinxLib.h>
#include <sofa/core/ObjectFactory.h>


#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/HapticDeviceEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

#include <sofa/simulation/common/Node.h>


#include <sofa/core/objectmodel/BaseObject.h>
#include <SofaBaseVisual/InteractiveCamera.h>

#ifndef WIN32
#  include <pthread.h>
#else
#  include <boost/thread/thread.hpp>
#  include <boost/thread/mutex.hpp>
#endif

#ifdef WIN32
#  include <windows.h>
#endif

namespace sofa
{
	namespace component
	{

		namespace controller{

			using sofa::component::controller::Controller;
			using namespace sofa::core::objectmodel;

			SOFA_DECL_CLASS(SpeechToText, sofa::core::objectmodel::BaseObject)

				int SpeechToTextClass = sofa::core::RegisterObject("Class to allow voice commands for camera control")
				.add<SpeechToText>()
				;

			SpeechToText::SpeechToText()
			{
				std::cout << "in SpeechToText::constructor" << std::endl;
				this->f_listening.setValue(true);
				STTThreadCreated = false;
			}

			SpeechToText::~SpeechToText()
			{
			}

			void SpeechToText::init()
			{
				std::cout << "in SpeechToText::init" << std::endl;

				pos_x = pos_y = 0;

				groot = dynamic_cast<sofa::simulation::Node *>(this->getContext()->getRootContext()); // access to current node
				currentCamera = this->getContext()->get<component::visualmodel::InteractiveCamera>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

				if (!currentCamera)
				{
					std::cout << "Camera is NULL" << std::endl;
					currentCamera = this->getContext()->get<component::visualmodel::InteractiveCamera>();
				}
				if (!currentCamera)
				{
					currentCamera = sofa::core::objectmodel::New<component::visualmodel::InteractiveCamera>();
					currentCamera->setName(core::objectmodel::Base::shortName(currentCamera.get()));
					groot->addObject(currentCamera);
					currentCamera->p_position.forceSet();
					currentCamera->p_orientation.forceSet();
					currentCamera->bwdInit();
				}
				if (!currentCamera)
				{
					serr << "Cannot find or create Camera." << sendl;
				}

				//Gets the current working directory
				TCHAR NPath[MAX_PATH];
				int bytes = GetModuleFileName(NULL, NPath, MAX_PATH);
				if (bytes == 0)
				{
					std::cout << "Unable to find current directory!" << std::endl;
				}
				else
				{
					std::wstring path(NPath);
					std::string str(path.begin(), path.end());
					std::cout << "path " << str << std::endl;
					if (str.find("\\Debug") != std::string::npos)
					{
						base_path = str.substr(0, str.find("\\bin\\Debug")).append("\\bin\\Debug");
					}
					else if (str.find("\\Release") != std::string::npos)
					{
						base_path = str.substr(0, str.find("\\bin\\Release")).append("\\bin\\Release");
					}
					else
					{
						base_path = str.substr(0, str.find("\\bin")).append("\\bin");
					}
					std::cout << "base path " << base_path << std::endl;
				}
			}

			void *SpeechToTextExecute(void *ptr, sofa::component::visualmodel::BaseCamera::SPtr currentCamera, std::string base_path, bool* stopCameraMotion, bool* enableSpeechRec)
			{

				std::cout << "\nIn SpeechToTextExecute Thread\n" << std::endl;

				SpeechToText *speechtotext = (SpeechToText*)ptr;
				asr_result result;
				
				while (true)
				{
					result = recognize_from_mic((base_path + "\\model\\en-us\\en-us").c_str(), (base_path + "\\model\\en-us\\en-us.lm.bin").c_str(),
						(base_path + "\\model\\en-us\\laparoscopicCamera.dict").c_str());
					if (strcmp(result.hyp, "") != 0)
					{
						if (*enableSpeechRec) {
							std::cout << "Camera Enabled" << std::endl;
							if (strcmp(result.hyp, "camera left") == 0)
							{
								speechtotext->setMoveMode(speechtotext->RIGHT);
								speechtotext->setMoveCount(100);
							}
						else if (strcmp(result.hyp, "right") == 0)
							{
								speechtotext->setMoveMode(speechtotext->LEFT);
								speechtotext->setMoveCount(100);
							}
						else if (strcmp(result.hyp, "up") == 0)
							{
								speechtotext->setMoveMode(speechtotext->DOWN);
								speechtotext->setMoveCount(100);
							}
						else if (strcmp(result.hyp, "down") == 0)
							{
								speechtotext->setMoveMode(speechtotext->UP);
								speechtotext->setMoveCount(100);
							}
						else if (strcmp(result.hyp, "slightly left") == 0)
							{
								speechtotext->setMoveMode(speechtotext->SLIGHTLY_RIGHT);
								speechtotext->setMoveCount(50);
							}
						else if (strcmp(result.hyp, "slightly right") == 0)
							{
								speechtotext->setMoveMode(speechtotext->SLIGHTLY_LEFT);
								speechtotext->setMoveCount(50);
							}
						else if (strcmp(result.hyp, "slightly up") == 0)
							{
								speechtotext->setMoveMode(speechtotext->SLIGHTLY_DOWN);
								speechtotext->setMoveCount(50);
							}
						else if (strcmp(result.hyp, "slightly down") == 0)
							{
								speechtotext->setMoveMode(speechtotext->SLIGHTLY_UP);
								speechtotext->setMoveCount(50);
							}
						else if (strcmp(result.hyp, "zoom in") == 0)
							{
								speechtotext->setMoveMode(speechtotext->ZOOM_IN);
								speechtotext->setMoveCount(100);
							}
						else if (strcmp(result.hyp, "zoom out") == 0)
							{
								speechtotext->setMoveMode(speechtotext->ZOOM_OUT);
								speechtotext->setMoveCount(100);
							}
							else if (strcmp(result.hyp, "stop") == 0)
							{
								*stopCameraMotion = true;
							}
							else if (strcmp(result.hyp, "camera disable") == 0)
							{
								*enableSpeechRec = false;
								*stopCameraMotion = true;
							}
							else
							{
								speechtotext->setMoveMode(speechtotext->UNRECOGNIZED);
							}
						}
						else
						{
							std::cout << "Camera Disabled. ";
							if (strcmp(result.hyp, "camera enable") == 0)
							{
								std::cout << "Enabling..." << std::endl;
								*enableSpeechRec = true;
							}
							else
							{
								speechtotext->setMoveMode(speechtotext->UNRECOGNIZED);
							}
						}
					}
				}
#ifndef WIN32
				pthread_exit(0);
#else
				return 0;
#endif
			}

			void SpeechToText::bwdInit()
			{
				std::cout << "In SpeechToText::bwdInit" << std::endl;
				if (STTThreadCreated)
				{
					serr << "Emulating thread already running" << sendl;

#ifndef WIN32
					int err = pthread_cancel(hapSimuThread);

					// no error: thread cancel
					if (err == 0)
					{
						std::cout << "SpeechToText: thread cancel" << std::endl;
					}

					// error
					else
					{
						std::cout << "thread not cancel = " << err << std::endl;
					}
#endif
				}
				//pthread_t hapSimuThread;

				if (thTimer == NULL)
					thTimer = new(helper::system::thread::CTime);

#ifndef WIN32
				if (pthread_create(&hapSimuThread, NULL, SpeechToTextExecute, (void*)this) == 0)
				{
					std::cout << "OmniDriver : Thread created for Omni simulation" << std::endl;
					omniSimThreadCreated = true;
				}
#else
				boost::thread STTThread(SpeechToTextExecute, this, currentCamera, base_path, &stopCameraMotion, &enableSpeechRec);
				setSTTThreadCreated(true);
#endif
			}

			void SpeechToText::handleEvent(core::objectmodel::Event *event)
			{
				if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
				{
					if (moveCount > 0)
					{
						sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::RightPressed, pos_x, pos_y);
						currentCamera->manageEvent(&me);

						switch (moveMode)
						{
						case LEFT:
						{
							pos_x -= 4;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
						}
						break;
						case RIGHT:
						{
							pos_x += 4;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
						}
						break;
						case UP:
						{
							pos_y -= 4;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
						}
						break;
						case DOWN:
						{
							pos_y += 4;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
						}
						break;
						case SLIGHTLY_LEFT:
						{
							pos_x -= 2;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
						}
						break;
						case SLIGHTLY_RIGHT:
						{
							pos_x += 2;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
						}
						break;
						case SLIGHTLY_UP:
						{
							pos_y -= 2;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
						}
						break;
						case SLIGHTLY_DOWN:
						{
							pos_y += 2;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
						}
						break;
						case ZOOM_IN:
						{
							wheelDelta = 3;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Wheel, wheelDelta);
							currentCamera->manageEvent(&me2);
						}
						break;
						case ZOOM_OUT:
						{
							wheelDelta = -3;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Wheel, wheelDelta);
							currentCamera->manageEvent(&me2);
						}
						break;
						case UNRECOGNIZED:
						{
							std::cout << "Did not understand command!" << std::endl;
							moveCount = 0;
						}
						break;
						default:
							std::cout << "Did not understand command!" << std::endl;
							moveCount = 0;
						}

						sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::RightReleased, pos_x, pos_y);
						currentCamera->manageEvent(&me3);

						moveCount--;
					}

				}
			}

		} // namespace controller

	} // namespace component

} // namespace sofa