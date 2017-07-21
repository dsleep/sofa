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
#  include <boost/date_time/posix_time/posix_time.hpp> 
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

				const int default_translate_count = 3;
				const int default_rotate_count = 5;
				const int default_zoom_count = 4;
				
				while (true)
				{
					sleep_msec(500);//wants to add sleeping time between two commands in order to reduce the cost
					result = recognize_from_mic((base_path + "\\model\\en-us\\en-us").c_str(), (base_path + "\\model\\en-us\\en-us.lm.bin").c_str(),
						(base_path + "\\model\\en-us\\laparoscopicCamera.dict").c_str());
					//std::cout << "got one result!!" << std::endl;
					if (strcmp(result.hyp, "") != 0)
					{
						if (enableSpeechRec) 
						{
							//std::cout << "Camera Enabled" << std::endl;
							std::string result_str(result.hyp);
							if (result_str.find("camera") != std::string::npos)
							{
								/* TRANSLATIONS */
								if (strcmp(result.hyp, "camera left") == 0)
								{
									speechtotext->setMoveMode(speechtotext->RIGHT);
									speechtotext->setMoveCount(default_translate_count);
								}
						else if (strcmp(result.hyp, "right") == 0)
								{
									speechtotext->setMoveMode(speechtotext->LEFT);
									speechtotext->setMoveCount(default_translate_count);
								}
						else if (strcmp(result.hyp, "up") == 0)
								{
									speechtotext->setMoveMode(speechtotext->DOWN);
									speechtotext->setMoveCount(default_translate_count);
								}
						else if (strcmp(result.hyp, "down") == 0)
								{
									speechtotext->setMoveMode(speechtotext->UP);
									speechtotext->setMoveCount(default_translate_count);
								}
						else if (strcmp(result.hyp, "slightly left") == 0)
								{
									speechtotext->setMoveMode(speechtotext->SLIGHTLY_RIGHT);
									speechtotext->setMoveCount(default_translate_count / 2);
								}
						else if (strcmp(result.hyp, "slightly right") == 0)
								{
									speechtotext->setMoveMode(speechtotext->SLIGHTLY_LEFT);
									speechtotext->setMoveCount(default_translate_count / 2);
								}
						else if (strcmp(result.hyp, "slightly up") == 0)
								{
									speechtotext->setMoveMode(speechtotext->SLIGHTLY_DOWN);
									speechtotext->setMoveCount(default_translate_count / 2);
								}
						else if (strcmp(result.hyp, "slightly down") == 0)
								{
									speechtotext->setMoveMode(speechtotext->SLIGHTLY_UP);
									speechtotext->setMoveCount(default_translate_count / 2);
								}				
								/*else if (strcmp(result.hyp, "stop") == 0 || strcmp(result.hyp, "camera stop") == 0)
								{
									*stopCameraMotion = true;
								}*/
								else if (strcmp(result.hyp, "camera disable") == 0)
								{
									*enableSpeechRec = false;
									*stopCameraMotion = true;
								}
							}
							else if (result_str.find("zoom") != std::string::npos)
							{
								/* ZOOM */
								if (result_str.find("zoom in slightly") != std::string::npos)
								{
									*stopCameraMotion = false;
									speechtotext->setMoveMode(speechtotext->ZOOM_IN);
									speechtotext->setMoveCount(default_zoom_count/2);
								}
								else if (result_str.find("zoom out slightly") != std::string::npos || result_str.find("zoom up slightly") != std::string::npos)
								{
									*stopCameraMotion = false;
									speechtotext->setMoveMode(speechtotext->ZOOM_OUT);
									speechtotext->setMoveCount(default_zoom_count/2);
								}
								else if (result_str.find("zoom in") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->ZOOM_IN);
									speechtotext->setMoveCount(default_zoom_count);
								}
								else if (result_str.find("zoom out") != std::string::npos || result_str.find("zoom up") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->ZOOM_OUT);
									speechtotext->setMoveCount(default_zoom_count);
								}

							}
							else if (result_str.find("rotate") != std::string::npos)
							{
								/* ROTATIONS/PIVOTS */
								if (result_str.find("left") != std::string::npos)
								{
									*stopCameraMotion = false;
									speechtotext->setMoveMode(speechtotext->ROTATE_RIGHT);
									speechtotext->setMoveCount(default_rotate_count);
								}
								else if (result_str.find("right") != std::string::npos)
								{
									*stopCameraMotion = false;
									speechtotext->setMoveMode(speechtotext->ROTATE_LEFT);
									speechtotext->setMoveCount(default_rotate_count);
								}
								else if (result_str.find("down") != std::string::npos)
								{
									*stopCameraMotion = false;
									speechtotext->setMoveMode(speechtotext->ROTATE_DOWN);
									speechtotext->setMoveCount(default_rotate_count);
								}
								else if (result_str.find("up") != std::string::npos)
								{
									*stopCameraMotion = false;
									speechtotext->setMoveMode(speechtotext->ROTATE_UP);
									speechtotext->setMoveCount(default_rotate_count);
								}
							}
							else if (result_str.find("switch") != std::string::npos)
							{	
								//Switch instruments
								if (result_str.find("caught") != std::string::npos
									|| result_str.find("out right") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->SWITCH_TO_CAUTERIZER);
									speechtotext->setMoveCount(1);
								}
								else if (result_str.find("grasp") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->SWITCH_TO_GRASPER);
									speechtotext->setMoveCount(1);
								}
								else if (result_str.find("scissor") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->SWITCH_TO_SCISSOR);
									speechtotext->setMoveCount(1);
								}
								else if (result_str.find("maryland") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->SWITCH_TO_DISSECTOR);
									speechtotext->setMoveCount(1);
								}
								else if (result_str.find("bag") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->SWITCH_TO_BAG);
									speechtotext->setMoveCount(1);
								}
								else if (result_str.find("clip") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->SWITCH_TO_CLAMP);
									speechtotext->setMoveCount(1);
								}
								else if (result_str.find("scalpel") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->SWITCH_TO_SCALPEL);
									speechtotext->setMoveCount(1);
								}
								else if (result_str.find("stapler") != std::string::npos ||
									result_str.find("endo") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->SWITCH_TO_STAPLER);
									speechtotext->setMoveCount(1);
								}
								else if (result_str.find("tract") != std::string::npos
									|| result_str.find("actor") != std::string::npos || result_str.find("rich") != std::string::npos)
								{
									speechtotext->setMoveMode(speechtotext->SWITCH_TO_RETRACTOR);
									speechtotext->setMoveCount(1);
								}
								else
								{
									std::cout << "i don't understand your command." << std::endl;
									speechtotext->setMoveMode(speechtotext->UNRECOGNIZED);
								}
							}
							else
							{
								std::cout << "i don't understand your command." << std::endl;
								speechtotext->setMoveMode(speechtotext->UNRECOGNIZED);
							}
						}
						else
						{
							std::cout << "Camera Disabled. ";
							if (strcmp(result.hyp, "camera enable") == 0 || strcmp(result.hyp, "enable") == 0)
							{
								std::cout << "Camera Enabled" << std::endl;
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
						switch (moveMode)
						{
							std::cout << "in switch movemode: " << std::endl;
							/* TRANSLATIONS */
						case LEFT:
						{
							//std::cout << "last execute time: " << groot->getTime() << std::endl;
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::RightPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_x -= 10;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::RightReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case RIGHT:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::RightPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_x += 10;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::RightReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case UP:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::RightPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_y -= 10;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::RightReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case DOWN:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::RightPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_y += 10;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::RightReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case SLIGHTLY_LEFT:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::RightPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_x -= 4;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::RightReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case SLIGHTLY_RIGHT:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::RightPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_x += 4;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::RightReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case SLIGHTLY_UP:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::RightPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_y -= 4;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::RightReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case SLIGHTLY_DOWN:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::RightPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_y += 4;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::RightReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;

						/* ROTATIONS/PIVOTS */
						case ROTATE_LEFT:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::LeftPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_x -= 10;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::LeftReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case ROTATE_RIGHT:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::LeftPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_x += 10;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::LeftReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case ROTATE_UP:
						{
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::LeftPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_y -= 10;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::LeftReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;
						case ROTATE_DOWN:
						{
							//pos_y += 6;
							sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::LeftPressed, pos_x, pos_y);
							currentCamera->manageEvent(&me);
							pos_y += 10;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Move, pos_x, pos_y);
							currentCamera->manageEvent(&me2);
							sofa::core::objectmodel::MouseEvent me3(sofa::core::objectmodel::MouseEvent::LeftReleased, pos_x, pos_y);
							currentCamera->manageEvent(&me3);
						}
						break;

						/* ZOOM */
						case ZOOM_IN:
						{
							wheelDelta = 6;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Wheel, wheelDelta);
							currentCamera->manageEvent(&me2);
						}
						break;
						case ZOOM_OUT:
						{
							wheelDelta = -6;
							sofa::core::objectmodel::MouseEvent me2(sofa::core::objectmodel::MouseEvent::Wheel, wheelDelta);
							currentCamera->manageEvent(&me2);
						}
						break;
						case SWITCH_TO_CAUTERIZER:
						{
							Vector3 dummyVector;
							Quat dummyQuat;
							sofa::core::objectmodel::HapticDeviceEvent event_switch(0, dummyVector, dummyQuat, 3);
							//simulation::Node *groot = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext()); 
							groot->propagateEvent(core::ExecParams::defaultInstance(), &event_switch);
							//boost::this_thread::sleep(boost::posix_time::milliseconds(100));
							//thTimer->sleep(1);
						}
						break;
						case SWITCH_TO_GRASPER:
						{
							Vector3 dummyVector;
							Quat dummyQuat;
							sofa::core::objectmodel::HapticDeviceEvent event_switch(0, dummyVector, dummyQuat, 4);
							//simulation::Node *groot = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext());
							groot->propagateEvent(core::ExecParams::defaultInstance(), &event_switch);
							//std::cout << "timer.gettime:" << thTimer->getFastTime() << std::endl;
							//boost::this_thread::sleep(boost::posix_time::milliseconds(100));
							//thTimer->sleep(1);
						}
						break;
						case SWITCH_TO_CLAMP:
						{
							Vector3 dummyVector;
							Quat dummyQuat;
							sofa::core::objectmodel::HapticDeviceEvent event_switch(0, dummyVector, dummyQuat, 5);
							//simulation::Node *groot = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext());
							groot->propagateEvent(core::ExecParams::defaultInstance(), &event_switch);
							//boost::this_thread::sleep(boost::posix_time::milliseconds(100));
							//thTimer->sleep(1);
						}
						break;
						case SWITCH_TO_SCISSOR:
						{
							Vector3 dummyVector;
							Quat dummyQuat;
							sofa::core::objectmodel::HapticDeviceEvent event_switch(0, dummyVector, dummyQuat, 6);
							//simulation::Node *groot = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext());
							groot->propagateEvent(core::ExecParams::defaultInstance(), &event_switch);
							//boost::this_thread::sleep(boost::posix_time::milliseconds(100));
							//thTimer->sleep(1);
						}
						break; 
						case SWITCH_TO_DISSECTOR:
						{
							Vector3 dummyVector;
							Quat dummyQuat;
							sofa::core::objectmodel::HapticDeviceEvent event_switch(0, dummyVector, dummyQuat, 7);
							//simulation::Node *groot = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext());
							groot->propagateEvent(core::ExecParams::defaultInstance(), &event_switch);
							//boost::this_thread::sleep(boost::posix_time::milliseconds(100));
							//thTimer->sleep(1);
						}
						break; 
						case SWITCH_TO_STAPLER:
						{
							Vector3 dummyVector;
							Quat dummyQuat;
							sofa::core::objectmodel::HapticDeviceEvent event_switch(0, dummyVector, dummyQuat, 8);
							//simulation::Node *groot = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext());
							groot->propagateEvent(core::ExecParams::defaultInstance(), &event_switch);
							//thTimer->sleep(1);
						}
						break;
						case SWITCH_TO_RETRACTOR:
						{
							Vector3 dummyVector;
							Quat dummyQuat;
							sofa::core::objectmodel::HapticDeviceEvent event_switch(0, dummyVector, dummyQuat, 9);
							//simulation::Node *groot = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext());
							groot->propagateEvent(core::ExecParams::defaultInstance(), &event_switch);
							//thTimer->sleep(1);
						}
						break;
						case SWITCH_TO_SCALPEL:
						{
							Vector3 dummyVector;
							Quat dummyQuat;
							sofa::core::objectmodel::HapticDeviceEvent event_switch(0, dummyVector, dummyQuat, 10);
							//simulation::Node *groot = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext());
							groot->propagateEvent(core::ExecParams::defaultInstance(), &event_switch);
							//thTimer->sleep(1);
						}
						break;
						case SWITCH_TO_BAG:
						{
							Vector3 dummyVector;
							Quat dummyQuat;
							sofa::core::objectmodel::HapticDeviceEvent event_switch(0, dummyVector, dummyQuat, 11);
							//simulation::Node *groot = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext());
							groot->propagateEvent(core::ExecParams::defaultInstance(), &event_switch);
							//thTimer->sleep(1);
						}
						break;
						case UNRECOGNIZED:
						{
							std::cout << "Did not understand the command!" << std::endl;
							moveCount = 0;
						}
						break;
						default:{
							std::cout << "Did not understand the command!" << std::endl;
							moveCount = 0;
							}
						}//switch

						moveCount--;
					}//if

				}//if

			}//SpeechToText

		} // namespace controller

	} // namespace component

} // namespace sofa