#include "ui_report.h"
#include "report.h"
#include <fstream>

namespace sofa
{
namespace gui
{
namespace qt
{
	SofaProcedureReport::SofaProcedureReport(QWidget *parent) :
		QScrollArea(parent),
		ui(new Ui_Report)
	{
		ui->setupUi(this);

		//obtain system date
		//time_t rawtime;
		//struct tm * timeinfo;
		//char buffer[80];

		//time(&rawtime);
		//timeinfo = localtime(&rawtime);
		//std::cout << "path.substr(0, path.find(examples))" << base_path_share << std::endl;
		//strftime(buffer, sizeof(buffer), "%d_%m_%Y-%I_%M_%S", timeinfo);
		//programStartDate = std::string(buffer);

		//---------- setting up directories ----------------------
		rootDirectory = QDir(QString::fromStdString(base_path_share));
		student = rootDirectory.homePath().section("/", 2);
		//sessionNum = QString::fromStdString(programStartDate);
		errDir = rootDirectory;
		succDir = rootDirectory;

		errDir.cd("Errors");
		succDir.cd("Achievements");
		/*QString edir = errDir.absolutePath();
		std::string erdir = edir.toStdString();
		std::cout << "error Dir: " << erdir << std::endl;
		QString sdir = succDir.absolutePath();
		std::string sudir = sdir.toStdString();
		std::cout << "success Dir: " << sudir << std::endl;*/
	}
	void SofaProcedureReport::populate(QString studentName, std::string programStartDate){
		programStartDate = programStartDate;
		sessionNum = QString::fromStdString(programStartDate);
		student = studentName;
		errList = errDir.entryList();
		succList = succDir.entryList();

    errCount = errList.size();
    succCount = succList.size();
    std::cout << "errCount: " << errCount << std::endl;
    std::cout << "succCount: " << succCount << std::endl;

    //--------EDIT LABELS--------------
    font.setPointSize(15);
    ui->StudentLabel->setFont(font);
    ui->StudentLabel->setText(student + "\'s Report: " + sessionNum);

    // ++++++++++++ loading images from directory for ERRORS
    for (int i = 2; i < errCount; i++)
    {
        if (errList.at(i).contains(sessionNum)) {
            errString.append(errList.at(i).toStdString());
            errString.append("\n");
            QLabel *img = new QLabel();
			//img->setPixmap(QPixmap::fromImage(QImage(errDir.absolutePath().append("/" + errList.at(i)))));
            QImage screenshot = QImage(errDir.absolutePath().append("/" + errList.at(i)));
            screenshot = screenshot.scaledToHeight(ui->ErrorArea->height(), Qt::FastTransformation);
            QPixmap scap = QPixmap::fromImage(screenshot);
            img->setPixmap(scap);
			img->setMinimumSize(480, 262);
			img->setMaximumSize(1920, 1046);
			//img->sizePolicy().hasWidthForHeight();
			img->setScaledContents(true);
            QLabel *name = new QLabel();
            name->setText(errList.at(i).section("_", 5));
            name->setFont(font);
            name->setStyleSheet("QLabel {  color : red; }");
            ui->ErrBox->addWidget(name);
            ui->ErrBox->addWidget(img);
            errCountThisTime++;
        }
    }
    // ++++++++++++ loading images from directory for SUCCESSES

    for (int i = 2; i < succCount; i++)
    {
        if (succList.at(i).contains(sessionNum)) {
            QLabel* img = new QLabel();
            img->setPixmap(QPixmap::fromImage(QImage(succDir.absolutePath().append("/" + succList.at(i)))));
			img->setMinimumSize(480, 262);
			img->setMaximumSize(1920, 1046);
			img->setScaledContents(true);
            QLabel* name = new QLabel();
            name->setText(succList.at(i).section("_", 5));
            name->setFont(font);
            name->setStyleSheet("QLabel {  color : green; }");
			//ui->PassList->layout->addWidget(name);
            ui->PassBox->addWidget(name);
            ui->PassBox->addWidget(img);
			succCountThisTime++;
        }
    }
	errCount = errCountThisTime;
	succCount = succCountThisTime;

	font.setPointSize(12);
	ui->ErrLabel->setFont(font);
	ui->ErrLabel->setText("Errors: ");
	ui->SuccLabel->setFont(font);
	ui->SuccLabel->setText("Achievements: ");
}
	//TIPS sending emails
	//note : mycurl.exe is the one reponsible for sending emails, the folder "curl" created by Sayak is for TIPS-Trainee 
	void SofaProcedureReport::emailReport(std::string studentEmail, std::string mentorEmail)
	{
		prepareReportEmail(studentEmail);
		std::cout << "TIPS reporter: sending email..." << std::endl;
		
		std::string strCommand = "mycurl smtp://smtp.gmail.com:587 -v --mail-from \"tips.surflab@gmail.com\" --mail-rcpt \"" + studentEmail + "\" --ssl -u tips.surflab@gmail.com:8a1fru$tips -T " + report_filename + " -k --anyauth";
		
		std::string strCommand_toMentor = "mycurl smtp://smtp.gmail.com:587 -v --mail-from \"tips.surflab@gmail.com\" --mail-rcpt \"" + mentorEmail + "\" --ssl -u tips.surflab@gmail.com:8a1fru$tips -T " + report_filename + " -k --anyauth";
		
		char * command = new char[strCommand.length() + 1];
		char * command_mentor = new char[strCommand_toMentor.length() + 1];
		std::strcpy(command, strCommand.c_str());
		std::strcpy(command_mentor, strCommand_toMentor.c_str());
		//std::cout << "emailReport() -> command : " << command << std::endl;
		std::string pathWithCommend = path.substr(0, path.find("examples")).append("/") + command;
		//std::cout << "pathWithCommend:" << pathWithCommend << std::endl;
		std::string pathWithCommendToMentor = path.substr(0, path.find("examples")).append("/") + command_mentor;
		int r1 = WinExec(pathWithCommend.c_str(), SW_SHOW);
		//ShellExecuteA(NULL, "open", pathWithCommend.c_str(), NULL, NULL, SW_NORMAL);
		int r2 = WinExec(pathWithCommendToMentor.c_str(), SW_SHOW);
		delete[] command;
		delete[] command_mentor;
		if (r1 >= 31) std::cout << "TIPS reporter: email sent!" << std::endl;
		else std::cout << "TIPS reporter: failed with return values: " << r1 << ", " << r2 << std::endl;
	}
	void SofaProcedureReport::prepareReportEmail(std::string studentEmail)
	{
		std::cout << "TIPS reporter: preparing email..." << std::endl;
		
		std::string path = sofa::helper::system::DataRepository.getFirstPath();
		std::string path_email_in = path.substr(0, path.find("examples")).append("/mail.txt");
		std::string path_email_out = path.substr(0, path.find("examples")).append("/");
		report_filename = path.substr(0, path.find("examples")).append("/") + programStartDate + "report.txt";
		path_email_out = path_email_out + programStartDate + "report.txt";
		//std::cout << "path_email_in: " << path_email_in << std::endl;
		//std::cout << "path_email_out: " << path_email_out << std::endl;
		
		std::string search_userName = "[userName]";
		std::string replace_userName = student.toStdString() + ", ";
		
		std::string search_date = "[dateEntry]";
		std::string replace_date = "on " + programStartDate;
		
		std::string search_count = "[errorsCount]";
		std::string replace_count = std::to_string(errCountThisTime);
		
		std::string search_errors = "[errorsEntry]";
		std::string replace_errors = errString;
		
		std::string search_userEmail = "destination@example.com";
		std::string replace_userEmail = studentEmail;
		
		std::string inbuf;
		std::fstream input_file(path_email_in, std::ios::in);
		std::ofstream output_file(path_email_out);
		while (!input_file.eof())
		{
			getline(input_file, inbuf);

			int spot = inbuf.find(search_userName);//add userName
			if (spot >= 0)
			{
				std::string tmpstring = inbuf.substr(0, spot); //the part before the appearence of the search string
				tmpstring += replace_userName;
				//tmpstring += inbuf.substr(spot + search_userName.length(), inbuf.length());
				inbuf = tmpstring;
			}

			spot = inbuf.find(search_userEmail);//add userEmail
			if (spot >= 0)
			{
				std::string tmpstring = inbuf.substr(0, spot); //the part before the appearence of the search string
				tmpstring += replace_userEmail;
				tmpstring += inbuf.substr(spot + search_userEmail.length(), inbuf.length());
				inbuf = tmpstring;
			}

			spot = inbuf.find(search_date);//add date
			if (spot >= 0)
			{
				std::string tmpstring = inbuf.substr(0, spot);
				tmpstring += replace_date;
				tmpstring += inbuf.substr(spot + search_date.length(), inbuf.length());
				inbuf = tmpstring;
			}

			spot = inbuf.find(search_count);//add errCount
			if (spot >= 0)
			{
				std::string tmpstring = inbuf.substr(0, spot);
				tmpstring += replace_count;
				tmpstring += inbuf.substr(spot + search_count.length(), inbuf.length());
				inbuf = tmpstring;
			}

			spot = inbuf.find(search_errors);//add errors
			if (spot >= 0)
			{
				if (errCountThisTime < 1)
					replace_errors += "congratulations! You proved proficiency without making any errors!";
				std::string tmpstring = inbuf.substr(0, spot);
				tmpstring += replace_errors;
				tmpstring += inbuf.substr(spot + search_errors.length(), inbuf.length());
				inbuf = tmpstring;
			}
			output_file << inbuf << std::endl;
		}
		input_file.close();
		output_file.close();
	}
	/*SofaProcedureReport* SofaProcedureReport::getInstance()
	{
		if (!instance)
			instance = new SofaProcedureReport;
		return instance;
	}*/
SofaProcedureReport::~SofaProcedureReport()
{
	//delete this;
}
}
}
}
