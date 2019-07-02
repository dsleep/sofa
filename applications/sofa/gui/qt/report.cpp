#include "ui_report.h"
#include "report.h"


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
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];

		time(&rawtime);
		timeinfo = localtime(&rawtime);
		//std::cout << "path.substr(0, path.find(examples))" << base_path_share << std::endl;
		strftime(buffer, sizeof(buffer), "%d_%m_%Y-%I_%M_%S", timeinfo);
		programStartDate = std::string(buffer);

		//---------- setting up directories ----------------------
		rootDirectory = QDir(QString::fromStdString(base_path_share));
		student = rootDirectory.homePath().section("/", 2);
		sessionNum = QString::fromStdString(programStartDate);
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
	void SofaProcedureReport::populate(QString studentName) {
		student = studentName;
		errList = errDir.entryList();
		succList = succDir.entryList();

    errCount = errList.size();
    succCount = succList.size();
    errCountThisTime = 0;
    std::cout << "errCount: " << errCount << std::endl;
    std::cout << "succCount: " << succCount << std::endl;

    //--------EDIT LABELS--------------
    font.setPointSize(15);
    ui->StudentLabel->setFont(font);
    ui->StudentLabel->setText(student + "\'s Report: " + sessionNum);

    font.setPointSize(12);
    ui->ErrLabel->setFont(font);
    ui->ErrLabel->setText("Errors");
    ui->SuccLabel->setFont(font);
    ui->SuccLabel->setText("Achievements");

    // ++++++++++++ loading images from directory for ERRORS
    for (int i = 2; i < errCount; i++)
    {
        if (errList.at(i).contains(sessionNum)) {
            /*errString.append(errList.at(i).toStdString());
            errString.append("\n");*/
            QLabel *img = new QLabel();
			img->setPixmap(QPixmap::fromImage(QImage(errDir.absolutePath().append("/" + errList.at(i)))));
            /*QImage screenshot = QImage(errDir.absolutePath().append("/" + errList.at(i)));
            screenshot = screenshot.scaledToWidth(ui->ErrorArea->width(), Qt::FastTransformation);
            QPixmap scap = QPixmap::fromImage(screenshot);
            img->setPixmap(scap);*/
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
        }
    }
}

SofaProcedureReport::~SofaProcedureReport()
{
	delete this;
}
}
}
}
