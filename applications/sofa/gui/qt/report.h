#ifndef SOFA_GUI_QT_REPORT_H
#define SOFA_GUI_QT_REPORT_H

#include <QScrollArea>
#include <QLabel>
#include <QListWidget>
#include <QString>
#include <QDir>
#include <windows.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/system/FileRepository.h>
#include "SofaGUIQt.h"
#include <ui_report.h>

class Ui_Report;

namespace sofa
{
namespace gui
{
namespace qt
{

	class SOFA_SOFAGUIQT_API SofaProcedureReport : public QScrollArea
{
		Q_OBJECT
		std::unique_ptr<Ui_Report> ui;

    int errCount = 0;
    int errCountThisTime = 0;
	int succCountThisTime = 0;
    int succCount = 0;
    QString student = "Student";//system username
    QString sessionNum = "sessionNum";
    QDir rootDirectory;
    QDir errDir;
    QDir succDir;
    QStringList errList;
    QStringList succList;
    std::string errString ="";
    std::string programStartDate = "";
	std::string report_filename = "";

    // +++++ File path modifications
    std::string path = sofa::helper::system::DataRepository.getFirstPath();
    std::string base_path_share = path.substr(0, path.find("examples")).append("/TIPS_screenshot");
    QFont font;

public:
    SofaProcedureReport(QWidget* parent = 0);
	void populate(QString studentName, std::string programStartDate);
	void emailReport(std::string studentEmail, std::string mentorEmail);
	void prepareReportEmail(std::string studentEmail);
	~SofaProcedureReport();
};
	//SofaProcedureReport* SofaProcedureReport::instance = 0;
}
}
}

#endif // SOFA_GUI_QT_REPORT_H
