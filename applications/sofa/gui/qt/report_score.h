#ifndef REPORT_SCORE_H
#define REPORT_SCORE_H

#include <QDialog>
#include "SofaGUIQt.h"
#include "ui_report_score.h"
#include <QtWidgets/QWidget>
#include <QAction>

namespace sofa
{
       namespace gui
       {
               namespace qt
               {

                class report_score : public QDialog , public Ui_report_score
                {
                    Q_OBJECT

                    public:
						report_score();
                        report_score(std::string date);
                        ~report_score();
                    };
               }
       }
}
#endif // REPORT_SCORE_H
