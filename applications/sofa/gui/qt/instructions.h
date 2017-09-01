#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include <QDialog>
#include "SofaGUIQt.h"
#include "ui_instructions.h"
#include <QtWidgets/QWidget>
#include <QAction>

namespace sofa
{
	namespace gui
	{
		namespace qt
		{
			class Instructions : public QDialog, public Ui_Instructions
			{
				Q_OBJECT

			public:
				Instructions();
				~Instructions();
				/*void Instructions::keyPressEvent(QKeyEvent *e){
					if (e->key() == Qt::Key_Space){
						this->close();
					}
				}*/

			};
		}
	}
}
#endif // INSTRUCTIONS_H
