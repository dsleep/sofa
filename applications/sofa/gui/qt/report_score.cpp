#include "report_score.h"
#include "ui_report_score.h"

namespace sofa
{
	namespace gui
	{
		namespace qt
		{
			report_score::report_score()
			{
				setupUi(this);
			}
			report_score::report_score(std::string date)
			{
				setupUi(this, date);
			}

			report_score::~report_score()
			{
			}
		}
	}
}
