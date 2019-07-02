#include "ui_surflablogin.h"
#include "surflablogin.h"
namespace sofa
{
namespace gui
{
namespace qt
{

SurfLabLogin::SurfLabLogin(QWidget *parent) :
    QDialog(parent),
    ui(new Ui_SurfLabLogin)
{
    ui->setupUi(this);
	// SIGNAL / SLOTS CONNECTIONS
	this->connect(ui->ButtonBox, SIGNAL(clicked()), this, SLOT(on_ButtonBox_clicked()));

}

SurfLabLogin::~SurfLabLogin()
{
	delete ui;
}

void SurfLabLogin::on_ButtonBox_clicked(QAbstractButton *button)
{
    if(QDialogButtonBox::Ok){
        studentEmail = ui->email->text();
        studentName = ui->name->text();
        destinationEmail = ui->email_dropdown->currentText();
        QDialog::close();
    }
    else if (QDialogButtonBox::Reset){
        ui->email->clear();
        ui->name->clear();
    }
}
}
}
}
