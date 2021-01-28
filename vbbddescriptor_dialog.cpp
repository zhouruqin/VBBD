#include "vbbddescriptor_dialog.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QTextCodec.h>
#include "qmessagebox.h"

AutoComputeAllDialog::AutoComputeAllDialog(QWidget* parent) : QDialog(parent)
, Ui::VBBDDescriptorDialog()
{
	m_index = 0;
	m_radius = 15;
	m_gridsizel = 9;
	m_sample = 1;
	m_ratio = 0.9;


	setupUi(this);

	//connect(keypointcomboBox,SIGNAL(currentIndexChanged(QString)),this,SLOT(print_s()));
	connect(keypointcomboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(setValue_index(int)));// 关键点算法序号
	connect(radiusspinBox, SIGNAL(valueChanged(int)), this, SLOT(SetValue_Radius(int)));//半径
	connect(gridspinBox, SIGNAL(valueChanged(int)), this, SLOT(SetValue_GridSize(int)));//格网
	connect(samplingspinBox, SIGNAL(valueChanged(int)), this, SLOT(SetValue_Sampling(int)));//采样
	connect(ratiodoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(SetValue_ratio(double)));//比率
	QTextCodec::setCodecForLocale(QTextCodec::codecForName("GB2312"));

}



void AutoComputeAllDialog::setValue_index(int value)
{
	m_index = value;
}

void AutoComputeAllDialog::SetValue_Radius(int value)
{
	m_radius = value;
}

void AutoComputeAllDialog::SetValue_GridSize(int value)
{
	m_gridsizel = value;
}

void AutoComputeAllDialog::SetValue_Sampling(int value)
{
	m_sample = value;
}

void AutoComputeAllDialog::SetValue_ratio(double value)
{
	m_ratio = value;
}


