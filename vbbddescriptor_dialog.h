#ifndef _AUTO_COMPUTE_ALL_DIALOG_H_
#define _AUTO_COMPUTE_ALL_DIALOG_H_

//Qt
#include <QObject>
#include <QDialog>

#include "ui_vbbddescriptor.h"

class AutoComputeAllDialog :public QDialog, public Ui::VBBDDescriptorDialog
{
	Q_OBJECT
public:
	explicit AutoComputeAllDialog(QWidget* parent = 0);

	virtual ~AutoComputeAllDialog() {}


	int Getindex() { return m_index; }

	int GetRadius() { return m_radius; }

	int GetGridSize() { return m_gridsizel; }

	int GetSampling() { return m_sample; }

	double GetRatio() { return m_ratio; }

protected slots:

	//关键点采样算法的序号
	void setValue_index(int);

	//半径
	void SetValue_Radius(int);

	//格网大小
	void SetValue_GridSize(int);

	//采样
	void SetValue_Sampling(int);

	//ratio
	void SetValue_ratio(double);


private:

	int     m_index;//选择的序号
	int    m_radius; //半径
	int    m_gridsizel;; //格网
	int    m_sample; //采样
	double   m_ratio; // 比率

};
#endif