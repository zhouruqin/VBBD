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

	//�ؼ�������㷨�����
	void setValue_index(int);

	//�뾶
	void SetValue_Radius(int);

	//������С
	void SetValue_GridSize(int);

	//����
	void SetValue_Sampling(int);

	//ratio
	void SetValue_ratio(double);


private:

	int     m_index;//ѡ������
	int    m_radius; //�뾶
	int    m_gridsizel;; //����
	int    m_sample; //����
	double   m_ratio; // ����

};
#endif