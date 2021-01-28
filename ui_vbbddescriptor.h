/********************************************************************************
** Form generated from reading UI file 'vbbddescriptor.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VBBDDESCRIPTOR_H
#define UI_VBBDDESCRIPTOR_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpinBox>

QT_BEGIN_NAMESPACE

class Ui_VBBDDescriptorDialog
{
public:
    QDialogButtonBox *buttonBox;
    QComboBox *keypointcomboBox;
    QSpinBox *gridspinBox;
    QSpinBox *samplingspinBox;
    QSpinBox *radiusspinBox;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QDoubleSpinBox *ratiodoubleSpinBox;

    void setupUi(QDialog *VBBDDescriptorDialog)
    {
        if (VBBDDescriptorDialog->objectName().isEmpty())
			VBBDDescriptorDialog->setObjectName(QStringLiteral("VBBDDescriptorDialog"));
		VBBDDescriptorDialog->resize(448, 343);
        buttonBox = new QDialogButtonBox(VBBDDescriptorDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(80, 220, 201, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        keypointcomboBox = new QComboBox(VBBDDescriptorDialog);
        keypointcomboBox->setObjectName(QStringLiteral("keypointcomboBox"));
        keypointcomboBox->setGeometry(QRect(90, 40, 181, 22));
        gridspinBox = new QSpinBox(VBBDDescriptorDialog);
        gridspinBox->setObjectName(QStringLiteral("gridspinBox"));
        gridspinBox->setGeometry(QRect(170, 110, 91, 22));
        gridspinBox->setMinimum(3);
        gridspinBox->setSingleStep(2);
        gridspinBox->setValue(9);
        samplingspinBox = new QSpinBox(VBBDDescriptorDialog);
        samplingspinBox->setObjectName(QStringLiteral("samplingspinBox"));
        samplingspinBox->setGeometry(QRect(170, 140, 91, 22));
        samplingspinBox->setValue(1);
        radiusspinBox = new QSpinBox(VBBDDescriptorDialog);
        radiusspinBox->setObjectName(QStringLiteral("radiusspinBox"));
        radiusspinBox->setGeometry(QRect(170, 80, 91, 22));
        radiusspinBox->setMaximum(100);
        radiusspinBox->setSingleStep(5);
        radiusspinBox->setValue(15);
        label = new QLabel(VBBDDescriptorDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(90, 80, 41, 16));
        label_2 = new QLabel(VBBDDescriptorDialog);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(90, 110, 72, 15));
        label_3 = new QLabel(VBBDDescriptorDialog);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(90, 140, 72, 15));
        label_4 = new QLabel(VBBDDescriptorDialog);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(90, 170, 72, 15));
        ratiodoubleSpinBox = new QDoubleSpinBox(VBBDDescriptorDialog);
        ratiodoubleSpinBox->setObjectName(QStringLiteral("ratiodoubleSpinBox"));
        ratiodoubleSpinBox->setGeometry(QRect(169, 170, 91, 22));
        ratiodoubleSpinBox->setDecimals(5);
        ratiodoubleSpinBox->setMaximum(1);
        ratiodoubleSpinBox->setSingleStep(0.05);
        ratiodoubleSpinBox->setValue(0.9);

        retranslateUi(VBBDDescriptorDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), VBBDDescriptorDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), VBBDDescriptorDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(VBBDDescriptorDialog);
    } // setupUi

    void retranslateUi(QDialog *VBBDDescriptorDialog)
    {
		VBBDDescriptorDialog->setWindowTitle(QApplication::translate("VBBDDescriptorDialog", "Dialog", 0));
        keypointcomboBox->clear();
        keypointcomboBox->insertItems(0, QStringList()
         << QApplication::translate("KeyPointDialog", "ISS\345\205\263\351\224\256\347\202\271\346\217\220\345\217\226", 0)
         << QApplication::translate("KeyPointDialog", "Harris3D\345\205\263\351\224\256\347\202\271\346\217\220\345\217\226", 0)
         << QApplication::translate("KeyPointDialog", "NARF\345\205\263\351\224\256\347\202\271\346\217\220\345\217\226", 0)
         << QApplication::translate("KeyPointDialog", "SIFT3D\345\205\263\351\224\256\347\202\271\346\217\220\345\217\226", 0)
         << QApplication::translate("KeyPointDialog", "uniform sampling", 0)
         << QApplication::translate("KeyPointDialog", "voxel sampling", 0)
        );
        label->setText(QApplication::translate("KeyPointDialog", "\345\215\212\345\276\204", 0));
        label_2->setText(QApplication::translate("KeyPointDialog", "\346\240\274\347\275\221", 0));
        label_3->setText(QApplication::translate("KeyPointDialog", "\351\207\207\346\240\267", 0));
        label_4->setText(QApplication::translate("KeyPointDialog", "\351\230\210\345\200\274", 0));
    } // retranslateUi

};

namespace Ui {
    class VBBDDescriptorDialog: public Ui_VBBDDescriptorDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VBBDDESCRIPTOR_H
