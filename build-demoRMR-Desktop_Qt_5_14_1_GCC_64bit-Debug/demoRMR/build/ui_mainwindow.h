/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QGridLayout *gridLayout_2;
    QPushButton *pushButton_5;
    QPushButton *pushButton_2;
    QPushButton *pushButton_6;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QFrame *frame_2;
    QFrame *frame;
    QGridLayout *gridLayout_4;
    QFormLayout *formLayout;
    QLabel *label_4;
    QLineEdit *lineEdit_5;
    QLabel *label_5;
    QLineEdit *lineEdit_6;
    QSpacerItem *verticalSpacer_2;
    QLabel *label_9;
    QLineEdit *lineEdit_10;
    QLabel *label_6;
    QLineEdit *lineEdit_7;
    QSpacerItem *verticalSpacer_3;
    QLabel *label_10;
    QLineEdit *lineEdit_11;
    QLabel *label_3;
    QLineEdit *lineEdit_4;
    QGridLayout *gridLayout_3;
    QPushButton *pushButton_10;
    QSpacerItem *verticalSpacer_4;
    QLineEdit *lineEdit_2;
    QLabel *label_8;
    QLabel *label;
    QLabel *label_7;
    QPushButton *pushButton;
    QSpacerItem *verticalSpacer;
    QLineEdit *lineEdit_9;
    QLineEdit *lineEdit_3;
    QPushButton *pushButton_7;
    QLabel *label_2;
    QLineEdit *lineEdit_8;
    QVBoxLayout *verticalLayout_2;
    QGridLayout *gridLayout_5;
    QSpacerItem *verticalSpacer_7;
    QLabel *label_14;
    QLineEdit *lineEdit_13;
    QLabel *label_12;
    QLabel *label_17;
    QLabel *label_16;
    QLineEdit *lineEdit;
    QLabel *label_18;
    QLabel *label_11;
    QLabel *label_15;
    QPushButton *pushButton_8;
    QPushButton *pushButton_11;
    QLabel *label_13;
    QLabel *label_19;
    QLineEdit *lineEdit_14;
    QLineEdit *lineEdit_12;
    QSpacerItem *verticalSpacer_6;
    QTextEdit *textEdit;
    QLabel *label_20;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_9;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1250, 868);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setSpacing(6);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        pushButton_5 = new QPushButton(centralWidget);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(pushButton_5->sizePolicy().hasHeightForWidth());
        pushButton_5->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(pushButton_5, 1, 2, 1, 1);

        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        sizePolicy.setHeightForWidth(pushButton_2->sizePolicy().hasHeightForWidth());
        pushButton_2->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(pushButton_2, 0, 1, 1, 1);

        pushButton_6 = new QPushButton(centralWidget);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        sizePolicy.setHeightForWidth(pushButton_6->sizePolicy().hasHeightForWidth());
        pushButton_6->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(pushButton_6, 1, 0, 1, 1);

        pushButton_3 = new QPushButton(centralWidget);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        sizePolicy.setHeightForWidth(pushButton_3->sizePolicy().hasHeightForWidth());
        pushButton_3->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(pushButton_3, 2, 1, 1, 1);

        pushButton_4 = new QPushButton(centralWidget);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        sizePolicy.setHeightForWidth(pushButton_4->sizePolicy().hasHeightForWidth());
        pushButton_4->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(pushButton_4, 1, 1, 1, 1);


        gridLayout->addLayout(gridLayout_2, 1, 0, 1, 1);

        frame_2 = new QFrame(centralWidget);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setMinimumSize(QSize(600, 450));
        frame_2->setMaximumSize(QSize(600, 450));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(frame_2, 2, 1, 1, 1);

        frame = new QFrame(centralWidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy1);
        frame->setMinimumSize(QSize(600, 450));
        frame->setMaximumSize(QSize(600, 450));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(frame, 2, 0, 1, 1);

        gridLayout_4 = new QGridLayout();
        gridLayout_4->setSpacing(6);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        formLayout = new QFormLayout();
        formLayout->setSpacing(6);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setContentsMargins(-1, -1, 30, -1);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_4);

        lineEdit_5 = new QLineEdit(centralWidget);
        lineEdit_5->setObjectName(QString::fromUtf8("lineEdit_5"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(lineEdit_5->sizePolicy().hasHeightForWidth());
        lineEdit_5->setSizePolicy(sizePolicy2);

        formLayout->setWidget(0, QFormLayout::FieldRole, lineEdit_5);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_5);

        lineEdit_6 = new QLineEdit(centralWidget);
        lineEdit_6->setObjectName(QString::fromUtf8("lineEdit_6"));
        sizePolicy2.setHeightForWidth(lineEdit_6->sizePolicy().hasHeightForWidth());
        lineEdit_6->setSizePolicy(sizePolicy2);

        formLayout->setWidget(1, QFormLayout::FieldRole, lineEdit_6);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        formLayout->setItem(2, QFormLayout::FieldRole, verticalSpacer_2);

        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_9);

        lineEdit_10 = new QLineEdit(centralWidget);
        lineEdit_10->setObjectName(QString::fromUtf8("lineEdit_10"));
        sizePolicy2.setHeightForWidth(lineEdit_10->sizePolicy().hasHeightForWidth());
        lineEdit_10->setSizePolicy(sizePolicy2);

        formLayout->setWidget(3, QFormLayout::FieldRole, lineEdit_10);

        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout->setWidget(4, QFormLayout::LabelRole, label_6);

        lineEdit_7 = new QLineEdit(centralWidget);
        lineEdit_7->setObjectName(QString::fromUtf8("lineEdit_7"));
        sizePolicy2.setHeightForWidth(lineEdit_7->sizePolicy().hasHeightForWidth());
        lineEdit_7->setSizePolicy(sizePolicy2);

        formLayout->setWidget(4, QFormLayout::FieldRole, lineEdit_7);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        formLayout->setItem(5, QFormLayout::FieldRole, verticalSpacer_3);

        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        formLayout->setWidget(6, QFormLayout::LabelRole, label_10);

        lineEdit_11 = new QLineEdit(centralWidget);
        lineEdit_11->setObjectName(QString::fromUtf8("lineEdit_11"));
        sizePolicy2.setHeightForWidth(lineEdit_11->sizePolicy().hasHeightForWidth());
        lineEdit_11->setSizePolicy(sizePolicy2);

        formLayout->setWidget(6, QFormLayout::FieldRole, lineEdit_11);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(7, QFormLayout::LabelRole, label_3);

        lineEdit_4 = new QLineEdit(centralWidget);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));
        sizePolicy2.setHeightForWidth(lineEdit_4->sizePolicy().hasHeightForWidth());
        lineEdit_4->setSizePolicy(sizePolicy2);
        lineEdit_4->setFocusPolicy(Qt::NoFocus);
        lineEdit_4->setReadOnly(true);

        formLayout->setWidget(7, QFormLayout::FieldRole, lineEdit_4);


        gridLayout_4->addLayout(formLayout, 0, 0, 1, 1);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setSpacing(6);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(-1, -1, 100, -1);
        pushButton_10 = new QPushButton(centralWidget);
        pushButton_10->setObjectName(QString::fromUtf8("pushButton_10"));
        sizePolicy2.setHeightForWidth(pushButton_10->sizePolicy().hasHeightForWidth());
        pushButton_10->setSizePolicy(sizePolicy2);
        pushButton_10->setMinimumSize(QSize(162, 0));

        gridLayout_3->addWidget(pushButton_10, 16, 1, 1, 1);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_3->addItem(verticalSpacer_4, 12, 1, 1, 1);

        lineEdit_2 = new QLineEdit(centralWidget);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));
        sizePolicy2.setHeightForWidth(lineEdit_2->sizePolicy().hasHeightForWidth());
        lineEdit_2->setSizePolicy(sizePolicy2);
        lineEdit_2->setFocusPolicy(Qt::NoFocus);
        lineEdit_2->setReadOnly(true);

        gridLayout_3->addWidget(lineEdit_2, 7, 1, 1, 1);

        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_3->addWidget(label_8, 10, 0, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_3->addWidget(label, 7, 0, 1, 1);

        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_3->addWidget(label_7, 6, 0, 1, 1);

        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        sizePolicy2.setHeightForWidth(pushButton->sizePolicy().hasHeightForWidth());
        pushButton->setSizePolicy(sizePolicy2);
        pushButton->setMinimumSize(QSize(162, 0));

        gridLayout_3->addWidget(pushButton, 15, 1, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_3->addItem(verticalSpacer, 9, 1, 1, 1);

        lineEdit_9 = new QLineEdit(centralWidget);
        lineEdit_9->setObjectName(QString::fromUtf8("lineEdit_9"));
        sizePolicy2.setHeightForWidth(lineEdit_9->sizePolicy().hasHeightForWidth());
        lineEdit_9->setSizePolicy(sizePolicy2);

        gridLayout_3->addWidget(lineEdit_9, 10, 1, 1, 1);

        lineEdit_3 = new QLineEdit(centralWidget);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));
        sizePolicy2.setHeightForWidth(lineEdit_3->sizePolicy().hasHeightForWidth());
        lineEdit_3->setSizePolicy(sizePolicy2);
        lineEdit_3->setFocusPolicy(Qt::NoFocus);
        lineEdit_3->setReadOnly(true);

        gridLayout_3->addWidget(lineEdit_3, 11, 1, 1, 1);

        pushButton_7 = new QPushButton(centralWidget);
        pushButton_7->setObjectName(QString::fromUtf8("pushButton_7"));
        sizePolicy2.setHeightForWidth(pushButton_7->sizePolicy().hasHeightForWidth());
        pushButton_7->setSizePolicy(sizePolicy2);
        pushButton_7->setMinimumSize(QSize(162, 0));

        gridLayout_3->addWidget(pushButton_7, 17, 1, 1, 1);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_3->addWidget(label_2, 11, 0, 1, 1);

        lineEdit_8 = new QLineEdit(centralWidget);
        lineEdit_8->setObjectName(QString::fromUtf8("lineEdit_8"));
        sizePolicy2.setHeightForWidth(lineEdit_8->sizePolicy().hasHeightForWidth());
        lineEdit_8->setSizePolicy(sizePolicy2);

        gridLayout_3->addWidget(lineEdit_8, 6, 1, 1, 1);


        gridLayout_4->addLayout(gridLayout_3, 0, 1, 1, 1);


        gridLayout->addLayout(gridLayout_4, 3, 0, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setSizeConstraint(QLayout::SetDefaultConstraint);
        verticalLayout_2->setContentsMargins(-1, 0, 150, -1);
        gridLayout_5 = new QGridLayout();
        gridLayout_5->setSpacing(6);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        gridLayout_5->setSizeConstraint(QLayout::SetDefaultConstraint);
        verticalSpacer_7 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_5->addItem(verticalSpacer_7, 10, 1, 1, 1);

        label_14 = new QLabel(centralWidget);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout_5->addWidget(label_14, 4, 1, 1, 1);

        lineEdit_13 = new QLineEdit(centralWidget);
        lineEdit_13->setObjectName(QString::fromUtf8("lineEdit_13"));

        gridLayout_5->addWidget(lineEdit_13, 5, 2, 1, 1);

        label_12 = new QLabel(centralWidget);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_5->addWidget(label_12, 2, 1, 1, 1);

        label_17 = new QLabel(centralWidget);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        gridLayout_5->addWidget(label_17, 0, 1, 1, 1);

        label_16 = new QLabel(centralWidget);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_5->addWidget(label_16, 6, 1, 1, 1);

        lineEdit = new QLineEdit(centralWidget);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        gridLayout_5->addWidget(lineEdit, 2, 2, 1, 1);

        label_18 = new QLabel(centralWidget);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_18, 11, 1, 1, 1);

        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_5->addWidget(label_11, 1, 1, 1, 1);

        label_15 = new QLabel(centralWidget);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_5->addWidget(label_15, 5, 1, 1, 1);

        pushButton_8 = new QPushButton(centralWidget);
        pushButton_8->setObjectName(QString::fromUtf8("pushButton_8"));
        sizePolicy2.setHeightForWidth(pushButton_8->sizePolicy().hasHeightForWidth());
        pushButton_8->setSizePolicy(sizePolicy2);
        pushButton_8->setMinimumSize(QSize(162, 0));

        gridLayout_5->addWidget(pushButton_8, 9, 1, 1, 1);

        pushButton_11 = new QPushButton(centralWidget);
        pushButton_11->setObjectName(QString::fromUtf8("pushButton_11"));
        pushButton_11->setEnabled(true);
        pushButton_11->setLayoutDirection(Qt::LeftToRight);

        gridLayout_5->addWidget(pushButton_11, 9, 2, 1, 1);

        label_13 = new QLabel(centralWidget);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_5->addWidget(label_13, 3, 1, 1, 1);

        label_19 = new QLabel(centralWidget);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        gridLayout_5->addWidget(label_19, 11, 2, 1, 1);

        lineEdit_14 = new QLineEdit(centralWidget);
        lineEdit_14->setObjectName(QString::fromUtf8("lineEdit_14"));

        gridLayout_5->addWidget(lineEdit_14, 6, 2, 1, 1);

        lineEdit_12 = new QLineEdit(centralWidget);
        lineEdit_12->setObjectName(QString::fromUtf8("lineEdit_12"));

        gridLayout_5->addWidget(lineEdit_12, 3, 2, 1, 1);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_5->addItem(verticalSpacer_6, 8, 1, 1, 1);

        textEdit = new QTextEdit(centralWidget);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        textEdit->setReadOnly(true);

        gridLayout_5->addWidget(textEdit, 2, 3, 5, 1);

        label_20 = new QLabel(centralWidget);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        gridLayout_5->addWidget(label_20, 1, 3, 1, 1);


        verticalLayout_2->addLayout(gridLayout_5);


        gridLayout->addLayout(verticalLayout_2, 3, 1, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(200, 10, 200, 10);
        pushButton_9 = new QPushButton(centralWidget);
        pushButton_9->setObjectName(QString::fromUtf8("pushButton_9"));
        pushButton_9->setEnabled(true);
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(pushButton_9->sizePolicy().hasHeightForWidth());
        pushButton_9->setSizePolicy(sizePolicy3);
        pushButton_9->setMinimumSize(QSize(162, 0));
        QFont font;
        font.setPointSize(15);
        pushButton_9->setFont(font);

        verticalLayout->addWidget(pushButton_9);


        gridLayout->addLayout(verticalLayout, 1, 1, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        pushButton_5->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "Forward", nullptr));
        pushButton_6->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        pushButton_3->setText(QCoreApplication::translate("MainWindow", "Back", nullptr));
        pushButton_4->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "enc left", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "enc right", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "R speed*:", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "R speed:", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "Rot*:", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Rot:", nullptr));
        pushButton_10->setText(QCoreApplication::translate("MainWindow", "ADD XY TO QUEUE ", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "Y*:", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "X*:", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "SET REQUIRED", nullptr));
        pushButton_7->setText(QCoreApplication::translate("MainWindow", "LASER SCAN", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        label_14->setText(QCoreApplication::translate("MainWindow", "Required position:", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        label_17->setText(QCoreApplication::translate("MainWindow", "NAVIGATION", nullptr));
        label_16->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        label_18->setText(QCoreApplication::translate("MainWindow", "RESULT:", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "Current position:", nullptr));
        label_15->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        pushButton_8->setText(QCoreApplication::translate("MainWindow", "Way from text map", nullptr));
        pushButton_11->setText(QCoreApplication::translate("MainWindow", "Way from robot map", nullptr));
        label_13->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        label_19->setText(QString());
        label_20->setText(QCoreApplication::translate("MainWindow", "POINTS:", nullptr));
        pushButton_9->setText(QCoreApplication::translate("MainWindow", "Start", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
