/********************************************************************************
** Form generated from reading UI file 'DataFusionpntECv.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef DATAFUSIONPNTECV_H
#define DATAFUSIONPNTECV_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_DataFusionClass
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_2;
    QWidget *widget;
    QVBoxLayout *verticalLayout_3;
    QVTKWidget *qvtkWidget;
    QPushButton *pushButton_view;
    QPushButton *pushButton_zeroView;
    QTabWidget *tabWidget;
    QWidget *tab;
    QGridLayout *gridLayout;
    QWidget *widget1;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_2;
    QComboBox *comboBox;
    QSpacerItem *horizontalSpacer;
    QWidget *widget2;
    QHBoxLayout *horizontalLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout;
    QSpacerItem *verticalSpacer_2;
    QPushButton *pushButton_plan1;
    QPushButton *pushButton_plan2;
    QPushButton *pushButton_plan3;
    QPushButton *pushButton_plan4;
    QSpacerItem *verticalSpacer;
    QTextEdit *textEdit;
    QFrame *frame1;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton_3points;
    QPushButton *pushButton_CoordinateTransform;
    QFrame *frame_2;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_8;
    QWidget *tab_2;
    QPushButton *pushButton_6;
    QPushButton *pushButton_7;
    QWidget *tab_3;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *DataFusionClass)
    {
        if (DataFusionClass->objectName().isEmpty())
            DataFusionClass->setObjectName(QString::fromUtf8("DataFusionClass"));
        DataFusionClass->resize(1298, 769);
        DataFusionClass->setStyleSheet(QString::fromUtf8("QMainWindow { background-color:rgb(160, 203, 238);}"));
        DataFusionClass->setAnimated(true);
        centralWidget = new QWidget(DataFusionClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setStyleSheet(QString::fromUtf8("QMainWindow {background-color:rgb(25, 35, 45);}"));
        gridLayout_2 = new QGridLayout(centralWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        widget = new QWidget(centralWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy);
        widget->setMinimumSize(QSize(500, 310));
        verticalLayout_3 = new QVBoxLayout(widget);
        verticalLayout_3->setSpacing(1);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        qvtkWidget = new QVTKWidget(widget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));

        verticalLayout_3->addWidget(qvtkWidget);

        pushButton_view = new QPushButton(widget);
        pushButton_view->setObjectName(QString::fromUtf8("pushButton_view"));
        pushButton_view->setStyleSheet(QString::fromUtf8("background-color:rgb(225, 225, 225);"));
        pushButton_view->setCheckable(false);
        pushButton_view->setAutoRepeat(false);
        pushButton_view->setAutoExclusive(false);
        pushButton_view->setAutoDefault(false);
        pushButton_view->setFlat(false);

        verticalLayout_3->addWidget(pushButton_view);

        pushButton_zeroView = new QPushButton(widget);
        pushButton_zeroView->setObjectName(QString::fromUtf8("pushButton_zeroView"));
        pushButton_zeroView->setStyleSheet(QString::fromUtf8("background-color:rgb(225, 225, 225);"));

        verticalLayout_3->addWidget(pushButton_zeroView);


        gridLayout_2->addWidget(widget, 0, 1, 1, 1);

        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy1);
        tabWidget->setMinimumSize(QSize(310, 500));
        tabWidget->setStyleSheet(QString::fromUtf8(""));
        tabWidget->setTabPosition(QTabWidget::South);
        tabWidget->setTabShape(QTabWidget::Rounded);
        tabWidget->setElideMode(Qt::ElideLeft);
        tabWidget->setUsesScrollButtons(false);
        tabWidget->setDocumentMode(false);
        tabWidget->setTabsClosable(false);
        tabWidget->setMovable(false);
        tabWidget->setTabBarAutoHide(false);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        gridLayout = new QGridLayout(tab);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        widget1 = new QWidget(tab);
        widget1->setObjectName(QString::fromUtf8("widget1"));
        verticalLayout_4 = new QVBoxLayout(widget1);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        comboBox = new QComboBox(widget1);
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->setObjectName(QString::fromUtf8("comboBox"));
        comboBox->setEnabled(true);
        sizePolicy1.setHeightForWidth(comboBox->sizePolicy().hasHeightForWidth());
        comboBox->setSizePolicy(sizePolicy1);
        comboBox->setMinimumSize(QSize(0, 28));
        comboBox->setEditable(false);

        horizontalLayout_2->addWidget(comboBox);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);


        verticalLayout_4->addLayout(horizontalLayout_2);

        widget2 = new QWidget(widget1);
        widget2->setObjectName(QString::fromUtf8("widget2"));
        horizontalLayout = new QHBoxLayout(widget2);
        horizontalLayout->setSpacing(11);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(5, -1, -1, -1);
        frame = new QFrame(widget2);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setFocusPolicy(Qt::NoFocus);
        verticalLayout = new QVBoxLayout(frame);
        verticalLayout->setSpacing(10);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        verticalLayout->setContentsMargins(1, 1, 1, 1);
        verticalSpacer_2 = new QSpacerItem(20, 10, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout->addItem(verticalSpacer_2);

        pushButton_plan1 = new QPushButton(frame);
        pushButton_plan1->setObjectName(QString::fromUtf8("pushButton_plan1"));
        pushButton_plan1->setMinimumSize(QSize(0, 30));
        pushButton_plan1->setStyleSheet(QString::fromUtf8("background-color:rgb(225, 225, 225);"));

        verticalLayout->addWidget(pushButton_plan1);

        pushButton_plan2 = new QPushButton(frame);
        pushButton_plan2->setObjectName(QString::fromUtf8("pushButton_plan2"));
        pushButton_plan2->setMinimumSize(QSize(0, 30));
        pushButton_plan2->setStyleSheet(QString::fromUtf8("background-color:rgb(225, 225, 225);"));

        verticalLayout->addWidget(pushButton_plan2);

        pushButton_plan3 = new QPushButton(frame);
        pushButton_plan3->setObjectName(QString::fromUtf8("pushButton_plan3"));
        pushButton_plan3->setMinimumSize(QSize(0, 30));
        pushButton_plan3->setStyleSheet(QString::fromUtf8("background-color:rgb(225, 225, 225);"));

        verticalLayout->addWidget(pushButton_plan3);

        pushButton_plan4 = new QPushButton(frame);
        pushButton_plan4->setObjectName(QString::fromUtf8("pushButton_plan4"));
        pushButton_plan4->setMinimumSize(QSize(0, 30));
        pushButton_plan4->setStyleSheet(QString::fromUtf8("background-color:rgb(225, 225, 225);"));

        verticalLayout->addWidget(pushButton_plan4);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addWidget(frame);

        textEdit = new QTextEdit(widget2);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        textEdit->setFrameShape(QFrame::StyledPanel);
        textEdit->setFrameShadow(QFrame::Sunken);
        textEdit->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        textEdit->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        textEdit->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        textEdit->setTabChangesFocus(false);
        textEdit->setReadOnly(true);
        textEdit->setTextInteractionFlags(Qt::NoTextInteraction);

        horizontalLayout->addWidget(textEdit);


        verticalLayout_4->addWidget(widget2);

        frame1 = new QFrame(widget1);
        frame1->setObjectName(QString::fromUtf8("frame1"));
        verticalLayout_2 = new QVBoxLayout(frame1);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        pushButton_3points = new QPushButton(frame1);
        pushButton_3points->setObjectName(QString::fromUtf8("pushButton_3points"));
        pushButton_3points->setStyleSheet(QString::fromUtf8("background-color:rgb(225, 225, 225);"));

        verticalLayout_2->addWidget(pushButton_3points);

        pushButton_CoordinateTransform = new QPushButton(frame1);
        pushButton_CoordinateTransform->setObjectName(QString::fromUtf8("pushButton_CoordinateTransform"));
        pushButton_CoordinateTransform->setStyleSheet(QString::fromUtf8("background-color:rgb(225, 225, 225);"));

        verticalLayout_2->addWidget(pushButton_CoordinateTransform);


        verticalLayout_4->addWidget(frame1);


        gridLayout->addWidget(widget1, 0, 0, 1, 1);

        frame_2 = new QFrame(tab);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setMinimumSize(QSize(0, 100));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        verticalLayout_5 = new QVBoxLayout(frame_2);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        pushButton = new QPushButton(frame_2);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        horizontalLayout_3->addWidget(pushButton);

        pushButton_2 = new QPushButton(frame_2);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        horizontalLayout_3->addWidget(pushButton_2);


        verticalLayout_5->addLayout(horizontalLayout_3);

        pushButton_3 = new QPushButton(frame_2);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        verticalLayout_5->addWidget(pushButton_3);

        pushButton_8 = new QPushButton(frame_2);
        pushButton_8->setObjectName(QString::fromUtf8("pushButton_8"));

        verticalLayout_5->addWidget(pushButton_8);


        gridLayout->addWidget(frame_2, 1, 0, 1, 1);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        pushButton_6 = new QPushButton(tab_2);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        pushButton_6->setGeometry(QRect(160, 150, 92, 28));
        pushButton_7 = new QPushButton(tab_2);
        pushButton_7->setObjectName(QString::fromUtf8("pushButton_7"));
        pushButton_7->setGeometry(QRect(150, 220, 92, 28));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        pushButton_4 = new QPushButton(tab_3);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        pushButton_4->setGeometry(QRect(60, 160, 251, 31));
        pushButton_5 = new QPushButton(tab_3);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        pushButton_5->setGeometry(QRect(60, 210, 251, 31));
        tabWidget->addTab(tab_3, QString());

        gridLayout_2->addWidget(tabWidget, 0, 0, 1, 1);

        DataFusionClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(DataFusionClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        statusBar->setStyleSheet(QString::fromUtf8("QMainWindow {background-color:rgb(69, 83, 100)}"));
        DataFusionClass->setStatusBar(statusBar);

        retranslateUi(DataFusionClass);

        pushButton_view->setDefault(false);
        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(DataFusionClass);
    } // setupUi

    void retranslateUi(QMainWindow *DataFusionClass)
    {
        DataFusionClass->setWindowTitle(QCoreApplication::translate("DataFusionClass", "DataFusion", nullptr));
        pushButton_view->setText(QCoreApplication::translate("DataFusionClass", "\346\230\276\347\244\272", nullptr));
        pushButton_zeroView->setText(QCoreApplication::translate("DataFusionClass", "\345\275\222\344\270\200\345\214\226\346\230\276\347\244\272", nullptr));
        comboBox->setItemText(0, QCoreApplication::translate("DataFusionClass", "\344\270\211\345\235\220\346\240\207\346\265\213\351\207\217\346\234\272\346\225\260\346\215\256", nullptr));
        comboBox->setItemText(1, QCoreApplication::translate("DataFusionClass", "\347\231\275\345\205\211\345\271\262\346\266\211\344\273\252\346\225\260\346\215\256", nullptr));

        pushButton_plan1->setText(QCoreApplication::translate("DataFusionClass", "\351\241\266\351\235\242", nullptr));
        pushButton_plan2->setText(QCoreApplication::translate("DataFusionClass", "\344\276\247\351\235\2421", nullptr));
        pushButton_plan3->setText(QCoreApplication::translate("DataFusionClass", "\344\276\247\351\235\2422", nullptr));
        pushButton_plan4->setText(QCoreApplication::translate("DataFusionClass", "\344\276\247\351\235\2423", nullptr));
        pushButton_3points->setText(QCoreApplication::translate("DataFusionClass", "\344\270\211\345\237\272\345\207\206\347\202\271", nullptr));
        pushButton_CoordinateTransform->setText(QCoreApplication::translate("DataFusionClass", "\345\235\220\346\240\207\350\275\254\346\215\242", nullptr));
        pushButton->setText(QCoreApplication::translate("DataFusionClass", "\345\271\262\346\266\211\344\273\252\346\225\260\346\215\256 ", nullptr));
        pushButton_2->setText(QCoreApplication::translate("DataFusionClass", "\345\235\220\346\240\207\346\234\272\346\225\260\346\215\256", nullptr));
        pushButton_3->setText(QCoreApplication::translate("DataFusionClass", "\345\212\240\346\235\203\345\271\263\345\235\207\346\263\225\350\236\215\345\220\210", nullptr));
        pushButton_8->setText(QCoreApplication::translate("DataFusionClass", "\345\212\240\346\235\203\346\234\200\345\260\217\344\272\214\344\271\230\350\236\215\345\220\210", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("DataFusionClass", "\345\235\220\346\240\207\347\273\237\344\270\200", nullptr));
        pushButton_6->setText(QCoreApplication::translate("DataFusionClass", "\347\220\203", nullptr));
        pushButton_7->setText(QCoreApplication::translate("DataFusionClass", "\351\235\242", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("DataFusionClass", "\346\225\260\346\215\256\350\236\215\345\220\210", nullptr));
        pushButton_4->setText(QCoreApplication::translate("DataFusionClass", "TXT\350\275\254PCD", nullptr));
        pushButton_5->setText(QCoreApplication::translate("DataFusionClass", "PCD\350\275\254TXT", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("DataFusionClass", "\346\225\260\346\215\256\350\275\254\346\215\242", nullptr));
    } // retranslateUi

};

namespace Ui {
    class DataFusionClass: public Ui_DataFusionClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // DATAFUSIONPNTECV_H
