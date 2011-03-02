#ifndef __ENVIEW_ICPHANDLER__
#define __ENVIEW_ICPHANDLER__

#include <QObject>
#include "ui_MainWindow.h"
#include <envire/Core.hpp>
#include <envire/icp.hpp>

namespace enview
{
    class ICPHandlerImpl; 

    class ICPHandler : public QObject 
    {
	Q_OBJECT

    public:
	ICPHandler(QObject* parent, Ui::MainWindow& ui, envire::EnvironmentItem*& selectedItem);

    public slots:
	void addToModel();
	void clearModel();
	void runICP();
	void runStep();

    private:
	envire::icp::TrimmedKD icp;

	Ui::MainWindow& ui;
	envire::EnvironmentItem*& selectedItem;
    };
}

#endif
