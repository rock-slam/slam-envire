#ifndef __ENVIEW_ICPHANDLER__
#define __ENVIEW_ICPHANDLER__

#include <QObject>
#include "ui_MainWindow.h"
#include <envire/Core.hpp>
#include "icp/icp.hpp"

#include <boost/function.hpp>

namespace enview
{
    class ICPHandlerImpl; 

    class ICPHandler : public QObject 
    {
	Q_OBJECT
	
    public:
	typedef boost::function<envire::EnvironmentItem* ()> callback; 
	ICPHandler(QObject* parent, Ui::MainWindow& ui, callback c);

    public slots:
	void addToModel();
	void clearModel();
	void runICP();
	void runStep();

    private:
	envire::icp::TrimmedKD icp;

	Ui::MainWindow& ui;
	callback getSelectedItem;
    };
}

#endif
