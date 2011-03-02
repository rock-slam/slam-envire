#include <iostream>

#include "ICPHandler.hpp"

#include "ui_MainWindow.h"
#include <envire/maps/LaserScan.hpp>
#include <envire/maps/TriMesh.hpp>

#include <QListWidgetItem>

using namespace enview;

struct ModelListItem : public QListWidgetItem
{
    envire::Pointcloud* mesh;

    ModelListItem( envire::Pointcloud* mesh ) :
	QListWidgetItem( QString::fromStdString(mesh->getClassName()) ),
	mesh(mesh) { };
};


ICPHandler::ICPHandler(QObject* parent, Ui::MainWindow& ui, callback c)
    : QObject(parent), ui(ui), getSelectedItem(c)
{
    connect( ui.addToModelButton, SIGNAL(pressed(void)), this, SLOT(addToModel()) );
    connect( ui.clearModelButton, SIGNAL(pressed(void)), this, SLOT(clearModel()) );
    connect( ui.runStepButton, SIGNAL(pressed(void)), this, SLOT(runStep()) );
    connect( ui.runICPButton, SIGNAL(pressed(void)), this, SLOT(runICP()) );


    ui.iterationsLineEdit->setText("5");
    ui.thresholdLineEdit->setText(".5");
    ui.densityLineEdit->setText(".01");
    ui.minDistanceLineEdit->setText("1e-5");
}

void ICPHandler::addToModel()
{
    envire::EnvironmentItem* selectedItem = getSelectedItem();

    if( selectedItem != NULL && dynamic_cast<envire::Pointcloud*>(selectedItem) )
    {
	ui.modelListWidget->addItem( new ModelListItem( dynamic_cast<envire::Pointcloud*>(selectedItem) ) );
    }
    else
    {
	std::cout << "Not a Pointcloud Item" << std::endl;
    }
}

void ICPHandler::clearModel()
{
    ui.modelListWidget->clear();
}

void ICPHandler::runICP()
{
    envire::EnvironmentItem* selectedItem = getSelectedItem();
    envire::Pointcloud* mesh = dynamic_cast<envire::Pointcloud*>(selectedItem);

    if( !mesh )
    {
	std::cout << "No Pointcloud Item selected." << std::endl;
	return;
    }

    size_t max_iter = 10;
    double min_mse = .02;
    double min_mse_diff = 0.01;
    double density = 0.1;

    double alpha = 0.4;
    double beta = 1.0;
    double eps = 0.01;

    try
    {
	max_iter = boost::lexical_cast<int>( ui.iterationsLineEdit->text().toStdString() );
	min_mse = boost::lexical_cast<double>( ui.thresholdLineEdit->text().toStdString() );
	density = boost::lexical_cast<double>( ui.densityLineEdit->text().toStdString() );
	min_mse_diff = boost::lexical_cast<double>( ui.minDistanceLineEdit->text().toStdString() );
    }
    catch(...) {}

    // add the model
    icp.clearModel();
    for(int i=0;i<ui.modelListWidget->count();i++)
    {
	ModelListItem* item = dynamic_cast<ModelListItem*>( ui.modelListWidget->item( i ) );
	if( item )
	{
	    icp.addToModel( envire::icp::PointcloudAdapter(item->mesh, density) );
	}
    }

    icp.align( envire::icp::PointcloudAdapter(mesh, density), max_iter, min_mse, min_mse_diff, alpha, beta, eps );
}

void ICPHandler::runStep()
{
    std::cout << "Not implemented yet" << std::endl;
}
