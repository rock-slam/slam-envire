#include <QApplication>
#include <osg/ArgumentParser>
#include <iostream>
#include <stdexcept>

#include "ApplicationWindow.hpp"

class Application: public QApplication {
public:
   Application(int &c, char **v): QApplication(c, v) {}
   bool notify(QObject *rec, QEvent *ev) {
       try {
           return QApplication::notify(rec, ev);
       }
       catch (char const *str) {
	   std::cerr << "EXCEPTION: " << str << std::endl;
           return false;
       }
   }
};

int main( int argc, char **argv )
{
    Application a( argc, argv );
    osg::ArgumentParser arguments(&argc, argv);

    envire::Environment *env = NULL;
    if( argc > 1 )
    {
	std::string env_name = argv[1];
	env = envire::Environment::unserialize( env_name );
    }
    enview::ApplicationWindow appWindow( env );

    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );

    return a.exec();
}
