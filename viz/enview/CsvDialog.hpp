#ifndef __ENVIEW_CSV_DIALOG_HPP__
#define __ENVIEW_CSV_DIALOG_HPP__

#include <QDialog>
#include <boost/shared_ptr.hpp>
#include <envire/maps/Pointcloud.hpp>

class QComboBox;
class QLabel;
class QLineEdit;
class QToolButton;
class QSpinBox;
class QGridLayout;
class QGroupBox;
class QDialogButtonBox;

namespace enview 
{

class CsvDialog : public QDialog
{
    Q_OBJECT
    
public:
    CsvDialog(QWidget * parent = 0, Qt::WindowFlags f = 0);
    
    QString getFileName() const;
    unsigned getSampleRate() const;
    envire::Pointcloud::TextFormat getFormat() const;
    
protected slots:
    void openFileDialog();
    
protected:    
    boost::shared_ptr<QLabel> path_label;
    boost::shared_ptr<QGroupBox> options_box;
    boost::shared_ptr<QLabel> input_label;
    boost::shared_ptr<QLabel> select_label;
    boost::shared_ptr<QLineEdit> path_edit;
    boost::shared_ptr<QToolButton> path_dialog_button;
    boost::shared_ptr<QComboBox> format_select;
    boost::shared_ptr<QSpinBox> sample_count;
    boost::shared_ptr<QDialogButtonBox> button_box;
    boost::shared_ptr<QGridLayout> layout;
    boost::shared_ptr<QGridLayout> options_layout;
    boost::shared_ptr<QStyle> style;
};

}

#endif
