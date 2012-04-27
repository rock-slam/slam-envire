#include "CsvDialog.hpp"
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QToolButton>
#include <QSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QDialogButtonBox>
#include <QPlastiqueStyle>
#include <QFileDialog>

namespace enview 
{
    
CsvDialog::CsvDialog(QWidget* parent, Qt::WindowFlags f): QDialog(parent, f)
{
    // set title
    setWindowTitle(tr("Add Csv File"));
        
    // create QWidgets
    options_box.reset(new QGroupBox("Options" ,this));
    path_label.reset(new QLabel("Path", this));
    input_label.reset(new QLabel("Input format per row", options_box.get()));
    select_label.reset(new QLabel("Select randomly one sample of ", options_box.get()));
    path_edit.reset(new QLineEdit(this));
    path_dialog_button.reset(new QToolButton(this));
    format_select.reset(new QComboBox(options_box.get()));
    sample_count.reset(new QSpinBox(options_box.get()));
    button_box.reset(new QDialogButtonBox(this));
    
    // add items to combo box
    format_select->addItem("X Y Z");
    format_select->addItem("X Y Z R");

    // configure widgets
    sample_count->setRange(1, std::numeric_limits<int>::max());
    sample_count->setValue(1);
    
    path_edit->setMinimumWidth(300);
    path_edit->setReadOnly(true);
    
    path_dialog_button->setText("...");
    
    button_box->setStandardButtons(QDialogButtonBox::Open | QDialogButtonBox::Cancel);
    
    style.reset(new QPlastiqueStyle);
    options_box->setStyle(style.get());
    
    // set layout
    options_layout.reset(new QGridLayout(this));
    options_layout->setColumnMinimumWidth(1, 10);
    options_layout->setColumnStretch(1,1);
    options_layout->addWidget(input_label.get(), 0, 0);
    options_layout->addWidget(format_select.get(), 0, 2);
    options_layout->addWidget(select_label.get(), 1, 0);
    options_layout->addWidget(sample_count.get(), 1, 2);
    options_box->setLayout(options_layout.get());
    
    layout.reset(new QGridLayout(this));
    layout->setColumnMinimumWidth(1, 10);
    layout->setColumnStretch(1,1);
    layout->addWidget(path_label.get(), 0, 0);
    layout->addWidget(path_edit.get(), 0, 1);
    layout->addWidget(path_dialog_button.get(), 0, 2);
    layout->addWidget(options_box.get(), 2, 0, 1, 3);
    layout->addWidget(button_box.get(), 5, 0, 1, 3);
    setLayout(layout.get());
    
    // setup connections
    connect(button_box.get(), SIGNAL(rejected()), this, SLOT(reject()));
    connect(button_box.get(), SIGNAL(accepted()), this, SLOT(accept()));
    connect(path_dialog_button.get(), SIGNAL(clicked(bool)), this, SLOT(openFileDialog()));
}

QString CsvDialog::getFileName() const
{
    return path_edit->displayText();
}

envire::Pointcloud::TextFormat CsvDialog::getFormat() const
{
    int index = format_select->currentIndex();
    switch (index)
    {
        case 0:
            return envire::Pointcloud::XYZ;
        case 1:
            return envire::Pointcloud::XYZR;
        default:
            return envire::Pointcloud::XYZ;
    }
}

unsigned int CsvDialog::getSampleRate() const
{
    return (unsigned)sample_count->value();
}

void CsvDialog::openFileDialog()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Add Csv File"),
        ".",
        tr("Csv (*.txt);;Leica (*.asc);;XYZ (*)"));
    
    if(fileName != NULL)
        path_edit->setText(fileName);
}
    
    
}