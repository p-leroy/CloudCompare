#include "ccDrawNormalsWidget.h"
#include "ui_ccDrawNormalsWidget.h"

#include <QSettings>

ccDrawNormalsWidget::ccDrawNormalsWidget(ccPointCloud *cloud, QWidget *parent) :
	QWidget(parent),
	ui(new Ui::ccDrawNormalsWidget),
	m_cloud(cloud)
{
	ui->setupUi(this);

	ui->label_cloudName->setText(m_cloud->getName());

	connect(this->ui->doubleSpinBox_normalLength, SIGNAL(valueChanged(double)), this, SLOT(normalLengthValueChanged(double)));

	readSettings();

	normalLengthValueChanged(ui->doubleSpinBox_normalLength->value());

	setWindowFlag(Qt::WindowStaysOnTopHint);
	setWindowFlag(Qt::Window);
	show();
}

ccDrawNormalsWidget::~ccDrawNormalsWidget()
{
	writeSettings();
	delete ui;
}

void ccDrawNormalsWidget::readSettings()
{
	QSettings settings("OSUR/CloudCompare", "drawNormals");

	double normalLength = settings.value("normalLength", 1.).toDouble();

	ui->doubleSpinBox_normalLength->setValue(normalLength);
}

void ccDrawNormalsWidget::writeSettings()
{
	QSettings settings("OSUR/CloudCompare", "drawNormals");

	settings.setValue("normalLength", ui->doubleSpinBox_normalLength->value());
}

void ccDrawNormalsWidget::normalLengthValueChanged(double value)
{
	m_cloud->setNormalLength(value);
}
