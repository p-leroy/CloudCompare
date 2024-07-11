#include "ccSFvsSFSetRange.h"
#include "ui_sfVsSFSetRange.h"

#include <ccLog.h>

ccSFvsSFSetRange::ccSFvsSFSetRange(QCPAxis* axis, QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::sfVsSFSetRange)
	, m_axis(axis)
{
	ui->setupUi(this);

	ui->doubleSpinBoxMin->setMinimum(std::numeric_limits<double>::min());
	ui->doubleSpinBoxMin->setMaximum(std::numeric_limits<double>::max());
	ui->doubleSpinBoxMax->setMinimum(std::numeric_limits<double>::min());
	ui->doubleSpinBoxMax->setMaximum(std::numeric_limits<double>::max());
	ui->lineEditAxisLabel->setText(axis->label());

	this->setWindowFlag(Qt::Window, true);
	this->setWindowTitle("Edit [" + axis->label() + "]");

	connect(ui->pushButtonApply, &QAbstractButton::pressed, this, &ccSFvsSFSetRange::applyChanges);
}

ccSFvsSFSetRange::~ccSFvsSFSetRange()
{
	delete ui;
}

void ccSFvsSFSetRange::setLower(double lower)
{
	ui->doubleSpinBoxMin->setValue(lower);
}

void ccSFvsSFSetRange::setUpper(double upper)
{
	ui->doubleSpinBoxMax->setValue(upper);
}

void ccSFvsSFSetRange::applyChanges()
{
	if ( ui->doubleSpinBoxMin->value() > ui->doubleSpinBoxMax->value())
	{
		ccLog::Warning("[SF/SF] lower and upper bounds are not coherent");
	}
	else
	{
		m_axis->setRange(QCPRange(ui->doubleSpinBoxMin->value(), ui->doubleSpinBoxMax->value()));
		m_axis->setLabel(ui->lineEditAxisLabel->text());
		emit replot();
	}
}

void ccSFvsSFSetRange::changeEvent(QEvent* event)
{
	if(event->type() == QEvent::ActivationChange)
	{
		if (!this->isActiveWindow())
		{
			this->deleteLater();
		}
	}
}
