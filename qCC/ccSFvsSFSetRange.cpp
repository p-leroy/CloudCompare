#include "ccSFvsSFSetRange.h"
#include "ui_sfVsSFSetRange.h"

ccSFvsSFSetRange::ccSFvsSFSetRange(QCPAxis* axis, QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::sfVsSFSetRange)
	, m_axis(axis)
{
	this->setFocusPolicy(Qt::StrongFocus);

	ui->setupUi(this);

	ui->doubleSpinBoxMin->setMinimum(std::numeric_limits<double>::min());
	ui->doubleSpinBoxMin->setMaximum(std::numeric_limits<double>::max());
	ui->doubleSpinBoxMax->setMinimum(std::numeric_limits<double>::min());
	ui->doubleSpinBoxMax->setMaximum(std::numeric_limits<double>::max());

	connect(ui->pushButtonApply, &QAbstractButton::pressed, this, &ccSFvsSFSetRange::emitSetRange);
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

void ccSFvsSFSetRange::emitSetRange()
{
	emit setRange(m_axis, ui->doubleSpinBoxMin->value(), ui->doubleSpinBoxMax->value());
}

void ccSFvsSFSetRange::focusOutEvent(QFocusEvent* event)
{
	delete this;
}
