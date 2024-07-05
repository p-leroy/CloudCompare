#include "ccSFvsSFWindowDlg.h"
#include "ui_sfVsSFWindowDlg.h"

#include <QSettings>
#include <QPushButton>

#include <math.h>

// ccSFvsSFPlot

ccSFvsSFPlot::ccSFvsSFPlot(QWidget *parent)
{
	setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
					QCP::iSelectLegend | QCP::iSelectPlottables);

	setSelectionRectMode(QCP::srmNone);

//	connect(this, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged()));
	connect(this, &QCustomPlot::selectionChangedByUser, this, &ccSFvsSFPlot::selectionChanged);
}

void ccSFvsSFPlot::selectionChanged()
{
    /*
   normally, axis base line, axis tick labels and axis labels are selectable separately, but we want
   the user only to be able to select the axis as a whole, so we tie the selected states of the tick labels
   and the axis base line together. However, the axis label shall be selectable individually.

   The selection state of the left and right axes shall be synchronized as well as the state of the
   bottom and top axes.
  */

	// make top and bottom axes be selected synchronously, and handle axis and tick labels as one selectable object:
	if (xAxis->selectedParts().testFlag(QCPAxis::spAxis) || xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
		xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
	{
		xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
		xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
	}
	// make left and right axes be selected synchronously, and handle axis and tick labels as one selectable object:
	if (yAxis->selectedParts().testFlag(QCPAxis::spAxis) || yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
		yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
	{
		yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
		yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
	}
}

void ccSFvsSFPlot::autoscale()
{
	rescaleAxes();
	replot();
}

void ccSFvsSFPlot::mousePressEvent(QMouseEvent *event)
{
	// if an axis is selected, only allow the direction of that axis to be dragged
	// if no axis is selected, both directions may be dragged

	if (xAxis->selectedParts().testFlag(QCPAxis::spAxis))
		axisRect()->setRangeDrag(xAxis->orientation());
	else if (yAxis->selectedParts().testFlag(QCPAxis::spAxis))
		axisRect()->setRangeDrag(yAxis->orientation());
	else
		axisRect()->setRangeDrag(Qt::Horizontal|Qt::Vertical);

	QCustomPlot::mousePressEvent(event); // forward event to QCustomPlot event handler
}

void ccSFvsSFPlot::wheelEvent(QWheelEvent *event)
{
	// if an axis is selected, only allow the direction of that axis to be zoomed
	// if no axis is selected, both directions may be zoomed

	if (xAxis->selectedParts().testFlag(QCPAxis::spAxis))
		axisRect()->setRangeZoom(xAxis->orientation());
	else if (yAxis->selectedParts().testFlag(QCPAxis::spAxis))
		axisRect()->setRangeZoom(yAxis->orientation());
	else
		axisRect()->setRangeZoom(Qt::Horizontal|Qt::Vertical);

	QCustomPlot::wheelEvent(event); // forward event to QCustomPlot event handler
}

// ccSFvsSFWindowDlg

ccSFvsSFWindowDlg::ccSFvsSFWindowDlg(ccPointCloud *cloud, QWidget *parent)
	: QDialog(parent)
	, m_cloud(cloud)
	, ui(new Ui::sfVsSFWindowDlg)
{
	assert(cloud);

	ui->setupUi(this);

	m_plot = this->ui->widget;

	m_plot->clearGraphs();

	// add a color map
	m_colorMap = new QCPColorMap(m_plot->xAxis, m_plot->yAxis);
	m_colorMap->setInterpolate(false);
	// "background" "grid" "main" "axes" "legend" "overlay"
	m_plot->moveLayer(m_plot->xAxis->grid()->layer(), m_plot->layer("main"));
	m_plot->xAxis->grid();

	m_graph = m_plot->addGraph();

	// add a color scale:
	m_colorScale = new QCPColorScale(m_plot);
	m_plot->plotLayout()->addElement(0, 1, m_colorScale); // add it to the right of the main axis rect
	m_colorScale->setType(QCPAxis::atRight); // scale shall be vertical bar with tick/axis labels right (actually atRight is already the default)
	m_colorScale->axis()->setLabel("Density");
	m_colorMap->setGradient(QCPColorGradient::gpSpectrum); // set the color gradient of the color map to one of the presets

	setComboBoxes();

	QSettings settings("OSUR", "ccSFvsSFWindowDlg");
	this->ui->checkBoxXLog->setChecked(settings.value("xLog", false).toBool());
	this->ui->checkBoxYLog->setChecked(settings.value("yLog", false).toBool());
	this->ui->checkBoxShowGraph->setChecked(settings.value("showGraph", true).toBool());
	this->ui->checkBoxShowDensityMap->setChecked(settings.value("showDensityMap", false).toBool());

	refresh();

	showGraph(ui->checkBoxShowGraph->isChecked());
	showMap(ui->checkBoxShowDensityMap->isChecked());

	connect(this->ui->checkBoxXLog, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setXScaleType);
	connect(this->ui->checkBoxYLog, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setYScaleType);
	connect(this->ui->checkBoxLogScaleDensityMap, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setColorScaleType);
	connect(this->ui->checkBoxShowGraph, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::showGraph);
	connect(this->ui->checkBoxShowDensityMap, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::showMap);
	connect(this->ui->pushButtonAutoscale, &QAbstractButton::pressed, m_plot, &ccSFvsSFPlot::autoscale);
}

ccSFvsSFWindowDlg::~ccSFvsSFWindowDlg()
{
	// save configuration
	QSettings settings("OSUR", "ccSFvsSFWindowDlg");
	settings.setValue("xIndex", this->ui->comboBoxXAxis->currentIndex());
	settings.setValue("yIndex", this->ui->comboBoxYAxis->currentIndex());
	settings.setValue("xLog", this->ui->checkBoxXLog->isChecked());
	settings.setValue("yLog", this->ui->checkBoxYLog->isChecked());
	settings.setValue("showGraph", this->ui->checkBoxShowGraph->isChecked());
	settings.setValue("showDensityMap", this->ui->checkBoxShowDensityMap->isChecked());

	delete ui;
}

void ccSFvsSFWindowDlg::setComboBoxes()
{
	unsigned int numberOfScalarFields = m_cloud->getNumberOfScalarFields();

	for (unsigned int index = 0; index < numberOfScalarFields; index++)
	{
		this->ui->comboBoxXAxis->addItem(m_cloud->getScalarFieldName(index));
		this->ui->comboBoxYAxis->addItem(m_cloud->getScalarFieldName(index));
	}

	// try to restore previous confirguration
	QSettings settings("OSUR", "ccSFvsSFWindowDlg");
	int previousXIndex = settings.value("xIndex", 0).toInt();
	int previousYIndex = settings.value("yIndex", 0).toInt();
	if (previousXIndex <= this->ui->comboBoxXAxis->count())
	{
		this->ui->comboBoxXAxis->setCurrentIndex(previousXIndex);
	}
	if (previousYIndex <= this->ui->comboBoxYAxis->count())
	{
		this->ui->comboBoxYAxis->setCurrentIndex(previousYIndex);
	}

	connect(this->ui->comboBoxXAxis, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ccSFvsSFWindowDlg::refresh);
	connect(this->ui->comboBoxYAxis, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ccSFvsSFWindowDlg::refresh);
}

void ccSFvsSFWindowDlg::refresh()
{
	int xIndex = this->ui->comboBoxXAxis->currentIndex();
	int yIndex = this->ui->comboBoxYAxis->currentIndex();

	// get data from scalar fields
	QVector<double> m_x;
	QVector<double> m_y;
	QString m_xName;
	QString m_yName;

	m_x.resize(m_cloud->size());
	m_y.resize(m_cloud->size());
	CCCoreLib::ScalarField *sfX = m_cloud->getScalarField(xIndex);
	CCCoreLib::ScalarField *sfY = m_cloud->getScalarField(yIndex);

	assert(sfX && sfY);

	for (unsigned int index = 0; index < m_cloud->size(); index++)
	{
		m_x[index] = sfX->getValue(index);
		m_y[index] = sfY->getValue(index);
	}
	m_xName = sfX->getName();
	m_yName = sfY->getName();

	// assign data to graph
	m_graph->data().clear();
	m_graph->setData(m_x, m_y);

	// give the axes some labels
	m_plot->xAxis->setLabel(m_xName);
	m_plot->yAxis->setLabel(m_yName);

	m_graph->setLineStyle(QCPGraph::lsNone);
	m_graph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle));

	m_plot->rescaleAxes();
	m_plot->replot();

	densityMap();
}

void ccSFvsSFWindowDlg::setXScaleType()
{
	if (ui->checkBoxXLog->isChecked())
	{
		m_plot->xAxis->setScaleType(QCPAxis::stLogarithmic);
		m_plot->xAxis->setTicker(QSharedPointer<QCPAxisTickerLog>(new QCPAxisTickerLog));
	}
	else
	{
		m_plot->xAxis->setScaleType(QCPAxis::stLinear);
		m_plot->xAxis->setTicker(QSharedPointer<QCPAxisTicker>(new QCPAxisTicker));
	}
	m_plot->replot();
}

void ccSFvsSFWindowDlg::setYScaleType()
{
	if (ui->checkBoxYLog->isChecked())
	{
		m_plot->yAxis->setScaleType(QCPAxis::stLogarithmic);
		m_plot->yAxis->setTicker(QSharedPointer<QCPAxisTickerLog>(new QCPAxisTickerLog));
	}
	else
	{
		m_plot->yAxis->setScaleType(QCPAxis::stLinear);
		m_plot->yAxis->setTicker(QSharedPointer<QCPAxisTicker>(new QCPAxisTicker));
	}
	m_plot->replot();
}

void ccSFvsSFWindowDlg::setColorScaleType()
{
	if (ui->checkBoxLogScaleDensityMap->isChecked())
	{
		m_colorScale->setDataScaleType(QCPAxis::stLogarithmic);
		m_colorScale->axis()->setTicker(QSharedPointer<QCPAxisTickerLog>(new QCPAxisTickerLog));
	}
	else
	{
		m_colorScale->setDataScaleType(QCPAxis::stLinear);
		m_colorScale->axis()->setTicker(QSharedPointer<QCPAxisTicker>(new QCPAxisTicker));
	}
	m_plot->replot();
}

void ccSFvsSFWindowDlg::densityMap()
{
	int nx = 10;
	int ny = 10;

	QSharedPointer<QCPGraphDataContainer> container = m_graph->data();
	bool foundKeyRange;
	bool foundValueRange;
	QCPRange keyRange = container->keyRange(foundKeyRange);
	QCPRange valueRange = container->valueRange(foundValueRange);

	assert(foundKeyRange && foundValueRange);

	double stepX = keyRange.size() / nx;
	double stepY = valueRange.size() / ny;

	// set up the QCPColorMap:
	m_colorMap->data()->clear();
	m_colorMap->data()->setSize(nx + 1, ny + 1); // we want the color map to have nx * ny data points
	m_colorMap->data()->setRange(keyRange, valueRange);
	m_colorMap->data()->fill(0);
	// now we assign some data, by accessing the QCPColorMapData instance of the color map:
	for (auto item : *container)
	{
		int keyIdx = floor((item.key - keyRange.lower) / stepX);
		int valueIdx = floor((item.value - valueRange.lower) / stepY);
		m_colorMap->data()->setCell(keyIdx, valueIdx, m_colorMap->data()->cell(keyIdx, valueIdx) + 1);
	}
	for (int keyIdx = 0; keyIdx < m_colorMap->data()->keySize(); keyIdx++)
	{
		for (int valueIdx = 0; valueIdx < m_colorMap->data()->valueSize(); valueIdx++)
		{
			m_colorMap->data()->setCell(keyIdx, valueIdx, m_colorMap->data()->cell(keyIdx, valueIdx) / m_graph->dataCount() * 100);
		}
	}

	m_colorMap->setColorScale(m_colorScale); // associate the color map with the color scale

	// rescale the data dimension (color) such that all data points lie in the span visualized by the color gradient:
	m_colorMap->rescaleDataRange();

	// make sure the axis rect and color scale synchronize their bottom and top margins (so they line up):
	QCPMarginGroup *marginGroup = new QCPMarginGroup(m_plot);
	m_plot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
	m_colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);

	// rescale the key (x) and value (y) axes so the whole color map is visible:
	m_plot->rescaleAxes();
}

void ccSFvsSFWindowDlg::linearFit()
{
	double sum_x;
	double sum_x2;
	double sum_y;
	double sum_xy;
	int n = m_graph->dataCount();

	QSharedPointer<QCPGraphDataContainer> container = m_graph->data();

	/* Calculating Required Sum */
	for (auto item : *container)
	{
		float x = item.key;
		float y = item.value;
		if (!isnan(x) && !isnan(y))
		{
			sum_x = sum_x + x;
			sum_x2 = sum_x2 + x * x;
			sum_y = sum_y + y;
			sum_xy = sum_xy + x * y;
		}
		else
		{
			ccLog::Print("NaN detected");
		}
	}


	/* Calculating a and b */
	double b = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
	double a = (sum_y - b * sum_x) / n;

	// add the fit to the plot

}

