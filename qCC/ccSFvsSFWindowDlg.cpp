#include "ccSFvsSFWindowDlg.h"
#include "ui_sfVsSFWindowDlg.h"

#include <QSettings>

#include <math.h>

// ccSFvsSFPlot

ccSFvsSFPlot::ccSFvsSFPlot(QWidget *parent)
{
	setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
	setSelectionRectMode(QCP::srmZoom);
}

void ccSFvsSFPlot::mousePressEvent(QMouseEvent *event)
{
	if (event->button() == Qt::RightButton)
	{
		if (selectionRectMode() == QCP::srmNone)
			setSelectionRectMode(QCP::srmZoom);
		else
			setSelectionRectMode(QCP::srmNone);
	}
	QCustomPlot::mousePressEvent(event);
}

void ccSFvsSFPlot::mouseDoubleClickEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		rescaleAxes();
	}
	QCustomPlot::mouseDoubleClickEvent(event);
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
	m_colorMap = new QCPColorMap(m_plot->xAxis, m_plot->yAxis);
	m_colorMap->setInterpolate(false);
	// "background"
	// "grid"
	// "main"
	// "axes"
	// "legend"
	// "overlay"
	m_plot->moveLayer(m_plot->xAxis->grid()->layer(), m_plot->layer("main"));
	m_plot->xAxis->grid();
	m_graph = m_plot->addGraph();

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
	connect(this->ui->checkBoxShowGraph, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::showGraph);
	connect(this->ui->checkBoxShowDensityMap, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::showMap);
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
		m_colorScale->setDataScaleType(QCPAxis::stLogarithmic);
		m_colorScale->axis()->setTicker(QSharedPointer<QCPAxisTickerLog>(new QCPAxisTickerLog));
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
		int idxMapX = floor((item.key - keyRange.lower) / stepX);
		int idxMapY = floor((item.value - valueRange.lower) / stepY);
		m_colorMap->data()->setCell(idxMapX, idxMapY, m_colorMap->data()->cell(idxMapX, idxMapY) + 1);
	}
	for (int key = 0; key < m_colorMap->data()->keySize(); key++)
	{
		for (int value = 0; value < m_colorMap->data()->valueSize(); value++)
		{
			m_colorMap->data()->setCell(key, value, m_colorMap->data()->cell(key, value) / m_graph->data()->size() * 100);
		}
	}
	// add a color scale:
	QCPColorScale *colorScale = new QCPColorScale(m_plot);
	m_plot->plotLayout()->addElement(0, 1, colorScale); // add it to the right of the main axis rect
	colorScale->setType(QCPAxis::atRight); // scale shall be vertical bar with tick/axis labels right (actually atRight is already the default)
	m_colorMap->setColorScale(colorScale); // associate the color map with the color scale
	colorScale->axis()->setLabel("Density");

	// set the color gradient of the color map to one of the presets:
	m_colorMap->setGradient(QCPColorGradient::gpSpectrum);
	// we could have also created a QCPColorGradient instance and added own colors to
	// the gradient, see the documentation of QCPColorGradient for what's possible.

	// rescale the data dimension (color) such that all data points lie in the span visualized by the color gradient:
	m_colorMap->rescaleDataRange();

	// make sure the axis rect and color scale synchronize their bottom and top margins (so they line up):
	QCPMarginGroup *marginGroup = new QCPMarginGroup(m_plot);
	m_plot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
	colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);

	// rescale the key (x) and value (y) axes so the whole color map is visible:
	m_plot->rescaleAxes();
}

