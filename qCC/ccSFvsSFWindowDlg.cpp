#include "ccSFvsSFWindowDlg.h"
#include "ui_sfVsSFWindowDlg.h"

#include "ccSFvsSFSetRange.h"

// CCPluginAPI
#include <ccPersistentSettings.h>

// qCC_db
#include <ccFileUtils.h>

// qCC_IO
#include <ImageFileFilter.h>

#include <QSettings>
#include <QPushButton>

#include <math.h>

#include <iostream>

// ccSFvsSFPlot

ccSFvsSFPlot::ccSFvsSFPlot(QWidget *parent)
{
	setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
					QCP::iSelectLegend | QCP::iSelectPlottables);

	setSelectionRectMode(QCP::srmNone);

	xAxis->grid()->setSubGridVisible(true);
	yAxis->grid()->setSubGridVisible(true);

	connect(this, &QCustomPlot::selectionChangedByUser, this, &ccSFvsSFPlot::selectionChanged);
	connect(this, &QCustomPlot::axisDoubleClick, this, &ccSFvsSFPlot::onAxisDoubleClick);
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
		xAxis2->setSelectedParts(QCPAxis::spAxis | QCPAxis::spTickLabels);
		xAxis->setSelectedParts(QCPAxis::spAxis | QCPAxis::spTickLabels);
	}
	// make left and right axes be selected synchronously, and handle axis and tick labels as one selectable object:
	if (yAxis->selectedParts().testFlag(QCPAxis::spAxis) || yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
		yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
	{
		yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
		yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
	}
}

void ccSFvsSFPlot::onAxisDoubleClick(QCPAxis *axis, QCPAxis::SelectablePart part)
{
	ccSFvsSFSetRange* setRange = new ccSFvsSFSetRange(axis, this);
	setRange->setLower(axis->range().lower);
	setRange->setUpper(axis->range().upper);
	setRange->show();
	setRange->setFocus();
	connect(setRange, &ccSFvsSFSetRange::replot, this, &ccSFvsSFPlot::onReplot);
}

void ccSFvsSFPlot::setAxisRange(QCPAxis *axis, double lower, double upper)
{
	axis->setRange(QCPRange(lower, upper));
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

	// hide density map when dragging
	emit hideDensityMap(false);

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

	// hide density map when wheeling
	emit hideDensityMap(false);

	QCustomPlot::wheelEvent(event); // forward event to QCustomPlot event handler
}

// ccSFvsSFWindowDlg

QCPScatterStyle::ScatterShape getShape(int index)
{
	switch (index) {
	case 0:
		return QCPScatterStyle::ssNone;
		break;
	case 1:
		return QCPScatterStyle::ssDot;
		break;
	case 2:
		return QCPScatterStyle::ssCross;
		break;
	case 3:
		return QCPScatterStyle::ssPlus;
		break;
	case 4:
		return QCPScatterStyle::ssCircle;
		break;
	case 5:
		return QCPScatterStyle::ssDisc;
		break;
	case 6:
		return QCPScatterStyle::ssSquare;
		break;
	case 7:
		return QCPScatterStyle::ssDiamond;
		break;
	case 8:
		return QCPScatterStyle::ssStar;
		break;
	default:
		return QCPScatterStyle::ssDisc;
		break;
	}
}

QCPColorGradient::GradientPreset getGradient(int index)
{
	switch (index) {
	case 0:
		return QCPColorGradient::gpGrayscale;
		break;
	case 1:
		return QCPColorGradient::gpHot;
		break;
	case 2:
		return QCPColorGradient::gpCold;
		break;
	case 3:
		return QCPColorGradient::gpNight;
		break;
	case 4:
		return QCPColorGradient::gpCandy;
		break;
	case 5:
		return QCPColorGradient::gpGeography;
		break;
	case 6:
		return QCPColorGradient::gpIon;
		break;
	case 7:
		return QCPColorGradient::gpThermal;
		break;
	case 8:
		return QCPColorGradient::gpPolar;
		break;
	case 9:
		return QCPColorGradient::gpSpectrum;
		break;
	case 10:
		return QCPColorGradient::gpJet;
		break;
	case 11:
		return QCPColorGradient::gpHues;
		break;
	default:
		return QCPColorGradient::gpSpectrum;
		break;
	}
}

ccSFvsSFWindowDlg::ccSFvsSFWindowDlg(ccPointCloud *cloud, QWidget *parent)
	: QDialog(parent)
	, m_cloud(cloud)
	, ui(new Ui::sfVsSFWindowDlg)
{	
	assert(cloud);

	ui->setupUi(this);

	m_plot = ui->widget;

	m_plot->clearGraphs();

	// add a color map
	m_colorMap = new QCPColorMap(m_plot->xAxis2, m_plot->yAxis2);
	m_colorMap->setInterpolate(false);
	// "background" "grid" "main" "axes" "legend" "overlay"
	m_plot->moveLayer(m_plot->xAxis->grid()->layer(), m_plot->layer("main"));
	m_plot->xAxis->grid();

	m_graph = m_plot->addGraph();
	m_graph->setLineStyle(QCPGraph::lsNone);
	QCPScatterStyle scatterStyle;
	QCPScatterStyle::ScatterShape shape = getShape(ui->comboBoxScatterStyle->currentIndex());
	scatterStyle.setShape(shape);
	scatterStyle.setPen(QPen(Qt::blue));
	scatterStyle.setBrush(Qt::white);
	scatterStyle.setSize(5);
	m_graph->setScatterStyle(scatterStyle);

	m_slr = m_plot->addGraph();
	QPen redPen;
	redPen.setColor(Qt::red);
	redPen.setStyle(Qt::DotLine);
	redPen.setWidthF(4);
	m_slr->setPen(redPen);

	// add a color scale:
	m_colorScale = new QCPColorScale(m_plot);
	m_colorScale->setType(QCPAxis::atRight); // scale shall be vertical bar with tick/axis labels right (actually atRight is already the default)
	m_plot->plotLayout()->addElement(0, 1, m_colorScale); // add it to the right of the main axis rect
	m_colorScale->axis()->setLabel("Density [pts / cell]");

	m_colorMap->setColorScale(m_colorScale); // associate the color map with the color scale

	// recover configuration
	QSettings settings("OSUR", "ccSFvsSFWindowDlg");
	ui->checkBoxXLog->setChecked(settings.value("xLog", false).toBool());
	ui->checkBoxYLog->setChecked(settings.value("yLog", false).toBool());
	ui->checkBoxShowGraph->setChecked(settings.value("showGraph", true).toBool());
	ui->checkBoxShowDensityMap->setChecked(settings.value("showDensityMap", false).toBool());
	m_nStepsX = settings.value("nbStepsX", 10).toInt();
	m_nStepsY = settings.value("nbStepsY", 10).toInt();
	ui->spinBoxNbStepsX->setValue(m_nStepsX);
	ui->spinBoxNbStepsY->setValue(m_nStepsY);
	ui->spinBoxMarkerSize->setValue(settings.value("markerSize", 5).toInt());
	ui->comboBoxScatterStyle->setCurrentIndex(settings.value("markerStyle", 5).toInt());
	ui->checkBoxInterpolateColorMap->setChecked(settings.value("interpolateColorMap", false).toBool());
	ui->comboBoxGradient->setCurrentIndex(settings.value("gradient", QCPColorGradient::gpGrayscale).toInt());

	ui->checkBoxLabelBold->setChecked(settings.value("labelBold", 0).toBool());
	ui->checkBoxTickBold->setChecked(settings.value("tickBold", 0).toBool());
	ui->spinBoxFontSize->setValue(settings.value("fontSize", 10).toUInt());
	ui->spinBoxTickLabelSize->setValue(settings.value("tickSize", 10).toUInt());
	ui->checkBoxWhiteBackground->setChecked(settings.value("whiteBackground", true).toBool());

	restoreGeometry(settings.value("geometry").toByteArray());

	setComboBoxes();

	refresh();

	showGraph(ui->checkBoxShowGraph->isChecked());
	showMap(ui->checkBoxShowDensityMap->isChecked());

	m_colorMap->setInterpolate(ui->checkBoxInterpolateColorMap->isChecked());
	setScatterStyle();
	setLabelFont();
	setTickFont();
	setXScaleType();
	setYScaleType();
	setGradient();

	connect(ui->checkBoxXLog, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setXScaleTypeAndReplot);
	connect(ui->checkBoxYLog, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setYScaleTypeAndReplot);
	connect(ui->checkBoxLogScaleDensityMap, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setColorScaleType);
	connect(ui->checkBoxShowGraph, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::showGraph);
	connect(ui->checkBoxShowDensityMap, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::showMap);
	connect(ui->checkBoxShowSLR, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::showSLR);
	connect(ui->pushButtonAutoscale, &QAbstractButton::pressed, this, &ccSFvsSFWindowDlg::autoscale);
	connect(ui->pushButtonLineaFit, &QAbstractButton::pressed, this, &ccSFvsSFWindowDlg::simpleLinearRegression);
	connect(ui->spinBoxNbStepsX, &QAbstractSpinBox::editingFinished, this, &ccSFvsSFWindowDlg::setNStepX);
	connect(ui->spinBoxNbStepsY, &QAbstractSpinBox::editingFinished, this, &ccSFvsSFWindowDlg::setNStepY);
	connect(ui->spinBoxMarkerSize, qOverload<int>(&QSpinBox::valueChanged), this, &ccSFvsSFWindowDlg::setScatterStyle);
	connect(ui->comboBoxScatterStyle, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccSFvsSFWindowDlg::setScatterStyle);
	connect(ui->checkBoxInterpolateColorMap, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setInterpolateColorMap);
	connect(ui->spinBoxFontSize, qOverload<int>(&QSpinBox::valueChanged), this, &ccSFvsSFWindowDlg::setLabelFont);
	connect(ui->spinBoxTickLabelSize, qOverload<int>(&QSpinBox::valueChanged), this, &ccSFvsSFWindowDlg::setTickFont);
	connect(ui->checkBoxLabelBold, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setLabelFont);
	connect(ui->checkBoxTickBold, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setTickFont);
	connect(ui->exportCSVToolButton, &QAbstractButton::clicked, this, &ccSFvsSFWindowDlg::onExportToCSV);
	connect(ui->exportImageToolButton, &QAbstractButton::clicked, this, &ccSFvsSFWindowDlg::onExportToImage);
	connect(ui->comboBoxGradient, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccSFvsSFWindowDlg::setGradient);
	connect(ui->checkBoxWhiteBackground, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setGradient);
	connect(ui->checkBoxReverseScale, &QCheckBox::stateChanged, this, &ccSFvsSFWindowDlg::setGradient);
	connect(m_plot, &ccSFvsSFPlot::hideDensityMap, ui->checkBoxShowDensityMap, &QCheckBox::setChecked);
}

ccSFvsSFWindowDlg::~ccSFvsSFWindowDlg()
{
	// save configuration
	QSettings settings("OSUR", "ccSFvsSFWindowDlg");
	settings.setValue("xIndex", ui->comboBoxXAxis->currentIndex());
	settings.setValue("yIndex", ui->comboBoxYAxis->currentIndex());
	settings.setValue("xLog", ui->checkBoxXLog->isChecked());
	settings.setValue("yLog", ui->checkBoxYLog->isChecked());
	settings.setValue("showGraph", ui->checkBoxShowGraph->isChecked());
	settings.setValue("showDensityMap", ui->checkBoxShowDensityMap->isChecked());
	settings.setValue("nbStepsX", ui->spinBoxNbStepsX->value());
	settings.setValue("nbStepsY", ui->spinBoxNbStepsY->value());
	settings.setValue("markerSize", ui->spinBoxMarkerSize->value());
	settings.setValue("markerStyle", ui->comboBoxScatterStyle->currentIndex());
	settings.setValue("interpolateColorMap", ui->checkBoxInterpolateColorMap->isChecked());
	settings.setValue("gradient", ui->comboBoxGradient->currentIndex());

	settings.setValue("labelBold", ui->checkBoxLabelBold->isChecked());
	settings.setValue("tickBold", ui->checkBoxTickBold->isChecked());
	settings.setValue("fontSize", ui->spinBoxFontSize->value());
	settings.setValue("tickSize", ui->spinBoxTickLabelSize->value());
	settings.setValue("whiteBackground", ui->checkBoxWhiteBackground->isChecked());

	settings.setValue("geometry", saveGeometry());

	delete ui;
}

void ccSFvsSFWindowDlg::setComboBoxes()
{
	unsigned int numberOfScalarFields = m_cloud->getNumberOfScalarFields();

	for (unsigned int index = 0; index < numberOfScalarFields; index++)
	{
		ui->comboBoxXAxis->addItem(QString::fromStdString(m_cloud->getScalarFieldName(index)));
		ui->comboBoxYAxis->addItem(QString::fromStdString(m_cloud->getScalarFieldName(index)));
	}

	// try to restore previous confirguration
	QSettings settings("OSUR", "ccSFvsSFWindowDlg");
	int previousXIndex = settings.value("xIndex", 0).toInt();
	int previousYIndex = settings.value("yIndex", 0).toInt();
	if (previousXIndex <= ui->comboBoxXAxis->count())
	{
		ui->comboBoxXAxis->setCurrentIndex(previousXIndex);
	}
	if (previousYIndex <= ui->comboBoxYAxis->count())
	{
		ui->comboBoxYAxis->setCurrentIndex(previousYIndex);
	}

	connect(ui->comboBoxXAxis, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ccSFvsSFWindowDlg::refresh);
	connect(ui->comboBoxYAxis, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ccSFvsSFWindowDlg::refresh);
}

void ccSFvsSFWindowDlg::refresh()
{
	int xIndex = ui->comboBoxXAxis->currentIndex();
	int yIndex = ui->comboBoxYAxis->currentIndex();

	// get data from scalar fields
	QVector<double> x;
	QVector<double> y;
	QString m_xName;
	QString m_yName;

	x.resize(m_cloud->size());
	y.resize(m_cloud->size());
	CCCoreLib::ScalarField *sfX = m_cloud->getScalarField(xIndex);
	CCCoreLib::ScalarField *sfY = m_cloud->getScalarField(yIndex);

	assert(sfX && sfY);

	for (unsigned int index = 0; index < m_cloud->size(); index++)
	{
		x[index] = sfX->getValue(index);
		y[index] = sfY->getValue(index);
	}
	m_xName = QString::fromStdString(sfX->getName());
	m_yName = QString::fromStdString(sfY->getName());

	// assign data to graph
	m_graph->data().clear();
	m_graph->setData(x, y);

	// give the axes some labels
	m_plot->xAxis->setLabel(m_xName);
	m_plot->yAxis->setLabel(m_yName);

	m_plot->rescaleAxes();
	m_plot->replot();

	densityMap();

	if (ui->checkBoxShowSLR->isChecked())
	{
		// update simple linear regression
		simpleLinearRegression();
	}
}

void ccSFvsSFWindowDlg::setXScaleType()
{
	if (ui->checkBoxXLog->isChecked())
	{
		// check that all values are > 0
		QSharedPointer<QCPGraphDataContainer> container = m_graph->data();
		bool foundKeyRange;
		QCPRange keyRange = container->keyRange(foundKeyRange);
		if (foundKeyRange && (keyRange.lower > 0) && (keyRange.upper > 0))
		{
			m_plot->xAxis->setScaleType(QCPAxis::stLogarithmic);
			m_plot->xAxis->setTicker(QSharedPointer<QCPAxisTickerLog>(new QCPAxisTickerLog));
		}
		else
		{
			ccLog::Error("[ccSFvsSFWindowDlg] impossible to change X scale, values <= 0 in the data");
			ui->checkBoxXLog->setChecked(false);
		}
	}
	else
	{
		m_plot->xAxis->setScaleType(QCPAxis::stLinear);
		m_plot->xAxis->setTicker(QSharedPointer<QCPAxisTicker>(new QCPAxisTicker));
	}
}

void ccSFvsSFWindowDlg::setXScaleTypeAndReplot()
{
	setXScaleType();

	densityMap();

	if (ui->checkBoxShowSLR->isChecked())
		simpleLinearRegression();
}

void ccSFvsSFWindowDlg::setYScaleType()
{
	if (ui->checkBoxYLog->isChecked())
	{
		// check that all values are > 0
		QSharedPointer<QCPGraphDataContainer> container = m_graph->data();
		bool foundValueRange;
		QCPRange valueRange = container->valueRange(foundValueRange);
		if (foundValueRange && (valueRange.lower > 0) && (valueRange.upper > 0))
		{
			m_plot->yAxis->setScaleType(QCPAxis::stLogarithmic);
			m_plot->yAxis->setTicker(QSharedPointer<QCPAxisTickerLog>(new QCPAxisTickerLog));
		}
		else
		{
			ccLog::Error("[ccSFvsSFWindowDlg] impossible to change Y scale, values <= 0 in the data");
			ui->checkBoxXLog->setChecked(false);
		}
	}
	else
	{
		m_plot->yAxis->setScaleType(QCPAxis::stLinear);
		m_plot->yAxis->setTicker(QSharedPointer<QCPAxisTicker>(new QCPAxisTicker));
	}
}

void ccSFvsSFWindowDlg::setYScaleTypeAndReplot()
{
	setYScaleType();

	densityMap();

	if (ui->checkBoxShowSLR->isChecked())
		simpleLinearRegression();
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

void ccSFvsSFWindowDlg::setNStepX()
{
	int value = ui->spinBoxNbStepsX->value();
	if (value != m_nStepsX)
	{
		m_nStepsX = value;
		// redraw the density map
		densityMap();
	}
}

void ccSFvsSFWindowDlg::setNStepY()
{
	int value = ui->spinBoxNbStepsY->value();
	if (value != m_nStepsY)
	{
		m_nStepsY = value;
		// redraw the density map
		densityMap();
	}
}

void ccSFvsSFWindowDlg::setScatterStyle()
{
	QCPScatterStyle scatterStyle = m_graph->scatterStyle();
	scatterStyle.setSize(ui->spinBoxMarkerSize->value());
	scatterStyle.setShape(getShape(ui->comboBoxScatterStyle->currentIndex()));
	m_graph->setScatterStyle(scatterStyle);
	m_plot->replot();
}

void ccSFvsSFWindowDlg::setGradient()
{
	QCPColorGradient::GradientPreset gradientPreset = getGradient(ui->comboBoxGradient->currentIndex());

	QCPColorGradient colorGradient(gradientPreset);

	QMap<double, QColor> map;

	if (ui->checkBoxReverseScale->isChecked())
	{
		QMap<double, QColor> reversedMap;
		map = colorGradient.colorStops();
		QMapIterator<double, QColor> i(map);
		while (i.hasNext()) {
			i.next();
			reversedMap[1.0 - i.key()] = i.value();
		}
		colorGradient.setColorStops(reversedMap);
	}

	if (ui->checkBoxWhiteBackground->isChecked())
	{
		map = colorGradient.colorStops();
		// add a step with the same color as the first one
		colorGradient.setColorStopAt(0.001, map.first());
		// set the first step to white
		colorGradient.setColorStopAt(0., QColorConstants::White);
	}

	m_colorMap->setGradient(colorGradient); // set the color gradient of the color map

	m_plot->replot();
}

void ccSFvsSFWindowDlg::setInterpolateColorMap(bool state)
{
	m_colorMap->setInterpolate(state);
	m_plot->replot();
}

void ccSFvsSFWindowDlg::setLabelFont()
{
	QFont serifFont;
	int size = ui->spinBoxFontSize->value();

	if (ui->checkBoxLabelBold->isChecked())
		serifFont = QFont("Times", size, QFont::Bold);
	else
		serifFont = QFont("Times", size);

	m_plot->xAxis->setLabelFont(serifFont);
	m_plot->yAxis->setLabelFont(serifFont);
	m_colorScale->axis()->setLabelFont(serifFont);

	m_plot->replot();
}

void ccSFvsSFWindowDlg::setTickFont()
{
	QFont serifFont;
	int size = ui->spinBoxTickLabelSize->value();

	if (ui->checkBoxTickBold->isChecked())
		serifFont = QFont("Times", size, QFont::Bold);
	else
		serifFont = QFont("Times", size);

	m_plot->xAxis->setTickLabelFont(serifFont);
	m_plot->yAxis->setTickLabelFont(serifFont);
	m_colorScale->axis()->setTickLabelFont(serifFont);

	m_plot->replot();
}

void ccSFvsSFWindowDlg::autoscale()
{
	m_plot->rescaleAxes();
	if (ui->checkBoxShowDensityMap->isChecked())
	{
		densityMap();
	}
}

void ccSFvsSFWindowDlg::densityMap()
{
	QCPRange keyRange = m_plot->xAxis->range();
	QCPRange valueRange = m_plot->yAxis->range();

	double xRange = keyRange.size();
	double xMin = keyRange.lower;
	double xMinLog = log10(xMin);
	double xRangeLog = log10(keyRange.upper) - log10(keyRange.lower);

	double yRange = valueRange.size();
	double yMin = valueRange.lower;
	double yMinLog = log10(yMin);
	double yRangeLog = log10(valueRange.upper) - log10(valueRange.lower);

	// compute the x and y steps of the map
	double stepX;
	if (ui->checkBoxXLog->isChecked())
	{
		stepX = xRangeLog / m_nStepsX;
	}
	else
	{
		stepX = xRange / m_nStepsX;
	}
	double stepY;
	if (ui->checkBoxYLog->isChecked())
	{
		stepY = yRangeLog / m_nStepsY;
	}
	else
	{
		stepY = yRange / m_nStepsY;
	}

	// fill the color map
	m_colorMap->data()->clear();
	m_colorMap->data()->setSize(m_nStepsX + 1, m_nStepsY + 1); // we want the color map to have nx * ny data points
	m_colorMap->data()->fill(0);
	QSharedPointer<QCPGraphDataContainer> container = m_graph->data();
	for (auto item : *container) // fill the colorMap data
	{
		int keyIdx;
		int valueIdx;

		if (item.key < keyRange.lower
			|| item.key > keyRange.upper
			|| item.value < valueRange.lower
			|| item.value > valueRange.upper)
		{ // the item is outside the displayed range
			continue;
		}

		if (ui->checkBoxXLog->isChecked())
		{
			keyIdx = floor((log10(item.key) - xMinLog + stepX / 2) / stepX);
		}
		else
		{
			keyIdx = floor((item.key - keyRange.lower + stepX / 2) / stepX);
		}

		if (ui->checkBoxYLog->isChecked())
		{
			valueIdx = floor((log10(item.value) - yMinLog + stepY / 2) / stepY);
		}
		else
		{
			valueIdx = floor((item.value - valueRange.lower + stepY / 2) / stepY);
		}

		double cell =  m_colorMap->data()->cell(keyIdx, valueIdx);
		m_colorMap->data()->setCell(keyIdx, valueIdx, cell + 1);
	}

	// range handling
	m_colorMap->data()->setRange(keyRange, valueRange);

	// rescale the data dimension (color) such that all data points lie in the span visualized by the color gradient:
	m_colorMap->rescaleDataRange(true);

	// make sure the axis rect and color scale synchronize their bottom and top margins (so they line up):
	QCPMarginGroup *marginGroup = new QCPMarginGroup(m_plot);
	m_plot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
	m_colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);

	m_plot->xAxis2->setRange(m_plot->xAxis->range());
	m_plot->yAxis2->setRange(m_plot->yAxis->range());

	/// update plot
	m_plot->replot();
}

void ccSFvsSFWindowDlg::showMap(bool state)
{
	m_colorMap->setVisible(state);
	showColorScale(state);
	if (state)
	{
		densityMap();
	}
}

void ccSFvsSFWindowDlg::simpleLinearRegression()
{
	double sum_x = 0;
	double sum_x2 = 0;
	double sum_y = 0;
	double sum_xy = 0;
	int n = m_graph->dataCount();
	int validCount = 0;

	QSharedPointer<QCPGraphDataContainer> container = m_graph->data();

	/* Calculating Required Sum */
	for (auto item : *container)
	{
		float x;
		float y;

		if (ui->checkBoxXLog->isChecked())
		{
			x = log10(item.key);
		}
		else
		{
			x = item.key;
		}

		if (ui->checkBoxYLog->isChecked())
		{
			y = log10(item.value);
		}
		else
		{
			y = item.value;
		}

		if (!isnan(x) && !isnan(y))
		{
			sum_x += x;
			sum_x2 += x * x;
			sum_y += y;
			sum_xy += x * y;
			validCount++;
		}
		else
		{
			ccLog::Print("NaN detected");
		}
	}

	/* Calculating alpha and beta */
	// The slope of the fitted line is equal to the correlation between y and x corrected by the ratio of standard deviations of these variables.
	// The intercept of the fitted line is such that the line passes through the center of mass (x, y) of the data points.
	double den = (n * sum_x2 - sum_x * sum_x);
	double alpha = (sum_y * sum_x2 - sum_x * sum_xy) / den;
	double beta  = (    n * sum_xy - sum_x * sum_y)  / den;

	// compute the coefficient of determination
	double sumOfSquaresOfResiduals = 0;
	double totalSumOfSquares = 0;
	double coefficientOfDetermination;
	double meanY = sum_y / validCount;
	double f;
	for (auto item : *container)
	{
		float x;
		float y;

		if (ui->checkBoxXLog->isChecked())
		{
			x = log10(item.key);
		}
		else
		{
			x = item.key;
		}

		if (ui->checkBoxYLog->isChecked())
		{
			y = log10(item.value);
		}
		else
		{
			y = item.value;
		}

		if (!isnan(x) && !isnan(y))
		{
			f = alpha + beta * x;
			sumOfSquaresOfResiduals += pow(y - f, 2);
			totalSumOfSquares += pow(y - meanY, 2);
		}
		else
		{
			ccLog::Print("NaN detected");
		}
	}
	coefficientOfDetermination = 1 - sumOfSquaresOfResiduals / totalSumOfSquares;

	// add the fit to the plot
	bool foundRange;
	double xMinLin = container->keyRange(foundRange).lower;
	double xMaxLin = container->keyRange(foundRange).upper;
	double xMin;
	double xMax;

	QVector<double> x_slr(2);
	QVector<double> y_slr(2);

	if (!ui->checkBoxXLog->isChecked() && ui->checkBoxYLog->isChecked())
	{
		x_slr[0] = xMinLin;
		x_slr[1] = xMaxLin;
		y_slr[0] = exp((alpha + beta * xMinLin) * log(10));
		y_slr[1] = exp((alpha + beta * xMaxLin) * log(10));
	}
	else if(ui->checkBoxXLog->isChecked() && !ui->checkBoxYLog->isChecked())
	{
		x_slr[0] = xMinLin;
		x_slr[1] = xMaxLin;
		y_slr[0] = alpha + beta * log10(xMinLin);
		y_slr[1] = alpha + beta * log10(xMaxLin);
	}
	else if(ui->checkBoxXLog->isChecked() && ui->checkBoxYLog->isChecked())
	{
		x_slr[0] = xMinLin;
		x_slr[1] = xMaxLin;
		y_slr[0] = exp((alpha + beta * log10(xMinLin)) * log(10));
		y_slr[1] = exp((alpha + beta * log10(xMaxLin)) * log(10));
	}
	else
	{
		xMin = xMinLin;
		xMax = xMaxLin;
		x_slr[0] = xMin;
		x_slr[1] = xMax;
		y_slr[0] = alpha + beta * xMin;
		y_slr[1] = alpha + beta * xMax;
	}

	m_slr->setData(x_slr, y_slr);
	m_plot->replot();

	ui->checkBoxShowSLR->setChecked(true);
	ui->labelAlpha->setText(QString::number(alpha));
	ui->labelBeta->setText(QString::number(beta));
	ui->doubleSpinBoxCoefficientOfDetermination->setValue(coefficientOfDetermination);
}

bool ccSFvsSFWindowDlg::exportToCSV(QString filename) const
{
	if (!m_graph || (m_graph->dataCount() == 0))
	{
		ccLog::Warning("[SF/SF] Plot has no associated values (can't save file)");
		return false;
	}

	QFile file(filename);
	if (!file.open(QFile::WriteOnly | QFile::Text))
	{
		ccLog::Warning(QString("[SF/SF] Failed to save plot to file '%1'").arg(filename));
		return false;
	}

	QTextStream stream(&file);
	stream.setRealNumberPrecision(12);
	stream.setRealNumberNotation(QTextStream::FixedNotation);

	//header
	stream << ui->comboBoxXAxis->currentText() << ", "
		   << ui->comboBoxYAxis->currentText() << endl;

	//data
	{
		QSharedPointer<QCPGraphDataContainer> container = m_graph->data();

		for (auto item : *container)
		{
			stream << item.key << " " << item.value << endl;
		}
	}

	file.close();

	ccLog::Print(QString("[SF/SF] File '%1' saved").arg(filename));

	return true;
}

void ccSFvsSFWindowDlg::onExportToCSV()
{
	if (!m_plot)
	{
		assert(false);
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	currentPath += QString("/") + m_plot->windowTitle() + ".csv";

	//ask for a filename
	QString filename = QFileDialog::getSaveFileName(this, "Select output file", currentPath, "*.csv");
	if (filename.isEmpty())
	{
		//process cancelled by user
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(), QFileInfo(filename).absolutePath());
	settings.endGroup();

	//save file
	exportToCSV(filename);
}

void ccSFvsSFWindowDlg::onExportToImage()
{
	if (!m_plot)
	{
		assert(false);
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString outputFilename = ImageFileFilter::GetSaveFilename("Select output file",
															  m_plot->windowTitle(),
															  currentPath,
															  this);

	if (outputFilename.isEmpty())
	{
		//process cancelled by user (or error)
		return;
	}

	//save current export path to persistent settings
	settings.setValue(ccPS::CurrentPath(), QFileInfo(outputFilename).absolutePath());
	settings.endGroup();

	//save the widget as an image file
	QPixmap image = m_plot->grab();
	if (image.save(outputFilename))
	{
		ccLog::Print(QString("[SF/SF] Image '%1' successfully saved").arg(outputFilename));
	}
	else
	{
		ccLog::Error(QString("Failed to save file '%1'").arg(outputFilename));
	}
}

void ccSFvsSFWindowDlg::showColorScale(bool state)
{
	m_colorScale->setVisible(state);
	if (state)
	{
		if (m_plot->plotLayout()->elementCount() == 2)
		{
			// nothing to do, the color scale is already on the plot
		}
		else
		{
			m_plot->plotLayout()->addElement(0, 1, m_colorScale); // add it to the right of the main axis rect
		}
	}
	else
	{
		m_plot->plotLayout()->take(m_colorScale);
		m_plot->plotLayout()->simplify();
	}

	m_plot->replot();
}

