#ifndef CCSFVSSFWINDOWDLG_H
#define CCSFVSSFWINDOWDLG_H

#include "ccPointCloud.h"

#include <QDialog>

namespace Ui {
class sfVsSFWindowDlg;
}

#include <qcustomplot.h>

class ccSFvsSFPlot : public QCustomPlot
{
	Q_OBJECT

public:
	explicit ccSFvsSFPlot(QWidget *parent=0);

	enum modeType{
		MOVE,
		SELECT
	} m_mode;

	void rescale();
	void setMode(modeType mode){m_mode = mode;}
	void selectionChanged();
	void autoscale();
	void onAxisDoubleClick(QCPAxis *axis, QCPAxis::SelectablePart part);
	void setAxisRange(QCPAxis *axis, double lower, double upper);

	void mousePressEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;

signals:
	void hideDensityMap(bool state);
};

class ccSFvsSFWindowDlg : public QDialog
{
	Q_OBJECT

public:
	explicit ccSFvsSFWindowDlg(ccPointCloud *cloud, QWidget *parent = nullptr);
	virtual ~ccSFvsSFWindowDlg();

	//! Returns encapsulated ccSFvsSFPlot
	inline ccSFvsSFPlot* plot() { return m_plot; }

	void setComboBoxes();

	void refresh();

	void setXScaleType();

	void setXScaleTypeAndReplot();

	void setYScaleType();

	void setYScaleTypeAndReplot();

	void setColorScaleType();

	void setNStepX();

	void setNStepY();

	void setScatterStyle();

	void setGradient();

	void setInterpolateColorMap(bool state);

	void setLabelFont();

	void setTickFont();

	void replot();

	void densityMap();

	void refreshDensityMapDisplay();

	void simpleLinearRegression();

	bool exportToCSV(QString filename) const;

	void onExportToCSV();

	void onExportToImage();

	void showGraph(bool state){ m_graph->setVisible(state); m_plot->replot(); }

	void showMap(bool state){ m_colorMap->setVisible(state); m_plot->replot(); }

	void showSLR(bool state){ m_slr->setVisible(state); m_plot->replot(); }

protected:

	//! Associated plot
	ccSFvsSFPlot* m_plot;

	//! Associated cloud
	ccPointCloud *m_cloud;

	QCPGraph* m_graph;

	QCPGraph* m_slr; // simple linear regression

	QCPColorMap* m_colorMap;

	QCPColorScale* m_colorScale;

	int m_nStepsX;

	int m_nStepsY;

	enum {
		GRAPH,
		COLORMAP
	} m_mode;

private:
	Ui::sfVsSFWindowDlg *ui;
};

#endif // CCSFVSSFWINDOWDLG_H
