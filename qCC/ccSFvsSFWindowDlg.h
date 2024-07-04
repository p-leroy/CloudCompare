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
public:
	explicit ccSFvsSFPlot(QWidget *parent=0);

	void rescale();

	void mousePressEvent(QMouseEvent *event) override;
	void mouseDoubleClickEvent(QMouseEvent *event) override;
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

	void setYScaleType();

	void setColorScaleType();

	void densityMap();

	void showGraph(bool state){ m_graph->setVisible(state); m_plot->replot(); }

	void showMap(bool state){ m_colorMap->setVisible(state); m_plot->replot(); }

protected:

	//! Associated plot
	ccSFvsSFPlot* m_plot;

	//! Associated cloud
	ccPointCloud *m_cloud;

	QCPGraph* m_graph;

	QCPColorMap *m_colorMap;

	enum {
		GRAPH,
		COLORMAP
	} m_mode;

private:
	Ui::sfVsSFWindowDlg *ui;
};

#endif // CCSFVSSFWINDOWDLG_H
