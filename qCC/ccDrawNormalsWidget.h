#ifndef CCDRAWNORMALSWIDGET_H
#define CCDRAWNORMALSWIDGET_H

#include <QWidget>

#include "ccPointCloud.h"

namespace Ui {
class ccDrawNormalsWidget;
}

class ccDrawNormalsWidget : public QWidget
{
	Q_OBJECT

public:
	explicit ccDrawNormalsWidget(ccPointCloud * cloud, QWidget *parent = nullptr);
	~ccDrawNormalsWidget();
	void readSettings();
	void writeSettings();

public slots:
	void normalLengthValueChanged(double value);

private:
	Ui::ccDrawNormalsWidget *ui;
	ccPointCloud * m_cloud;
};

#endif // CCDRAWNORMALSWIDGET_H
