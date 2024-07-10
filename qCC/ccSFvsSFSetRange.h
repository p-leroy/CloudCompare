#ifndef CCSFVSSFSETRANGE_H
#define CCSFVSSFSETRANGE_H

#include <QWidget>

#include <qcustomplot.h>

namespace Ui {
class sfVsSFSetRange;
}

class ccSFvsSFSetRange : public QWidget
{
	Q_OBJECT

public:
	explicit ccSFvsSFSetRange(QCPAxis* axis, QWidget *parent = nullptr);
	~ccSFvsSFSetRange();
	void setLower(double lower);
	void setUpper(double upper);
	void emitSetRange();

	void focusOutEvent(QFocusEvent* event);

signals:
	void setRange(QCPAxis *axis, double lower, double upper);

private:
	Ui::sfVsSFSetRange *ui;

	QCPAxis* m_axis;
};

#endif // CCSFVSSFSETRANGE_H
