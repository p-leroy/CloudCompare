#ifndef CCSUBSAMPLEUSINGNORMALSDLG_H
#define CCSUBSAMPLEUSINGNORMALSDLG_H

#include <QDialog>

#include <ccPointCloud.h>

namespace Ui {
class ccSubsampleUsingNormalsDlg;
}

class ccSubsampleUsingNormalsDlg : public QDialog
{
    Q_OBJECT

public:
    explicit ccSubsampleUsingNormalsDlg(QWidget *parent = nullptr);
    ~ccSubsampleUsingNormalsDlg();

    ccPointCloud* subsampleUsingNormals(ccPointCloud *&cloud, const double &angle);

private:
    Ui::ccSubsampleUsingNormalsDlg *ui;
};

#endif // CCSUBSAMPLEUSINGNORMALSDLG_H
