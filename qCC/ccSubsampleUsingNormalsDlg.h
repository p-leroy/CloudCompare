#ifndef CCSUBSAMPLEUSINGNORMALSDLG_H
#define CCSUBSAMPLEUSINGNORMALSDLG_H

#include <QDialog>

#include <ccPointCloud.h>
#include <ccMainAppInterface.h>
#include <ccScalarField.h>

namespace Ui {
class ccSubsampleUsingNormalsDlg;
}

class ccSubsampleUsingNormalsDlg : public QDialog
{
    Q_OBJECT

public:

    enum ComputationMode
    {
        DEFAULT_MODE			= 0, //compute normals on core points
        USE_CLOUD1_NORMALS		= 1,
        MULTI_SCALE_MODE		= 2,
        VERT_MODE				= 3,
        HORIZ_MODE				= 4,
        USE_CORE_POINTS_NORMALS	= 5,
    };

    //! Default constructor
    ccSubsampleUsingNormalsDlg(ccMainAppInterface *app, QWidget *parent = nullptr);
    ~ccSubsampleUsingNormalsDlg();
    void readSettings();

    bool computeOctree(ccPointCloud *cloud);
    ccPointCloud* subsampleCloud(ccPointCloud *cloud, double spacing, bool addToDB);
    ccPointCloud* getSampledCloud(ccPointCloud *&cloud, const double &angle, bool keepIntermediateClouds);
    double getAngle();
    bool keepIntermediateClouds();
    void populateCorePtsComboBox();
    QString getEntityName(ccHObject *obj);
    bool computeNormals(ccPointCloud* coreCloud, ccPointCloud* refCloud);
    static void ComputeCorePointNormal(unsigned index);
    static void MakeNormalsHorizontal(NormsIndexesTableType& normsCodes);
    static bool ComputeCorePointsNormals(CCCoreLib::GenericIndexedCloud* corePoints,
                                  NormsIndexesTableType* corePointsNormals,
                                  ccGenericPointCloud* sourceCloud,
                                  const std::vector<PointCoordinateType>& sortedRadii,
                                  bool& invalidNormals,
                                  int maxThreadCount=0,
                                  ccScalarField* normalScale=nullptr,
                                  CCCoreLib::GenericProgressCallback* progressCb=nullptr,
                                  CCCoreLib::DgmOctree* inputOctree=nullptr);
    static bool UpdateNormalOrientationsWithCloud(CCCoreLib::GenericIndexedCloud* normCloud,
                                                  NormsIndexesTableType& normsCodes,
                                                  CCCoreLib::GenericIndexedCloud* orientationCloud,
                                                  int maxThreadCount=0,
                                                  CCCoreLib::GenericProgressCallback* progressCb=nullptr);
    ccPointCloud* getNormalsOrientationCloud() const;

    private:
        Ui::ccSubsampleUsingNormalsDlg *ui;

    ccMainAppInterface *m_app;
    QWidget *m_parent;
};

#endif // CCSUBSAMPLEUSINGNORMALSDLG_H
