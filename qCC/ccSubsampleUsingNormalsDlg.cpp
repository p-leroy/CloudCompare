#include "ccSubsampleUsingNormalsDlg.h"
#include "ui_ccSubsampleUsingNormalsDlg.h"

ccSubsampleUsingNormalsDlg::ccSubsampleUsingNormalsDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ccSubsampleUsingNormalsDlg)
{
    ui->setupUi(this);
}

ccSubsampleUsingNormalsDlg::~ccSubsampleUsingNormalsDlg()
{
    delete ui;
}

ccPointCloud* ccSubsampleUsingNormalsDlg::subsampleUsingNormals(ccPointCloud *&cloud, const double &angle)
{
    if (angle < 0)
        return nullptr;

    unsigned int Npts = cloud->size();
    NormsIndexesTableType *normsTable = cloud->normals(); // core cloud should have normals

    //==========================================================
    // build a cloud with the normal values as point coordinates

    // create the cloud
    ccPointCloud* normalCloud = new ccPointCloud();
    // add a scalar field corresponding to the global indexes
    int scalarIdx = normalCloud->addScalarField("globalIndex");
    normalCloud->reserve(Npts);
    CCCoreLib::ScalarField *globalIndexSF = normalCloud->getScalarField(scalarIdx);
    globalIndexSF->resizeSafe(Npts, true, CCCoreLib::NAN_VALUE);

    CCVector3 P;
    unsigned int NnormalCloud;
    for (unsigned int globalIdx = 0; globalIdx < Npts; globalIdx++)
    {
        P = ccNormalVectors::GetNormal(normsTable->getValue(globalIdx));
        if (P.x != P.x
                ||	P.y != P.y
                ||	P.z != P.z)
        {
            // point not valid, do not add it to the normalCloud
        }
        else
        {
            normalCloud->addPoint(P);
            NnormalCloud = normalCloud->size();
            globalIndexSF->setValue(NnormalCloud - 1, globalIdx);
        }
    }
    normalCloud->resize(normalCloud->size());

    // add the normal cloud to the DB
    normalCloud->setName("normalCloud");
    m_app->addToDB(normalCloud);

    // subsample the cloud
    ccPointCloud* subsampledNormalCloud;
    double spacing = 2 * sin(angle / 2 * M_PI / 180); // convert angle in spacing
    subsampledNormalCloud = subsampleCloud(normalCloud, spacing, nullptr, m_app, false);
    int ssGlobalIdx = subsampledNormalCloud->getScalarFieldIndexByName("globalIndex");
    CCCoreLib::ScalarField* ssGlobalIndexSF = subsampledNormalCloud->getScalarField(ssGlobalIdx);
    unsigned int NSubsampled = subsampledNormalCloud->size();

    // add the subsampled normal cloud to the DB
    subsampledNormalCloud->setName("subsampledNormalCloud");
    m_app->addToDB(subsampledNormalCloud);

    // keep only points which are in the subsampledNormalCloud
    bool keep;
    CCCoreLib::ReferenceCloud *tmpCloud = new CCCoreLib::ReferenceCloud(cloud);
    // for each point in the initial cloud, we check that it is in the ssGlobalIndexSF scalar field
    for (unsigned int globalIdx = 0; globalIdx < Npts; globalIdx++)
    {
        keep = false; // drop the point by default
        for (unsigned int idx = 0; idx < NSubsampled; idx++)
        {
            if (ssGlobalIndexSF->getValue(idx) == globalIdx)
                keep = true;
        }
        if (keep)
        {
            tmpCloud->addPointIndex(globalIdx);
        }
    }

    ccPointCloud *subsampledCloud = cloud->partialClone(tmpCloud);

    return subsampledCloud;
}
