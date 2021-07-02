#include "ccSubsampleUsingNormalsDlg.h"
#include "ui_ccSubsampleUsingNormalsDlg.h"

// CCCoreLib
#include <Neighbourhood.h>
#include <CloudSamplingTools.h>
#include <Jacobi.h>

// qCC_db
#include <ccOctreeProxy.h>
#include <ccProgressDialog.h>

// Qt
#include <QSettings>
#include <QThread>
#include <QtConcurrentMap>

static struct
{
    CCCoreLib::GenericIndexedCloud* corePoints;
    ccGenericPointCloud* sourceCloud;
    CCCoreLib::DgmOctree* octree;
    unsigned char octreeLevel;
    std::vector<PointCoordinateType> radii;
    NormsIndexesTableType* normCodes;
    ccScalarField* normalScale;
    bool invalidNormals;

    CCCoreLib::NormalizedProgress* nProgress;
    bool processCanceled;

} s_params;

static struct
{
    NormsIndexesTableType* normsCodes;
    CCCoreLib::GenericIndexedCloud* normCloud;
    CCCoreLib::GenericIndexedCloud* orientationCloud;

    CCCoreLib::NormalizedProgress* nProgress;
    bool processCanceled;

} s_normOriWithCloud;

ccSubsampleUsingNormalsDlg::ccSubsampleUsingNormalsDlg(ccMainAppInterface *app, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ccSubsampleUsingNormalsDlg),
    m_app(app),
    m_parent(parent)
{
    ui->setupUi(this);
    populateCorePtsComboBox();
}

ccSubsampleUsingNormalsDlg::~ccSubsampleUsingNormalsDlg()
{
    QSettings settings("OSUR", "ccSubsampleUsingNormalsDlg");

    settings.setValue("angle", ui->doubleSpinBox_angle->value());
    settings.setValue("keepIntermediateClouds", ui->checkBox_keepIntermediate->isChecked());

    delete ui;
}

void ccSubsampleUsingNormalsDlg::readSettings()
{
    QSettings settings("OSUR", "ccSubsampleUsingNormalsDlg");

    double angle = settings.value("angle", 0).toDouble();
    bool keep = settings.value("keepIntermediateClouds", false).toBool();

    ui->doubleSpinBox_angle->setValue(angle);
    ui->checkBox_keepIntermediate->setChecked(keep);
}

bool ccSubsampleUsingNormalsDlg::computeOctree(ccPointCloud *cloud)
{
    ccOctree::Shared octree = nullptr;
    QString errorMessage;

    //progress dialog
    ccProgressDialog pDlg(m_parent);

    //compute reference cloud octree if necessary
    octree = cloud->getOctree();
    if (!octree)
    {
        octree = cloud->computeOctree(&pDlg);
        if (octree && cloud->getParent() && m_app)
        {
            m_app->addToDB(cloud->getOctreeProxy());
        }
    }
    if (!octree)
    {
        errorMessage = "Failed to compute reference cloud octree!";
        return false;
    }

    QApplication::processEvents();

    return true;
}

ccPointCloud* ccSubsampleUsingNormalsDlg::subsampleCloud(ccPointCloud *cloud, double spacing, bool addToDB)
{
    m_app->dispToConsole("[ccSubsampleUsingNormalsDlg::subsampleCloud]");

    ccPointCloud *outputCloud = nullptr;
    ccGLMatrix transMat;

    //progress dialog
    ccProgressDialog pDlg(m_parent);

    if (!cloud->getOctree())
        computeOctree(cloud);

    CCCoreLib::CloudSamplingTools::SFModulationParams modParams(false);
    CCCoreLib::ReferenceCloud* subsampled = CCCoreLib::CloudSamplingTools::resampleCloudSpatially(
                cloud,
                static_cast<PointCoordinateType>(spacing),
                modParams,
                cloud->getOctree().data(),
                &pDlg);

    if (subsampled)
    {
        outputCloud = cloud->partialClone(subsampled);
        //don't need those references anymore
        delete subsampled;
        subsampled = nullptr;
    }

    if (outputCloud)
    {
        if (addToDB)
        {
            outputCloud->setName(cloud->getName() + "_core(subsampled " + QString::number(spacing) + ")");
            outputCloud->setVisible(true);
            outputCloud->setDisplay(cloud->getDisplay());
            if (m_app)
            {
                m_app->dispToConsole(QString("[M3C2] Subsampled cloud has been saved ('%1')").arg(outputCloud->getName()), ccMainAppInterface::STD_CONSOLE_MESSAGE);
                m_app->addToDB(outputCloud);
            }
        }
    }
    else
    {
        m_app->dispToConsole("Failed to compute subsampled core points!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
    }

    QApplication::processEvents();

    return outputCloud;
}

ccPointCloud* ccSubsampleUsingNormalsDlg::getSampledCloud(ccPointCloud *&cloud, const double &angle, bool keepIntermediateClouds)
{
    if (angle < 0)
        return nullptr;

    if (!cloud->hasNormals())
    {
        ccLog::Error("[ccSubsampleUsingNormalsDlg] cloud does not have normals");
        return nullptr;;
    }

    unsigned int Npts = cloud->size();
    NormsIndexesTableType *normsTable = cloud->normals(); // core cloud should have normals
    std::vector<ccPointCloud*> garbage;

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
    if (keepIntermediateClouds)
    {
        normalCloud->setName("normalCloud");
        m_app->addToDB(normalCloud);
    }
    else
        garbage.push_back(normalCloud);

    // subsample the cloud
    ccPointCloud* subsampledNormalCloud;
    double spacing = 2 * sin(angle / 2 * M_PI / 180); // convert angle in spacing
    subsampledNormalCloud = subsampleCloud(normalCloud, spacing, false);
    int ssGlobalIdx = subsampledNormalCloud->getScalarFieldIndexByName("globalIndex");
    CCCoreLib::ScalarField* ssGlobalIndexSF = subsampledNormalCloud->getScalarField(ssGlobalIdx);
    unsigned int NSubsampled = subsampledNormalCloud->size();

    // add the subsampled normal cloud to the DB
    if (keepIntermediateClouds)
    {
        subsampledNormalCloud->setName("subsampledNormalCloud");
        m_app->addToDB(subsampledNormalCloud);
    }
    else
        garbage.push_back(subsampledNormalCloud);

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

    while (!garbage.empty())
      {
        delete garbage.back();
        garbage.pop_back();
      }

    return subsampledCloud;
}

double ccSubsampleUsingNormalsDlg::getAngle()
{
    return ui->doubleSpinBox_angle->value();
}

bool ccSubsampleUsingNormalsDlg::keepIntermediateClouds()
{
    return ui->checkBox_keepIntermediate->isChecked();
}

void ccSubsampleUsingNormalsDlg::populateCorePtsComboBox()
{
    if (m_app)
    {
        //add list of clouds to the combo-boxes
        ccHObject::Container clouds;
        if (m_app->dbRootObject())
        {
            m_app->dbRootObject()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
        }

        for (size_t i = 0; i < clouds.size(); ++i)
        {
            if (clouds[i]->isA(CC_TYPES::POINT_CLOUD)) //as filterChildren only test 'isKindOf'
            {
                ui->comboBox_referenceCloud->addItem(getEntityName(clouds[i]), QVariant(clouds[i]->getUniqueID()));
            }
        }
    }
}

QString ccSubsampleUsingNormalsDlg::getEntityName(ccHObject *obj)
{
    if (!obj)
    {
        assert(false);
        return QString();
    }

    QString name = obj->getName();
    if (name.isEmpty())
        name = "unnamed";
    name += QString(" [ID %1]").arg(obj->getUniqueID());

    return name;
}

bool ccSubsampleUsingNormalsDlg::computeNormals(ccPointCloud* coreCloud, ccPointCloud* refCloud)
{
    QSettings settings("OSUR", "qICPM3C2");
    double normalScale = settings.value("NormalScale", 0.).toDouble();
    unsigned int maxThreadCount = settings.value("MaxThreadCount", 1).toUInt();
    unsigned int normalMode = settings.value("NormalMode").toUInt();

    QString errorMessage;
    bool normalsAreOk = false;
    bool invalidNormals = false;
    bool error = false;
    std::vector<PointCoordinateType> radii;
    ccScalarField* normalScaleSF = nullptr; //normal scale (multi-scale mode only)
    NormsIndexesTableType *coreNormals = new NormsIndexesTableType();

    //progress dialog
    ccProgressDialog pDlg(m_parent);

    radii.push_back(static_cast<PointCoordinateType>(normalScale / 2)); //we want the radius in fact ;)

    //dedicated core points method
    normalsAreOk = ComputeCorePointsNormals(coreCloud,
                                            coreNormals,
                                            refCloud,
                                            radii,
                                            invalidNormals,
                                            maxThreadCount,
                                            normalScaleSF,
                                            &pDlg,
                                            refCloud->getOctree().data());

    //now fix the orientation
    if (normalsAreOk)
    {
        //some invalid normals?
        if (invalidNormals && m_app)
        {
            m_app->dispToConsole("[M3C2] Some normals are invalid! You may have to increase the scale.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
        }

        //make normals horizontal if necessary
        if (normalMode == HORIZ_MODE)
        {
            MakeNormalsHorizontal(*coreNormals);
        }

        //then either use a simple heuristic
        bool usePreferredOrientation = ui->normOriPreferredRadioButton->isChecked();
        if (usePreferredOrientation)
        {
            int preferredOrientation = ui->normOriPreferredComboBox->currentIndex();
            assert(preferredOrientation >= ccNormalVectors::MINUS_X && preferredOrientation <= ccNormalVectors::PLUS_ZERO);
            if (!ccNormalVectors::UpdateNormalOrientations(coreCloud,
                                                           *coreNormals,
                                                           static_cast<ccNormalVectors::Orientation>(preferredOrientation)))
            {
                errorMessage = "[ICPM3C2] Failed to re-orient the normals (invalid parameter?)";
                error = true;
            }
        }
        else //or use external points
        {
            ccPointCloud* orientationCloud = getNormalsOrientationCloud();
            assert(orientationCloud);

            if (!UpdateNormalOrientationsWithCloud(coreCloud,
                                                   *coreNormals,
                                                   orientationCloud,
                                                   maxThreadCount,
                                                   &pDlg))
            {
                errorMessage = "[ICPM3C2] Failed to re-orient the normals with input point cloud!";
                error = true;
            }
        }

        if (!error && coreNormals)
        {
            coreCloud->setNormsTable(coreNormals);
            coreCloud->showNormals(true);
        }
    }

    if (!error && coreNormals)
    {
        if (coreCloud->hasNormals() || coreCloud->resizeTheNormsTable())
        {
            for (unsigned i = 0; i < coreNormals->currentSize(); ++i)
                coreCloud->setPointNormalIndex(i, coreNormals->getValue(i));
            coreCloud->showNormals(true);
            m_app->dispToConsole("coreNormals size: " + QString::number(coreNormals->size()));
        }
        else if (m_app)
        {
            m_app->dispToConsole("Failed to allocate memory for core points normals!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
        }
    }

    return true;
}

bool ccSubsampleUsingNormalsDlg::ComputeCorePointsNormals(CCCoreLib::GenericIndexedCloud* corePoints,
                                            NormsIndexesTableType* corePointsNormals,
                                            ccGenericPointCloud* sourceCloud,
                                            const std::vector<PointCoordinateType>& sortedRadii,
                                            bool& invalidNormals,
                                            int maxThreadCount/*=0*/,
                                            ccScalarField* normalScale/*=nullptr*/,
                                            CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/,
                                            CCCoreLib::DgmOctree* inputOctree/*=nullptr*/)
{
    assert(corePoints && sourceCloud && corePointsNormals);
    assert(!sortedRadii.empty());

    invalidNormals = true;

    unsigned corePtsCount = corePoints->size();
    if (corePtsCount == 0)
        return false;

    if (normalScale)
    {
        if (normalScale->currentSize() != corePtsCount && !normalScale->resizeSafe(corePtsCount))
        {
            //not enough memory
            return false;
        }
        normalScale->fill(CCCoreLib::NAN_VALUE);
    }

    CCCoreLib::DgmOctree* theOctree = inputOctree;
    if (!theOctree)
    {
        theOctree = new CCCoreLib::DgmOctree(sourceCloud);
        if (theOctree->build() == 0)
        {
            delete theOctree;
            return false;
        }
    }

    CCCoreLib::NormalizedProgress nProgress(progressCb, corePtsCount);
    if (progressCb)
    {
        if (progressCb->textCanBeEdited())
        {
            progressCb->setInfo(qPrintable(QString("Core points: %1\nSource points: %2").arg(corePtsCount).arg(sourceCloud->size())));
            progressCb->setMethodTitle("Computing normals");
        }
        progressCb->start();
    }

    //reserve memory for normals (codes) storage
    if (!corePointsNormals->isAllocated() || corePointsNormals->currentSize() < corePtsCount)
    {
        if (!corePointsNormals->resizeSafe(corePtsCount))
        {
            if (!inputOctree)
                delete theOctree;
            return false;
        }
    }
    PointCoordinateType biggestRadius = sortedRadii.back(); //we extract the biggest neighborhood
    unsigned char octreeLevel = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(biggestRadius);

    s_params.corePoints = corePoints;
    s_params.normCodes = corePointsNormals;
    s_params.sourceCloud = sourceCloud;
    s_params.radii = sortedRadii;
    s_params.octree = theOctree;
    s_params.octreeLevel = octreeLevel;
    s_params.nProgress = progressCb ? &nProgress : nullptr;
    s_params.processCanceled = false;
    s_params.invalidNormals = false;
    s_params.normalScale = normalScale;

    //we try the parallel way (if we have enough memory)
    bool useParallelStrategy = true;
#ifdef _DEBUG
    useParallelStrategy = false;
#endif

    std::vector<unsigned> corePointsIndexes;
    if (useParallelStrategy)
    {
        try
        {
            corePointsIndexes.resize(corePtsCount);
        }
        catch (const std::bad_alloc&)
        {
            //not enough memory
            useParallelStrategy = false;
        }
    }

    if (useParallelStrategy)
    {
        for (unsigned i = 0; i < corePtsCount; ++i)
        {
            corePointsIndexes[i] = i;
        }

        if (maxThreadCount == 0)
        {
            maxThreadCount = QThread::idealThreadCount();
        }
        QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
        QtConcurrent::blockingMap(corePointsIndexes, ComputeCorePointNormal);
    }
    else
    {
        //manually call the static per-point method!
        for (unsigned i = 0; i < corePtsCount; ++i)
        {
            ComputeCorePointNormal(i);
        }
    }

    //output flags
    bool wasCanceled = s_params.processCanceled;
    invalidNormals = s_params.invalidNormals;

    if (progressCb)
    {
        progressCb->stop();
    }

    if (!inputOctree)
        delete theOctree;

    return !wasCanceled;
}

void ccSubsampleUsingNormalsDlg::ComputeCorePointNormal(unsigned index)
{
    if (s_params.processCanceled)
        return;

    CCVector3 bestNormal(0, 0, 0);
    ScalarType bestScale = CCCoreLib::NAN_VALUE;

    const CCVector3* P = s_params.corePoints->getPoint(index);
    CCCoreLib::DgmOctree::NeighboursSet neighbours;
    CCCoreLib::ReferenceCloud subset(s_params.sourceCloud);

    int n = s_params.octree->getPointsInSphericalNeighbourhood(*P,
                                                               s_params.radii.back(), //we use the biggest neighborhood
                                                               neighbours,
                                                               s_params.octreeLevel);

    //if the widest neighborhood has less than 3 points in it, there's nothing we can do for this core point!
    if (n >= 3)
    {
        size_t radiiCount = s_params.radii.size();

        double bestPlanarityCriterion = 0;
        unsigned bestSamplePointCount = 0;

        for (size_t i = 0; i < radiiCount; ++i)
        {
            double radius = s_params.radii[radiiCount - 1 - i]; //we start from the biggest
            double squareRadius = radius*radius;

            subset.clear(false);
            for (unsigned j = 0; j < static_cast<unsigned>(n); ++j)
                if (neighbours[j].squareDistd <= squareRadius)
                    subset.addPointIndex(neighbours[j].pointIndex);

            //as we start from the biggest neighborhood, if we have less than 3 points for the current radius
            //it won't be better for the next one(s)!
            if (subset.size() < 3)
                break;
            //see the 'M3C2' article:
            //<< we ensure that a minimum of 10 points is used to compute the normal at Dopt
            //	otherwise we choose the scale immediatly larger that fulfils this requirement >>
            //if (subset.size() < 10) //DGM -> not implemented in Brodu's code?!
            //	break;

            CCCoreLib::Neighbourhood Z(&subset);

            /*** we manually compute the least squares best fitting plane (so as to get the PCA eigen values) ***/

            //we determine the plane normal by computing the smallest eigen value of M = 1/n * S[(p-µ)*(p-µ)']
            CCCoreLib::SquareMatrixd eigVectors;
            std::vector<double> eigValues;
            if (CCCoreLib::Jacobi<double>::ComputeEigenValuesAndVectors(Z.computeCovarianceMatrix(), eigVectors, eigValues, true))
            {
                /*** code and comments below are from the original 'm3c2' code (N. Brodu) ***/

                // The most 2D scale. For the criterion for how "2D" a scale is, see canupo
                // Ideally first and second eigenvalue are equal
                // convert to percent variance explained by each dim
                double totalvar = 0;
                CCVector3d svalues;
                for (unsigned k = 0; k < 3; ++k)
                {
                    // singular values are squared roots of eigenvalues
                    svalues.u[k] = eigValues[k];
                    svalues.u[k] = svalues.u[k] * svalues.u[k];
                    totalvar += svalues.u[k];
                }
                svalues /= totalvar;
                //sort eigenvalues
                std::sort(svalues.u, svalues.u + 3);
                std::swap(svalues.x, svalues.z);

                // ideally, 2D means first and second entries are both 1/2 and third is 0
                // convert to barycentric coordinates and take the coefficient of the 2D
                // corner as a quality measure.
                // Use barycentric coordinates : a for 1D, b for 2D and c for 3D
                // Formula on wikipedia page for barycentric coordinates
                // using directly the triangle in %variance space, they simplify a lot
                // double a = svalues[0] - svalues[1];
                double b = 2 * svalues[0] + 4 * svalues[1] - 2;
                // double c = 1 - a - b; // they sum to 1
                if (bestSamplePointCount == 0 || b > bestPlanarityCriterion)
                {
                    bestPlanarityCriterion = b;
                    bestSamplePointCount = subset.size();

                    //the smallest eigen vector corresponds to the "least square best fitting plane" normal
                    double vec[3];
                    double minEigValue = 0;
                    CCCoreLib::Jacobi<double>::GetMinEigenValueAndVector(eigVectors, eigValues, minEigValue, vec);

                    CCVector3 N = CCVector3::fromArray(vec);
                    N.normalize();

                    bestNormal = N;
                    bestScale = static_cast<ScalarType>(radius * 2);
                }
            }
        }

        if (bestSamplePointCount < 3)
        {
            s_params.invalidNormals = true;
        }
    }
    else
    {
        s_params.invalidNormals = true;
    }

    //compress the best normal and store it
    CompressedNormType normCode = ccNormalVectors::GetNormIndex(bestNormal.u);
    s_params.normCodes->setValue(index, normCode);

    //if necessary, store 'best radius'
    if (s_params.normalScale)
        s_params.normalScale->setValue(index, bestScale);

    //progress notification
    if (s_params.nProgress && !s_params.nProgress->oneStep())
    {
        s_params.processCanceled = true;
    }
}

void ccSubsampleUsingNormalsDlg::MakeNormalsHorizontal(NormsIndexesTableType& normsCodes)
{
    //for each normal
    unsigned count = normsCodes.currentSize();
    for (unsigned i = 0; i < count; i++)
    {
        const CompressedNormType& nCode = normsCodes.getValue(i);

        //de-compress the normal
        CCVector3 N(ccNormalVectors::GetNormal(nCode));

        N.z = 0;
        N.normalize();

        //re-compress the normal
        normsCodes.setValue(i,ccNormalVectors::GetNormIndex(N.u));
    }
}

static void OrientPointNormalWithCloud(unsigned index)
{
    if (s_normOriWithCloud.processCanceled)
        return;

    const CompressedNormType& nCode = s_normOriWithCloud.normsCodes->getValue(index);
    CCVector3 N(ccNormalVectors::GetNormal(nCode));

    //corresponding point
    const CCVector3* P = s_normOriWithCloud.normCloud->getPoint(index);

    //find nearest point in 'orientation cloud'
    //(brute force: we don't expect much points!)
    CCVector3 orientation(0, 0, 1);
    PointCoordinateType minSquareDist = 0;
    for (unsigned j = 0; j < s_normOriWithCloud.orientationCloud->size(); ++j)
    {
        const CCVector3* Q = s_normOriWithCloud.orientationCloud->getPoint(j);
        CCVector3 PQ = (*Q - *P);
        PointCoordinateType squareDist = PQ.norm2();
        if (j == 0 || squareDist < minSquareDist)
        {
            orientation = PQ;
            minSquareDist = squareDist;
        }
    }

    //we check sign
    if (N.dot(orientation) < 0)
    {
        //inverse normal and re-compress it
        N *= -1;
        s_normOriWithCloud.normsCodes->setValue(index, ccNormalVectors::GetNormIndex(N.u));
    }

    if (s_normOriWithCloud.nProgress && !s_normOriWithCloud.nProgress->oneStep())
    {
        s_normOriWithCloud.processCanceled = true;
    }
}

bool ccSubsampleUsingNormalsDlg::UpdateNormalOrientationsWithCloud(	CCCoreLib::GenericIndexedCloud* normCloud,
                                                        NormsIndexesTableType& normsCodes,
                                                        CCCoreLib::GenericIndexedCloud* orientationCloud,
                                                        int maxThreadCount/*=0*/,
                                                        CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/)
{
    //input normals
    unsigned count = normsCodes.currentSize();
    if (!normCloud || normCloud->size() != count)
    {
        assert(false);
        ccLog::Warning("[qM3C2Tools::UpdateNormalOrientationsWithCloud] Cloud/normals set mismatch!");
        return false;
    }

    //orientation points
    unsigned orientationCount = orientationCloud ? orientationCloud->size() : 0;
    if (orientationCount == 0)
    {
        //nothing to do?
        assert(false);
        return true;
    }

    CCCoreLib::NormalizedProgress nProgress(progressCb, count);
    if (progressCb)
    {
        if (progressCb->textCanBeEdited())
        {
            progressCb->setInfo(qPrintable(QString("Normals: %1\nOrientation points: %2").arg(count).arg(orientationCloud->size())));
            progressCb->setMethodTitle("Orienting normals");
        }
        progressCb->start();
    }

    s_normOriWithCloud.normCloud = normCloud;
    s_normOriWithCloud.orientationCloud = orientationCloud;
    s_normOriWithCloud.normsCodes = &normsCodes;
    s_normOriWithCloud.nProgress = &nProgress;
    s_normOriWithCloud.processCanceled = false;

    //we check each normal's orientation
    {
        std::vector<unsigned> pointIndexes;
        bool useParallelStrategy = true;
#ifdef _DEBUG
        useParallelStrategy = false;
#endif
        if (useParallelStrategy)
        {
            try
            {
                pointIndexes.resize(count);
            }
            catch (const std::bad_alloc&)
            {
                //not enough memory
                useParallelStrategy = false;
            }
        }

        if (useParallelStrategy)
        {
            for (unsigned i = 0; i < count; ++i)
            {
                pointIndexes[i] = i;
            }

            if (maxThreadCount == 0)
            {
                maxThreadCount = QThread::idealThreadCount();
            }
            QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
            QtConcurrent::blockingMap(pointIndexes, OrientPointNormalWithCloud);
        }
        else
        {
            //manually call the static per-point method!
            for (unsigned i = 0; i < count; ++i)
            {
                OrientPointNormalWithCloud(i);
            }
        }
    }

    if (progressCb)
    {
        progressCb->stop();
    }

    return true;
}

static ccPointCloud* GetCloudFromCombo(QComboBox* comboBox, ccHObject* dbRoot)
{
    assert(comboBox && dbRoot);
    if (!comboBox || !dbRoot)
    {
        assert(false);
        return nullptr;
    }

    //return the cloud currently selected in the combox box
    int index = comboBox->currentIndex();
    if (index < 0)
    {
        assert(false);
        return nullptr;
    }
    assert(comboBox->itemData(index).isValid());
    unsigned uniqueID = comboBox->itemData(index).toUInt();
    ccHObject* item = dbRoot->find(uniqueID);
    if (!item || !item->isA(CC_TYPES::POINT_CLOUD))
    {
        assert(false);
        return nullptr;
    }
    return static_cast<ccPointCloud*>(item);
}

ccPointCloud* ccSubsampleUsingNormalsDlg::getNormalsOrientationCloud() const
{
    if (ui->normOriUseCloudRadioButton->isChecked())
    {
        //return the cloud currently selected in the combox box
        return GetCloudFromCombo(ui->normOriCloudComboBox, m_app->dbRootObject());
    }
    else
    {
        return nullptr;
    }
}
