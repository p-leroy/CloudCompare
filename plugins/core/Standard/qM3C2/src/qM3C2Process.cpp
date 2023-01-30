//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#include "qM3C2Process.h"

//local
#include "qM3C2Tools.h"
#include "qM3C2Dialog.h"

//CCCoreLib
#include <CloudSamplingTools.h>
#include <Neighbourhood.h>
#include <Jacobi.h>

//qCC_plugins
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccHObjectCaster.h>
#include <ccProgressDialog.h>
#include <ccNormalVectors.h>
#include <ccScalarField.h>
#include <ccColorScalesManager.h>

//Qt
#include <QtGui>
#include <QtCore>
#include <QApplication>
#include <QElapsedTimer>
#include <QtConcurrentMap>
#include <QMessageBox>

#include <iostream>

// Boost
#include <boost/math/distributions/students_t.hpp>

//! Default name for M3C2 scalar fields
static const char M3C2_DIST_SF_NAME[]			= "M3C2 distance";
static const char DIST_UNCERTAINTY_SF_NAME[]	= "distance uncertainty";
static const char SIG_CHANGE_SF_NAME[]			= "significant change";
static const char STD_DEV_CLOUD1_SF_NAME[]		= "%1_cloud1";
static const char STD_DEV_CLOUD2_SF_NAME[]		= "%1_cloud2";
static const char DENSITY_CLOUD1_SF_NAME[]		= "Npoints_cloud1";
static const char DENSITY_CLOUD2_SF_NAME[]		= "Npoints_cloud2";
static const char NORMAL_SCALE_SF_NAME[]		= "normal scale";
static const char SEARCH_DEPTH1_SF_NAME[]		= "search depth 1";
static const char SEARCH_DEPTH2_SF_NAME[]		= "search depth 2";
static const char WELCH_T_SF_NAME[]             = "Welch t (test)";
static const char WELCH_V_SF_NAME[]             = "Welch v (degrees of freedom)";
static const char WELCH_Q_SF_NAME[]             = "Welch q (probability)";
static const char WELCH_P_SF_NAME[]             = "Welch p (critical value)";
static const char WELCH_LOD_SF_NAME[]           = "Welch level of detection";
static const char WELCH_SIG_SF_NAME[]           = "Welch significant change";
static const char MEAN_MINUS_MEDIAN_1_SF[]      = "Mean minus median 1";
static const char MEAN_MINUS_MEDIAN_2_SF[]      = "Mean minus median 2";
static const char NORMALS_ANGLE_SF[]            = "Normals angle";
static const char SECTOR_SF[]                   = "sector1";
static const char GAP_SF[]                      = "gap1";
static const char SECTOR2_SF[]                  = "sector2";
static const char GAP2_SF[]                     = "gap2";
static const char SECTOR_GAP_SF[]               = "sectorGap";

static ccPointCloud *projectionCloud;
static int projCloud_sfIdx_index;
static int projCloud_sfIdx_cloud;
static CCCoreLib::ScalarField *projCloud_SF_index;
static CCCoreLib::ScalarField *projCloud_SF_cloud;
static bool storeProjectionInfo = false;
static bool exportSearchDepth = false;
static bool computeWelch = false;
static bool sharpMean = false;

static void RemoveScalarField(ccPointCloud* cloud, const char sfName[])
{
	int sfIdx = cloud ? cloud->getScalarFieldIndexByName(sfName) : -1;
	if (sfIdx >= 0)
	{
		cloud->deleteScalarField(sfIdx);
	}
}

static ScalarType SCALAR_ZERO = 0;
static ScalarType SCALAR_ONE = 1;

// Precision maps (See "3D uncertainty-based topographic change detection with SfM photogrammetry: precision maps for ground control and directly georeferenced surveys" by James et al.)
struct PrecisionMaps
{
	PrecisionMaps() : sX(nullptr), sY(nullptr), sZ(nullptr), scale(1.0) {}
	bool valid() const { return (sX != nullptr && sY != nullptr && sZ != nullptr); }
	CCCoreLib::ScalarField *sX, *sY, *sZ;
	double scale;
};

// Computes the uncertainty based on 'precision maps' (as scattered scalar fields)
static double ComputePMUncertainty(CCCoreLib::DgmOctree::NeighboursSet& set, const CCVector3& N, const PrecisionMaps& PM)
{
	size_t count = set.size();
	if (count == 0)
	{
		assert(false);
		return 0;
	}
	
	int minIndex = -1;
	if (count == 1)
	{
		minIndex = 0;
	}
	else
	{
		//compute gravity center
		CCVector3d G(0, 0, 0);
		for (size_t i = 0; i < count; ++i)
		{
			G.x += set[i].point->x;
			G.y += set[i].point->y;
			G.z += set[i].point->z;
		}

		G.x /= count;
		G.y /= count;
		G.z /= count;

		//now look for the point that is the closest to the gravity center
		double minSquareDist = -1.0;
		minIndex = -1;
		for (size_t i = 0; i < count; ++i)
		{
			CCVector3d dG(	G.x - set[i].point->x,
							G.y - set[i].point->y,
							G.z - set[i].point->z );
			double squareDist = dG.norm2();
			if (minIndex < 0 || squareDist < minSquareDist)
			{
				minSquareDist = squareDist;
				minIndex = static_cast<int>(i);
			}
		}
	}
	
	assert(minIndex >= 0);
	unsigned pointIndex = set[minIndex].pointIndex;
	CCVector3d sigma(	PM.sX->getValue(pointIndex) * PM.scale,
						PM.sY->getValue(pointIndex) * PM.scale,
						PM.sZ->getValue(pointIndex) * PM.scale);

	CCVector3d NS(	N.x * sigma.x,
					N.y * sigma.y,
					N.z * sigma.z);
	
	return NS.norm();
}

// Structure for parallel call to ComputeM3C2DistForPoint
struct M3C2Params
{
	//input data
	ccPointCloud* outputCloud = nullptr;
    ccPointCloud* outputCloud2 = nullptr;
	ccPointCloud* corePoints = nullptr;
	NormsIndexesTableType* coreNormals = nullptr;
    NormsIndexesTableType* coreNormals2 = nullptr;

	//main options
	PointCoordinateType projectionRadius = 0;
	PointCoordinateType projectionDepth = 0;
	bool updateNormal = false;
	bool exportNormal = false;
	bool computeConfidence = false;
	bool progressiveSearch = false;
	bool onlyPositiveSearch = false;
	unsigned minPoints4Stats = 3;
	double registrationRms = 0;

	//export
	qM3C2Dialog::ExportOptions exportOption;
	bool keepOriginalCloud = false;

	//octrees
	ccOctree::Shared cloud1Octree;
	unsigned char level1 = 0;
	ccOctree::Shared cloud2Octree;
	unsigned char level2 = 0;

	//scalar fields
	ccScalarField* m3c2DistSF = nullptr;		//M3C2 distance
	ccScalarField* distUncertaintySF = nullptr;	//distance uncertainty
	ccScalarField* sigChangeSF = nullptr;		//significant change
	ccScalarField* stdDevCloud1SF = nullptr;	//standard deviation information for cloud #1
	ccScalarField* stdDevCloud2SF = nullptr;	//standard deviation information for cloud #2
	ccScalarField* densityCloud1SF = nullptr;	//export point density at projection scale for cloud #1
	ccScalarField* densityCloud2SF = nullptr;	//export point density at projection scale for cloud #2
	ccScalarField* searchDepth1SF = nullptr;    //the search depth used during the projection of cloud #1
	ccScalarField* searchDepth2SF = nullptr;    //the search depth used during the projection of cloud #2
	ccScalarField* indexSF = nullptr;           // index of the core point
	ccScalarField* welch_t_SF = nullptr;        // Welch's test
	ccScalarField* welch_v_SF = nullptr;        // Welch's test degrees of freedom
	ccScalarField* welch_q_SF = nullptr;        // probability that difference is due to chance
	ccScalarField* welch_p_SF = nullptr;        // critical value
	ccScalarField* welch_lod_SF = nullptr;      // lod aka distance uncertainty
	ccScalarField* welch_sig_SF = nullptr;      // significant change
	ccScalarField* meanMinusMed1SF = nullptr;   // the search depth used during the projection of cloud #1
	ccScalarField* meanMinusMed2SF = nullptr;   // the search depth used during the projection of cloud #2
	ccScalarField* normalsAngleSF = nullptr;    // the angle between the normal and the normal of the extracted cloud 2 neighbour set
	ccScalarField* sector1SF = nullptr;          // angular sector covered by the points of cloud 2 projected in the plane perpendicular to the normal
	ccScalarField* gap1SF = nullptr;
	ccScalarField* sector2SF = nullptr;         // angular sector covered by the points of cloud 2 projected in the plane perpendicular to the normal
	ccScalarField* gap2SF = nullptr;
	ccScalarField* sectorGapSF = nullptr;

	//precision maps
	PrecisionMaps cloud1PM, cloud2PM;
	bool usePrecisionMaps = false;

	//progress notification
	CCCoreLib::NormalizedProgress* nProgress = nullptr;
	bool processCanceled = false;
	bool processFailed = false;

	// distance and uncertainty computation method
	qM3C2Tools::DistAndUncerMethod distAndUncerMethod;
};
static M3C2Params s_M3C2Params;

void Cout(QString str)
{
	std::cout << str.toLatin1().data() << std::endl;
}

void CoutPoint(Vector3Tpl<float> P)
{
	Cout("(" + QString::number(P.x) + ", " + QString::number(P.y) + ", " + QString::number(P.z) + ")");
}

double isSharp(double mean, double stdDev, double halfLength)
{
	bool res = false;

	if (std::abs(mean) + 2 * stdDev < static_cast<double>(halfLength))
		res = true;

	return res;
}

double meanMinusMedian(CCCoreLib::DgmOctree::NeighboursSet& set, double mean)
{
	std::sort(set.begin(), set.end(), CCCoreLib::DgmOctree::PointDescriptor::distComp);
	double median = qM3C2Tools::Median(set);

	return (mean - median);
}

CCVector3 computeNormal(CCCoreLib::DgmOctree::NeighboursSet neighbours, CCCoreLib::GenericIndexedCloudPersist *cloud)
{
	CCVector3 N(0, 0, 0);
	size_t n = neighbours.size();
	//if the widest neighborhood has less than 3 points in it, there's nothing we can do for this core point!
	if (n >= 3)
	{
		CCCoreLib::ReferenceCloud subset(cloud);

		// populate the subset
		for (unsigned j = 0; j < static_cast<unsigned>(n); ++j)
			subset.addPointIndex(neighbours[j].pointIndex);

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
			// double b = 2 * svalues[0] + 4 * svalues[1] - 2;
			// double c = 1 - a - b; // they sum to 1

			//the smallest eigen vector corresponds to the "least square best fitting plane" normal
			double vec[3];
			double minEigValue = 0;
			CCCoreLib::Jacobi<double>::GetMinEigenValueAndVector(eigVectors, eigValues, minEigValue, vec);

			N = CCVector3::fromArray(vec);
			N.normalize();
		}
	}
	return N;
}

double angle_rad(const CCVector3 &p, const CCVector3 &q, const CCVector3 &n)
{
	double productNorm = p.normd() * q.normd();
	if (productNorm < std::numeric_limits<double>::epsilon())
	{
		return std::numeric_limits<double>::quiet_NaN();
	}

	// atan2((p x q) . n, p . q)
	return atan2((p.cross(q)).dot(n), p.dot(q));
}

double computeSector(const CCCoreLib::DgmOctree::ProgressiveCylindricalNeighbourhood &cn, const unsigned &index)
{
	size_t count = cn.neighbours.size();
	std::vector<double> thetas(count);

	// compute one vector belonging to the plane perpendicular to the normal
	CCVector3 V = cn.dir.orthogonal(); // normalize is included in orthogonal!

	// compute the angle between AP and V
	// A the point in the neighbour set
	// P the projection of A on the axis of the cylinder
	// V a unit vector orthogonal to the cylinder axis
	CCVector3 PA; // the vector between P and A
	auto it = thetas.begin();
	for (auto A : cn.neighbours)
	{
		PA = *(A.point) - (cn.center + static_cast<PointCoordinateType>(A.squareDistd) * cn.dir);
		//        *it = PA.angle_rad(V) * 180. / M_PI;
		*it = angle_rad(V, PA, cn.dir) * 180. / M_PI;
		it++;
	}
	double maxTheta = *std::max_element(thetas.begin(), thetas.end());
	double minTheta = *std::min_element(thetas.begin(), thetas.end());

	// initialize the histogram
	unsigned numberOfClasses = 360;
	std::vector<unsigned> histo(numberOfClasses);
	std::fill(histo.begin(), histo.end(), 0);

	// build the histogram
	double step = 1.;
	for (auto theta : thetas)
	{
		unsigned bin = static_cast<unsigned>(floor((theta + 180.) * step));
		++histo[std::min(bin, numberOfClasses - 1)];
	}

	// count the ajacent bins equal to zero
	signed previousBin = -1;
	std::vector<unsigned> nbins;
	for (auto bin : histo)
	{
		if ((bin == 0) && (previousBin == 0))
		{
			nbins.back()++;
		}
		else if (bin == 0)
		{
			nbins.push_back(1);
		}
		previousBin = bin;
	}

	// merge the first and the last interval if needed
	bool merge = false;
	if ((histo.front() == 0) && (histo.back() == 0))
	{
		nbins.front() += nbins.back();
		merge = true;
	}

	// now that we have the size of the intervals, find the largest one
	unsigned maxNbBins = *std::max_element(nbins.begin(), nbins.end());

	double sector = maxNbBins * step;

	if (false)
	{
		QString msg = "index " + QString::number(index)
				+ " Npoints " + QString::number(count)
				+ " sector " + QString::number(sector)
				+ " Nintervals " + QString::number(nbins.size());

		msg = msg
				+ " (" + QString::number(minTheta)
				+ ", " + QString::number(maxTheta) + ")";

		if (merge)
			msg = msg
					+ " merge (" + QString::number(nbins.front())
					+ ", " + QString::number(nbins.back()) + ")";

		Cout(msg);
	}

	return (360. - sector); // the size of the sector containing the data
}

double computeGap(const CCCoreLib::DgmOctree::ProgressiveCylindricalNeighbourhood &cn, const unsigned &index)
{
	size_t count = cn.neighbours.size();

	double dmax = std::max_element(cn.neighbours.begin(), cn.neighbours.end(), [] (auto a, auto b) {return a.squareDistd < b.squareDistd;})->squareDistd;
	double dmin = std::min_element(cn.neighbours.begin(), cn.neighbours.end(), [] (auto a, auto b) {return a.squareDistd < b.squareDistd;})->squareDistd;
	double range = dmax - dmin;

	// initialize the histogram
	unsigned numberOfClasses = 100;
	std::vector<unsigned> histo(numberOfClasses);
	std::fill(histo.begin(), histo.end(), 0);

	// build the histogram
	double step = static_cast<double>(numberOfClasses) / (2. * cn.maxHalfLength);
	double shift = cn.maxHalfLength;;
	if (cn.onlyPositiveDir) // no shift needed, all distances are >=0
		shift = 0;

	for (auto A : cn.neighbours)
	{
		unsigned bin = static_cast<unsigned>(floor((A.squareDistd + shift) * step));
		++histo[std::min(bin, numberOfClasses - 1)];
	}

	// count the ajacent bins equal to zero
	signed previousBin = -1;
	std::vector<unsigned> nbins;
	for (auto bin : histo)
	{
		if ((previousBin == 0) && (bin == 0))
		{
			nbins.back()++;
		}
		else if (bin == 0)
		{
			nbins.push_back(1);
		}
		previousBin = bin;
	}

	// there is at least two intervals as we have points in the neighbourhood
	double gap = 0;
	if (nbins.size()>2)
	{
		// now that we have the size of the intervals, find the largest one, with some precautions
		auto elem1 = nbins.begin();
		auto elem2 = nbins.end();
		if (histo.front() == 0) // the first gap is not relevent except when there is a point in the very first bin of the histogram
			elem1++;
		if (histo.back() == 0) // the last gap is not relevent except when there is a point in the very last bin of the histogram
			elem2--;

		unsigned maxNbBins = *std::max_element(elem1, elem2);

		gap = maxNbBins / step;
	}

	if (false)
	{
		Cout("index " + QString::number(index)
			 + " n " + QString::number(count)
			 + " range " + QString::number(range)
			 + " gap " + QString::number(gap)
			 + " Nintervals " + QString::number(nbins.size()));
	}

	return gap;
}

void ComputeM3C2DistForPoint(unsigned index)
{
	if (s_M3C2Params.processCanceled)
		return;

	if (storeProjectionInfo)
	{
		s_M3C2Params.indexSF->setValue(index, index);
	}

	ScalarType dist = CCCoreLib::NAN_VALUE;

	//get core point #i
	CCVector3 P;
	s_M3C2Params.corePoints->getPoint(index, P);

	//get core point's normal #i
	CCVector3 N(0, 0, 1);
	CCVector3 N2(0, 0, 1);
	if (s_M3C2Params.updateNormal) //i.e. all cases but the VERTICAL mode
	{
		N = ccNormalVectors::GetNormal(s_M3C2Params.coreNormals->getValue(index));
	}

	//output point
	CCVector3 outputP = P;
	CCVector3 outputP2 = P;
	double sector1 {0};
	double gap1 {std::numeric_limits<float>::infinity()};
	double sector2 {0};
	double gap2 {std::numeric_limits<float>::infinity()};
	bool sectorGap {false};

	//compute M3C2 distance
	try
	{
		double mean1 = 0;
		double stdDev1 = 0;
		bool validStats1 = false;
		bool mean1IsSharp = false;
		bool mean2IsSharp = false;

		//extract cloud #1's neighbourhood
		CCCoreLib::DgmOctree::ProgressiveCylindricalNeighbourhood cn1;
		cn1.center = P;
		cn1.dir = N;
		cn1.level = s_M3C2Params.level1;
		cn1.maxHalfLength = s_M3C2Params.projectionDepth;
		cn1.radius = s_M3C2Params.projectionRadius;
		cn1.onlyPositiveDir = s_M3C2Params.onlyPositiveSearch;

		if (s_M3C2Params.progressiveSearch)
		{
			//progressive search
			size_t previousNeighbourCount = 0;
			while (cn1.currentHalfLength < cn1.maxHalfLength)
			{

				// use the default step for the search
				size_t neighbourCount = s_M3C2Params.cloud1Octree->getPointsInCylindricalNeighbourhoodProgressive(cn1);
				if (neighbourCount != previousNeighbourCount)
				{
					//do we have enough points for computing stats?
					if (neighbourCount >= s_M3C2Params.minPoints4Stats)
					{
						qM3C2Tools::ComputeStatistics(cn1.neighbours, s_M3C2Params.distAndUncerMethod, mean1, stdDev1);
						validStats1 = true;
						//do we have a sharp enough 'mean' to stop?
						bool sharp = std::abs(mean1) + 2 * stdDev1 < static_cast<double>(cn1.currentHalfLength);
						if (sharp)
							break;
					}
				}
				previousNeighbourCount = neighbourCount;
			}
		}
		else
		{
			s_M3C2Params.cloud1Octree->getPointsInCylindricalNeighbourhood(cn1);
		}

		if (exportSearchDepth)
		{
			s_M3C2Params.searchDepth1SF->setValue(index, cn1.currentHalfLength);
			s_M3C2Params.meanMinusMed1SF->setValue(index, meanMinusMedian(cn1.neighbours, mean1));
			sector1 = computeSector(cn1, index);
			s_M3C2Params.sector1SF->setValue(index, sector1);
			gap1 = computeGap(cn1, index);
			s_M3C2Params.gap1SF->setValue(index, gap1);
		}
		size_t n1 = cn1.neighbours.size();
		if (n1 != 0)
		{
			//compute stat. dispersion on cloud #1 neighbours (if necessary)
			if (!validStats1)
			{
				qM3C2Tools::ComputeStatistics(cn1.neighbours, s_M3C2Params.distAndUncerMethod, mean1, stdDev1);
			}

			if (s_M3C2Params.usePrecisionMaps && (s_M3C2Params.computeConfidence || s_M3C2Params.stdDevCloud1SF))
			{
				//compute the Precision Maps derived sigma
				stdDev1 = ComputePMUncertainty(cn1.neighbours, N, s_M3C2Params.cloud1PM);
			}

			if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD1
					|| s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD1_AND_CLOUD2)
			{
				//shift output point on the 1st cloud
				outputP += static_cast<PointCoordinateType>(mean1) * N;
			}

			//save cloud #1's std. dev.
			if (s_M3C2Params.stdDevCloud1SF)
			{
				ScalarType val = static_cast<ScalarType>(stdDev1);
				s_M3C2Params.stdDevCloud1SF->setValue(index, val);
			}

			if (storeProjectionInfo)
			{
				unsigned currentSize = projectionCloud->size();
				projectionCloud->reserve(currentSize + 2 * n1 + 1);
				projCloud_SF_index->resizeSafe(currentSize + 2 * n1 + 1, true, CCCoreLib::NAN_VALUE);
				projCloud_SF_cloud->resizeSafe(currentSize + 2 * n1 + 1, true, CCCoreLib::NAN_VALUE);
				for(unsigned int k = 0; k < n1; k++)
				{
					projectionCloud->addPoint(*cn1.neighbours[k].point);
					projCloud_SF_index->setValue(currentSize + 2 * k, index);
					projCloud_SF_cloud->setValue(currentSize + 2 * k, 1);

					projectionCloud->addPoint(cn1.center + static_cast<PointCoordinateType>(cn1.neighbours[k].squareDistd) * cn1.dir);
					projCloud_SF_index->setValue(currentSize + 2 * k + 1, index);
					projCloud_SF_cloud->setValue(currentSize + 2 * k + 1, 1);
				}
				// add the core point to the cloud, with cloud number 0
				projectionCloud->addPoint(P);
				projCloud_SF_index->setValue(currentSize + 2 * n1, index);
				projCloud_SF_cloud->setValue(currentSize + 2 * n1, 0);
			}
		}

		//save cloud #1's density
		if (s_M3C2Params.densityCloud1SF)
		{
			ScalarType val = static_cast<ScalarType>(n1);
			s_M3C2Params.densityCloud1SF->setValue(index, val);
		}

		//now we can process cloud #2
		if (	n1 != 0
				||	s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD2
				||  s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD2_WITH_NORM2
			||	s_M3C2Params.stdDevCloud2SF
			||	s_M3C2Params.densityCloud2SF
			)
		{
			double mean2 = 0;
			double stdDev2 = 0;
			bool validStats2 = false;
			
			//extract cloud #2's neighbourhood
			CCCoreLib::DgmOctree::ProgressiveCylindricalNeighbourhood cn2;
			cn2.center = P;
			cn2.dir = N;
			cn2.level = s_M3C2Params.level2;
			cn2.maxHalfLength = s_M3C2Params.projectionDepth;
			cn2.radius = s_M3C2Params.projectionRadius;
			cn2.onlyPositiveDir = s_M3C2Params.onlyPositiveSearch;

			if (s_M3C2Params.progressiveSearch)
			{
				//progressive search
				size_t previousNeighbourCount = 0;
				while (cn2.currentHalfLength < cn2.maxHalfLength)
				{


					// use the default step for the search
					size_t neighbourCount = s_M3C2Params.cloud2Octree->getPointsInCylindricalNeighbourhoodProgressive(cn2);
					if (neighbourCount != previousNeighbourCount)
					{
						//do we have enough points for computing stats?
						if (neighbourCount >= s_M3C2Params.minPoints4Stats)
						{
							qM3C2Tools::ComputeStatistics(cn2.neighbours, s_M3C2Params.distAndUncerMethod, mean2, stdDev2);
							validStats2 = true;
							//do we have a sharp enough 'mean' to stop?
							bool sharp = std::abs(mean2) + 2 * stdDev2 < static_cast<double>(cn2.currentHalfLength);
							if (sharp)
								break;
						}
					}
					previousNeighbourCount = neighbourCount;
				}
			}
			else
			{
				s_M3C2Params.cloud2Octree->getPointsInCylindricalNeighbourhood(cn2);
			}

			if (exportSearchDepth)
			{
				s_M3C2Params.searchDepth2SF->setValue(index, cn2.currentHalfLength);
				s_M3C2Params.meanMinusMed2SF->setValue(index, meanMinusMedian(cn2.neighbours, mean2));
				// compute the angle between the normal of the neighbour set and the current normal
				CCVector3 N2 = computeNormal(cn2.neighbours, s_M3C2Params.cloud2Octree->associatedCloud());
				// the case N2 = (0, 0, 0) is handle by angle_rad
				auto angle = acos(N2.angle_rad(cn2.dir)) * 180 / M_PI;
				s_M3C2Params.normalsAngleSF->setValue(index, angle);
				sector2 = computeSector(cn2, index);
				s_M3C2Params.sector2SF->setValue(index, sector2);
				gap2 = computeGap(cn2, index);
				s_M3C2Params.gap2SF->setValue(index, gap2);
				sectorGap = (sector1 > 180) && (gap1 < s_M3C2Params.projectionRadius)
						&& (sector2 > 180) && (gap2 < s_M3C2Params.projectionRadius);
				if (sectorGap)
					s_M3C2Params.sectorGapSF->setValue(index, true);
			}
			size_t n2 = cn2.neighbours.size();
			if (n2 != 0)
			{
				//compute stat. dispersion on cloud #2 neighbours (if necessary)
				if (!validStats2)
				{
					qM3C2Tools::ComputeStatistics(cn2.neighbours, s_M3C2Params.distAndUncerMethod, mean2, stdDev2);
				}
				assert(stdDev2 != stdDev2 || stdDev2 >= 0); //first inequality fails if stdDev2 is NaN ;)

				if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD2 || s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD2_WITH_NORM2)
					//shift output point on the 2nd cloud
					outputP += static_cast<PointCoordinateType>(mean2) * N;

				else if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD1_AND_CLOUD2)
					//shift output point on the 2nd cloud
					outputP2 += static_cast<PointCoordinateType>(mean2) * N;

				if (s_M3C2Params.usePrecisionMaps && (s_M3C2Params.computeConfidence || s_M3C2Params.stdDevCloud2SF))
				{
					//compute the Precision Maps derived sigma
					stdDev2 = ComputePMUncertainty(cn2.neighbours, N, s_M3C2Params.cloud2PM);
				}

				if (n1 != 0)
				{
					//m3c2 dist = distance between i1 and i2 (i.e. either the mean or the median of both neighborhoods)
					dist = static_cast<ScalarType>(mean2 - mean1);
					s_M3C2Params.m3c2DistSF->setValue(index, dist);

					//confidence interval
					if (s_M3C2Params.computeConfidence)
					{
						ScalarType LODStdDev = CCCoreLib::NAN_VALUE;
						if (s_M3C2Params.usePrecisionMaps)
						{
							LODStdDev = stdDev1*stdDev1 + stdDev2*stdDev2; //equation (2) in M3C2-PM article
						}
						//standard M3C2 algortihm: have we enough points for computing the confidence interval?
						else if (n1 >= s_M3C2Params.minPoints4Stats && n2 >= s_M3C2Params.minPoints4Stats)
						{
							LODStdDev = (stdDev1*stdDev1) / n1 + (stdDev2*stdDev2) / n2;
						}

						if (!std::isnan(LODStdDev))
						{
							//distance uncertainty (see eq. (1) in M3C2 article)
							ScalarType LOD = static_cast<ScalarType>(1.96 * (sqrt(LODStdDev) + s_M3C2Params.registrationRms));

							if (s_M3C2Params.distUncertaintySF)
							{
								s_M3C2Params.distUncertaintySF->setValue(index, LOD);
							}

							// compute values for the sharp means criteria
							mean1IsSharp = isSharp(mean1, stdDev1, cn1.currentHalfLength);
							mean2IsSharp = isSharp(mean2, stdDev2, cn2.currentHalfLength);

							if (s_M3C2Params.sigChangeSF)
							{
								bool significant = (dist < -LOD || dist > LOD);
								if (significant)
								{
									if (sharpMean)
									{
										if (mean1IsSharp && mean2IsSharp) // check that both means are sharp
											s_M3C2Params.sigChangeSF->setValue(index, SCALAR_ONE); //already equal to SCALAR_ZERO otherwise
									}
									else
										s_M3C2Params.sigChangeSF->setValue(index, SCALAR_ONE); //already equal to SCALAR_ZERO otherwise
								}
							}

							if (computeWelch)
							{
								// A Students t test applied to two sets of data. We are testing the null hypothesis that the two
								// samples have the same mean and that any difference if due to chance.
								// Welch's t-test
								// t-statistic
								// t = (mu_1 - mu_2) / sqrt(std1^2 / n1 + std2^2 / n2)
								double s1 = stdDev1 * sqrt(n1 / (n1 - 1.)); //  one needs a non biased estimator in Welch formula
								double s2 = stdDev2 * sqrt(n2 / (n2 - 1.)); //  one needs a non biased estimator in Welch formula
								//double s1 = stdDev1; //  one needs a non biased estimator in Welch formula
								//double s2 = stdDev2; //  one needs a non biased estimator in Welch formula
								double t_num = sqrt(pow(s1, 2) / n1 + pow(s2, 2) / n2);
								double t = (mean1 - mean2) / t_num;
								s_M3C2Params.welch_t_SF->setValue(index, t);
								// degrees of freedom (possible to round down to the next lowest integer)
								//          (std1^2 / N1 + std2^2 / N2)^2
								// v = ---------------------------------------
								//     std1^4 / (N1^2 v1) + std2^4 / (N2^2 v2)
								// v1 = N1 - 1 degrees of freedom associated with the iith variance estimate
								// v2 = N2 - 1
								double v1 = n1 - 1;
								double v2 = n2 - 1;
								if (n1 != 1 && n2 != 1)
								{
									double v = pow((pow(s1, 2) / n1 + pow(s2, 2) / n2), 2)
											/ (pow(s1, 4) / (pow(n1, 2) * v1) + pow(s2, 4) / (pow(n2, 2) * v2));
									boost::math::students_t t_dist(v);
									// cdf(complement()) <=> survival function
									double q = 2 * boost::math::cdf(boost::math::complement(t_dist, fabs(t)));
									double welch_p = boost::math::quantile(t_dist, 0.975);
									double welch_lod = welch_p * (t_num + s_M3C2Params.registrationRms);
									s_M3C2Params.welch_v_SF->setValue(index, v);
									s_M3C2Params.welch_q_SF->setValue(index, q);
									s_M3C2Params.welch_p_SF->setValue(index, welch_p);
									s_M3C2Params.welch_lod_SF->setValue(index, welch_lod);
									bool welch_sig = (fabs(dist) > welch_lod);
									if (welch_sig)
									{
										if (sharpMean)
										{
											if (mean1IsSharp && mean2IsSharp) // check that both means are sharp
												s_M3C2Params.welch_sig_SF->setValue(index, SCALAR_ONE); //already equal to SCALAR_ZERO otherwise
										}
										else
											s_M3C2Params.welch_sig_SF->setValue(index, SCALAR_ONE); //already equal to SCALAR_ZERO otherwise
									}
								}
							}
						}
						//else //DGM: scalar fields have already been initialized with the right 'default' values
						//{
						//	if (distUncertaintySF)
						//		distUncertaintySF->setValue(index, CCCoreLib::NAN_VALUE);
						//	if (sigChangeSF)
						//		sigChangeSF->setValue(index, SCALAR_ZERO);
						//}
					}
				}

				//save cloud #2's std. dev.
				if (s_M3C2Params.stdDevCloud2SF)
				{
					ScalarType val = static_cast<ScalarType>(stdDev2);
					s_M3C2Params.stdDevCloud2SF->setValue(index, val);
				}

				if (storeProjectionInfo)
				{
					unsigned currentSize = projectionCloud->size();
					projectionCloud->reserve(currentSize + 2 * n2);
					projCloud_SF_index->resizeSafe(currentSize + 2 * n2, true, CCCoreLib::NAN_VALUE);
					projCloud_SF_cloud->resizeSafe(currentSize + 2 * n2, true, CCCoreLib::NAN_VALUE);
					for(unsigned int k = 0; k < n2; k++)
					{
						projectionCloud->addPoint(*cn2.neighbours[k].point);
						projCloud_SF_index->setValue(currentSize + 2 * k, index);
						projCloud_SF_cloud->setValue(currentSize + 2 * k, 2);

						projectionCloud->addPoint(cn2.center + static_cast<PointCoordinateType>(cn2.neighbours[k].squareDistd) * cn2.dir);
						projCloud_SF_index->setValue(currentSize + 2 * k + 1, index);
						projCloud_SF_cloud->setValue(currentSize + 2 * k + 1, 2);
					}
				}
			}

			//save cloud #2's density
			if (s_M3C2Params.densityCloud2SF)
			{
				ScalarType val = static_cast<ScalarType>(n2);
				s_M3C2Params.densityCloud2SF->setValue(index, val);
			}
		}
	}
	catch (std::bad_alloc&)
	{
		//Not enough memory
		s_M3C2Params.processFailed = true;
		return;
	}

	//output point
	if (s_M3C2Params.outputCloud != s_M3C2Params.corePoints)
	{
		*const_cast<CCVector3*>(s_M3C2Params.outputCloud->getPoint(index)) = outputP;
        if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD1_AND_CLOUD2)
            *const_cast<CCVector3*>(s_M3C2Params.outputCloud2->getPoint(index)) = outputP2;
	}
    if (s_M3C2Params.exportNormal)
        s_M3C2Params.outputCloud->setPointNormal(index, N);

    // if requested, the normals are replaced by the normals computed using cloud 2 as the base cloud
    if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD2_WITH_NORM2)
    {
        N2 = ccNormalVectors::GetNormal(s_M3C2Params.coreNormals2->getValue(index));
        s_M3C2Params.outputCloud->setPointNormal(index, N2);
    }

	//progress notification
	if (s_M3C2Params.nProgress && !s_M3C2Params.nProgress->oneStep())
	{
		s_M3C2Params.processCanceled = true;
	}
}

bool qM3C2Process::Compute(const qM3C2Dialog& dlg, QString& errorMessage, ccPointCloud*& outputCloud, ccPointCloud *&outputCloud2/*=nullptr*/, bool allowDialogs, QWidget* parentWidget/*=nullptr*/, ccMainAppInterface* app/*=nullptr*/)
{
	errorMessage.clear();
	outputCloud = nullptr;
    outputCloud2 = nullptr;

	//get the clouds in the right order
	ccPointCloud* cloud1 = dlg.getCloud1();
	ccPointCloud* cloud2 = dlg.getCloud2();

	if (!cloud1 || !cloud2)
	{
		assert(false);
		return false;
	}

	//normals computation parameters
	double normalScale = dlg.normalScaleDoubleSpinBox->value();
	double projectionScale = dlg.cylDiameterDoubleSpinBox->value();
	qM3C2Normals::ComputationMode normMode = dlg.getNormalsComputationMode();
	double samplingDist = dlg.cpSubsamplingDoubleSpinBox->value();
	ccScalarField* normalScaleSF = nullptr; //normal scale (multi-scale mode only)
    ccScalarField* normalScaleSF2 = nullptr; //normal scale (multi-scale mode only)

	//other parameters are stored in 's_M3C2Params' for parallel call
	s_M3C2Params = M3C2Params();
	s_M3C2Params.projectionRadius = static_cast<PointCoordinateType>(projectionScale / 2); //we want the radius in fact ;)
	s_M3C2Params.projectionDepth = static_cast<PointCoordinateType>(dlg.cylHalfHeightDoubleSpinBox->value());
	s_M3C2Params.corePoints = dlg.getCorePointsCloud();
	s_M3C2Params.registrationRms = dlg.rmsCheckBox->isChecked() ? dlg.rmsDoubleSpinBox->value() : 0.0;
	s_M3C2Params.exportOption = dlg.getExportOption();
	s_M3C2Params.keepOriginalCloud = dlg.keepOriginalCloud();
    s_M3C2Params.distAndUncerMethod = dlg.getDistAndUncerMethod();
	s_M3C2Params.minPoints4Stats = dlg.getMinPointsForStats();
	s_M3C2Params.progressiveSearch = !dlg.useSinglePass4DepthCheckBox->isChecked();
	s_M3C2Params.onlyPositiveSearch = dlg.positiveSearchOnlyCheckBox->isChecked();

    // MULTI-INTERCEPTION
    storeProjectionInfo = dlg.getProjectionDetails();
    if (storeProjectionInfo)
    {
        projectionCloud = new ccPointCloud();
        projCloud_sfIdx_index = projectionCloud->addScalarField("index");
        projCloud_sfIdx_cloud = projectionCloud->addScalarField("cloud");
        projCloud_SF_index = projectionCloud->getScalarField(projCloud_sfIdx_index);
        projCloud_SF_cloud = projectionCloud->getScalarField(projCloud_sfIdx_cloud);
        projectionCloud->setName("projectionCloud");
        // add the index to the output cloud
        s_M3C2Params.indexSF = new ccScalarField("index");
        s_M3C2Params.indexSF->link(); //will be released anyway at the end of the process
    }
    // EXPORT SEARCH DEPTH
    exportSearchDepth = dlg.exportSearchDepth();
    if (exportSearchDepth)
    {
        s_M3C2Params.searchDepth1SF = new ccScalarField(SEARCH_DEPTH1_SF_NAME);
        s_M3C2Params.searchDepth1SF->link(); //will be released anyway at the end of the process
        s_M3C2Params.searchDepth2SF = new ccScalarField(SEARCH_DEPTH2_SF_NAME);
        s_M3C2Params.searchDepth2SF->link(); //will be released anyway at the end of the process
        s_M3C2Params.meanMinusMed1SF = new ccScalarField(MEAN_MINUS_MEDIAN_1_SF);
        s_M3C2Params.meanMinusMed1SF->link(); //will be released anyway at the end of the process
        s_M3C2Params.meanMinusMed2SF = new ccScalarField(MEAN_MINUS_MEDIAN_2_SF);
        s_M3C2Params.meanMinusMed2SF->link(); //will be released anyway at the end of the process
        s_M3C2Params.normalsAngleSF = new ccScalarField(NORMALS_ANGLE_SF);
        s_M3C2Params.normalsAngleSF->link(); //will be released anyway at the end of the process
        s_M3C2Params.sector1SF = new ccScalarField(SECTOR_SF);
        s_M3C2Params.sector1SF->link(); //will be released anyway at the end of the process
        s_M3C2Params.gap1SF = new ccScalarField(GAP_SF);
        s_M3C2Params.gap1SF->link(); //will be released anyway at the end of the process
        s_M3C2Params.sector2SF = new ccScalarField(SECTOR2_SF);
        s_M3C2Params.sector2SF->link(); //will be released anyway at the end of the process
        s_M3C2Params.gap2SF = new ccScalarField(GAP2_SF);
        s_M3C2Params.gap2SF->link(); //will be released anyway at the end of the process
        s_M3C2Params.sectorGapSF = new ccScalarField(SECTOR_GAP_SF);
        s_M3C2Params.sectorGapSF->link(); //will be released anyway at the end of the process
    }
    // COMPUTE WELCH
    computeWelch = dlg.computeWelch();
    if (computeWelch)
    {
        s_M3C2Params.welch_t_SF = new ccScalarField(WELCH_T_SF_NAME);
        s_M3C2Params.welch_t_SF->link(); //will be released anyway at the end of the process

        s_M3C2Params.welch_v_SF = new ccScalarField(WELCH_V_SF_NAME);
        s_M3C2Params.welch_v_SF->link(); //will be released anyway at the end of the process

        s_M3C2Params.welch_q_SF = new ccScalarField(WELCH_Q_SF_NAME);
        s_M3C2Params.welch_q_SF->link(); //will be released anyway at the end of the process

        s_M3C2Params.welch_p_SF = new ccScalarField(WELCH_P_SF_NAME);
        s_M3C2Params.welch_p_SF->link(); //will be released anyway at the end of the process

        s_M3C2Params.welch_lod_SF = new ccScalarField(WELCH_LOD_SF_NAME);
        s_M3C2Params.welch_lod_SF->link(); //will be released anyway at the end of the process

        s_M3C2Params.welch_sig_SF = new ccScalarField(WELCH_SIG_SF_NAME);
        s_M3C2Params.welch_sig_SF->link(); //will be released anyway at the end of the process
    }
    // SHARP MEAN
    sharpMean = dlg.getSharpMean(); // use sharp mean diagnostic to reject projected points

    //ccLog::Print("s_M3C2Params.distAndUncerMethod %d", s_M3C2Params.distAndUncerMethod);

    // PLE to allow the usage of core points with normals using the command line interface
    if (!app)
    {
        int requestedNormMode = dlg.getRequestedNormMode(); // the normMode in the parameters file
        if ((requestedNormMode == qM3C2Normals::USE_CORE_POINTS_NORMALS) && s_M3C2Params.corePoints->hasNormals())
        {
            //ccLog::Warning("core points have normals, normMode has been changed from %d (auto) to %d (requested)", normMode, requestedNormMode);
            normMode = qM3C2Normals::USE_CORE_POINTS_NORMALS;
        }
    }

	//precision maps
	{
		s_M3C2Params.usePrecisionMaps = dlg.precisionMapsGroupBox->isEnabled() && dlg.precisionMapsGroupBox->isChecked();
		if (s_M3C2Params.usePrecisionMaps)
		{
			if (allowDialogs && QMessageBox::question(parentWidget, "Precision Maps", "Are you sure you want to compute the M3C2 distances with precision maps?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
			{
				s_M3C2Params.usePrecisionMaps = false;
				dlg.precisionMapsGroupBox->setChecked(false);
			}
		}
		if (s_M3C2Params.usePrecisionMaps)
		{
			s_M3C2Params.cloud1PM.sX = cloud1->getScalarField(dlg.c1SxComboBox->currentIndex());
			s_M3C2Params.cloud1PM.sY = cloud1->getScalarField(dlg.c1SyComboBox->currentIndex());
			s_M3C2Params.cloud1PM.sZ = cloud1->getScalarField(dlg.c1SzComboBox->currentIndex());
			s_M3C2Params.cloud1PM.scale = dlg.pm1ScaleDoubleSpinBox->value();

			s_M3C2Params.cloud2PM.sX = cloud2->getScalarField(dlg.c2SxComboBox->currentIndex());
			s_M3C2Params.cloud2PM.sY = cloud2->getScalarField(dlg.c2SyComboBox->currentIndex());
			s_M3C2Params.cloud2PM.sZ = cloud2->getScalarField(dlg.c2SzComboBox->currentIndex());
			s_M3C2Params.cloud2PM.scale = dlg.pm2ScaleDoubleSpinBox->value();

			if (!s_M3C2Params.cloud1PM.valid() || !s_M3C2Params.cloud2PM.valid())
			{
				errorMessage = "Invalid 'Precision maps' settings!";
				return false;
			}
		}
	}

	//max thread count
	int maxThreadCount = dlg.getMaxThreadCount();

	if (app)
		app->dispToConsole(QString("[M3C2] Will use %1 threads").arg(maxThreadCount == 0 ? "the max number of" : QString::number(maxThreadCount)), ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//progress dialog
	ccProgressDialog pDlg(parentWidget);

	//Duration: initialization & normals computation
	QElapsedTimer initTimer;
	initTimer.start();

	//compute octree(s) if necessary
	s_M3C2Params.cloud1Octree = cloud1->getOctree();
	if (!s_M3C2Params.cloud1Octree)
	{
		s_M3C2Params.cloud1Octree = cloud1->computeOctree(&pDlg);
		if (s_M3C2Params.cloud1Octree && cloud1->getParent() && app)
		{
			app->addToDB(cloud1->getOctreeProxy());
		}
	}
	if (!s_M3C2Params.cloud1Octree)
	{
		errorMessage = "Failed to compute cloud #1's octree!";
		return false;
	}

	s_M3C2Params.cloud2Octree = cloud2->getOctree();
	if (!s_M3C2Params.cloud2Octree)
	{
		s_M3C2Params.cloud2Octree = cloud2->computeOctree(&pDlg);
		if (s_M3C2Params.cloud2Octree && cloud2->getParent() && app)
		{
			app->addToDB(cloud2->getOctreeProxy());
		}
	}
	if (!s_M3C2Params.cloud2Octree)
	{
		errorMessage = "Failed to compute cloud #2's octree!";
		return false;
	}

	//start the job
	bool error = false;

	//should we generate the core points?
	bool corePointsHaveBeenSubsampled = false;
	if (!s_M3C2Params.corePoints && samplingDist > 0)
	{
		CCCoreLib::CloudSamplingTools::SFModulationParams modParams(false);
		CCCoreLib::ReferenceCloud* subsampled = CCCoreLib::CloudSamplingTools::resampleCloudSpatially(cloud1,
			static_cast<PointCoordinateType>(samplingDist),
			modParams,
			s_M3C2Params.cloud1Octree.data(),
			&pDlg);

		if (subsampled)
		{
			s_M3C2Params.corePoints = static_cast<ccPointCloud*>(cloud1)->partialClone(subsampled);

			//don't need those references anymore
			delete subsampled;
			subsampled = nullptr;
		}

		if (s_M3C2Params.corePoints)
		{
			s_M3C2Params.corePoints->setName(QString("%1.subsampled [min dist. = %2]").arg(cloud1->getName()).arg(samplingDist));
			s_M3C2Params.corePoints->setVisible(true);
			s_M3C2Params.corePoints->setDisplay(cloud1->getDisplay());
			if (app)
			{
				app->dispToConsole(QString("[M3C2] Sub-sampled cloud has been saved ('%1')").arg(s_M3C2Params.corePoints->getName()), ccMainAppInterface::STD_CONSOLE_MESSAGE);
				app->addToDB(s_M3C2Params.corePoints);
			}
			corePointsHaveBeenSubsampled = true;
		}
		else
		{
			errorMessage = "Failed to compute sub-sampled core points!";
			error = true;
		}
	}

	//output
	QString outputName(s_M3C2Params.usePrecisionMaps ? "M3C2-PM output" : "M3C2 output");
    QString outputName2(s_M3C2Params.usePrecisionMaps ? "M3C2-PM_2 output" : "M3C2_2 output");
    switch (s_M3C2Params.distAndUncerMethod)
    {
    case qM3C2Tools::USE_MEAN_AND_STD_DEV:
        outputName += " [mean+std]";
        outputName2 += " [mean+std]";
        break;
    case qM3C2Tools::USE_MEDIAN_AND_IQR:
        outputName += " [med+iqr]";
        outputName2 += " [med+iqr]";
        break;
    case qM3C2Tools::USE_MIN_AND_MAX_MINUS_MIN:
        outputName += " [min+max]";
        outputName2 += " [min+max]";
        break;
    case qM3C2Tools::USE_PERCENTILES:
        outputName += " [prctile5+95]";
        outputName2 += " [prctile5+95]";
        break;
    }

	if (!error)
	{
		//whatever the case, at this point we should have core points
		assert(s_M3C2Params.corePoints);
		if (app)
			app->dispToConsole(QString("[M3C2] Core points: %1").arg(s_M3C2Params.corePoints->size()), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		if (s_M3C2Params.keepOriginalCloud)
		{
			s_M3C2Params.outputCloud = s_M3C2Params.corePoints;
		}
		else
		{
			s_M3C2Params.outputCloud = new ccPointCloud(/*outputName*/); //setName will be called at the end
			if (!s_M3C2Params.outputCloud->resize(s_M3C2Params.corePoints->size())) //resize as we will 'set' the new points positions in 'ComputeM3C2DistForPoint'
			{
				errorMessage = "Not enough memory!";
				error = true;
			}
            if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD1_AND_CLOUD2)
            {
                s_M3C2Params.outputCloud2 = new ccPointCloud(/*outputName*/); //setName will be called at the end
                if (!s_M3C2Params.outputCloud2->resize(s_M3C2Params.corePoints->size())) //resize as we will 'set' the new points positions in 'ComputeM3C2DistForPoint'
                {
                    errorMessage = "Not enough memory!";
                    error = true;
                }
            }
			s_M3C2Params.corePoints->setEnabled(false); //we can hide the core points
		}
	}

	//compute normals
	if (!error)
	{
		bool normalsAreOk = false;
		bool useCorePointsOnly = dlg.normUseCorePointsCheckBox->isChecked();

		switch (normMode)
		{
		case qM3C2Normals::HORIZ_MODE:
			outputName += QString(" [HORIZONTAL]");
		case qM3C2Normals::DEFAULT_MODE:
		case qM3C2Normals::MULTI_SCALE_MODE:
		{
			s_M3C2Params.coreNormals = new NormsIndexesTableType();
			s_M3C2Params.coreNormals->link(); //will be released anyway at the end of the process

			std::vector<PointCoordinateType> radii;
			if (normMode == qM3C2Normals::MULTI_SCALE_MODE)
			{
				//get multi-scale parameters
				double startScale = dlg.minScaleDoubleSpinBox->value();
				double step = dlg.stepScaleDoubleSpinBox->value();
				double stopScale = dlg.maxScaleDoubleSpinBox->value();
				stopScale = std::max(startScale, stopScale); //just to be sure
				//generate all corresponding 'scales'
				for (double scale = startScale; scale <= stopScale; scale += step)
				{
					radii.push_back(static_cast<PointCoordinateType>(scale / 2));
				}

				outputName += QString(" scale=[%1:%2:%3]").arg(startScale).arg(step).arg(stopScale);
                outputName2 += QString(" scale=%1").arg(normalScale);

				normalScaleSF = new ccScalarField(NORMAL_SCALE_SF_NAME);
				normalScaleSF->link(); //will be released anyway at the end of the process
			}
			else
			{
				outputName += QString(" scale=%1").arg(normalScale);
                outputName2 += QString(" scale=%1").arg(normalScale);
				//otherwise, we use a unique scale by default
				radii.push_back(static_cast<PointCoordinateType>(normalScale / 2)); //we want the radius in fact ;)
			}

			bool invalidNormals = false;
			ccPointCloud* baseCloud = (useCorePointsOnly ? s_M3C2Params.corePoints : cloud1);
			ccOctree* baseOctree = (baseCloud == cloud1 ? s_M3C2Params.cloud1Octree.data() : nullptr);

			//dedicated core points method
			normalsAreOk = qM3C2Normals::ComputeCorePointsNormals(s_M3C2Params.corePoints,
				s_M3C2Params.coreNormals,
				baseCloud,
				radii,
				invalidNormals,
				maxThreadCount,
				normalScaleSF,
				&pDlg,
				baseOctree);

			//now fix the orientation
			if (normalsAreOk)
			{
				//some invalid normals?
				if (invalidNormals && app)
				{
					app->dispToConsole("[M3C2] Some normals are invalid! You may have to increase the scale.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				}

				//make normals horizontal if necessary
				if (normMode == qM3C2Normals::HORIZ_MODE)
				{
					qM3C2Normals::MakeNormalsHorizontal(*s_M3C2Params.coreNormals);
				}

				//then either use a simple heuristic
				bool usePreferredOrientation = dlg.normOriPreferredRadioButton->isChecked();
				if (usePreferredOrientation)
				{
					int preferredOrientation = dlg.normOriPreferredComboBox->currentIndex();
					assert(preferredOrientation >= ccNormalVectors::PLUS_X && preferredOrientation <= ccNormalVectors::MINUS_SENSOR_ORIGIN);
					if (!ccNormalVectors::UpdateNormalOrientations(	s_M3C2Params.corePoints,
																	*s_M3C2Params.coreNormals,
																	static_cast<ccNormalVectors::Orientation>(preferredOrientation))
						)
					{
						errorMessage = "[M3C2] Failed to re-orient the normals (invalid parameter?)";
						error = true;
					}
				}
				else //or use external points
				{
					ccPointCloud* orientationCloud = dlg.getNormalsOrientationCloud();
					assert(orientationCloud);

					if (!qM3C2Normals::UpdateNormalOrientationsWithCloud(	s_M3C2Params.corePoints,
																			*s_M3C2Params.coreNormals,
																			orientationCloud,
																			maxThreadCount,
																			&pDlg)
						)
					{
						errorMessage = "[M3C2] Failed to re-orient the normals with input point cloud!";
						error = true;
					}
				}

				if (!error && s_M3C2Params.coreNormals)
				{
					s_M3C2Params.outputCloud->setNormsTable(s_M3C2Params.coreNormals);
					s_M3C2Params.outputCloud->showNormals(true);
				}
			}
		}
		break;

		case qM3C2Normals::USE_CLOUD1_NORMALS:
		{
			outputName += QString(" scale=%1").arg(normalScale);
			ccPointCloud* sourceCloud = (corePointsHaveBeenSubsampled ? s_M3C2Params.corePoints : cloud1);
			s_M3C2Params.coreNormals = sourceCloud->normals();
			normalsAreOk = (s_M3C2Params.coreNormals && s_M3C2Params.coreNormals->currentSize() == sourceCloud->size());
			s_M3C2Params.coreNormals->link(); //will be released anyway at the end of the process

			//DGM TODO: should we export the normals to the output cloud?
		}
		break;

		case qM3C2Normals::USE_CORE_POINTS_NORMALS:
		{
			normalsAreOk = s_M3C2Params.corePoints && s_M3C2Params.corePoints->hasNormals();
			if (normalsAreOk)
            {
				s_M3C2Params.coreNormals = s_M3C2Params.corePoints->normals();
				s_M3C2Params.coreNormals->link(); //will be released anyway at the end of the process
			}
		}
		break;

		case qM3C2Normals::VERT_MODE:
		{
			outputName += QString(" scale=%1").arg(normalScale);
			outputName += QString(" [VERTICAL]");

			//nothing to do
			normalsAreOk = true;
		}
		break;
		}

		if (!normalsAreOk)
		{
			errorMessage = "Failed to compute normals!";
			error = true;
		}
	}

	// PLE compute the normals of cloud 2
	if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD2_WITH_NORM2)
	{
		if (app)
			app->dispToConsole("[M3C2] compute normals using cloud 2 as base", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		else
			ccLog::Print("[M3C2] compute normals using cloud 2 as base");

		bool normalsAreOk2 = false;

		s_M3C2Params.coreNormals2 = new NormsIndexesTableType();
		s_M3C2Params.coreNormals2->link(); //will be released anyway at the end of the process

		std::vector<PointCoordinateType> radii;
		if (normMode == qM3C2Normals::MULTI_SCALE_MODE)
		{
			//get multi-scale parameters
			double startScale = dlg.minScaleDoubleSpinBox->value();
			double step = dlg.stepScaleDoubleSpinBox->value();
			double stopScale = dlg.maxScaleDoubleSpinBox->value();
			stopScale = std::max(startScale, stopScale); //just to be sure
			//generate all corresponding 'scales'
			for (double scale = startScale; scale <= stopScale; scale += step)
			{
				radii.push_back(static_cast<PointCoordinateType>(scale / 2));
			}

			normalScaleSF2 = new ccScalarField(NORMAL_SCALE_SF_NAME);
			normalScaleSF2->link(); //will be released anyway at the end of the process
		}
		else
		{
			//otherwise, we use a unique scale by default
			radii.push_back(static_cast<PointCoordinateType>(normalScale / 2)); //we want the radius in fact ;)
		}

		bool invalidNormals2 = false;
		ccPointCloud* baseCloud2 = cloud2;
		ccOctree* baseOctree2 = (baseCloud2 == cloud2 ? s_M3C2Params.cloud2Octree.data() : nullptr);

		//dedicated core points method
		normalsAreOk2 = qM3C2Normals::ComputeCorePointsNormals(s_M3C2Params.corePoints,
															   s_M3C2Params.coreNormals2,
															   baseCloud2,
															   radii,
															   invalidNormals2,
															   maxThreadCount,
															   normalScaleSF2,
															   &pDlg,
															   baseOctree2);

		//now fix the orientation
		if (normalsAreOk2)
		{
			//some invalid normals?
			if (invalidNormals2 && app)
			{
				app->dispToConsole("[M3C2] Some normals are invalid! You may have to increase the scale.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			}

			//make normals horizontal if necessary
			if (normMode == qM3C2Normals::HORIZ_MODE)
			{
				qM3C2Normals::MakeNormalsHorizontal(*s_M3C2Params.coreNormals2);
			}

			//then either use a simple heuristic
			bool usePreferredOrientation = dlg.normOriPreferredRadioButton->isChecked();
			if (usePreferredOrientation)
			{
				int preferredOrientation = dlg.normOriPreferredComboBox->currentIndex();
				assert(preferredOrientation >= ccNormalVectors::PLUS_X && preferredOrientation <= ccNormalVectors::MINUS_SENSOR_ORIGIN);
				if (!ccNormalVectors::UpdateNormalOrientations(s_M3C2Params.corePoints,
															   *s_M3C2Params.coreNormals2,
															   static_cast<ccNormalVectors::Orientation>(preferredOrientation)))
				{
					errorMessage = "[M3C2] Failed to re-orient the normals (invalid parameter?)";
					error = true;
				}
			}
			else //or use external points
			{
				ccPointCloud* orientationCloud = dlg.getNormalsOrientationCloud();
				assert(orientationCloud);

				if (!qM3C2Normals::UpdateNormalOrientationsWithCloud(s_M3C2Params.corePoints,
																	 *s_M3C2Params.coreNormals2,
																	 orientationCloud,
																	 maxThreadCount,
																	 &pDlg))
				{
					errorMessage = "[M3C2] Failed to re-orient the normals with input point cloud!";
					error = true;
				}
			}
		}

		if (!normalsAreOk2)
		{
			errorMessage = "Failed to compute normals!";
			error = true;
		}
	}

	if (!error && s_M3C2Params.coreNormals && corePointsHaveBeenSubsampled)
	{
		if (s_M3C2Params.corePoints->hasNormals() || s_M3C2Params.corePoints->resizeTheNormsTable())
		{
			for (unsigned i = 0; i < s_M3C2Params.coreNormals->currentSize(); ++i)
				s_M3C2Params.corePoints->setPointNormalIndex(i, s_M3C2Params.coreNormals->getValue(i));
			s_M3C2Params.corePoints->showNormals(true);
		}
		else if (app)
		{
			app->dispToConsole("Failed to allocate memory for core points normals!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
	}

	qint64 initTime_ms = initTimer.elapsed();

	while (!error) //fake loop for easy break
	{
		//we display init. timing only if no error occurred!
		if (app)
			app->dispToConsole(QString("[M3C2] Initialization & normal computation: %1 s.").arg(initTime_ms / 1000.0, 0, 'f', 3), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		QElapsedTimer distCompTimer;
		distCompTimer.start();

		//we are either in vertical mode or we have as many normals as core points
		unsigned corePointCount = s_M3C2Params.corePoints->size();
		assert(normMode == qM3C2Normals::VERT_MODE || (s_M3C2Params.coreNormals && corePointCount == s_M3C2Params.coreNormals->currentSize()));

		pDlg.reset();
		CCCoreLib::NormalizedProgress nProgress(&pDlg, corePointCount);
		pDlg.setMethodTitle(QObject::tr("M3C2 Distances Computation"));
		pDlg.setInfo(QObject::tr("Core points: %1").arg(corePointCount));
		pDlg.start();
		s_M3C2Params.nProgress = &nProgress;

		//allocate distances SF
		s_M3C2Params.m3c2DistSF = new ccScalarField(M3C2_DIST_SF_NAME);
		s_M3C2Params.m3c2DistSF->link();
		if (!s_M3C2Params.m3c2DistSF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
		{
			errorMessage = "Failed to allocate memory for distance values!";
			error = true;
			break;
		}

        // allocate search depth scalar fields
        if (storeProjectionInfo)
        {
            if (!s_M3C2Params.indexSF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
        }
        if (exportSearchDepth)
        {
            if (!s_M3C2Params.searchDepth1SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.searchDepth2SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.meanMinusMed1SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.meanMinusMed2SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.normalsAngleSF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.sector1SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.gap1SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.sector2SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.gap2SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.sectorGapSF->resizeSafe(corePointCount, true, false))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
        }
        if (computeWelch)
        {
            if (!s_M3C2Params.welch_t_SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.welch_v_SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.welch_q_SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.welch_p_SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.welch_lod_SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
            if (!s_M3C2Params.welch_sig_SF->resizeSafe(corePointCount, true, SCALAR_ZERO))
            {
                errorMessage = "Failed to allocate memory for distance values!";
                error = true;
                break;
            }
        }

		//allocate dist. uncertainty SF
		s_M3C2Params.distUncertaintySF = new ccScalarField(DIST_UNCERTAINTY_SF_NAME);
		s_M3C2Params.distUncertaintySF->link();
		if (!s_M3C2Params.distUncertaintySF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
		{
			errorMessage = "Failed to allocate memory for dist. uncertainty values!";
			error = true;
			break;
		}
		//allocate change significance SF
		s_M3C2Params.sigChangeSF = new ccScalarField(SIG_CHANGE_SF_NAME);
		s_M3C2Params.sigChangeSF->link();
		if (!s_M3C2Params.sigChangeSF->resizeSafe(corePointCount, true, SCALAR_ZERO))
		{
			if (app)
				app->dispToConsole("Failed to allocate memory for change significance values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			s_M3C2Params.sigChangeSF->release();
			s_M3C2Params.sigChangeSF = nullptr;
			//no need to stop just for this SF!
			//error = true;
			//break;
		}

		if (dlg.exportStdDevInfoCheckBox->isChecked())
		{
			QString prefix("STD");
			if (s_M3C2Params.usePrecisionMaps)
			{
				prefix = "SigmaN";
			}
			else if (s_M3C2Params.distAndUncerMethod == qM3C2Tools::USE_MEDIAN_AND_IQR)
			{
				prefix = "IQR";
			}
			else if (s_M3C2Params.distAndUncerMethod == qM3C2Tools::USE_MIN_AND_MAX_MINUS_MIN)
			{
				prefix = "MAX-MIN";
			}
			else if (s_M3C2Params.distAndUncerMethod == qM3C2Tools::USE_MIN_AND_MAX_MINUS_MIN)
			{
				prefix = "PRCT";
			}
			//allocate cloud #1 std. dev. SF
			QString stdDevSFName1 = QString(STD_DEV_CLOUD1_SF_NAME).arg(prefix);
			s_M3C2Params.stdDevCloud1SF = new ccScalarField(qPrintable(stdDevSFName1));
			s_M3C2Params.stdDevCloud1SF->link();
			if (!s_M3C2Params.stdDevCloud1SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
			{
				if (app)
					app->dispToConsole("Failed to allocate memory for cloud #1 std. dev. values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_M3C2Params.stdDevCloud1SF->release();
				s_M3C2Params.stdDevCloud1SF = nullptr;
			}
			//allocate cloud #2 std. dev. SF
			QString stdDevSFName2 = QString(STD_DEV_CLOUD2_SF_NAME).arg(prefix);
			s_M3C2Params.stdDevCloud2SF = new ccScalarField(qPrintable(stdDevSFName2));
			s_M3C2Params.stdDevCloud2SF->link();
			if (!s_M3C2Params.stdDevCloud2SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
			{
				if (app)
					app->dispToConsole("Failed to allocate memory for cloud #2 std. dev. values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_M3C2Params.stdDevCloud2SF->release();
				s_M3C2Params.stdDevCloud2SF = nullptr;
			}
		}
		if (dlg.exportDensityAtProjScaleCheckBox->isChecked())
		{
			//allocate cloud #1 density SF
			s_M3C2Params.densityCloud1SF = new ccScalarField(DENSITY_CLOUD1_SF_NAME);
			s_M3C2Params.densityCloud1SF->link();
			if (!s_M3C2Params.densityCloud1SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
			{
				if (app)
					app->dispToConsole("Failed to allocate memory for cloud #1 density values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_M3C2Params.densityCloud1SF->release();
				s_M3C2Params.densityCloud1SF = nullptr;
			}
			//allocate cloud #2 density SF
			s_M3C2Params.densityCloud2SF = new ccScalarField(DENSITY_CLOUD2_SF_NAME);
			s_M3C2Params.densityCloud2SF->link();
			if (!s_M3C2Params.densityCloud2SF->resizeSafe(corePointCount, true, CCCoreLib::NAN_VALUE))
			{
				if (app)
					app->dispToConsole("Failed to allocate memory for cloud #2 density values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_M3C2Params.densityCloud2SF->release();
				s_M3C2Params.densityCloud2SF = nullptr;
			}
		}

		//get best levels for neighbourhood extraction on both octrees
		assert(s_M3C2Params.cloud1Octree && s_M3C2Params.cloud2Octree);
		PointCoordinateType equivalentRadius = pow(s_M3C2Params.projectionDepth * s_M3C2Params.projectionDepth * s_M3C2Params.projectionRadius, CCCoreLib::PC_ONE / 3);
		s_M3C2Params.level1 = s_M3C2Params.cloud1Octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(equivalentRadius);
		if (app)
			app->dispToConsole(QString("[M3C2] Working subdivision level (cloud #1): %1").arg(s_M3C2Params.level1), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		s_M3C2Params.level2 = s_M3C2Params.cloud2Octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(equivalentRadius);
		if (app)
			app->dispToConsole(QString("[M3C2] Working subdivision level (cloud #2): %1").arg(s_M3C2Params.level2), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//other options
		s_M3C2Params.updateNormal = (normMode != qM3C2Normals::VERT_MODE);
		s_M3C2Params.exportNormal = s_M3C2Params.updateNormal && !s_M3C2Params.outputCloud->hasNormals();
		if (s_M3C2Params.exportNormal && !s_M3C2Params.outputCloud->resizeTheNormsTable()) //resize because we will 'set' the normal in ComputeM3C2DistForPoint
		{
			if (app)
				app->dispToConsole("Failed to allocate memory for exporting normals!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			s_M3C2Params.exportNormal = false;
		}
		s_M3C2Params.computeConfidence = (s_M3C2Params.distUncertaintySF || s_M3C2Params.sigChangeSF);

		//compute distances
		{
			std::vector<unsigned> pointIndexes;
			bool useParallelStrategy = true;
#ifdef _DEBUG
			useParallelStrategy = false;
#endif

			if (storeProjectionInfo)
			{
				if (app)
					app->dispToConsole("multiInterception", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				useParallelStrategy = false;
			}

			if (useParallelStrategy)
			{
				try
				{
					pointIndexes.resize(corePointCount);
				}
				catch (const std::bad_alloc&)
				{
					//not enough memory
					useParallelStrategy = false;
				}
			}

			if (useParallelStrategy)
			{
				for (unsigned i = 0; i < corePointCount; ++i)
				{
					pointIndexes[i] = i;
				}

				if (maxThreadCount == 0)
				{
					maxThreadCount = ccQtHelpers::GetMaxThreadCount();
				}
				assert(maxThreadCount > 0 && maxThreadCount <= QThread::idealThreadCount());
				QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
				QtConcurrent::blockingMap(pointIndexes, ComputeM3C2DistForPoint);
			}
			else
			{
				//manually call the static per-point method!
				for (unsigned i = 0; i < corePointCount; ++i)
				{
					ComputeM3C2DistForPoint(i);
				}
			}
		}

		if (s_M3C2Params.processCanceled)
		{
			errorMessage = "Process canceled by user!";
			error = true;
		}
		else if (s_M3C2Params.processFailed)
		{
			errorMessage = "Process failed (not enough memory?)";
			error = true;
		}
		else
		{
			qint64 distTime_ms = distCompTimer.elapsed();
			//we display init. timing only if no error occurred!
			if (app)
				app->dispToConsole(QString("[M3C2] Distances computation: %1 s.").arg(static_cast<double>(distTime_ms) / 1000.0, 0, 'f', 3), ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}

		s_M3C2Params.nProgress = nullptr;

		break; //to break from fake loop
	}

	//associate scalar fields to the output cloud
	//(use reverse order so as to get the index of
	//the most important one at the end)
	if (!error)
	{
		assert(s_M3C2Params.outputCloud && s_M3C2Params.corePoints);
		int sfIdx = -1;
        int sfIdxM3C2Dist = -1;

		//normal scales
		if (normalScaleSF)
		{
			normalScaleSF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, normalScaleSF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(normalScaleSF);
		}

		//add clouds' density SFs to output cloud
		if (s_M3C2Params.densityCloud1SF)
		{
			s_M3C2Params.densityCloud1SF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.densityCloud1SF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.densityCloud1SF);
		}
		if (s_M3C2Params.densityCloud2SF)
		{
			s_M3C2Params.densityCloud2SF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.densityCloud2SF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.densityCloud2SF);
		}

		//add clouds' std. dev. SFs to output cloud
		if (s_M3C2Params.stdDevCloud1SF)
		{
			s_M3C2Params.stdDevCloud1SF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.stdDevCloud1SF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.stdDevCloud1SF);
		}
		if (s_M3C2Params.stdDevCloud2SF)
		{
			//add cloud #2 std. dev. SF to output cloud
			s_M3C2Params.stdDevCloud2SF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.stdDevCloud2SF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.stdDevCloud2SF);
		}

		if (s_M3C2Params.sigChangeSF)
		{
			//add significance SF to output cloud
			s_M3C2Params.sigChangeSF->computeMinAndMax();
			s_M3C2Params.sigChangeSF->setMinDisplayed(SCALAR_ONE);
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.sigChangeSF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.sigChangeSF);
		}

		if (s_M3C2Params.distUncertaintySF)
		{
			//add dist. uncertainty SF to output cloud
			s_M3C2Params.distUncertaintySF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.distUncertaintySF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.distUncertaintySF);
		}

		if (s_M3C2Params.m3c2DistSF)
		{
			//add M3C2 distances SF to output cloud
			s_M3C2Params.m3c2DistSF->computeMinAndMax();
			s_M3C2Params.m3c2DistSF->setSymmetricalScale(true);
            s_M3C2Params.m3c2DistSF->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::BWR));
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.m3c2DistSF->getName());
            sfIdxM3C2Dist = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.m3c2DistSF);
		}

        // MULTI-INTERCEPTION
        if (s_M3C2Params.searchDepth1SF)
        {
            s_M3C2Params.searchDepth1SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.searchDepth1SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.searchDepth1SF);
        }
        if (s_M3C2Params.searchDepth2SF)
        {
            s_M3C2Params.searchDepth2SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.searchDepth2SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.searchDepth2SF);
        }
        if (s_M3C2Params.indexSF)
        {
            s_M3C2Params.indexSF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.indexSF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.indexSF);
        }
        if (s_M3C2Params.meanMinusMed1SF)
        {
            s_M3C2Params.meanMinusMed1SF->computeMinAndMax();
            s_M3C2Params.meanMinusMed1SF->setSymmetricalScale(true);
            s_M3C2Params.meanMinusMed1SF->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::BWR));
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.meanMinusMed1SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.meanMinusMed1SF);
        }
        if (s_M3C2Params.meanMinusMed2SF)
        {
            s_M3C2Params.meanMinusMed2SF->computeMinAndMax();
            s_M3C2Params.meanMinusMed2SF->setSymmetricalScale(true);
            s_M3C2Params.meanMinusMed2SF->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::BWR));
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.meanMinusMed2SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.meanMinusMed2SF);
        }
        if (s_M3C2Params.normalsAngleSF)
        {
            s_M3C2Params.normalsAngleSF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.normalsAngleSF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.normalsAngleSF);
        }
        if (s_M3C2Params.sector1SF)
        {
            s_M3C2Params.sector1SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.sector1SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.sector1SF);
        }
        if (s_M3C2Params.gap1SF)
        {
            s_M3C2Params.gap1SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.gap1SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.gap1SF);
        }
        if (s_M3C2Params.sector2SF)
        {
            s_M3C2Params.sector2SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.sector2SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.sector2SF);
        }
        if (s_M3C2Params.gap2SF)
        {
            s_M3C2Params.gap2SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.gap2SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.gap2SF);
        }
        if (s_M3C2Params.sectorGapSF)
        {
            s_M3C2Params.sectorGapSF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.sectorGapSF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.sectorGapSF);
        }
        // WELCH
        if (s_M3C2Params.welch_t_SF)
        {
            s_M3C2Params.welch_t_SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.welch_t_SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.welch_t_SF);
        }
        if (s_M3C2Params.welch_v_SF)
        {
            s_M3C2Params.welch_v_SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.welch_v_SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.welch_v_SF);
        }
        if (s_M3C2Params.welch_q_SF)
        {
            s_M3C2Params.welch_q_SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.welch_q_SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.welch_q_SF);
        }
        if (s_M3C2Params.welch_p_SF)
        {
            s_M3C2Params.welch_p_SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.welch_p_SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.welch_p_SF);
        }
        if (s_M3C2Params.welch_lod_SF)
        {
            s_M3C2Params.welch_lod_SF->computeMinAndMax();
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.welch_lod_SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.welch_lod_SF);
        }
        if (s_M3C2Params.welch_sig_SF)
        {
            s_M3C2Params.welch_sig_SF->computeMinAndMax();
            s_M3C2Params.sigChangeSF->setMinDisplayed(SCALAR_ONE);
            RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.welch_sig_SF->getName());
            sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.welch_sig_SF);
        }

		s_M3C2Params.outputCloud->invalidateBoundingBox(); //see 'const_cast<...>' in ComputeM3C2DistForPoint ;)
        s_M3C2Params.outputCloud->setCurrentDisplayedScalarField(sfIdxM3C2Dist);
		s_M3C2Params.outputCloud->showSF(true);
		s_M3C2Params.outputCloud->showNormals(true);
		s_M3C2Params.outputCloud->setVisible(true);
		s_M3C2Params.outputCloud->prepareDisplayForRefresh();

        if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD1_AND_CLOUD2)
        {
            s_M3C2Params.outputCloud->invalidateBoundingBox(); //see 'const_cast<...>' in ComputeM3C2DistForPoint ;)
            s_M3C2Params.outputCloud->showNormals(true);
            s_M3C2Params.outputCloud->setVisible(true);
        }

		if (s_M3C2Params.outputCloud != cloud1 && s_M3C2Params.outputCloud != cloud2)
		{
			s_M3C2Params.outputCloud->setName(outputName);
            if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD1_AND_CLOUD2)
            {
                s_M3C2Params.outputCloud2->setName(outputName2);
            }
			s_M3C2Params.outputCloud->setDisplay(s_M3C2Params.corePoints->getDisplay());
			s_M3C2Params.outputCloud->importParametersFrom(s_M3C2Params.corePoints);
            if (s_M3C2Params.outputCloud2)
                s_M3C2Params.outputCloud2->importParametersFrom(s_M3C2Params.corePoints);
			if (app)
			{
				app->addToDB(s_M3C2Params.outputCloud);
                if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD1_AND_CLOUD2)
                {
                    app->addToDB(s_M3C2Params.outputCloud2);
                }
			}
			else
			{
				//command line mode
				outputCloud = s_M3C2Params.outputCloud;
                outputCloud2 = s_M3C2Params.outputCloud2;
			}
		}
	}
	else if (s_M3C2Params.outputCloud)
	{
		if (s_M3C2Params.outputCloud != s_M3C2Params.corePoints)
		{
			delete s_M3C2Params.outputCloud;
		}
		s_M3C2Params.outputCloud = nullptr;
	}

    if (app && storeProjectionInfo)
    {
        app->addToDB(projectionCloud);
        projCloud_SF_index->computeMinAndMax();
        projCloud_SF_cloud->computeMinAndMax();
        projectionCloud->setCurrentDisplayedScalarField(projCloud_sfIdx_index);
        projectionCloud->showSF(true);
    }

	if (app)
		app->refreshAll();

	//release structures
	if (normalScaleSF)
		normalScaleSF->release();
	if (s_M3C2Params.coreNormals)
		s_M3C2Params.coreNormals->release();
	if (s_M3C2Params.m3c2DistSF)
		s_M3C2Params.m3c2DistSF->release();
	if (s_M3C2Params.sigChangeSF)
		s_M3C2Params.sigChangeSF->release();
	if (s_M3C2Params.distUncertaintySF)
		s_M3C2Params.distUncertaintySF->release();
	if (s_M3C2Params.stdDevCloud1SF)
		s_M3C2Params.stdDevCloud1SF->release();
	if (s_M3C2Params.stdDevCloud2SF)
		s_M3C2Params.stdDevCloud2SF->release();
	if (s_M3C2Params.densityCloud1SF)
		s_M3C2Params.densityCloud1SF->release();
	if (s_M3C2Params.densityCloud2SF)
		s_M3C2Params.densityCloud2SF->release();

    if (s_M3C2Params.searchDepth1SF)
        s_M3C2Params.searchDepth1SF->release();
    if (s_M3C2Params.searchDepth2SF)
        s_M3C2Params.searchDepth2SF->release();
    if (s_M3C2Params.normalsAngleSF)
        s_M3C2Params.normalsAngleSF->release();
    if (s_M3C2Params.sector1SF)
        s_M3C2Params.sector1SF->release();
    if (s_M3C2Params.gap1SF)
        s_M3C2Params.gap1SF->release();
    if (s_M3C2Params.sector2SF)
        s_M3C2Params.sector2SF->release();
    if (s_M3C2Params.gap2SF)
        s_M3C2Params.gap2SF->release();
    if (s_M3C2Params.sectorGapSF)
        s_M3C2Params.sectorGapSF->release();
    if (s_M3C2Params.indexSF)
        s_M3C2Params.indexSF->release();
    if (s_M3C2Params.meanMinusMed1SF)
        s_M3C2Params.meanMinusMed1SF->release();
    if (s_M3C2Params.meanMinusMed2SF)
        s_M3C2Params.meanMinusMed2SF->release();
    if (s_M3C2Params.welch_t_SF)
        s_M3C2Params.welch_t_SF->release();
    if (s_M3C2Params.welch_v_SF)
        s_M3C2Params.welch_v_SF->release();
    if (s_M3C2Params.welch_q_SF)
        s_M3C2Params.welch_q_SF->release();
    if (s_M3C2Params.welch_p_SF)
        s_M3C2Params.welch_p_SF->release();
    if (s_M3C2Params.welch_lod_SF)
        s_M3C2Params.welch_lod_SF->release();
    if (s_M3C2Params.welch_sig_SF)
        s_M3C2Params.welch_sig_SF->release();

    if (normalScaleSF2)
        normalScaleSF2->release();
    if (s_M3C2Params.coreNormals2)
        s_M3C2Params.coreNormals2->release();

    return !error;
}

