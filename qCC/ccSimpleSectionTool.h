#ifndef CCSIMPLESECTIONTOOL_H
#define CCSIMPLESECTIONTOOL_H

//qCC_db
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>

#include <ccGLWindowInterface.h>

class ccSimpleSectionTool : public QObject
{
	Q_OBJECT

public:
	explicit ccSimpleSectionTool(ccGLWindowInterface *window, QObject *parent);

	inline void addPointToPolyline(int x, int y) { return addPointToPolylineExt(x, y, false); }
	void addPointToPolylineExt(int x, int y, bool allowClicksOutside);
	void closePolyLine(int x = 0, int y = 0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
	void updatePolyLine(int x, int y, Qt::MouseButtons buttons);
	void closeRectangle();
	void stopRunning();
	void run();

protected:

	//! Process states
	enum ProcessStates
	{
		POLYLINE		= 1,
		RECTANGLE		= 2,
		//...			= 4,
		//...			= 8,
		//...			= 16,
		PAUSED			= 32,
		STARTED			= 64,
		RUNNING			= 128,
	};

	//! Current process state
	unsigned m_state;

	//! Segmentation polyline
	ccPolyline* m_segmentationPoly;

	//! Segmentation polyline vertices
	ccPointCloud* m_polyVertices;

private:
	//! Associated (MDI) window
	ccGLWindowInterface* m_associatedWin;

	//! Selection mode
	bool m_rectangularSelection;
};

#endif // CCSIMPLESECTIONTOOL_H
