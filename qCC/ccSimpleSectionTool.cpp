#include "ccSimpleSectionTool.h"

#include <QApplication>

ccSimpleSectionTool::ccSimpleSectionTool(ccGLWindowInterface *window, QObject *parent)
	: QObject{parent}
	, m_associatedWin{window}
{
	connect(m_associatedWin->signalEmitter(), &ccGLWindowSignalEmitter::leftButtonClicked,	this, &ccSimpleSectionTool::addPointToPolyline);
	connect(m_associatedWin->signalEmitter(), &ccGLWindowSignalEmitter::rightButtonClicked,	this, &ccSimpleSectionTool::closePolyLine);
	connect(m_associatedWin->signalEmitter(), &ccGLWindowSignalEmitter::mouseMoved,			this, &ccSimpleSectionTool::updatePolyLine);
}

void ccSimpleSectionTool::closePolyLine(int, int)
{
	//only for polyline in RUNNING mode
	if ((m_state & POLYLINE) == 0 || (m_state & RUNNING) == 0)
		return;

	if (m_associatedWin)
	{
		m_associatedWin->doReleaseMouse();
	}

	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	if (vertCount < 4)
	{
		m_segmentationPoly->clear();
		m_polyVertices->clear();
	}
	else
	{
		//remove last point!
		m_segmentationPoly->resize(vertCount - 1); //can't fail --> smaller
		m_segmentationPoly->setClosed(true);
	}

	//stop
	stopRunning();

	if (m_associatedWin)
	{
		m_associatedWin->redraw(true, false);
	}
}

void ccSimpleSectionTool::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	//process not started yet?
	if ((m_state & RUNNING) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	assert(m_polyVertices);
	assert(m_segmentationPoly);

	unsigned vertCount = m_polyVertices->size();

	//new point (expressed relatively to the screen center)
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
				static_cast<PointCoordinateType>(pos2D.y()),
				0);

	if (m_state & RECTANGLE)
	{
		//we need 4 points for the rectangle!
		if (vertCount != 4)
			m_polyVertices->resize(4);

		const CCVector3 *A = m_polyVertices->getPointPersistentPtr(0);
		CCVector3 *B = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(1));
		CCVector3 *C = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(2));
		CCVector3 *D = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(3));
		*B = CCVector3(A->x, P.y, 0);
		*C = P;
		*D = CCVector3(P.x, A->y, 0);

		if (vertCount != 4)
		{
			m_segmentationPoly->clear();
			if (!m_segmentationPoly->addPointIndex(0, 4))
			{
				ccLog::Error("Out of memory!");
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
	}
	else if (m_state & POLYLINE)
	{
		if (vertCount < 2)
			return;
		//we replace last point by the current one
		CCVector3 *lastP = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(vertCount - 1));
		*lastP = P;
	}

	m_associatedWin->redraw(true, false);
}

void ccSimpleSectionTool::stopRunning()
{
	m_state &= (~RUNNING);
}

void ccSimpleSectionTool::addPointToPolylineExt(int x, int y, bool allowClicksOutside)
{
	if ((m_state & STARTED) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	if (	!allowClicksOutside
		&&	(x < 0 || y < 0 || x >= m_associatedWin->qtWidth() || y >= m_associatedWin->qtHeight())
		)
	{
		//ignore clicks outside of the 3D view
		return;
	}

	assert(m_polyVertices);
	assert(m_segmentationPoly);
	unsigned vertCount = m_polyVertices->size();

	//new point
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
				static_cast<PointCoordinateType>(pos2D.y()),
				0);

	//CTRL key pressed at the same time?
	bool ctrlKeyPressed = ((QApplication::keyboardModifiers() & Qt::ControlModifier) == Qt::ControlModifier);

	//start new polyline?
	if (((m_state & RUNNING) == 0) || vertCount == 0 || ctrlKeyPressed)
	{
		//reset state
		m_state = POLYLINE;
		m_state |= STARTED;
		run();

		//reset polyline
		m_polyVertices->clear();
		if (!m_polyVertices->reserve(2))
		{
			ccLog::Error("Out of memory!");
			return;
		}
		//we add the same point twice (the last point will be used for display only)
		m_polyVertices->addPoint(P);
		m_polyVertices->addPoint(P);
		m_segmentationPoly->clear();
		if (!m_segmentationPoly->addPointIndex(0, 2))
		{
			ccLog::Error("Out of memory!");
			return;
		}
	}
	else //next points in "polyline mode" only
	{
		//we were already in 'polyline' mode?
		if (m_state & POLYLINE)
		{
			if (!m_polyVertices->reserve(vertCount+1))
			{
				ccLog::Error("Out of memory!");
				return;
			}

			//we replace last point by the current one
			CCVector3 *lastP = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(vertCount-1));
			*lastP = P;
			//and add a new (equivalent) one
			m_polyVertices->addPoint(P);
			if (!m_segmentationPoly->addPointIndex(vertCount))
			{
				ccLog::Error("Out of memory!");
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
		else //we must change mode
		{
			assert(false); //we shouldn't fall here?!
			stopRunning();
			addPointToPolylineExt(x, y, allowClicksOutside);
			return;
		}
	}

	//DGM: to increase the poll rate of the mouse movements in ccGLWindow::mouseMoveEvent
	//we have to completely grab the mouse focus!
	//(the only way to take back the control is to right-click now...)
	m_associatedWin->doGrabMouse();
	m_associatedWin->redraw(true, false);
}

void ccSimpleSectionTool::run()
{
	m_state |= RUNNING;
}
