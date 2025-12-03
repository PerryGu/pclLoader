#pragma once

#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>

namespace PclUtilities
{
	struct Bounds
	{
		MPoint min{0.0, 0.0, 0.0};
		MPoint max{0.0, 0.0, 0.0};
	};

	Bounds computeBounds(const MPointArray& points);

	void appendLinesFromBounds(const MPoint& minCorner,
	                           const MPoint& maxCorner,
	                           MPointArray& outLineSegments);
} // namespace PclUtilities


