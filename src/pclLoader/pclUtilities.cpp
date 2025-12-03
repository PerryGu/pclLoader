#include "pclUtilities.h"
#include "core/pclGeometryUtils.h"

#include <vector>

namespace
{
	Eigen::Vector3d toEigen(const MPoint& point)
	{
		return Eigen::Vector3d(point.x, point.y, point.z);
	}

	MPoint toMPoint(const Eigen::Vector3d& point)
	{
		return MPoint(point.x(), point.y(), point.z());
	}
}

namespace PclUtilities
{
	Bounds computeBounds(const MPointArray& points)
	{
		Bounds result;
		if (points.length() == 0)
		{
			return result;
		}

		std::vector<Eigen::Vector3d> eigenPoints;
		eigenPoints.reserve(points.length());
		for (unsigned int i = 0; i < points.length(); ++i)
		{
			eigenPoints.push_back(toEigen(points[i]));
		}

		const auto bbox = pclCore::computeBoundingBox(eigenPoints);
		if (!bbox.valid)
		{
			return result;
		}

		result.min = toMPoint(bbox.min);
		result.max = toMPoint(bbox.max);
		return result;
	}

	void appendLinesFromBounds(const MPoint& minCorner,
	                           const MPoint& maxCorner,
	                           MPointArray& outLineSegments)
	{
		outLineSegments.clear();

		const auto bbox = pclCore::BoundingBox::FromMinMax(toEigen(minCorner), toEigen(maxCorner));
		const auto linePoints = pclCore::buildBoundingBoxLines(bbox);
		for (const auto& point : linePoints)
		{
			outLineSegments.append(toMPoint(point));
		}
	}
} // namespace PclUtilities
