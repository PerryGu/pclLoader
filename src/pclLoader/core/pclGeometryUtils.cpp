#include "pclGeometryUtils.h"

#include <algorithm>

namespace pclCore
{
	BoundingBox BoundingBox::FromMinMax(const Eigen::Vector3d& minPoint, const Eigen::Vector3d& maxPoint)
	{
		BoundingBox box;
		box.min = minPoint;
		box.max = maxPoint;
		box.valid = true;
		return box;
	}

	void BoundingBox::expand(const Eigen::Vector3d& point)
	{
		if (!valid)
		{
			min = point;
			max = point;
			valid = true;
			return;
		}

		min = min.cwiseMin(point);
		max = max.cwiseMax(point);
	}

	bool BoundingBox::contains(const Eigen::Vector3d& point) const
	{
		if (!valid)
		{
			return false;
		}

		return point.x() >= min.x() && point.x() <= max.x()
			&& point.y() >= min.y() && point.y() <= max.y()
			&& point.z() >= min.z() && point.z() <= max.z();
	}

	BoundingBox computeBoundingBox(const std::vector<Eigen::Vector3d>& points)
	{
		BoundingBox box;
		for (const auto& point : points)
		{
			box.expand(point);
		}
		return box;
	}

	std::vector<Eigen::Vector3d> buildBoundingBoxLines(const BoundingBox& bbox)
	{
		std::vector<Eigen::Vector3d> lines;
		if (!bbox.valid)
		{
			return lines;
		}

		// Pre-allocate space for 12 edges × 2 vertices = 24 points.
		lines.reserve(24);

		// Compute the 8 corner vertices of the bounding box.
		const Eigen::Vector3d v0(bbox.max.x(), bbox.max.y(), bbox.max.z());  // Top-front-right
		const Eigen::Vector3d v1(bbox.min.x(), bbox.max.y(), bbox.max.z());  // Top-front-left
		const Eigen::Vector3d v2(bbox.min.x(), bbox.min.y(), bbox.max.z());  // Bottom-front-left
		const Eigen::Vector3d v3(bbox.max.x(), bbox.min.y(), bbox.max.z());  // Bottom-front-right
		const Eigen::Vector3d v4(bbox.max.x(), bbox.min.y(), bbox.min.z());   // Bottom-back-right
		const Eigen::Vector3d v5(bbox.max.x(), bbox.max.y(), bbox.min.z());   // Top-back-right
		const Eigen::Vector3d v6(bbox.min.x(), bbox.max.y(), bbox.min.z());    // Top-back-left
		const Eigen::Vector3d v7(bbox.min.x(), bbox.min.y(), bbox.min.z());   // Bottom-back-left

		// Helper lambda to append a line segment (two vertices) to the output.
		auto append = [&lines](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
		{
			lines.push_back(a);
			lines.push_back(b);
		};

		// Front face edges (z = max).
		append(v0, v1);
		append(v1, v2);
		append(v2, v3);
		append(v3, v0);

		// Back face edges (z = min).
		append(v4, v5);
		append(v5, v6);
		append(v6, v7);
		append(v7, v4);

		// Connecting edges between front and back faces.
		append(v0, v5);
		append(v1, v6);
		append(v2, v7);
		append(v3, v4);

		return lines;
	}

	std::vector<size_t> filterPointIndices(const BoundingBox& bbox, const std::vector<Eigen::Vector3d>& points, bool keepInside)
	{
		std::vector<size_t> indices;
		if (!bbox.valid || points.empty())
		{
			return indices;
		}

		// Pre-allocate with a conservative estimate (assume ~50% match rate).
		indices.reserve(points.size() / 2);

		// Iterate through all points and collect indices matching the filter criteria.
		for (size_t i = 0; i < points.size(); ++i)
		{
			const bool inside = bbox.contains(points[i]);
			if (inside == keepInside)
			{
				indices.push_back(i);
			}
		}
		return indices;
	}
}

