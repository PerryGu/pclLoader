#ifndef PCLGEOMETRYUTILS_H
#define PCLGEOMETRYUTILS_H

#include <Eigen/Core>
#include <vector>

namespace pclCore
{
	/// Simple axis-aligned bounding box helper used by core utilities.
	/// Provides efficient point containment testing and expansion operations.
	struct BoundingBox
	{
		Eigen::Vector3d min{ 0.0, 0.0, 0.0 };  ///< Minimum corner coordinates.
		Eigen::Vector3d max{ 0.0, 0.0, 0.0 };  ///< Maximum corner coordinates.
		bool            valid = false;         ///< True if the box has been initialized with at least one point.

		/// Creates a bounding box from explicit min and max corners.
		/// @param minPoint The minimum corner point.
		/// @param maxPoint The maximum corner point.
		/// @return A valid bounding box containing the specified region.
		static BoundingBox FromMinMax(const Eigen::Vector3d& minPoint, const Eigen::Vector3d& maxPoint);

		/// Expands the bounding box to include the given point.
		/// If the box is invalid, it becomes a zero-volume box at the point.
		/// @param point The point to include in the bounding box.
		void expand(const Eigen::Vector3d& point);

		/// Tests whether a point lies within the bounding box (inclusive boundaries).
		/// @param point The point to test.
		/// @return True if the point is inside or on the boundary, false if invalid or outside.
		bool contains(const Eigen::Vector3d& point) const;
	};

	/// Computes the axis-aligned bounding box for a collection of 3D points.
	/// @param points The input points to compute bounds for.
	/// @return A bounding box containing all points, or an invalid box if the input is empty.
	BoundingBox computeBoundingBox(const std::vector<Eigen::Vector3d>& points);

	/// Builds line segments representing the 12 edges of a bounding box.
	/// The result is a flat array of points: [v0, v1, v0, v2, ...] where each pair forms a line segment.
	/// @param bbox The bounding box to generate lines for.
	/// @return A vector of points (24 elements = 12 edges × 2 vertices per edge).
	std::vector<Eigen::Vector3d> buildBoundingBoxLines(const BoundingBox& bbox);

	/// Filters point indices based on whether they lie inside or outside a bounding box.
	/// @param bbox The bounding box to test against.
	/// @param points The full point array to filter.
	/// @param keepInside If true, returns indices of points inside the box; if false, returns indices of points outside.
	/// @return A vector of indices into the points array matching the filter criteria.
	std::vector<size_t> filterPointIndices(const BoundingBox& bbox, const std::vector<Eigen::Vector3d>& points, bool keepInside);
}

#endif // PCLGEOMETRYUTILS_H

