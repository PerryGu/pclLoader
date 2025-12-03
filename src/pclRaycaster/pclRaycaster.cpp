// Standard library
#include <algorithm>
#include <cmath>

// Project headers
#include "pclRaycaster.h"

// Error checking macro
#ifndef CHECK_MSTATUS_AND_RETURN_IT
#define CHECK_MSTATUS_AND_RETURN_IT(status) \
	do { \
		if ((status) != MS::kSuccess) { \
			return (status); \
		} \
	} while (0)
#endif

// Maya API 
#include <maya/MFnDagNode.h>
#include <maya/MFnCamera.h>
#include <maya/MFloatMatrix.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnVectorArrayData.h>
#include <maya/MSelectionList.h>
#include <maya/MGlobal.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MTime.h>
#include <maya/MPlugArray.h>
#include <maya/MDagPath.h>
#include <maya/M3dView.h>
#include <maya/MBoundingBox.h>
#include <maya/MMatrix.h>
#include <maya/MIntArray.h>
#include <maya/MFloatArray.h>

namespace {
	/// Retrieves the render resolution from Maya's defaultResolution node using the C++ API.
	/// @param[out] width The render width in pixels.
	/// @param[out] height The render height in pixels.
	/// @return True if successful, false otherwise.
	bool getRenderResolution(double& width, double& height)
	{
		MSelectionList selection;
		MStatus status = selection.add("defaultResolution");
		if (status != MS::kSuccess)
		{
			return false;
		}

		MObject resolutionNode;
		status = selection.getDependNode(0, resolutionNode);
		if (status != MS::kSuccess)
		{
			return false;
		}

		MFnDependencyNode fnNode(resolutionNode);
		MPlug widthPlug = fnNode.findPlug("width", true, &status);
		if (status != MS::kSuccess)
		{
			return false;
		}

		MPlug heightPlug = fnNode.findPlug("height", true, &status);
		if (status != MS::kSuccess)
		{
			return false;
		}

		status = widthPlug.getValue(width);
		if (status != MS::kSuccess)
		{
			return false;
		}

		status = heightPlug.getValue(height);
		return (status == MS::kSuccess);
	}
}

MTypeId PclRaycaster::id(0x000001162642);
MString	PclRaycaster::drawDbClassification("drawdb/geometry/PclRaycaster_GeometryOverride");
MString	PclRaycaster::drawRegistrantId("PclRaycasterNode_GeometryOverridePlugin");

MObject PclRaycaster::particleCount;
MObject PclRaycaster::inPointsPosition_Attr;
MObject PclRaycaster::inPointsColor_Attr;
MObject PclRaycaster::inTime_Attr;
MObject PclRaycaster::timeOffset_Attr;
MObject PclRaycaster::pointNumber_Attr;

MObject PclRaycaster::displayType_Attr;
MObject PclRaycaster::raycastOnOff_Attr;
MObject PclRaycaster::raycastInputX_Attr;
MObject PclRaycaster::raycastInputY_Attr;
MObject PclRaycaster::minRadius_Attr;
MObject PclRaycaster::pointsSize_Attr;

MObject PclRaycaster::outPosition_Attr;
MObject PclRaycaster::outColor_Attr;
MObject PclRaycaster::outPositionRaycast_Attr;
MObject PclRaycaster::closestPointHit_Attr;

MObject PclRaycaster::cameraMatrix_Attr;


PclRaycaster::PclRaycaster()
{}

PclRaycaster::~PclRaycaster()
{}

void* PclRaycaster::creator()
{
	return new PclRaycaster();
}


void PclRaycaster::postConstructor()
{
	MStatus status;
	thisNode = thisMObject();
	MFnDependencyNode fnSequencer(thisNode);
	MFnDagNode this_dag(thisNode);
	fnSequencer.setName("pclRaycasterShape#");

}




MStatus PclRaycaster::compute(const MPlug& plug, MDataBlock& dataBlock)
{
	MStatus status = MS::kUnknownParameter;

	// Gather all frequently-used inputs at the start for clarity.
	EvaluateContext ctx;
	ctx.timeVal = dataBlock.inputValue(inTime_Attr).asFloat();
	ctx.timeOffset = dataBlock.inputValue(timeOffset_Attr).asFloat();
	ctx.displayType = dataBlock.inputValue(displayType_Attr).asShort();
	ctx.raycastOnOff = dataBlock.inputValue(raycastOnOff_Attr).asShort();
	ctx.raycastInputX = dataBlock.inputValue(raycastInputX_Attr).asInt();
	ctx.raycastInputY = dataBlock.inputValue(raycastInputY_Attr).asInt();
	ctx.minRadius = dataBlock.inputValue(minRadius_Attr).asFloat();
	ctx.pointSize = dataBlock.inputValue(pointsSize_Attr).asFloat();
	ctx.cameraMatrix = dataBlock.inputValue(cameraMatrix_Attr).asMatrix();

	// Mirror frequently-used inputs into member variables for legacy helpers.
	minRadius = ctx.minRadius;
	pointsSize = ctx.pointSize;

	// Prepare output handles.
	MDataHandle closestPointHit_h = dataBlock.outputValue(closestPointHit_Attr);
	MDataHandle pointNumber_h = dataBlock.outputValue(pointNumber_Attr);

	// Initialize working variables.
	MBoundingBox the_bbox;
	MPoint closestPointHit(MPoint(0, 0, 0));
	int numberOfPointsforFrame = 0;
	MVectorArray pointsInContainer;

	// Get input points from pclLoader via dependency graph connection.
	MPlug pointsPositionPlug(thisNode, inPointsPosition_Attr);
	MPlug pointsColorPlug(thisNode, inPointsColor_Attr);
	
	if (!pointsPositionPlug.isConnected())
	{
		// Input connection is required - node cannot function without pclLoader connection.
		MGlobal::displayError("pclRaycaster: inPointsPosition must be connected to pclLoader output");
		dataBlock.setClean(plug);
		return MS::kFailure;
	}

	// Get points from pclLoader input.
	MObject pointPos_in = dataBlock.inputValue(inPointsPosition_Attr).data();
	MFnPointArrayData dataPointArrayFn_in;
	status = dataPointArrayFn_in.setObject(pointPos_in);
	if (status != MS::kSuccess)
	{
		MGlobal::displayError("pclRaycaster: failed to read input points");
		dataBlock.setClean(plug);
		return MS::kFailure;
	}
	inPosAry = dataPointArrayFn_in.array();

	// Get colors from pclLoader input (if connected), otherwise default to gray.
	if (pointsColorPlug.isConnected())
	{
		MObject pointCol_in = dataBlock.inputValue(inPointsColor_Attr).data();
		MFnVectorArrayData dataVectorArrayFn_in;
		status = dataVectorArrayFn_in.setObject(pointCol_in);
		if (status == MS::kSuccess)
		{
			inColAry = dataVectorArrayFn_in.array();
		}
		else
		{
			// Default to gray if color read fails.
			const unsigned int pointCount = inPosAry.length();
			inColAry.setLength(pointCount);
			for (unsigned int i = 0; i < pointCount; ++i)
			{
				inColAry.set(MVector(0.5f, 0.5f, 0.5f), i);
			}
		}
	}
	else
	{
		// Default to gray if no color input.
		const unsigned int pointCount = inPosAry.length();
		inColAry.setLength(pointCount);
		for (unsigned int i = 0; i < pointCount; ++i)
		{
			inColAry.set(MVector(0.5f, 0.5f, 0.5f), i);
		}
	}

	numberOfPointsforFrame = inPosAry.length();

	// Clear output arrays.
	outPosAry.clear();
	outColAry.clear();
	pointsInContainer.clear();
	pointsInRaycastArray.clear();

	// If raycasting/frustum is disabled, output all points without filtering.
	if (ctx.raycastOnOff == 0)
	{
		outPosAry.copy(inPosAry);
		outColAry.copy(inColAry);
		numberOfPointsforFrame = outPosAry.length();
	}

	// Gather camera data for raycasting or frustum culling operations.
	MDagPath m_camDagPath;
	if (ctx.raycastOnOff == 1 || ctx.raycastOnOff == 2)
	{
		// Find the camera connected to the cameraMatrix attribute.
		MPlug cam = MPlug(thisNode, cameraMatrix_Attr);
		MPlugArray mPlugArray = MPlugArray();
		cam.connectedTo(mPlugArray, true, true);

		// Validate camera connection to prevent crashes.
		if (mPlugArray.length() == 0)
		{
			return MS::kInvalidParameter;
		}

		// Extract camera DAG path from the connection.
		MObject mObj = mPlugArray[0].node();
		MFnDagNode dagN(mObj);
		MString cameraStringName = dagN.fullPathName();
		MSelectionList list;
		list.add(cameraStringName);
		list.getDagPath(0, m_camDagPath);

		// Check if camera has override color enabled for display customization.
		MFnDependencyNode mFnDependNode(mObj);
		MPlug overrideEnabled = mFnDependNode.findPlug("overrideEnabled");
		if (overrideEnabled.asInt() == 1)
		{
			MPlug overrideColor_plug = mFnDependNode.findPlug("overrideColor");
			overrideColor = overrideColor_plug.asInt();
		}
		else
		{
			overrideColor = -1;
		}
	}

	// Perform ray casting from 2D screen coordinates into the 3D point cloud.
	// The ray is defined by near and far clipping plane positions, then we transform
	// all points into the ray's local coordinate system for efficient containment tests.
	if (ctx.raycastOnOff == 1)
	{
		pointsInRaycastArray.clear();

		// Convert 2D screen coordinates (raycastInputX/Y) to 3D world positions
		// at the near and far clipping planes.
		MVectorArray return3DPos = getCameraData(cameraMatrix_Attr, m_camDagPath, ctx.raycastInputX, ctx.raycastInputY);
		MVector nearPosition = return3DPos[0];
		MVector farPosition = return3DPos[1];

		// Compute the ray direction vector (from near to far, normalized).
		MVector rayDir = MVector(nearPosition - farPosition);
		rayDir.normalize();

		// Get camera function set for accessing camera properties.
		MFnCamera mFnCamera(m_camDagPath);

		// Build an orthonormal basis from the ray direction and camera up vector.
		// This creates a local coordinate system aligned with the ray for efficient point filtering.
		MVector upCamDirection = mFnCamera.upDirection(MSpace::kWorld);

		// Construct right (vecU3) and up (vecU4) vectors perpendicular to the ray direction.
		// If ray direction is parallel to camera up, fall back to a default right vector.
		MVector	vecU3 = rayDir ^ upCamDirection;
		if (vecU3.length() < 1e-6)
		{
			// Ray is parallel to camera up, use world X-axis as fallback.
			MVector worldX(1, 0, 0);
			vecU3 = rayDir ^ worldX;
			if (vecU3.length() < 1e-6)
			{
				// Still parallel, use world Y-axis.
				MVector worldY(0, 1, 0);
				vecU3 = rayDir ^ worldY;
			}
		}
		vecU3.normalize();
		MVector	vecU4 = vecU3 ^ rayDir;
		vecU4.normalize();

		// Build a transformation matrix that aligns the Z-axis with the ray direction
		// and places the origin at the near clipping plane position.
		float matrixList[4][4];
		matrixList[0][0] = vecU3.x;
		matrixList[0][1] = vecU3.y;
		matrixList[0][2] = vecU3.z;
		matrixList[0][3] = 0.0;

		matrixList[1][0] = vecU4.x;
		matrixList[1][1] = vecU4.y;
		matrixList[1][2] = vecU4.z;
		matrixList[1][3] = 0.0;

		matrixList[2][0] = rayDir.x;
		matrixList[2][1] = rayDir.y;
		matrixList[2][2] = rayDir.z;
		matrixList[2][3] = 0.0;

		matrixList[3][0] = nearPosition.x;
		matrixList[3][1] = nearPosition.y;
		matrixList[3][2] = nearPosition.z;
		matrixList[3][3] = 1;

		// Create transformation matrix and compute its inverse.
		MMatrix newTempMatrix(matrixList);
		MTransformationMatrix newTempTransformationMatrix = MTransformationMatrix(newTempMatrix);
		MVector newTempPos = newTempTransformationMatrix.translation(MSpace::kWorld);

		MMatrix newTempMatrixInvers = newTempMatrix.inverse();
		MTransformationMatrix newTempTransformationMatrixInvers = MTransformationMatrix(newTempMatrixInvers);
		MVector newTempPosInv = newTempTransformationMatrixInvers.translation(MSpace::kWorld);

		// Transform all point positions into the ray's local coordinate system.
		// This allows us to use a simple axis-aligned bounding box test along the ray.
		const unsigned int pointCount = inPosAry.length();
		MPointArray inversPos;
		inversPos.setLength(pointCount);
		for (unsigned int i = 0; i < pointCount; ++i)
		{
			// Transform each point from world space to ray-local space.
			MPoint worldPos = (MVector(inPosAry[i]) * newTempMatrixInvers) + newTempPosInv;
			inversPos.set(worldPos, i);
		}

		// Transform the ray origin (near position) into the same local space.
		MVector newNearPosition = (nearPosition  * newTempMatrixInvers) + newTempPosInv;
		
		// Build a bounding box along the ray axis (Z in local space) with radius tolerance.
		// The box extends from the near plane to the far clipping plane, with minRadius padding.
		double farClipPlane = mFnCamera.farClippingPlane();
		MVector minOffset(minRadius, minRadius, minRadius);
		MVector maxOffset(minRadius, minRadius, minRadius + farClipPlane);

		MPoint near_corner(newNearPosition + minOffset);
		MPoint far_corner(newNearPosition - maxOffset);
		the_bbox.expand(near_corner);
		the_bbox.expand(far_corner);

		// Filter points to find those within the ray's bounding box.
		PointsInOutContainer pointsInOutContainer = checkIfPointsInContaner(inversPos, inColAry, the_bbox);

		// Extract the closest hit point and transform it back to world space.
		if (pointsInOutContainer.closestPointHit.length() == 1)
		{
			closestPointHit = (pointsInOutContainer.closestPointHit[0] * newTempMatrix) + newTempPos;
		}
		else
		{
			// No hit point found, set to origin.
			closestPointHit = MPoint(0, 0, 0);
		}

		// Transform filtered points back to world space and populate outputs based on display mode.
		// Display modes: 0 = no display, 1 = show points inside raycast, 2 = show points outside raycast.
		if (ctx.displayType == 0 || ctx.displayType == 1)
		{
			// Transform points inside the raycast bounding box back to world space.
			const unsigned int pointsInCount = pointsInOutContainer.pointsInContainer.length();
			pointsInRaycastArray.setLength(pointsInCount);
			for (unsigned int i = 0; i < pointsInCount; ++i)
			{
				MPoint pointsInRaycast = (MVector(pointsInOutContainer.pointsInContainer[i]) * newTempMatrix) + newTempPos;
				pointsInRaycastArray.set(pointsInRaycast, i);
			}
			numberOfPointsforFrame = pointsInCount;
		}

		// Display mode 2: show points outside the raycast (useful for debugging or masking).
		if (ctx.displayType == 2)
		{
			// Store points inside raycast for reference.
			const unsigned int pointsInCount = pointsInOutContainer.pointsInContainer.length();
			pointsInRaycastArray.setLength(pointsInCount);
			for (unsigned int i = 0; i < pointsInCount; ++i)
			{
				MPoint pointsInRaycast = (MVector(pointsInOutContainer.pointsInContainer[i]) * newTempMatrix) + newTempPos;
				pointsInRaycastArray.set(pointsInRaycast, i);
			}

			// Transform and output points outside the raycast bounding box.
			const unsigned int pointsOutCount = pointsInOutContainer.pointsOutContainer.length();
			outPosAry.setLength(pointsOutCount);
			for (unsigned int i = 0; i < pointsOutCount; ++i)
			{
				MPoint pointsOutRaycast = (MVector(pointsInOutContainer.pointsOutContainer[i]) * newTempMatrix) + newTempPos;
				outPosAry.set(pointsOutRaycast, i);
			}
			outColAry.copy(pointsInOutContainer.pointsColorOutContainer);
			numberOfPointsforFrame = pointsOutCount;
		}
	}

	// Filter points based on whether they lie within the camera's viewing frustum.
	// This is useful for culling points outside the visible field of view.
	if (ctx.raycastOnOff == 2)
	{
		pointsInRaycastArray.clear();

		// Extract camera projection matrix and compute inverse world matrix
		// to transform points into camera space for frustum testing.
		MFnCamera mFnCamera(m_camDagPath);
		MFloatMatrix projectionFloatMatrix = mFnCamera.projectionMatrix();
		MMatrix projectionMatrix = MMatrix(projectionFloatMatrix.matrix);

		MMatrix camMatrixInverse = ctx.cameraMatrix.inverse();

		// Test all points against the camera frustum and separate into inside/outside groups.
		PointsInOutContainer pointsInOutContainer = getPointsInFrustum(inPosAry, inColAry, camMatrixInverse, projectionMatrix);

		if (pointsInOutContainer.pointsInContainer.length() > 0)
		{
			outPosAry.copy(pointsInOutContainer.pointsInContainer);
			outColAry.copy(pointsInOutContainer.pointsColorInContainer);
			numberOfPointsforFrame = pointsInOutContainer.pointsInContainer.length();
		}

		// Display mode 2: separate points inside and outside frustum with different colors.
		if (ctx.displayType == 2)
		{
			// Store points inside frustum for reference.
			const unsigned int pointsInCount = pointsInOutContainer.pointsInContainer.length();
			pointsInRaycastArray.setLength(pointsInCount);
			for (unsigned int i = 0; i < pointsInCount; ++i)
			{
				MPoint pointsInRaycast = (MVector(pointsInOutContainer.pointsInContainer[i]));
				pointsInRaycastArray.set(pointsInRaycast, i);
			}

			// Output points outside frustum.
			const unsigned int pointsOutCount = pointsInOutContainer.pointsOutContainer.length();
			outPosAry.setLength(pointsOutCount);
			for (unsigned int i = 0; i < pointsOutCount; ++i)
			{
				MPoint pointsOutRaycast = (MVector(pointsInOutContainer.pointsOutContainer[i]));
				outPosAry.set(pointsOutRaycast, i);
			}
			outColAry.copy(pointsInOutContainer.pointsColorOutContainer);
			numberOfPointsforFrame = pointsOutCount;
		}
	}

	// Write output attribute values.
	pointNumber_h.setInt(numberOfPointsforFrame);
	pointNumber_h.setClean();
	closestPointHit_h.set3Float(closestPointHit.x, closestPointHit.y, closestPointHit.z);
	closestPointHit_h.setClean();

	// Clear outputs if display is disabled.
	if (ctx.displayType == 0)
	{
		outPosAry.clear();
		outColAry.clear();
		pointsInRaycastArray.clear();
	}

	// Write point position arrays to output attributes.
	if (ctx.displayType != 0)
	{
		status = MS::kSuccess;
		MFnPointArrayData dataPointArrayFn;
		MPointArray outPointsPosAry;

		MObject posD = dataBlock.outputValue(outPosition_Attr).data();
		status = dataPointArrayFn.setObject(posD);

		if (status == MS::kSuccess)
		{
			outPointsPosAry = dataPointArrayFn.array();
		}

		outPointsPosAry.setLength(outPosAry.length());
		dataPointArrayFn.set(outPosAry);

		// Write raycast point positions to output attribute.
		status = MS::kSuccess;
		MFnPointArrayData dataPointArray_inRaycastFn;
		MPointArray outPointsPosAry_inRaycast;

		posD = dataBlock.outputValue(outPositionRaycast_Attr).data();
		status = dataPointArray_inRaycastFn.setObject(posD);

		if (status == MS::kSuccess)
		{
			outPointsPosAry_inRaycast = dataPointArray_inRaycastFn.array();
		}

		outPointsPosAry_inRaycast.setLength(pointsInRaycastArray.length());
		dataPointArray_inRaycastFn.set(pointsInRaycastArray);
	}


	return status;
}


/// Converts 2D screen coordinates to 3D world positions at the near and far clipping planes.
/// This is used to define a ray for point cloud intersection testing.
/// @param cameraMatrix_Attr The camera matrix attribute (unused, kept for API compatibility).
/// @param m_camDagPath The camera's DAG path.
/// @param raycastInputX The X screen coordinate (0 to resolution width).
/// @param raycastInputY The Y screen coordinate (0 to resolution height).
/// @return A vector array containing [nearPosition, farPosition] in world space.
MVectorArray PclRaycaster::getCameraData(MObject /*cameraMatrix_Attr*/, MDagPath m_camDagPath, int raycastInputX, int raycastInputY)
{
	MMatrix camMatrix;
	MMatrix projectionMatrix;
	MVectorArray return3DPos;
	double resolutionX, resolutionY, horizontalFilmOffset, verticalFilmOffset, preScale;
	double horizontalAperture, verticalAperture, nearClipPlane, farClipPlane, focalLength, left, right, bottom, top, windowAspect;

	// Query render resolution from Maya's default resolution settings.
	if (!getRenderResolution(resolutionX, resolutionY))
	{
		MGlobal::displayError("pclRaycaster: failed to get render resolution");
		resolutionX = 1920.0;
		resolutionY = 1080.0;
	}
	// Extract camera properties and matrices.
	MFnCamera mFnCamera(m_camDagPath);
	MFloatMatrix projectionFloatMatrix = mFnCamera.projectionMatrix();
	projectionMatrix = MMatrix(projectionFloatMatrix.matrix);
	camMatrix = m_camDagPath.inclusiveMatrix();

	// Get camera film and lens properties.
	horizontalAperture = mFnCamera.horizontalFilmAperture();
	verticalAperture = mFnCamera.verticalFilmAperture();
	focalLength = mFnCamera.focalLength();
	windowAspect = horizontalAperture / verticalAperture;

	// Compute viewing frustum bounds and clip planes.
	mFnCamera.getViewingFrustum(windowAspect, left, right, bottom, top);
	nearClipPlane = mFnCamera.nearClippingPlane();
	farClipPlane = mFnCamera.farClippingPlane();
	preScale = mFnCamera.preScale();

	// Calculate frustum corner positions at near and far planes.
	MFloatVectorArray horizontalVertical = get3dPoint(nearClipPlane, farClipPlane, left, right, bottom, top);

	// Compute far clipping plane position by interpolating screen coordinates across frustum.
	MVector farTopLeft = horizontalVertical[0];
	MVector farTopRight = horizontalVertical[1];
	MVector farLowLeft = horizontalVertical[2];

	double farTopLeft_x = farTopLeft.x;
	double farTopRight_x = farTopRight.x;
	double farTopLeft_y = farTopLeft.y;
	double farLowLeft_y = farLowLeft.y;

	// Interpolate X coordinate across frustum width based on screen X position.
	double distXA = farTopRight_x - farTopLeft_x;
	double distXB = distXA / preScale;
	double distXC = (distXA - distXB) / 2;
	double output_x = (farTopLeft_x + distXC) + (((farTopRight_x - distXC) - (farTopLeft_x + distXC)) / resolutionX) * raycastInputX;

	// Interpolate Y coordinate across frustum height based on screen Y position.
	double distYA = farLowLeft_y - farTopLeft_y;
	double distYB = distYA / preScale;
	double distYC = (distYA - distYB) / 2;
	double output_y = (farTopLeft_y + distYC) + (((farLowLeft_y - distYC) - (farTopLeft_y + distYC)) / resolutionY) * raycastInputY;

	// Set Z coordinate to far clipping plane depth.
	double dist = farClipPlane * -1;
	MVector farPosition = MVector(MPoint(output_x, output_y, dist) * camMatrix);

	// Compute near clipping plane position using the same interpolation method.
	MVector nearTopLeft = horizontalVertical[3];
	MVector nearTopRight = horizontalVertical[4];
	MVector nearLowLeft = horizontalVertical[5];

	double nearTopLeft_x = nearTopLeft.x;
	double nearTopRight_x = nearTopRight.x;
	double nearTopLeft_y = nearTopLeft.y;
	double nearLowLeft_y = nearLowLeft.y;

	// Interpolate X coordinate for near plane.
	distXA = nearTopRight_x - nearTopLeft_x;
	distXB = distXA / preScale;
	distXC = (distXA - distXB) / 2;
	output_x = (nearTopLeft_x + distXC) + (((nearTopRight_x - distXC) - (nearTopLeft_x + distXC)) / resolutionX) * raycastInputX;

	// Interpolate Y coordinate for near plane.
	distYA = nearLowLeft_y - nearTopLeft_y;
	distYB = distYA / preScale;
	distYC = (distYA - distYB) / 2;
	output_y = (nearTopLeft_y + distYC) + (((nearLowLeft_y - distYC) - (nearTopLeft_y + distYC)) / resolutionY) * raycastInputY;

	// Set Z coordinate to near clipping plane depth.
	dist = nearClipPlane * -1;
	MVector nearPosition = MVector(MPoint(output_x, output_y, dist) * camMatrix);

	// Return both positions as a vector array [near, far].
	return3DPos.append(nearPosition);
	return3DPos.append(farPosition);

	return return3DPos;

}

/// Calculates the 3D positions of frustum corners at the near and far clipping planes.
/// Used internally by getCameraData to compute the viewing frustum geometry.
/// @param nearClipPlane Distance to the near clipping plane.
/// @param farClipPlane Distance to the far clipping plane.
/// @param left, right, bottom, top Frustum bounds in camera space.
/// @return An array of corner positions: [farTopLeft, farTopRight, farLowLeft, nearTopLeft, nearTopRight, nearLowLeft].
MFloatVectorArray PclRaycaster::get3dPoint(double nearClipPlane, double farClipPlane, double left, double right, double bottom, double top)
{
	MFloatVectorArray horizontal_vertical;

	left = left / nearClipPlane;
	right = right / nearClipPlane;
	bottom = bottom / nearClipPlane;
	top = top / nearClipPlane;

	double farRight = (farClipPlane * right);
	double farTop = (farClipPlane * top);
	double nearRight = (nearClipPlane * right);
	double nearTop = (nearClipPlane * top);

	double farLeft = (farClipPlane * left);
	double farBottom = (farClipPlane * bottom);
	double nearLeft = (nearClipPlane * left);
	double nearBottom = (nearClipPlane * bottom);

	// Compute corner positions at the far clipping plane.
	MVector farTopLeft(farLeft, farTop, -farClipPlane);
	MVector farTopRight(farRight, farTop, -farClipPlane);
	MVector farLowLeft(farLeft, farBottom, -farClipPlane);
	MVector farLowRight(farRight, farBottom, -farClipPlane);

	// Compute corner positions at the near clipping plane.
	MVector nearTopLeft(nearLeft, nearTop, -nearClipPlane);
	MVector nearTopRight(nearRight, nearTop, -nearClipPlane);
	MVector nearLowLeft(nearLeft, nearBottom, -nearClipPlane);
	MVector nearLowRight(nearRight, nearBottom, -nearClipPlane);

	horizontal_vertical.append(farTopLeft);
	horizontal_vertical.append(farTopRight);
	horizontal_vertical.append(farLowLeft);

	horizontal_vertical.append(nearTopLeft);
	horizontal_vertical.append(nearTopRight);
	horizontal_vertical.append(nearLowLeft);


	return horizontal_vertical;
}


/// Tests which points lie inside or outside a given bounding box.
/// Also finds the closest point to the camera (highest Z value in local space).
/// @param inPosAry Input point positions (in local coordinate system).
/// @param inColAry Input point colors.
/// @param the_bbox The bounding box to test against.
/// @return A structure containing separated points and colors for inside/outside, plus the closest hit point.
PointsInOutContainer PclRaycaster::checkIfPointsInContaner(MPointArray inPosAry, MVectorArray inColAry, MBoundingBox the_bbox)
{
	PointsInOutContainer pointsInOutContainer;
	MPointArray pointsInContainer;
	MPointArray pointsOutContainer;
	MVectorArray pointsColorInContainer;
	MVectorArray pointsColorOutContainer;
	MVectorArray closestPointHit;
	
	// Track the closest point along the ray (highest Z in local space).
	float closestDistHit = -10000;

	// Test each point against the bounding box and separate into inside/outside groups.
	const unsigned int pointCount = inPosAry.length();
	for (unsigned int i = 0; i < pointCount; i++)
	{
		bool contain = the_bbox.contains(inPosAry[i]);
		if (contain == true)
		{
			pointsInContainer.append(inPosAry[i]);
			pointsColorInContainer.append(inColAry[i]);

			// Update closest hit point (points with higher Z are closer to the camera in local space).
			if (inPosAry[i].z > closestDistHit)
			{
				closestPointHit.clear();
				closestDistHit = inPosAry[i].z;
				closestPointHit.append(inPosAry[i]);
			}
		}
		else
		{
			pointsOutContainer.append(inPosAry[i]);
			pointsColorOutContainer.append(inColAry[i]);
		}
	}

	// Populate the return structure with separated points, colors, and closest hit.
	pointsInOutContainer.pointsInContainer = pointsInContainer;
	pointsInOutContainer.pointsOutContainer = pointsOutContainer;
	pointsInOutContainer.pointsColorInContainer = pointsColorInContainer;
	pointsInOutContainer.pointsColorOutContainer = pointsColorOutContainer;
	pointsInOutContainer.closestPointHit = closestPointHit;

	return pointsInOutContainer;
}


/// Tests which points lie within the camera's viewing frustum by projecting them to screen space.
/// Points that project within the render resolution bounds are considered visible.
/// @param inPosAry Input point positions in world space.
/// @param inColAry Input point colors.
/// @param camMatrixInverse The inverse of the camera's world matrix.
/// @param projectionMatrix The camera's projection matrix.
/// @return A structure containing separated points and colors for inside/outside the frustum.
PointsInOutContainer PclRaycaster::getPointsInFrustum(MPointArray inPosAry, MVectorArray inColAry, MMatrix camMatrixInverse, MMatrix projectionMatrix)
{
	PointsInOutContainer pointsInOutContainer;
	MPointArray pointsInContainer;
	MPointArray pointsOutContainer;
	MVectorArray pointsColorInContainer;

	// Query render resolution from Maya's default resolution settings.
	double resolutionX;
	double resolutionY;
	unsigned int ssX;
	unsigned int ssY;

	if (!getRenderResolution(resolutionX, resolutionY))
	{
		MGlobal::displayError("pclRaycaster: failed to get render resolution");
		resolutionX = 1920.0;
		resolutionY = 1080.0;
	}

	// Pre-compute view-projection matrix once (outside the loop for performance).
	MMatrix viewProjMatrix = camMatrixInverse * projectionMatrix;

	// Project each point to screen space and test if it lies within the viewport bounds.
	const unsigned int pointCount = inPosAry.length();
	for (unsigned int i = 0; i < pointCount; ++i)
	{
		// Transform point to clip space, then to normalized device coordinates (NDC).
		MPoint multPoint = inPosAry[i] * viewProjMatrix;

		// Convert from NDC [-1,1] to screen coordinates [0, resolution].
		// Check for division by zero (w component should not be zero for valid points).
		if (std::abs(multPoint[3]) > 1e-6)
		{
			ssX = static_cast<unsigned int>((multPoint[0] / multPoint[3] * 0.5 + 0.5) * resolutionX);
			ssY = static_cast<unsigned int>((multPoint[1] / multPoint[3] * 0.5 + 0.5) * resolutionY);
		}
		else
		{
			// Invalid point (behind camera or at infinity), mark as outside frustum.
			ssX = static_cast<unsigned int>(resolutionX + 1);
			ssY = static_cast<unsigned int>(resolutionY + 1);
		}

		// If the projected point lies within the screen bounds, it's inside the frustum.
		// Note: screen coordinates are typically 0 to width-1, but we use <= for safety.
		if (ssX < static_cast<unsigned int>(resolutionX) && ssY < static_cast<unsigned int>(resolutionY))
		{
			pointsInContainer.append(inPosAry[i]);
			pointsColorInContainer.append(inColAry[i]);
		}
		else
		{
			pointsOutContainer.append(inPosAry[i]);
		}
	}

	// Populate the return structure with separated points and colors.
	pointsInOutContainer.pointsInContainer = pointsInContainer;
	pointsInOutContainer.pointsOutContainer = pointsOutContainer;
	pointsInOutContainer.pointsColorInContainer = pointsColorInContainer;

	return pointsInOutContainer;

}

/// Draws point cloud data in Viewport 1.0 using OpenGL.
/// Supports different display modes: show all points, show only raycast hits, or show points outside raycast.
void PclRaycaster::draw(M3dView& view, const MDagPath& /*DGpath*/, M3dView::DisplayStyle /*style*/, M3dView::DisplayStatus /*status*/)
{
	MObject thisNode = thisMObject();
	MFnDagNode dagThis(thisNode);

	short action = dagThis.findPlug("action").asShort();
	short displayType = dagThis.findPlug("displayType").asShort();

	// Set up OpenGL state for transparent point rendering.
	MColor pointColor(0, 0.2, 0.9);
	MColor pointCol;
	view.beginGL();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glClearDepth(1.0);
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthMask(GL_FALSE);

	if (displayType != 0)
		if (action == 0 || action == 1 || action == 2)
		{
			if (overrideColor == -1)
			{
				pointCol = pointColor;
			}
			else
			{
				pointColor = M3dView().colorAtIndex(overrideColor - 1);
			}

			// Draw all output points, using per-point colors if available, otherwise use default color.
			for (int i = 0; i < outPosAry.length(); ++i)
			{
				if (overrideColor == -1)
				{
					if (outColAry.length() > 0)
					{
						pointCol = MColor(outColAry[i].x, outColAry[i].y, outColAry[i].z);
					}
					else
					{
						pointCol = pointColor;
					}
				}
				else
				{
					pointCol = pointColor;
				}

				drawPoints(outPosAry[i], pointCol, pointsSize);
			}
		}


	// Draw raycast-specific visualization when action mode is raycasting.
	// Points inside the raycast are drawn in red, points outside use their original colors.
	if (action == 1)
	{
		MColor pointColoInRay(1, 0, 0); // Red color for raycast hit points.
		// Display only points inside the raycast (displayType == 1).
		if (displayType == 1)
		{
			// Draw all raycast hit points in red.
			for (int i = 0; i < pointsInRaycastArray.length(); ++i)
			{
				drawPoints(pointsInRaycastArray[i], pointColoInRay, pointsSize);
			}
		}

		// Display both points inside and outside the raycast with different colors (displayType == 2).
		if (displayType == 2)
		{
			// Draw points inside raycast in red.
			for (int i = 0; i < pointsInRaycastArray.length(); ++i)
			{
				drawPoints(pointsInRaycastArray[i], pointColoInRay, pointsSize);
			}

			// Draw points outside raycast with their original colors.
			for (int i = 0; i < outPosAry.length(); ++i)
			{
				if (overrideColor == -1)
				{
					if (outColAry.length() >0)
					{
						pointCol = MColor(outColAry[i].x, outColAry[i].y, outColAry[i].z);
					}
					else
					{
						pointCol = pointColor;
					}
				}
				else
				{
					pointCol = pointColor;
				}

				drawPoints(outPosAry[i], pointCol, pointsSize);
				}
			}
		}

		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
		glPopAttrib();
		view.endGL();

}

/// Draws a single point in Viewport 1.0 using OpenGL immediate mode.
/// @param pointPos The 3D position of the point.
/// @param pointCol The color to draw the point.
/// @param pointsSize The size of the point in pixels.
void PclRaycaster::drawPoints(MFloatPoint pointPos, MColor pointCol, float pointsSize)
{
	glColor4f(pointCol.r, pointCol.g, pointCol.b, 1);
	glPointSize(pointsSize);
	glDisable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	glVertex3f(pointPos.x, pointPos.y, pointPos.z);
	glEnd();
	glPopAttrib();
}

/// Declares which outputs are dirtied by a given input.
MStatus PclRaycaster::setDependentsDirty(const MPlug &plugBeingDirtied, MPlugArray &affectedPlugs)
{
	MStatus status;
	MObject thisNode = thisMObject();
	MFnDependencyNode fnThisNode(thisNode);

	// Declare which output plugs are affected when input plugs change.
	// This ensures Maya's dependency graph correctly invalidates downstream nodes.
	if (plugBeingDirtied.partialName() == "createPoints") {
		MPlug PointHit(thisNode, PclRaycaster::closestPointHit_Attr);
		affectedPlugs.append(PointHit);
		MPlug PointNum(thisNode, PclRaycaster::pointNumber_Attr);
		affectedPlugs.append(PointNum);
		MPlug oP(thisNode, PclRaycaster::outPosition_Attr);
		affectedPlugs.append(oP);
	}

	if (plugBeingDirtied.partialName() == "fromCamera") {
		MPlug PointHit(thisNode, PclRaycaster::closestPointHit_Attr);
		affectedPlugs.append(PointHit);
		MPlug PointNum(thisNode, PclRaycaster::pointNumber_Attr);
		affectedPlugs.append(PointNum);
		MPlug oP(thisNode, PclRaycaster::outPosition_Attr);
		affectedPlugs.append(oP);
	}

	if (plugBeingDirtied.partialName() == "cameraMatrix") {
		MPlug PointHit(thisNode, PclRaycaster::closestPointHit_Attr);
		affectedPlugs.append(PointHit);
		MPlug PointNum(thisNode, PclRaycaster::pointNumber_Attr);
		affectedPlugs.append(PointNum);
		MPlug oP(thisNode, PclRaycaster::outPosition_Attr);
		affectedPlugs.append(oP);
	}

	if (plugBeingDirtied.partialName() == "raycastInputX") {
		MPlug PointHit(thisNode, PclRaycaster::closestPointHit_Attr);
		affectedPlugs.append(PointHit);
		MPlug PointNum(thisNode, PclRaycaster::pointNumber_Attr);
		affectedPlugs.append(PointNum);
		MPlug oP(thisNode, PclRaycaster::outPosition_Attr);
		affectedPlugs.append(oP);
	}

	if (plugBeingDirtied.partialName() == "raycastInputY") {
		MPlug PointHit(thisNode, PclRaycaster::closestPointHit_Attr);
		affectedPlugs.append(PointHit);
		MPlug PointNum(thisNode, PclRaycaster::pointNumber_Attr);
		affectedPlugs.append(PointNum);
		MPlug oP(thisNode, PclRaycaster::outPosition_Attr);
		affectedPlugs.append(oP);
	}
	
	if (plugBeingDirtied.partialName() == "minRadius") {
		MPlug PointHit(thisNode, PclRaycaster::closestPointHit_Attr);
		affectedPlugs.append(PointHit);
		MPlug PointNum(thisNode, PclRaycaster::pointNumber_Attr);
		affectedPlugs.append(PointNum);
		MPlug oP(thisNode, PclRaycaster::outPosition_Attr);
		affectedPlugs.append(oP);
	}
	
	return MS::kSuccess;
}

/// Defines all node attributes and their properties.
MStatus PclRaycaster::initialize()
{
	MStatus status = MS::kSuccess;

	// In addition to the standard arrayMapper attributes,
	// create an additional vector attribute.
	//
	MFnTypedAttribute typedAttrFn;
	MFnNumericAttribute numAttrFn;
	MFnMessageAttribute msgAttr;
	MFnEnumAttribute eAttr;
	MFnMatrixAttribute mFnMatrixAtt;
	MVectorArray defaultVectArray;
	MFnVectorArrayData vectArrayDataFn;
	MPointArray defaultPotArray;
	MFnPointArrayData pointArrayDataFn;


	// Input points from pclLoader (required - node receives data via dependency graph).
	pointArrayDataFn.create(defaultPotArray);
	inPointsPosition_Attr = typedAttrFn.create("inPointsPosition", "inPointsPosition", MFnData::kPointArray, pointArrayDataFn.object(), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	typedAttrFn.setWritable(true);
	typedAttrFn.setStorable(true);
	typedAttrFn.setKeyable(false);
	typedAttrFn.setConnectable(true);
	addAttribute(inPointsPosition_Attr);

	// Input colors from pclLoader (optional).
	vectArrayDataFn.create(defaultVectArray);
	inPointsColor_Attr = typedAttrFn.create("inPointsColor", "inPointsColor", MFnData::kVectorArray, vectArrayDataFn.object(), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	typedAttrFn.setWritable(true);
	typedAttrFn.setStorable(true);
	typedAttrFn.setKeyable(false);
	typedAttrFn.setConnectable(true);
	addAttribute(inPointsColor_Attr);

	// Input time value (typically connected to Maya's time).
	inTime_Attr = numAttrFn.create("inTime", "inTime", MFnNumericData::kFloat, 0.0);
	addAttribute(inTime_Attr);

	// Time offset for frame selection in sequences.
	timeOffset_Attr = numAttrFn.create("timeOffset", "timeOffset", MFnNumericData::kFloat, 0);
	addAttribute(timeOffset_Attr);

	// Output: number of points in the current frame.
	pointNumber_Attr = numAttrFn.create("numberOfPoints", "numberOfPoints", MFnNumericData::kInt);
	numAttrFn.setKeyable(true);
	numAttrFn.setStorable(false);
	numAttrFn.setWritable(true);
	numAttrFn.setChannelBox(true);
	numAttrFn.setConnectable(true);
	addAttribute(pointNumber_Attr);

	// Raycast action mode: off (0), raycasting (1), or frustum culling (2).
	raycastOnOff_Attr = eAttr.create("action", "action");
	eAttr.addField("off", 0);
	eAttr.addField("in Raycasting", 1);
	eAttr.addField("in Frustum", 2);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(raycastOnOff_Attr);

	// Display type: off (0), only raycast hits (1), or all points with colors (2).
	displayType_Attr = eAttr.create("displayType", "displayType", 2);
	eAttr.addField("display Off", 0);
	eAttr.addField("display Only in Raycasting", 1);
	eAttr.addField("display all Points Different Colors", 2);
	eAttr.setDefault(2);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(displayType_Attr);

	// Screen X coordinate for raycasting (1 to resolution width).
	raycastInputX_Attr = numAttrFn.create("raycastInputX", "raycastInputX", MFnNumericData::kInt, 1);
	numAttrFn.setMin(1);
	numAttrFn.setKeyable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setConnectable(true);
	addAttribute(raycastInputX_Attr);

	// Screen Y coordinate for raycasting (1 to resolution height).
	raycastInputY_Attr = numAttrFn.create("raycastInputY", "raycastInputY", MFnNumericData::kInt, 1);
	numAttrFn.setMin(1);
	numAttrFn.setKeyable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setConnectable(true);
	addAttribute(raycastInputY_Attr);

	// Minimum search radius for point filtering (in world units).
	minRadius_Attr = numAttrFn.create("minRadius", "minRadius", MFnNumericData::kFloat, 1);
	numAttrFn.setMin(0.01);
	numAttrFn.setKeyable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setConnectable(true);
	addAttribute(minRadius_Attr);

	// Point size for viewport display (in pixels).
	pointsSize_Attr = numAttrFn.create("pointsSize", "pointsSize", MFnNumericData::kFloat, 1);
	numAttrFn.setMin(0.0);
	numAttrFn.setKeyable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setConnectable(true);
	addAttribute(pointsSize_Attr);

	// Output: all point positions (filtered based on display type and action mode).
	pointArrayDataFn.create(defaultPotArray);
	typedAttrFn.create("outPositionPP", "outPositionPP", MFnData::kPointArray, pointArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outPosition_Attr = typedAttrFn.object();
	addAttribute(outPosition_Attr);

	// Output: point positions from raycast hits only.
	pointArrayDataFn.create(defaultPotArray);
	typedAttrFn.create("outPositionRaycast", "outPositionRaycast", MFnData::kPointArray, pointArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outPositionRaycast_Attr = typedAttrFn.object();
	addAttribute(outPositionRaycast_Attr);

	// Output: closest point hit position from raycasting.
	closestPointHit_Attr = numAttrFn.createPoint("closestPointHit", "closestPointHit");
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(false);
	numAttrFn.setStorable(true);
	numAttrFn.setKeyable(true);
	numAttrFn.setConnectable(true);
	addAttribute(closestPointHit_Attr);

	// Input: camera world matrix (typically connected from a camera node).
	cameraMatrix_Attr = mFnMatrixAtt.create("cameraMatrix", "cameraMatrix");
	mFnMatrixAtt.setReadable(false);
	mFnMatrixAtt.setWritable(true);
	mFnMatrixAtt.setStorable(true);
	mFnMatrixAtt.setKeyable(false);
	mFnMatrixAtt.setConnectable(true);
	addAttribute(cameraMatrix_Attr);

	// Attribute dependencies are handled dynamically in setDependentsDirty().
	return status;

}


//---------------------------------------------------------------------------
// Viewport 2.0 override implementation
//---------------------------------------------------------------------------
PclRaycasterDrawOverride::PclRaycasterDrawOverride(const MObject& obj)
	: MHWRender::MPxDrawOverride(obj, PclRaycasterDrawOverride::draw)
{
}

PclRaycasterDrawOverride::~PclRaycasterDrawOverride()
{
}

MHWRender::DrawAPI PclRaycasterDrawOverride::supportedDrawAPIs() const
{
	// this plugin supports both GL and DX
	return (MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile);
}


/// Retrieves the raycast on/off state from the node.
short PclRaycasterDrawOverride::get_raycastOnOff(const MDagPath& objPath) const
{
	MObject raycasterNode = objPath.node();
	MFnDagNode dagThis(raycasterNode);
	return dagThis.findPlug(PclRaycaster::raycastOnOff_Attr).asShort();
}

/// Retrieves the display type from the node.
short PclRaycasterDrawOverride::get_displayType(const MDagPath& objPath) const
{
	MObject raycasterNode = objPath.node();
	MFnDagNode dagThis(raycasterNode);
	return dagThis.findPlug(PclRaycaster::displayType_Attr).asShort();
}

/// Retrieves the point size from the node.
float PclRaycasterDrawOverride::get_pointsSize(const MDagPath& objPath) const
{
	MStatus status;
	MObject raycasterNode = objPath.node(&status);
	if (status != MS::kSuccess)
	{
		return 0.0f;
	}
	MFnDagNode dagThis(raycasterNode);
	return dagThis.findPlug(PclRaycaster::pointsSize_Attr).asFloat();
}

/// Retrieves the output point positions from the node.
MPointArray PclRaycasterDrawOverride::get_pointPosition(const MDagPath& objPath) const
{
	MPointArray outPosArray;
	MObject raycasterNode = objPath.node();
	MFnDagNode dagThis(raycasterNode);

	MPlug plug_aPoints = dagThis.findPlug(PclRaycaster::outPosition_Attr);
	MObject o_aPoints;
	plug_aPoints.getValue(o_aPoints);
	MFnPointArrayData fn_aPoints(o_aPoints);
	fn_aPoints.copyTo(outPosArray);

	return outPosArray;
}

/// Retrieves the raycast hit point positions from the node.
MPointArray PclRaycasterDrawOverride::get_pointPositionRaycast(const MDagPath& objPath) const
{
	MPointArray outPosArrayRaycast;
	MObject raycasterNode = objPath.node();
	MFnDagNode dagThis(raycasterNode);

	MPlug plug_aPoints = dagThis.findPlug(PclRaycaster::outPositionRaycast_Attr);
	MObject o_aPoints;
	plug_aPoints.getValue(o_aPoints);
	MFnPointArrayData fn_aPoints(o_aPoints);
	fn_aPoints.copyTo(outPosArrayRaycast);

	return outPosArrayRaycast;
}


/// Prepares drawing data for Viewport 2.0 rendering.
/// Called by Maya each time the object needs to be drawn.
MUserData* PclRaycasterDrawOverride::prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath, const MHWRender::MFrameContext& frameContext, MUserData* oldData)
{
	PclRaycasterData* data = dynamic_cast<PclRaycasterData*>(oldData);
	if (!data)
	{
		data = new PclRaycasterData();
	}

	// Gather all drawing data from the node attributes.
	short raycastOnOff = get_raycastOnOff(objPath);
	short displayType = get_displayType(objPath);
	float pointSize = get_pointsSize(objPath);
	MPointArray pointPos = get_pointPosition(objPath);
	MPointArray pointPosRaycast = get_pointPositionRaycast(objPath);

	data->raycastOnOff = raycastOnOff;
	data->displayType = displayType;
	data->pointSize = pointSize;
	data->inPosAryV2 = pointPos;
	data->inPosAryRaycastV2 = pointPosRaycast;
	


	return data;
}


/// Queues UI drawables for Viewport 2.0 rendering.
/// Uses MUIDrawManager to draw points with different colors based on display mode and action type.
void PclRaycasterDrawOverride::addUIDrawables(const MDagPath& objPath, MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext, const MUserData* data)
{
	// Get data cached by prepareForDraw() for each drawable instance.
	PclRaycasterData* drawData = (PclRaycasterData*)data;
	if (!drawData)
	{
		return;
	}

	short raycastOnOff = drawData->raycastOnOff;
	short displayType = drawData->displayType;
	float pointSize = drawData->pointSize;
	MPointArray inPosAryV2 = drawData->inPosAryV2;
	MPointArray inPosAryRaycastV2 = drawData->inPosAryRaycastV2;

	if (displayType != 0)
	{
		// Draw all points when raycasting is off or when using frustum mode.
		if (raycastOnOff == 0 || raycastOnOff == 2)
		{
			drawManager.setPointSize(pointSize);
			if (inPosAryV2.length() > 0)
			{
				MColor color(0, 0.2, 0.9);
				drawManager.setColor(color);
				drawManager.points(inPosAryV2, FALSE);
			}
		}

		// Draw raycast-specific visualization.
		if (raycastOnOff == 1)
		{
			// Display only points inside raycast (displayType == 1).
			if (displayType == 1)
			{
				drawManager.setPointSize(pointSize);
				if (inPosAryRaycastV2.length() > 0)
				{
					MColor color(1.0, 0.0, 0.0);
					drawManager.setColor(color);
					drawManager.points(inPosAryRaycastV2, FALSE);
				}
			}

			// Display points inside and outside raycast with different colors (displayType == 2).
			if (displayType == 2)
			{
				// Draw points outside raycast in blue.
				drawManager.setPointSize(pointSize);
				if (inPosAryV2.length() > 0)
				{
					MColor color(0, 0.2, 0.9);
					drawManager.setColor(color);
					drawManager.points(inPosAryV2, FALSE);
				}
				// Draw points inside raycast in red.
				drawManager.setPointSize(pointSize);
				if (inPosAryRaycastV2.length() > 0)
				{
					MColor color(1.0, 0.0, 0.0);
					drawManager.setColor(color);
					drawManager.points(inPosAryRaycastV2, FALSE);
				}
			}
		}

		// Draw frustum-specific visualization.
		if (raycastOnOff == 2)
		{
			// Display only points inside frustum (displayType == 1).
			if (displayType == 1)
			{
				drawManager.setPointSize(pointSize);
				if (inPosAryRaycastV2.length() > 0)
				{
					MColor color(1.0, 0.0, 0.0);
					drawManager.setColor(color);
					drawManager.points(inPosAryRaycastV2, FALSE);
				}
			}

			// Display points inside and outside frustum with different colors (displayType == 2).
			if (displayType == 2)
			{
				// Draw points inside frustum in blue.
				drawManager.setPointSize(pointSize);
				if (inPosAryV2.length() > 0)
				{
					MColor color(0, 0.2, 0.9);
					drawManager.setColor(color);
					drawManager.points(inPosAryV2, FALSE);
				}

				// Draw points outside frustum in red.
				drawManager.setPointSize(pointSize);
				if (inPosAryRaycastV2.length() > 0)
				{
					MColor color(1.0, 0.0, 0.0);
					drawManager.setColor(color);
					drawManager.points(inPosAryRaycastV2, FALSE);
				}
			}
		}
	}

	drawManager.endDrawable();
}
