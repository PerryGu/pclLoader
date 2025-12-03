#include "pclCluster.h"

// Standard library
#include <algorithm>

// Maya API
#include <maya/MGlobal.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MFnParticleSystem.h>
#include <maya/MFnTransform.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnVectorArrayData.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MAnimControl.h>
#include <maya/MTime.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MSelectionList.h>
#include <maya/M3dView.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MStatus.h>

// Error checking macro
#ifndef CHECK_MSTATUS_AND_RETURN_IT
#define CHECK_MSTATUS_AND_RETURN_IT(status) \
	do { \
		if ((status) != MS::kSuccess) { \
			return (status); \
		} \
	} while (0)
#endif

MTypeId pclCluster::id(0x00000112555);
MString	pclCluster::drawDbClassification("drawdb/geometry/pclCluster_GeometryOverride");
MString	pclCluster::drawRegistrantId("PclContainerNode_GeometryOverridePlugin");

// Attribute definitions
MObject	pclCluster::saperator_Attr;
MObject	pclCluster::pointsSampling_Attr;
MObject	pclCluster::action_Attr;
MObject	pclCluster::globalBoundingBox_Attr;
MObject	pclCluster::recursionDepth_Attr;
MObject	pclCluster::minimumPointCount_Attr;
MObject	pclCluster::boundingBoxVis_Attr;
MObject	pclCluster::clustersGrouping_Attr;
MObject	pclCluster::clusterCenter_Attr;
MObject pclCluster::clusterDrawScale_Attr;
MObject	pclCluster::clustersDetection_Attr;
MObject	pclCluster::createClusterPath_Attr;
MObject	pclCluster::nextFrameSearchRadius_Attr;
MObject	pclCluster::inParticleMatrix_Attr;
MObject	pclCluster::inPointsPosition_Attr;
MObject	pclCluster::inPointsColor_Attr;
MObject	pclCluster::inParticlePP_Attr;
MObject pclCluster::inTime_Attr;

MObject	pclCluster::clusterCenterPos_Attr;

MObject	pclCluster::inBoundingBoxCorner_Attr;
MObject	pclCluster::outDivBoundingBoxDraw_Attr;
MObject	pclCluster::outCrossDraw_Attr;
MObject	pclCluster::outClustersSequencePathDraw_Attr;
MObject pclCluster::out_Attr;
MObject pclCluster::createCurvs_Attr;

/// Default constructor.
pclCluster::pclCluster()
{
}

/// Destructor.
pclCluster::~pclCluster()
{
}

/// Assigns a unique shape name after creation.
void pclCluster::postConstructor()
{
	mThisNode = thisMObject();
	MFnDependencyNode fnNode(mThisNode);
	fnNode.setName("pclClusterShape#");
}

/// Maya factory hook.
void* pclCluster::creator()
{
	return new pclCluster();
}


/// Main evaluation entry point. Performs octree-based clustering on input points.
MStatus pclCluster::compute(const MPlug& plug, MDataBlock& dataBlock)
{
	MStatus status;

	// Gather all frequently-used inputs at the start for clarity.
	EvaluateContext ctx;
	ctx.pointsSampling = dataBlock.inputValue(pointsSampling_Attr).asInt();
	ctx.action = dataBlock.inputValue(action_Attr).asBool();
	ctx.createClusterPath = dataBlock.inputValue(createClusterPath_Attr).asShort();
	ctx.boundingBoxVis = dataBlock.inputValue(boundingBoxVis_Attr).asBool();
	ctx.clusterGrouping = dataBlock.inputValue(clustersGrouping_Attr).asShort();
	ctx.timeVal = dataBlock.inputValue(inTime_Attr).asTime();
	
	// Store member variables from inputs
	mRecursionDepth = dataBlock.inputValue(recursionDepth_Attr).asInt();
	mMinimumPoint_count = dataBlock.inputValue(minimumPointCount_Attr).asInt();
	mClusterCentering = dataBlock.inputValue(clusterCenter_Attr).asBool();
	mClusterDrawScale = dataBlock.inputValue(clusterDrawScale_Attr).asDouble();
	mNextFrameSearchRadius = dataBlock.inputValue(nextFrameSearchRadius_Attr).asDouble();

	// Calculate frame offset
	ctx.frame = (int)ctx.timeVal.as(MTime::kNTSCFrame);
	MAnimControl anmCon;
	MTime startTimeLineM = anmCon.minTime();
	int startTimeLine = (int)startTimeLineM.as(MTime::uiUnit());
	ctx.adjustFrame = ctx.frame - startTimeLine;

	// Get bounding box corners from pclLoader input
	MObject posBbC = dataBlock.inputValue(inBoundingBoxCorner_Attr).data();
	MFnPointArrayData dataPointArrayGlobBBFn;
	status = dataPointArrayGlobBBFn.setObject(posBbC);
	if (status == MS::kSuccess)
	{
		ctx.boundingBoxCorner_list = dataPointArrayGlobBBFn.array();
	}
	
	// Build global bounding box from corner points
	const unsigned int cornerCount = ctx.boundingBoxCorner_list.length();
	if (cornerCount >= 2)
	{
		MBoundingBox container_bbox;
		container_bbox.expand(MPoint(ctx.boundingBoxCorner_list[0].x, ctx.boundingBoxCorner_list[0].y, ctx.boundingBoxCorner_list[0].z));
		container_bbox.expand(MPoint(ctx.boundingBoxCorner_list[1].x, ctx.boundingBoxCorner_list[1].y, ctx.boundingBoxCorner_list[1].z));
		mGlobalBoundingBox = container_bbox;	
	}

	// Get connected particle system if matrix plug is connected
	MPlug particleMatrix_plug = MPlug(mThisNode, inParticleMatrix_Attr);
	ctx.particleMatrixConnected = particleMatrix_plug.isConnected();
	if (ctx.particleMatrixConnected)
	{
		if (mParticle_sys.name() == "")
		{
			status = getConnetdedParticleSys(particleMatrix_plug);
			if (status != MS::kSuccess)
			{
				return MS::kFailure;
			}
			mParticle_sys.setObject(mParticle_path);
		}
	}

	// Clear all cached data when action is disabled
	if (ctx.action == 0) {
		eval_state = true;
		eval_state2 = true;
		eval_state3 = true;
		eval_state4 = true;
		eval_state5 = true;
		mMinWidthBb = 0.5;
		mBoundingBoxIndex = 0;
		mBoundingBoxInfo_list.clear();
		mLastBoundingBox_list.clear();
		mDupBoundingBox_list.clear();
		mBoundingBoxOutputMain_list.clear();
		mBoundingBoxsToDrawInfo.clear();
		mBoundingBoxsToDraw_info.clear();
		mCrossToDraw_Info.clear();
		mClustersSequencePath_list.clear();	
	}

	MPointArray pointsIn_boundingBox;

	// Process points when action is enabled
	if (ctx.action == 1)
	{
		// Get input points array from pclLoader
		MObject pointPos_in = dataBlock.inputValue(inPointsPosition_Attr).data();
		MFnPointArrayData dataPointArrayFn_in;
		status = dataPointArrayFn_in.setObject(pointPos_in);
		MPointArray pointsInSampling_boundingBox;
		if (status == MS::kSuccess)
		{
			pointsInSampling_boundingBox = dataPointArrayFn_in.array();
		}

		// Get input colors array from pclLoader (optional, for consistency with pclRaycaster)
		MPlug pointsColorPlug(mThisNode, inPointsColor_Attr);
		if (pointsColorPlug.isConnected())
		{
			MObject pointCol_in = dataBlock.inputValue(inPointsColor_Attr).data();
			MFnVectorArrayData dataVectorArrayFn_in;
			status = dataVectorArrayFn_in.setObject(pointCol_in);
			// Colors are read but not currently used in clustering logic
			// This maintains consistency with pclRaycaster and allows for future color-based features
		}

		// Apply point sampling if enabled
		const unsigned int inputPointCount = pointsInSampling_boundingBox.length();
		if (inputPointCount > 0) 
		{
			if (ctx.pointsSampling > 0)
			{
				MVectorArray particleInSampling_input;
				pointsIn_boundingBox = getPointSampling(pointsInSampling_boundingBox, particleInSampling_input, ctx.pointsSampling);
			}
			else
			{
				pointsIn_boundingBox = dataPointArrayFn_in.array();
			}
		}

		// Get particle system input if connected
		MObject pointPosPartcle_in = dataBlock.inputValue(inParticlePP_Attr).data();
		MFnVectorArrayData dataVectorArrayFn;
		MVectorArray particleInSampling_input;
		status = dataVectorArrayFn.setObject(pointPosPartcle_in);
		if (status == MS::kSuccess)
		{
			particleInSampling_input = dataVectorArrayFn.array();
		}
		const unsigned int particleCount = particleInSampling_input.length();
		if (particleCount > 0)
		{
			pointsIn_boundingBox = getPointSampling(pointsInSampling_boundingBox, particleInSampling_input, ctx.pointsSampling);
		}

		// Evaluate only once per frame update
		if (mLastTimeVal != ctx.adjustFrame)
		{
			eval_state = true;
			mLastTimeVal = ctx.adjustFrame;
		}

		// Build octree structure when evaluation state is reset
		if (eval_state == true)
		{
			mMinWidthBb = 0.5;
			mBoundingBoxIndex = 0;
			mBoundingBoxInfo_list.clear();
			mLastBoundingBox_list.clear();
			mBoundingBoxsToDrawInfo.clear();
			mDupBoundingBox_list.clear();
			mBoundingBoxOutputMain_list.clear();
			mBoundingBoxsToDraw_info.clear();
			mCrossToDraw_Info.clear();

			unsigned int recursion_count = 0;

			// Get bounding box from connected particle system if available
			if (mParticle_sys.name() != "")
			{			
				// Get points from particle system if input is not connected
				const unsigned int pointCount = pointsIn_boundingBox.length();
				if (pointCount == 0) {
					MVectorArray pointsIn_particle_sys;
					mParticle_sys.position(pointsIn_particle_sys);
					const unsigned int particleSysCount = pointsIn_particle_sys.length();
					pointsIn_boundingBox.setLength(particleSysCount);
					for (unsigned int i = 0; i < particleSysCount; ++i) {
						pointsIn_boundingBox.set(MPoint(pointsIn_particle_sys[i].x, pointsIn_particle_sys[i].y, pointsIn_particle_sys[i].z), i);
					}
				}
			
				// Get bounding box from particle system if container input is not connected
				if (ctx.boundingBoxCorner_list.length() == 0)
				{
					mParticle_sys.setObject(mParticle_path);
					mGlobalBoundingBox = mParticle_sys.boundingBox();
				}
			}

			//-- append root boundingBox to struct ------------------
			BoundingBoxInfo  rootBoundingBoxInfo;
			rootBoundingBoxInfo.boundingBox_parent = mGlobalBoundingBox;

			//-- get the min Width of the BoundingBox affter dividing to recursionDepth ----
			mMinWidthBb = mGlobalBoundingBox.width() / mRecursionDepth;


			//-- call startOctree ------------ 
			BoundingBoxInfo mainBoundingBoxInfo = startOctree(rootBoundingBoxInfo, pointsIn_boundingBox, recursion_count);

			// Find the minimum width among all bounding boxes (deepest level in octree)
			float lastMinWidthBb = 10000.0f;
			const size_t boundingBoxCount = mBoundingBoxInfo_list.size();
			for (size_t i = 0; i < boundingBoxCount; ++i) {
				const float minWidthBb = mBoundingBoxInfo_list[i].boundingBox_parent.width();
				if (lastMinWidthBb > minWidthBb)
				{
					lastMinWidthBb = minWidthBb;
				}
			}

			// Collect all bounding boxes at the deepest level (smallest width)
			for (size_t i = 0; i < boundingBoxCount; ++i) {
				const float minWidthBb = mBoundingBoxInfo_list[i].boundingBox_parent.width();
				if (lastMinWidthBb + 0.05f >= minWidthBb)
				{
					mBoundingBoxInfo_list[i].boundingBox_id = static_cast<unsigned int>(i);
					mLastBoundingBox_list.push_back(mBoundingBoxInfo_list[i]);
				}
			}

			//-- gather all BoundingBox data for draw ---------------------------------------------------------  
			status = gatherBoundingBoxsData(mLastBoundingBox_list);
			if (status == true)
			{
				//-- stop eval_state --------
				eval_state = false;
				eval_state2 = true;
			}
		}
		// Cluster nearby bounding boxes together
		if (ctx.clusterGrouping == 1)
		{
			if (eval_state2 == true)
			{
				if (!mLastBoundingBox_list.empty()) {
					mBoundingBoxOutputMain_list = clusteringBoundingBoxs(mLastBoundingBox_list);
					eval_state2 = false;
					eval_state3 = true;
				}
			}

			if (eval_state3 == true)
			{
				if (!mLastBoundingBox_list.empty()) {
					status = gatherClustersCrosessData(mBoundingBoxOutputMain_list);
					if (status == MS::kSuccess)
					{		
						eval_state3 = false;
						eval_state5 = true;
					}
				}
			}
		}
		else
		{
			// Clear clustering data when grouping is disabled
			mDupBoundingBox_list.clear();
			mBoundingBoxOutputMain_list.clear();
			mCrossToDraw_Info.clear();
			dataBlock.outputValue(clustersDetection_Attr).setInt(0);
		}

		// Track cluster sequence paths across frames
		const unsigned int crossCount = mCrossToDraw_Info.length();
		if (crossCount > 0)
		{
			if (ctx.createClusterPath == 1)
			{
				if (eval_state4 == true)
				{
					// Initialize cluster sequence paths on first frame
					mClustersSequencePath_list.clear();
					populateClustersSequencePath_struct(ctx.adjustFrame, mCrossToDraw_Info);
					eval_state4 = false;
					eval_state5 = true;
				}

				if (eval_state5 == true)
				{
					// Find closest cluster in next frame for each current cluster
					for (unsigned int c = 0; c < crossCount; ++c)
					{
						findClosestClusterInNextFrame(mCrossToDraw_Info[c], ctx.adjustFrame);
					}
					eval_state5 = false;
				}
			}
			else
			{
				// Clear sequence path data when disabled
				mClustersSequencePath_list.clear();
				eval_state4 = true;
			}
		}

		// Set output: number of detected clusters
		if (crossCount > 0) {
			dataBlock.outputValue(clustersDetection_Attr).setInt(static_cast<int>(crossCount));
		}

	}

	// Output bounding box point data for Viewport 2.0
	MFnPointArrayData dataPointArrayFn;
	MPointArray outPosArray;
	MObject posBb = dataBlock.outputValue(outDivBoundingBoxDraw_Attr).data();
	status = dataPointArrayFn.setObject(posBb);
	if (status == MS::kSuccess)
	{
		outPosArray = dataPointArrayFn.array();
	}
	outPosArray.setLength(mBoundingBoxsToDrawInfo.length());
	dataPointArrayFn.set(mBoundingBoxsToDrawInfo);

	// Output cross marker point data for Viewport 2.0
	MFnPointArrayData dataCrossPointArrayFn;
	MPointArray defaultCrossPointArray;
	MObject crossposD = dataBlock.outputValue(outCrossDraw_Attr).data();
	status = dataCrossPointArrayFn.setObject(crossposD);
	if (status == MS::kSuccess)
	{
		defaultCrossPointArray = dataCrossPointArrayFn.array();
	}

	defaultCrossPointArray.setLength(mCrossToDraw_Info.length());
	dataCrossPointArrayFn.set(mCrossToDraw_Info);


	//== viewport2 draw clusters Sequence Path ============================================================
	unsigned int numElements = mClustersSequencePath_list.size();
	if (numElements > 0)
	{
		MFnPointArrayData datalCustersPatheArrayFn;
		MPointArray outCustersPatheArray;
		MArrayDataHandle posCSP_dataHandle = dataBlock.outputArrayValue(outClustersSequencePathDraw_Attr);
		unsigned int numElements = mClustersSequencePath_list.size();
		unsigned int logicalIndex = 0;
		for (unsigned int c = 0; c < numElements; ++c)
		{
			MPointArray clusterPosition_list;
			posCSP_dataHandle.jumpToArrayElement(c);
			logicalIndex = posCSP_dataHandle.elementIndex();
			jumpToElement(posCSP_dataHandle, c);

			MDataHandle clusterClustersSequence = posCSP_dataHandle.outputValue();
			MObject posCSP = clusterClustersSequence.data();
			status = datalCustersPatheArrayFn.setObject(posCSP);
			if (status == MS::kSuccess)
			{
				outCustersPatheArray = datalCustersPatheArrayFn.array();
			}

			if (mClustersSequencePath_list.size() != 0)
			{
				clusterPosition_list = mClustersSequencePath_list[c].clusterPosition_list;
			}

			outCustersPatheArray.setLength(clusterPosition_list.length());
			datalCustersPatheArrayFn.set(clusterPosition_list);
		}
	}
	
	
	// Set output number of endFrame
	dataBlock.outputValue(out_Attr).setInt(adjustFrame);

	// Create curves if enabled
	bool createCurvs = dataBlock.inputValue(createCurvs_Attr).asBool();
	if (createCurvs == true)
	{
		createCurves();

		dataBlock.outputValue(createCurvs_Attr).setBool(false);
	}

	return MS::kSuccess;
}



/// Samples points from input arrays based on sampling rate to reduce point count.
/// @param pointsInSampling_input Points from pclLoader input.
/// @param particleInSampling_input Points from particle system input.
/// @param pointsSampling Sampling rate (every Nth point). If 0, returns all points.
/// @return Sampled point array.
/// Samples points from input arrays using a stride pattern.
/// Combines points from both pclLoader input and particle system input, applying sampling if specified.
/// @param pointsInSampling_input Points from pclLoader node.
/// @param particleInSampling_input Points from particle system (as vectors).
/// @param pointsSampling Sampling stride (0 = no sampling, use all points).
/// @return Combined array of sampled points.
MPointArray pclCluster::getPointSampling(MPointArray pointsInSampling_input, MVectorArray particleInSampling_input, unsigned int pointsSampling)
{
	MPointArray pointsInSampling;

	// Sample points from pclLoader input
	const unsigned int inputPointCount = pointsInSampling_input.length();
	if (inputPointCount > 0)
	{
		if (pointsSampling == 0 || pointsSampling >= inputPointCount)
		{
			// No sampling needed, copy all points
			pointsInSampling = pointsInSampling_input;
		}
		else
		{
			// Pre-allocate array for sampled points
			const unsigned int sampledCount = (inputPointCount / pointsSampling) + 1;
			pointsInSampling.setLength(sampledCount);
			unsigned int outputIndex = 0;
			for (unsigned int p = 0; p < inputPointCount; p += pointsSampling) 
			{
				pointsInSampling.set(pointsInSampling_input[p], outputIndex++);
			}
			pointsInSampling.setLength(outputIndex);
		}
	}

	// Sample points from particle system input
	const unsigned int particleInputCount = particleInSampling_input.length();
	if (particleInputCount > 0)
	{
		const unsigned int currentLength = pointsInSampling.length();
		if (pointsSampling == 0 || pointsSampling >= particleInputCount)
		{
			// No sampling needed, append all points
			const unsigned int newLength = currentLength + particleInputCount;
			pointsInSampling.setLength(newLength);
			for (unsigned int p = 0; p < particleInputCount; ++p)
			{
				pointsInSampling.set(MPoint(particleInSampling_input[p].x, particleInSampling_input[p].y, particleInSampling_input[p].z), currentLength + p);
			}
		}
		else
		{
			// Pre-allocate array for sampled points
			const unsigned int sampledCount = (particleInputCount / pointsSampling) + 1;
			const unsigned int newLength = currentLength + sampledCount;
			pointsInSampling.setLength(newLength);
			unsigned int outputIndex = currentLength;
			for (unsigned int p = 0; p < particleInputCount; p += pointsSampling)
			{
				pointsInSampling.set(MPoint(particleInSampling_input[p].x, particleInSampling_input[p].y, particleInSampling_input[p].z), outputIndex++);
			}
			pointsInSampling.setLength(outputIndex);
		}
	}

	return pointsInSampling;
}

/// Recursively builds a spatial tree structure by subdividing bounding boxes.
/// Uses a quadtree-like subdivision (4 children) along X and Z axes, keeping full Y range.
/// @param boundingBoxInfo_parent Parent bounding box information.
/// @param pointsIn_boundingBox Points contained within the parent bounding box (pre-filtered).
/// @param recursion_count Current recursion depth (0-based).
/// @return Bounding box information with child nodes populated.
BoundingBoxInfo pclCluster::startOctree(BoundingBoxInfo boundingBoxInfo_parent, MPointArray pointsIn_boundingBox, unsigned int recursion_count)
{
	// Store recursion depth and points for this node
	boundingBoxInfo_parent.recursion_count = recursion_count;
	boundingBoxInfo_parent.pointsIn_boundingBox = pointsIn_boundingBox;

	// Base case: stop recursion if depth limit reached or insufficient points
	const unsigned int pointCount = pointsIn_boundingBox.length();
	const bool shouldRecurse = (recursion_count + 1 < mRecursionDepth) && 
	                           (pointCount > mMinimumPoint_count);

	if (shouldRecurse)
	{
		// Divide the bounding box into 4 child boxes (quadtree-like subdivision)
		std::vector<BoundingBoxInfo> newBoundingBoxInfo_list = divideBoundingBox(boundingBoxInfo_parent.boundingBox_parent);
		const size_t childCount = newBoundingBoxInfo_list.size();

		// Process each child bounding box
		for (size_t i = 0; i < childCount; ++i)
		{
			// Filter points that lie within this child bounding box
			// Note: We filter from the parent's points, which are already pre-filtered
			MPointArray pointsIn_bbox = isPointInBoundingBox(
				newBoundingBoxInfo_list[i].boundingBox_parent, 
				pointsIn_boundingBox
			);

			const unsigned int childPointCount = pointsIn_bbox.length();
			
			// Only recurse if child has enough points
			if (childPointCount > mMinimumPoint_count)
			{
				// Recursively build subtree for this child
				BoundingBoxInfo boundingBoxInfo_child = startOctree(
					newBoundingBoxInfo_list[i], 
					pointsIn_bbox, 
					recursion_count + 1
				);

				// Store child in parent's child list
				boundingBoxInfo_parent.boundingBoxChild_list.push_back(boundingBoxInfo_child);

				// Add to global list if it meets the minimum width criteria
				const float childWidth = newBoundingBoxInfo_list[i].boundingBox_parent.width();
				if (mMinWidthBb >= childWidth)
				{
					mBoundingBoxInfo_list.push_back(boundingBoxInfo_child);
				}
			}
		}
	}

	return boundingBoxInfo_parent;
}



/// Divides a bounding box into 4 sub-boxes by splitting along X and Z axes (keeps full Y range).
/// Creates 4 child boxes: front-right, front-left, back-right, back-left.
/// @param boundingBox The parent bounding box to subdivide.
/// @return Vector of 4 BoundingBoxInfo structures with populated boundingBox_parent members.
std::vector<BoundingBoxInfo> pclCluster::divideBoundingBox(MBoundingBox boundingBox)
{
	// Extract bounding box dimensions
	const float xMin = boundingBox.min().x;
	const float xMax = boundingBox.max().x;
	const float yMin = boundingBox.min().y;
	const float yMax = boundingBox.max().y;
	const float zMin = boundingBox.min().z;
	const float zMax = boundingBox.max().z;
	
	const float xlen = xMax - xMin;
	const float ylen = yMax - yMin;
	const float zlen = zMax - zMin;
	const float xMid = xMax - (xlen * 0.5f);
	const float zMid = zMax - (zlen * 0.5f);

	std::vector<BoundingBoxInfo> newBoundingBox_list;
	newBoundingBox_list.resize(4);

	// Helper lambda to create a bounding box from min/max coordinates
	auto createBoundingBox = [](float xMin, float xMax, float yMin, float yMax, float zMin, float zMax) -> MBoundingBox
	{
		MBoundingBox bbox;
		bbox.expand(MPoint(xMin, yMin, zMin));
		bbox.expand(MPoint(xMax, yMax, zMax));
		return bbox;
	};

	// Box 1: Front-right (x: mid to max, z: mid to max, y: full range)
	newBoundingBox_list[0].boundingBox_parent = createBoundingBox(xMid, xMax, yMin, yMax, zMid, zMax);

	// Box 2: Front-left (x: min to mid, z: mid to max, y: full range)
	newBoundingBox_list[1].boundingBox_parent = createBoundingBox(xMin, xMid, yMin, yMax, zMid, zMax);

	// Box 3: Back-right (x: mid to max, z: min to mid, y: full range)
	newBoundingBox_list[2].boundingBox_parent = createBoundingBox(xMid, xMax, yMin, yMax, zMin, zMid);

	// Box 4: Back-left (x: min to mid, z: min to mid, y: full range)
	newBoundingBox_list[3].boundingBox_parent = createBoundingBox(xMin, xMid, yMin, yMax, zMin, zMid);


	return newBoundingBox_list;
}

/// Filters points to only those contained within the given bounding box.
/// @param boundingBox The bounding box to test against.
/// @param pointsIn_boundingBox Input array of points to filter.
/// @return Array containing only points within the bounding box.
MPointArray pclCluster::isPointInBoundingBox(MBoundingBox boundingBox, MPointArray pointsIn_boundingBox)
{
	const unsigned int pointCount = pointsIn_boundingBox.length();
	if (pointCount == 0)
	{
		return MPointArray();
	}

	// Pre-allocate array with worst-case size (all points might be inside)
	MPointArray pointsIn_bb;
	pointsIn_bb.setLength(pointCount);

	unsigned int validIndex = 0;
	for (unsigned int i = 0; i < pointCount; ++i)
	{
		const MPoint& pointPos = pointsIn_boundingBox[i];
		if (boundingBox.contains(pointPos))
		{
			pointsIn_bb.set(pointPos, validIndex++);
		}
	}

	// Resize to actual number of points found
	pointsIn_bb.setLength(validIndex);
	return pointsIn_bb;
}

/// Collects bounding box corner data for visualization in both Viewport 1.0 and 2.0.
/// Extracts the 8 corner vertices from each bounding box and organizes them for drawing.
/// @param lastBoundingBox_list List of bounding box information to process.
/// @return MStatus indicating success or failure.
MStatus pclCluster::gatherBoundingBoxsData(std::vector<BoundingBoxInfo> lastBoundingBox_list)
{
	mBoundingBoxsToDrawInfo.clear();
	mBoundingBoxsToDraw_info.clear();
	
	const size_t boxCount = lastBoundingBox_list.size();
	if (boxCount == 0)
	{
		return MS::kSuccess;
	}

	// Pre-allocate space for all bounding box vertices (8 corners per box, plus line segments)
	// Viewport 1.0: one array per box (8 points)
	mBoundingBoxsToDraw_info.reserve(boxCount);
	// Viewport 2.0: 8 points + 24 line segment points (3 sets of 8) = 32 points per box
	mBoundingBoxsToDrawInfo.setLength(boxCount * 32);

	unsigned int viewport2Index = 0;
	
	// Process each bounding box
	for (size_t i = 0; i < boxCount; ++i)
	{
		// Cache bounding box reference to avoid repeated access
		const MBoundingBox& bbox = lastBoundingBox_list[i].boundingBox_parent;
		const MPoint bboxMin = bbox.min();
		const MPoint bboxMax = bbox.max();

		// Calculate the 8 corner vertices of the bounding box
		const MFloatPoint vtx0(bboxMax.x, bboxMax.y, bboxMax.z);  // Top-front-right
		const MFloatPoint vtx1(bboxMin.x, bboxMax.y, bboxMax.z);  // Top-front-left
		const MFloatPoint vtx2(bboxMax.x, bboxMin.y, bboxMax.z);  // Bottom-front-right
		const MFloatPoint vtx3(bboxMin.x, bboxMin.y, bboxMax.z);  // Bottom-front-left
		const MFloatPoint vtx4(bboxMax.x, bboxMax.y, bboxMin.z);  // Top-back-right
		const MFloatPoint vtx5(bboxMin.x, bboxMax.y, bboxMin.z);  // Top-back-left
		const MFloatPoint vtx6(bboxMax.x, bboxMin.y, bboxMin.z);  // Bottom-back-right
		const MFloatPoint vtx7(bboxMin.x, bboxMin.y, bboxMin.z);  // Bottom-back-left

		// For Viewport 1.0: store all 8 corners as a separate array
		MPointArray boundingBoxsInfo;
		boundingBoxsInfo.setLength(8);
		boundingBoxsInfo.set(vtx0, 0);
		boundingBoxsInfo.set(vtx1, 1);
		boundingBoxsInfo.set(vtx2, 2);
		boundingBoxsInfo.set(vtx3, 3);
		boundingBoxsInfo.set(vtx4, 4);
		boundingBoxsInfo.set(vtx5, 5);
		boundingBoxsInfo.set(vtx6, 6);
		boundingBoxsInfo.set(vtx7, 7);
		mBoundingBoxsToDraw_info.push_back(boundingBoxsInfo);

		// For Viewport 2.0: build line segments for drawing (3 sets of 8 points each)
		// Set 1: All 8 corners in order
		mBoundingBoxsToDrawInfo.set(vtx0, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx1, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx2, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx3, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx4, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx5, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx6, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx7, viewport2Index++);

		// Set 2: Line segments pattern (vertical edges)
		mBoundingBoxsToDrawInfo.set(vtx0, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx4, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx2, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx6, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx1, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx5, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx3, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx7, viewport2Index++);

		// Set 3: Line segments pattern (horizontal edges)
		mBoundingBoxsToDrawInfo.set(vtx0, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx2, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx1, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx3, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx4, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx6, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx5, viewport2Index++);
		mBoundingBoxsToDrawInfo.set(vtx7, viewport2Index++);
	}

	// Resize to actual used length
	mBoundingBoxsToDrawInfo.setLength(viewport2Index);

	return MS::kSuccess;
}

/// Collects cluster center positions for visualization (cross markers).
/// Calculates average center positions either from bounding box centers or from all points within bounding boxes.
/// @param boundingBoxOutputMain_list List of clusters, each containing multiple bounding boxes.
/// @return MStatus indicating success or failure.
MStatus pclCluster::gatherClustersCrosessData(std::vector<std::vector<BoundingBoxInfo>> boundingBoxOutputMain_list)
{
	mCrossToDraw_Info.clear();

	const size_t clusterCount = boundingBoxOutputMain_list.size();
	if (clusterCount == 0)
	{
		return MS::kSuccess;
	}

	// Pre-allocate output array
	mCrossToDraw_Info.setLength(clusterCount);

	// Calculate cluster centers based on centering mode
	if (mClusterCentering == 0)
	{
		// Mode 0: Use bounding box centers, Y from particle system
		const float locPos_y = mParticle_sys.boundingBox().center().y;
		
		for (size_t i = 0; i < clusterCount; ++i)
		{
			const size_t boxCount = boundingBoxOutputMain_list[i].size();
			if (boxCount == 0)
			{
				continue;
			}

			float locPos_x = 0.0f;
			float locPos_z = 0.0f;
			
			// Sum all bounding box center positions
			for (size_t b = 0; b < boxCount; ++b)
			{
				const MPoint center = boundingBoxOutputMain_list[i][b].boundingBox_parent.center();
				locPos_x += center.x;
				locPos_z += center.z;
			}

			// Calculate average position
			const MPoint locPos(locPos_x / boxCount, locPos_y, locPos_z / boxCount);
			mCrossToDraw_Info.set(locPos, i);
		}
	}
	else
	{
		// Mode 1: Use average of all points within all bounding boxes in the cluster
		for (size_t i = 0; i < clusterCount; ++i)
		{
			const size_t boxCount = boundingBoxOutputMain_list[i].size();
			
			// Collect all points from all bounding boxes in this cluster
			MPointArray clusterBoundingBoxCenterPos_list;
			size_t totalPointCount = 0;
			
			for (size_t b = 0; b < boxCount; ++b)
			{
				const MPointArray& pointsInBoundingBox_list = boundingBoxOutputMain_list[i][b].pointsIn_boundingBox;
				const unsigned int pointCount = pointsInBoundingBox_list.length();
				totalPointCount += pointCount;
			}

			// Pre-allocate array for all points
			clusterBoundingBoxCenterPos_list.setLength(totalPointCount);
			unsigned int pointIndex = 0;

			for (size_t b = 0; b < boxCount; ++b)
			{
				const MPointArray& pointsInBoundingBox_list = boundingBoxOutputMain_list[i][b].pointsIn_boundingBox;
				const unsigned int pointCount = pointsInBoundingBox_list.length();
				
				for (unsigned int x = 0; x < pointCount; ++x)
				{
					clusterBoundingBoxCenterPos_list.set(pointsInBoundingBox_list[x], pointIndex++);
				}
			}

			// Calculate average position of all points
			if (totalPointCount > 0)
			{
				float locPos_x = 0.0f;
				float locPos_y = 0.0f;
				float locPos_z = 0.0f;

				for (unsigned int p = 0; p < totalPointCount; ++p)
				{
					const MPoint& pt = clusterBoundingBoxCenterPos_list[p];
					locPos_x += pt.x;
					locPos_y += pt.y;
					locPos_z += pt.z;
				}

				const MPoint clusterCenterPos(
					locPos_x / totalPointCount,
					locPos_y / totalPointCount,
					locPos_z / totalPointCount
				);
				mCrossToDraw_Info.set(clusterCenterPos, i);
			}
		}
	}

	return MS::kSuccess;
}


/// Groups nearby bounding boxes into clusters based on spatial proximity.
/// Uses a distance-based clustering algorithm to find all connected bounding boxes.
/// @param boundingBoxInfo_list List of bounding boxes to cluster.
/// @return Vector of clusters, where each cluster is a vector of connected bounding boxes.
std::vector<std::vector<BoundingBoxInfo>> pclCluster::clusteringBoundingBoxs(std::vector<BoundingBoxInfo> boundingBoxInfo_list)
{
	if (boundingBoxInfo_list.empty())
	{
		return std::vector<std::vector<BoundingBoxInfo>>();
	}

	mDupBoundingBox_list = boundingBoxInfo_list;
	const float minWith = mDupBoundingBox_list[0].boundingBox_parent.width() + 0.01f;
	const float minDepth = mDupBoundingBox_list[0].boundingBox_parent.depth() + 0.01f;
	std::vector<std::vector<BoundingBoxInfo>> boundingBoxOutputMain_list;

	// Process all bounding boxes until none remain
	while (!mDupBoundingBox_list.empty())
	{
		const MPoint boundingBox_center = mDupBoundingBox_list[0].boundingBox_parent.center();
		std::vector<BoundingBoxInfo> boundingBoxTempOutput_list;
		std::vector<BoundingBoxInfo> boundingBoxOutput_list = getClosestBoundingBox(boundingBox_center, minWith, minDepth, boundingBoxTempOutput_list, 0);

		boundingBoxOutputMain_list.push_back(boundingBoxOutput_list);
	}

	return boundingBoxOutputMain_list;
}

/// Recursively finds all bounding boxes within a distance threshold from the given center point.
/// Groups nearby bounding boxes into clusters by recursively searching for neighbors.
/// @param boundingBox_center Center point to search from.
/// @param minWith Minimum width threshold for distance checking.
/// @param minDepth Minimum depth threshold for distance checking.
/// @param boundingBoxOutput_list Current list of bounding boxes in this cluster.
/// @param counter Recursion counter to prevent infinite loops.
/// @return Updated list of bounding boxes in the cluster.
std::vector<BoundingBoxInfo> pclCluster::getClosestBoundingBox(MPoint boundingBox_center, float minWith, float minDepth, std::vector<BoundingBoxInfo> boundingBoxOutput_list, unsigned int counter)
{
	const std::vector<BoundingBoxInfo> boundingBoxTemp_list = mDupBoundingBox_list;
	const size_t tempListSize = boundingBoxTemp_list.size();

	// Loop over all bounding boxes to find neighbors
	for (size_t i = 0; i < tempListSize; ++i)
	{
		const MPoint nextBB_center = boundingBoxTemp_list[i].boundingBox_parent.center();
		const unsigned int boundingBox_id = boundingBoxTemp_list[i].boundingBox_id;

		// Calculate 2D distance (XZ plane)
		const float dx = nextBB_center.x - boundingBox_center.x;
		const float dz = nextBB_center.z - boundingBox_center.z;
		const float dist = sqrt(dx * dx + dz * dz);

		// Check if within distance threshold
		if (dist <= minWith || dist <= minDepth)
		{
			// If not already in cluster, add it
			if (!boundingBoxTemp_list[i].inCluster)
			{
				boundingBoxOutput_list.push_back(boundingBoxTemp_list[i]);

				// Remove from main list
				const size_t dupListSize = mDupBoundingBox_list.size();
				for (size_t b = 0; b < dupListSize; ++b)
				{
					if (boundingBox_id == mDupBoundingBox_list[b].boundingBox_id)
					{
						mDupBoundingBox_list.erase(mDupBoundingBox_list.begin() + b);
						break; // Found and removed, exit loop
					}
				}
			}

			boundingBoxTemp_list[i].inCluster = true;
		}
	}

	counter++;

	// Recursively search from newly added bounding boxes
	const size_t outputListSize = boundingBoxOutput_list.size();
	if (counter < outputListSize && !mDupBoundingBox_list.empty())
	{
		const MPoint nextCenter = boundingBoxOutput_list[counter].boundingBox_parent.center();
		boundingBoxOutput_list = getClosestBoundingBox(nextCenter, minWith, minDepth, boundingBoxOutput_list, counter);
	}

	return boundingBoxOutput_list;
}


/// Creates NURBS curves from cluster sequence paths for visualization.
/// Each cluster's position sequence is converted into a linear NURBS curve.
void pclCluster::createCurves()
{
	const size_t clusterCount = mClustersSequencePath_list.size();
	
	for (size_t c = 0; c < clusterCount; ++c)
	{
		const MPointArray& controlVertices = mClustersSequencePath_list[c].clusterPosition_list;
		const unsigned int ncvs = controlVertices.length();

		// Need at least 2 control vertices to create a curve
		if (ncvs > 1)
		{
			const unsigned int deg = 1;  // Linear curve degree
			const unsigned int spans = ncvs - deg;
			const unsigned int nknots = spans + 2 * deg - 1;

			// Pre-allocate knot sequence array
			MDoubleArray knotSequences;
			knotSequences.setLength(ncvs);
			
			for (unsigned int i = 0; i < ncvs; ++i)
			{
				knotSequences.set(static_cast<double>(i), i);
			}

			// Create the NURBS curve
			MFnNurbsCurve curveFn;
			curveFn.create(controlVertices, knotSequences, deg, MFnNurbsCurve::kOpen, false, false, MObject::kNullObj);
		}
	}
}
/// Initializes cluster sequence path structures for each cluster position at the given time.
/// Creates a new sequence path entry for each cluster center position.
/// @param timeVal The current frame/time value.
/// @param clustersPosition Array of cluster center positions.
void pclCluster::populateClustersSequencePath_struct(unsigned int timeVal, MPointArray clustersPosition)
{
	const unsigned int clusterCount = clustersPosition.length();
	
	for (unsigned int c = 0; c < clusterCount; ++c)
	{
		ClustersSequencePath clustersSequencePath;
		clustersSequencePath.cluster_id = c;
		clustersSequencePath.startFrame = timeVal;
		clustersSequencePath.clusterPosition_list.append(clustersPosition[c]);
		clustersSequencePath.clusterFrame_list.append(timeVal);
		clustersSequencePath.lastFrame = timeVal;
		mClustersSequencePath_list.push_back(clustersSequencePath);
	}
}

/// Finds or creates a cluster sequence path entry for the given cluster center at the specified time.
/// If time moves backwards, removes future entries. If a nearby cluster sequence exists, appends to it;
/// otherwise creates a new sequence path.
/// @param clusterCenter The 3D position of the cluster center.
/// @param timeVal The current frame/time value.
void pclCluster::findClosestClusterInNextFrame(MPoint clusterCenter, unsigned int timeVal)
{
	const size_t pathListSize = mClustersSequencePath_list.size();

	// Remove future entries if time moved backwards
	for (size_t i = 0; i < pathListSize; ++i)
	{
		unsigned int listCounter = mClustersSequencePath_list[i].listCounter();
		if (listCounter > 1 && timeVal < mClustersSequencePath_list[i].lastFrame)
		{
			const unsigned int valDifference = mClustersSequencePath_list[i].lastFrame - timeVal;
			const unsigned int positionCount = mClustersSequencePath_list[i].clusterPosition_list.length();
			
			if (positionCount > 1)
			{
				// Remove entries from the end
				for (unsigned int d = 0; d < valDifference && d < listCounter; ++d)
				{
					const unsigned int removeIndex = (listCounter - 1) - d;
					if (removeIndex < positionCount)
					{
						mClustersSequencePath_list[i].clusterPosition_list.remove(removeIndex);
						mClustersSequencePath_list[i].clusterFrame_list.remove(removeIndex);
					}
				}
				mClustersSequencePath_list[i].lastFrame = timeVal;
			}
		}
	}

	// Search for existing cluster sequence within radius
	unsigned int clustersInRadiusIndex = 0;
	unsigned int searchResult = 0;
	
	for (size_t i = 0; i < pathListSize; ++i)
	{
		if (timeVal > mClustersSequencePath_list[i].lastFrame)
		{
			const unsigned int listCounter = mClustersSequencePath_list[i].listCounter();
			if (listCounter > 0)
			{
				const MPoint clusterLastPos = mClustersSequencePath_list[i].clusterPosition_list[listCounter - 1];

				// Calculate 3D distance
				const float dx = clusterLastPos.x - clusterCenter.x;
				const float dy = clusterLastPos.y - clusterCenter.y;
				const float dz = clusterLastPos.z - clusterCenter.z;
				const float dist = sqrt(dx * dx + dy * dy + dz * dz);

				// Check if within search radius
				if (mNextFrameSearchRadius != 0.0 && dist <= mNextFrameSearchRadius)
				{
					clustersInRadiusIndex = static_cast<unsigned int>(i);
					searchResult = 1;
					break; // Found a match, exit loop
				}
			}
		}
	}

	// Append to existing sequence or create new one
	if (searchResult == 1)
	{
		mClustersSequencePath_list[clustersInRadiusIndex].clusterPosition_list.append(clusterCenter);
		mClustersSequencePath_list[clustersInRadiusIndex].clusterFrame_list.append(timeVal);
		mClustersSequencePath_list[clustersInRadiusIndex].lastFrame = timeVal;
	}
	else
	{
		// Create new cluster sequence path
		ClustersSequencePath clustersSequencePath;
		clustersSequencePath.cluster_id = static_cast<unsigned int>(pathListSize) + 1;
		clustersSequencePath.startFrame = timeVal;
		clustersSequencePath.clusterPosition_list.append(clusterCenter);
		clustersSequencePath.clusterFrame_list.append(timeVal);
		clustersSequencePath.lastFrame = timeVal;
		mClustersSequencePath_list.push_back(clustersSequencePath);
	}
}

/// Retrieves the connected particle system from the given plug connection.
/// @param mPlug The plug to check for particle system connections.
/// @return MStatus indicating success or failure.
MStatus pclCluster::getConnetdedParticleSys(MPlug mPlug)
{
	MStatus status;

	MPlugArray mPlugArray;
	mPlug.connectedTo(mPlugArray, true, true);

	//-- TO Prevent MAYA crashing ------------
	if (mPlugArray.length() == 0)
	{
		return MS::kInvalidParameter;
	}

	MObject particle_obj = mPlugArray[0].node();
	
	MFnDagNode dagN(particle_obj);
	MString particleStringName = dagN.fullPathName();
	MSelectionList list;
	list.add(particleStringName);

	list.getDagPath(0, mParticle_path);
	mParticle_path.extendToShape();


	return status;
}



//-- draw -----------------------------------------------
void pclCluster::draw(M3dView& view, const MDagPath& DGpath, M3dView::DisplayStyle style, M3dView::DisplayStatus status)
{
	view.beginGL();
	glPushAttrib(GL_CURRENT_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthMask(GL_FALSE);

	MColor wireColor;
	if (status == M3dView::kActive)
	{
		wireColor = MColor(1.0f, 1.0f, 1.0f, 1.0f);
	}
	else if (status == M3dView::kLead)
	{
		wireColor = MColor(.26f, 1.0f, .64f, 1.0f);
	}
	else
	{
		wireColor = MColor(1.0f, 1.0f, 0.0f, 1.0f);
	}


	//-- call the draw Global BoundingBox --------------------
	if (mGlobalBoundingBoxsToDraw_info.length() > 0) {
		drawGlobalBoundingBox(view, mGlobalBoundingBoxsToDraw_info);
	}


	//-- call the draw BoundingBox ----------------	
	if (mBoundingBoxsToDraw_info.size() > 0) {

		for (unsigned int i = 0; i < mBoundingBoxsToDraw_info.size(); ++i) {
			drawBoundingBox(view, mBoundingBoxsToDraw_info[i]);
		}
	}
	
	//-- call the draw  Cross ----------------	
	if (mCrossToDraw_Info.length() > 0) {	
		drawCross(view, mCrossToDraw_Info, mClusterDrawScale);
	}


	//-- call the draw clusters Sequence Path ----------------
	for (int c = 0; c < mClustersSequencePath_list.size(); ++c)
	{
		drawClusterSequencPath(view, mClustersSequencePath_list[c].clusterPosition_list);	
	}	

	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);
	glPopAttrib();
	view.endGL();

}


//-- draw the node BoundingBox----------------
void pclCluster::drawGlobalBoundingBox(M3dView& view, MFloatArray globalBoundingBoxsToDraw_info)
{
	//MGlobal::displayInfo(MString("**  drawBoundingBox *** ") );

	MPoint minVertex = MPoint(globalBoundingBoxsToDraw_info[0], globalBoundingBoxsToDraw_info[1], globalBoundingBoxsToDraw_info[2]);

	float w = globalBoundingBoxsToDraw_info[3];
	float h = globalBoundingBoxsToDraw_info[4];
	float d = globalBoundingBoxsToDraw_info[5];

	// Draw first side
	glBegin(GL_LINE_LOOP);
	MPoint vertex = minVertex;

	glVertex3f((float)vertex[0], (float)vertex[1], (float)vertex[2]);
	glVertex3f((float)vertex[0] + w, (float)vertex[1], (float)vertex[2]);
	glVertex3f((float)vertex[0] + w, (float)vertex[1] + h, (float)vertex[2]);
	glVertex3f((float)vertex[0], (float)vertex[1] + h, (float)vertex[2]);
	glVertex3f((float)vertex[0], (float)vertex[1], (float)vertex[2]);
	glEnd();

	// Draw second side
	MPoint sideFactor(0, 0, d);
	MPoint vertex2 = minVertex + sideFactor;
	glBegin(GL_LINE_LOOP);
	glVertex3f((float)vertex2[0], (float)vertex2[1], (float)vertex2[2]);
	glVertex3f((float)vertex2[0] + w, (float)vertex2[1], (float)vertex2[2]);
	glVertex3f((float)vertex2[0] + w, (float)vertex2[1] + h, (float)vertex2[2]);
	glVertex3f((float)vertex2[0], (float)vertex2[1] + h, (float)vertex2[2]);
	glVertex3f((float)vertex2[0], (float)vertex2[1], (float)vertex2[2]);
	glEnd();

	// Connect the edges together
	glBegin(GL_LINES);
	glVertex3f((float)vertex2[0], (float)vertex2[1], (float)vertex2[2]);
	glVertex3f((float)vertex[0], (float)vertex[1], (float)vertex[2]);

	glVertex3f((float)vertex2[0] + w, (float)vertex2[1], (float)vertex2[2]);
	glVertex3f((float)vertex[0] + w, (float)vertex[1], (float)vertex[2]);

	glVertex3f((float)vertex2[0] + w, (float)vertex2[1] + h, (float)vertex2[2]);
	glVertex3f((float)vertex[0] + w, (float)vertex[1] + h, (float)vertex[2]);

	glVertex3f((float)vertex2[0], (float)vertex2[1] + h, (float)vertex2[2]);
	glVertex3f((float)vertex[0], (float)vertex[1] + h, (float)vertex[2]);
	glEnd();
	glPopAttrib();

}


//-- draw the BoundingBox----------------
void pclCluster::drawBoundingBox(M3dView& view, MPointArray boundingBoxsToDrawInfo)
{
	//MGlobal::displayInfo(MString("**  drawBoundingBox *** ")+ boundingBoxsToDrawInfo.length());
	
	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[0].x, boundingBoxsToDrawInfo[0].y, boundingBoxsToDrawInfo[0].z);
	glVertex3f(boundingBoxsToDrawInfo[1].x, boundingBoxsToDrawInfo[1].y, boundingBoxsToDrawInfo[1].z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[2].x, boundingBoxsToDrawInfo[2].y, boundingBoxsToDrawInfo[2].z);
	glVertex3f(boundingBoxsToDrawInfo[3].x, boundingBoxsToDrawInfo[3].y, boundingBoxsToDrawInfo[3].z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[4].x, boundingBoxsToDrawInfo[4].y, boundingBoxsToDrawInfo[4].z);
	glVertex3f(boundingBoxsToDrawInfo[5].x, boundingBoxsToDrawInfo[5].y, boundingBoxsToDrawInfo[5].z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[6].x, boundingBoxsToDrawInfo[6].y, boundingBoxsToDrawInfo[6].z);
	glVertex3f(boundingBoxsToDrawInfo[7].x, boundingBoxsToDrawInfo[7].y, boundingBoxsToDrawInfo[7].z);
	glEnd();

	//----------------------------------------------------------
	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[0].x, boundingBoxsToDrawInfo[0].y, boundingBoxsToDrawInfo[0].z);
	glVertex3f(boundingBoxsToDrawInfo[4].x, boundingBoxsToDrawInfo[4].y, boundingBoxsToDrawInfo[4].z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[2].x, boundingBoxsToDrawInfo[2].y, boundingBoxsToDrawInfo[2].z);
	glVertex3f(boundingBoxsToDrawInfo[6].x, boundingBoxsToDrawInfo[6].y, boundingBoxsToDrawInfo[6].z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[1].x, boundingBoxsToDrawInfo[1].y, boundingBoxsToDrawInfo[1].z);
	glVertex3f(boundingBoxsToDrawInfo[5].x, boundingBoxsToDrawInfo[5].y, boundingBoxsToDrawInfo[5].z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[3].x, boundingBoxsToDrawInfo[3].y, boundingBoxsToDrawInfo[3].z);
	glVertex3f(boundingBoxsToDrawInfo[7].x, boundingBoxsToDrawInfo[7].y, boundingBoxsToDrawInfo[7].z);
	glEnd();

	//----------------------------------------------------------
	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[0].x, boundingBoxsToDrawInfo[0].y, boundingBoxsToDrawInfo[0].z);
	glVertex3f(boundingBoxsToDrawInfo[2].x, boundingBoxsToDrawInfo[2].y, boundingBoxsToDrawInfo[2].z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[1].x, boundingBoxsToDrawInfo[1].y, boundingBoxsToDrawInfo[1].z);
	glVertex3f(boundingBoxsToDrawInfo[3].x, boundingBoxsToDrawInfo[3].y, boundingBoxsToDrawInfo[3].z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[4].x, boundingBoxsToDrawInfo[4].y, boundingBoxsToDrawInfo[4].z);
	glVertex3f(boundingBoxsToDrawInfo[6].x, boundingBoxsToDrawInfo[6].y, boundingBoxsToDrawInfo[6].z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(boundingBoxsToDrawInfo[5].x, boundingBoxsToDrawInfo[5].y, boundingBoxsToDrawInfo[5].z);
	glVertex3f(boundingBoxsToDrawInfo[7].x, boundingBoxsToDrawInfo[7].y, boundingBoxsToDrawInfo[7].z);
	glEnd();
	
	glPopAttrib();

}


/// Draws cross markers at cluster center positions.
/// @param view The 3D view to draw into.
/// @param crossToDrawInfo Array of positions where crosses should be drawn.
/// @param clusterDrawScale Scale factor for the cross size.
void pclCluster::drawCross(M3dView& view, MPointArray crossToDrawInfo, float clusterDrawScale)
{
	const unsigned int crossCount = mCrossToDraw_Info.length();
	for (unsigned int i = 0; i < crossCount; ++i) {
		glBegin(GL_LINE_LOOP);
		glVertex3f(crossToDrawInfo[i].x - 2 * clusterDrawScale, crossToDrawInfo[i].y, crossToDrawInfo[i].z);
		glVertex3f(crossToDrawInfo[i].x + 2 * clusterDrawScale, crossToDrawInfo[i].y, crossToDrawInfo[i].z);
		glEnd();

		glBegin(GL_LINE_LOOP);
		glVertex3f(crossToDrawInfo[i].x, crossToDrawInfo[i].y - 2 * clusterDrawScale, crossToDrawInfo[i].z);
		glVertex3f(crossToDrawInfo[i].x, crossToDrawInfo[i].y + 2 * clusterDrawScale, crossToDrawInfo[i].z);
		glEnd();

		glBegin(GL_LINE_LOOP);
		glVertex3f(crossToDrawInfo[i].x, crossToDrawInfo[i].y, crossToDrawInfo[i].z - 2 * clusterDrawScale);
		glVertex3f(crossToDrawInfo[i].x, crossToDrawInfo[i].y, crossToDrawInfo[i].z + 2 * clusterDrawScale);
		glEnd();
	}

	glPopAttrib();
}


/// Draws a line connecting cluster positions across frames to visualize the sequence path.
/// @param view The 3D view to draw into.
/// @param clustersSequencePos Array of cluster positions forming the sequence path.
void pclCluster::drawClusterSequencPath(M3dView& view, MPointArray clustersSequencePos)
{
	const unsigned int pointCount = clustersSequencePos.length();
	if (pointCount == 0)
	{
		return;
	}

	MColor wireColor = MColor(1.0f, 0.1f, 0.1f, 1.0f);
	glBegin(GL_LINE_LOOP);
	for (unsigned int i = 0; i < pointCount; ++i) {
		glVertex3f(clustersSequencePos[i].x, clustersSequencePos[i].y, clustersSequencePos[i].z);
	}
	glEnd();

	glPopAttrib();
}

/*
//-- timeUpdade ----------------------------
void pclCluster::timeUpdate()
{
	//MGlobal::displayInfo(MString("** timeUpdate ** ") + eval_state);
	
	if (eval_state == false)
	{
		mParticle_sys.setObject(mParticle_path);

		eval_state = true;
		
	}

	//if (eval_state4 == false)
	//{
	//	eval_state4 = true;
	//}
}
*/


bool pclCluster::isBounded() const
{
	return true;
}


bool pclCluster::isTransparent() const
{
	return true;
}


MBoundingBox pclCluster::boundingBox() const
{
	MPoint corner1;
	MPoint corner2;
	MBoundingBox bbox;
	//MGlobal::displayInfo(MString("mBoundingBox_scale[0] ") + mBoundingBox_scale[0]);

	if (mParticle_sys.name() != "")
	{
		mParticle_sys.boundingBox();
		corner1 = mParticle_sys.boundingBox().min();
		corner2 = mParticle_sys.boundingBox().max();
	}
	
	else
	{
		corner1 = MPoint(-5* mBoundingBox_scale[0], 0* mBoundingBox_scale[1], -5* mBoundingBox_scale[2]);
		corner2 = MPoint(5* mBoundingBox_scale[0], 5* mBoundingBox_scale[1], 5* mBoundingBox_scale[2]);
	}

	bbox.expand(corner1);
	bbox.expand(corner2);


	return bbox;

}


//-- Ex: Setting an output attribute for each corresponding input attribute -----
MStatus pclCluster::jumpToElement(MArrayDataHandle& hArray, unsigned int index)
{
	MStatus status;
	status = hArray.jumpToElement(index);
	if (MFAIL(status))
	{
		//MGlobal::displayInfo(MString() + "** jumpToElement ** ");
		MArrayDataBuilder builder = hArray.builder(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		builder.addElement(index, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = hArray.set(builder);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = hArray.jumpToElement(index);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	return status;
}


//-- set Dependents Dirty -------------------- -
MStatus pclCluster::setDependentsDirty(const MPlug& plugBeingDirtied, MPlugArray& affectedPlugs)
{
	MStatus status;

	// Check if createCurvs attribute changed
	if (plugBeingDirtied.partialName() == "createCurvs") {
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
	}

	// Check if nextFrameSearchRadius attribute changed
	if (plugBeingDirtied.partialName() == "nextFrameSearchRadius") {
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
	}

	// Check if samplingRate attribute changed
	if (plugBeingDirtied.partialName() == "samplingRate") {
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
	}

	//-- check if "particleSysMatrix" is update ---------------------
	if (plugBeingDirtied.partialName() == "Action") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
		eval_state = true;
	}


	//-- check if "inTime" is update ---------------------
	if (plugBeingDirtied.partialName() == "inTime") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
		eval_state = true;
	}

	//-- check if "particleSysMatrix" is update ---------------------
	if (plugBeingDirtied.partialName() == "particleSysMatrix") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
		eval_state = true;
	}


	// Check if inBoundingBoxCorner attribute changed
	if (plugBeingDirtied.partialName() == "inBoundingBoxCorner") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
		eval_state = true;
	}


	// Check if inPointsPosition attribute changed
	if (plugBeingDirtied.partialName() == "inPointsPosition") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
		eval_state = true;
	}


	// Check if inParticlePosition attribute changed
	if (plugBeingDirtied.partialName() == "inParticlePosition") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
	}


	//-- check if "particleSysMatrix" is update ---------------------
	if (plugBeingDirtied.partialName() == "particleSysMatrix") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
		eval_state = true;
	}

	//-- check if "recursionDepth" is update ---------------------
	if (plugBeingDirtied.partialName() == "recursionDepth") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
		//clusterPosForFrame_map.clear();
		eval_state = true;
	}

	//-- check if "minimumPointCount" is update ---------------------
	if (plugBeingDirtied.partialName() == "minimumPointCount") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
		//clusterPosForFrame_map.clear();
		eval_state = true;
	}

	//-- check if "clusterBoundingBox" is update ---------------------
	if (plugBeingDirtied.partialName() == "clustersGrouping") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		MPlug centerPos(mThisNode, pclCluster::clusterCenterPos_Attr);
		affectedPlugs.append(centerPos);
		eval_state2 = true;
		eval_state3 = true;
	}

	//-- check if "clusterBoundingBox" is update ---------------------
	if (plugBeingDirtied.partialName() == "clusterCenter") {
		MPlug out(mThisNode, pclCluster::out_Attr);
		affectedPlugs.append(out);
		MPlug outPos(mThisNode, pclCluster::outDivBoundingBoxDraw_Attr);
		affectedPlugs.append(outPos);
		eval_state3 = true;
	}	


	return MS::kSuccess;
}



//-- initialize ------------------------------------
MStatus pclCluster::initialize()
{

	MStatus status = MS::kSuccess;

	MFnMatrixAttribute mAttr;
	MFnEnumAttribute eAttr;
	MFnNumericAttribute nAttr;
	MFnTypedAttribute typedAttrFn;
	MFnMatrixAttribute mFnMatrixAtt;
	MFnUnitAttribute uAttr;

	MVectorArray defaultVectArray;
	MFnVectorArrayData vectArrayDataFn;
	MFnPointArrayData pointArrayDataFn;
	MFnPointArrayData bbCorArrayDataFn;
	MPointArray defaultPointArray;
	MPointArray defaultCrossPointArray;
	

	//-- set Saperator attr ---------------------------------------------
	saperator_Attr = eAttr.create("========", "=======");
	eAttr.addField("======", 0);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(saperator_Attr);

	//-- recursion Depth attr ---------------------------------------------------------
	pointsSampling_Attr = nAttr.create("samplingRate", "samplingRate", MFnNumericData::kInt, 1);
	nAttr.setMin(1);
	nAttr.setKeyable(true);
	nAttr.setStorable(true);
	nAttr.setWritable(true);
	nAttr.setChannelBox(true);
	nAttr.setConnectable(true);
	addAttribute(pointsSampling_Attr);


	//-- create points from mesh  ---------------------------------------------------
	action_Attr = eAttr.create("Action", "Action");
	eAttr.addField("off", 0);
	eAttr.addField("srart", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(action_Attr);


	//-- recursion Depth attr ---------------------------------------------------------
	recursionDepth_Attr = nAttr.create("recursionDepth", "recursionDepth", MFnNumericData::kInt, 2);
	nAttr.setMin(1);
	nAttr.setKeyable(true);
	nAttr.setStorable(true);
	nAttr.setWritable(true);
	nAttr.setChannelBox(true);
	nAttr.setConnectable(true);
	addAttribute(recursionDepth_Attr);


	//-- minimumPointCount attr ---------------------------------------------------------
	minimumPointCount_Attr = nAttr.create("minimumPointCount", "minimumPointCount", MFnNumericData::kInt, 10);
	nAttr.setMin(1);
	nAttr.setKeyable(true);
	nAttr.setStorable(true);
	nAttr.setWritable(true);
	nAttr.setChannelBox(true);
	nAttr.setConnectable(true);
	addAttribute(minimumPointCount_Attr);


	//-- clusterBoundingBox_Attr ---------------------------------------------------
	clustersGrouping_Attr = eAttr.create("clustersGrouping", "clustersGrouping");
	eAttr.addField("off", 0);
	eAttr.addField("on", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(clustersGrouping_Attr);


	//-- clusterBoundingBox_Attr ---------------------------------------------------
	clusterCenter_Attr = eAttr.create("clusterGroupingCenter", "clusterGroupingCenter");
	eAttr.addField("boundingBox", 0);
	eAttr.addField("pointCloud", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(clusterCenter_Attr);


	//-- clusterDrawScale_Attr -------------------------
	clusterDrawScale_Attr = nAttr.create("clusterDrawScale", "clusterDrawScale", MFnNumericData::kDouble, 5.0);
	nAttr.setKeyable(true);
	nAttr.setMin(0);
	addAttribute(clusterDrawScale_Attr);


	//-- createClusterPath Attr  ---------------------------------------------------
	createClusterPath_Attr = eAttr.create("createClusterPath", "createClusterPath");
	eAttr.addField("off ", 0);
	eAttr.addField("on", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(createClusterPath_Attr);

	
	//-- nextFrameSearchRadius -------------------------
	nextFrameSearchRadius_Attr = nAttr.create("nextFrameSearchRadius", "nextFrameSearchRadius", MFnNumericData::kDouble, 5.0);
	nAttr.setKeyable(true);
	nAttr.setMin(0);
	addAttribute(nextFrameSearchRadius_Attr);


	//-- boundingBoxVis  ---------------------------------------------------
	boundingBoxVis_Attr = eAttr.create("boundingBoxVis", "boundingBoxVis");
	eAttr.addField("on", 0);
	eAttr.addField("off", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(boundingBoxVis_Attr);


	//-- particleSysMatrix Attr ------------------------------------------------------------
	inParticleMatrix_Attr = mFnMatrixAtt.create("particleSysMatrix", "particleSysMatrix");
	mFnMatrixAtt.setReadable(false);
	mFnMatrixAtt.setStorable(false);
	mFnMatrixAtt.setConnectable(true);
	addAttribute(inParticleMatrix_Attr);

	//-- set the in time attr (for time input ) --------------
	inTime_Attr = uAttr.create("inTime", "inTime", MFnUnitAttribute::kTime);
	addAttribute(inTime_Attr);

	//-- inBoundingBox (container) corner  ---------------------------------------------------------------------
	pointArrayDataFn.create(defaultPointArray);
	typedAttrFn.create("inBoundingBoxCorner", "inBoundingBoxCorner", MFnData::kPointArray, pointArrayDataFn.object(), &status);
	typedAttrFn.setWritable(true);
	typedAttrFn.setStorable(true);
	inBoundingBoxCorner_Attr = typedAttrFn.object();
	addAttribute(inBoundingBoxCorner_Attr);


	//-- in Points Position (from pclLoader) --------------------------------------
	pointArrayDataFn.create(defaultPointArray);
	typedAttrFn.create("inPointsPosition", "inPointsPosition", MFnData::kPointArray, pointArrayDataFn.object(), &status);
	typedAttrFn.setWritable(true);
	typedAttrFn.setStorable(true);
	typedAttrFn.setKeyable(false);
	typedAttrFn.setConnectable(true);
	inPointsPosition_Attr = typedAttrFn.object();
	addAttribute(inPointsPosition_Attr);

	//-- in Points Color (from pclLoader, optional) --------------------------------------
	vectArrayDataFn.create(defaultVectArray);
	typedAttrFn.create("inPointsColor", "inPointsColor", MFnData::kVectorArray, vectArrayDataFn.object(), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	typedAttrFn.setWritable(true);
	typedAttrFn.setStorable(true);
	typedAttrFn.setKeyable(false);
	typedAttrFn.setConnectable(true);
	inPointsColor_Attr = typedAttrFn.object();
	addAttribute(inPointsColor_Attr);

	//-- in Particle --------------------------------------
	vectArrayDataFn.create(defaultVectArray);
	typedAttrFn.create("inParticlePosition", "inParticlePosition", MFnData::kVectorArray, vectArrayDataFn.object(), &status);
	typedAttrFn.setWritable(true);
	typedAttrFn.setStorable(true);
	inParticlePP_Attr = typedAttrFn.object();
	addAttribute(inParticlePP_Attr);

	//-- OUTPUT -----------------------------------------------------
	//-- output the number of clusters Detection ---------------------------------------------------------
	clustersDetection_Attr = nAttr.create("clustersDetection ", "clustersDetection", MFnNumericData::kInt, 0);
	nAttr.setKeyable(true);
	nAttr.setStorable(false);
	nAttr.setWritable(true);
	nAttr.setChannelBox(true);
	nAttr.setConnectable(true);
	addAttribute(clustersDetection_Attr);

	//***********************************************************
	createCurvs_Attr = eAttr.create("createCurvs", "createCurvs");
	eAttr.addField("off", 0);
	eAttr.addField("on", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(createCurvs_Attr);
	//******************************************************


	//-- cluster Center Pos Array to connect loacators --------------------------
	clusterCenterPos_Attr = nAttr.createPoint("clusterCenterPos", "clusterCenterPos");
	nAttr.setStorable(true);
	nAttr.setWritable(true);
	nAttr.setChannelBox(false);
	nAttr.setConnectable(true);
	nAttr.setArray(true);
	nAttr.setIndexMatters(false);
	nAttr.setUsesArrayDataBuilder(true);
	addAttribute(clusterCenterPos_Attr);


	//-- divide boundingBoxs data --------------------------------
	pointArrayDataFn.create(defaultPointArray);
	typedAttrFn.create("outPositionDraw", "outPositionDraw", MFnData::kPointArray, pointArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outDivBoundingBoxDraw_Attr = typedAttrFn.object();
	addAttribute(outDivBoundingBoxDraw_Attr);

	//-- Cross data --------------------------------
	pointArrayDataFn.create(defaultCrossPointArray);
	typedAttrFn.create("outCrossDraw", "outCrossDraw", MFnData::kPointArray, pointArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outCrossDraw_Attr = typedAttrFn.object();
	addAttribute(outCrossDraw_Attr);


	//-- ClustersSequencePath data--------------------------------
	pointArrayDataFn.create(defaultCrossPointArray);
	typedAttrFn.create("outClustersSequencePathDraw", "outClustersSequencePathDraw", MFnData::kPointArray, pointArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	typedAttrFn.setArray(true);
	typedAttrFn.setIndexMatters(true);
	typedAttrFn.setUsesArrayDataBuilder(true);
	outClustersSequencePathDraw_Attr = typedAttrFn.object();
	addAttribute(outClustersSequencePathDraw_Attr);

	//-- set the out attr ---------------------------------------------
	out_Attr = nAttr.create("out", "out", MFnNumericData::kInt, 0);
	addAttribute(out_Attr);



	return MS::kSuccess;
}




// -------------------------------------------------------------------------- -
//---------------------------------------------------------------------------
// Viewport 2.0 override implementation
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

PclClusterDrawOverride::PclClusterDrawOverride(const MObject& obj)
	: MHWRender::MPxDrawOverride(obj, PclClusterDrawOverride::draw)
{
}

PclClusterDrawOverride::~PclClusterDrawOverride()
{
}

//-- Add boundingBox -----------------------------------------
MHWRender::DrawAPI PclClusterDrawOverride::supportedDrawAPIs() const
{
	// this plugin supports both GL and DX
	return (MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile);
}



bool PclClusterDrawOverride::isBounded(const MDagPath& /*objPath*/, const MDagPath& /*cameraPath*/) const
{
	return true;
}


//-- boundingBox size -----------------------------------------
MBoundingBox PclClusterDrawOverride::boundingBox(const MDagPath& objPath) const
{

	MObject pcllClusterNode = objPath.node();
	MFnDagNode dagThis(pcllClusterNode);

	//-- check boundingBox  ------------------------------
	MObject matrix_obj = dagThis.findPlug(pclCluster::inParticleMatrix_Attr).asMObject();
	MPoint corner1(-2.5, -2.5, -2.5);
	MPoint corner2(2.5, 2.5, 2.5);
	MBoundingBox bbox;
	if (matrix_obj.isNull() != NULL)
	{
		MFnTransform particleSysTrans;
		particleSysTrans.setObject(matrix_obj);
		MTransformationMatrix sysTransMatrix;
		particleSysTrans.set(sysTransMatrix);
		bbox.transformUsing(sysTransMatrix.asMatrix());
		bbox.expand(corner1);
		bbox.expand(corner2);
	}
	
	bbox.expand(corner1);
	bbox.expand(corner2);

	return bbox;
	
}

//-- call get_loadFile_Attr ------------------ -
bool PclClusterDrawOverride::get_boundingBoxVis(const MDagPath& objPath) const
{
	MStatus status;
	if (status)
	{
		MObject pclLoaderNode = objPath.node();
		MFnDagNode dagThis(pclLoaderNode);
		bool boundingBoxVis  = dagThis.findPlug(pclCluster::boundingBoxVis_Attr).asBool();
		return boundingBoxVis;
	}

	return 0;
}



//-- call get_createClusterPath  ------------------ -
bool PclClusterDrawOverride::get_createClusterPath(const MDagPath& objPath) const
{
	MStatus status;
	if (status)
	{
		MObject pclLoaderNode = objPath.node();
		MFnDagNode dagThis(pclLoaderNode);
		bool createClusterPath = dagThis.findPlug(pclCluster::createClusterPath_Attr).asBool();
		return createClusterPath;
	}

	return 0;
}


//-- call outDivBoundingBoxDraw_Attr info ----------------------------------
MPointArray PclClusterDrawOverride::get_outDivBBoxPP(const MDagPath& objPath) const
{
	MStatus status;
	
	MFnPointArrayData dataPointArrayFn;
	MPointArray bpPoint_list;
	if (status)
	{
		MObject pcllClusterNode = objPath.node(&status);
		MFnDagNode dagThis(pcllClusterNode);

		//-- check contanerOnOff_Attr Attr ------------------------------
		MObject plug_aPoints = dagThis.findPlug(pclCluster::outDivBoundingBoxDraw_Attr).asMObject();
		dataPointArrayFn.setObject(plug_aPoints);
		if (status == MS::kSuccess)
		{
			bpPoint_list = dataPointArrayFn.array();
		}
	}

	return bpPoint_list;
}


// -- call Cross points_Info ---------------------------------
MPointArray PclClusterDrawOverride::get_outCrossPP(const MDagPath & objPath) const
{
	MStatus status;

	MFnPointArrayData dataPointArrayFn;
	MPointArray bpPoint_list;
	if (status)
	{
		MObject pcllClusterNode = objPath.node(&status);
		MFnDagNode dagThis(pcllClusterNode);

		//-- check contanerOnOff_Attr Attr ------------------------------
		MObject plug_aPoints = dagThis.findPlug(pclCluster::outCrossDraw_Attr).asMObject();
		dataPointArrayFn.setObject(plug_aPoints);
		if (status == MS::kSuccess)
		{
			bpPoint_list = dataPointArrayFn.array();
		}
	}

	return bpPoint_list;
}



// -- call clustersSequencePath Info ---------------------------------
std::vector<MPointArray> PclClusterDrawOverride::get_clustersSequencePath(const MDagPath& objPath) const
{

	MStatus status;
	MObject pclCluster_node = objPath.node();
	MFnPointArrayData dataPointArrayFn;
	std::vector<MPointArray> clustersPathList_vec;
	if (status)
	{
		//-- check contanerOnOff_Attr Attr ------------------------------
		MPlug pclClusterSequencePath_plug(pclCluster_node, pclCluster::outClustersSequencePathDraw_Attr);
		unsigned int count = pclClusterSequencePath_plug.numElements();
		
		for (unsigned int c = 0; c < count; c++)
		{
			MObject plug_aPoints = pclClusterSequencePath_plug.elementByPhysicalIndex(c).asMObject();
			dataPointArrayFn.setObject(plug_aPoints);
			if (status == MS::kSuccess)
			{
				MPointArray clustersPath_list = dataPointArrayFn.array();
				clustersPathList_vec.push_back(clustersPath_list);
			}	
		}	
	}

	return clustersPathList_vec;
}


//-- call get_clusterDrawScale ----------------------------------
float PclClusterDrawOverride::get_clusterDrawScale(const MDagPath& objPath) const
{
	MStatus status;
	float clusterDrawScale = 1;
	if (status)
	{
		MObject pclContainerNode = objPath.node(&status);
		MFnDagNode dagThis(pclContainerNode);

		//-- check contanerOnOff_Attr Attr ------------------------------
		clusterDrawScale = dagThis.findPlug(pclCluster::clusterDrawScale_Attr).asFloat();
	}

	return clusterDrawScale;
}


//-- call get_nextFrameSearchRadius ----------------------------------
float PclClusterDrawOverride::get_nextFrameSearchRadius(const MDagPath& objPath) const
{
	MStatus status;
	float searchRadius = 1;
	if (status)
	{
		MObject pclContainerNode = objPath.node(&status);
		MFnDagNode dagThis(pclContainerNode);

		//-- check contanerOnOff_Attr Attr ------------------------------
		searchRadius = dagThis.findPlug(pclCluster::nextFrameSearchRadius_Attr).asFloat();

	}

	return searchRadius;
}


// Called by Maya each time the object needs to be drawn.
MUserData* PclClusterDrawOverride::prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath, const MHWRender::MFrameContext& frameContext, MUserData* oldData)
{
	PclClusterData* data = dynamic_cast<PclClusterData*>(oldData);
	if (!data)
	{
		data = new PclClusterData();
	}
	MFnNumericData fnData();


	//--  get correct color and depth priority based on the state of object, e.g. active or dormant ----
	switch (MHWRender::MGeometryUtilities::displayStatus(objPath))
	{
	case MHWRender::kLead:
	case MHWRender::kActive:
	case MHWRender::kHilite:
	case MHWRender::kActiveComponent:
		data->fDepthPriority = MHWRender::MRenderItem::sActiveWireDepthPriority;
		break;
	default:
		data->fDepthPriority = MHWRender::MRenderItem::sDormantFilledDepthPriority;
		break;
	}


	//-- call _boundingBoxVis ----------------------------------
	bool boundingBoxVis = get_boundingBoxVis(objPath);

	//-- call createClusterPath Position ----------
	bool createClusterPath = get_createClusterPath(objPath);

	//-- call get out div boundingBox Position ----------
	MPointArray bpDivPoint_list = get_outDivBBoxPP(objPath);

	//-- call get out Cross Position ----------
	MPointArray crossPoint_list = get_outCrossPP(objPath);
	
	//-- call get_clustersSequencePath Position ----------
	std::vector<MPointArray> clustersSequencePath_vec = get_clustersSequencePath(objPath);

	//-- call get clusterDrawScale ----------
	float clusterDrawScale = get_clusterDrawScale(objPath);
	
	//-- get_nextFrameSearchRadius ---------------------
	float nextFrameSearchRadius = get_nextFrameSearchRadius(objPath);

	//---append to list ---------------------------
	data->fBoundingBoxVis = boundingBoxVis;

	data->fCreateClusterPath = createClusterPath;

	data->fBpDivPoint_list.clear();
	data->fBpDivPoint_list = bpDivPoint_list;

	data->fCrossPoint_list.clear();
	data->fCrossPoint_list = crossPoint_list;

	data->fClustersSequencePath_vec.clear();
	data->fClustersSequencePath_vec = clustersSequencePath_vec;

	data->fClusterDrawScale = clusterDrawScale;

	data->fNextFrameSearchRadius = nextFrameSearchRadius;



	return data;
}


// addUIDrawables() provides access to the MUIDrawManager, which can be used
// to queue up operations for drawing simple UI elements such as lines, circles and
// text. To enable addUIDrawables(), override hasUIDrawables() and make it return true.
void PclClusterDrawOverride::addUIDrawables(const MDagPath& objPath, MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext, const MUserData* data)
{
	//MGlobal::displayInfo(MString() + "addUIDrawables");
	// Get data cached by prepareForDraw() for each drawable instance, then MUIDrawManager 
	// can draw simple UI by these data.
	PclClusterData* pLocatorData = (PclClusterData*)data;
	if (!pLocatorData)
	{
		return;
	}


	//-- get div BpPoint_list  -----------------
	MPointArray bpDivPoint_list = pLocatorData->fBpDivPoint_list;

	//-- get crossPoint_list -----------
	MPointArray crossPoint_list = pLocatorData->fCrossPoint_list;

	//-- get ClustersSequence Path list -----------
	std::vector<MPointArray> clustersSequencePath_vec = pLocatorData->fClustersSequencePath_vec;

	//-- get BoundingBox Vis -----------
	bool BoundingBoxVis = pLocatorData->fBoundingBoxVis;

	//-- get createClusterPath vis -----------
	bool createClusterPath = pLocatorData->fCreateClusterPath;

	//-- get clusterDrawScale -----------
	float clusterDrawScale = pLocatorData->fClusterDrawScale;

	//-- get nextFrameSearchRadius -----------
	float nextFrameSearchRadius = pLocatorData->fNextFrameSearchRadius;

	// Draw bounding boxes
	if (BoundingBoxVis == false)
	{
		MColor solidColor = MColor(1, 1, 0.133, 1);
		drawManager.beginDrawable();
		drawManager.setColor(solidColor);
		drawManager.setDepthPriority(2);
		drawManager.setLineWidth(1);
		drawManager.lineList(bpDivPoint_list, false);
		drawManager.endDrawable();
		
	}

	// Draw cross markers at cluster centers
	MPointArray crossPos_list;
	MColor crossdColor = MColor(0.9, 0.2, 0.2, 1);
	const unsigned int crossCount = crossPoint_list.length();
	for (unsigned int i = 0; i < crossCount; ++i) 
	{
		MPoint posA(crossPoint_list[i].x - (2* clusterDrawScale), crossPoint_list[i].y, crossPoint_list[i].z);
		MPoint posB(crossPoint_list[i].x + (2* clusterDrawScale), crossPoint_list[i].y, crossPoint_list[i].z);
		crossPos_list.append(posA);
		crossPos_list.append(posB);

		MPoint posC(crossPoint_list[i].x, crossPoint_list[i].y - (2 * clusterDrawScale), crossPoint_list[i].z);
		MPoint posD(crossPoint_list[i].x, crossPoint_list[i].y + (2 * clusterDrawScale), crossPoint_list[i].z);
		crossPos_list.append(posC);
		crossPos_list.append(posD);

		MPoint posE(crossPoint_list[i].x, crossPoint_list[i].y, crossPoint_list[i].z - (2 * clusterDrawScale));
		MPoint posF(crossPoint_list[i].x, crossPoint_list[i].y, crossPoint_list[i].z + (2 * clusterDrawScale));
		crossPos_list.append(posE);
		crossPos_list.append(posF);

		drawManager.beginDrawable();
		drawManager.setColor(crossdColor);
		drawManager.setDepthPriority(3);
		drawManager.setLineWidth(1);
		drawManager.lineList(crossPos_list, false);
		drawManager.endDrawable();
	}



	// Draw cluster sequence paths
	if (createClusterPath == true)
	{		
		MColor curveColor = MColor(0.3, 0.3, 1);
		const size_t clusterCount = clustersSequencePath_vec.size();
		for (size_t c = 0; c < clusterCount; ++c)
		{
			const unsigned int pathLength = clustersSequencePath_vec[c].length();
			if (pathLength < 2)
			{
				continue; // Need at least 2 points to draw a line
			}

			drawManager.beginDrawable();
			drawManager.setColor(curveColor);
			drawManager.setDepthPriority(5);
			drawManager.setLineWidth(2);

			for (unsigned int p = 0; p < pathLength - 1; ++p)
			{
				drawManager.line(clustersSequencePath_vec[c][p], clustersSequencePath_vec[c][p + 1]);
			}

			drawManager.endDrawable();
		}
	}
		

	// Draw search radius disks
	if (BoundingBoxVis == false)
	{
		MVector norm = MVector(0, 1, 0);
		MColor solidColor = MColor(1, 1, 0.133, 0.1);
		drawManager.beginDrawable();
		drawManager.setColor(solidColor);
		drawManager.setDepthPriority(1);

		const unsigned int crossCount = crossPoint_list.length();
		for (unsigned int i = 0; i < crossCount; ++i)
		{
			drawManager.circle(crossPoint_list[i], norm, nextFrameSearchRadius, true);
		}

		drawManager.endDrawable();
	}
}

