#include "pclLoader.h"
#include "pclUtilities.h"
#include "core/pclGeometryUtils.h"

// Standard library
#include <limits>
#include <thread>

// Maya API
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MFnVectorArrayData.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MSelectionList.h>
#include <maya/MGlobal.h>



MTypeId PclLoader::id(0x0000011626111);
MString	PclLoader::drawDbClassification("drawdb/geometry/PclLoader_GeometryOverride");
MString	PclLoader::drawRegistrantId("PclLoaderNode_GeometryOverridePlugin");

MObject PclLoader::saperator_Attr;
MObject PclLoader::mainPath_Attr;
MObject PclLoader::containerOnOff_Attr;
MObject PclLoader::startFrame_Attr;
MObject PclLoader::endFrame_Attr;
MObject PclLoader::loadStartFrame_Attr;
MObject PclLoader::loadEndFrame_Attr;
MObject PclLoader::skipFilesRows_Attr;
MObject PclLoader::loadFile_Attr;
MObject PclLoader::displayPoints_Attr;
MObject PclLoader::pointsSize_Attr;
MObject PclLoader::pointNumber_Attr;
MObject PclLoader::inContainerMatrix_Attr;
MObject PclLoader::inlRaycasterPoints_Attr;
MObject PclLoader::inTime_Attr;
MObject PclLoader::timeOffset_Attr;

MObject PclLoader::outBoundingBoxCorner_Attr;
MObject PclLoader::outDrawContainer_Attr;
MObject PclLoader::outPosition_Attr;
MObject PclLoader::outColor_Attr;
MObject PclLoader::outPositionMaxFrame_Attr;
MObject PclLoader::outPositionVec_Attr;
MObject PclLoader::outColorVec_Attr;
MObject PclLoader::theHighestNumberOfPoints_Attr;
MObject PclLoader::out_Attr;


// Basic ctor/dtor/creator scaffolding.
PclLoader::PclLoader()
{}

PclLoader::~PclLoader()
{}

void* PclLoader::creator()
{
	return new PclLoader();
}


// Assign shape name after DG construction.
void PclLoader::postConstructor()
{
	mThisNode = thisMObject();

	MFnDependencyNode fnSequencer(mThisNode);
	MFnDagNode this_dag(mThisNode);
	fnSequencer.setName("pclLoaderShape#");
	
}


// Entry point triggered whenever Maya dirties any plug on this node.
MStatus PclLoader::compute(const MPlug& plug, MDataBlock& dataBlock)
{
	MStatus status = MS::kSuccess;

	// Gather all frequently-used inputs at the start for clarity.
	EvaluateContext ctx;
	ctx.mainPath = dataBlock.inputValue(mainPath_Attr).asString();
	ctx.loadFileMode = dataBlock.inputValue(loadFile_Attr).asShort();
	ctx.loadStartFrame = dataBlock.inputValue(loadStartFrame_Attr).asInt();
	ctx.loadEndFrame = dataBlock.inputValue(loadEndFrame_Attr).asInt();
	ctx.skipRows = dataBlock.inputValue(skipFilesRows_Attr).asInt();
	ctx.displayPoints = dataBlock.outputValue(displayPoints_Attr).asShort();
	ctx.containerOnOff = dataBlock.inputValue(containerOnOff_Attr).asShort();
	ctx.pointSize = dataBlock.outputValue(pointsSize_Attr).asFloat();
	ctx.containerMatrix = dataBlock.inputValue(inContainerMatrix_Attr).asMatrix();
	ctx.timeOffset = dataBlock.inputValue(timeOffset_Attr).asInt();

	// Refresh particle connection state each evaluation so downstream outputs stay correct.
	MFnDependencyNode fnNode(mThisNode);
	mParticlesConnected = fnNode.findPlug(PclLoader::theHighestNumberOfPoints_Attr).isConnected();

	// Mirror frequently-used inputs into member variables for legacy helpers.
	mLoadFile = ctx.loadFileMode;
	mContainerOnOff = ctx.containerOnOff;
	mDisplayPoints = ctx.displayPoints;
	mPointsSize = ctx.pointSize;

	//-- set default Container_bBox -----------------
	if (mContainerState.defaultBox)
	{
		//-- set Default container boundingBox --------
		mContainerState.lastMin = MPoint(10, 10, 10);
		mContainerState.lastMax = MPoint(-10, 0, -10);
		mContainerState.defaultBox = false;
	}


	//-- get the ContanerOnOff plug ---------------------
	if (theContanerOnOff_plug.name() == "")
	{
		getTheContainer_plugj();
	}


	//-- check if mainPath exist and if the main path change --------------------
	if (mMainPath != ctx.mainPath)
	{
		clearLoadedData();
		mMainPath = ctx.mainPath;
	}

	if (ctx.mainPath.length() == 0)
	{
		clearLoadedData();
		dataBlock.outputValue(startFrame_Attr).setInt(0);
		dataBlock.outputValue(endFrame_Attr).setInt(0);
		dataBlock.outputValue(loadStartFrame_Attr).setInt(0);
		dataBlock.outputValue(loadEndFrame_Attr).setInt(0);
		dataBlock.outputValue(pointNumber_Attr).setInt(0);
		dataBlock.outputValue(theHighestNumberOfPoints_Attr).setInt(0);
		mContainerState.reset();
	}
	else if (!mLoadOnes && ctx.skipRows != mLoadedSkipRows)
	{
		clearLoadedData();
	}

	clampLoadWindow(ctx, dataBlock);

	
	status = processLoadRequest(ctx, dataBlock);
	if (status != MS::kSuccess)
	{
		return status;
	}

	// Re-evaluate only when time offset changes.
	if (mLastTimeVal != ctx.timeOffset)
	{
		eval_state = true;
		mLastTimeVal = ctx.timeOffset;
	}

	if (eval_state == true)
	{
		updateFrameOutputs(ctx, dataBlock);
		eval_state = false;
	}

	updateBoundingBoxOutputs(ctx, dataBlock);
	updateRaycasterInputs(dataBlock);

	//--set output number of endFrame----------------------------------
	dataBlock.outputValue(out_Attr).setInt(ctx.timeOffset);


	eval_state = false;



	return status;
}


//-- filter the Points withboundingBox - Container ----------------------------------------------------
void PclLoader::filterPoints(const MPointArray& sourcePos,
                             const MPointArray& sourceCol,
                             const MBoundingBox& bbox,
                             bool keepInside,
                             const MVectorArray* sourcePosVec,
                             const MVectorArray* sourceColVec,
                             MPointArray& filteredPos,
                             MPointArray& filteredCol,
                             MVectorArray* filteredPosVec,
                             MVectorArray* filteredColVec)
{
	if (sourcePos.length() == 0)
	{
		return;
	}

	std::vector<Eigen::Vector3d> eigenPoints;
	const unsigned int pointCount = sourcePos.length();
	eigenPoints.reserve(pointCount);
	for (unsigned int i = 0; i < pointCount; ++i)
	{
		const MPoint& point = sourcePos[i];
		eigenPoints.emplace_back(point.x, point.y, point.z);
	}

	const MPoint bboxMin = bbox.min();
	const MPoint bboxMax = bbox.max();
	const auto coreBox = pclCore::BoundingBox::FromMinMax(
		Eigen::Vector3d(bboxMin.x, bboxMin.y, bboxMin.z),
		Eigen::Vector3d(bboxMax.x, bboxMax.y, bboxMax.z));

	const std::vector<size_t> indices = pclCore::filterPointIndices(coreBox, eigenPoints, keepInside);
	for (size_t idx : indices)
	{
		const unsigned int i = static_cast<unsigned int>(idx);
		filteredPos.append(sourcePos[i]);
		filteredCol.append(sourceCol[i]);

		if (filteredPosVec && sourcePosVec)
		{
			filteredPosVec->append((*sourcePosVec)[i]);
		}
		if (filteredColVec && sourceColVec)
		{
			filteredColVec->append((*sourceColVec)[i]);
		}
	}
}



//-- this function is for connected particle system - add Extra Points To Frames With Low Point Number - all frames with the same point count ---------------------------
MStatus PclLoader::addExtraPointsToFramesWithLowPointNumber()
{
	MStatus status;

	//unsigned int theHighestNumberOfPoints = 0;
	//-- get the Highest number of points from all the frames - (the parameter is intended only for particle system output) ---------
	for (unsigned int p = 0; p < mPointArrayPosAllFrameVec_list.size(); ++p)
	{
		if (mTheHighestNumberOfPoints < mPointArrayPosAllFrameVec_list[p].length())
		{
			mTheHighestNumberOfPoints = mPointArrayPosAllFrameVec_list[p].length();
		}
	}

	//-- loop each frame and check the point count --------------------------------------------
	for (unsigned int f = 0; f < mPointArrayPosAllFrameVec_list.size(); ++f)
	{
		unsigned int pointCount = mPointArrayPosAllFrameVec_list[f].length();

		// If necessary, pad the frame arrays to match the highest point count.
		if (pointCount < mTheHighestNumberOfPoints)
		{
			unsigned int pointDif = mTheHighestNumberOfPoints - pointCount;
			const unsigned int currentSize = mPointArrayPosAllFrameVec_list[f].length();
			mPointArrayPosAllFrameVec_list[f].setLength(currentSize + pointDif);
			mPointArrayColAllFrameVec_list[f].setLength(currentSize + pointDif);
			
			const MVector zeroPoint(0, 0, 0);
			for (unsigned int p = 0; p < pointDif; ++p)
			{
				mPointArrayPosAllFrameVec_list[f].set(zeroPoint, currentSize + p);
				mPointArrayColAllFrameVec_list[f].set(zeroPoint, currentSize + p);
			}
		}
	}
	
	return MS::kSuccess;

}

//-- find the BoundingBoxCorners ---------------------------------------------
void PclLoader::getBoundingBoxCornersFromPCL(MPointArray pointArrayPos_list)
{
	float minVal_x = 10000;
	float minVal_y = 10000;
	float minVal_z = 10000;
	float maxVal_x = -10000;
	float maxVal_y = -10000;
	float maxVal_z = -10000;

	MPointArray boundingBoxCorners_list;
	// Loop through all points to find bounding box corners.
	const unsigned int pointCount = pointArrayPos_list.length();
	for (unsigned int p = 0; p < pointCount; ++p)
	{
		//== MIN ============================
		//-- find min x ----------------------
		if (pointArrayPos_list[p].x < minVal_x)
		{
			minVal_x = pointArrayPos_list[p].x;
		}

		//-- find min y ----------------------
		if (pointArrayPos_list[p].y < minVal_y)
		{
			minVal_y = pointArrayPos_list[p].y;
		}

		//-- find min z ----------------------
		if (pointArrayPos_list[p].z < minVal_z)
		{
			minVal_z = pointArrayPos_list[p].z;
		}

		//== MAX =============================
		//-- find max x ----------------------
		if (pointArrayPos_list[p].x > maxVal_x)
		{
			maxVal_x = pointArrayPos_list[p].x;
		}

		//-- find max y ----------------------
		if (pointArrayPos_list[p].y > maxVal_y)
		{
			maxVal_y = pointArrayPos_list[p].y;
		}

		//-- find max z ----------------------
		if (pointArrayPos_list[p].z > maxVal_z)
		{
			maxVal_z = pointArrayPos_list[p].z;
		}
	}

	MPoint minVal(minVal_x, minVal_y, minVal_z);
	MPoint maxVal(maxVal_x, maxVal_y, maxVal_z);
	boundingBoxCorners_list.append(minVal);
	boundingBoxCorners_list.append(maxVal);
	mBoundingBoxCorner_list.push_back(boundingBoxCorners_list);


}

//-- Draw in viewport 1.0 -----------------------------------------------------------
void PclLoader::draw(M3dView& view, const MDagPath& DGpath, M3dView::DisplayStyle style, M3dView::DisplayStatus status)
{
	beginViewportDraw(view);
	drawPointsPrimitive();
	drawContainerPrimitive(view);
	endViewportDraw(view);
}

void PclLoader::beginViewportDraw(M3dView& view) const
{
	view.beginGL();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glClearDepth(1.0);
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthMask(GL_FALSE);
}

void PclLoader::drawPointsPrimitive() const
{
	if ((mDisplayPoints == 1 || mDisplayPoints == 2) && mPointPosFrameOut_list.length() > 0)
	{
		const unsigned int pointCount = mPointPosFrameOut_list.length();
		for (unsigned int p = 0; p < pointCount; ++p)
		{
			MColor pointCol = MColor(mPointColFrameOut_list[p].x, mPointColFrameOut_list[p].y, mPointColFrameOut_list[p].z);
			drawPoints(mPointPosFrameOut_list[p], pointCol, mPointsSize);
		}
	}
}

// Writes an MPointArray into the provided output plug.
void PclLoader::setPointArrayOutput(MDataBlock& dataBlock, MObject attr, const MPointArray& values)
{
	MFnPointArrayData fn;
	MObject dataObj = dataBlock.outputValue(attr).data();
	if (fn.setObject(dataObj) == MS::kSuccess)
	{
		fn.set(values);
	}
}

// Writes an MVectorArray into the provided output plug.
void PclLoader::setVectorArrayOutput(MDataBlock& dataBlock, MObject attr, const MVectorArray& values)
{
	MFnVectorArrayData fn;
	MObject dataObj = dataBlock.outputValue(attr).data();
	if (fn.setObject(dataObj) == MS::kSuccess)
	{
		fn.set(values);
	}
}

void PclLoader::drawContainerPrimitive(M3dView& view) const
{
	if (mContainerState.drawLines.length() > 0)
	{
		MColor pointCol(1.0f, 0.2f, 0.9f);
		drawContaineBoundingBox(view, mContainerState.drawLines, pointCol);
	}
}

void PclLoader::endViewportDraw(M3dView& view) const
{
	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);
	glPopAttrib();
	view.endGL();
}


//-- draw the Points -----------------------------------------
void PclLoader::drawPoints(MFloatPoint pointPos, MColor pointCol, float pointsSize)
{
	glColor4f(pointCol.r, pointCol.g, pointCol.b, 1);
	glPointSize(pointsSize);
	glDisable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	glVertex3f(pointPos.x, pointPos.y, pointPos.z);
	glEnd();
}


//-- draw the BoundingBox----------------
void PclLoader::drawContaineBoundingBox(M3dView & view, MPointArray boundingBoxsToDrawInfo, MColor pointCol)
{
	glColor4f(pointCol.r, pointCol.g, pointCol.b, 1);
	glBegin(GL_LINES);
	const unsigned int lineCount = boundingBoxsToDrawInfo.length();
	for (unsigned int i = 0; i + 1 < lineCount; i += 2)
	{
		glVertex3f(boundingBoxsToDrawInfo[i].x, boundingBoxsToDrawInfo[i].y, boundingBoxsToDrawInfo[i].z);
		glVertex3f(boundingBoxsToDrawInfo[i + 1].x, boundingBoxsToDrawInfo[i + 1].y, boundingBoxsToDrawInfo[i + 1].z);
	}
	glEnd();
}


//-- edit keyframe on strat and end frame on timeOffset attr  -----------------------
MStatus PclLoader::editKeyframesOnTimeAttr(unsigned int loadStartFrame, unsigned int loadEndFrame)
{
	MStatus status = MS::kSuccess;

	//-- set the global  time line ------------------------------
	MAnimControl animControl;
	MTime sTime, eTime;
	sTime.setValue(loadStartFrame);
	animControl.setMinTime(sTime);

	eTime.setValue(loadEndFrame);
	animControl.setMaxTime(eTime);
	
	
	//-- editstart & end kyeframes -------------------------------
	MDagModifier dagMod;
	MFnDagNode dagNodeFn_thisNode;
	dagNodeFn_thisNode.setObject(mThisNode);
	
	MString startFrame, endFrame;
	startFrame = loadStartFrame;
	endFrame = loadEndFrame;

	//-- edit the start key and the end key -------------------------
	MString command = "keyframe -edit -index 0 -e -timeChange " + startFrame + " -valueChange " + startFrame + " " + dagNodeFn_thisNode.fullPathName() + ".timeOffset;";
	MGlobal::executeCommand(command, false, true);

	command = "keyframe -edit -index 1 -e -timeChange " + endFrame + " -valueChange " + endFrame + " " + dagNodeFn_thisNode.fullPathName() + ".timeOffset;";
	MGlobal::executeCommand(command, false, true);


	
	return status;

}


//-- get the particle_sys from particleDraiver connection -------------------------------
MStatus PclLoader::getTheContainer_plugj()
{

	MStatus status;
	MFnDependencyNode fnPrNode(mThisNode);
	MPlug inContainerMatrix_plug = fnPrNode.findPlug("inContainerMatrix", true);
	MPlugArray mPlugArray;
	inContainerMatrix_plug.connectedTo(mPlugArray, true, true);
	if (mPlugArray.length() == 0) {
		return MS::kInvalidParameter;
	}

	//-- get container to object ------------
	MObject container_obj = mPlugArray[0].node();

	// Get container containerOnOff attribute.
	MFnDagNode nDagNode;
	nDagNode.setObject(container_obj);
	MObject containerShape_obj = nDagNode.child(0);

	fnPrNode.setObject(containerShape_obj);
	theContanerOnOff_plug = fnPrNode.findPlug("containerOnOff", true);
	
	return MS::kSuccess;

}

//-- setDependentsDirty Attr ---------------------------------------
MStatus PclLoader::setDependentsDirty(const MPlug& plugBeingDirtied, MPlugArray& affectedPlugs)
{
	MStatus status;
	MFnDependencyNode fnThisNode(mThisNode);

	//PclLoader::displayColor_Attr

	//-- check if "inTime" is update ---------------------
	if (plugBeingDirtied.partialName() == "inTime") {
		MPlug out(mThisNode, PclLoader::out_Attr);
		affectedPlugs.append(out);
		MPlug outP(mThisNode, PclLoader::outPosition_Attr);
		affectedPlugs.append(outP);
		MPlug outPV(mThisNode, PclLoader::outPositionVec_Attr);
		affectedPlugs.append(outPV);
		MPlug outCV(mThisNode, PclLoader::outColorVec_Attr);
		affectedPlugs.append(outCV);
		MPlug outC(mThisNode, PclLoader::outColor_Attr);
		affectedPlugs.append(outC);
		MPlug bbC(mThisNode, PclLoader::outBoundingBoxCorner_Attr);
		affectedPlugs.append(bbC);
		eval_state = true;
	}
	
	//-- check if "inTimeOffset" is update ---------------------
	if (plugBeingDirtied.partialName() == "timeOffset") {
		MPlug out(mThisNode, PclLoader::out_Attr);
		affectedPlugs.append(out);
		MPlug outP(mThisNode, PclLoader::outPosition_Attr);
		affectedPlugs.append(outP);
		MPlug outPV(mThisNode, PclLoader::outPositionVec_Attr);
		affectedPlugs.append(outPV);
		MPlug outCV(mThisNode, PclLoader::outColorVec_Attr);
		affectedPlugs.append(outCV);
		MPlug outC(mThisNode, PclLoader::outColor_Attr);
		affectedPlugs.append(outC);
		MPlug bbC(mThisNode, PclLoader::outBoundingBoxCorner_Attr);
		affectedPlugs.append(bbC);
		eval_state = true;
	}
	

	//-- loadFile effect -----------------------
	if (plugBeingDirtied.partialName() == "loadFile") {
		MPlug out(mThisNode, PclLoader::out_Attr);
		affectedPlugs.append(out);	
		MPlug stFP(mThisNode, PclLoader::startFrame_Attr);
		affectedPlugs.append(stFP);
		MPlug enFP(mThisNode, PclLoader::endFrame_Attr);
		affectedPlugs.append(enFP);
		MPlug bbC(mThisNode, PclLoader::outBoundingBoxCorner_Attr);
		affectedPlugs.append(bbC);
		MPlug opm(mThisNode, PclLoader::outPositionMaxFrame_Attr);
		affectedPlugs.append(opm);
		MPlug hnp(mThisNode, PclLoader::theHighestNumberOfPoints_Attr);
		affectedPlugs.append(hnp);	
		//MPlug upo(mThisNode, PclLoader::updateOut_Attr);
		//affectedPlugs.append(upo);
	}
	

	//-- displayPoints effect  -----------------------
	if (plugBeingDirtied.partialName() == "displayPoints") {
		MPlug out(mThisNode, PclLoader::out_Attr);
		affectedPlugs.append(out);
		MPlug outP(mThisNode, PclLoader::outPosition_Attr);
		affectedPlugs.append(outP);
		MPlug outPV(mThisNode, PclLoader::outPositionVec_Attr);
		affectedPlugs.append(outPV);
		MPlug outCV(mThisNode, PclLoader::outColorVec_Attr);
		affectedPlugs.append(outCV);
		MPlug outC(mThisNode, PclLoader::outColor_Attr);
		affectedPlugs.append(outC);
		MPlug opm(mThisNode, PclLoader::outPositionMaxFrame_Attr);
		affectedPlugs.append(opm);
		MPlug hnp(mThisNode, PclLoader::theHighestNumberOfPoints_Attr);
		affectedPlugs.append(hnp);
	}


	// Container mode effect.
	if (plugBeingDirtied.partialName() == "containerOnOff") {
		MPlug out(mThisNode, PclLoader::out_Attr);
		affectedPlugs.append(out);
		MPlug outDC(mThisNode, PclLoader::outDrawContainer_Attr);
		affectedPlugs.append(outDC);
		MPlug outP(mThisNode, PclLoader::outPosition_Attr);
		affectedPlugs.append(outP);
		MPlug outPV(mThisNode, PclLoader::outPositionVec_Attr);
		affectedPlugs.append(outPV);
		MPlug outCV(mThisNode, PclLoader::outColorVec_Attr);
		affectedPlugs.append(outCV);
		MPlug outC(mThisNode, PclLoader::outColor_Attr);
		affectedPlugs.append(outC);
		MPlug bbC(mThisNode, PclLoader::outBoundingBoxCorner_Attr);
		affectedPlugs.append(bbC);
		MPlug hnp(mThisNode, PclLoader::theHighestNumberOfPoints_Attr);
		affectedPlugs.append(hnp);
		eval_state = true;

	}

	//-- inContainerMatrix effect  -----------------------
	if (plugBeingDirtied.partialName() == "inContainerMatrix") {
		MPlug out(mThisNode, PclLoader::out_Attr);
		affectedPlugs.append(out);
		MPlug outDC(mThisNode, PclLoader::outDrawContainer_Attr);
		affectedPlugs.append(outDC);
		MPlug outP(mThisNode, PclLoader::outPosition_Attr);
		affectedPlugs.append(outP);
		MPlug outPV(mThisNode, PclLoader::outPositionVec_Attr);
		affectedPlugs.append(outPV);
		MPlug outCV(mThisNode, PclLoader::outColorVec_Attr);
		affectedPlugs.append(outCV);
		MPlug outC(mThisNode, PclLoader::outColor_Attr);
		affectedPlugs.append(outC);
		MPlug poInF(mThisNode, PclLoader::pointNumber_Attr);
		affectedPlugs.append(poInF);
		MPlug bbC(mThisNode, PclLoader::outBoundingBoxCorner_Attr);
		affectedPlugs.append(bbC);
		MPlug hnp(mThisNode, PclLoader::theHighestNumberOfPoints_Attr);
		affectedPlugs.append(hnp);
		//MPlug upo(mThisNode, PclLoader::updateOut_Attr);
		//affectedPlugs.append(upo);
		eval_state = true;
	}


	//-- inlRaycasterPoints effect  -----------------------
	if (plugBeingDirtied.partialName() == "inlRaycasterPoints") {
		MPlug out(mThisNode, PclLoader::out_Attr);
		affectedPlugs.append(out);
		affectedPlugs.append(out);
		MPlug outDC(mThisNode, PclLoader::outDrawContainer_Attr);
		MPlug outP(mThisNode, PclLoader::outPosition_Attr);
		affectedPlugs.append(outP);
		MPlug outPV(mThisNode, PclLoader::outPositionVec_Attr);
		affectedPlugs.append(outPV);
		MPlug outCV(mThisNode, PclLoader::outColorVec_Attr);
		affectedPlugs.append(outCV);
		MPlug outC(mThisNode, PclLoader::outColor_Attr);
		affectedPlugs.append(outC);
		MPlug opm(mThisNode, PclLoader::outPositionMaxFrame_Attr);
		affectedPlugs.append(opm);
		MPlug hnp(mThisNode, PclLoader::theHighestNumberOfPoints_Attr);
		affectedPlugs.append(hnp);
	}
	

	return MS::kSuccess;
}

bool PclLoader::loadFramesFromCache(const MString& mainPath, unsigned int skipFilesRows)
{
	// Use hardware concurrency for parallel loading (0 = auto-detect).
	// This prevents Maya from freezing during heavy I/O operations.
	const unsigned int maxThreads = std::thread::hardware_concurrency();
	const bool loaded = mFrameCache.loadSequence(mainPath.asChar(), static_cast<int>(skipFilesRows), maxThreads);
	if (!loaded)
	{
		return false;
	}

	mPointArrayPosAllFrame_list.clear();
	mPointArrayColAllFrame_list.clear();
	mPointArrayPosAllFrameVec_list.clear();
	mPointArrayColAllFrameVec_list.clear();
	mBoundingBoxCorner_list.clear();

	const auto& frames = mFrameCache.frames();
	for (size_t idx = 0; idx < frames.size(); ++idx)
	{
		MPointArray positions;
		MPointArray colors;
		MVectorArray positionsVec;
		MVectorArray colorsVec;
		MPointArray bbox;
		convertCacheFrame(frames[idx], positions, colors, positionsVec, colorsVec, bbox);

		mPointArrayPosAllFrame_list.push_back(positions);
		mPointArrayColAllFrame_list.push_back(colors);
		mPointArrayPosAllFrameVec_list.push_back(positionsVec);
		mPointArrayColAllFrameVec_list.push_back(colorsVec);
		mBoundingBoxCorner_list.push_back(bbox);

	}

	mNumberOfFrames = static_cast<unsigned int>(frames.size());
	mNumberOfPointsEachFrame = frames.empty() ? 0 : static_cast<unsigned int>(frames.front().positions.size());
	mStartFrameVal = mFrameCache.startFrame();
	mLastFrameVal = mFrameCache.endFrame();
	mTheHighestNumberOfPoints = mFrameCache.highestPointCount();
	mLoadOnes = false;
	return true;
}

// Clears cached point data and resets bookkeeping so a fresh load can occur.
void PclLoader::clearLoadedData()
{
	mFrameCache.clear();
	mNumberOfFrames = 0;
	mPointArrayPosAllFrame_list.clear();
	mPointArrayColAllFrame_list.clear();
	mPointArrayPosAllFrameVec_list.clear();
	mPointArrayColAllFrameVec_list.clear();
	mPointPosFrameOut_list.clear();
	mPointColFrameOut_list.clear();
	mPointPosFrameOutVec_list.clear();
	mPointColFrameOutVec_list.clear();
	mContainerState.reset();
	mBoundingBoxCorner_list.clear();
	mTheHighestNumberOfPoints = 0;
	mNumberOfPointsEachFrame = 0;
	mLoadOnes = true;
	mLoadedSkipRows = 0;
}

// Clamps the user-provided load range so it stays within the cached frame span.
void PclLoader::clampLoadWindow(EvaluateContext& ctx, MDataBlock& dataBlock)
{
	if (ctx.loadEndFrame <= ctx.loadStartFrame)
	{
		ctx.loadEndFrame = ctx.loadStartFrame + 1;
		dataBlock.outputValue(loadEndFrame_Attr).setInt(ctx.loadEndFrame);
	}

	if (ctx.loadStartFrame < mStartFrameVal)
	{
		ctx.loadStartFrame = mStartFrameVal;
		dataBlock.outputValue(loadStartFrame_Attr).setInt(mStartFrameVal);
	}

	if (ctx.loadStartFrame > mLastFrameVal)
	{
		ctx.loadStartFrame = mStartFrameVal;
		dataBlock.outputValue(loadStartFrame_Attr).setInt(mStartFrameVal);
	}

	if (ctx.loadEndFrame > mLastFrameVal)
	{
		ctx.loadEndFrame = mLastFrameVal;
		dataBlock.outputValue(loadEndFrame_Attr).setInt(mLastFrameVal);
	}
}

// Loads or clears cached frames based on the current load mode and path settings.
MStatus PclLoader::processLoadRequest(EvaluateContext& ctx, MDataBlock& dataBlock)
{
	if (ctx.loadFileMode == 0)
	{
		if (!mLoadOnes)
		{
			clearLoadedData();
		}
		dataBlock.outputValue(startFrame_Attr).setInt(0);
		dataBlock.outputValue(endFrame_Attr).setInt(0);
		dataBlock.outputValue(loadStartFrame_Attr).setInt(0);
		dataBlock.outputValue(loadEndFrame_Attr).setInt(0);
		dataBlock.outputValue(pointNumber_Attr).setInt(0);
		dataBlock.outputValue(theHighestNumberOfPoints_Attr).setInt(0);
		return MS::kSuccess;
	}

	if (ctx.mainPath.length() == 0)
	{
		MGlobal::displayError("pclLoader: loadFiles is enabled but mainPath is empty. Please specify a valid path to point cloud files.");
		return MS::kFailure;
	}

	if (mLoadOnes)
	{
		if (!loadFramesFromCache(ctx.mainPath, ctx.skipRows))
		{
			MGlobal::displayError(MString("pclLoader: Failed to load point cloud sequence from: ") + ctx.mainPath + ". Please verify the path exists and contains valid .ply or .pcd files.");
			return MS::kFailure;
		}

		mLoadedSkipRows = ctx.skipRows;
		dataBlock.outputValue(startFrame_Attr).setInt(mStartFrameVal);
		dataBlock.outputValue(endFrame_Attr).setInt(mLastFrameVal);
		dataBlock.outputValue(loadStartFrame_Attr).setInt(mStartFrameVal);
		ctx.loadStartFrame = mStartFrameVal;
		dataBlock.outputValue(loadEndFrame_Attr).setInt(mLastFrameVal);
		ctx.loadEndFrame = mLastFrameVal;
		dataBlock.outputValue(pointNumber_Attr).setInt(mNumberOfPointsEachFrame);
		editKeyframesOnTimeAttr(mStartFrameVal, mLastFrameVal);

		updateContainerState(ctx);
		buildContainerDrawData();
		theContanerOnOff_plug.setValue(0);
		ctx.containerOnOff = 0;
		mContainerOnOff = 0;

		eval_state = true;
		mLoadOnes = false;
	}

	dataBlock.outputValue(theHighestNumberOfPoints_Attr).setInt(mTheHighestNumberOfPoints);
	return MS::kSuccess;
}

// Extracts the correct frame for the requested time and pushes point arrays to the outputs.
void PclLoader::updateFrameOutputs(const EvaluateContext& ctx, MDataBlock& dataBlock)
{
	if (mPointArrayPosAllFrame_list.empty())
	{
		return;
	}

	const int timeIndex = ctx.timeOffset;
	if (timeIndex < 0 || static_cast<size_t>(timeIndex) >= mPointArrayPosAllFrame_list.size())
	{
		return;
	}

	if (ctx.containerOnOff == 0 || ctx.containerOnOff == 1)
	{
		mPointPosFrameOut_list = mPointArrayPosAllFrame_list[timeIndex];
		mPointColFrameOut_list = mPointArrayColAllFrame_list[timeIndex];

		if (mParticlesConnected && mTheHighestNumberOfPoints > 0)
		{
			mPointPosFrameOutVec_list = mPointArrayPosAllFrameVec_list[timeIndex];
			mPointColFrameOutVec_list = mPointArrayColAllFrameVec_list[timeIndex];
		}
	}
	else
	{
		const bool keepInside = (ctx.containerOnOff == 2);
		const MPointArray& sourcePos = mPointArrayPosAllFrame_list[timeIndex];
		const MPointArray& sourceCol = mPointArrayColAllFrame_list[timeIndex];
		const MVectorArray* sourcePosVec = (mParticlesConnected && mTheHighestNumberOfPoints > 0)
			? &mPointArrayPosAllFrameVec_list[timeIndex]
			: nullptr;
		const MVectorArray* sourceColVec = (mParticlesConnected && mTheHighestNumberOfPoints > 0)
			? &mPointArrayColAllFrameVec_list[timeIndex]
			: nullptr;

		MPointArray filteredPos;
		MPointArray filteredCol;
		MVectorArray filteredPosVec;
		MVectorArray filteredColVec;

		filterPoints(sourcePos,
		             sourceCol,
		             mContainerState.bbox,
		             keepInside,
		             sourcePosVec,
		             sourceColVec,
		             filteredPos,
		             filteredCol,
		             sourcePosVec ? &filteredPosVec : nullptr,
		             sourceColVec ? &filteredColVec : nullptr);

		mPointPosFrameOut_list = filteredPos;
		mPointColFrameOut_list = filteredCol;

		if (sourcePosVec)
		{
			mPointPosFrameOutVec_list = filteredPosVec;
			mPointColFrameOutVec_list = filteredColVec;
		}
	}

	mNumberOfPointsEachFrame = mPointPosFrameOut_list.length();
	dataBlock.outputValue(pointNumber_Attr).setInt(mNumberOfPointsEachFrame);

	setPointArrayOutput(dataBlock, outPosition_Attr, mPointPosFrameOut_list);
	setPointArrayOutput(dataBlock, outColor_Attr, mPointColFrameOut_list);

	if (mParticlesConnected)
	{
		setVectorArrayOutput(dataBlock, outPositionVec_Attr, mPointPosFrameOutVec_list);
		setVectorArrayOutput(dataBlock, outColorVec_Attr, mPointColFrameOutVec_list);
	}
}

// Updates container bounding-box outputs and viewport-draw data.
void PclLoader::updateBoundingBoxOutputs(const EvaluateContext& ctx, MDataBlock& dataBlock)
{
	updateContainerState(ctx);
	buildContainerDrawData();

	const MPoint minPt = mContainerState.bbox.min();
	const MPoint maxPt = mContainerState.bbox.max();
	MPointArray boundingBoxCorner_list;
	boundingBoxCorner_list.append(minPt);
	boundingBoxCorner_list.append(maxPt);

	setPointArrayOutput(dataBlock, outBoundingBoxCorner_Attr, boundingBoxCorner_list);
	setPointArrayOutput(dataBlock, outDrawContainer_Attr, mContainerState.drawLinesV2);

	dataBlock.outputValue(pointNumber_Attr).setInt(mNumberOfPointsEachFrame);
	dataBlock.outputValue(theHighestNumberOfPoints_Attr).setInt(mTheHighestNumberOfPoints);
}

// Reads raycaster inputs so downstream connections stay valid.
void PclLoader::updateRaycasterInputs(MDataBlock& dataBlock)
{
	MFnPointArrayData dataPointPosArrayFn;
	MObject pointPos_in = dataBlock.inputValue(inlRaycasterPoints_Attr).data();
	if (dataPointPosArrayFn.setObject(pointPos_in) == MS::kSuccess)
	{
		mPointPosFrameOut_list = dataPointPosArrayFn.array();
	}
}

void PclLoader::updateContainerState(const EvaluateContext& ctx)
{
	if (mContainerState.defaultBox)
	{
		mContainerState.lastMin = MPoint(10, 10, 10);
		mContainerState.lastMax = MPoint(-10, 0, -10);
		mContainerState.defaultBox = false;
	}

	MPoint cor1 = mContainerState.lastMin;
	MPoint cor2 = mContainerState.lastMax;

	if (ctx.containerOnOff == 0)
	{
		const size_t index = static_cast<size_t>(std::max(0, ctx.timeOffset));
		if (!mBoundingBoxCorner_list.empty() && index < mBoundingBoxCorner_list.size())
		{
			const MPointArray& corners = mBoundingBoxCorner_list[index];
			if (corners.length() >= 2)
			{
				mContainerState.lastMin = corners[0];
				mContainerState.lastMax = corners[1];
				cor1 = mContainerState.lastMin;
				cor2 = mContainerState.lastMax;
			}
		}
	}
	else
	{
		cor1 = mContainerState.lastMin * ctx.containerMatrix;
		cor2 = mContainerState.lastMax * ctx.containerMatrix;
	}

	mContainerState.bbox.clear();
	mContainerState.bbox.expand(cor1);
	mContainerState.bbox.expand(cor2);
}

void PclLoader::buildContainerDrawData()
{
	const MPoint minPt = mContainerState.bbox.min();
	const MPoint maxPt = mContainerState.bbox.max();

	PclUtilities::appendLinesFromBounds(minPt, maxPt, mContainerState.drawLines);
	PclUtilities::appendLinesFromBounds(minPt, maxPt, mContainerState.drawLinesV2);
}

void PclLoader::convertCacheFrame(
	const pclFrameCache::FrameData& frame,
	MPointArray& outPositions,
	MPointArray& outColors,
	MVectorArray& outPosVec,
	MVectorArray& outColorVec,
	MPointArray& outBBox)
{
	outPositions.clear();
	outColors.clear();
	outPosVec.clear();
	outColorVec.clear();
	outBBox.clear();

	if (frame.positions.empty())
	{
		return;
	}

	double minX = std::numeric_limits<double>::max();
	double minY = std::numeric_limits<double>::max();
	double minZ = std::numeric_limits<double>::max();
	double maxX = -std::numeric_limits<double>::max();
	double maxY = -std::numeric_limits<double>::max();
	double maxZ = -std::numeric_limits<double>::max();

	const size_t pointCount = frame.positions.size();
	outPositions.setLength(static_cast<unsigned int>(pointCount));
	outPosVec.setLength(static_cast<unsigned int>(pointCount));
	outColors.setLength(static_cast<unsigned int>(pointCount));
	outColorVec.setLength(static_cast<unsigned int>(pointCount));

	for (size_t i = 0; i < pointCount; ++i)
	{
		const auto& pos = frame.positions[i];
		MPoint mayaPoint(pos.x(), pos.y(), pos.z());
		outPositions.set(mayaPoint, static_cast<unsigned int>(i));
		outPosVec.set(MVector(pos.x(), pos.y(), pos.z()), static_cast<unsigned int>(i));

		const auto& col = frame.colors[i];
		outColors.set(MPoint(col.x(), col.y(), col.z()), static_cast<unsigned int>(i));
		outColorVec.set(MVector(col.x(), col.y(), col.z()), static_cast<unsigned int>(i));

		minX = std::min(minX, pos.x());
		minY = std::min(minY, pos.y());
		minZ = std::min(minZ, pos.z());
		maxX = std::max(maxX, pos.x());
		maxY = std::max(maxY, pos.y());
		maxZ = std::max(maxZ, pos.z());
	}

	outBBox.append(MPoint(minX, minY, minZ));
	outBBox.append(MPoint(maxX, maxY, maxZ));
}



//-- initialize Attr ---------------------------------------
MStatus PclLoader::initialize()
{
	MStatus status = MS::kSuccess;

	MFnEnumAttribute eAttr;
	MFnUnitAttribute uAttr;
	MFnTypedAttribute typedAttrFn;
	MFnNumericAttribute numAttrFn;
	MFnMatrixAttribute mFnMatrixAtt;

	MFnPointArrayData posArrayDataFn;
	MFnPointArrayData colArrayDataFn;
	MFnPointArrayData conArrayDataFn;
	MPointArray defaultPosArray;

	MFnVectorArrayData pointArrayDataFnVec;
	MVectorArray defaultPointPosArray_vec;
	MVectorArray defaultPointColArray_vec;

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

	//-- set the Main path --------------------------------------------------
	mainPath_Attr = typedAttrFn.create("mainPath", "mainPath", MFnData::kString);
	typedAttrFn.setStorable(true);
	typedAttrFn.setWritable(true);
	typedAttrFn.setKeyable(false);
	typedAttrFn.setReadable(true);
	addAttribute(mainPath_Attr);


	//-- set startFrame attr ---------------------------------------------
	startFrame_Attr = numAttrFn.create("startFrame", "startFrame", MFnNumericData::kInt, 0);
	numAttrFn.setDefault(0);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setKeyable(true);
	numAttrFn.setConnectable(true);
	numAttrFn.setChannelBox(true);
	addAttribute(startFrame_Attr);

	//-- set end Frame attr ---------------------------------------------
	endFrame_Attr = numAttrFn.create("endFrame", "endFrame", MFnNumericData::kInt, 0);
	numAttrFn.setDefault(0);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setStorable(false);
	numAttrFn.setKeyable(false);
	numAttrFn.setConnectable(true);
	numAttrFn.setChannelBox(true);
	addAttribute(endFrame_Attr);


	//-- set loadStartFrame attr ---------------------------------------------
	loadStartFrame_Attr = numAttrFn.create("loadStartFrame", "loadStartFrame", MFnNumericData::kInt, 0);
	numAttrFn.setDefault(0);
	numAttrFn.setMin(0);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setKeyable(true);
	numAttrFn.setConnectable(true);
	addAttribute(loadStartFrame_Attr);

	//-- set loadEndFrame attr ---------------------------------------------
	loadEndFrame_Attr = numAttrFn.create("loadEndFrame", "loadEndFrame", MFnNumericData::kInt, 0);
	numAttrFn.setDefault(0);
	numAttrFn.setMin(0);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setKeyable(true);
	numAttrFn.setConnectable(true);
	addAttribute(loadEndFrame_Attr);

	//-- set loadEndFrame attr ---------------------------------------------
	skipFilesRows_Attr = numAttrFn.create("skipFilesRows", "skipFilesRows", MFnNumericData::kInt, 0);
	numAttrFn.setDefault(0);
	numAttrFn.setMin(0);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setKeyable(true);
	numAttrFn.setConnectable(true);
	addAttribute(skipFilesRows_Attr);

	//-- load File attr ---------------------------------------------------
	loadFile_Attr = eAttr.create("loadFiles", "loadFiles");
	eAttr.addField("off", 0);
	eAttr.addField("load all Points", 1);
	eAttr.addField("load Points in Container", 2);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(loadFile_Attr);

	//-- displayPoints attr  ---------------------------------------------------
	displayPoints_Attr = eAttr.create("displayPoints", "displayPoints");
	eAttr.addField("Points Off", 0);
	eAttr.addField("Points On - No Colors", 1);
	eAttr.addField("Points On - With Colors", 2);
	eAttr.setDefault(1);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(displayPoints_Attr);


	//-- contaner - Off / all points inside / all points outside ---------------------------------------------------
	containerOnOff_Attr = eAttr.create("containerOnOff", "containerOnOff");
	//eAttr.addField("Off", 0);
	eAttr.addField("Auto contaner", 0);
	eAttr.addField("Manual contaner", 1);
	eAttr.addField("All In contaner ", 2);
	eAttr.addField("All Out contaner", 3);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(containerOnOff_Attr);


	//-- set pointsSize attr ---------------------------------------------
	pointsSize_Attr = numAttrFn.create("pointsSize", "pointsSize", MFnNumericData::kFloat, 1.0);
	numAttrFn.setMin(0);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setKeyable(true);
	numAttrFn.setConnectable(true);
	numAttrFn.setChannelBox(true);
	addAttribute(pointsSize_Attr);


	//-- set pointsInFrame attr ---------------------------------------------
	pointNumber_Attr = numAttrFn.create("pointNumber", "pointNumber", MFnNumericData::kInt, 0);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setKeyable(true);
	numAttrFn.setConnectable(true);
	numAttrFn.setChannelBox(true);
	addAttribute(pointNumber_Attr);


	//-- ContainerMatrixr Attr ------------------------------------------------------------
	inContainerMatrix_Attr = mFnMatrixAtt.create("inContainerMatrix", "inContainerMatrix");
	mFnMatrixAtt.setReadable(false);
	mFnMatrixAtt.setStorable(false);
	mFnMatrixAtt.setConnectable(true);
	addAttribute(inContainerMatrix_Attr);

	//-- in Points from Raycaster  (from Raycaster node) --------------------------------------
	posArrayDataFn.create(defaultPosArray);
	typedAttrFn.create("inlRaycasterPoints", "inlRaycasterPoints", MFnData::kPointArray, posArrayDataFn.object(), &status);
	typedAttrFn.setWritable(true);
	typedAttrFn.setStorable(true);
	inlRaycasterPoints_Attr = typedAttrFn.object();
	addAttribute(inlRaycasterPoints_Attr);
	

	//== Time attributs ============================================================================
	//-- set the in time attr --------------
	inTime_Attr = uAttr.create("inTime", "inTime", MFnUnitAttribute::kTime);
	addAttribute(inTime_Attr);

	//-- set the in time attr (for time modifyr in the graph editot animation )--------------
	timeOffset_Attr = numAttrFn.create("timeOffset", "timeOffset", MFnNumericData::kInt, 0);
	numAttrFn.setMin(0);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setKeyable(true);
	numAttrFn.setConnectable(true);
	addAttribute(timeOffset_Attr);
	//== Time End ============================================================================


	//==  Output data for Viewport2 and Other nodes - particleSys  / pclSequencer / pclCluster / pclRaycaster -----------
	//-- outBoundingBox (container) corner  ---------------------------------------------------------
	posArrayDataFn.create(defaultPosArray);
	typedAttrFn.create("outBoundingBoxCorner", "outBoundingBoxCorner", MFnData::kPointArray, posArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outBoundingBoxCorner_Attr = typedAttrFn.object();
	addAttribute(outBoundingBoxCorner_Attr);


	//== Output Data as PointArray ============================================================
	//-- out out Points Position ---------------------------------------------------------
	posArrayDataFn.create(defaultPosArray);
	typedAttrFn.create("outPointsPosition", "outPointsPosition", MFnData::kPointArray, posArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outPosition_Attr = typedAttrFn.object();
	addAttribute(outPosition_Attr);

	//-- output points color for viewport2 and Other nodes (PclRaycaster) ----------------------------------------------------------------
	MPointArray defaultColArray;
	colArrayDataFn.create(defaultColArray);
	typedAttrFn.create("outPointsColor", "outPointsColor", MFnData::kPointArray, colArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outColor_Attr = typedAttrFn.object();
	addAttribute(outColor_Attr);

	//-- output Draw Container for viewport2 and Other nodes -----------------------------------------------------------------
	MPointArray defaultConArray;
	conArrayDataFn.create(defaultConArray);
	typedAttrFn.create("outDrawContainer", "outDrawContainer", MFnData::kPointArray, conArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outDrawContainer_Attr = typedAttrFn.object();
	addAttribute(outDrawContainer_Attr);

	//== Atrributs for connected particle sys =============================================================================
	//== Output Data Point position / color Vector (for particle sys) =====================================================
	//-- out the MAX Count of point position from one frame --------------------------------------------------------------  
	posArrayDataFn.create(defaultPosArray);
	typedAttrFn.create("outPointsPositionMaxFrame", "outPointsPositionMaxFrame", MFnData::kPointArray, posArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outPositionMaxFrame_Attr = typedAttrFn.object();
	addAttribute(outPositionMaxFrame_Attr);

	//-- out out Points Position ---------------------------------------------------------
	pointArrayDataFnVec.create(defaultPointPosArray_vec);
	typedAttrFn.create("outPointsPosition_vec", "outPointsPosition_vec", MFnData::kVectorArray, pointArrayDataFnVec.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outPositionVec_Attr = typedAttrFn.object();
	addAttribute(outPositionVec_Attr);

	//-- out Points Color ---------------------------------------------------------
	pointArrayDataFnVec.create(defaultPointColArray_vec);
	typedAttrFn.create("outPointsColor_vec", "outPointsColor_vec", MFnData::kVectorArray, pointArrayDataFnVec.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outColorVec_Attr = typedAttrFn.object();
	addAttribute(outColorVec_Attr);


	//-- theHighestNumberOfPoints_Attr ---------------------------------------------
	theHighestNumberOfPoints_Attr = numAttrFn.create("outTheHighestNumberOfPoints", "outTheHighestNumberOfPoints", MFnNumericData::kInt, 0);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setKeyable(true);
	numAttrFn.setConnectable(true);
	numAttrFn.setChannelBox(true);
	addAttribute(theHighestNumberOfPoints_Attr);

	
	//-- set the out attr ---------------------------------------------
	out_Attr = numAttrFn.create("out", "out", MFnNumericData::kInt, 0);
	addAttribute(out_Attr);


	return MS::kSuccess;
}



// -------------------------------------------------------------------------- -
//---------------------------------------------------------------------------
// Viewport 2.0 override implementation
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
PclLoaderDrawOverride::PclLoaderDrawOverride(const MObject& obj)
	: MHWRender::MPxDrawOverride(obj, PclLoaderDrawOverride::draw)
{
}

PclLoaderDrawOverride::~PclLoaderDrawOverride()
{
}

MHWRender::DrawAPI PclLoaderDrawOverride::supportedDrawAPIs() const
{
	// this plugin supports both GL and DX
	return (MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile);
}



//-- call get_loadFile_Attr ------------------ -
short PclLoaderDrawOverride::get_loadFile(const MDagPath& objPath) const
{
	MStatus status;
	if (status)
	{
		MObject pclLoaderNode = objPath.node();
		MFnDagNode dagThis(pclLoaderNode);
		short loadFile = dagThis.findPlug(PclLoader::loadFile_Attr).asShort();
		return loadFile;
	}

	return 0;
}

//-- call displayPoints_Attr ------------------ -
short PclLoaderDrawOverride::get_displayPoints(const MDagPath & objPath) const
{
	MStatus status;
	if (status)
	{
		MObject pclLoaderNode = objPath.node();
		MFnDagNode dagThis(pclLoaderNode);
		short displayPoints = dagThis.findPlug(PclLoader::displayPoints_Attr).asShort();
		return displayPoints;
	}

	return 0;
}

//-- call displayColor_Attr ------------------ -
short PclLoaderDrawOverride::get_containerOnOff(const MDagPath& objPath) const
{
	MStatus status;
	if (status)
	{
		MObject pclLoaderNode = objPath.node();
		MFnDagNode dagThis(pclLoaderNode);
		short containerOnOff = dagThis.findPlug(PclLoader::containerOnOff_Attr).asShort();
		return containerOnOff;
	}

	return 0;
}


//--call get_point Size------------------------------
float PclLoaderDrawOverride::get_pointsSize(const MDagPath & objPath) const
{
	MStatus status;
	if (status)
	{
		MObject pclLoaderNode = objPath.node(&status);
		MFnDagNode dagThis(pclLoaderNode);
		float pointSize = dagThis.findPlug(PclLoader::pointsSize_Attr).asFloat();
		return pointSize;
	}

	return 1;
}


//--call get_pointPosition-------------------------------------------------------------
MPointArray PclLoaderDrawOverride::get_pointPosition(const MDagPath & objPath) const
{
	MStatus status;
	MPointArray outPosArray;
	if (status)
	{
		MObject pclLoaderNode = objPath.node();
		MFnDagNode dagThis(pclLoaderNode);

		MPlug plug_aPoints = dagThis.findPlug(PclLoader::outPosition_Attr);
		MObject o_aPoints;
		plug_aPoints.getValue(o_aPoints);
		MFnPointArrayData fn_aPoints(o_aPoints);
		fn_aPoints.copyTo(outPosArray);

		return outPosArray;

	}
	return outPosArray;
}

//--call get_pointColor-----------------------------------------------------------
MPointArray PclLoaderDrawOverride::get_pointColor(const MDagPath& objPath) const
{
	MStatus status;
	MPointArray outColArray;
	if (status)
	{
		MObject pclLoaderNode = objPath.node();
		MFnDagNode dagThis(pclLoaderNode);

		MPlug plug_aPoints = dagThis.findPlug(PclLoader::outColor_Attr);
		MObject o_aPoints;
		plug_aPoints.getValue(o_aPoints);
		MFnPointArrayData fn_aPoints(o_aPoints);
		fn_aPoints.copyTo(outColArray);

		return outColArray;

	}
	return outColArray;
}


//-- call get_outGlobalBBox info ----------------------------------
MPointArray PclLoaderDrawOverride::get_containerBBox(const MDagPath& objPath) const
{
	MStatus status;
	MPointArray containerBoundingBoxs_list;
	if (status)
	{
		MObject pclLoaderNode = objPath.node();
		MFnDagNode dagThis(pclLoaderNode);

		MPlug plug_aPoints = dagThis.findPlug(PclLoader::outDrawContainer_Attr);
		MObject o_aPoints;
		plug_aPoints.getValue(o_aPoints);
		MFnPointArrayData fn_aPoints(o_aPoints);
		fn_aPoints.copyTo(containerBoundingBoxs_list);

		return containerBoundingBoxs_list;

	}

	return containerBoundingBoxs_list;
}


//-- call get_pointsIמRaycast this data come from pcllRaycaster node -------------------
MPointArray PclLoaderDrawOverride::get_pointsInRaycaste(const MDagPath& objPath) const
{
	MStatus status;

	MFnPointArrayData dataPointArrayFn;
	MPointArray raycasterPoints_list;
	MObject pclSequencerNode = objPath.node();
	MFnDagNode dagThis(pclSequencerNode);

	MPlug plug_aPoints = dagThis.findPlug(PclLoader::inlRaycasterPoints_Attr);
	MObject o_aPoints;
	plug_aPoints.getValue(o_aPoints);
	MFnPointArrayData fn_aPoints(o_aPoints);
	fn_aPoints.copyTo(raycasterPoints_list);


	return raycasterPoints_list;

}

//-----------------------------------------------------------------------------
// Called by Maya each time the object needs to be drawn.
MUserData* PclLoaderDrawOverride::prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath, const MHWRender::MFrameContext& frameContext, MUserData* oldData)
{
	PclLoaderData* data = dynamic_cast<PclLoaderData*>(oldData);
	if (!data)
	{
		data = new PclLoaderData();
	}

	MFnNumericData fnData();

	short loadFile = get_loadFile(objPath);
	short displayPoints = get_displayPoints(objPath);
	short containerOnOff = get_containerOnOff(objPath);
	float  pointSize = get_pointsSize(objPath);
	MPointArray pointPos = get_pointPosition(objPath);
	MPointArray pointcol = get_pointColor(objPath);
	MPointArray containerBBox = get_containerBBox(objPath);
	MPointArray raycasterPoints = get_pointsInRaycaste(objPath);

	data->mLoadFile = loadFile;
	data->mDisplayPoints = displayPoints;
	data->mContainerOnOff = containerOnOff;
	data->mPointSize = pointSize;
	data->mContainerBBox.clear();
	data->mContainerBBox = containerBBox;
	data->mPointPosition.clear();
	data->mPointPosition = pointPos;
	data->mPointColor.clear();
	data->mPointColor = pointcol;
	data->mRaycasterPoints.clear();
	data->mRaycasterPoints = raycasterPoints;

	return data;
}

// addUIDrawables() provides access to the MUIDrawManager, which can be used
// to queue up operations for drawing simple UI elements such as lines, circles and
// text. To enable addUIDrawables(), override hasUIDrawables() and make it return true.
void PclLoaderDrawOverride::addUIDrawables(const MDagPath& objPath, MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext, const MUserData* data)
{
	// Get data cached by prepareForDraw() for each drawable instance, then MUIDrawManager 
	// can draw simple UI by these data.
	PclLoaderData* drawData = (PclLoaderData*)data;
	if (!drawData)
	{
		return;
	}

	short loadFile = drawData->mLoadFile;
	short displayPoints = drawData->mDisplayPoints;
	short containerOnOff = drawData->mContainerOnOff;
	short pointSize = drawData->mPointSize;
	MPointArray pointPosition = drawData->mPointPosition;
	MPointArray pointcolor = drawData->mPointColor;
	MPointArray containerBBox = drawData->mContainerBBox;
	MPointArray raycasterPoints = drawData->mRaycasterPoints;

	

	if (loadFile == 1 || loadFile == 2)
	{
		//-- set point size ------------
		drawManager.setPointSize(pointSize);

	
		//-- displayColor OFF - no color -----------------
		if (displayPoints == 1)
		{
			MColor color(0, 0.2, 0.9);
			drawManager.beginDrawable();
			drawManager.setDepthPriority(2);
			drawManager.setColor(color);
			drawManager.points(pointPosition, FALSE);
			drawManager.endDrawable();
		}

		//-- displayColor ON - set color --------------
		if (displayPoints == 2)
		{
			drawManager.beginDrawable();

			// Loop over colors and positions to draw points.
			const unsigned int pointCount = pointPosition.length();
			for (unsigned int i = 0; i < pointCount; ++i)
			{
				MColor color(pointcolor[i].x, pointcolor[i].y, pointcolor[i].z);
				drawManager.setColor(color);
				drawManager.point(pointPosition[i]);
			}
			drawManager.endDrawable();
		}
	}

	//-- check the contaner --------------------------------
	if (containerOnOff == 1 || containerOnOff == 2 || containerOnOff == 3|| loadFile == 1 || loadFile == 2)
	{
		//== set container BBox ================
		MColor solidColor = MColor(0.9, 0.3, 1, 1);
		drawManager.beginDrawable();
		drawManager.setColor(solidColor);
		drawManager.setDepthPriority(3);
		drawManager.setLineWidth(1);
		drawManager.lineList(containerBBox, false);
		drawManager.endDrawable();
	}


	//-- check the Raycast (from the raycaster node ) --------------------------------

	if (raycasterPoints.length() > 0)
	{
		MColor color(1, 0.1, 0.1);
		drawManager.beginDrawable();
		drawManager.setPointSize(pointSize);
		drawManager.setDepthPriority(5);
		drawManager.setColor(color);
		drawManager.points(raycasterPoints, FALSE);
		drawManager.endDrawable();
	}
}