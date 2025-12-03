// Maya API
#include <maya/MFnDependencyNode.h>
#include <maya/MFnDagNode.h>
#include <maya/MSelectionList.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnVectorArrayData.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MPlugArray.h>
#include <maya/MAnimControl.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MStatus.h>

// Project headers
#include "pclSequencer.h"

// Error checking macro
#ifndef CHECK_MSTATUS_AND_RETURN_IT
#define CHECK_MSTATUS_AND_RETURN_IT(status) \
	do { \
		if ((status) != MS::kSuccess) { \
			return (status); \
		} \
	} while (0)
#endif



MTypeId PclSequencer::id = 0x8103126;
MObject PclSequencer::outPositionPP;
MObject PclSequencer::particleCount;
MObject PclSequencer::pointsCreate_Attr;
MObject PclSequencer::missingFrames_Attr;
MObject PclSequencer::inPointsPosition_Attr;
MObject PclSequencer::inPointsColor_Attr;
MObject PclSequencer::inTime_Attr;
MObject PclSequencer::outTime_Attr;
MObject PclSequencer::timeOffset_Attr;
MObject PclSequencer::pointNumber_Attr;
MObject PclSequencer::savePCDPLY_Attr;
MObject PclSequencer::saveAsciiBinary_Attr;
MObject PclSequencer::saveMainPath_Attr;
MObject PclSequencer::saveFileName_Attr;
MObject PclSequencer::saveToFile_Attr;
MObject PclSequencer::messagePart_Attr;
MObject PclSequencer::messageEmittor_Attr;
MObject PclSequencer::startFrame_Attr;
MObject PclSequencer::loadFilesDisplayOnOff_Attr;
MObject PclSequencer::containerOnOff_Attr;
MObject PclSequencer::locatorMatrix_Attr;

// Builds a snapshot of all plugs that are repeatedly queried during compute.
PclSequencer::EvaluateContext PclSequencer::buildEvaluateContext(MDataBlock& dataBlock) const
{
	EvaluateContext ctx;
	ctx.createMode = dataBlock.inputValue(pointsCreate_Attr).asShort();
	ctx.timeValue = dataBlock.inputValue(inTime_Attr).asTime().value();
	ctx.timeOffset = dataBlock.inputValue(timeOffset_Attr).asFloat();
	ctx.startFrame = dataBlock.inputValue(startFrame_Attr).asInt();
	ctx.containerMode = dataBlock.inputValue(containerOnOff_Attr).asShort();
	ctx.saveFormat = dataBlock.inputValue(savePCDPLY_Attr).asShort();
	ctx.saveEncoding = dataBlock.inputValue(saveAsciiBinary_Attr).asShort();
	ctx.saveMainPath = dataBlock.inputValue(saveMainPath_Attr).asString();
	ctx.saveFileName = dataBlock.inputValue(saveFileName_Attr).asString();
	ctx.saveTrigger = dataBlock.inputValue(saveToFile_Attr).asShort();
	ctx.locatorMatrix = dataBlock.inputValue(locatorMatrix_Attr).asMatrix();
	return ctx;
}



// Default constructor keeps member initialization centralized in headers.
PclSequencer::PclSequencer()
{}

// Destructor provided for symmetry/future expansion.
PclSequencer::~PclSequencer()
{}

// Maya factory hook for node instantiation.
void* PclSequencer::creator()
{
	return new PclSequencer();
}


// Main evaluation entry point. Handles loading, filtering, and exporting frames.
MStatus PclSequencer::compute(const MPlug& plug, MDataBlock& dataBlock)
{
	MStatus status = MS::kUnknownParameter;

	thisNode = thisMObject();

	const EvaluateContext ctx = buildEvaluateContext(dataBlock);
	short dataHandle_pointsCreate = ctx.createMode;
	MString dataHandle_mainPath = ctx.mainPath;
	MString dataHandle_subDir = ctx.subPath;
	float timeVal = static_cast<float>(ctx.timeValue);
	float timeOffset = ctx.timeOffset;
	int startFrame = ctx.startFrame;
	short containerOnOff = ctx.containerMode;
	short savePCDPLY = ctx.saveFormat;
	short saveAsciiBinary = ctx.saveEncoding;
	MString saveMainPath = ctx.saveMainPath;
	MString saveFileName = ctx.saveFileName;
	short saveToFile = ctx.saveTrigger;


	// Get particle system and emitter objects if not already initialized.
	if (particle_sys.name() == "")
	{
		getTheParticleSys();
		getTheEmittor_obj();
	}

	// Set load file display attribute (for scene refresh only).
	dataBlock.outputValue(loadFilesDisplayOnOff_Attr).setShort(1);

	MVectorArray inPosAry;
	MVectorArray inColAry;
	MVectorArray pointsInContainer;
	int numberOfPointsforFrame = 0;

	// Clear points if create mode is set to off.
	if (dataHandle_pointsCreate == 1)
	{	
		clearLoadedData(plug, dataBlock, inPosAry, inColAry);
		numberOfPointsforFrame = 0;
	}
	// Load points from pclLoader via dependency graph connection.
	else if (dataHandle_pointsCreate == 2)
	{
		// Get input points from pclLoader via dependency graph connection.
		MPlug pointsPositionPlug(thisNode, inPointsPosition_Attr);
		if (!pointsPositionPlug.isConnected())
		{
			MGlobal::displayError("pclSequencer: inPointsPosition must be connected to pclLoader output");
			dataBlock.setClean(plug);
			return MS::kFailure;
		}

		// Get points from pclLoader input.
		MObject pointPos_in = dataBlock.inputValue(inPointsPosition_Attr).data();
		MFnPointArrayData dataPointArrayFn_in;
		status = dataPointArrayFn_in.setObject(pointPos_in);
		if (status != MS::kSuccess)
		{
			MGlobal::displayError("pclSequencer: failed to read input points");
			dataBlock.setClean(plug);
			return MS::kFailure;
		}

		// Convert MPointArray to MVectorArray for particle system.
		MPointArray pointArray = dataPointArrayFn_in.array();
		const unsigned int pointCount = pointArray.length();
		inPosAry.setLength(pointCount);
		for (unsigned int i = 0; i < pointCount; ++i)
		{
			const MPoint& point = pointArray[i];
			inPosAry.set(MVector(point.x, point.y, point.z), i);
		}

		// Get colors from pclLoader input (if connected), otherwise default to black.
		MPlug pointsColorPlug(thisNode, inPointsColor_Attr);
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
				// Default to black if color read fails.
				inColAry.setLength(pointCount);
				for (unsigned int i = 0; i < pointCount; ++i)
				{
					inColAry.set(MVector(0.0, 0.0, 0.0), i);
				}
			}
		}
		else
		{
			// Default to black if no color input.
			inColAry.setLength(pointCount);
			for (unsigned int i = 0; i < pointCount; ++i)
			{
				inColAry.set(MVector(0.0, 0.0, 0.0), i);
			}
		}

		numberOfPointsforFrame = static_cast<int>(pointCount);

		// Update max particle count for emission rate calculation.
		if (pointCount > maxPartVal)
		{
			maxPartVal = pointCount;
		}

		applyContainerFilter(ctx, inPosAry, numberOfPointsforFrame);
		CHECK_MSTATUS_AND_RETURN_IT(setPointPosition(plug, dataBlock, inPosAry));
		CHECK_MSTATUS_AND_RETURN_IT(setPointColor(plug, dataBlock, inColAry));

		// Update first frame creation points for particle emission rate calculation.
		const unsigned int pointCount = inPosAry.length();
		if (firstFrameCreateionPoints.length() < maxPartVal)
		{
			// Initialize first frame points if needed.
			if (pointCount > 0)
			{
				firstFrameCreateionPoints.setLength(pointCount);
				for (unsigned int i = 0; i < pointCount; ++i)
				{
					firstFrameCreateionPoints.set(MPoint(inPosAry[i].x, inPosAry[i].y, inPosAry[i].z), i);
				}
			}
			else
			{
				// Pad with zeros if needed for max particle count.
				firstFrameCreateionPoints.setLength(maxPartVal);
				for (unsigned int i = 0; i < maxPartVal; ++i)
				{
					firstFrameCreateionPoints.set(MPoint(0, 0, 0), i);
				}
			}
		}

		// Manage particle emission rate based on current frame and particle count.
		if (ctx.timeValue == ctx.startFrame || ctx.timeValue == ctx.startFrame + 1)
		{
			if (particle_sys.count() >= firstFrameCreateionPoints.length())
			{
				setEmitterRate(0);
			}
			else if (firstFrameCreateionPoints.length() > particle_sys.count())
			{
				setEmitterRate(maxPartVal * 30);
			}
		}
		else if (particle_sys.count() >= firstFrameCreateionPoints.length())
		{
			setEmitterRate(0);
		}
	}

	// Output the number of points in the current frame.
	dataBlock.outputValue(pointNumber_Attr).setInt(numberOfPointsforFrame);
	
	// Handle save-to-file trigger if enabled.
	if (saveToFile == 1) {
		handleSaveToFile(ctx, dataBlock);
	}

	// Set load file display attribute to off.
	dataBlock.outputValue(loadFilesDisplayOnOff_Attr).setShort(0);
	dataBlock.setClean(plug);
	
	return status;
}

// Clears all cached frame data and resets key output plugs.
void PclSequencer::clearLoadedData(const MPlug& plug, MDataBlock& dataBlock, MVectorArray& positions, MVectorArray& colors)
{
	MGlobal::displayInfo("Clear points");
	dataBlock.outputValue(loadFilesDisplayOnOff_Attr).setShort(1);

	positions.clear();
	colors.clear();
	firstFrameCreateionPoints.clear();
	maxPartVal = 0;
	setEmitterRate(0);
	particle_sys.setCount(0);

	setPointPosition(plug, dataBlock, positions);
	setPointColor(plug, dataBlock, colors);

	MTime t;
	t.setValue(1);
	particle_sys.evaluateDynamics(t, true);

	MSelectionList node_list;
	if (node_list.add(particle_sys.name()) == MS::kSuccess)
	{
		MObject particle_obj;
		if (node_list.getDependNode(0, particle_obj) == MS::kSuccess)
		{
			MStatus status;
			MFnDependencyNode fnPart(particle_obj, &status);
			if (status)
			{
				MPlug particlePluginCreate = fnPart.findPlug("createPoints", false, &status);
				if (status)
				{
					particlePluginCreate.setShort(0);
				}
			}
		}
	}
}

// Applies the container mode to either keep or remove points based on bounds.
void PclSequencer::applyContainerFilter(const EvaluateContext& ctx, MVectorArray& positions, int& pointCount)
{
	if (ctx.containerMode != 2 && ctx.containerMode != 3)
	{
		return;
	}

	MBoundingBox bbox;
	MPoint corner1(2.5, 2.5, 2.5);
	corner1 *= ctx.locatorMatrix;
	MPoint corner2(-2.5, -2.5, -2.5);
	corner2 *= ctx.locatorMatrix;
	bbox.expand(corner1);
	bbox.expand(corner2);

	MVectorArray filtered;
	MVectorArray counted;
	const bool keepInside = (ctx.containerMode == 2);
	const MVector zero(0, 0, 0);

	const unsigned int pointCount = positions.length();
	// Reserve space for filtered array (worst case: all points pass filter).
	filtered.setLength(pointCount);
	unsigned int filteredIdx = 0;

	for (unsigned int i = 0; i < pointCount; ++i)
	{
		const bool contains = bbox.contains(positions[i]);
		const bool accept = (keepInside && contains) || (!keepInside && !contains);

		if (accept)
		{
			filtered.set(positions[i], filteredIdx++);
			if (positions[i].x != 0 || positions[i].y != 0 || positions[i].z != 0)
			{
				counted.append(positions[i]);
			}
		}
		else if (keepInside)
		{
			filtered.set(zero, filteredIdx++);
		}
	}

	// Resize to actual filtered count.
	filtered.setLength(filteredIdx);

	positions = filtered;
	pointCount = counted.length();
}


// Responds to the save trigger by exporting frames to disk.
void PclSequencer::handleSaveToFile(const EvaluateContext& ctx, MDataBlock& dataBlock)
{
	dataBlock.outputValue(loadFilesDisplayOnOff_Attr).setShort(1);

	if (ctx.saveMainPath.length() > 0)
	{
		MString pathFile = ctx.saveMainPath.asChar();
		savePointsToFile(pathFile, ctx.saveFileName, ctx.saveFormat, ctx.saveEncoding);
	}
	else
	{
		MGlobal::displayInfo("** No File Path To Save! **");
	}

	MSelectionList node_list;
	if (node_list.add(particle_sys.name()) == MS::kSuccess)
	{
		MObject particle_obj;
		if (node_list.getDependNode(0, particle_obj) == MS::kSuccess)
		{
			MStatus status;
			MFnDependencyNode fnPart(particle_obj, &status);
			if (status)
			{
				MPlug particle_pluginSaveF = fnPart.findPlug("SaveToFile", false, &status);
				if (status)
				{
					particle_pluginSaveF.setShort(0);
				}
			}
		}
	}
}


/// Copies the supplied positions into the particle system's outPositionPP.
MStatus PclSequencer::setPointPosition(const MPlug& plug, MDataBlock& dataBlock, MVectorArray inPointPos)
{
	MStatus status = MS::kSuccess;
	MFnVectorArrayData dataVectorArrayFn;
	MVectorArray outPosArray;

	unsigned int nParticles = 0;
	int nSignedPart = dataBlock.inputValue(particleCount).asInt();
	if (nSignedPart > 0)
	{
		nParticles = nSignedPart;
	}

	// Get pointer to destination attribute: outPositionPP.
	MObject posD = dataBlock.outputValue(outPositionPP).data();
	status = dataVectorArrayFn.setObject(posD);
	if (status == MS::kSuccess)
	{
		outPosArray = dataVectorArrayFn.array();
	}

	outPosArray.setLength(nParticles);
	dataVectorArrayFn.set(inPointPos);


	return MS::kSuccess;
}


/// Copies the supplied colors into the particle system's outColorPP.
MStatus PclSequencer::setPointColor(const MPlug& plug, MDataBlock& dataBlock, MVectorArray inPointCol)
{
	MStatus status = MS::kSuccess;

	// Touch color plugs so Maya regards them as clean after we overwrite outputs.
	dataBlock.inputValue(computeNodeColor);
	dataBlock.inputValue(computeNodeColorR);
	dataBlock.inputValue(computeNodeColorG);
	dataBlock.inputValue(computeNodeColorB);

	bool doUcoord = false;
	bool doVcoord = false;
	bool doOutColor = (plug.attribute() == outColorPP);
	bool doOutValue = (plug.attribute() == outValuePP);

	// Get pointers to the source attributes: aUCoordPP, aVCoordPP.
	//
	MFnDoubleArrayData dataDoubleArrayFn;
	MObject uCoordD = dataBlock.inputValue(uCoordPP).data();
	status = dataDoubleArrayFn.setObject(uCoordD);
	MDoubleArray uAry;
	if (status == MS::kSuccess)
	{
		uAry = dataDoubleArrayFn.array();
		if (uAry.length())
		{
			doUcoord = true;
		}
	}

	status = MS::kSuccess;
	MObject vCoordD = dataBlock.inputValue(vCoordPP).data();
	status = dataDoubleArrayFn.setObject(vCoordD);
	MDoubleArray vAry;
	if (status == MS::kSuccess)
	{
		vAry = dataDoubleArrayFn.array();
		if (vAry.length())
		{
			doVcoord = true;
		}
	}

	// Get pointers to destination attributes: outColorPP, outValuePP.
	//
	status = MS::kSuccess;
	MFnVectorArrayData dataVectorArrayFn;
	MVectorArray outColorAry;
	if (doOutColor)
	{
		MObject colorD = dataBlock.outputValue(outColorPP).data();
		status = dataVectorArrayFn.setObject(colorD);

		if (status == MS::kSuccess)
		{
			outColorAry = dataVectorArrayFn.array();
		}
	}

	status = MS::kSuccess;
	MDoubleArray outValueAry;
	if (doOutValue)
	{
		MObject valueD = dataBlock.outputValue(outValuePP).data();
		status = dataDoubleArrayFn.setObject(valueD);

		if (status == MS::kSuccess)
		{
			outValueAry = dataDoubleArrayFn.array();
		}
	}

	unsigned int uCount = (doUcoord ? uAry.length() : 0);
	unsigned int vCount = (doVcoord ? vAry.length() : 0);
	unsigned int count = ((uCount) > (vCount) ? (uCount) : (vCount));

	if (doOutColor) outColorAry.setLength(count);
	if (doOutValue) outValueAry.setLength(count);


	dataVectorArrayFn.set(inPointCol);
	dataDoubleArrayFn.set(outValueAry);
	return MS::kSuccess;
}

/// Placeholder for the future save-to-disk implementation.
MStatus PclSequencer::savePointsToFile(MString pathFile, MString saveFileName, short savePCDPLY, short saveAsciiBinary)
{
	MStatus status;
	MGlobal::displayInfo(MString(" saveToFile To: ") + pathFile);

	return MS::kSuccess;
}


// Computes the maximum number of particles across all frames.
int PclSequencer::getMaxParticlrCountInFrame(MIntArray pixelSize_list)
{
	MGlobal::displayInfo(MString("**  getMaxParticlrCountInFrame  "));
	
	MIntArray minMaxPixelSize_list;
	MIntArray tempPixelSize_list;
	// Find minimum and maximum pixel sizes, filtering out zero values.
	const int inputLength = pixelSize_list.length();
	if (inputLength >= 2)
	{
		for (int i = 0; i < inputLength; i++)
		{
			if (pixelSize_list[i] != 0)
			{
				tempPixelSize_list.append(pixelSize_list[i]);
			}
		}
	}

	const int tempLength = tempPixelSize_list.length();
	if (tempLength >= 2)
	{
		int minPix, maxPix;
		maxPix = minPix = tempPixelSize_list[0];
		for (int i = 0; i < tempLength; i++)
		{

			if (tempPixelSize_list[i] > maxPix) //compare biggest value with current element
			{
				maxPix = tempPixelSize_list[i];
			}
			if (tempPixelSize_list[i] < minPix) //compares smallest value with current element
			{
				if (tempPixelSize_list[i] > 0)
				{
					minPix = tempPixelSize_list[i];
				}
			}
		}
		minMaxPixelSize_list.append(minPix);
		minMaxPixelSize_list.append(maxPix);
	}

	if (tempPixelSize_list.length() == 1)
	{
		minMaxPixelSize_list.append(0);
		minMaxPixelSize_list.append(tempPixelSize_list[0]);
	}
	if (tempPixelSize_list.length() == 0)
	{
		minMaxPixelSize_list.append(0);
		minMaxPixelSize_list.append(0);
	}

	if (pixelSize_list.length() == 1)
	{
		minMaxPixelSize_list.append(0);
		minMaxPixelSize_list.append(pixelSize_list[0]);
	}

	return minMaxPixelSize_list[1];
}

// Resolves and caches the particle system connected to this node.
MStatus PclSequencer::getTheParticleSys()
{
	MStatus status;
	MFnDependencyNode fnPrNode(thisNode);
	MPlug particle_plug = fnPrNode.findPlug("particlesSys_msg", true);
	MPlugArray mPlugArray;
	particle_plug.connectedTo(mPlugArray, true, false);
	if (mPlugArray.length() == 0) {
		return MS::kInvalidParameter;
	}

	MObject particle_obj = mPlugArray[0].node();
	particle_sys.setObject(particle_obj);

	return MS::kSuccess;

}


// Resolves and caches the emitter connected to this node.
MStatus PclSequencer::getTheEmittor_obj()
{
	//MGlobal::displayInfo(MString("****  getTheEmittor_obj  ****"));
	
	MStatus status;
	MFnDependencyNode fnPrNode(thisNode);
	MPlug emittor_plug = fnPrNode.findPlug("emittorSys_msg", true);
	MPlugArray mPlugArray;
	emittor_plug.connectedTo(mPlugArray, true, true);
	if (mPlugArray.length() == 0) {
		return MS::kInvalidParameter;
	}
	
	// Set emitter to object.
	emittor_obj = mPlugArray[0].node();
	

	return MS::kSuccess;

}


// Convenience helper for adjusting the emitter rate safely.
MStatus PclSequencer::setEmitterRate(unsigned int rateValue)
{
	MStatus status = MS::kSuccess;
	
	// Get setRate attribute and set value.
	MFnDependencyNode fnThis(emittor_obj, &status);
	MPlug emittor_plugin = fnThis.findPlug("rate", false, &status);
	emittor_plugin.setInt(rateValue);

	return status;
}

// Declares which outputs need re-evaluation when particular inputs change.
MStatus PclSequencer::setDependentsDirty(const MPlug &plugBeingDirtied, MPlugArray &affectedPlugs)
{
	MStatus status;
	MObject thisNode = thisMObject();

	// Check if "createPoints" is updated.
	if (plugBeingDirtied.partialName() == "createPoints") {
		MPlug pLoadF(thisNode, PclSequencer::loadFilesDisplayOnOff_Attr);
		affectedPlugs.append(pLoadF);
		MPlug pOutP(thisNode, PclSequencer::outPositionPP);
		affectedPlugs.append(pOutP);
	}

	// Check if "inTime" is updated.
	if (plugBeingDirtied.partialName() == "inTime") {
		MPlug pOutP(thisNode, PclSequencer::outPositionPP);
		affectedPlugs.append(pOutP);

		MPlug pNumber(thisNode, PclSequencer::pointNumber_Attr);
		affectedPlugs.append(pNumber);
	}

	// Check if container mode is updated.
	if (plugBeingDirtied.partialName() == "containerOnOff")
	{
		MPlug pNumber(thisNode, PclSequencer::pointNumber_Attr);
		affectedPlugs.append(pNumber);
	}

	// Check if "locatorMatrix" is updated.
	if (plugBeingDirtied.partialName() == "locatorMatrix") {
		MPlug pNumber(thisNode, PclSequencer::pointNumber_Attr);
		affectedPlugs.append(pNumber);
	}
	
	// Check if "createPoints" is updated.
	if (plugBeingDirtied.partialName() == "SaveToFile") {
		MPlug pSaveF(thisNode, PclSequencer::loadFilesDisplayOnOff_Attr);
		affectedPlugs.append(pSaveF);
	}


	
	return MS::kSuccess;
}



// Defines all node attributes and their relationships.
MStatus PclSequencer::initialize()
{
	MStatus status = MS::kSuccess;

	// In addition to the standard arrayMapper attributes,
	// create an additional vector attribute.
	//
	MFnTypedAttribute typedAttrFn;
	MVectorArray defaultVectArray;
	MFnVectorArrayData vectArrayDataFn;
	MFnNumericAttribute numAttrFn;
	MFnMessageAttribute msgAttr;
	MFnEnumAttribute eAttr;
	MFnMatrixAttribute mFnMatrixAtt;
	MFnUnitAttribute uAttr;

	
	// Particle count attribute.
	numAttrFn.create("particleCount", "pc", MFnNumericData::kInt);
	particleCount = numAttrFn.object();
	addAttribute(particleCount);

	// Missing frames handling: how to treat missing frames in a sequence.
	missingFrames_Attr = eAttr.create("missingFrames", "missingFrames");
	eAttr.addField("hide missing", 0);
	eAttr.addField("freez missing", 1);
	eAttr.addField("skip missing", 2);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(missingFrames_Attr);


	// Point creation mode: off (0), clear points (1), or load from pclLoader (2).
	pointsCreate_Attr = eAttr.create("createPoints", "createPoints");
	eAttr.addField("off", 0);
	eAttr.addField("clear points", 1);
	eAttr.addField("load from pclLoader", 2);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(pointsCreate_Attr);


	// Input points from pclLoader (required - node receives data via dependency graph).
	MPointArray defaultPointArray;
	MFnPointArrayData pointArrayDataFn;
	pointArrayDataFn.create(defaultPointArray);
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

	// Output: number of points in the current frame.
	pointNumber_Attr = numAttrFn.create("numberOfPoints", "numberOfPoints", MFnNumericData::kInt);
	numAttrFn.setKeyable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setReadable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setChannelBox(true);
	numAttrFn.setConnectable(true);
	addAttribute(pointNumber_Attr);

	// Save file format: PCD (0) or PLY (1).
	savePCDPLY_Attr = eAttr.create("Save_PCD_PLY", "Save_PCD_PLY");
	eAttr.addField("PCD", 0);
	eAttr.addField("PLY", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(savePCDPLY_Attr);

	// Save file encoding: Ascii (0), Binary (1), or BinaryCompressed (2).
	saveAsciiBinary_Attr = eAttr.create("Save_Ascii_Binary", "Save_Ascii_Binary");
	eAttr.addField("Ascii", 0);
	eAttr.addField("Binary", 1);
	eAttr.addField("BinaryCompressed", 2);
	eAttr.setDefault(1);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(saveAsciiBinary_Attr);

	// Main path for saving files.
	saveMainPath_Attr = typedAttrFn.create("SaveMainPath", "SaveMainPath", MFnData::kString);
	typedAttrFn.setStorable(true);
	typedAttrFn.setWritable(true);
	typedAttrFn.setKeyable(false);
	typedAttrFn.setReadable(true);
	addAttribute(saveMainPath_Attr);

	// File name for saving.
	saveFileName_Attr = typedAttrFn.create("SaveFileName", "SaveFileName", MFnData::kString);
	typedAttrFn.setStorable(true);
	typedAttrFn.setWritable(true);
	typedAttrFn.setKeyable(false);
	typedAttrFn.setReadable(true);
	addAttribute(saveFileName_Attr);

	// Save to file trigger.
	saveToFile_Attr = eAttr.create("SaveToFile", "SaveToFile");
	eAttr.addField("off", 0);
	eAttr.addField("Save", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(saveToFile_Attr);

	// Input time attribute (typically connected to Maya's time).
	inTime_Attr = uAttr.create("inTime", "inTime", MFnUnitAttribute::kTime);
	addAttribute(inTime_Attr);


	// Time offset for frame selection in sequences.
	timeOffset_Attr = numAttrFn.create("timeOffset", "timeOffset", MFnNumericData::kFloat, 0);
	numAttrFn.setKeyable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setChannelBox(true);
	numAttrFn.setConnectable(true);
	addAttribute(timeOffset_Attr);


	// Output time attribute (for updating particle evaluation).
	outTime_Attr = uAttr.create("outTime", "outTime", MFnUnitAttribute::kTime);
	uAttr.setKeyable(true);
	uAttr.setStorable(true);
	uAttr.setWritable(true);
	uAttr.setChannelBox(true);
	uAttr.setConnectable(true);
	addAttribute(outTime_Attr);

	// Message attribute for particle system connection.
	messagePart_Attr = msgAttr.create("particlesSys_msg", "particlesSys_msg");
	msgAttr.setReadable(true);
	msgAttr.setStorable(true);
	msgAttr.setConnectable(true);
	addAttribute(messagePart_Attr);

	// Message attribute for emitter connection.
	messageEmittor_Attr = msgAttr.create("emittorSys_msg", "emittorSys_msg");
	msgAttr.setReadable(true);
	msgAttr.setStorable(true);
	msgAttr.setConnectable(true);
	addAttribute(messageEmittor_Attr);

	vectArrayDataFn.create(defaultVectArray);
	typedAttrFn.create("outPosition", "opos", MFnData::kVectorArray, vectArrayDataFn.object(), &status);
	typedAttrFn.setWritable(false);
	typedAttrFn.setStorable(false);
	outPositionPP = typedAttrFn.object();
	addAttribute(outPositionPP);


	// Start frame for sequence playback.
	startFrame_Attr = numAttrFn.create("startFrame", "startFrame", MFnNumericData::kInt);
	numAttrFn.setKeyable(true);
	numAttrFn.setStorable(true);
	numAttrFn.setWritable(true);
	numAttrFn.setChannelBox(true);
	numAttrFn.setConnectable(true);
	addAttribute(startFrame_Attr);

	// Load files display attribute (for scene refresh only).
	loadFilesDisplayOnOff_Attr = eAttr.create("LoadFilesDisplay", "LoadFilesDisplay");
	eAttr.addField("Off", 0);
	eAttr.addField("On", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	eAttr.setChannelBox(false);
	addAttribute(loadFilesDisplayOnOff_Attr);


	// Container mode: Off (0), DisplayOn (1), All In (2), or All Out (3).
	containerOnOff_Attr = eAttr.create("containerOnOff", "containerOnOff");
	eAttr.addField("Off", 0);
	eAttr.addField("DisplayOn", 1);
	eAttr.addField("All In", 2);
	eAttr.addField("All Out", 3);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(containerOnOff_Attr);
	

	// Locator (self) world matrix attribute.
	locatorMatrix_Attr = mFnMatrixAtt.create("locatorMatrix", "locatorMatrix");
	mFnMatrixAtt.setReadable(false);
	mFnMatrixAtt.setStorable(false);
	mFnMatrixAtt.setConnectable(true);
	addAttribute(locatorMatrix_Attr);
	
	


	return status;


}