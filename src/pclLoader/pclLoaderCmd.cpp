#include "pclLoaderCmd.h"
#include "pclCommandHelpers.h"

// Maya API
#include <maya/MArgDatabase.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MDagModifier.h>
#include <maya/MDGModifier.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>
#include <maya/MTime.h>
#include <boost/algorithm/string.hpp>
#include <string>

// Common macro for error checking
#ifndef CHECK_MSTATUS_AND_RETURN_IT
#define CHECK_MSTATUS_AND_RETURN_IT(status) \
	do { \
		if ((status) != MS::kSuccess) { \
			return (status); \
		} \
	} while (0)
#endif

namespace
{
// Command flag names.
constexpr char kFlagNameShort[] = "-n";
constexpr char kFlagNameLong[] = "-name";
constexpr char kFlagMainPathShort[] = "-mP";
constexpr char kFlagMainPathLong[] = "-mainPath";
constexpr char kFlagClusterShort[] = "-pc";
constexpr char kFlagClusterLong[] = "-pclCluster";
constexpr char kFlagSequencerShort[] = "-ps";
constexpr char kFlagSequencerLong[] = "-pclSequencer";
constexpr char kFlagRaycasterShort[] = "-pr";
constexpr char kFlagRaycasterLong[] = "-pclRaycaster";
constexpr char kFlagCameraShort[] = "-sc";
constexpr char kFlagCameraLong[] = "-selCamera";
constexpr char kFlagHelpShort[] = "-h";
constexpr char kFlagHelpLong[] = "-help";

// Maya node/attribute names used throughout the command.
constexpr char kNodeTypePclLoader[] = "pclLoader";
constexpr char kNodeTypePclContainer[] = "pclContainer";
constexpr char kNodeTypePclCluster[] = "pclCluster";
constexpr char kNodeTypePclSequencer[] = "pclSequencer";
constexpr char kNodeTypePclRaycaster[] = "pclRaycaster";
constexpr char kTimeNodeName[] = "time1";

constexpr char kAttrWorldMatrix[] = "worldMatrix";
constexpr char kAttrInContainerMatrix[] = "inContainerMatrix";
constexpr char kAttrContainerOnOff[] = "containerOnOff";
constexpr char kAttrInTime[] = "inTime";
constexpr char kAttrOutTime[] = "outTime";
constexpr char kAttrMainPath[] = "mainPath";
constexpr char kAttrOutPointsPosition[] = "outPointsPosition";
constexpr char kAttrInPointsPosition[] = "inPointsPosition";
constexpr char kAttrOutBoundingBoxCorner[] = "outBoundingBoxCorner";
	constexpr char kAttrInBoundingBoxCorner[] = "inBoundingBoxCorner";
	constexpr char kAttrOutPointsPositionVec[] = "outPointsPosition_vec";
constexpr char kAttrOutPointsColorVec[] = "outPointsColor_vec";
constexpr char kAttrInPointsColor[] = "inPointsColor";
constexpr char kAttrOutPositionRaycast[] = "outPositionRaycast";
constexpr char kAttrInRaycasterPoints[] = "inlRaycasterPoints";

// Using shared helpers from PclCommandHelpers namespace
using PclCommandHelpers::createDagNode;
using PclCommandHelpers::renameParentTransform;
using PclCommandHelpers::appendNodeNameIfValid;
}

// Print the command help text to the script editor.
void DisplayHelp() {
	MString help;
	help += "Flags:\n";
	help += "-name (-n) <string>           : Base name for created nodes.\n";
	help += "-mainPath (-mP) <string>      : Path to the PCL/PLY sequence.\n";
	help += "-pclCluster (-pc) <bool>      : Create and connect pclCluster node.\n";
	help += "-pclSequencer (-ps) <bool>    : Create and connect pclSequencer node.\n";
	help += "-pclRaycaster (-pr) <bool>    : Create and connect pclRaycaster node.\n";
	help += "-selCamera (-sc) <string>     : Camera to connect when -pclRaycaster is true.\n";
	help += "-help (-h)                    : Display this help.\n";
	MGlobal::displayInfo(help);
}



// Construct command with default naming prefix (used if -name not provided).
PclLoaderCmd::PclLoaderCmd() :
	mName("PclLoader#")
{

}

// Describe supported command flags for MSyntax.
MSyntax PclLoaderCmd::newSyntax() {
	MSyntax syntax;
	syntax.addFlag(kFlagNameShort, kFlagNameLong, MSyntax::kString);
	syntax.addFlag(kFlagMainPathShort, kFlagMainPathLong, MSyntax::kString);
	syntax.addFlag(kFlagClusterShort, kFlagClusterLong, MSyntax::kBoolean);
	syntax.addFlag(kFlagSequencerShort, kFlagSequencerLong, MSyntax::kBoolean);
	syntax.addFlag(kFlagRaycasterShort, kFlagRaycasterLong, MSyntax::kBoolean);
	syntax.addFlag(kFlagCameraShort, kFlagCameraLong, MSyntax::kString);
	syntax.addFlag(kFlagHelpShort, kFlagHelpLong);
	syntax.setObjectType(MSyntax::kSelectionList);
	syntax.useSelectionAsDefault(true);

	return syntax;

}


void* PclLoaderCmd::creator()
{
	return new PclLoaderCmd;
}



// Entry point for command execution: parse → create nodes → connect → animate → rename.
MStatus PclLoaderCmd::doIt(const MArgList& argList) {

	CHECK_MSTATUS_AND_RETURN_IT(parseArguments(argList));
	if (mShowHelp)
	{
		DisplayHelp();
		return MS::kSuccess;
	}
	CHECK_MSTATUS_AND_RETURN_IT(createRequestedNodes());
	CHECK_MSTATUS_AND_RETURN_IT(connectAll());
	CHECK_MSTATUS_AND_RETURN_IT(setKeyframesOnTimeAttr());
	renameRequestedNodes();

	MStringArray nodeList;
	collectCreatedNodeNames(nodeList);
	MPxCommand::setResult(nodeList);
	MGlobal::select(mPclLoader_obj, MGlobal::kReplaceList);

	return MS::kSuccess;
}

// Parse command flags, validate input, and populate member variables.
MStatus PclLoaderCmd::parseArguments(const MArgList& argList)
{
	mNamePclLoader.clear();
	mNamePclContainer.clear();
	mNamePclCluster.clear();
	mNamePclSequencer.clear();
	mNamePclRaycaster.clear();
	mMainPath.clear();
	mSelCamera.clear();
	mPclCluster_flag = false;
	pclSequencer_flag = false;
	pclRaycaster_flag = false;

	MStatus status;
	MArgDatabase argData(syntax(), argList, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	mShowHelp = argData.isFlagSet(kFlagHelpShort) || argData.isFlagSet(kFlagHelpLong);
	if (mShowHelp)
	{
		return MS::kSuccess;
	}

	// Optional base name for newly created nodes.
	if (argData.isFlagSet(kFlagNameShort) || argData.isFlagSet(kFlagNameLong))
	{
		const char* flag = argData.isFlagSet(kFlagNameShort) ? kFlagNameShort : kFlagNameLong;
		mName = argData.flagArgumentString(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		mNamePclLoader = mName + "pclLoader#";
		mNamePclContainer = mName + "pclContainer#";
		mNamePclCluster = mName + "pclCluster#";
		mNamePclSequencer = mName + "pclSequencer#";
		mNamePclRaycaster = mName + "pclRaycaster#";
	}

	// Optional main path string.
	const char* mainPathFlag = argData.isFlagSet(kFlagMainPathShort) ? kFlagMainPathShort :
		(argData.isFlagSet(kFlagMainPathLong) ? kFlagMainPathLong : nullptr);
	if (mainPathFlag)
	{
		mMainPath = argData.flagArgumentString(mainPathFlag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	// Optional: enable cluster node creation.
	if (argData.isFlagSet(kFlagClusterShort) || argData.isFlagSet(kFlagClusterLong))
	{
		const char* flag = argData.isFlagSet(kFlagClusterShort) ? kFlagClusterShort : kFlagClusterLong;
		mPclCluster_flag = argData.flagArgumentBool(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	// Optional: enable sequencer node creation.
	if (argData.isFlagSet(kFlagSequencerShort) || argData.isFlagSet(kFlagSequencerLong))
	{
		const char* flag = argData.isFlagSet(kFlagSequencerShort) ? kFlagSequencerShort : kFlagSequencerLong;
		pclSequencer_flag = argData.flagArgumentBool(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	// Optional: enable raycaster node creation.
	if (argData.isFlagSet(kFlagRaycasterShort) || argData.isFlagSet(kFlagRaycasterLong))
	{
		const char* flag = argData.isFlagSet(kFlagRaycasterShort) ? kFlagRaycasterShort : kFlagRaycasterLong;
		pclRaycaster_flag = argData.flagArgumentBool(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	if (pclRaycaster_flag)
	{
		// Raycaster requires a single camera: use -selCamera or current selection.
		MSelectionList cameraSel;
		if (argData.isFlagSet(kFlagCameraShort) || argData.isFlagSet(kFlagCameraLong))
		{
			const char* flag = argData.isFlagSet(kFlagCameraShort) ? kFlagCameraShort : kFlagCameraLong;
			mSelCamera = argData.flagArgumentString(flag, 0, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			CHECK_MSTATUS_AND_RETURN_IT(MGlobal::getSelectionListByName(mSelCamera, cameraSel));
		}
		else
		{
			CHECK_MSTATUS_AND_RETURN_IT(argData.getObjects(cameraSel));
		}

		if (cameraSel.length() != 1)
		{
			MGlobal::displayError("Select a single camera or use -selCamera when enabling -pclRaycaster.");
			return MS::kInvalidParameter;
		}

		MDagPath cameraPath;
		CHECK_MSTATUS_AND_RETURN_IT(cameraSel.getDagPath(0, cameraPath));
		CHECK_MSTATUS_AND_RETURN_IT(getShapeNode(cameraPath));

		MString camFullPathName = cameraPath.fullPathName();
		MStringArray parts;
		camFullPathName.split('|', parts);
		if (parts.length() >= 2)
		{
			mSelCamera = parts[parts.length() - 2];
		}
		else
		{
			mSelCamera = camFullPathName;
		}
	}

	return MS::kSuccess;
}

// Create the nodes requested by the parsed flags.
MStatus PclLoaderCmd::createRequestedNodes()
{
	CHECK_MSTATUS_AND_RETURN_IT(createDagNode(kNodeTypePclLoader, mPclLoader_obj));
	CHECK_MSTATUS_AND_RETURN_IT(createDagNode(kNodeTypePclContainer, mPclContainer_obj));

	if (mPclCluster_flag)
	{
		// Call pclClusteringCmd to create and configure cluster node
		MString command = "pclClusteringCmd";
		if (mName.length() > 0)
		{
			command += " -name \"" + mName + "\"";
		}
		// Pass pclLoader node name for connection
		MFnDependencyNode fnPclLoader(mPclLoader_obj);
		command += " -pclLoader \"" + fnPclLoader.name() + "\"";
		MStringArray commandResult;
		CHECK_MSTATUS_AND_RETURN_IT(MGlobal::executeCommand(command, commandResult, false, true));
		if (commandResult.length() < 1)
		{
			MGlobal::displayError("pclClusteringCmd did not return the expected node name.");
			return MS::kFailure;
		}
		// Extract cluster node name from result
		MSelectionList clusterList;
		CHECK_MSTATUS_AND_RETURN_IT(clusterList.add(commandResult[0]));
		CHECK_MSTATUS_AND_RETURN_IT(clusterList.getDependNode(0, mPclCluster_obj));
	}
	else
	{
		mPclCluster_obj = MObject::kNullObj;
	}

	if (pclSequencer_flag)
	{
		// Call pclSequencerCmd to create and configure sequencer setup
		MString command = "pclSequencerCmd";
		if (mName.length() > 0)
		{
			command += " -name \"" + mName + "\"";
		}
		// Pass pclLoader node name for connection
		MFnDependencyNode fnPclLoader(mPclLoader_obj);
		command += " -pclLoader \"" + fnPclLoader.name() + "\"";
		MStringArray commandResult;
		CHECK_MSTATUS_AND_RETURN_IT(MGlobal::executeCommand(command, commandResult, false, true));
		if (commandResult.length() < 4)
		{
			MGlobal::displayError("pclSequencerCmd did not return the expected node list.");
			return MS::kFailure;
		}
		// Extract sequencer mapper node (last in result: particle, emitter, nucleus, mapper)
		MSelectionList sequencerList;
		CHECK_MSTATUS_AND_RETURN_IT(sequencerList.add(commandResult[3]));
		CHECK_MSTATUS_AND_RETURN_IT(sequencerList.getDependNode(0, mPclSequencer_obj));
	}
	else
	{
		mPclSequencer_obj = MObject::kNullObj;
	}

	if (pclRaycaster_flag)
	{
		// Call pclRaycasterCmd to create and configure raycaster setup
		MString command = "pclRaycasterCmd";
		if (mName.length() > 0)
		{
			command += " -name \"" + mName + "\"";
		}
		if (mSelCamera.length() > 0)
		{
			command += " -selCamera \"" + mSelCamera + "\"";
		}
		// Pass pclLoader node name for connection
		MFnDependencyNode fnPclLoader(mPclLoader_obj);
		command += " -pclLoader \"" + fnPclLoader.name() + "\"";
		MStringArray commandResult;
		CHECK_MSTATUS_AND_RETURN_IT(MGlobal::executeCommand(command, commandResult, false, true));
		if (commandResult.length() < 1)
		{
			MGlobal::displayError("pclRaycasterCmd did not return the expected node name.");
			return MS::kFailure;
		}
		// Extract raycaster node name from result
		MSelectionList raycasterList;
		CHECK_MSTATUS_AND_RETURN_IT(raycasterList.add(commandResult[0]));
		CHECK_MSTATUS_AND_RETURN_IT(raycasterList.getDependNode(0, mPclRaycaster_obj));
	}
	else
	{
		mPclRaycaster_obj = MObject::kNullObj;
	}

	return MS::kSuccess;
}

// Rename created nodes when a base name was supplied.
void PclLoaderCmd::renameRequestedNodes()
{
	if (mNamePclLoader.length() == 0)
	{
		return;
	}

	renameParentTransform(mPclLoader_obj, mNamePclLoader);
	renameParentTransform(mPclContainer_obj, mNamePclContainer);
	if (mPclCluster_flag)
	{
		// Cluster node is renamed by pclClusteringCmd, no need to rename here
	}
	if (pclSequencer_flag)
	{
		// Sequencer nodes are renamed by pclSequencerCmd, no need to rename here
	}
	if (pclRaycaster_flag)
	{
		// Raycaster node is renamed by pclRaycasterCmd, no need to rename here
	}
}

// Collect the node names to return in command result.
void PclLoaderCmd::collectCreatedNodeNames(MStringArray& outNames) const
{
	appendNodeNameIfValid(mPclLoader_obj, outNames);
	appendNodeNameIfValid(mPclContainer_obj, outNames);
	appendNodeNameIfValid(mPclCluster_obj, outNames);
	
	// For sequencer, just add the mapper node (pclSequencerCmd handles its own result)
	if (pclSequencer_flag)
	{
		if (!mPclSequencer_obj.isNull())
		{
			MFnDependencyNode fnSequencer(mPclSequencer_obj);
			outNames.append(fnSequencer.name());
		}
	}
	
	appendNodeNameIfValid(mPclRaycaster_obj, outNames);
}


// Connect all required attributes between the newly created nodes.
MStatus PclLoaderCmd::connectAll()
{
	MStatus status;
	MDagModifier dagMod;

	MFnDependencyNode fnPclLoader(mPclLoader_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDependencyNode fnPclContainer(mPclContainer_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Route the container's worldMatrix to the loader.
	MFnDagNode containerShape(mPclContainer_obj);
	MObject containerTransform = (containerShape.parentCount() > 0) ? containerShape.parent(0) : mPclContainer_obj;
	MFnDagNode containerTransformFn(containerTransform);
	MPlug containerWorldMatrix = containerTransformFn.findPlug(kAttrWorldMatrix, true, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	containerWorldMatrix = containerWorldMatrix.elementByLogicalIndex(0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug loaderInMatrix = fnPclLoader.findPlug(kAttrInContainerMatrix, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(containerWorldMatrix, loaderInMatrix));

	// Synchronize containerOnOff between container and loader.
	MPlug loaderContainerOnOff = fnPclLoader.findPlug(kAttrContainerOnOff, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug containerContainerOnOff = fnPclContainer.findPlug(kAttrContainerOnOff, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(containerContainerOnOff, loaderContainerOnOff));

	// Connect time1.outTime to pclLoader.inTime.
	MPlug loaderInTime = fnPclLoader.findPlug(kAttrInTime, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MSelectionList timeList;
	CHECK_MSTATUS_AND_RETURN_IT(MGlobal::getSelectionListByName(kTimeNodeName, timeList));
	MObject timeObj;
	CHECK_MSTATUS_AND_RETURN_IT(timeList.getDependNode(0, timeObj));
	MFnDependencyNode fnTime(timeObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug plugOutTime = fnTime.findPlug(kAttrOutTime, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(plugOutTime, loaderInTime));

	// Persist the mainPath value if provided.
	if (mMainPath.length() > 0)
	{
		std::string pathStringStd = mMainPath.asChar();
		boost::replace_all(pathStringStd, "\\", "/");
		MPlug mainPathPlug = fnPclLoader.findPlug(kAttrMainPath, false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(mainPathPlug.setString(pathStringStd.c_str()));
	}

	// Note: Connections to pclCluster, pclSequencer, and pclRaycaster are now handled
	// by their respective command files (pclClusteringCmd, pclSequencerCmd, pclRaycasterCmd)
	// when called with the -pclLoader flag. We only need to handle the reverse connection
	// from raycaster back to loader for visualization.
	if (pclRaycaster_flag)
	{
		MFnDependencyNode fnPclRaycaster(mPclRaycaster_obj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Connect raycaster output back to loader (for visualization).
		MPlug raycasterOutHit = fnPclRaycaster.findPlug(kAttrOutPositionRaycast, false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MPlug loaderInRayPoints = fnPclLoader.findPlug(kAttrInRaycasterPoints, false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(raycasterOutHit, loaderInRayPoints));
	}

	CHECK_MSTATUS_AND_RETURN_IT(dagMod.doIt());

	// Lock containerOnOff on the loader to prevent user edits.
	MPlug loaderContainerOnOffLock = fnPclLoader.findPlug(kAttrContainerOnOff, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(loaderContainerOnOffLock.setLocked(true));

	return MS::kSuccess;
}

// Validate that the provided DAG path refers to a camera shape.
MStatus PclLoaderCmd::getShapeNode(MDagPath& path)
{
	MStatus status;

	if (path.apiType() == MFn::kCamera)
	{
		return MS::kSuccess;
	}

	unsigned int numShapes;
	status = path.numberOfShapesDirectlyBelow(numShapes);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	for (unsigned int i = 0; i < numShapes; ++i)
	{
		status = path.extendToShapeDirectlyBelow(i);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		if (!path.hasFn(MFn::kCamera))
		{
			path.pop();
			continue;
		}

		MFnDagNode fnNode(path, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		if (!fnNode.isIntermediateObject())
		{
			return MS::kSuccess;
		}
		path.pop();
	}

	return MS::kFailure;
}


// Create basic linear keyframes on loader.timeOffset for playback controls.
MStatus PclLoaderCmd::setKeyframesOnTimeAttr()
{
	MStatus status;

	// Fetch the timeOffset plug we animate.
	MFnDagNode pclLoaderDag(mPclLoader_obj);
	MPlug timeOffsetPlug = pclLoaderDag.findPlug("timeOffset", false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MTime startTime;
	startTime.setValue(1);
	MTime endTime;
	endTime.setValue(10);

	MFnAnimCurve animCurve;
	animCurve.create(timeOffsetPlug, MFnAnimCurve::kAnimCurveTU, nullptr, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	const MFnAnimCurve::TangentType tangent = MFnAnimCurve::kTangentLinear;
	CHECK_MSTATUS_AND_RETURN_IT(animCurve.addKeyframe(startTime, 1.0, tangent, tangent));
	CHECK_MSTATUS_AND_RETURN_IT(animCurve.addKeyframe(endTime, 10.0, tangent, tangent));

	return MS::kSuccess;
}

