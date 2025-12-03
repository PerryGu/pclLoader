#include "pclClusteringCmd.h"
#include "../pclLoader/pclCommandHelpers.h"

// Maya API
#include <maya/MArgDatabase.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>
#include <maya/MStringArray.h>

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
	// Command flag names
	constexpr char kFlagNameShort[] = "-n";
	constexpr char kFlagNameLong[] = "-name";
	constexpr char kFlagPclLoaderShort[] = "-pl";
	constexpr char kFlagPclLoaderLong[] = "-pclLoader";
	constexpr char kFlagHelpShort[] = "-h";
	constexpr char kFlagHelpLong[] = "-help";
	
	// Node type names
	constexpr char kNodeTypePclCluster[] = "pclCluster";
	
	/// Display help text for the command.
	void DisplayHelp()
	{
		MString help;
		help += "pclClusteringCmd - Create and configure a pclCluster node.\n";
		help += "Flags:\n";
		help += "-name (-n) <string>           : Base name for created node.\n";
		help += "-pclLoader (-pl) <string>     : Name of pclLoader node to connect to.\n";
		help += "-help (-h)                    : Display this help.\n";
		MGlobal::displayInfo(help);
	}
}

/// Constructor with default naming prefix.
PclClusteringCmd::PclClusteringCmd() : mName("pclCluster")
{
}

/// Creator function for Maya plugin system.
void* PclClusteringCmd::creator()
{
	return new PclClusteringCmd;
}

/// Define command syntax and flags.
MSyntax PclClusteringCmd::newSyntax()
{
	MSyntax syntax;
	syntax.addFlag(kFlagNameShort, kFlagNameLong, MSyntax::kString);
	syntax.addFlag(kFlagPclLoaderShort, kFlagPclLoaderLong, MSyntax::kString);
	syntax.addFlag(kFlagHelpShort, kFlagHelpLong);
	syntax.setObjectType(MSyntax::kSelectionList);
	syntax.useSelectionAsDefault(true);
	
	return syntax;
}

/// Main command execution entry point.
MStatus PclClusteringCmd::doIt(const MArgList& argList)
{
	CHECK_MSTATUS_AND_RETURN_IT(parseArguments(argList));
	
	if (mName.length() == 0)
	{
		DisplayHelp();
		return MS::kSuccess;
	}
	
	CHECK_MSTATUS_AND_RETURN_IT(createClusterNode());
	CHECK_MSTATUS_AND_RETURN_IT(connectToPclLoader());
	renameClusterNode();
	
	MStringArray nodeNames;
	collectCreatedNodeName(nodeNames);
	MPxCommand::setResult(nodeNames);
	
	// Select the created node
	if (!mCluster_obj.isNull())
	{
		MFnDagNode fnCluster(mCluster_obj);
		MGlobal::selectByName(fnCluster.fullPathName(), MGlobal::kReplaceList);
	}
	
	return MS::kSuccess;
}

/// Parse command arguments and flags.
MStatus PclClusteringCmd::parseArguments(const MArgList& argList)
{
	MStatus status;
	MArgDatabase argData(syntax(), argList, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	// Check for help flag
	if (argData.isFlagSet(kFlagHelpShort) || argData.isFlagSet(kFlagHelpLong))
	{
		mName.clear();
		return MS::kSuccess;
	}
	
	// Get name flag if provided
	if (argData.isFlagSet(kFlagNameShort) || argData.isFlagSet(kFlagNameLong))
	{
		const char* flag = argData.isFlagSet(kFlagNameShort) ? kFlagNameShort : kFlagNameLong;
		mName = argData.flagArgumentString(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		mName += "pclCluster#";
	}
	else
	{
		mName = "pclCluster#";
	}
	
	// Get pclLoader node name if provided
	mPclLoaderName.clear();
	mPclLoader_obj = MObject::kNullObj;
	if (argData.isFlagSet(kFlagPclLoaderShort) || argData.isFlagSet(kFlagPclLoaderLong))
	{
		const char* flag = argData.isFlagSet(kFlagPclLoaderShort) ? kFlagPclLoaderShort : kFlagPclLoaderLong;
		mPclLoaderName = argData.flagArgumentString(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	
	return MS::kSuccess;
}

/// Create the pclCluster node.
MStatus PclClusteringCmd::createClusterNode()
{
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::createDagNode(kNodeTypePclCluster, mCluster_obj));
	return MS::kSuccess;
}

/// Rename the cluster node if a name was provided.
void PclClusteringCmd::renameClusterNode()
{
	if (mName.length() > 0 && !mCluster_obj.isNull())
	{
		PclCommandHelpers::renameParentTransform(mCluster_obj, mName);
	}
}

/// Connect cluster to pclLoader node if provided.
MStatus PclClusteringCmd::connectToPclLoader()
{
	if (mPclLoaderName.length() == 0)
	{
		return MS::kSuccess; // No loader specified, skip connection
	}
	
	MStatus status;
	MSelectionList loaderList;
	CHECK_MSTATUS_AND_RETURN_IT(MGlobal::getSelectionListByName(mPclLoaderName, loaderList));
	
	if (loaderList.length() == 0)
	{
		MGlobal::displayWarning("pclLoader node not found: " + mPclLoaderName);
		return MS::kSuccess; // Non-fatal, continue without connection
	}
	
	CHECK_MSTATUS_AND_RETURN_IT(loaderList.getDependNode(0, mPclLoader_obj));
	
	MFnDependencyNode fnPclLoader(mPclLoader_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDependencyNode fnCluster(mCluster_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	MDagModifier dagMod;
	
	// Connect time from time1 to cluster.
	MSelectionList timeList;
	if (MGlobal::getSelectionListByName("time1", timeList) == MS::kSuccess && timeList.length() > 0)
	{
		MObject timeObj;
		if (timeList.getDependNode(0, timeObj) == MS::kSuccess)
		{
			MFnDependencyNode fnTime(timeObj, &status);
			if (status == MS::kSuccess)
			{
				MPlug plugOutTime = fnTime.findPlug("outTime", false, &status);
				if (status == MS::kSuccess)
				{
					MPlug clusterInTime = fnCluster.findPlug("inTime", false, &status);
					if (status == MS::kSuccess)
					{
						dagMod.connect(plugOutTime, clusterInTime);
					}
				}
			}
		}
	}
	
	// Connect point positions from loader to cluster.
	MPlug loaderOutPoints = fnPclLoader.findPlug("outPointsPosition", false, &status);
	if (status == MS::kSuccess)
	{
		MPlug clusterInPoints = fnCluster.findPlug("inPointsPosition", false, &status);
		if (status == MS::kSuccess)
		{
			dagMod.connect(loaderOutPoints, clusterInPoints);
		}
	}
	
	// Connect point colors from loader to cluster.
	MPlug loaderOutPointsColor = fnPclLoader.findPlug("outPointsColor", false, &status);
	if (status == MS::kSuccess)
	{
		MPlug clusterInPointsColor = fnCluster.findPlug("inPointsColor", false, &status);
		if (status == MS::kSuccess)
		{
			dagMod.connect(loaderOutPointsColor, clusterInPointsColor);
		}
	}
	
	// Connect bounding box corners from loader to cluster.
	MPlug loaderOutBBox = fnPclLoader.findPlug("outBoundingBoxCorner", false, &status);
	if (status == MS::kSuccess)
	{
		MPlug clusterInBBox = fnCluster.findPlug("inBoundingBoxCorner", false, &status);
		if (status == MS::kSuccess)
		{
			dagMod.connect(loaderOutBBox, clusterInBBox);
		}
	}
	
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.doIt());
	
	return MS::kSuccess;
}

/// Collect created node names for command result.
void PclClusteringCmd::collectCreatedNodeName(MStringArray& outNames) const
{
	if (!mCluster_obj.isNull())
	{
		MFnDagNode fnCluster(mCluster_obj);
		PclCommandHelpers::appendNodeNameIfValid(mCluster_obj, outNames);
	}
}

