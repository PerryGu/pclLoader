#include "pclRaycasterCmd.h"
#include "../pclLoader/pclCommandHelpers.h"

// Maya API
#include <maya/MArgDatabase.h>
#include <maya/MDagModifier.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>

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
	/// Retrieves the render resolution from Maya's defaultResolution node using the C++ API.
	/// @param[out] width The render width in pixels.
	/// @param[out] height The render height in pixels.
	/// @return MStatus indicating success or failure.
	MStatus getRenderResolution(double& width, double& height)
	{
		MStatus status;
		MSelectionList selection;
		status = selection.add("defaultResolution");
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		MObject resolutionNode;
		status = selection.getDependNode(0, resolutionNode);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		MFnDependencyNode fnResolution(resolutionNode, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		MPlug widthPlug = fnResolution.findPlug("width", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = widthPlug.getValue(width);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		MPlug heightPlug = fnResolution.findPlug("height", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = heightPlug.getValue(height);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		return MS::kSuccess;
	}
}

void DisplayHelp() {
	MString help;
	help += "Flags:\n";
	help += "-name (-n) <string>          : Name of the raycaster node to create.\n";
	help += "-selCamera (-sc) <string>    : Camera to connect.\n";
	help += "-pclLoader (-pl) <string>    : Name of pclLoader node to connect to.\n";
	help += "-addHitTargetLoc (-aL) <bool>: Create and connect locator to show the hit point.\n";
	help += "-help (-h)                   : Display this text.\n";
	MGlobal::displayInfo(help);
}


pclRaycasterCmd::pclRaycasterCmd() :name_("PclRaycasterCmd#"), m_pclLoader_obj(MObject::kNullObj)
{

}


MSyntax pclRaycasterCmd::newSyntax() {
	MSyntax syntax;
	syntax.addFlag("-n", "-name", MSyntax::kString);
	syntax.addFlag("-sc", "-selCamera", MSyntax::kString);
	syntax.addFlag("-pl", "-pclLoader", MSyntax::kString);
	syntax.addFlag("-aL", "-addHitTargetLoc", MSyntax::kBoolean);
	syntax.setObjectType(MSyntax::kSelectionList);
	syntax.useSelectionAsDefault(true);

	return syntax;

}


void* pclRaycasterCmd::creator()
{
	return new pclRaycasterCmd;
}


MStatus pclRaycasterCmd::doIt(const MArgList& argList) {

	MStatus status;
	MGlobal::displayInfo(" pclRaycasterCmd doIt!!");

	// Read all the flag arguments
	MArgDatabase argData(syntax(), argList, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);


	//-- get the name of the pluge -----------------------------------
	if (argData.isFlagSet("-n"))
	{
		m_name = argData.flagArgumentString("-n", 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		m_namePclSequencer = m_name + "pclRaycaster#";
	}
	else
	{
		m_namePclSequencer = "pclRaycaster#";
	}


	// Get the selected camera name
	if (argData.isFlagSet("-sc") || argData.isFlagSet("-selCamera"))
	{
		const char* flag = argData.isFlagSet("-sc") ? "-sc" : "-selCamera";
		m_selCamera = argData.flagArgumentString(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	// Get pclLoader node name if provided
	if (argData.isFlagSet("-pl") || argData.isFlagSet("-pclLoader"))
	{
		const char* flag = argData.isFlagSet("-pl") ? "-pl" : "-pclLoader";
		m_pclLoaderName = argData.flagArgumentString(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	// Get the locator flag
	if (argData.isFlagSet("-aL") || argData.isFlagSet("-addHitTargetLoc"))
	{
		const char* flag = argData.isFlagSet("-aL") ? "-aL" : "-addHitTargetLoc";
		m_loc = argData.flagArgumentBool(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}


	MSelectionList cameraSel;
	//-- if camera name in the args -------------------------
	if (m_selCamera != "")
	{
		MGlobal::getSelectionListByName(m_selCamera, cameraSel);
	}

	//-- if camera is aelected -------------------------
	else
	{
		status = argData.getObjects(cameraSel);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	
	//-- get camera name pluge -----------------------------------	
	MDagPath m_pathCam;
	if (cameraSel.length() == 1)
	{
		status = cameraSel.getDagPath(0, m_pathCam);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = getShapeNode(m_pathCam);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		//-- Get camera transform Name -------------------------------
		MString camFullPathName = m_pathCam.fullPathName();
		MStringArray stringToSplit;
		camFullPathName.split('|', stringToSplit);
		int stringNumber = stringToSplit.length();
		m_cameraName = stringToSplit[stringNumber - 2];
	}

	// Create the pclRaycaster node using C++ API
	MObject pclRaycaster_obj;
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::createDagNode("pclRaycaster", pclRaycaster_obj));
	MFnDagNode dagNodeFn_par(pclRaycaster_obj);

	// Get camera node if camera name is provided
	MObject camera_obj = MObject::kNullObj;
	MSelectionList node_list;
	if (m_cameraName != "")
	{
		status = node_list.add(m_cameraName);
		if (status == MS::kSuccess)
		{
			status = node_list.getDependNode(0, camera_obj);
			if (status != MS::kSuccess)
			{
				MGlobal::displayWarning("Failed to get camera node: " + m_cameraName);
			}
		}
	}
	else
	{
		MGlobal::displayWarning("NO CAMERA TO CONNECT!");
	}


	// Create locator if requested
	MObject loc_obj = MObject::kNullObj;
	if (m_loc == true)
	{
		CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::createDagNode("locator", loc_obj));
	}


	// Connect all attributes between camera, raycaster, and locator
	CHECK_MSTATUS_AND_RETURN_IT(connectAll(pclRaycaster_obj, camera_obj, loc_obj));
	
	// Connect to pclLoader if provided
	CHECK_MSTATUS_AND_RETURN_IT(connectToPclLoader(pclRaycaster_obj));

	// Set createPoints attribute if requested
	MFnDependencyNode fnPclRaycaster(pclRaycaster_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	if (m_createPoints == true)
	{
		MPlug createPointsPlug = fnPclRaycaster.findPlug("createPoints", false, &status);
		if (status == MS::kSuccess)
		{
			createPointsPlug.setValue(1);
		}
	}

	// Get render resolution from defaultResolution node using C++ API
	double resolutionX = 1920.0;
	double resolutionY = 1080.0;
	if (getRenderResolution(resolutionX, resolutionY) != MS::kSuccess)
	{
		MGlobal::displayWarning("Failed to get render resolution, using default 1920x1080");
	}

	// Set raycast input to center of frame (default) using C++ API
	MPlug raycastInputXPlug = fnPclRaycaster.findPlug("raycastInputX", false, &status);
	if (status == MS::kSuccess)
	{
		raycastInputXPlug.setValue(resolutionX / 2.0);
	}
	
	MPlug raycastInputYPlug = fnPclRaycaster.findPlug("raycastInputY", false, &status);
	if (status == MS::kSuccess)
	{
		raycastInputYPlug.setValue(resolutionY / 2.0);
	}

	// Return names of the created nodes.
	MStringArray nodeList;
	PclCommandHelpers::appendNodeNameIfValid(pclRaycaster_obj, nodeList);
	MPxCommand::setResult(nodeList);

	//-- select the main node -----------------------------------------------
	MGlobal::selectByName(dagNodeFn_par.fullPathName(), MGlobal::kReplaceList);



	return MS::kSuccess;
}



/// Connect all required attributes between camera, raycaster, and locator nodes.
MStatus pclRaycasterCmd::connectAll(MObject pclRaycaster_obj, MObject camera_obj, MObject loc_obj)
{
	MStatus status;
	MDagModifier dagMod;
	
	MFnDependencyNode fnPclRaycaster(pclRaycaster_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	// Connect camera attributes to raycaster node if camera is provided
	if (m_cameraName != "" && !camera_obj.isNull())
	{
		MFnDependencyNode fnCamera(camera_obj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		// Connect camera worldMatrix to raycaster cameraMatrix
		MPlug cameraWorldMatrix = fnCamera.findPlug("worldMatrix", true, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		cameraWorldMatrix = cameraWorldMatrix.elementByLogicalIndex(0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MPlug raycasterCameraMatrix = fnPclRaycaster.findPlug("cameraMatrix", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(cameraWorldMatrix, raycasterCameraMatrix));
		
		// Connect camera focalLength to raycaster cameraFocalLength
		MPlug cameraFocalLength = fnCamera.findPlug("focalLength", true, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MPlug raycasterFocalLength = fnPclRaycaster.findPlug("cametaFocalLength", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(cameraFocalLength, raycasterFocalLength));
		
		// Connect camera nearClipPlane to raycaster cameraNearClipPlane
		MPlug cameraNearClip = fnCamera.findPlug("nearClipPlane", true, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MPlug raycasterNearClip = fnPclRaycaster.findPlug("cameraNearClipPlane", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(cameraNearClip, raycasterNearClip));
		
		// Connect camera farClipPlane to raycaster cameraFarClipPlane
		MPlug cameraFarClip = fnCamera.findPlug("farClipPlane", true, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MPlug raycasterFarClip = fnPclRaycaster.findPlug("cameraFarClipPlane", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(cameraFarClip, raycasterFarClip));
		
		// Connect camera horizontalFilmAperture to raycaster cameraHorizontalFilmAperture
		MPlug cameraHorizAperture = fnCamera.findPlug("horizontalFilmAperture", true, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MPlug raycasterHorizAperture = fnPclRaycaster.findPlug("cameraHorizontalFilmAperture", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(cameraHorizAperture, raycasterHorizAperture));
		
		// Connect camera verticalFilmAperture to raycaster cameraVerticalFilmAperture
		MPlug cameraVertAperture = fnCamera.findPlug("verticalFilmAperture", true, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MPlug raycasterVertAperture = fnPclRaycaster.findPlug("cameraVerticalFilmAperture", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(cameraVertAperture, raycasterVertAperture));
	}
	
	// Connect raycaster closestPointHit to locator translate if locator is provided
	if (m_loc == true && !loc_obj.isNull())
	{
		MFnDagNode locDag(loc_obj);
		MFnDependencyNode fnLocator(locDag.transform(), &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		MPlug raycasterClosestPoint = fnPclRaycaster.findPlug("closestPointHit", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MPlug locatorTranslate = fnLocator.findPlug("translate", true, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(raycasterClosestPoint, locatorTranslate));
	}
	
	// Execute all connections
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.doIt());
	
	return MS::kSuccess;
}

/// Connect raycaster to pclLoader node if provided.
MStatus pclRaycasterCmd::connectToPclLoader(MObject pclRaycaster_obj)
{
	if (m_pclLoaderName.length() == 0)
	{
		return MS::kSuccess; // No loader specified, skip connection
	}
	
	MStatus status;
	MSelectionList loaderList;
	CHECK_MSTATUS_AND_RETURN_IT(MGlobal::getSelectionListByName(m_pclLoaderName, loaderList));
	
	if (loaderList.length() == 0)
	{
		MGlobal::displayWarning("pclLoader node not found: " + m_pclLoaderName);
		return MS::kSuccess; // Non-fatal, continue without connection
	}
	
	CHECK_MSTATUS_AND_RETURN_IT(loaderList.getDependNode(0, m_pclLoader_obj));
	
	MFnDependencyNode fnPclLoader(m_pclLoader_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDependencyNode fnPclRaycaster(pclRaycaster_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	MDagModifier dagMod;
	
	// Connect point positions from loader to raycaster.
	MPlug loaderOutPoints = fnPclLoader.findPlug("outPointsPosition", false, &status);
	if (status == MS::kSuccess)
	{
		MPlug raycasterInPoints = fnPclRaycaster.findPlug("inPointsPosition", false, &status);
		if (status == MS::kSuccess)
		{
			dagMod.connect(loaderOutPoints, raycasterInPoints);
		}
	}
	
	// Connect point colors from loader to raycaster.
	MPlug loaderOutPointsColor = fnPclLoader.findPlug("outPointsColor", false, &status);
	if (status == MS::kSuccess)
	{
		MPlug raycasterInPointsColor = fnPclRaycaster.findPlug("inPointsColor", false, &status);
		if (status == MS::kSuccess)
		{
			dagMod.connect(loaderOutPointsColor, raycasterInPointsColor);
		}
	}
	
	// Connect time from time1 to raycaster.
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
					MPlug raycasterInTime = fnPclRaycaster.findPlug("inTime", false, &status);
					if (status == MS::kSuccess)
					{
						dagMod.connect(plugOutTime, raycasterInTime);
					}
				}
			}
		}
	}
	
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.doIt());
	
	return MS::kSuccess;
}

//-- get the shape of camera tansform  to check if it is a camera node type --------------------------------
MStatus pclRaycasterCmd::getShapeNode(MDagPath& path)
{
	MStatus status;
	//MGlobal::displayInfo(MString(" getShapeNode!! ") + path.fullPathName());

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


MStatus pclRaycasterCmd::redoIt() {
	MStatus status;

	//MGlobal::displayInfo(" ScatteringNodeCmd redoIt!!");
	status = dgMod_.doIt();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDependencyNode fnNode(oWrapNode_, &status);
	setResult(fnNode.name());
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;

}

MStatus pclRaycasterCmd::undoIt() {
	MStatus status;
	status = dgMod_.undoIt();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}

bool pclRaycasterCmd::isUndoable() const {
	return true;
	//return command_ == kCommandCreate;  // Only creation will be undoable
}



