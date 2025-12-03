#include "pclSequencerCmd.h"
#include "../pclLoader/pclCommandHelpers.h"

// Maya API
#include <maya/MArgDatabase.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MDagModifier.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
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
	constexpr char kFlagCreatePointsShort[] = "-cP";
	constexpr char kFlagCreatePointsLong[] = "-createPoints";
	constexpr char kFlagPclLoaderShort[] = "-pl";
	constexpr char kFlagPclLoaderLong[] = "-pclLoader";
	constexpr char kFlagHelpShort[] = "-h";
	constexpr char kFlagHelpLong[] = "-help";
	
	// Node type names
	constexpr char kNodeTypePclSequencer[] = "pclSequencer";
	constexpr char kTimeNodeName[] = "time1";
	
	// Attribute names
	constexpr char kAttrInTime[] = "inTime";
	constexpr char kAttrOutTime[] = "outTime";
	constexpr char kAttrMapperMsg[] = "mapper_msg";
	constexpr char kAttrParticlesSysMsg[] = "particlesSys_msg";
	constexpr char kAttrEmittorSysMsg[] = "emittorSys_msg";
	constexpr char kAttrCount[] = "count";
	constexpr char kAttrParticleCount[] = "particleCount";
	constexpr char kAttrStartFrame[] = "startFrame";
	constexpr char kAttrOutColorPP[] = "outColorPP";
	constexpr char kAttrRgbPP[] = "rgbPP";
	constexpr char kAttrOutPosition[] = "outPosition";
	constexpr char kAttrRampPosition[] = "rampPosition";
	constexpr char kAttrCreatePoints[] = "createPoints";
	constexpr char kAttrNumberOfPoints[] = "numberOfPoints";
	
	/// Display help text for the command.
	void DisplayHelp()
	{
		MString help;
		help += "pclSequencerCmd - Create and configure a pclSequencer node with particle system.\n";
		help += "Flags:\n";
		help += "-name (-n) <string>           : Base name for created nodes.\n";
		help += "-createPoints (-cP) <bool>     : Create points flag (legacy, not used).\n";
		help += "-pclLoader (-pl) <string>     : Name of pclLoader node to connect to.\n";
		help += "-help (-h)                    : Display this help.\n";
		MGlobal::displayInfo(help);
	}
}

/// Constructor with default naming prefix.
PclSequencerCmd::PclSequencerCmd() : mName("pclSequencer")
{
}

/// Creator function for Maya plugin system.
void* PclSequencerCmd::creator()
{
	return new PclSequencerCmd;
}

/// Define command syntax and flags.
MSyntax PclSequencerCmd::newSyntax()
{
	MSyntax syntax;
	syntax.addFlag(kFlagNameShort, kFlagNameLong, MSyntax::kString);
	syntax.addFlag(kFlagCreatePointsShort, kFlagCreatePointsLong, MSyntax::kBoolean);
	syntax.addFlag(kFlagPclLoaderShort, kFlagPclLoaderLong, MSyntax::kString);
	syntax.addFlag(kFlagHelpShort, kFlagHelpLong);
	syntax.setObjectType(MSyntax::kSelectionList);
	syntax.useSelectionAsDefault(true);
	
	return syntax;
}

/// Main command execution entry point.
MStatus PclSequencerCmd::doIt(const MArgList& argList)
{
	CHECK_MSTATUS_AND_RETURN_IT(parseArguments(argList));
	
	if (mName.length() == 0)
	{
		DisplayHelp();
		return MS::kSuccess;
	}
	
	CHECK_MSTATUS_AND_RETURN_IT(createSequencerNodes());
	CHECK_MSTATUS_AND_RETURN_IT(connectSequencerNodes());
	CHECK_MSTATUS_AND_RETURN_IT(connectToPclLoader());
	renameSequencerNodes();
	
	MStringArray nodeList;
	collectCreatedNodeNames(nodeList);
	MPxCommand::setResult(nodeList);
	
	// Select the particle node (main output)
	MGlobal::select(mParticle_obj, MGlobal::kReplaceList);
	
	return MS::kSuccess;
}

/// Parse command flags and arguments.
MStatus PclSequencerCmd::parseArguments(const MArgList& argList)
{
	mName.clear();
	mNameMapper.clear();
	mNameEmitter.clear();
	mNameParticle.clear();
	mNameNucleus.clear();
	mPclLoaderName.clear();
	mPclLoader_obj = MObject::kNullObj;
	mCreatePoints = false;
	
	MStatus status;
	MArgDatabase argData(syntax(), argList, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	// Check for help flag
	if (argData.isFlagSet(kFlagHelpShort) || argData.isFlagSet(kFlagHelpLong))
	{
		mName.clear(); // Signal to show help
		return MS::kSuccess;
	}
	
	// Get base name for nodes
	if (argData.isFlagSet(kFlagNameShort) || argData.isFlagSet(kFlagNameLong))
	{
		const char* flag = argData.isFlagSet(kFlagNameShort) ? kFlagNameShort : kFlagNameLong;
		mName = argData.flagArgumentString(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		mNameMapper = mName + "Mapper#";
		mNameEmitter = mName + "Emitter#";
		mNameParticle = mName + "Particle#";
		mNameNucleus = mName + "Nucleus#";
	}
	else
	{
		mNameMapper = "pclSequencerMapper#";
		mNameEmitter = "pclSequencerEmitter#";
		mNameParticle = "pclSequencerParticle#";
		mNameNucleus = "pclSequencerNucleus#";
	}
	
	// Get createPoints flag (legacy, kept for compatibility)
	if (argData.isFlagSet(kFlagCreatePointsShort) || argData.isFlagSet(kFlagCreatePointsLong))
	{
		const char* flag = argData.isFlagSet(kFlagCreatePointsShort) ? kFlagCreatePointsShort : kFlagCreatePointsLong;
		mCreatePoints = argData.flagArgumentBool(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	
	// Get pclLoader node name if provided
	if (argData.isFlagSet(kFlagPclLoaderShort) || argData.isFlagSet(kFlagPclLoaderLong))
	{
		const char* flag = argData.isFlagSet(kFlagPclLoaderShort) ? kFlagPclLoaderShort : kFlagPclLoaderLong;
		mPclLoaderName = argData.flagArgumentString(flag, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	
	return MS::kSuccess;
}

/// Create all sequencer-related nodes (sequencer, emitter, particle, nucleus).
MStatus PclSequencerCmd::createSequencerNodes()
{
	// Create pclSequencer node (non-DAG dependency node)
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::createDependencyNode(kNodeTypePclSequencer, mSequencer_obj));
	
	// Create emitter node
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::createEmitterNode(mEmitter_obj));
	
	// Create nParticle node
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::createNParticleNode(mParticle_obj));
	
	// Get nucleus node from particle system
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::getNucleusFromParticle(mParticle_obj, mNucleus_obj));
	
	// Add custom attributes to particle and emitter
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::addParticleSystemAttributes(mParticle_obj));
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::addEmitterAttributes(mEmitter_obj));
	
	return MS::kSuccess;
}

/// Connect all sequencer-related nodes together.
MStatus PclSequencerCmd::connectSequencerNodes()
{
	MStatus status;
	MDagModifier dagMod;
	
	MFnDependencyNode fnSequencer(mSequencer_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDependencyNode fnParticle(mParticle_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDependencyNode fnEmitter(mEmitter_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	// Connect emitter to particle using connectDynamic
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::connectEmitterToParticle(mEmitter_obj, mParticle_obj));
	
	// Connect time1.outTime to sequencer.inTime
	MSelectionList timeList;
	CHECK_MSTATUS_AND_RETURN_IT(MGlobal::getSelectionListByName(kTimeNodeName, timeList));
	MObject timeObj;
	CHECK_MSTATUS_AND_RETURN_IT(timeList.getDependNode(0, timeObj));
	MFnDependencyNode fnTime(timeObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug plugOutTime = fnTime.findPlug(kAttrOutTime, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug sequencerInTime = fnSequencer.findPlug(kAttrInTime, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(plugOutTime, sequencerInTime));
	
	// Connect message attributes between sequencer and particle/emitter
	MPlug particleMapperMsg = fnParticle.findPlug(kAttrMapperMsg, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug sequencerParticleMsg = fnSequencer.findPlug(kAttrParticlesSysMsg, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(particleMapperMsg, sequencerParticleMsg));
	
	MPlug emitterMapperMsg = fnEmitter.findPlug(kAttrMapperMsg, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug sequencerEmitterMsg = fnSequencer.findPlug(kAttrEmittorSysMsg, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(emitterMapperMsg, sequencerEmitterMsg));
	
	// Connect particle attributes to sequencer
	MPlug particleCount = fnParticle.findPlug(kAttrCount, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug sequencerParticleCount = fnSequencer.findPlug(kAttrParticleCount, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(particleCount, sequencerParticleCount));
	
	MPlug particleStartFrame = fnParticle.findPlug(kAttrStartFrame, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug sequencerStartFrame = fnSequencer.findPlug(kAttrStartFrame, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(particleStartFrame, sequencerStartFrame));
	
	// Connect sequencer outputs to particle
	MPlug sequencerOutColorPP = fnSequencer.findPlug(kAttrOutColorPP, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug particleRgbPP = fnParticle.findPlug(kAttrRgbPP, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(sequencerOutColorPP, particleRgbPP));
	
	MPlug sequencerOutPosition = fnSequencer.findPlug(kAttrOutPosition, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug particleRampPosition = fnParticle.findPlug(kAttrRampPosition, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(sequencerOutPosition, particleRampPosition));
	
	// Connect control attributes between particle and sequencer
	MPlug particleCreatePoints = fnParticle.findPlug(kAttrCreatePoints, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug sequencerCreatePoints = fnSequencer.findPlug(kAttrCreatePoints, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(particleCreatePoints, sequencerCreatePoints));
	
	MPlug sequencerNumberOfPoints = fnSequencer.findPlug(kAttrNumberOfPoints, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug particleNumberOfPoints = fnParticle.findPlug(kAttrNumberOfPoints, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.connect(sequencerNumberOfPoints, particleNumberOfPoints));
	
	// Execute all connections
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.doIt());
	
	// Set default values on particle, nucleus, and emitter
	CHECK_MSTATUS_AND_RETURN_IT(PclCommandHelpers::setSequencerNodeDefaults(mParticle_obj, mNucleus_obj, mEmitter_obj));
	
	return MS::kSuccess;
}

/// Connect sequencer to pclLoader node if provided.
MStatus PclSequencerCmd::connectToPclLoader()
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
	MFnDependencyNode fnSequencer(mSequencer_obj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	MDagModifier dagMod;
	
	// Connect point positions (VectorArray) from loader to sequencer.
	MPlug loaderOutPointsVec = fnPclLoader.findPlug("outPointsPosition_vec", false, &status);
	if (status == MS::kSuccess)
	{
		MPlug sequencerInPoints = fnSequencer.findPlug("inPointsPosition", false, &status);
		if (status == MS::kSuccess)
		{
			dagMod.connect(loaderOutPointsVec, sequencerInPoints);
		}
	}
	
	// Connect point colors (VectorArray) from loader to sequencer.
	MPlug loaderOutColorVec = fnPclLoader.findPlug("outPointsColor_vec", false, &status);
	if (status == MS::kSuccess)
	{
		MPlug sequencerInColor = fnSequencer.findPlug("inPointsColor", false, &status);
		if (status == MS::kSuccess)
		{
			dagMod.connect(loaderOutColorVec, sequencerInColor);
		}
	}
	
	CHECK_MSTATUS_AND_RETURN_IT(dagMod.doIt());
	
	return MS::kSuccess;
}

/// Rename all created nodes with the specified base name.
void PclSequencerCmd::renameSequencerNodes()
{
	// Rename sequencer node (non-DAG, so use MFnDependencyNode)
	if (!mSequencer_obj.isNull() && mNameMapper.length() > 0)
	{
		MFnDependencyNode fnSequencer(mSequencer_obj);
		fnSequencer.setName(mNameMapper);
	}
	
	// Rename emitter, particle, and nucleus
	if (!mEmitter_obj.isNull() && mNameEmitter.length() > 0)
	{
		PclCommandHelpers::renameParentTransform(mEmitter_obj, mNameEmitter);
	}
	if (!mParticle_obj.isNull() && mNameParticle.length() > 0)
	{
		PclCommandHelpers::renameParentTransform(mParticle_obj, mNameParticle);
	}
	if (!mNucleus_obj.isNull() && mNameNucleus.length() > 0)
	{
		MFnDependencyNode fnNucleus(mNucleus_obj);
		fnNucleus.setName(mNameNucleus);
	}
}

/// Collect names of all created nodes for command result.
void PclSequencerCmd::collectCreatedNodeNames(MStringArray& outNames) const
{
	// Return: particle, emitter, nucleus, mapper (in that order, matching old behavior)
	PclCommandHelpers::appendNodeNameIfValid(mParticle_obj, outNames);
	PclCommandHelpers::appendNodeNameIfValid(mEmitter_obj, outNames);
	PclCommandHelpers::appendNodeNameIfValid(mNucleus_obj, outNames);
	if (!mSequencer_obj.isNull())
	{
		MFnDependencyNode fnSequencer(mSequencer_obj);
		outNames.append(fnSequencer.name());
	}
}

