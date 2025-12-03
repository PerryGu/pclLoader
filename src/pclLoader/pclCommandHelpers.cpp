#include "pclCommandHelpers.h"

// Maya API
#include <maya/MDagModifier.h>
#include <maya/MDGModifier.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
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

namespace PclCommandHelpers
{
	// ============================================================================
	// Common Node Creation Utilities
	// ============================================================================
	
	/// Creates a DAG node with an MDagModifier (undoable).
	MStatus createDagNode(const MString& typeName, MObject& outObject)
	{
		MStatus status;
		MDagModifier dagMod;
		outObject = dagMod.createNode(typeName, MObject::kNullObj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		return dagMod.doIt();
	}
	
	/// Creates a dependency node (non-DAG) with an MDGModifier.
	MStatus createDependencyNode(const MString& typeName, MObject& outObject)
	{
		MStatus status;
		MDGModifier dgMod;
		outObject = dgMod.createNode(typeName, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		return dgMod.doIt();
	}
	
	/// Renames the parent transform (if it exists) for the provided DAG node.
	MStatus renameParentTransform(const MObject& dagNodeObj, const MString& newName)
	{
		if (dagNodeObj.isNull() || newName.length() == 0)
		{
			return MS::kSuccess;
		}
		
		MFnDagNode dagNode(dagNodeObj);
		if (dagNode.parentCount() == 0)
		{
			return dagNode.setName(newName);
		}
		
		MObject parent = dagNode.parent(0);
		dagNode.setObject(parent);
		return dagNode.setName(newName);
	}
	
	/// Appends a DAG node's name to the provided array when it is valid.
	void appendNodeNameIfValid(const MObject& obj, MStringArray& list)
	{
		if (obj.isNull())
		{
			return;
		}
		
		MFnDagNode dagNode(obj);
		list.append(dagNode.partialPathName());
	}
	
	// ============================================================================
	// Sequencer-Specific Helpers
	// ============================================================================
	
	// Sequencer-related node type names.
	namespace
	{
		constexpr char kNodeTypeEmitter[] = "emitter";
		constexpr char kNodeTypeNParticle[] = "nParticle";
	}
	
	/// Creates an emitter node and returns its shape object.
	MStatus createEmitterNode(MObject& outEmitterObj)
	{
		MStatus status;
		MDagModifier dagMod;
		
		// Create emitter node - this creates both transform and shape
		outEmitterObj = dagMod.createNode(kNodeTypeEmitter, MObject::kNullObj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.doIt());
		
		return MS::kSuccess;
	}
	
	/// Creates an nParticle node and returns its shape object.
	MStatus createNParticleNode(MObject& outParticleObj)
	{
		MStatus status;
		MDagModifier dagMod;
		
		// Create nParticle node - this creates both transform and shape
		outParticleObj = dagMod.createNode(kNodeTypeNParticle, MObject::kNullObj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		CHECK_MSTATUS_AND_RETURN_IT(dagMod.doIt());
		
		return MS::kSuccess;
	}
	
	/// Gets the nucleus node connected to a particle system via startState.
	MStatus getNucleusFromParticle(const MObject& particleObj, MObject& outNucleusObj)
	{
		MStatus status;
		MFnDependencyNode fnParticle(particleObj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		MPlug startStatePlug = fnParticle.findPlug("startState", true, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		MPlugArray connectedPlugs;
		startStatePlug.connectedTo(connectedPlugs, false, true, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		if (connectedPlugs.length() == 0)
		{
			return MS::kNotFound;
		}
		
		outNucleusObj = connectedPlugs[0].node();
		return MS::kSuccess;
	}
	
	/// Connects an emitter to a particle system using connectDynamic.
	/// Note: connectDynamic doesn't have a direct C++ API, so we use MEL.
	MStatus connectEmitterToParticle(const MObject& emitterObj, const MObject& particleObj)
	{
		MStatus status;
		MFnDagNode emitterDag(emitterObj);
		MFnDagNode particleDag(particleObj);
		
		MString command = "connectDynamic - em \"" + emitterDag.fullPathName() + "\" \"" + particleDag.fullPathName() + "\"";
		CHECK_MSTATUS_AND_RETURN_IT(MGlobal::executeCommand(command, false, true));
		
		return MS::kSuccess;
	}
	
	/// Adds custom attributes to a particle system node.
	MStatus addParticleSystemAttributes(const MObject& particleObj)
	{
		MStatus status;
		MFnDependencyNode fnParticle(particleObj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		MFnDagNode particleDag(particleObj);
		MFnEnumAttribute eAttr;
		MFnNumericAttribute numAttr;
		MFnTypedAttribute typedAttr;
		MFnMessageAttribute msgAttr;
		
		// Add rgbPP attribute using MEL (vectorArray type requires MEL)
		MString command = "addAttr -ln \"rgbPP\" -dt vectorArray \"" + particleDag.fullPathName() + "\"";
		CHECK_MSTATUS_AND_RETURN_IT(MGlobal::executeCommand(command, false, true));
		command = "addAttr -ln \"rgbPP0\" -dt vectorArray \"" + particleDag.fullPathName() + "\"";
		CHECK_MSTATUS_AND_RETURN_IT(MGlobal::executeCommand(command, false, true));
		
		// Add pointSize attribute using MEL
		command = "addAttr -is true -ln \"pointSize\" -at long -min 1 -max 60 -dv 6 \"" + particleDag.fullPathName() + "\"";
		CHECK_MSTATUS_AND_RETURN_IT(MGlobal::executeCommand(command, false, true));
		
		// Create MapperAttributs enum attribute
		MObject mapperAttributs = eAttr.create("MapperAttributs", "MapperAttributs", &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		eAttr.addField("mapperAttributs", 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		eAttr.setKeyable(true);
		eAttr.setStorable(false);
		eAttr.setWritable(true);
		eAttr.setChannelBox(true);
		eAttr.setConnectable(true);
		CHECK_MSTATUS_AND_RETURN_IT(fnParticle.addAttribute(mapperAttributs));
		
		// Create createPoints enum attribute
		MObject createPointsAttr = eAttr.create("createPoints", "createPoints", &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		eAttr.addField("clear points", 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		eAttr.addField("create points", 1, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		eAttr.setDefault(0);
		eAttr.setReadable(true);
		eAttr.setWritable(true);
		eAttr.setStorable(true);
		eAttr.setKeyable(true);
		eAttr.setConnectable(true);
		CHECK_MSTATUS_AND_RETURN_IT(fnParticle.addAttribute(createPointsAttr));
		
		// Create numberOfPoints numeric attribute
		MObject numberOfPointsAttr = numAttr.create("numberOfPoints", "numberOfPoints", MFnNumericData::kInt, 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		typedAttr.setStorable(true);
		typedAttr.setWritable(true);
		typedAttr.setKeyable(true);
		typedAttr.setReadable(true);
		typedAttr.setConnectable(true);
		CHECK_MSTATUS_AND_RETURN_IT(fnParticle.addAttribute(numberOfPointsAttr));
		
		// Add mapper_msg message attribute
		MObject mapperMsgAttr = msgAttr.create("mapper_msg", "mapper_msg", &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		msgAttr.setReadable(true);
		msgAttr.setStorable(true);
		msgAttr.setConnectable(true);
		msgAttr.setHidden(true);
		CHECK_MSTATUS_AND_RETURN_IT(fnParticle.addAttribute(mapperMsgAttr));
		
		return MS::kSuccess;
	}
	
	/// Adds custom attributes to an emitter node.
	MStatus addEmitterAttributes(const MObject& emitterObj)
	{
		MStatus status;
		MFnDependencyNode fnEmitter(emitterObj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		MFnMessageAttribute msgAttr;
		
		// Add mapper_msg message attribute
		MObject mapperMsgAttr = msgAttr.create("mapper_msg", "mapper_msg", &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		msgAttr.setReadable(true);
		msgAttr.setStorable(true);
		msgAttr.setConnectable(true);
		CHECK_MSTATUS_AND_RETURN_IT(fnEmitter.addAttribute(mapperMsgAttr));
		
		return MS::kSuccess;
	}
	
	/// Sets default attribute values on particle, nucleus, and emitter nodes.
	MStatus setSequencerNodeDefaults(const MObject& particleObj, const MObject& nucleusObj, const MObject& emitterObj)
	{
		MStatus status;
		MFnDependencyNode fnParticle(particleObj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MFnDependencyNode fnNucleus(nucleusObj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MFnDependencyNode fnEmitter(emitterObj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		// Set particle attributes using C++ API
		MPlug particleLifespanMode = fnParticle.findPlug("lifespanMode", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(particleLifespanMode.setValue(0));
		}
		
		MPlug particleCollide = fnParticle.findPlug("collide", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(particleCollide.setValue(false));
		}
		
		MPlug particleForcesInWorld = fnParticle.findPlug("forcesInWorld", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(particleForcesInWorld.setValue(false));
		}
		
		MPlug particleIgnoreSolverWind = fnParticle.findPlug("ignoreSolverWind", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(particleIgnoreSolverWind.setValue(true));
		}
		
		MPlug particleIgnoreSolverGravity = fnParticle.findPlug("ignoreSolverGravity", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(particleIgnoreSolverGravity.setValue(true));
		}
		
		MPlug particlePointSize = fnParticle.findPlug("pointSize", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(particlePointSize.setValue(1));
		}
		
		MPlug particleMaxIterations = fnParticle.findPlug("maxIterations", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(particleMaxIterations.setValue(0));
		}
		
		// Set nucleus attributes using C++ API
		MPlug nucleusSubSteps = fnNucleus.findPlug("subSteps", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(nucleusSubSteps.setValue(1));
		}
		
		MPlug nucleusMaxCollisionIterations = fnNucleus.findPlug("maxCollisionIterations", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(nucleusMaxCollisionIterations.setValue(0));
		}
		
		MPlug nucleusStartFrame = fnNucleus.findPlug("startFrame", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(nucleusStartFrame.setValue(0));
		}
		
		// Set emitter attributes using C++ API
		MPlug emitterRate = fnEmitter.findPlug("rate", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(emitterRate.setValue(0.0));
		}
		
		MPlug emitterSpeed = fnEmitter.findPlug("speed", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(emitterSpeed.setValue(0.0));
		}
		
		// Lock numberOfPoints attribute using C++ API
		MPlug particleNumberOfPoints = fnParticle.findPlug("numberOfPoints", false, &status);
		if (status == MS::kSuccess)
		{
			CHECK_MSTATUS_AND_RETURN_IT(particleNumberOfPoints.setKeyable(true));
			CHECK_MSTATUS_AND_RETURN_IT(particleNumberOfPoints.setChannelBox(false));
			CHECK_MSTATUS_AND_RETURN_IT(particleNumberOfPoints.setLocked(true));
		}
		
		return MS::kSuccess;
	}
}

