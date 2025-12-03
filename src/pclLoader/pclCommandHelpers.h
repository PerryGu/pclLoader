#ifndef PCLCOMMANDHELPERS_H
#define PCLCOMMANDHELPERS_H

// Maya API
#include <maya/MObject.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MStatus.h>

/// Shared utility functions for PCL command implementations.
/// These helpers provide common node creation, connection, and attribute management
/// functionality that can be reused across pclLoaderCmd, pclSequencerCmd, pclRaycasterCmd, etc.
namespace PclCommandHelpers
{
	// ============================================================================
	// Common Node Creation Utilities
	// ============================================================================
	
	/// Creates a DAG node with an MDagModifier (undoable).
	/// @param typeName The Maya node type name (e.g., "pclLoader", "locator").
	/// @param outObject Output parameter for the created node object.
	/// @return MStatus indicating success or failure.
	MStatus createDagNode(const MString& typeName, MObject& outObject);
	
	/// Creates a dependency node (non-DAG) with an MDGModifier.
	/// @param typeName The Maya node type name (e.g., "pclSequencer").
	/// @param outObject Output parameter for the created node object.
	/// @return MStatus indicating success or failure.
	MStatus createDependencyNode(const MString& typeName, MObject& outObject);
	
	/// Renames the parent transform (if it exists) for the provided DAG node.
	/// @param dagNodeObj The DAG node object to rename.
	/// @param newName The new name for the node.
	/// @return MStatus indicating success or failure.
	MStatus renameParentTransform(const MObject& dagNodeObj, const MString& newName);
	
	/// Appends a DAG node's name to the provided array when it is valid.
	/// @param obj The node object to check.
	/// @param list The array to append the name to.
	void appendNodeNameIfValid(const MObject& obj, MStringArray& list);
	
	// ============================================================================
	// Sequencer-Specific Helpers
	// ============================================================================
	
	/// Creates an emitter node and returns its shape object.
	/// @param outEmitterObj Output parameter for the created emitter node.
	/// @return MStatus indicating success or failure.
	MStatus createEmitterNode(MObject& outEmitterObj);
	
	/// Creates an nParticle node and returns its shape object.
	/// @param outParticleObj Output parameter for the created particle node.
	/// @return MStatus indicating success or failure.
	MStatus createNParticleNode(MObject& outParticleObj);
	
	/// Gets the nucleus node connected to a particle system via startState.
	/// @param particleObj The particle system node.
	/// @param outNucleusObj Output parameter for the nucleus node.
	/// @return MStatus indicating success or failure.
	MStatus getNucleusFromParticle(const MObject& particleObj, MObject& outNucleusObj);
	
	/// Connects an emitter to a particle system using connectDynamic.
	/// Note: connectDynamic doesn't have a direct C++ API, so we use MEL.
	/// @param emitterObj The emitter node.
	/// @param particleObj The particle system node.
	/// @return MStatus indicating success or failure.
	MStatus connectEmitterToParticle(const MObject& emitterObj, const MObject& particleObj);
	
	/// Adds custom attributes to a particle system node.
	/// Adds: rgbPP, pointSize, MapperAttributs, createPoints, numberOfPoints, mapper_msg.
	/// @param particleObj The particle system node.
	/// @return MStatus indicating success or failure.
	MStatus addParticleSystemAttributes(const MObject& particleObj);
	
	/// Adds custom attributes to an emitter node.
	/// Adds: mapper_msg.
	/// @param emitterObj The emitter node.
	/// @return MStatus indicating success or failure.
	MStatus addEmitterAttributes(const MObject& emitterObj);
	
	/// Sets default attribute values on particle, nucleus, and emitter nodes.
	/// Configures particle physics settings, nucleus solver settings, and emitter rate/speed.
	/// @param particleObj The particle system node.
	/// @param nucleusObj The nucleus solver node.
	/// @param emitterObj The emitter node.
	/// @return MStatus indicating success or failure.
	MStatus setSequencerNodeDefaults(const MObject& particleObj, const MObject& nucleusObj, const MObject& emitterObj);
}

#endif // PCLCOMMANDHELPERS_H

