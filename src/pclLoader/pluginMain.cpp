// Maya API
#include <maya/MFnPlugin.h>
#include <maya/MHWRender.h>
#include "pclLoaderCmd.h"
#include "pclLoader.h"
#include "pclContainer.h"
#include "../pclSequencer/pclSequencer.h"
#include "../pclSequencer/pclSequencerCmd.h"
#include "../pclRaycaster/pclRaycaster.h"
#include "../pclRaycaster/pclRaycasterCmd.h"
#include "../pclClustering/pclCluster.h"
#include "../pclClustering/pclClusteringCmd.h"

namespace
{
	// Plugin metadata
	constexpr char kPluginVendor[] = "Guy Perry";
	constexpr char kPluginVersion[] = "maya2016_1.0.1_open3D";
	constexpr char kPluginRequiredApi[] = "vs_19";

	// Registration data structures
	// Describes a single command to register
	struct CommandRegistration
	{
		const char* name;
		MCreateFunction creator;
		MSyntax(*syntaxFn)();
	};

	// Describes a single node to register
	struct NodeRegistration
	{
		const char* name;
		MTypeId typeId;
		MCreateFunction creator;
		MInitializeFunction initializer;
		MPxNode::Type nodeType;
		const MString* classification;
	};

	// Describes a single draw override to register
	struct DrawOverrideRegistration
	{
		const MString* classification;
		const MString* registrantId;
		MHWRender::MDrawOverrideCreator creator;
	};

	// Registration tables
	// Commands exposed by this plugin
	const CommandRegistration kCommands[] = {
		{ "pclLoaderCmd", PclLoaderCmd::creator, PclLoaderCmd::newSyntax },
		{ "pclSequencerCmd", PclSequencerCmd::creator, PclSequencerCmd::newSyntax },
		{ "pclRaycasterCmd", pclRaycasterCmd::creator, pclRaycasterCmd::newSyntax },
		{ "pclClusteringCmd", PclClusteringCmd::creator, PclClusteringCmd::newSyntax }
	};

	// Nodes exposed by this plugin
	const NodeRegistration kNodes[] = {
		{ "pclLoader",   PclLoader::id,   PclLoader::creator,   PclLoader::initialize,   MPxNode::kLocatorNode, &PclLoader::drawDbClassification },
		{ "pclContainer", PclContainer::id, PclContainer::creator, PclContainer::initialize, MPxNode::kLocatorNode, nullptr },
		{ "pclSequencer", PclSequencer::id, PclSequencer::creator, PclSequencer::initialize, MPxNode::kParticleAttributeMapperNode, nullptr },
		{ "pclRaycaster", PclRaycaster::id, PclRaycaster::creator, PclRaycaster::initialize, MPxNode::kLocatorNode, &PclRaycaster::drawDbClassification },
		{ "pclCluster", pclCluster::id, pclCluster::creator, pclCluster::initialize, MPxNode::kLocatorNode, &pclCluster::drawDbClassification }
	};

	// Draw overrides exposed by this plugin
	const DrawOverrideRegistration kDrawOverrides[] = {
		{ &PclLoader::drawDbClassification, &PclLoader::drawRegistrantId, PclLoaderDrawOverride::Creator },
		{ &PclRaycaster::drawDbClassification, &PclRaycaster::drawRegistrantId, PclRaycasterDrawOverride::Creator },
		{ &pclCluster::drawDbClassification, &pclCluster::drawRegistrantId, PclClusterDrawOverride::Creator }
	};

	// Register all commands for this plugin.
	MStatus registerCommands(MFnPlugin& plugin)
	{
		for (const auto& cmd : kCommands)
		{
			const MStatus status = plugin.registerCommand(cmd.name, cmd.creator, cmd.syntaxFn);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		return MS::kSuccess;
	}

	// Deregister all commands for this plugin.
	MStatus deregisterCommands(MFnPlugin& plugin)
	{
		for (const auto& cmd : kCommands)
		{
			const MStatus status = plugin.deregisterCommand(cmd.name);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		return MS::kSuccess;
	}

	// Register all nodes for this plugin.
	MStatus registerNodes(MFnPlugin& plugin)
	{
		for (const auto& node : kNodes)
		{
			const MStatus status = plugin.registerNode(
				node.name,
				node.typeId,
				node.creator,
				node.initializer,
				node.nodeType,
				node.classification);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		return MS::kSuccess;
	}

	// Deregister all nodes for this plugin.
	MStatus deregisterNodes(MFnPlugin& plugin)
	{
		const size_t nodeCount = sizeof(kNodes) / sizeof(kNodes[0]);
		for (size_t i = nodeCount; i > 0; --i)
		{
			const MStatus status = plugin.deregisterNode(kNodes[i - 1].typeId);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		return MS::kSuccess;
	}

	// Register all draw overrides for this plugin.
	MStatus registerDrawOverrides()
	{
		for (const auto& entry : kDrawOverrides)
		{
			const MStatus status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
				*entry.classification,
				*entry.registrantId,
				entry.creator);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		return MS::kSuccess;
	}

	// Deregister all draw overrides for this plugin.
	MStatus deregisterDrawOverrides()
	{
		for (const auto& entry : kDrawOverrides)
		{
			const MStatus status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
				*entry.classification,
				*entry.registrantId);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		return MS::kSuccess;
	}
}

// Entry point called when Maya loads the plugin.
MStatus initializePlugin(MObject obj)
{
	MFnPlugin plugin(obj, kPluginVendor, kPluginVersion, kPluginRequiredApi);

	CHECK_MSTATUS_AND_RETURN_IT(registerCommands(plugin));
	CHECK_MSTATUS_AND_RETURN_IT(registerNodes(plugin));
	CHECK_MSTATUS_AND_RETURN_IT(registerDrawOverrides());

	return MS::kSuccess;
}

// Entry point called when Maya unloads the plugin.
MStatus uninitializePlugin(MObject obj)
{
	MFnPlugin plugin(obj);

	CHECK_MSTATUS_AND_RETURN_IT(deregisterDrawOverrides());
	CHECK_MSTATUS_AND_RETURN_IT(deregisterNodes(plugin));
	CHECK_MSTATUS_AND_RETURN_IT(deregisterCommands(plugin));

	return MS::kSuccess;
}