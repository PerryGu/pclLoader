#ifndef PCLCLUSTERINGCMD_H
#define PCLCLUSTERINGCMD_H

// Maya API
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MObject.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MSyntax.h>

/// Command to create and configure a pclCluster node.
/// Creates the cluster node and sets up default attributes.
class PclClusteringCmd : public MPxCommand
{
public:
	PclClusteringCmd();
	static void* creator();
	
	static MSyntax newSyntax();
	
	MStatus doIt(const MArgList& argList) override;
	bool isUndoable() const override { return false; }
	
private:
	MStatus parseArguments(const MArgList& argList);
	MStatus createClusterNode();
	MStatus connectToPclLoader();
	void renameClusterNode();
	void collectCreatedNodeName(MStringArray& outNames) const;
	
	MString mName;
	MString mPclLoaderName;
	MObject mCluster_obj;
	MObject mPclLoader_obj;
};

#endif // PCLCLUSTERINGCMD_H

