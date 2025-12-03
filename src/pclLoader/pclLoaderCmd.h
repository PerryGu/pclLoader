#ifndef PCLOADERCMD_H
#define PCLOADERCMD_H

// Maya API
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MObject.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MSyntax.h>

class PclLoaderCmd : public MPxCommand
{
public:
	PclLoaderCmd();
	static void* creator();

	static MSyntax newSyntax();

	MStatus doIt(const MArgList& argList) override;
	MStatus connectAll();
	MStatus getShapeNode(MDagPath& path);
	MStatus setKeyframesOnTimeAttr();
	bool    isUndoable() const override { return false; }

private:
	MStatus parseArguments(const MArgList& argList);
	MStatus createRequestedNodes();
	void renameRequestedNodes();
	void collectCreatedNodeNames(MStringArray& outNames) const;

	MString	  mName;
	MString	  mNamePclLoader;
	MString	  mNamePclContainer;
	MString	  mNamePclCluster;
	MString	  mNamePclSequencer;
	MString	  mNamePclRaycaster;
	MString	  mMainPath;
	MString	  mSelCamera;
	bool	  mPclCluster_flag = false;
	bool	  pclSequencer_flag = false;
	bool	  pclRaycaster_flag = false;
	bool      mShowHelp = false;
	MObject	  mPclLoader_obj;
	MObject	  mPclContainer_obj;
	MObject	  mPclCluster_obj;
	MObject	  mPclSequencer_obj;
	MObject	  mPclRaycaster_obj;
};

#endif