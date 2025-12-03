#ifndef PCLSEQUENCERCMD_H
#define PCLSEQUENCERCMD_H

// Maya API
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MObject.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MSyntax.h>

/// Command to create and configure a pclSequencer node with its required particle system setup.
/// Creates: pclSequencer (mapper), emitter, nParticle, and nucleus nodes, then connects them.
class PclSequencerCmd : public MPxCommand
{
public:
	PclSequencerCmd();
	static void* creator();
	
	static MSyntax newSyntax();
	
	MStatus doIt(const MArgList& argList) override;
	bool isUndoable() const override { return false; }
	
private:
	MStatus parseArguments(const MArgList& argList);
	MStatus createSequencerNodes();
	MStatus connectSequencerNodes();
	MStatus connectToPclLoader();
	void renameSequencerNodes();
	void collectCreatedNodeNames(MStringArray& outNames) const;
	
	MString mName;
	MString mNameMapper;
	MString mNameEmitter;
	MString mNameParticle;
	MString mNameNucleus;
	MString mPclLoaderName;
	bool mCreatePoints = false;
	
	MObject mSequencer_obj;
	MObject mEmitter_obj;
	MObject mParticle_obj;
	MObject mNucleus_obj;
	MObject mPclLoader_obj;
};

#endif // PCLSEQUENCERCMD_H

