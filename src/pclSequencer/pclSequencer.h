#ifndef PCLSEQUENCER_H
#define PCLSEQUENCER_H

// Standard library
#include <vector> 

// Maya API
#include <maya/MTypeId.h>
#include <maya/MStatus.h>
#include <maya/MString.h>
#include <maya/MObject.h>
#include <maya/MPlugArray.h>
#include <maya/MMatrix.h>
#include <maya/MBoundingBox.h>
#include <maya/MVectorArray.h>
#include <maya/MVector.h>
#include <maya/MPointArray.h>
#include <maya/MTime.h>
#include <maya/MFnParticleSystem.h>
#include <maya/MPxParticleAttributeMapperNode.h>



class MPlug;
class MDataBlock;

/// Drives a particle system using sequences of point-cloud frames.
class PclSequencer : public MPxParticleAttributeMapperNode
{
public:
	/// Default constructor.
	PclSequencer();
	/// Destructor ensures Maya releases any resources.
	~PclSequencer() override;

	/// Maya factory method.
	static void* creator();
	/// Defines all node attributes.
	static MStatus initialize();

	/// Main evaluation entry point.
	MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;
	/// Declares which outputs depend on a given input.
	MStatus setDependentsDirty(const MPlug& plugBeingDirtied, MPlugArray& affectedPlugs) override;

protected:
	/// Writes color data to the particle system.
	MStatus setPointColor(const MPlug& plug, MDataBlock& dataBlock, MVectorArray inPointCol);
	/// Writes position data to the particle system.
	MStatus setPointPosition(const MPlug& plug, MDataBlock& dataBlock, MVectorArray inPointPos);
	/// Applies the container mode filtering to remove or zero points.
	void applyContainerFilter(const EvaluateContext& ctx, MVectorArray& positions, int& pointCount);
	/// Handles the save-to-disk trigger for exporting frames.
	void handleSaveToFile(const EvaluateContext& ctx, MDataBlock& dataBlock);

public:
	static MTypeId id;
	static MObject outPositionPP;
	static MObject particleCount;
	static MObject pointsCreate_Attr;
	static MObject inPointsPosition_Attr;
	static MObject inPointsColor_Attr;
	static MObject inTime_Attr;
	static MObject outTime_Attr;
	static MObject pointNumber_Attr;
	static MObject savePCDPLY_Attr;
	static MObject saveAsciiBinary_Attr;
	static MObject saveMainPath_Attr;
	static MObject saveFileName_Attr;
	static MObject saveToFile_Attr;
	static MObject timeOffset_Attr;
	static MObject messagePart_Attr;
	static MObject messageEmittor_Attr;
	static MObject startFrame_Attr;
	static MObject loadFilesDisplayOnOff_Attr;
	static MObject containerOnOff_Attr;
	static MObject locatorMatrix_Attr;
	
	
	

private:
	/// Snapshot of frequently accessed plugs for a single evaluation.
	struct EvaluateContext
	{
		short createMode = 0;
		double timeValue = 0.0;
		float timeOffset = 0.0f;
		int startFrame = 0;
		short containerMode = 0;
		short saveFormat = 0;
		short saveEncoding = 0;
		short saveTrigger = 0;
		MString saveMainPath;
		MString saveFileName;
		MMatrix locatorMatrix;
	};

	virtual				MStatus	savePointsToFile(MString pathFile, MString saveFileName, short savePCDPLY, short saveAsciiBinary);
	virtual				MStatus getTheParticleSys();
	virtual				MStatus getTheEmittor_obj();
	virtual				MStatus setEmitterRate(unsigned int rateValue);

	MPointArray				firstFrameCreateionPoints;
	MFnParticleSystem		particle_sys;
	MObject					emittor_obj;
	MBoundingBox			this_bbox;
	unsigned int			maxPartVal = 0;
	MObject					thisNode;
	bool					locatorMatrixExsis = false;
	bool					setLocatorMatrixDirty = false;
	
	/// Builds an EvaluateContext from the provided datablock.
	EvaluateContext buildEvaluateContext(MDataBlock& dataBlock) const;
	/// Clears cached points and resets downstream plugs.
	void clearLoadedData(const MPlug& plug, MDataBlock& dataBlock, MVectorArray& positions, MVectorArray& colors);
};

#endif
