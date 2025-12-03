#ifndef PCLLOADER
#define PCLLOADER

// Standard library
#include <vector>

// Project headers
#include "core/pclFrameCache.h"

// Maya API
#include <maya/MPxLocatorNode.h>
#include <maya/MTypeId.h>
#include <maya/MStatus.h>
#include <maya/MString.h>
#include <maya/MObject.h>
#include <maya/MPlugArray.h>
#include <maya/MPointArray.h>
#include <maya/MVectorArray.h>
#include <maya/MMatrix.h>
#include <maya/MBoundingBox.h>
#include <maya/MColor.h>
#include <maya/MFloatPoint.h>

// Maya draw/viewport types
#include <maya/M3dView.h>
#include <maya/MDagPath.h>
#include <maya/MUserData.h>
#include <maya/MDrawContext.h>
#include <maya/MDrawRegistry.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MPxDrawOverride.h>



class PclLoader : public MPxLocatorNode
{
public:
	/// Default constructor establishes baseline member values.
	PclLoader();
	/// Ensures Maya deletes any owned resources at shutdown.
	~PclLoader() override;
	/// Factory method used by Maya when instantiating the node.
	static void* creator();
	/// Defines all node attributes and affects relationships.
	static MStatus initialize();
	/// Renames the created shape for predictable node names.
	void postConstructor() override;
	/// Main evaluation entry point for dependency graph dirties.
	MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;

	/// Viewport 1.0 draw callback.
	void draw(M3dView& view, const MDagPath& DGpath, M3dView::DisplayStyle style, M3dView::DisplayStatus status) override;
	/// Draws the container bounding box when VP1 is active.
	void drawContaineBoundingBox(M3dView& view, MPointArray boundingBoxsToDrawInfo, MColor pointCol);
	/// Draws a single point in VP1 legacy mode.
	void drawPoints(MFloatPoint pointPos, MColor inColAry, float pointsSize);

	/// Pads frames so that every frame has the highest point count.
	MStatus addExtraPointsToFramesWithLowPointNumber();
	/// Extracts container corners from a PCL point list.
	void getBoundingBoxCornersFromPCL(MPointArray pointArrayPos_list);

	/// Writes an MPointArray value to an output plug.
	void setPointArrayOutput(MDataBlock& dataBlock, MObject attr, const MPointArray& values);
	/// Writes an MVectorArray value to an output plug.
	void setVectorArrayOutput(MDataBlock& dataBlock, MObject attr, const MVectorArray& values);

	/// Begins legacy VP1 OpenGL drawing (state setup).
	void beginViewportDraw(M3dView& view) const;
	/// Emits the cached point cloud primitive for VP1.
	void drawPointsPrimitive() const;
	/// Emits the container primitive for VP1.
	void drawContainerPrimitive(M3dView& view) const;
	/// Restores GL state after VP1 drawing.
	void endViewportDraw(M3dView& view) const;

	/// Lazily discovers the container on/off plug on this node.
	MStatus getTheContainer_plugj();
	/// Keyframes the time attribute to match requested range.
	MStatus editKeyframesOnTimeAttr(unsigned int loadStartFrame, unsigned int loadEndFrame);
	/// Declares which outputs are affected by which inputs.
	MStatus setDependentsDirty(const MPlug& plugBeingDirtied, MPlugArray& affectedPlugs) override;

	/// Loads the requested frames from the cache into Maya arrays.
	bool loadFramesFromCache(const MString& mainPath, unsigned int skipFilesRows);
	/// Converts cached frame data into Maya-specific array data.
	void convertCacheFrame(const pclFrameCache::FrameData& frame, MPointArray& outPositions, MPointArray& outColors, MVectorArray& outPosVec, MVectorArray& outColorVec, MPointArray& outBBox);
	/// Clears all cached frame data and resets evaluation state.
	void clearLoadedData();

public:
	static MTypeId id;
	static MString drawDbClassification;
	static MString drawRegistrantId;

	static MObject saperator_Attr;
	static MObject mainPath_Attr;
	static MObject containerOnOff_Attr;
	static MObject startFrame_Attr;
	static MObject endFrame_Attr;
	static MObject loadStartFrame_Attr;
	static MObject loadEndFrame_Attr;
	static MObject skipFilesRows_Attr;
	static MObject loadFile_Attr;
	static MObject displayPoints_Attr;
	static MObject pointsSize_Attr;
	static MObject pointNumber_Attr;
	static MObject inContainerMatrix_Attr;
	static MObject inTime_Attr;
	static MObject inlRaycasterPoints_Attr;
	static MObject timeOffset_Attr;

	static MObject outBoundingBoxCorner_Attr;
	static MObject outDrawContainer_Attr;
	static MObject outPosition_Attr;
	static MObject outColor_Attr;
	static MObject outPositionMaxFrame_Attr;
	static MObject outPositionVec_Attr;
	static MObject outColorVec_Attr;
	static MObject theHighestNumberOfPoints_Attr;
	static MObject out_Attr;

	
private:
	/// Snapshot of frequently accessed plug values per compute.
	struct EvaluateContext
	{
		MString mainPath;
		unsigned int loadStartFrame = 0;
		unsigned int loadEndFrame = 0;
		unsigned int skipRows = 0;
		short loadFileMode = 0;
		short containerOnOff = 0;
		short displayPoints = 0;
		float pointSize = 1.0f;
		int timeOffset = 0;
		MMatrix containerMatrix;
	};

	MObject	mThisNode;
	MString mMainPath;
	MBoundingBox mContainer_bbox;
	MPlug theContanerOnOff_plug;
	MPoint mBboxCorner1_lastPos;
	MPoint mBboxCorner2_lastPos;
	bool defaultContainer_bBox = true;
	bool mParticlesConnected = false;
	bool mLoadOnes = true;
	bool eval_state = true;
	short mLoadFile = 0;
	short mDisplayPoints = 0;
	short mContainerOnOff = 0;
	unsigned int mLastTimeVal = 0;
	unsigned int mStartFrameVal = 0;
	unsigned int mLastFrameVal = 0;
	unsigned int mNumberOfFrames = 0;
	unsigned int mNumberOfPointsEachFrame = 0;
	unsigned int mTheHighestNumberOfPoints = 0;
	float mPointsSize = 1.0;
	std::vector<MPointArray> mBoundingBoxCorner_list;
	std::vector<MPointArray> mPointArrayPosAllFrame_list;
	std::vector<MPointArray> mPointArrayColAllFrame_list;
	MPointArray mPointPosFrameOut_list;
	MPointArray mPointColFrameOut_list;

	//-- this attributs are for particle system ------------------------
	std::vector<MVectorArray> mPointArrayPosAllFrameVec_list;
	std::vector<MVectorArray> mPointArrayColAllFrameVec_list;
	MVectorArray mPointPosFrameOutVec_list;
	MVectorArray mPointColFrameOutVec_list;

	/// Tracks container bounds and cached draw data.
	struct ContainerState
	{
		MBoundingBox bbox;
		MPointArray drawLines;
		MPointArray drawLinesV2;
		MPoint lastMin = MPoint(10, 10, 10);
		MPoint lastMax = MPoint(-10, 0, -10);
		bool defaultBox = true;

		void reset()
		{
			drawLines.clear();
			drawLinesV2.clear();
			bbox.clear();
			lastMin = MPoint(10, 10, 10);
			lastMax = MPoint(-10, 0, -10);
			defaultBox = true;
		}
	} mContainerState;

	pclFrameCache mFrameCache;
	unsigned int mLoadedSkipRows = 0;

	// Applies loadFiles mode and mainPath settings to the cache.
	MStatus processLoadRequest(EvaluateContext& ctx, MDataBlock& dataBlock);
	// Ensures the requested load window stays within available frame bounds.
	void clampLoadWindow(EvaluateContext& ctx, MDataBlock& dataBlock);
	// Pushes the selected frame's point data into the node's outputs.
	void updateFrameOutputs(const EvaluateContext& ctx, MDataBlock& dataBlock);
	// Updates bounding-box outputs and container draw data.
	void updateBoundingBoxOutputs(const EvaluateContext& ctx, MDataBlock& dataBlock);
	// Writes surface/raycaster dependent outputs to the datablock.
	void updateRaycasterInputs(MDataBlock& dataBlock);

	// Rebuilds cached bounding box state from the latest frame.
	void updateContainerState(const EvaluateContext& ctx);
	// Prepares point arrays used by both VP1 and VP2 drawing paths.
	void buildContainerDrawData();
	
	/// Filters points in/out of the container for scalar + particle streams.
	static void filterPoints(const MPointArray& sourcePos,
	                         const MPointArray& sourceCol,
	                         const MBoundingBox& bbox,
	                         bool keepInside,
	                         const MVectorArray* sourcePosVec,
	                         const MVectorArray* sourceColVec,
	                         MPointArray& filteredPos,
	                         MPointArray& filteredCol,
	                         MVectorArray* filteredPosVec,
	                         MVectorArray* filteredColVec);
	
};


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Viewport 2.0 override implementation
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
class PclLoaderData : public MUserData
{
public:
	PclLoaderData() : MUserData(false) {} // don't delete after draw
	virtual ~PclLoaderData() {}

	short			mLoadFile;
	short			mDisplayPoints;
	short			mContainerOnOff;
	float			mPointSize;
	MPointArray		mPointPosition;
	MPointArray		mPointColor;
	MPointArray		mContainerBBox;
	MPointArray     mRaycasterPoints;

};


//---------------------------------------------------------------------
class PclLoaderDrawOverride : public MHWRender::MPxDrawOverride
{
public:
	static MHWRender::MPxDrawOverride* Creator(const MObject& obj)
	{
		return new PclLoaderDrawOverride(obj);
	}
	
	virtual ~PclLoaderDrawOverride();
	virtual MHWRender::DrawAPI supportedDrawAPIs() const;
	virtual MUserData* prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath, const MHWRender::MFrameContext& frameContext, MUserData* oldData);
	virtual bool hasUIDrawables() const { return true; }
	virtual void addUIDrawables(const MDagPath& objPath, MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext, const MUserData* data);
	static void draw(const MHWRender::MDrawContext& context, const MUserData* data) {};
	
private:

	PclLoaderDrawOverride(const MObject& obj);

	short		 get_loadFile(const MDagPath& objPath) const;
	short		 get_displayPoints(const MDagPath& objPath) const;
	short		 get_containerOnOff(const MDagPath& objPath) const;
	float		 get_pointsSize(const MDagPath& objPath) const;
	MPointArray  get_pointPosition(const MDagPath& objPath) const;
	MPointArray  get_pointColor(const MDagPath& objPath) const;
	MPointArray  get_containerBBox(const MDagPath& objPath) const;
	MPointArray	 get_pointsInRaycaste(const MDagPath& objPath) const;

};

#endif