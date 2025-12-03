#ifndef PCRAYCASTER
#define PCRAYCASTER

// Standard library
#include <vector>

// Maya API
#include <maya/MPxLocatorNode.h>
#include <maya/MStatus.h>
#include <maya/MString.h>
#include <maya/MPointArray.h>
#include <maya/MObject.h>
#include <maya/MPlugArray.h>
#include <maya/MVectorArray.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MFloatPoint.h>
#include <maya/MColor.h>
#include <maya/MDrawRegistry.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MUserData.h>
#include <maya/MDrawContext.h>

// Forward declarations
class MDagPath;
class M3dView;
class MBoundingBox;
class MMatrix;
class MIntArray;
class MFloatArray;



/// Stores the various point sets generated when filtering against the container.
struct PointsInOutContainer
{
	MPointArray pointsInContainer;
	MPointArray pointsOutContainer;
	MVectorArray closestPointHit;
	MVectorArray pointsColorInContainer;
	MVectorArray pointsColorOutContainer;
};


class MPlug;
class MDataBlock;

/// Performs ray casting against cached point-cloud frames.
class PclRaycaster : public MPxLocatorNode
{
public:
	PclRaycaster();
	~PclRaycaster() override;

	/// Maya factory hook.
	static  void*		creator();
	/// Defines node attributes.
	static  MStatus		initialize();
	/// Assigns a unique shape name after creation.
	void		postConstructor() override;
	/// Main evaluation entry point.
	MStatus		compute(const MPlug& plug, MDataBlock& dataBlock) override;
	/// Declares which outputs are dirtied by a given input.
	MStatus		setDependentsDirty(const MPlug &plugBeingDirtied, MPlugArray &affectedPlugs) override;
	/// Draws the locator in the classic viewport.
	void		draw(M3dView& view, const MDagPath& DGpath, M3dView::DisplayStyle style, M3dView::DisplayStatus status) override;
	/// Helper for drawing a single point.
	void		drawPoints(MFloatPoint pointPos, MColor inColAry, float pointScale);
	/// Extracts camera data for ray casting.
	MVectorArray		getCameraData(MObject cameraMatrix_Attr, MDagPath m_camDagPath, int raycastInputX, int raycastInputY);
	/// Computes the camera frustum corners given clip planes.
	MFloatVectorArray	get3dPoint(double nearClipPlane, double farClipPlane, double left, double right, double bottom, double top);

public:
	static  MTypeId		id;
	static	MString		drawDbClassification;
	static	MString		drawRegistrantId;
	static	MObject		particleCount;

	static MObject inPointsPosition_Attr;
	static MObject inPointsColor_Attr;
	static MObject inTime_Attr;
	static MObject pointNumber_Attr;
	static MObject timeOffset_Attr;
	static MObject displayType_Attr;
	static MObject raycastOnOff_Attr;
	static MObject raycastInputX_Attr;
	static MObject raycastInputY_Attr;
	static MObject minRadius_Attr;
	static MObject pointsSize_Attr;
	static	MObject	outPosition_Attr;
	static	MObject	outColor_Attr;
	static	MObject	outPositionRaycast_Attr;
	static	MObject	closestPointHit_Attr;
	static MObject cameraMatrix_Attr;

private:
	/// Snapshot of frequently accessed plug values per compute.
	struct EvaluateContext
	{
		float timeVal = 0.0f;
		float timeOffset = 0.0f;
		short displayType = 0;
		short raycastOnOff = 0;
		unsigned int raycastInputX = 1;
		unsigned int raycastInputY = 1;
		float minRadius = 1.0f;
		float pointSize = 1.0f;
		MMatrix cameraMatrix;
	};
	/// Filters points based on container bounds.
	PointsInOutContainer	checkIfPointsInContaner(MPointArray inPosAry, MVectorArray inColAry, MBoundingBox the_bbox);
	/// Filters points based on the camera frustum.
	PointsInOutContainer	getPointsInFrustum(MPointArray inPosAry, MVectorArray inColAry, MMatrix camMatrixInverse, MMatrix projectionMatrix);

	MObject					thisNode;
	MPointArray				inPosAry;
	MPointArray				outPosAry;
	MVectorArray			inColAry;
	MVectorArray			outColAry;
	MPointArray				pointsInRaycastArray;
	float					minRadius;
	float					pointsSize;
	MIntArray				locInScene;
	MPointArray		 		pointsInRadius;
	MFloatArray				boundingBoxsToDraw_info;
	int						overrideColor = -1;
};


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Viewport 2.0 override implementation
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
class PclRaycasterData : public MUserData
{
public:
	PclRaycasterData() : MUserData(false) {} // don't delete after draw
	virtual ~PclRaycasterData() {}

	short			displayType;
	short			raycastOnOff;
	float			pointSize;
	MPointArray		inPosAryV2;
	MPointArray		inPosAryRaycastV2;
	
};

//---------------------------------------------------------------------
class PclRaycasterDrawOverride : public MHWRender::MPxDrawOverride
{
public:
	static MHWRender::MPxDrawOverride* Creator(const MObject& obj)
	{
		return new PclRaycasterDrawOverride(obj);
	}

	virtual ~PclRaycasterDrawOverride();
	virtual MHWRender::DrawAPI supportedDrawAPIs() const;
	virtual MUserData* prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath, const MHWRender::MFrameContext& frameContext, MUserData* oldData);
	virtual bool hasUIDrawables() const { return true; }
	virtual void addUIDrawables(const MDagPath& objPath, MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext, const MUserData* data);
	static void draw(const MHWRender::MDrawContext& context, const MUserData* data) {};

private:
	PclRaycasterDrawOverride(const MObject& obj);

	short		 get_displayType(const MDagPath& objPath) const;
	short		 get_raycastOnOff(const MDagPath& objPath) const;
	float		 get_pointsSize(const MDagPath& objPath) const;
	MPointArray  get_pointPosition(const MDagPath& objPath) const;
	MPointArray  get_pointPositionRaycast(const MDagPath& objPath) const;

};



#endif
