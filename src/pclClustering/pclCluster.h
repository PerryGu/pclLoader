#ifndef PCLCLUSTER_H
#define PCLCLUSTER_H

// Standard library
#include <vector>

// Maya API
#include <maya/MPxLocatorNode.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>
#include <maya/MObject.h>
#include <maya/MStatus.h>
#include <maya/MPlugArray.h>
#include <maya/MPointArray.h>
#include <maya/MVectorArray.h>
#include <maya/MFloatArray.h>
#include <maya/MIntArray.h>
#include <maya/MBoundingBox.h>
#include <maya/MDagPath.h>
#include <maya/MDrawRegistry.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MUserData.h>

// Forward declarations
class MPlug;
class MDataBlock;
class MArrayDataHandle;
class M3dView;
class MFnParticleSystem;

/// Octree bounding box information used for spatial clustering.
struct BoundingBoxInfo
{
	unsigned int boundingBox_id;                    ///< Unique identifier for this bounding box.
	bool inCluster = false;                          ///< True if this box is part of a cluster.
	unsigned int recursion_count = 0;               ///< Depth level in the octree hierarchy.
	MBoundingBox boundingBox_parent;                 ///< Parent bounding box in the octree.
	std::vector<BoundingBoxInfo> boundingBoxChild_list;  ///< Child bounding boxes (8 for octree).
	MPointArray pointsIn_boundingBox;                ///< Points contained within this bounding box.
};

/// Tracks a cluster's position sequence across multiple frames.
struct ClustersSequencePath
{
	MPointArray clusterPosition_list;  ///< 3D positions of the cluster center over time.
	MIntArray clusterFrame_list;       ///< Frame numbers corresponding to each position.
	unsigned int cluster_id;            ///< Unique identifier for this cluster sequence.
	unsigned int startFrame;            ///< First frame where this cluster appears.
	unsigned int lastFrame;             ///< Last frame where this cluster appears.

	/// Returns the number of positions in the sequence.
	unsigned int listCounter() const
	{
		return clusterPosition_list.length();
	}
};

/// Performs octree-based spatial clustering on point cloud data.
/// Divides space hierarchically and groups nearby points into clusters.
class pclCluster : public MPxLocatorNode
{
public:
	pclCluster();
	virtual ~pclCluster();
	virtual void postConstructor() override;

	/// Main evaluation entry point.
	virtual MStatus compute(const MPlug& plug, MDataBlock& data) override;
	/// Helper to navigate array data handles.
	MStatus jumpToElement(MArrayDataHandle& hArray, unsigned int index);
	/// Declares which outputs are dirtied by a given input.
	virtual MStatus setDependentsDirty(const MPlug& plugBeingDirtied, MPlugArray& affectedPlugs) override;
	/// Draws the locator in the classic viewport.
	virtual void draw(M3dView& view, const MDagPath& DGpath, M3dView::DisplayStyle style, M3dView::DisplayStatus status) override;
	/// Draws the global bounding box visualization.
	void drawGlobalBoundingBox(M3dView& view, MFloatArray globalBoundingBoxsToDraw_info);
	/// Draws individual bounding boxes for octree cells.
	void drawBoundingBox(M3dView& view, MPointArray boundingBoxsToDraw_info);
	/// Draws cross markers at cluster centers.
	void drawCross(M3dView& view, MPointArray crossToDrawInfo, float clusterDrawScale);
	/// Draws the sequence path lines connecting cluster positions across frames.
	void drawClusterSequencPath(M3dView& view, MPointArray clustersSequencePos);
	virtual bool isBounded() const override;
	virtual bool isTransparent() const override;
	virtual MBoundingBox boundingBox() const override;

	/// Samples points from the input particle system.
	MPointArray getPointSampling(MPointArray pointsInSampling_input, MVectorArray particleInSampling_input, unsigned int pointsSampling);
	/// Recursively builds an octree structure from the input points.
	BoundingBoxInfo startOctree(BoundingBoxInfo boundingBoxInfo_parent, MPointArray pointsIn_boundingBox, unsigned int recursion_count);
	/// Filters points that lie within a given bounding box.
	MPointArray isPointInBoundingBox(MBoundingBox bBox, MPointArray pointsIn_boundingBox);
	/// Divides a bounding box into 8 child boxes (octree subdivision).
	std::vector<BoundingBoxInfo> divideBoundingBox(MBoundingBox bBox);
	/// Collects bounding box data for visualization.
	MStatus gatherBoundingBoxsData(std::vector<BoundingBoxInfo> lastBoundingBox_list);
	/// Groups nearby bounding boxes into clusters.
	std::vector<std::vector<BoundingBoxInfo>> clusteringBoundingBoxs(std::vector<BoundingBoxInfo> boundingBoxInfo_list);
	/// Extracts cluster center positions for drawing.
	MStatus gatherClustersCrosessData(std::vector<std::vector<BoundingBoxInfo>> boundingBoxOutputMain_list);
	/// Finds the closest bounding box to a given center point.
	std::vector<BoundingBoxInfo> getClosestBoundingBox(MPoint boundingBox_center, float minWith, float minDepth, std::vector<BoundingBoxInfo> boundingBoxOutput_list, unsigned int counter);
	/// Tracks cluster movement across frames by finding nearest neighbors.
	void findClosestClusterInNextFrame(MPoint clusterCenter, unsigned int timeVal);
	/// Populates the cluster sequence path structure with frame data.
	void populateClustersSequencePath_struct(unsigned int timeVal, MPointArray clustersPosition);
	/// Retrieves the connected particle system node.
	MStatus getConnetdedParticleSys(MPlug mPlug);
	/// Creates NURBS curves representing cluster paths.
	void createCurves();

	// Static members
	static MTypeId id;
	static MString drawDbClassification;
	static MString drawRegistrantId;
	static void* creator();
	static MStatus initialize();

	// Attribute declarations
	static MObject saperator_Attr;
	static MObject pointsSampling_Attr;
	static MObject action_Attr;
	static MObject globalBoundingBox_Attr;
	static MObject recursionDepth_Attr;
	static MObject minimumPointCount_Attr;
	static MObject boundingBoxVis_Attr;
	static MObject clusterCenter_Attr;
	static MObject clustersGrouping_Attr;
	static MObject clusterDrawScale_Attr;
	static MObject clustersDetection_Attr;
	static MObject createClusterPath_Attr;
	static MObject nextFrameSearchRadius_Attr;
	static MObject inParticleMatrix_Attr;
	static MObject inPointsPosition_Attr;
	static MObject inPointsColor_Attr;
	static MObject inParticlePP_Attr;
	static MObject inTime_Attr;
	static MObject clusterCenterPos_Attr;
	static MObject inBoundingBoxCorner_Attr;
	static MObject outDivBoundingBoxDraw_Attr;
	static MObject outCrossDraw_Attr;
	static MObject outClustersSequencePathDraw_Attr;
	static MObject out_Attr;
	static MObject createCurvs_Attr;

private:
	/// Bundles frequently accessed inputs from compute() for clarity and efficiency.
	struct EvaluateContext
	{
		unsigned int pointsSampling;
		bool action;
		short createClusterPath;
		bool boundingBoxVis;
		short clusterGrouping;
		MTime timeVal;
		int frame;
		int adjustFrame;
		MPointArray boundingBoxCorner_list;
		bool particleMatrixConnected;
	};

	MObject mThisNode;
	MFnParticleSystem mParticle_sys;
	MDagPath mParticle_path;
	MBoundingBox mGlobalBoundingBox;
	double mBoundingBox_scale[3];
	bool eval_state = false;
	bool eval_state2 = false;
	bool eval_state3 = false;
	bool eval_state4 = false;
	bool eval_state5 = false;
	bool mClusterCentering = false;
	unsigned int mMinimumPoint_count;
	unsigned int mLastTimeVal = 0;
	unsigned int mRecursionDepth = 0;
	unsigned int mRecursion_count = 0;
	unsigned int mBoundingBoxIndex = 0;
	double mClusterDrawScale = 1;
	double mNextFrameSearchRadius = 1;
	float mMinWidthBb = 0.5;

	std::vector<BoundingBoxInfo> mBoundingBoxInfo_list;
	std::vector<BoundingBoxInfo> mLastBoundingBox_list;
	std::vector<MBoundingBox> mClusterBoundingBox;
	std::vector<BoundingBoxInfo> mDupBoundingBox_list;
	std::vector<std::vector<BoundingBoxInfo>> mBoundingBoxOutputMain_list;
	std::vector<MPointArray> clustersSequencePos_list;
	MPointArray mInPointPos_list;
	MPointArray mBoundingBoxsToDrawInfo;
	MPointArray mCrossToDraw_Info;
	MPointArray mClustersPathToDraw_list;
	MFloatArray mGlobalBoundingBoxsToDraw_info;
	std::vector<MPointArray> mBoundingBoxsToDraw_info;
	std::vector<ClustersSequencePath> mClustersSequencePath_list;
};

/// User data structure for Viewport 2.0 draw override.
class PclClusterData : public MUserData
{
public:
	PclClusterData() : MUserData(false) {} // don't delete after draw
	~PclClusterData() override {}

	bool fBoundingBoxVis;
	bool fCreateClusterPath;
	unsigned int fDepthPriority;
	short fcontanerOnOff;
	short floadFilesOnOff;
	float fClusterDrawScale;
	float fNextFrameSearchRadius;
	MPointArray fGlobalBoundingBoxs_list;
	MPointArray fBpDivPoint_list;
	MPointArray fCrossPoint_list;
	std::vector<MPointArray> fClustersSequencePath_vec;
};

/// Viewport 2.0 draw override for pclCluster visualization.
class PclClusterDrawOverride : public MHWRender::MPxDrawOverride
{
public:
	static MHWRender::MPxDrawOverride* Creator(const MObject& obj)
	{
		return new PclClusterDrawOverride(obj);
	}

	~PclClusterDrawOverride();

	virtual MHWRender::DrawAPI supportedDrawAPIs() const override;
	virtual MUserData* prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath, const MHWRender::MFrameContext& frameContext, MUserData* oldData) override;
	virtual bool hasUIDrawables() const override { return true; }
	virtual void addUIDrawables(const MDagPath& objPath, MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext, const MUserData* data) override;
	static void draw(const MHWRender::MDrawContext& context, const MUserData* data) {};
	virtual bool isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const override;
	virtual MBoundingBox boundingBox(const MDagPath& objPath) const override;

private:
	PclClusterDrawOverride(const MObject& obj);

	bool get_boundingBoxVis(const MDagPath& objPath) const;
	bool get_createClusterPath(const MDagPath& objPath) const;
	MPointArray get_outDivBBoxPP(const MDagPath& objPath) const;
	MPointArray get_outCrossPP(const MDagPath& objPath) const;
	std::vector<MPointArray> get_clustersSequencePath(const MDagPath& objPath) const;
	float get_clusterDrawScale(const MDagPath& objPath) const;
	float get_nextFrameSearchRadius(const MDagPath& objPath) const;
};

#endif // PCLCLUSTER_H
