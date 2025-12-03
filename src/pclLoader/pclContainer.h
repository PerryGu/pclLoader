#ifndef PCLCONTAINER_H
#define PCLCONTAINER_H

// Maya API
#include <maya/MPxLocatorNode.h>
#include <maya/MTypeId.h>
#include <maya/MStatus.h>
#include <maya/MObject.h>
#include <maya/MString.h>
#include <maya/MBoundingBox.h>

// Forward declarations
class MPlug;
class MDataBlock;
class M3dView;
class MDagPath;

/// A simple locator node that serves as a container reference for point cloud filtering.
/// This node doesn't perform any computation itself; it's used by pclLoader to define
/// bounding box regions for filtering points. The actual bounding box is provided by
/// pclLoader via the inContainerMatrix attribute.
class PclContainer : public MPxLocatorNode
{
public:
	PclContainer();
	virtual ~PclContainer();

	/// Maya factory hook.
	static void* creator();
	/// Defines node attributes.
	static MStatus initialize();
	/// Assigns a unique shape name after creation.
	virtual void postConstructor() override;
	/// Empty compute method - this node doesn't perform any computation.
	virtual MStatus compute(const MPlug& plug, MDataBlock& data) override;
	/// Empty draw method - this node has no custom drawing.
	virtual void draw(M3dView& view, const MDagPath& DGpath, M3dView::DisplayStyle style, M3dView::DisplayStatus status) override;
	/// Returns true to indicate this node has a bounding box.
	virtual bool isBounded() const override;
	/// Returns true to allow transparent rendering.
	virtual bool isTransparent() const override;
	/// Returns a static bounding box (actual bounds come from pclLoader).
	virtual MBoundingBox boundingBox() const override;

public:
	static MTypeId id;
	static MString drawRegistrantId;
	static MObject saperator_Attr;  // Visual separator in attribute editor
	static MObject contanerOnOff_Attr;  // Container mode selector

private:
	MObject mThisNode;
	short mContanerOnOff;
};

#endif
