
// Project headers
#include "pclContainer.h"

// Maya API
#include <maya/MFnDependencyNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MPoint.h>

MTypeId PclContainer::id(0x00000112263);
MString PclContainer::drawRegistrantId("");

MObject PclContainer::saperator_Attr;
MObject PclContainer::contanerOnOff_Attr;

/// Default constructor.
PclContainer::PclContainer() = default;

/// Destructor.
PclContainer::~PclContainer() = default;

/// Maya factory method for creating instances of this node.
void* PclContainer::creator()
{
	return new PclContainer();
}

/// Assigns a unique shape name after dependency graph construction.
void PclContainer::postConstructor()
{
	mThisNode = thisMObject();
	MFnDependencyNode fnNode(mThisNode);
	fnNode.setName("pclContainerShape#");
}

/// Empty compute method - this node doesn't perform any computation.
/// The actual container logic is handled by pclLoader, which reads this node's attributes.
MStatus PclContainer::compute(const MPlug&, MDataBlock& )
{
	return MS::kSuccess;
}

/// Empty draw method - this node has no custom viewport drawing.
/// The container visualization is handled by pclLoader.
void PclContainer::draw(M3dView&, const MDagPath&, M3dView::DisplayStyle, M3dView::DisplayStatus)
{
}

/// Returns true to indicate this node has a bounding box.
bool PclContainer::isBounded() const
{
	return true;
}

/// Returns true to allow transparent rendering of the container.
bool PclContainer::isTransparent() const
{
	return true;
}

/// Returns a static bounding box for Maya's culling system.
/// Note: The actual dynamic bounding box is provided by pclLoader via the inContainerMatrix attribute.
MBoundingBox PclContainer::boundingBox() const
{
	// Return a static bounding box for Maya's viewport culling.
	// The actual container bounds come from pclLoader's inContainerMatrix.
	MPoint corner1(-10, 0, -10);
	MPoint corner2(20, 20, 20);
	return MBoundingBox(corner1, corner2);
}

/// Defines all node attributes and their properties.
MStatus PclContainer::initialize()
{
	MStatus status = MS::kSuccess;
	MFnEnumAttribute eAttr;

	// Visual separator in the attribute editor (for UI organization).
	saperator_Attr = eAttr.create("========", "=======");
	eAttr.addField("======", 0);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(saperator_Attr);

	// Container mode selector: Auto (0), Manual (1), All In (2), All Out (3).
	contanerOnOff_Attr = eAttr.create("contanerOnOff", "contanerOnOff");
	eAttr.addField("Auto container", 0);
	eAttr.addField("Manual container", 1);
	eAttr.addField("All In container", 2);
	eAttr.addField("All Out container", 3);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(contanerOnOff_Attr);

	return status;
}
