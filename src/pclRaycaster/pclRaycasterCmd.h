
#ifndef PCLRAYCASTERCMD_H
#define PCLRAYCASTERCMD_H

// Maya API
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MDGModifier.h>
#include <maya/MObject.h>
#include <maya/MSelectionList.h>
#include <maya/MString.h>
#include <maya/MSyntax.h>



class pclRaycasterCmd : public MPxCommand
{
public:
	enum CommandMode { kCommandCreate, kCommandExport, kCommandImport, kCommandHelp, kCommandRebind };
	pclRaycasterCmd();
	static void*      creator();

	virtual MStatus   doIt(const MArgList& argList);
	virtual MStatus   connectAll(MObject pclRaycaster_obj, MObject camera_obj, MObject loc_obj);
	virtual MStatus   connectToPclLoader(MObject pclRaycaster_obj);
	virtual MStatus   getShapeNode(MDagPath& path);
	virtual MStatus   undoIt();
	virtual MStatus   redoIt();
	virtual bool      isUndoable() const;
	static MSyntax    newSyntax();

	//-- the name of the command ------
	const static char* kName;

	//-- Specifies the name of the scattering node ----
	const static char* kNameFlagShort;
	const static char* kNameFlagLong;



private:

	MString		  m_name;
	MString		  m_selCamera = "";
	MString		  m_pclLoaderName = "";
	bool		  m_createPoints = false;
	bool		  m_loc = false;
	MString		  m_namePclSequencer;
	MString		  m_cameraName = "";
	MDagPath	  m_pathCam;
	MObject		  m_pclLoader_obj;

	//--------------------------------
	MString name_;
	double radius_;
	CommandMode command_;
	MSelectionList selectionList_;  //< Selected command input nodes. 
	MObject oWrapNode_;  //< MObject to the cvWrap node in focus. 
	MDGModifier dgMod_;

};


#endif