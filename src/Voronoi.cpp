/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.
   
   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/lgpl.html>.
   
   Author:     Helge Mathee      helge.mathee@gmx.net
   Company:    Studio Nest (TM)
   Date:       2010 / 09 / 21
*/

#include <xsi_application.h>
#include <xsi_context.h>
#include <xsi_pluginregistrar.h>
#include <xsi_status.h>
#include <xsi_customoperator.h>
#include <xsi_operatorcontext.h>
#include <xsi_ppglayout.h>
#include <xsi_ppgeventcontext.h>
#include <xsi_selection.h>
#include <xsi_command.h>
#include <xsi_factory.h>
#include <xsi_primitive.h>
#include <xsi_kinematics.h>
#include <xsi_kinematicstate.h>
#include <xsi_outputport.h>
#include <xsi_polygonmesh.h>
#include <xsi_nurbscurvelist.h>
#include <xsi_nurbscurve.h>
#include <xsi_controlpoint.h>
#include <xsi_polygonface.h>
#include <xsi_edge.h>
#include <xsi_vertex.h>
#include <xsi_sample.h>
#include <xsi_triangle.h>
#include <xsi_trianglevertex.h>
#include <xsi_pointlocatordata.h>
#include <xsi_x3dobject.h>
#include <xsi_iceattribute.h>
#include <xsi_iceattributedataarray.h>
#include <xsi_iceattributedataarray2D.h>
#include <xsi_boolarray.h>
#include <xsi_model.h>
#include <xsi_icetree.h>
#include <xsi_icenodecontext.h>
#include <xsi_icenodedef.h>
#include <xsi_command.h>
#include <xsi_factory.h>
#include <xsi_math.h>
#include <xsi_vector2f.h>
#include <xsi_vector3f.h>
#include <xsi_vector4f.h>
#include <xsi_matrix3f.h>
#include <xsi_matrix4f.h>
#include <xsi_rotationf.h>
#include <xsi_quaternionf.h>
#include <xsi_color4f.h>
#include <xsi_shape.h>
#include <xsi_indexset.h>
#include <xsi_dataarray.h>
#include <xsi_dataarray2D.h>
#include <xsi_userdatablob.h>
#include <xsi_project.h>
#include <xsi_null.h>
#include <xsi_uitoolkit.h>
#include <xsi_progressbar.h>
#include <xsi_geometryaccessor.h>
#include <xsi_userdatablob.h>

#include <Essence/snTriangleMesh.h>
#include <Essence/snTimer.h>
#include <Essence/snString.h>
#include "snVoroMain.h"

using namespace XSI;
using namespace XSI::MATH;
using namespace snEssence;

// forward declarations
XSIPLUGINCALLBACK CStatus update_snVoronoi_Execute( CRef& in_ctxt );

XSIPLUGINCALLBACK CStatus apply_snVoronoi_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create an instance of snVoronoi operator");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_snVoronoi_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CValueArray args = ctxt.GetAttribute(L"Arguments");
   CValueArray cmdArgs;
   CValue returnVal;

   Selection l_pSelection = Application().GetSelection();
   CRef l_pMesh;
   bool l_bHaveMesh = false;

   // search the selection for a mesh and a pointcloud
   for(long i=0;i<l_pSelection.GetCount();i++)
   {
      X3DObject l_pSelObj(l_pSelection.GetItem(i));
      if(l_pSelObj.GetType().IsEqualNoCase(L"polymsh"))
      {
         l_pMesh = l_pSelObj.GetActivePrimitive().GetRef();
         l_bHaveMesh = true;
         break;
      }
   }

   // if we are missing the mesh or the pointcloud, error out
   if( ! l_bHaveMesh )
   {
      Application().LogMessage(L"Please select one polygonmesh!",XSI::siErrorMsg);
      return CStatus::Fail;
   }

   // check if this is already fractured... sometimes happens since the
   // button is too close...
   CRef voronoiRef;
   voronoiRef.Set(Primitive(l_pMesh).GetParent().GetAsText()+L".voronoiData");
   if(voronoiRef.IsValid())
   {
      Application().LogMessage(L"This object is already shattered. Please use 'Update Shatter' instead.",siWarningMsg);
      return CStatus::OK;
   }

   // let's check to ensure the mesh is closed and triangulated
   PolygonMesh geo(Primitive(l_pMesh).GetGeometry());
   CVertexRefArray points(geo.GetVertices());
   CPolygonFaceRefArray polies(geo.GetPolygons());
   for(LONG i=0;i<points.GetCount();i++)
   {
      Vertex point(points[i]);
      if(point.GetIsBoundary())
      {
         Application().LogMessage(L"Please ensure that the mesh is closed (no boundaries)!",XSI::siErrorMsg);
         return CStatus::Fail;
      }
   }

   // let's check that it has a zero transform
   X3DObject meshX3DObject(Primitive(l_pMesh).GetParent());
   CTransformation meshGlobal = meshX3DObject.GetKinematics().GetGlobal().GetTransform();
   if(meshGlobal.GetPosX() != 0.0 || meshGlobal.GetPosY() != 0.0 || meshGlobal.GetPosZ() != 0.0 ||
      meshGlobal.GetRotX() != 0.0 || meshGlobal.GetRotY() != 0.0 || meshGlobal.GetRotZ() != 0.0 ||
      meshGlobal.GetSclX() != 1.0 || meshGlobal.GetSclY() != 1.0 || meshGlobal.GetSclZ() != 1.0)
   {
      Application().LogMessage(L"Please ensure that the mesh has a zero transform (FreezeTransform)!",XSI::siErrorMsg);
      return CStatus::OK;
   }

   // let's create the output mesh!
   cmdArgs.Resize(1);
   cmdArgs[0] = L"EmptyPolygonMesh";
   Application().ExecuteCommand(L"SIGetPrim",cmdArgs,returnVal);
   CRef cellMesh = (CRef)((CValueArray&)returnVal)[0];

   // name the output mesh according to the source object
   X3DObject(Primitive(cellMesh).GetParent()).PutName(X3DObject(Primitive(l_pMesh).GetParent()).GetName()+L"_SingleCell");

   // we really need to create a binary user data blob!
   UserDataBlob myBlob ;
   X3DObject(Primitive(l_pMesh).GetParent()).AddProperty( L"UserDataBlob", false, L"voronoiData", myBlob ) ;

   // let's create the centers!
   cmdArgs.Resize(1);
   cmdArgs[0] = L"PointCloud";
   Application().ExecuteCommand(L"SIGetPrim",cmdArgs,returnVal);
   CRef outputCloud = (CRef)((CValueArray&)returnVal)[0];

   // name the output mesh according to the source object
   X3DObject(Primitive(outputCloud).GetParent()).PutName(X3DObject(Primitive(l_pMesh).GetParent()).GetName()+L"_Particle_Centers");

   // apply the ice op for create the centers!
   cmdArgs.Resize(4);
   cmdArgs[0] = L"Emit for VoronoiShatter";
   cmdArgs[1] = Primitive(outputCloud).GetParent().GetAsText();
   Application().ExecuteCommand(L"ApplyICEOp",cmdArgs,returnVal);
   CRef iceTree = (CRef)returnVal;

   // add the get data node!
   cmdArgs.Resize(2);
   cmdArgs[0] = L"GetDataNode";
   cmdArgs[1] = iceTree.GetAsText();
   Application().ExecuteCommand(L"AddICENode",cmdArgs,returnVal);

   // connect the get data node
   cmdArgs.Resize(2);
   cmdArgs[0] = iceTree.GetAsText()+L".Emit_for_VoronoiShatter.Geometry_Name";
   cmdArgs[1] = iceTree.GetAsText()+L".SceneReferenceNode.outname";
   Application().ExecuteCommand(L"ConnectICENodes",cmdArgs,returnVal);

   // set the reference on the get data node
   cmdArgs.Resize(2);
   cmdArgs[0] = iceTree.GetAsText()+L".SceneReferenceNode.reference";
   cmdArgs[1] = Primitive(l_pMesh).GetParent().GetAsText();
   Application().ExecuteCommand(L"SetValue",cmdArgs,returnVal);

   // create the operator for the udb
   CustomOperator newOp = Application().GetFactory().CreateObject(L"snVoronoi");
   newOp.AddOutputPort(myBlob,L"outBlob");
   newOp.AddInputPort(outputCloud);
   newOp.AddInputPort(l_pMesh);
   newOp.Connect();

   // create the operator for the mesh
   newOp = Application().GetFactory().CreateObject(L"snVoronoiCell");
   newOp.AddOutputPort(cellMesh,L"outMesh");
   newOp.AddInputPort(myBlob);
   newOp.Connect();

   // cool now let's bool this!
   cmdArgs.Resize(6);
   cmdArgs[0] = L"BooleanGenIntersection";
   cmdArgs[1] = L"";
   cmdArgs[2] = Primitive(cellMesh).GetParent3DObject().GetFullName()+L";"+Primitive(l_pMesh).GetParent3DObject().GetFullName();
   cmdArgs[3] = (LONG)3;
   cmdArgs[4] = siPersistentOperation;
   cmdArgs[5] = siKeepGenOpInputs;
   Application().ExecuteCommand(L"ApplyGenOp",cmdArgs,returnVal);

   // we will have the boolean now in the selection
   X3DObject boolMesh(Application().GetSelection().GetItem(0));
   boolMesh.PutName(X3DObject(Primitive(l_pMesh).GetParent()).GetName()+L"_Booler");

   // let's create the final mesh!
   cmdArgs.Resize(1);
   cmdArgs[0] = L"EmptyPolygonMesh";
   Application().ExecuteCommand(L"SIGetPrim",cmdArgs,returnVal);
   CRef finalMesh = (CRef)((CValueArray&)returnVal)[0];

   // add also a user data blob to the fractured mesh
   X3DObject(Primitive(finalMesh).GetParent()).AddProperty( L"UserDataBlob", false, L"voronoiData", myBlob ) ;

   // name the output mesh according to the source object
   X3DObject(Primitive(finalMesh).GetParent()).PutName(X3DObject(Primitive(l_pMesh).GetParent()).GetName()+L"_Fractured");

   // hide the booler and the single cell
   cmdArgs.Resize(1);
   cmdArgs[0] = Primitive(l_pMesh).GetParent().GetAsText()+L","+boolMesh.GetFullName()+L","+Primitive(cellMesh).GetParent().GetAsText();
   Application().ExecuteCommand(L"ToggleVisibility",cmdArgs,returnVal);

   // select the final mesh
   Application().GetSelection().Clear();
   Application().GetSelection().Add(Primitive(finalMesh).GetParent());

   // check if we want to update
   cmdArgs.Resize(5);
   cmdArgs[0] = iceTree.GetAsText()+L".Emit_for_VoronoiShatter";
   cmdArgs[1] = L"";
   cmdArgs[2] = L"Choose Shatter Settings";
   cmdArgs[3] = siModal;
   cmdArgs[4] = false;
   Application().ExecuteCommand(L"InspectObj",cmdArgs,returnVal);

   if((bool)returnVal == false)
   {
      // run the update
      update_snVoronoi_Execute(in_ctxt);
   }

   // now connect the mesh operator...!
   newOp = Application().GetFactory().CreateObject(L"snVoronoiMesh");
   newOp.AddOutputPort(finalMesh,L"outMesh");
   newOp.AddInputPort(myBlob);
   newOp.Connect();

   ctxt.PutAttribute( L"ReturnValue", newOp.GetRef() );
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus update_snVoronoi_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Update the result of the snVoronoi operator");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus update_snVoronoi_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CValueArray args = ctxt.GetAttribute(L"Arguments");
   CValueArray cmdArgs;
   CValue returnVal;

   Selection l_pSelection = Application().GetSelection();
   X3DObject meshFractured;
   snEssence::snString baseName;
   bool l_bHaveMesh = false;

   // search the selection for a mesh and a pointcloud
   for(long i=0;i<l_pSelection.GetCount();i++)
   {
      X3DObject l_pSelObj(l_pSelection.GetItem(i));
      if(l_pSelObj.GetType().IsEqualNoCase(L"polymsh"))
      {
         // now check if the name ends with "fractured"
         snEssence::snString meshName(l_pSelObj.GetName().GetAsciiString());
         if(meshName.GetLowerCase().EndsWith("_fractured"))
         {
            meshName.TruncateRight(10);
            baseName = meshName;
            meshFractured = l_pSelObj;
            l_bHaveMesh = true;
            break;
         }
      }
   }

   // check if we have the fractured mesh
   if(!l_bHaveMesh)
   {
      Application().LogMessage(L"Please select the fractured mesh!",siErrorMsg);
      return CStatus::OK;
   }

   CRef test;

   // get the current frame
   test.Set(L"PlayControl.Current");
   LONG currentFrame = Parameter(test).GetValue();

   // try to find the user data blob
   test.Set(CString(baseName.GetAsWChar())+L"_Fractured.voronoiData");
   if(!test.IsValid())
   {
      Application().LogMessage(L"Cannot find the userdatablob on the fractured mesh!",siErrorMsg);
      return CStatus::OK;
   }
   UserDataBlob udb1(test);

   // try to find the base mesh
   test.Set(CString(baseName.GetAsWChar()));
   if(!test.IsValid())
   {
      Application().LogMessage(L"Cannot find the corresponding base mesh!",siErrorMsg);
      return CStatus::OK;
   }
   X3DObject meshBase(test);

   // try to find the user data blob
   test.Set(CString(baseName.GetAsWChar())+L".voronoiData");
   if(!test.IsValid())
   {
      Application().LogMessage(L"Cannot find the userdatablob on the base mesh!",siErrorMsg);
      return CStatus::OK;
   }
   UserDataBlob udb2(test);

   // try to find the single cell mesh
   test.Set(CString(baseName.GetAsWChar())+L"_SingleCell");
   if(!test.IsValid())
   {
      Application().LogMessage(L"Cannot find the corresponding singlecell mesh!",siErrorMsg);
      return CStatus::OK;
   }
   X3DObject meshSingleCell(test);

   // try to find the voronoi op
   test.Set(meshSingleCell.GetFullName()+L".polymsh.snVoronoiCell");
   if(!test.IsValid())
   {
      Application().LogMessage(L"The single cell mesh doesn't have a voronoi operator applied!",siErrorMsg);
      return CStatus::OK;
   }
   CustomOperator voronoiOp(test);

   // try to find the boolean mesh
   test.Set(CString(baseName.GetAsWChar())+L"_Booler");
   if(!test.IsValid())
   {
      Application().LogMessage(L"Cannot find the corresponding booler mesh!",siErrorMsg);
      return CStatus::OK;
   }
   X3DObject meshBooler(test);

   // get the levels of undo
   test.Set("preferences.General.undo");
   Parameter undoParam(test);
   LONG currentUndos = undoParam.GetValue();
   undoParam.PutValue(0);

   // access the userdatablob to find out how many cells we have
   const unsigned char * buffer;
   unsigned int bufferSize;
   udb2.GetValue(buffer,bufferSize);
   if(bufferSize==0)
      return CStatus::Unexpected;

   // create a voronoi info to init it
   VoronoiInfo info;
   info.SetFromBuffer(buffer,bufferSize);
   LONG cellCount = (LONG)info.points.size();

   // show the cut piece
   cmdArgs.Resize(1);
   cmdArgs[0] = meshSingleCell.GetFullName();
   Application().ExecuteCommand(L"ToggleVisibility",cmdArgs,returnVal);

   // now we have everything we need
   ProgressBar prog = Application().GetUIToolkit().GetProgressBar();
   prog.PutMinimum(0);
   prog.PutMaximum(cellCount);
   prog.PutVisible(true);
   prog.PutCaption(L"Creating volume shatter elements...");
   prog.PutCancelEnabled(true);

   // prepare the command args for a refresh!
   cmdArgs.Resize(1);
   cmdArgs[0] = currentFrame;

   snEssence::snMesh mergedMesh;

   for(LONG cellIndex=0; cellIndex < cellCount; cellIndex++)
   {
      // check if we need to abort
      if(prog.IsCancelPressed())
         return CStatus::Abort;

      // set the index on the custom op
      voronoiOp.PutParameterValue(L"cellIndex",cellIndex);

      // refresh the scene
      Application().ExecuteCommand(L"Refresh",cmdArgs,returnVal);

      Application().LogMessage(L"Computing Voronoi Cell "+CValue(cellIndex).GetAsText()+L"...",siVerboseMsg);

      // now gather the mesh from the boolean
      Primitive primBool = meshBooler.GetActivePrimitive();
      PolygonMesh mesh = primBool.GetGeometry();
      CVector3Array pos;
      CLongArray polies;
      mesh.Get(pos,polies);

      Application().LogMessage(L"Received booled cell "+CValue(cellIndex).GetAsText()+L"...",siVerboseMsg);

      // skip empty meshes...
      if(pos.GetCount() == 0 || polies.GetCount() == 0)
      {
         Application().LogMessage(L"Skipped empty cell "+CValue(cellIndex).GetAsText()+L".",siVerboseMsg);
         continue;
      }

      // create an essence mesh to merge this one
      snEssence::snMesh singleMesh;

      for(LONG i=0;i<pos.GetCount();i++)
         singleMesh.AddPoint(snEssence::snVector3f((float)pos[i].GetX(),(float)pos[i].GetY(),(float)pos[i].GetZ()));

      Application().LogMessage(L"Copied points for cell "+CValue(cellIndex).GetAsText()+L"...",siVerboseMsg);

      LONG count = 0;
      snEssence::snPolygon * poly = NULL;
      for(LONG i=0;i<polies.GetCount();i++)
      {
         if(count == 0)
         {
            count = polies[i];
            poly = singleMesh.AddPolygon();
         }
         else
         {
            poly->AddPointByIndex((size_t)polies[i]);
            count--;
         }
      }

      Application().LogMessage(L"Copied polygons for cell "+CValue(cellIndex).GetAsText()+L"...",siVerboseMsg);

      // merge the mesh
      mergedMesh.Merge(&singleMesh);
      Application().LogMessage(L"Merged booled Cell "+CValue(cellIndex).GetAsText()+L".",siVerboseMsg);

      prog.Increment();
   }

   prog.PutVisible(false);

   Application().LogMessage(L"Done. All Cells computed.",siVerboseMsg);

   // we have all the data, output it
   VoronoiInfo outInfo;
   outInfo.points.push_back(mergedMesh.GetPoints());
   outInfo.polies.push_back(mergedMesh.GetPointIndicesCombined());

   // set the result on the fractured mesh!
   unsigned char * outBuffer;
   size_t size = outInfo.GetAsBuffer(&outBuffer);

   // save the buffer
   udb1.PutValue(outBuffer,size);

   // free the memory
   free(outBuffer);

   Application().LogMessage(L"Data stored in userdatablob. Freezing now...",siVerboseMsg);

   // Freeze the udb
   cmdArgs.Resize(1);
   cmdArgs[0] = udb1.GetFullName();
   Application().ExecuteCommand(L"FreezeObj",cmdArgs,returnVal);

   Application().LogMessage(L"Voronoi update finished.",siVerboseMsg);

   // show the cut piece
   cmdArgs.Resize(1);
   cmdArgs[0] = meshSingleCell.GetFullName();
   Application().ExecuteCommand(L"ToggleVisibility",cmdArgs,returnVal);

   // restore the undo setting
   undoParam.PutValue(currentUndos);

   ctxt.PutAttribute( L"ReturnValue", false );
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snVoronoi_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CustomOperator oCustomOperator;
   Parameter oParam;
   CRef oPDef;
   Factory oFactory = Application().GetFactory();
   oCustomOperator = ctxt.GetSource();

   oCustomOperator.PutAlwaysEvaluate(false);
   oCustomOperator.PutDebug(0);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snVoronoi_DefineLayout( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   PPGLayout oLayout;
   PPGItem oItem;
   oLayout = ctxt.GetSource();
   oLayout.Clear();
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snVoronoi_Update( CRef& in_ctxt )
{
   OperatorContext ctxt( in_ctxt );

   bool mDebug = true;

   // get the positions of all particles
   CVector3Array pointPos = Primitive(ctxt.GetInputValue(0)).GetGeometry().GetPoints().GetPositionArray();
   PolygonMesh mesh(Primitive(ctxt.GetInputValue(1)).GetGeometry());
   CVector3Array meshPos = mesh.GetPoints().GetPositionArray();

   // create the bounding box
   snBboxf bbox;
   for(LONG i=0;i<meshPos.GetCount();i++)
   {
      snVector3f pos((float)meshPos[i].GetX(),(float)meshPos[i].GetY(),(float)meshPos[i].GetZ());
      bbox.Merge(pos);
   }

   // create the container
   float tol = 0.1;
   container con(
      bbox.GetMin().GetX()-tol,bbox.GetMax().GetX()+tol,
      bbox.GetMin().GetY()-tol,bbox.GetMax().GetY()+tol,
      bbox.GetMin().GetZ()-tol,bbox.GetMax().GetZ()+tol,
      8,8,8, /* subdivisions... no idea? */
      false,false,false, /* periodic... no idea? */
      8 /* max 8 particles per cell */
      );

   // store all particles
   for(LONG i=0;i<pointPos.GetCount();i++)
      con.put(i,(float)pointPos[i].GetX(),(float)pointPos[i].GetY(),(float)pointPos[i].GetZ());

   // now compute the cells and get the data!
   snpTriangleMeshVec cells;
   con.draw_cells_snTriangleMesh(&cells);

   // loop over all cells
   VoronoiInfo info;
   for(size_t i=0;i<cells.size();i++)
   {
      info.points.push_back(cells[i]->GetPoints());
      info.polies.push_back(cells[i]->GetPointIndicesCombined());
   }

   // convert it into a buffer
   unsigned char * buffer;
   size_t size = info.GetAsBuffer(&buffer);

   // save the buffer
   UserDataBlob udb(ctxt.GetOutputTarget());
   udb.PutValue(buffer,size);

   // free the memory
   free(buffer);
   for(snIndex i=0;i<cells.size();i++)
      delete(cells[i]);

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snVoronoiCell_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CustomOperator oCustomOperator;
   Parameter oParam;
   CRef oPDef;
   Factory oFactory = Application().GetFactory();
   oCustomOperator = ctxt.GetSource();

   oPDef = oFactory.CreateParamDef(L"cellIndex",CValue::siInt4,siAnimatable | siPersistable,L"cellIndex",L"cellIndex",0,0,1000000,0,100);
   oCustomOperator.AddParameter(oPDef,oParam);

   oCustomOperator.PutAlwaysEvaluate(false);
   oCustomOperator.PutDebug(0);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snVoronoiCell_DefineLayout( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   PPGLayout oLayout;
   PPGItem oItem;
   oLayout = ctxt.GetSource();
   oLayout.Clear();

   oLayout.AddItem(L"cellIndex",L"Cell Index");

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snVoronoiCell_Update( CRef& in_ctxt )
{
   OperatorContext ctxt( in_ctxt );
   size_t cellIndex = (size_t)(LONG)ctxt.GetParameterValue(L"cellIndex");

   UserDataBlob udb(ctxt.GetInputValue(0));

   // access the buffer
   const unsigned char * buffer;
   unsigned int bufferSize;
   udb.GetValue(buffer,bufferSize);
   if(bufferSize==0)
      return CStatus::Unexpected;

   // create a voronoi info
   VoronoiInfo info;
   info.SetFromBuffer(buffer,bufferSize);

   // check if we know about that cell...!?
   if(info.points.size() <= cellIndex || info.polies.size() <= cellIndex)
   {
      // the given cell is out of range
      Application().LogMessage(L"snVoronoi: The cellIndex of "+CValue((LONG)cellIndex).GetAsText()+L" is out of range!",siErrorMsg);
      return CStatus::OK;
   }

   // allocate enough space
   CVector3Array pos(info.points[cellIndex].size());
   CLongArray poly(info.polies[cellIndex].size());

   // copy the position data
   for(size_t i=0;i<info.points[cellIndex].size();i++)
      pos[i].Set(info.points[cellIndex][i].GetX(),info.points[cellIndex][i].GetY(),info.points[cellIndex][i].GetZ());

   // copy the polygon data
   for(size_t i=0;i<info.polies[cellIndex].size();i++)
      poly[i] = info.polies[cellIndex][i];

   PolygonMesh outMesh = Primitive(ctxt.GetOutputTarget()).GetGeometry();
   outMesh.Set(pos,poly);

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snVoronoiMesh_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CustomOperator oCustomOperator;
   Parameter oParam;
   CRef oPDef;
   Factory oFactory = Application().GetFactory();
   oCustomOperator = ctxt.GetSource();
   oCustomOperator.PutAlwaysEvaluate(false);
   oCustomOperator.PutDebug(0);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snVoronoiMesh_DefineLayout( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   PPGLayout oLayout;
   PPGItem oItem;
   oLayout = ctxt.GetSource();
   oLayout.Clear();

   oLayout.AddItem(L"cellIndex",L"Cell Index");

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snVoronoiMesh_Update( CRef& in_ctxt )
{
   OperatorContext ctxt( in_ctxt );
   UserDataBlob udb(ctxt.GetInputValue(0));

   // access the buffer
   const unsigned char * buffer;
   unsigned int bufferSize;
   udb.GetValue(buffer,bufferSize);
   if(bufferSize==0)
      return CStatus::Unexpected;

   // create a voronoi info
   VoronoiInfo info;
   info.SetFromBuffer(buffer,bufferSize);

   // check if we know about that cell...!?
   if(info.points.size() != 1 || info.polies.size() != 1)
   {
      // the given cell is out of range
      Application().LogMessage(L"snVoronoi: User data blob does not contain a mesh.",siErrorMsg);
      return CStatus::OK;
   }

   // allocate enough space
   CVector3Array pos(info.points[0].size());
   CLongArray poly(info.polies[0].size());

   // copy the position data
   for(size_t i=0;i<info.points[0].size();i++)
      pos[i].Set(info.points[0][i].GetX(),info.points[0][i].GetY(),info.points[0][i].GetZ());

   // copy the polygon data
   for(size_t i=0;i<info.polies[0].size();i++)
      poly[i] = info.polies[0][i];

   PolygonMesh outMesh = Primitive(ctxt.GetOutputTarget()).GetGeometry();
   outMesh.Set(pos,poly);

   return CStatus::OK;
}
