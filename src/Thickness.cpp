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

using namespace XSI;
using namespace XSI::MATH;

XSIPLUGINCALLBACK CStatus apply_snThickness_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create an instance of snThickness operator");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_snThickness_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CValueArray args = ctxt.GetAttribute(L"Arguments");

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

   // if we are missing either a mesh or the pointcloud, error out
   if( ! l_bHaveMesh )
   {
      Application().LogMessage(L"Please select one mesh!",XSI::siErrorMsg);
      return CStatus::Fail;
   }

   // create the operator
   CustomOperator newOp = Application().GetFactory().CreateObject(L"snThickness");
   newOp.AddOutputPort(l_pMesh);
   newOp.AddInputPort(l_pMesh);
   newOp.Connect();
   ctxt.PutAttribute( L"ReturnValue", newOp.GetRef() );

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snThickness_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CustomOperator oCustomOperator;
   Parameter oParam;
   CRef oPDef;
   Factory oFactory = Application().GetFactory();
   oCustomOperator = ctxt.GetSource();

   oPDef = oFactory.CreateParamDef(L"thickness",CValue::siFloat,siAnimatable|siPersistable,L"thickness",L"thickness",-.1,-1000,1000,-1,1);
   oCustomOperator.AddParameter(oPDef,oParam);
   oPDef = oFactory.CreateParamDef(L"shift",CValue::siFloat,siAnimatable|siPersistable,L"shift",L"shift",.5,0,1,0,1);
   oCustomOperator.AddParameter(oPDef,oParam);

   oCustomOperator.PutAlwaysEvaluate(false);
   oCustomOperator.PutDebug(0);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snThickness_Update( CRef& in_ctxt )
{
   OperatorContext ctxt( in_ctxt );
   PolygonMesh meshGeo = Primitive(ctxt.GetInputValue(0)).GetGeometry();

   // get all ice attributes
   float thickness = ctxt.GetParameterValue(L"thickness");
   float shift = ctxt.GetParameterValue(L"shift");

   CVertexRefArray vertices = meshGeo.GetVertices();
   CVector3Array pos;
   CLongArray poly;
   meshGeo.Get(pos,poly);
   CLongArray vertexMap(vertices.GetCount());

   bool isNormalValid;
   CVector3 normal;
   for(long i=0;i<vertices.GetCount();i++)
   {
      Vertex vertex(vertices[i]);
      if(vertex.GetNeighborPolygons().GetCount()==0)
      {
         vertexMap[i] = -1;
         continue;
      }

      normal = vertex.GetNormal(isNormalValid);
      if(!isNormalValid)
         normal.PutNull();

      vertexMap[i] = pos.GetCount();
      pos.Add(normal);
      pos[vertexMap[i]].ScaleAddInPlace(thickness * (1.0 - shift),pos[i]);
      normal.ScaleInPlace((-thickness) * shift);
      pos[i].AddInPlace(normal);
   }

   CLongArray outPoly(poly.GetCount()*2);

   long count = 0;
   long offset = poly.GetCount();
   long currentcount;
   long currentindex;
   for(long i=0;i<poly.GetCount();i++)
   {
      outPoly[i] = poly[i];
      if(count == 0)
      {
         count = poly[i];
         currentcount = count;
         currentindex = i;
         outPoly[offset+i] = count;
      }
      else
      {
         count--;
         outPoly[offset+i] = vertexMap[poly[2 * currentindex + 1 + currentcount - i]];
      }
   }

   // now we walk all edges and check if they have only one neighbor polygon!
   CEdgeRefArray edges(meshGeo.GetEdges());
   for(long i=0;i<edges.GetCount();i++)
   {
      Edge edge(edges[i]);
      if(edge.GetNeighborPolygons().GetCount() == 1)
      {
         CLongArray pointIndices = edge.GetVertices().GetIndexArray();
         outPoly.Add(4);
         outPoly.Add(pointIndices[1]);
         outPoly.Add(pointIndices[0]);
         outPoly.Add(vertexMap[pointIndices[0]]);
         outPoly.Add(vertexMap[pointIndices[1]]);
      }
   }

   PolygonMesh outMesh(Primitive(ctxt.GetOutputTarget()).GetGeometry());
   outMesh.Set(pos,outPoly);

   return CStatus::OK;
}
