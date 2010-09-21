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
#include <xsi_selection.h>
#include <xsi_command.h>
#include <xsi_argument.h>
#include <xsi_factory.h>
#include <xsi_primitive.h>
#include <xsi_polygonmesh.h>
#include <xsi_polygonface.h>
#include <xsi_edge.h>
#include <xsi_vertex.h>
#include <xsi_sample.h>
#include <xsi_x3dobject.h>
#include <xsi_model.h>
#include <xsi_math.h>
#include <xsi_customoperator.h>
#include <xsi_operatorcontext.h>

using namespace XSI;
using namespace XSI::MATH;

XSIPLUGINCALLBACK CStatus apply_snUniquePoints_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create an instance of snUniquePoints operator");
   ArgumentArray args = oCmd.GetArguments();
   args.Add(L"createICETree",false);
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_snUniquePoints_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CValueArray args = ctxt.GetAttribute(L"Arguments");
   bool createICETree = args[0];

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

   CValueArray cmdArgs;
   CValue returnVal;

   CRef outputMesh;
   if(createICETree)
   {
      // duplicate an empty polygonmesh!
      cmdArgs.Resize(1);
      cmdArgs[0] = Primitive(l_pMesh).GetParent().GetAsText();
      Application().ExecuteCommand(L"Clone",cmdArgs,returnVal);
      outputMesh = X3DObject(Application().GetSelection().GetItem(0)).GetActivePrimitive().GetRef();

      // name the output mesh according to the source object
      X3DObject(Primitive(outputMesh).GetParent()).PutName(X3DObject(Primitive(l_pMesh).GetParent()).GetName()+L"_UniquePoints");
   }
   else
      outputMesh = l_pMesh;

   // create the operator
   CustomOperator newOp = Application().GetFactory().CreateObject(L"snUniquePoints");
   newOp.AddOutputPort(outputMesh);
   newOp.AddInputPort(outputMesh);
   newOp.Connect();
   ctxt.PutAttribute( L"ReturnValue", newOp.GetRef() );

   if(createICETree)
   {
      // great, now create the particles
      cmdArgs.Resize(1);
      cmdArgs[0] = L"PointCloud";
      Application().ExecuteCommand(L"SIGetPrim",cmdArgs,returnVal);
      CRef outputCloud = (CRef)((CValueArray&)returnVal)[0];

      // name the output mesh according to the source object
      X3DObject(Primitive(outputCloud).GetParent()).PutName(X3DObject(Primitive(l_pMesh).GetParent()).GetName()+L"_Particle_Centers");

      // apply the ice op for create the centers!
      cmdArgs.Resize(4);
      cmdArgs[0] = L"Emit for PolygonShatter";
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
      cmdArgs[0] = iceTree.GetAsText()+L".Emit_for_PolygonShatter.Geometry_Name";
      cmdArgs[1] = iceTree.GetAsText()+L".SceneReferenceNode.outname";
      Application().ExecuteCommand(L"ConnectICENodes",cmdArgs,returnVal);

      // set the reference on the get data node
      cmdArgs.Resize(2);
      cmdArgs[0] = iceTree.GetAsText()+L".SceneReferenceNode.reference";
      cmdArgs[1] = Primitive(l_pMesh).GetParent().GetAsText();
      Application().ExecuteCommand(L"SetValue",cmdArgs,returnVal);

      // apply the ice op for create the centers!
      cmdArgs.Resize(4);
      cmdArgs[0] = L"Project Polygon To Particle";
      cmdArgs[1] = Primitive(outputMesh).GetParent().GetAsText();
      Application().ExecuteCommand(L"ApplyICEOp",cmdArgs,returnVal);
      iceTree = (CRef)returnVal;

      // add the get data node!
      cmdArgs.Resize(2);
      cmdArgs[0] = L"GetDataNode";
      cmdArgs[1] = iceTree.GetAsText();
      Application().ExecuteCommand(L"AddICENode",cmdArgs,returnVal);

      // connect the get data node
      cmdArgs.Resize(2);
      cmdArgs[0] = iceTree.GetAsText()+L".Project_Polygon_To_Particle.Cloud_Name";
      cmdArgs[1] = iceTree.GetAsText()+L".SceneReferenceNode.outname";
      Application().ExecuteCommand(L"ConnectICENodes",cmdArgs,returnVal);

      // set the reference on the get data node
      cmdArgs.Resize(2);
      cmdArgs[0] = iceTree.GetAsText()+L".SceneReferenceNode.reference";
      cmdArgs[1] = Primitive(outputCloud).GetParent().GetAsText();
      Application().ExecuteCommand(L"SetValue",cmdArgs,returnVal);
   }
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus snUniquePoints_Define( CRef& in_ctxt )
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

SICALLBACK snUniquePoints_Update( CRef& in_ctxt )
{
   OperatorContext ctxt( in_ctxt );
   Primitive meshPrimitive = (CRef)ctxt.GetInputValue(0);
   PolygonMesh mesh(meshPrimitive.GetGeometry());
   LONG sampleCount = mesh.GetSamples().GetCount();

   // get the current polygon descriptions
   CVector3Array currentPoints;
   CLongArray polyIndices;
   mesh.Get(currentPoints,polyIndices);

   // prepare the output points
   CVector3Array finalPoints(sampleCount);

   // walk the polygon indices and retarget
   LONG count = 0;
   LONG sampleIndex = 0;
   for(LONG i=0;i<polyIndices.GetCount();i++)
   {
      if(count == 0)
      {
         count = polyIndices[i];
      }
      else
      {
         LONG index = polyIndices[i];
         finalPoints[sampleIndex] = currentPoints[index];
         polyIndices[i] = sampleIndex;
         sampleIndex++;
         count--;
      }
   }

   // output the results
   Primitive output = ctxt.GetOutputTarget();
   PolygonMesh outputMesh(output.GetGeometry());
   outputMesh.Set(finalPoints,polyIndices);
   return CStatus::OK;
}
