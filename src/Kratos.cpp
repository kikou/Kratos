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

using namespace XSI;
using namespace XSI::MATH;

XSIPLUGINCALLBACK CStatus XSILoadPlugin( PluginRegistrar& in_reg )
{
   in_reg.PutAuthor(L"hmathee");
   in_reg.PutName(L"Kratos_Plugin");
   in_reg.PutEmail(L"");
   in_reg.PutURL(L"");
   in_reg.PutVersion(1,0);
   in_reg.RegisterOperator(L"snUniquePoints");
   in_reg.RegisterOperator(L"snThickness");
   in_reg.RegisterOperator(L"snVoronoi");
   in_reg.RegisterOperator(L"snVoronoiCell");
   in_reg.RegisterOperator(L"snVoronoiMesh");
   in_reg.RegisterCommand(L"apply_snUniquePoints",L"apply_snUniquePoints");
   in_reg.RegisterCommand(L"apply_snThickness",L"apply_snThickness");
   in_reg.RegisterCommand(L"apply_snVoronoi",L"apply_snVoronoi");
   in_reg.RegisterCommand(L"update_snVoronoi",L"update_snVoronoi");
   in_reg.RegisterCommand(L"split_polygon_islands",L"split_polygon_islands");

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus XSIUnloadPlugin( const PluginRegistrar& in_reg )
{
   CString strPluginName;
   strPluginName = in_reg.GetName();
   Application().LogMessage(strPluginName + L" has been unloaded.",siVerboseMsg);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus split_polygon_islands_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Split evey single polygon island!");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus split_polygon_islands_Execute( CRef& in_ctxt )
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

   // great, let's get the geo
   X3DObject parent(Primitive(l_pMesh).GetParent());
   PolygonMesh mesh(Primitive(l_pMesh).GetGeometry());
   CVector3Array meshPos = mesh.GetPoints().GetPositionArray();
   CPolygonFaceRefArray meshPolies = mesh.GetPolygons();
   CLongArray meshPolyDone(meshPolies.GetCount());
   for(LONG i=0;i<meshPolyDone.GetCount();i++)
      meshPolyDone[i] = -1;

   // cool, so let's do this!
   LONG meshPolyOffset = 0;
   while(meshPolyOffset < meshPolies.GetCount())
   {
      // try to find the first unused polygon
      while(meshPolyDone[meshPolyOffset]==1 && meshPolyOffset < meshPolies.GetCount())
         meshPolyOffset++;

      // create a mapping for the points
      CLongArray pntIndex(meshPos.GetCount());
      for(LONG i=0;i<pntIndex.GetCount();i++)
         pntIndex[i] = -1;

      // great now let's add it!
      CLongArray currentPolies;
      if(meshPolyOffset < meshPolies.GetCount())
         currentPolies.Add(meshPolyOffset);

      // ok, let's prepare output data
      CVector3Array newPos;
      CLongArray newPoly;

      LONG pieceCount = 0;
      for(LONG i=0;i<currentPolies.GetCount();i++)
      {
         // first, add this one
         PolygonFace poly(meshPolies[currentPolies[i]]);
         CLongArray indices = poly.GetPoints().GetIndexArray();
         newPoly.Add(indices.GetCount());

         // loop over all points of this!
         for(LONG j=0;j<indices.GetCount();j++)
         {
            // check if we added the point!
            if(pntIndex[indices[j]]==-1)
            {
               pntIndex[indices[j]] = newPos.GetCount();
               newPos.Add(meshPos[indices[j]]);
            }
            // add the point!
            newPoly.Add(pntIndex[indices[j]]);
         }

         // remember this as finished!
         meshPolyDone[currentPolies[i]] = 1;

         // now get the neighbors!
         CLongArray neighbors = poly.GetNeighborPolygons().GetIndexArray();
         for(LONG j=0;j<neighbors.GetCount();j++)
         {
            if(meshPolyDone[neighbors[j]]==-1)
            {
               currentPolies.Add(neighbors[j]);
               // remember as processed, but not finished
               meshPolyDone[neighbors[j]] = 0;
            }
         }
      }

      if(newPos.GetCount() > 0)
      {
         X3DObject piece;
         parent.AddPolygonMesh(newPos,newPoly,parent.GetName()+L"_piece_"+CValue(pieceCount).GetAsText(),piece);
         pieceCount++;
      }
   }


   return CStatus::OK;
}

