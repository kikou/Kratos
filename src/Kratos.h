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

#ifndef __SN_KRATOS__
#define __SN_KRATOS__

#include <vector>
#include <map>

#include <Essence/snPolygon.h>

struct VoronoiInfo
{
   snEssence::snVector3fVecVec points;
	snEssence::snIndexVecVec polies;

	size_t GetFloatCount()
	{
	   size_t count = 2 + points.size() + polies.size(); // for the count of points and polies vec
	   for(size_t i=0;i<points.size();i++)
         count += points[i].size() * 3;
	   for(size_t i=0;i<polies.size();i++)
         count += polies[i].size();
	   return count;
	}

	size_t GetAsBuffer(unsigned char ** in_pBuffer)
	{
      size_t floatCount = GetFloatCount();
	   *in_pBuffer = (unsigned char*)malloc(floatCount * sizeof(float));
	   float * floats = (float*)*in_pBuffer;

	   size_t offset = 0;
	   floats[offset++] = (float)points.size();
	   floats[offset++] = (float)polies.size();
	   for(size_t i=0;i<points.size();i++)
         floats[offset++] = (float)points[i].size();
	   for(size_t i=0;i<polies.size();i++)
         floats[offset++] = (float)polies[i].size();

	   // copy all points
	   for(size_t i=0;i<points.size();i++)
	   {
         for(size_t j=0;j<points[i].size();j++)
         {
            floats[offset++] = points[i][j].GetX();
            floats[offset++] = points[i][j].GetY();
            floats[offset++] = points[i][j].GetZ();
         }
	   }

      // copy all poly info
	   for(size_t i=0;i<polies.size();i++)
	   {
         for(size_t j=0;j<polies[i].size();j++)
            floats[offset++] = (float)polies[i][j];
	   }

	   return floatCount * sizeof(float);
	}

	bool SetFromBuffer(const unsigned char * in_pBuffer, size_t in_Size)
	{
	   float * floats = (float*)in_pBuffer;
	   size_t offset=0;

      points.resize((size_t)floats[offset++],snEssence::snVector3fVec());
      polies.resize((size_t)floats[offset++],snEssence::snIndexVec());

      // resize points for every cell
      for(size_t i=0;i<points.size();i++)
         points[i].resize((size_t)floats[offset++]);
      // resize polies for every cell
      for(size_t i=0;i<polies.size();i++)
         polies[i].resize((size_t)floats[offset++]);

      // check the buffer size!
	   if(GetFloatCount()*sizeof(float) != in_Size)
         return false;

      // read the points
      for(size_t i=0;i<points.size();i++)
      {
         for(size_t j=0;j<points[i].size();j++)
         {
            points[i][j].SetX(floats[offset++]);
            points[i][j].SetY(floats[offset++]);
            points[i][j].SetZ(floats[offset++]);
         }
      }

      // read the polies
      for(size_t i=0;i<polies.size();i++)
      {
         for(size_t j=0;j<polies[i].size();j++)
            polies[i][j] = (size_t)floats[offset++];
      }

      return true;
	}
};


#endif
