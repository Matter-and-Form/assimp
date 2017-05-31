/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2016, assimp team
All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/

/** @file  STEPUtil.cpp
 *  @brief Implementation of conversion routines for some common STEP helper entities.
 */



#ifndef ASSIMP_BUILD_NO_STEP_IMPORTER

#include "STEPUtil.h"
#include "PolyTools.h"
#include "ProcessHelper.h"
#include "Defines.h"

namespace Assimp {
    namespace STEP {

// ------------------------------------------------------------------------------------------------
void TempOpening::Transform(const STEPMatrix4& mat)
{
    if(profileMesh) {
        profileMesh->Transform(mat);
    }
    if(profileMesh2D) {
        profileMesh2D->Transform(mat);
    }
    extrusionDir *= STEPMatrix3(mat);
}

// ------------------------------------------------------------------------------------------------
aiMesh* TempMesh::ToMesh()
{
    ai_assert(verts.size() == std::accumulate(vertcnt.begin(),vertcnt.end(),size_t(0)));

    if (verts.empty()) {
        return NULL;
    }

    std::unique_ptr<aiMesh> mesh(new aiMesh());

    // copy vertices
    mesh->mNumVertices = static_cast<unsigned int>(verts.size());
    mesh->mVertices = new aiVector3D[mesh->mNumVertices];
    std::copy(verts.begin(),verts.end(),mesh->mVertices);

    // and build up faces
    mesh->mNumFaces = static_cast<unsigned int>(vertcnt.size());
    mesh->mFaces = new aiFace[mesh->mNumFaces];

    for(unsigned int i = 0,n=0, acc = 0; i < mesh->mNumFaces; ++n) {
        aiFace& f = mesh->mFaces[i];
        if (!vertcnt[n]) {
            --mesh->mNumFaces;
            continue;
        }

        f.mNumIndices = vertcnt[n];
        f.mIndices = new unsigned int[f.mNumIndices];
        for(unsigned int a = 0; a < f.mNumIndices; ++a) {
            f.mIndices[a] = acc++;
        }

        ++i;
    }

    return mesh.release();
}

// ------------------------------------------------------------------------------------------------
void TempMesh::Clear()
{
    verts.clear();
    vertcnt.clear();
}

// ------------------------------------------------------------------------------------------------
void TempMesh::Transform(const STEPMatrix4& mat)
{
    for(STEPVector3& v : verts) {
        v *= mat;
    }
}

// ------------------------------------------------------------------------------
STEPVector3 TempMesh::Center() const
{
    return (verts.size() == 0) ? STEPVector3(0.0f, 0.0f, 0.0f) : (std::accumulate(verts.begin(),verts.end(),STEPVector3()) / static_cast<STEPFloat>(verts.size()));
}

// ------------------------------------------------------------------------------------------------
void TempMesh::Append(const TempMesh& other)
{
    verts.insert(verts.end(),other.verts.begin(),other.verts.end());
    vertcnt.insert(vertcnt.end(),other.vertcnt.begin(),other.vertcnt.end());
}

// ------------------------------------------------------------------------------------------------
void TempMesh::RemoveDegenerates()
{
    // The strategy is simple: walk the mesh and compute normals using
    // Newell's algorithm. The length of the normals gives the area
    // of the polygons, which is close to zero for lines.

    std::vector<STEPVector3> normals;
    ComputePolygonNormals(normals, false);

    bool drop = false;
    size_t inor = 0;

    std::vector<STEPVector3>::iterator vit = verts.begin();
    for (std::vector<unsigned int>::iterator it = vertcnt.begin(); it != vertcnt.end(); ++inor) {
        const unsigned int pcount = *it;

        if (normals[inor].SquareLength() < 1e-10f) {
            it = vertcnt.erase(it);
            vit = verts.erase(vit, vit + pcount);

            drop = true;
            continue;
        }

        vit += pcount;
        ++it;
    }

    if(drop) {
        STEPImporter::LogDebug("removing degenerate faces");
    }
}

// ------------------------------------------------------------------------------------------------
STEPVector3 TempMesh::ComputePolygonNormal(const STEPVector3* vtcs, size_t cnt, bool normalize)
{
    std::vector<STEPFloat> temp((cnt+2)*3);
    for( size_t vofs = 0, i = 0; vofs < cnt; ++vofs )
    {
        const STEPVector3& v = vtcs[vofs];
        temp[i++] = v.x;
        temp[i++] = v.y;
        temp[i++] = v.z;
    }

    STEPVector3 nor;
    NewellNormal<3, 3, 3>(nor, static_cast<int>(cnt), &temp[0], &temp[1], &temp[2]);
    return normalize ? nor.Normalize() : nor;
}

// ------------------------------------------------------------------------------------------------
void TempMesh::ComputePolygonNormals(std::vector<STEPVector3>& normals,
    bool normalize,
    size_t ofs) const
{
    size_t max_vcount = 0;
    std::vector<unsigned int>::const_iterator begin = vertcnt.begin()+ofs, end = vertcnt.end(),  iit;
    for(iit = begin; iit != end; ++iit) {
        max_vcount = std::max(max_vcount,static_cast<size_t>(*iit));
    }

    std::vector<STEPFloat> temp((max_vcount+2)*4);
    normals.reserve( normals.size() + vertcnt.size()-ofs );

    // `NewellNormal()` currently has a relatively strange interface and need to
    // re-structure things a bit to meet them.
    size_t vidx = std::accumulate(vertcnt.begin(),begin,0);
    for(iit = begin; iit != end; vidx += *iit++) {
        if (!*iit) {
            normals.push_back(STEPVector3());
            continue;
        }
        for(size_t vofs = 0, cnt = 0; vofs < *iit; ++vofs) {
            const STEPVector3& v = verts[vidx+vofs];
            temp[cnt++] = v.x;
            temp[cnt++] = v.y;
            temp[cnt++] = v.z;
#ifdef ASSIMP_BUILD_DEBUG
            temp[cnt] = std::numeric_limits<STEPFloat>::quiet_NaN();
#endif
            ++cnt;
        }

        normals.push_back(STEPVector3());
        NewellNormal<4,4,4>(normals.back(),*iit,&temp[0],&temp[1],&temp[2]);
    }

    if(normalize) {
        for(STEPVector3& n : normals) {
            n.Normalize();
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Compute the normal of the last polygon in the given mesh
STEPVector3 TempMesh::ComputeLastPolygonNormal(bool normalize) const
{
    return ComputePolygonNormal(&verts[verts.size() - vertcnt.back()], vertcnt.back(), normalize);
}

struct CompareVector
{
    bool operator () (const STEPVector3& a, const STEPVector3& b) const
    {
        STEPVector3 d = a - b;
        STEPFloat eps = 1e-6;
        return d.x < -eps || (std::abs(d.x) < eps && d.y < -eps) || (std::abs(d.x) < eps && std::abs(d.y) < eps && d.z < -eps);
    }
};
struct FindVector
{
    STEPVector3 v;
    FindVector(const STEPVector3& p) : v(p) { }
    bool operator () (const STEPVector3& p) { return FuzzyVectorCompare(1e-6)(p, v); }
};

// ------------------------------------------------------------------------------------------------
void TempMesh::FixupFaceOrientation()
{
    const STEPVector3 vavg = Center();

    // create a list of start indices for all faces to allow random access to faces
    std::vector<size_t> faceStartIndices(vertcnt.size());
    for( size_t i = 0, a = 0; a < vertcnt.size(); i += vertcnt[a], ++a )
        faceStartIndices[a] = i;

    // list all faces on a vertex
    std::map<STEPVector3, std::vector<size_t>, CompareVector> facesByVertex;
    for( size_t a = 0; a < vertcnt.size(); ++a )
    {
        for( size_t b = 0; b < vertcnt[a]; ++b )
            facesByVertex[verts[faceStartIndices[a] + b]].push_back(a);
    }
    // determine neighbourhood for all polys
    std::vector<size_t> neighbour(verts.size(), SIZE_MAX);
    std::vector<size_t> tempIntersect(10);
    for( size_t a = 0; a < vertcnt.size(); ++a )
    {
        for( size_t b = 0; b < vertcnt[a]; ++b )
        {
            size_t ib = faceStartIndices[a] + b, nib = faceStartIndices[a] + (b + 1) % vertcnt[a];
            const std::vector<size_t>& facesOnB = facesByVertex[verts[ib]];
            const std::vector<size_t>& facesOnNB = facesByVertex[verts[nib]];
            // there should be exactly one or two faces which appear in both lists. Our face and the other side
            std::vector<size_t>::iterator sectstart = tempIntersect.begin();
            std::vector<size_t>::iterator sectend = std::set_intersection(
                facesOnB.begin(), facesOnB.end(), facesOnNB.begin(), facesOnNB.end(), sectstart);

            if( std::distance(sectstart, sectend) != 2 )
                continue;
            if( *sectstart == a )
                ++sectstart;
            neighbour[ib] = *sectstart;
        }
    }

    // now we're getting started. We take the face which is the farthest away from the center. This face is most probably
    // facing outwards. So we reverse this face to point outwards in relation to the center. Then we adapt neighbouring
    // faces to have the same winding until all faces have been tested.
    std::vector<bool> faceDone(vertcnt.size(), false);
    while( std::count(faceDone.begin(), faceDone.end(), false) != 0 )
    {
        // find the farthest of the remaining faces
        size_t farthestIndex = SIZE_MAX;
        STEPFloat farthestDistance = -1.0;
        for( size_t a = 0; a < vertcnt.size(); ++a )
        {
            if( faceDone[a] )
                continue;
            STEPVector3 faceCenter = std::accumulate(verts.begin() + faceStartIndices[a],
                verts.begin() + faceStartIndices[a] + vertcnt[a], STEPVector3(0.0)) / STEPFloat(vertcnt[a]);
            STEPFloat dst = (faceCenter - vavg).SquareLength();
            if( dst > farthestDistance ) { farthestDistance = dst; farthestIndex = a; }
        }

        // calculate its normal and reverse the poly if its facing towards the mesh center
        STEPVector3 farthestNormal = ComputePolygonNormal(verts.data() + faceStartIndices[farthestIndex], vertcnt[farthestIndex]);
        STEPVector3 farthestCenter = std::accumulate(verts.begin() + faceStartIndices[farthestIndex],
            verts.begin() + faceStartIndices[farthestIndex] + vertcnt[farthestIndex], STEPVector3(0.0))
            / STEPFloat(vertcnt[farthestIndex]);
        // We accapt a bit of negative orientation without reversing. In case of doubt, prefer the orientation given in
        // the file.
        if( (farthestNormal * (farthestCenter - vavg).Normalize()) < -0.4 )
        {
            size_t fsi = faceStartIndices[farthestIndex], fvc = vertcnt[farthestIndex];
            std::reverse(verts.begin() + fsi, verts.begin() + fsi + fvc);
            std::reverse(neighbour.begin() + fsi, neighbour.begin() + fsi + fvc);
            // because of the neighbour index belonging to the edge starting with the point at the same index, we need to
            // cycle the neighbours through to match the edges again.
            // Before: points A - B - C - D with edge neighbour p - q - r - s
            // After: points D - C - B - A, reversed neighbours are s - r - q - p, but the should be
            //                r   q   p   s
            for( size_t a = 0; a < fvc - 1; ++a )
                std::swap(neighbour[fsi + a], neighbour[fsi + a + 1]);
        }
        faceDone[farthestIndex] = true;
        std::vector<size_t> todo;
        todo.push_back(farthestIndex);

        // go over its neighbour faces recursively and adapt their winding order to match the farthest face
        while( !todo.empty() )
        {
            size_t tdf = todo.back();
            size_t vsi = faceStartIndices[tdf], vc = vertcnt[tdf];
            todo.pop_back();

            // check its neighbours
            for( size_t a = 0; a < vc; ++a )
            {
                // ignore neighbours if we already checked them
                size_t nbi = neighbour[vsi + a];
                if( nbi == SIZE_MAX || faceDone[nbi] )
                    continue;

                const STEPVector3& vp = verts[vsi + a];
                size_t nbvsi = faceStartIndices[nbi], nbvc = vertcnt[nbi];
                std::vector<STEPVector3>::iterator it = std::find_if(verts.begin() + nbvsi, verts.begin() + nbvsi + nbvc, FindVector(vp));
                ai_assert(it != verts.begin() + nbvsi + nbvc);
                size_t nb_vidx = std::distance(verts.begin() + nbvsi, it);
                // two faces winded in the same direction should have a crossed edge, where one face has p0->p1 and the other
                // has p1'->p0'. If the next point on the neighbouring face is also the next on the current face, we need
                // to reverse the neighbour
                nb_vidx = (nb_vidx + 1) % nbvc;
                size_t oursideidx = (a + 1) % vc;
                if( FuzzyVectorCompare(1e-6)(verts[vsi + oursideidx], verts[nbvsi + nb_vidx]) )
                {
                    std::reverse(verts.begin() + nbvsi, verts.begin() + nbvsi + nbvc);
                    std::reverse(neighbour.begin() + nbvsi, neighbour.begin() + nbvsi + nbvc);
                    for( size_t a = 0; a < nbvc - 1; ++a )
                        std::swap(neighbour[nbvsi + a], neighbour[nbvsi + a + 1]);
                }

                // either way we're done with the neighbour. Mark it as done and continue checking from there recursively
                faceDone[nbi] = true;
                todo.push_back(nbi);
            }
        }

        // no more faces reachable from this part of the surface, start over with a disjunct part and its farthest face
    }
}

// ------------------------------------------------------------------------------------------------
void TempMesh::RemoveAdjacentDuplicates()
{

    bool drop = false;
    std::vector<STEPVector3>::iterator base = verts.begin();
    for(unsigned int& cnt : vertcnt) {
        if (cnt < 2){
            base += cnt;
            continue;
        }

        STEPVector3 vmin,vmax;
        ArrayBounds(&*base, cnt ,vmin,vmax);


        const STEPFloat epsilon = (vmax-vmin).SquareLength() / static_cast<STEPFloat>(1e9);
        //const STEPFloat dotepsilon = 1e-9;

        //// look for vertices that lie directly on the line between their predecessor and their
        //// successor and replace them with either of them.

        //for(size_t i = 0; i < cnt; ++i) {
        //  STEPVector3& v1 = *(base+i), &v0 = *(base+(i?i-1:cnt-1)), &v2 = *(base+(i+1)%cnt);
        //  const STEPVector3& d0 = (v1-v0), &d1 = (v2-v1);
        //  const STEPFloat l0 = d0.SquareLength(), l1 = d1.SquareLength();
        //  if (!l0 || !l1) {
        //      continue;
        //  }

        //  const STEPFloat d = (d0/std::sqrt(l0))*(d1/std::sqrt(l1));

        //  if ( d >= 1.f-dotepsilon ) {
        //      v1 = v0;
        //  }
        //  else if ( d < -1.f+dotepsilon ) {
        //      v2 = v1;
        //      continue;
        //  }
        //}

        // drop any identical, adjacent vertices. this pass will collect the dropouts
        // of the previous pass as a side-effect.
        FuzzyVectorCompare fz(epsilon);
        std::vector<STEPVector3>::iterator end = base+cnt, e = std::unique( base, end, fz );
        if (e != end) {
            cnt -= static_cast<unsigned int>(std::distance(e, end));
            verts.erase(e,end);
            drop  = true;
        }

        // check front and back vertices for this polygon
        if (cnt > 1 && fz(*base,*(base+cnt-1))) {
            verts.erase(base+ --cnt);
            drop  = true;
        }

        // removing adjacent duplicates shouldn't erase everything :-)
        ai_assert(cnt>0);
        base += cnt;
    }
    if(drop) {
        STEPImporter::LogDebug("removing duplicate vertices");
    }
}

// ------------------------------------------------------------------------------------------------
void TempMesh::Swap(TempMesh& other)
{
    vertcnt.swap(other.vertcnt);
    verts.swap(other.verts);
}

// ------------------------------------------------------------------------------------------------
bool IsTrue(const EXPRESS::BOOLEAN& in)
{
    return (std::string)in == "TRUE" || (std::string)in == "T";
}

// ------------------------------------------------------------------------------------------------
STEPFloat ConvertSIPrefix(const std::string& prefix)
{
    if (prefix == "EXA") {
        return 1e18f;
    }
    else if (prefix == "PETA") {
        return 1e15f;
    }
    else if (prefix == "TERA") {
        return 1e12f;
    }
    else if (prefix == "GIGA") {
        return 1e9f;
    }
    else if (prefix == "MEGA") {
        return 1e6f;
    }
    else if (prefix == "KILO") {
        return 1e3f;
    }
    else if (prefix == "HECTO") {
        return 1e2f;
    }
    else if (prefix == "DECA") {
        return 1e-0f;
    }
    else if (prefix == "DECI") {
        return 1e-1f;
    }
    else if (prefix == "CENTI") {
        return 1e-2f;
    }
    else if (prefix == "MILLI") {
        return 1e-3f;
    }
    else if (prefix == "MICRO") {
        return 1e-6f;
    }
    else if (prefix == "NANO") {
        return 1e-9f;
    }
    else if (prefix == "PICO") {
        return 1e-12f;
    }
    else if (prefix == "FEMTO") {
        return 1e-15f;
    }
    else if (prefix == "ATTO") {
        return 1e-18f;
    }
    else {
        STEPImporter::LogError("Unrecognized SI prefix: " + prefix);
        return 1;
    }
}

// ------------------------------------------------------------------------------------------------
void ConvertColor(aiColor4D& out, const STEPColour_Rgb &in)
{
    out.r = static_cast<float>( in.Red );
    out.g = static_cast<float>( in.Green );
    out.b = static_cast<float>( in.Blue );
    out.a = static_cast<float>( 1.f );
}

// ------------------------------------------------------------------------------------------------
void ConvertCartesianPoint(STEPVector3& out, const STEPCartesian_Point& in)
{
    out = STEPVector3();
    for(size_t i = 0; i < in.Coordinates.size(); ++i) {
        out[static_cast<unsigned int>(i)] = in.Coordinates[i];
    }
}

// ------------------------------------------------------------------------------------------------
void ConvertVector(STEPVector3& out, const STEPVector& in)
{
    ConvertDirection(out,in.Orientation);
    out *= in.Magnitude;
}

// ------------------------------------------------------------------------------------------------
void ConvertDirection(STEPVector3& out, const STEPDirection& in)
{
    out = STEPVector3();
    for(size_t i = 0; i < in.DirectionRatios.size(); ++i) {
        out[static_cast<unsigned int>(i)] = in.DirectionRatios[i];
    }
    const STEPFloat len = out.Length();
    if (len<1e-6) {
        STEPImporter::LogWarn("direction vector magnitude too small, normalization would result in a division by zero");
        return;
    }
    out /= len;
}

// ------------------------------------------------------------------------------------------------
void AssignMatrixAxes(STEPMatrix4& out, const STEPVector3& x, const STEPVector3& y, const STEPVector3& z)
{
    out.a1 = x.x;
    out.b1 = x.y;
    out.c1 = x.z;

    out.a2 = y.x;
    out.b2 = y.y;
    out.c2 = y.z;

    out.a3 = z.x;
    out.b3 = z.y;
    out.c3 = z.z;
}

// ------------------------------------------------------------------------------------------------
void ConvertAxisPlacement(STEPMatrix4& out, const STEPAxis2_Placement_3d& in)
{
    STEPVector3 loc;
    ConvertCartesianPoint(loc,in.Location);

    STEPVector3 z(0.f,0.f,1.f),r(1.f,0.f,0.f),x;

    if (in.Axis) {
        ConvertDirection(z,*in.Axis.Get());
    }
    if (in.RefDirection) {
        ConvertDirection(r,*in.RefDirection.Get());
    }

    STEPVector3 v = r.Normalize();
    STEPVector3 tmpx = z * (v*z);

    x = (v-tmpx).Normalize();
    STEPVector3 y = (z^x);

    STEPMatrix4::Translation(loc,out);
    AssignMatrixAxes(out,x,y,z);
}

// ------------------------------------------------------------------------------------------------
void ConvertAxisPlacement(STEPMatrix4& out, const STEPAxis2_Placement_2d& in)
{
    STEPVector3 loc;
    ConvertCartesianPoint(loc,in.Location);

    STEPVector3 x(1.f,0.f,0.f);
    if (in.RefDirection) {
        ConvertDirection(x,*in.RefDirection.Get());
    }

    const STEPVector3 y = STEPVector3(x.y,-x.x,0.f);

    STEPMatrix4::Translation(loc,out);
    AssignMatrixAxes(out,x,y,STEPVector3(0.f,0.f,1.f));
}

// ------------------------------------------------------------------------------------------------
void ConvertAxisPlacement(STEPVector3& axis, STEPVector3& pos, const STEPAxis1_Placement& in)
{
    ConvertCartesianPoint(pos,in.Location);
    if (in.Axis) {
        ConvertDirection(axis,in.Axis.Get());
    }
    else {
        axis = STEPVector3(0.f,0.f,1.f);
    }
}

// ------------------------------------------------------------------------------------------------
void ConvertAxisPlacement(STEPMatrix4& out, const STEPAxis2_Placement& in, ConversionData& conv)
{
    if(const STEPAxis2_Placement_3d* pl3 = in.ResolveSelectPtr<STEPAxis2_Placement_3d>(conv.db)) {
        ConvertAxisPlacement(out,*pl3);
    }
    else if(const STEPAxis2_Placement_2d* pl2 = in.ResolveSelectPtr<STEPAxis2_Placement_2d>(conv.db)) {
        ConvertAxisPlacement(out,*pl2);
    }
    else {
        STEPImporter::LogWarn("skipping unknown STEPAxis2_Placement entity");
    }
}

// ------------------------------------------------------------------------------------------------
void ConvertTransformOperator(STEPMatrix4& out, const STEPCartesian_Transformation_Operator& op)
{
    STEPVector3 loc;
    ConvertCartesianPoint(loc,op.LocalOrigin);

    STEPVector3 x(1.f,0.f,0.f),y(0.f,1.f,0.f),z(0.f,0.f,1.f);
    if (op.Axis1) {
        ConvertDirection(x,*op.Axis1.Get());
    }
    if (op.Axis2) {
        ConvertDirection(y,*op.Axis2.Get());
    }
    if (const STEPCartesian_Transformation_Operator_3d* op2 = op.ToPtr<STEPCartesian_Transformation_Operator_3d>()) {
        if(op2->Axis3) {
            ConvertDirection(z,*op2->Axis3.Get());
        }
    }

    STEPMatrix4 locm;
    STEPMatrix4::Translation(loc,locm);
    AssignMatrixAxes(out,x,y,z);

    const STEPFloat sc = op.Scale ? op.Scale.Get() : 1.f;
    STEPVector3 vscale(sc,sc,sc);

    STEPMatrix4 s;
    STEPMatrix4::Scaling(vscale,s);

    out = locm * out * s;
}


} // ! STEP
} // ! Assimp

#endif
