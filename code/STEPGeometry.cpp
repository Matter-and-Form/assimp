/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2010, assimp team
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

/** @file  STEPGeometry.cpp
 *  @brief Geometry conversion and synthesis for STEP
 */



#ifndef ASSIMP_BUILD_NO_STEP_IMPORTER
#include "STEPUtil.h"
#include "PolyTools.h"
#include "ProcessHelper.h"

#include "../contrib/poly2tri/poly2tri/poly2tri.h"
#include "../contrib/clipper/clipper.hpp"
#include <memory>

#include <iterator>

namespace Assimp {
    namespace STEP {

// ------------------------------------------------------------------------------------------------
bool ProcessPolyloop(const STEPPoly_Loop& loop, TempMesh& meshout, ConversionData& /*conv*/)
{
    size_t cnt = 0;
    for(const STEPCartesian_Point& c : loop.Polygon) {
        STEPVector3 tmp;
        ConvertCartesianPoint(tmp,c);

        meshout.verts.push_back(tmp);
        ++cnt;
    }

    meshout.vertcnt.push_back(static_cast<unsigned int>(cnt));

    // zero- or one- vertex polyloops simply ignored
    if (meshout.vertcnt.back() > 1) {
        return true;
    }

    if (meshout.vertcnt.back()==1) {
        meshout.vertcnt.pop_back();
        meshout.verts.pop_back();
    }
    return false;
}

// ------------------------------------------------------------------------------------------------
void ProcessPolygonBoundaries(TempMesh& result, const TempMesh& inmesh, size_t master_bounds = (size_t)-1)
{
    // handle all trivial cases
    if(inmesh.vertcnt.empty()) {
        return;
    }
    if(inmesh.vertcnt.size() == 1) {
        result.Append(inmesh);
        return;
    }

    ai_assert(std::count(inmesh.vertcnt.begin(), inmesh.vertcnt.end(), 0) == 0);

    typedef std::vector<unsigned int>::const_iterator face_iter;

    face_iter begin = inmesh.vertcnt.begin(), end = inmesh.vertcnt.end(), iit;
    std::vector<unsigned int>::const_iterator outer_polygon_it = end;

    // major task here: given a list of nested polygon boundaries (one of which
    // is the outer contour), reduce the triangulation task arising here to
    // one that can be solved using the "quadrulation" algorithm which we use
    // for pouring windows out of walls. The algorithm does not handle all
    // cases but at least it is numerically stable and gives "nice" triangles.

    // first compute normals for all polygons using Newell's algorithm
    // do not normalize 'normals', we need the original length for computing the polygon area
    std::vector<STEPVector3> normals;
    inmesh.ComputePolygonNormals(normals,false);

    // One of the polygons might be a STEPFace_Outer_Bound (in which case `master_bounds`
    // is its index). Sadly we can't rely on it, the docs say 'At most one of the bounds
    // shall be of the type STEPFace_Outer_Bound'
    STEPFloat area_outer_polygon = 1e-10f;
    if (master_bounds != (size_t)-1) {
        ai_assert(master_bounds < inmesh.vertcnt.size());
        outer_polygon_it = begin + master_bounds;
    }
    else {
        for(iit = begin; iit != end; iit++) {
            // find the polygon with the largest area and take it as the outer bound.
            STEPVector3& n = normals[std::distance(begin,iit)];
            const STEPFloat area = n.SquareLength();
            if (area > area_outer_polygon) {
                area_outer_polygon = area;
                outer_polygon_it = iit;
            }
        }
    }

    ai_assert(outer_polygon_it != end);

    const size_t outer_polygon_size = *outer_polygon_it;
    const STEPVector3& master_normal = normals[std::distance(begin, outer_polygon_it)];

    // Generate fake openings to meet the interface for the quadrulate
    // algorithm. It boils down to generating small boxes given the
    // inner polygon and the surface normal of the outer contour.
    // It is important that we use the outer contour's normal because
    // this is the plane onto which the quadrulate algorithm will
    // project the entire mesh.
    std::vector<TempOpening> fake_openings;
    fake_openings.reserve(inmesh.vertcnt.size()-1);

    std::vector<STEPVector3>::const_iterator vit = inmesh.verts.begin(), outer_vit;

    for(iit = begin; iit != end; vit += *iit++) {
        if (iit == outer_polygon_it) {
            outer_vit = vit;
            continue;
        }

        // Filter degenerate polygons to keep them from causing trouble later on
        STEPVector3& n = normals[std::distance(begin,iit)];
        const STEPFloat area = n.SquareLength();
        if (area < 1e-5f) {
            STEPImporter::LogWarn("skipping degenerate polygon (ProcessPolygonBoundaries)");
            continue;
        }

        fake_openings.push_back(TempOpening());
        TempOpening& opening = fake_openings.back();

        opening.extrusionDir = master_normal;
        opening.solid = NULL;

        opening.profileMesh = std::make_shared<TempMesh>();
        opening.profileMesh->verts.reserve(*iit);
        opening.profileMesh->vertcnt.push_back(*iit);

        std::copy(vit, vit + *iit, std::back_inserter(opening.profileMesh->verts));
    }

    // fill a mesh with ONLY the main polygon
    TempMesh temp;
    temp.verts.reserve(outer_polygon_size);
    temp.vertcnt.push_back(static_cast<unsigned int>(outer_polygon_size));
    std::copy(outer_vit, outer_vit+outer_polygon_size,
        std::back_inserter(temp.verts));

    result.Append(temp);
}

// ------------------------------------------------------------------------------------------------
void ProcessConnectedFaceSet(const STEPConnected_Face_Set& fset, TempMesh& result, ConversionData& conv)
{
    for(const STEPFace& face : fset.CfsFaces) {
        // size_t ob = -1, cnt = 0;
        TempMesh meshout;
        for(const STEPFace_Bound& bound : face.Bounds) {

            if(const STEPPoly_Loop* const polyloop = bound.Bound->ToPtr<STEPPoly_Loop>()) {
                if(ProcessPolyloop(*polyloop, meshout,conv)) {

                    // The outer boundary is better determined by checking which
                    // polygon covers the largest area.

                    //if(bound.ToPtr<STEPFace_Outer_Bound>()) {
                    //  ob = cnt;
                    //}
                    //++cnt;

                }
            }
            else {
                STEPImporter::LogWarn("skipping unknown STEPFace_Bound entity, type is " + bound.Bound->GetClassName());
                continue;
            }

            // And this, even though it is sometimes TRUE and sometimes FALSE,
            // does not really improve results.

            /*if(!IsTrue(bound.Orientation)) {
                size_t c = 0;
                for(unsigned int& c : meshout.vertcnt) {
                    std::reverse(result.verts.begin() + cnt,result.verts.begin() + cnt + c);
                    cnt += c;
                }
            }*/
        }
        ProcessPolygonBoundaries(result, meshout);
    }
}

// ------------------------------------------------------------------------------------------------
void ProcessRevolvedAreaSolid(const STEPRevolved_Area_Solid& solid, TempMesh& result, ConversionData& conv)
{
    TempMesh meshout;

    STEPVector3 axis, pos;
    ConvertAxisPlacement(axis,pos,solid.Axis);

    STEPMatrix4 tb0,tb1;
    STEPMatrix4::Translation(pos,tb0);
    STEPMatrix4::Translation(-pos,tb1);

    const std::vector<STEPVector3>& in = meshout.verts;
    const size_t size=in.size();

    bool has_area = /*solid.SweptArea->ProfileType == "AREA" &&*/ size>2;
    const STEPFloat max_angle = solid.Angle*conv.angle_scale;
    if(std::fabs(max_angle) < 1e-3) {
        if(has_area) {
            result = meshout;
        }
        return;
    }

    const unsigned int cnt_segments = std::max(2u,static_cast<unsigned int>(16 * std::fabs(max_angle)/AI_MATH_HALF_PI_F));
    const STEPFloat delta = max_angle/cnt_segments;

    has_area = has_area && std::fabs(max_angle) < AI_MATH_TWO_PI_F*0.99;

    result.verts.reserve(size*((cnt_segments+1)*4+(has_area?2:0)));
    result.vertcnt.reserve(size*cnt_segments+2);

    STEPMatrix4 rot;
    rot = tb0 * STEPMatrix4::Rotation(delta,axis,rot) * tb1;

    size_t base = 0;
    std::vector<STEPVector3>& out = result.verts;

    // dummy data to simplify later processing
    for(size_t i = 0; i < size; ++i) {
        out.insert(out.end(),4,in[i]);
    }

    for(unsigned int seg = 0; seg < cnt_segments; ++seg) {
        for(size_t i = 0; i < size; ++i) {
            const size_t next = (i+1)%size;

            result.vertcnt.push_back(4);
            const STEPVector3 base_0 = out[base+i*4+3],base_1 = out[base+next*4+3];

            out.push_back(base_0);
            out.push_back(base_1);
            out.push_back(rot*base_1);
            out.push_back(rot*base_0);
        }
        base += size*4;
    }

    out.erase(out.begin(),out.begin()+size*4);

    if(has_area) {
        // leave the triangulation of the profile area to the ear cutting
        // implementation in aiProcess_Triangulate - for now we just
        // feed in two huge polygons.
        base -= size*8;
        for(size_t i = size; i--; ) {
            out.push_back(out[base+i*4+3]);
        }
        for(size_t i = 0; i < size; ++i ) {
            out.push_back(out[i*4]);
        }
        result.vertcnt.push_back(static_cast<unsigned int>(size));
        result.vertcnt.push_back(static_cast<unsigned int>(size));
    }

    STEPImporter::LogDebug("generate mesh procedurally by radial extrusion (STEPRevolved_Area_Solid)");
}



// ------------------------------------------------------------------------------------------------
void ProcessSweptDiskSolid(const STEPSwept_Disk_Solid solid, TempMesh& result, ConversionData& conv)
{
    const Curve* const curve = Curve::Convert(*solid.Directrix, conv);
    if(!curve) {
        STEPImporter::LogError("failed to convert Directrix curve (STEPSwept_Disk_Solid)");
        return;
    }

    const unsigned int cnt_segments = 16;
    const STEPFloat deltaAngle = AI_MATH_TWO_PI/cnt_segments;

    const size_t samples = curve->EstimateSampleCount(solid.StartParam,solid.EndParam);

    result.verts.reserve(cnt_segments * samples * 4);
    result.vertcnt.reserve((cnt_segments - 1) * samples);

    std::vector<STEPVector3> points;
    points.reserve(cnt_segments * samples);

    TempMesh temp;
    curve->SampleDiscrete(temp,solid.StartParam,solid.EndParam);
    const std::vector<STEPVector3>& curve_points = temp.verts;

    if(curve_points.empty()) {
        STEPImporter::LogWarn("curve evaluation yielded no points (STEPSwept_Disk_Solid)");
        return;
    }

    STEPVector3 current = curve_points[0];
    STEPVector3 previous = current;
    STEPVector3 next;

    STEPVector3 startvec;
    startvec.x = 1.0f;
    startvec.y = 1.0f;
    startvec.z = 1.0f;

    unsigned int last_dir = 0;

    // generate circles at the sweep positions
    for(size_t i = 0; i < samples; ++i) {

        if(i != samples - 1) {
            next = curve_points[i + 1];
        }

        // get a direction vector reflecting the approximate curvature (i.e. tangent)
        STEPVector3 d = (current-previous) + (next-previous);

        d.Normalize();

        // figure out an arbitrary point q so that (p-q) * d = 0,
        // try to maximize ||(p-q)|| * ||(p_last-q_last)||
        STEPVector3 q;
        bool take_any = false;

        for (unsigned int i = 0; i < 2; ++i, take_any = true) {
            if ((last_dir == 0 || take_any) && std::abs(d.x) > 1e-6) {
                q.y = startvec.y;
                q.z = startvec.z;
                q.x = -(d.y * q.y + d.z * q.z) / d.x;
                last_dir = 0;
                break;
            }
            else if ((last_dir == 1 || take_any) && std::abs(d.y) > 1e-6) {
                q.x = startvec.x;
                q.z = startvec.z;
                q.y = -(d.x * q.x + d.z * q.z) / d.y;
                last_dir = 1;
                break;
            }
            else if ((last_dir == 2 && std::abs(d.z) > 1e-6) || take_any) {
                q.y = startvec.y;
                q.x = startvec.x;
                q.z = -(d.y * q.y + d.x * q.x) / d.z;
                last_dir = 2;
                break;
            }
        }

        q *= solid.Radius / q.Length();
        startvec = q;

        // generate a rotation matrix to rotate q around d
        STEPMatrix4 rot;
        STEPMatrix4::Rotation(deltaAngle,d,rot);

        for (unsigned int seg = 0; seg < cnt_segments; ++seg, q *= rot ) {
            points.push_back(q + current);
        }

        previous = current;
        current = next;
    }

    // make quads
    for(size_t i = 0; i < samples - 1; ++i) {

        const aiVector3D& this_start = points[ i * cnt_segments ];

        // locate corresponding point on next sample ring
        unsigned int best_pair_offset = 0;
        float best_distance_squared = 1e10f;
        for (unsigned int seg = 0; seg < cnt_segments; ++seg) {
            const aiVector3D& p = points[ (i+1) * cnt_segments + seg];
            const float l = (p-this_start).SquareLength();

            if(l < best_distance_squared) {
                best_pair_offset = seg;
                best_distance_squared = l;
            }
        }

        for (unsigned int seg = 0; seg < cnt_segments; ++seg) {

            result.verts.push_back(points[ i * cnt_segments + (seg % cnt_segments)]);
            result.verts.push_back(points[ i * cnt_segments + (seg + 1) % cnt_segments]);
            result.verts.push_back(points[ (i+1) * cnt_segments + ((seg + 1 + best_pair_offset) % cnt_segments)]);
            result.verts.push_back(points[ (i+1) * cnt_segments + ((seg + best_pair_offset) % cnt_segments)]);

            STEPVector3& v1 = *(result.verts.end()-1);
            STEPVector3& v2 = *(result.verts.end()-2);
            STEPVector3& v3 = *(result.verts.end()-3);
            STEPVector3& v4 = *(result.verts.end()-4);

            if (((v4-v3) ^ (v4-v1)) * (v4 - curve_points[i]) < 0.0f) {
                std::swap(v4, v1);
                std::swap(v3, v2);
            }

            result.vertcnt.push_back(4);
        }
    }

    STEPImporter::LogDebug("generate mesh procedurally by sweeping a disk along a curve (STEPSwept_Disk_Solid)");
}

// ------------------------------------------------------------------------------------------------
STEPMatrix3 DerivePlaneCoordinateSpace(const TempMesh& curmesh, bool& ok, STEPVector3& norOut)
{
    const std::vector<STEPVector3>& out = curmesh.verts;
    STEPMatrix3 m;

    ok = true;

    // The input "mesh" must be a single polygon
    const size_t s = out.size();
    assert(curmesh.vertcnt.size() == 1 && curmesh.vertcnt.back() == s);

    const STEPVector3 any_point = out[s-1];
    STEPVector3 nor;

    // The input polygon is arbitrarily shaped, therefore we might need some tries
    // until we find a suitable normal. Note that Newell's algorithm would give
    // a more robust result, but this variant also gives us a suitable first
    // axis for the 2D coordinate space on the polygon plane, exploiting the
    // fact that the input polygon is nearly always a quad.
    bool done = false;
    size_t i, j;
    for (i = 0; !done && i < s-2; done || ++i) {
        for (j = i+1; j < s-1; ++j) {
            nor = -((out[i]-any_point)^(out[j]-any_point));
            if(std::fabs(nor.Length()) > 1e-8f) {
                done = true;
                break;
            }
        }
    }

    if(!done) {
        ok = false;
        return m;
    }

    nor.Normalize();
    norOut = nor;

    STEPVector3 r = (out[i]-any_point);
    r.Normalize();

    //if(d) {
    //  *d = -any_point * nor;
    //}

    // Reconstruct orthonormal basis
    // XXX use Gram Schmidt for increased robustness
    STEPVector3 u = r ^ nor;
    u.Normalize();

    m.a1 = r.x;
    m.a2 = r.y;
    m.a3 = r.z;

    m.b1 = u.x;
    m.b2 = u.y;
    m.b3 = u.z;

    m.c1 = -nor.x;
    m.c2 = -nor.y;
    m.c3 = -nor.z;

    return m;
}

// Extrudes the given polygon along the direction, converts it into an opening or applies all openings as necessary.
void ProcessExtrudedArea(const STEPExtruded_Area_Solid& solid, const TempMesh& curve,
    const STEPVector3& extrusionDir, TempMesh& result)
{
    // Outline: 'curve' is now a list of vertex points forming the underlying profile, extrude along the given axis,
    // forming new triangles.
    const bool has_area = curve.verts.size() > 2;
    if( solid.Depth < 1e-6 ) {
        if( has_area ) {
            result.Append(curve);
        }
        return;
    }

    result.verts.reserve(curve.verts.size()*(has_area ? 4 : 2));
    result.vertcnt.reserve(curve.verts.size() + 2);
    std::vector<STEPVector3> in = curve.verts;

    TempMesh& curmesh = result;
    std::vector<STEPVector3>& out = curmesh.verts;

    for( size_t i = 0; i < in.size(); ++i ) {
        const size_t next = (i + 1) % in.size();

        curmesh.vertcnt.push_back(4);

        out.push_back(in[i]);
        out.push_back(in[next]);
        out.push_back(in[next] + extrusionDir);
        out.push_back(in[i] + extrusionDir);
    }

    if( has_area ) {

        for( size_t n = 0; n < 2; ++n ) {
            if( n > 0 ) {
                for( size_t i = 0; i < in.size(); ++i )
                    out.push_back(in[i] + extrusionDir);
            }
            else {
                for( size_t i = in.size(); i--; )
                    out.push_back(in[i]);
            }

            curmesh.vertcnt.push_back(static_cast<unsigned int>(in.size()));
        }
    }

    STEPImporter::LogDebug("generate mesh procedurally by extrusion (STEPExtruded_Area_Solid)");
}

// ------------------------------------------------------------------------------------------------
void ProcessExtrudedAreaSolid(const STEPExtruded_Area_Solid& solid, TempMesh& result)
{
    TempMesh meshout;

    STEPVector3 dir;
    ConvertDirection(dir,solid.ExtrudedDirection);
    dir *= solid.Depth;

    ProcessExtrudedArea(solid, meshout, dir, result);
}

// ------------------------------------------------------------------------------------------------
void ProcessSweptAreaSolid(const STEPSwept_Area_Solid& swept, TempMesh& meshout,
    ConversionData& conv)
{
    if(const STEPExtruded_Area_Solid* const solid = swept.ToPtr<STEPExtruded_Area_Solid>()) {
        ProcessExtrudedAreaSolid(*solid,meshout);
    }
    else if(const STEPRevolved_Area_Solid* const rev = swept.ToPtr<STEPRevolved_Area_Solid>()) {
        ProcessRevolvedAreaSolid(*rev,meshout, conv);
    }
    else {
        STEPImporter::LogWarn("skipping unknown STEPSwept_Area_Solid entity, type is " + swept.GetClassName());
    }
}

// ------------------------------------------------------------------------------------------------
bool ProcessGeometricItem(const STEPRepresentation_Item& geo, unsigned int matid, std::vector<unsigned int>& mesh_indices,
    ConversionData& conv)
{
    bool fix_orientation = false;
    std::shared_ptr< TempMesh > meshtmp = std::make_shared<TempMesh>();
    if(const STEPShell_Based_Surface_Model* shellmod = geo.ToPtr<STEPShell_Based_Surface_Model>()) {
        for(std::shared_ptr<const STEPShell> shell :shellmod->SbsmBoundary) {
            try {
                const EXPRESS::ENTITY& e = shell->To<ENTITY>();
                const STEPConnected_Face_Set& fs = conv.db.MustGetObject(e).To<STEPConnected_Face_Set>();

                ProcessConnectedFaceSet(fs,*meshtmp.get(),conv);
            }
            catch(std::bad_cast&) {
                STEPImporter::LogWarn("unexpected type error, STEPShell ought to inherit from STEPConnected_Face_Set");
            }
        }
        fix_orientation = true;
    }
    else  if(const STEPConnected_Face_Set* fset = geo.ToPtr<STEPConnected_Face_Set>()) {
        ProcessConnectedFaceSet(*fset,*meshtmp.get(),conv);
        fix_orientation = true;
    }
    else  if(const STEPSwept_Area_Solid* swept = geo.ToPtr<STEPSwept_Area_Solid>()) {
        ProcessSweptAreaSolid(*swept,*meshtmp.get(),conv);
    }
    else  if(const STEPSwept_Disk_Solid* disk = geo.ToPtr<STEPSwept_Disk_Solid>()) {
        ProcessSweptDiskSolid(*disk,*meshtmp.get(),conv);
    }
    else if(const STEPManifold_Solid_Brep* brep = geo.ToPtr<STEPManifold_Solid_Brep>()) {
        ProcessConnectedFaceSet(brep->Outer,*meshtmp.get(),conv);
        fix_orientation = true;
    }
    else if(const STEPFace_Based_Surface_Model* surf = geo.ToPtr<STEPFace_Based_Surface_Model>()) {
        for(const STEPConnected_Face_Set& fc : surf->FbsmFaces) {
            ProcessConnectedFaceSet(fc,*meshtmp.get(),conv);
        }
        fix_orientation = true;
    }
    else  if(const STEPBoolean_Result* boolean = geo.ToPtr<STEPBoolean_Result>()) {
        STEPImporter::LogWarn("skipping STEPBoolean_Result entity");
        return false;
        //ProcessBoolean(*boolean,*meshtmp.get(),conv);
    }
    else {
        STEPImporter::LogWarn("skipping unknown STEPGeometric_Representation_Item entity, type is " + geo.GetClassName());
        return false;
    }

    if (meshtmp->IsEmpty()) {
        return false;
    }

    meshtmp->RemoveAdjacentDuplicates();
    meshtmp->RemoveDegenerates();

    if(fix_orientation) {
//      meshtmp->FixupFaceOrientation();
    }

    aiMesh* const mesh = meshtmp->ToMesh();
    if(mesh) {
        mesh->mMaterialIndex = matid;
        mesh_indices.push_back(static_cast<unsigned int>(conv.meshes.size()));
        conv.meshes.push_back(mesh);
        return true;
    }
    return false;
}

// ------------------------------------------------------------------------------------------------
void AssignAddedMeshes(std::vector<unsigned int>& mesh_indices,aiNode* nd,
    ConversionData& /*conv*/)
{
    if (!mesh_indices.empty()) {

        // make unique
        std::sort(mesh_indices.begin(),mesh_indices.end());
        std::vector<unsigned int>::iterator it_end = std::unique(mesh_indices.begin(),mesh_indices.end());

        nd->mNumMeshes = static_cast<unsigned int>(std::distance(mesh_indices.begin(),it_end));

        nd->mMeshes = new unsigned int[nd->mNumMeshes];
        for(unsigned int i = 0; i < nd->mNumMeshes; ++i) {
            nd->mMeshes[i] = mesh_indices[i];
        }
    }
}

// ------------------------------------------------------------------------------------------------
bool TryQueryMeshCache(const STEPRepresentation_Item& item,
    std::vector<unsigned int>& mesh_indices, unsigned int mat_index,
    ConversionData& conv)
{
    ConversionData::MeshCacheIndex idx(&item, mat_index);
    ConversionData::MeshCache::const_iterator it = conv.cached_meshes.find(idx);
    if (it != conv.cached_meshes.end()) {
        std::copy((*it).second.begin(),(*it).second.end(),std::back_inserter(mesh_indices));
        return true;
    }
    return false;
}

// ------------------------------------------------------------------------------------------------
void PopulateMeshCache(const STEPRepresentation_Item& item,
    const std::vector<unsigned int>& mesh_indices, unsigned int mat_index,
    ConversionData& conv)
{
    ConversionData::MeshCacheIndex idx(&item, mat_index);
    conv.cached_meshes[idx] = mesh_indices;
}

// ------------------------------------------------------------------------------------------------
bool ProcessRepresentationItem(const STEPRepresentation_Item& item, unsigned int matid,
    std::vector<unsigned int>& mesh_indices,
    ConversionData& conv)
{
    // determine material
    unsigned int localmatid = ProcessMaterials(item.GetID(), matid, conv, true);

    if (!TryQueryMeshCache(item,mesh_indices,localmatid,conv)) {
        if(ProcessGeometricItem(item,localmatid,mesh_indices,conv)) {
            if(mesh_indices.size()) {
                PopulateMeshCache(item,mesh_indices,localmatid,conv);
            }
        }
        else return false;
    }
    return true;
}


} // ! STEP
} // ! Assimp

#endif
