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

/** @file  STEP.cpp
 *  @brief Implementation of the Industry Foundation Classes loader.
 */

#ifndef INCLUDED_STEPUTIL_H
#define INCLUDED_STEPUTIL_H

#include "STEPReaderGen.h"
#include "STEPImporter.h"
#include "STEPFile.h"
#include <assimp/mesh.h>
#include <assimp/material.h>

struct aiNode;

namespace Assimp {
namespace STEP {

    typedef double STEPFloat;

    // STEPFloat-precision math data types
    typedef aiVector2t<STEPFloat> STEPVector2;
    typedef aiVector3t<STEPFloat> STEPVector3;
    typedef aiMatrix4x4t<STEPFloat> STEPMatrix4;
    typedef aiMatrix3x3t<STEPFloat> STEPMatrix3;
    typedef aiColor4t<STEPFloat> STEPColor4;


// ------------------------------------------------------------------------------------------------
// Helper for std::for_each to delete all heap-allocated items in a container
// ------------------------------------------------------------------------------------------------
template<typename T>
struct delete_fun
{
    void operator()(T* del) {
        delete del;
    }
};



// ------------------------------------------------------------------------------------------------
// Helper used during mesh construction. Aids at creating aiMesh'es out of relatively few polygons.
// ------------------------------------------------------------------------------------------------
struct TempMesh
{
    std::vector<STEPVector3> verts;
    std::vector<unsigned int> vertcnt;

    // utilities
    aiMesh* ToMesh();
    void Clear();
    void Transform(const STEPMatrix4& mat);
    STEPVector3 Center() const;
    void Append(const TempMesh& other);

    bool IsEmpty() const {
        return verts.empty() && vertcnt.empty();
    }

    void RemoveAdjacentDuplicates();
    void RemoveDegenerates();

    void FixupFaceOrientation();

    static STEPVector3 ComputePolygonNormal(const STEPVector3* vtcs, size_t cnt, bool normalize = true);
    STEPVector3 ComputeLastPolygonNormal(bool normalize = true) const;
    void ComputePolygonNormals(std::vector<STEPVector3>& normals, bool normalize = true, size_t ofs = 0) const;

    void Swap(TempMesh& other);
};


// ------------------------------------------------------------------------------------------------
// Temporary representation of an opening in a wall or a floor
// ------------------------------------------------------------------------------------------------
struct TempOpening
{
    const STEP::STEPSolid_Model* solid;
    STEPVector3 extrusionDir;

    std::shared_ptr<TempMesh> profileMesh;
    std::shared_ptr<TempMesh> profileMesh2D;

    // list of points generated for this opening. This is used to
    // create connections between two opposing holes created
    // from a single opening instance (two because walls tend to
    // have two sides). If !empty(), the other side of the wall
    // has already been processed.
    std::vector<STEPVector3> wallPoints;

    // ------------------------------------------------------------------------------
    TempOpening()
        : solid()
        , extrusionDir()
        , profileMesh()
    {
    }

    // ------------------------------------------------------------------------------
    TempOpening(const STEP::STEPSolid_Model* solid,STEPVector3 extrusionDir,
        std::shared_ptr<TempMesh> profileMesh,
        std::shared_ptr<TempMesh> profileMesh2D)
        : solid(solid)
        , extrusionDir(extrusionDir)
        , profileMesh(profileMesh)
        , profileMesh2D(profileMesh2D)
    {
    }

    // ------------------------------------------------------------------------------
    void Transform(const STEPMatrix4& mat); // defined later since TempMesh is not complete yet



    // ------------------------------------------------------------------------------
    // Helper to sort openings by distance from a given base point
    struct DistanceSorter {

        DistanceSorter(const STEPVector3& base) : base(base) {}

        bool operator () (const TempOpening& a, const TempOpening& b) const {
            return (a.profileMesh->Center()-base).SquareLength() < (b.profileMesh->Center()-base).SquareLength();
        }

        STEPVector3 base;
    };
};


// ------------------------------------------------------------------------------------------------
// Intermediate data storage during conversion. Keeps everything and a bit more.
// ------------------------------------------------------------------------------------------------
struct ConversionData
{
    ConversionData(const STEP::DB& db, const STEP::STEPProduct& prod, aiScene* out,const STEPImporter::Settings& settings)
        : len_scale(1.0)
        , angle_scale(-1.0)
        , db(db)
        , prod(prod)
        , out(out)
        , settings(settings)
    {}

    ~ConversionData() {
        std::for_each(meshes.begin(),meshes.end(),delete_fun<aiMesh>());
        std::for_each(materials.begin(),materials.end(),delete_fun<aiMaterial>());
    }

    STEPFloat len_scale, angle_scale;
    bool plane_angle_in_radians;

    const STEP::DB& db;
    const STEP::STEPProduct& prod;
    aiScene* out;

    STEPMatrix4 wcs;
    std::vector<aiMesh*> meshes;
    std::vector<aiMaterial*> materials;

    struct MeshCacheIndex {
        const STEP::STEPRepresentation_Item* item; unsigned int matindex;
        MeshCacheIndex() : item(NULL), matindex(0) { }
        MeshCacheIndex(const STEP::STEPRepresentation_Item* i, unsigned int mi) : item(i), matindex(mi) { }
        bool operator == (const MeshCacheIndex& o) const { return item == o.item && matindex == o.matindex; }
        bool operator < (const MeshCacheIndex& o) const { return item < o.item || (item == o.item && matindex < o.matindex); }
    };
    typedef std::map<MeshCacheIndex, std::vector<unsigned int> > MeshCache;
    MeshCache cached_meshes;

    typedef std::map<const STEP::STEPSurface_Side_Style*, unsigned int> MaterialCache;
    MaterialCache cached_materials;

    const STEPImporter::Settings& settings;

    std::set<uint64_t> already_processed;
};


// ------------------------------------------------------------------------------------------------
// Binary predicate to compare vectors with a given, quadratic epsilon.
// ------------------------------------------------------------------------------------------------
struct FuzzyVectorCompare {

    FuzzyVectorCompare(STEPFloat epsilon) : epsilon(epsilon) {}
    bool operator()(const STEPVector3& a, const STEPVector3& b) {
        return std::abs((a-b).SquareLength()) < epsilon;
    }

    const STEPFloat epsilon;
};


// ------------------------------------------------------------------------------------------------
// Ordering predicate to totally order R^2 vectors first by x and then by y
// ------------------------------------------------------------------------------------------------
struct XYSorter {

    // sort first by X coordinates, then by Y coordinates
    bool operator () (const STEPVector2&a, const STEPVector2& b) const {
        if (a.x == b.x) {
            return a.y < b.y;
        }
        return a.x < b.x;
    }
};



// conversion routines for common STEP entities, implemented in STEPUtil.cpp
void ConvertColor(aiColor4D& out, const STEPColour_Rgb& in);
void ConvertCartesianPoint(STEPVector3& out, const STEPCartesian_Point& in);
void ConvertDirection(STEPVector3& out, const STEPDirection& in);
void ConvertVector(STEPVector3& out, const STEPVector& in);
void AssignMatrixAxes(STEPMatrix4& out, const STEPVector3& x, const STEPVector3& y, const STEPVector3& z);
void ConvertAxisPlacement(STEPMatrix4& out, const STEPAxis2_Placement_3d& in);
void ConvertAxisPlacement(STEPMatrix4& out, const STEPAxis2_Placement_2d& in);
void ConvertAxisPlacement(STEPVector3& axis, STEPVector3& pos, const STEP::STEPAxis1_Placement& in);
void ConvertAxisPlacement(STEPMatrix4& out, const STEPAxis2_Placement& in, ConversionData& conv);
void ConvertTransformOperator(STEPMatrix4& out, const STEPCartesian_Transformation_Operator& op);
bool IsTrue(const EXPRESS::BOOLEAN& in);
STEPFloat ConvertSIPrefix(const std::string& prefix);

bool ProcessCurve(const STEPCurve& curve,  TempMesh& meshout, ConversionData& conv);

// STEPMaterial.cpp
unsigned int ProcessMaterials(uint64_t id, unsigned int prevMatId, ConversionData& conv, bool forceDefaultMat);

// STEPGeometry.cpp
STEPMatrix3 DerivePlaneCoordinateSpace(const TempMesh& curmesh, bool& ok, STEPVector3& norOut);
bool ProcessRepresentationItem(const STEPRepresentation_Item& item, unsigned int matid, std::vector<unsigned int>& mesh_indices, ConversionData& conv);
void AssignAddedMeshes(std::vector<unsigned int>& mesh_indices,aiNode* nd,ConversionData& /*conv*/);

void ProcessSweptAreaSolid(const STEPSwept_Area_Solid& swept, TempMesh& meshout,
                           ConversionData& conv);

void ProcessExtrudedAreaSolid(const STEPExtruded_Area_Solid& solid, TempMesh& result,
                              ConversionData& conv, bool collect_openings);

// STEPBoolean.cpp

void ProcessBoolean(const STEPBoolean_Result& boolean, TempMesh& result, ConversionData& conv);
void ProcessBooleanHalfSpaceDifference(const STEPHalf_Space_Solid* hs, TempMesh& result,
                                       const TempMesh& first_operand,
                                       ConversionData& conv);

void ProcessBooleanExtrudedAreaSolidDifference(const STEPExtruded_Area_Solid* as, TempMesh& result,
                                               const TempMesh& first_operand,
                                               ConversionData& conv);

// STEPCurve.cpp

// ------------------------------------------------------------------------------------------------
// Custom exception for use by members of the Curve class
// ------------------------------------------------------------------------------------------------
class CurveError
{
public:
    CurveError(const std::string& s)
        : s(s)
    {
    }

    std::string s;
};


// ------------------------------------------------------------------------------------------------
// Temporary representation for an arbitrary sub-class of STEPCurve. Used to sample the curves
// to obtain a list of line segments.
// ------------------------------------------------------------------------------------------------
class Curve
{
protected:

    Curve(const STEPCurve& base_entity, ConversionData& conv)
        : base_entity(base_entity)
        , conv(conv)
    {}

public:

    typedef std::pair<STEPFloat, STEPFloat> ParamRange;

public:


    virtual ~Curve() {}


    // check if a curve is closed
    virtual bool IsClosed() const = 0;

    // evaluate the curve at the given parametric position
    virtual STEPVector3 Eval(STEPFloat p) const = 0;

    // try to match a point on the curve to a given parameter
    // for self-intersecting curves, the result is not ambiguous and
    // it is undefined which parameter is returned.
    virtual bool ReverseEval(const STEPVector3& val, STEPFloat& paramOut) const;

    // get the range of the curve (both inclusive).
    // +inf and -inf are valid return values, the curve is not bounded in such a case.
    virtual std::pair<STEPFloat,STEPFloat> GetParametricRange() const = 0;
    STEPFloat GetParametricRangeDelta() const;

    // estimate the number of sample points that this curve will require
    virtual size_t EstimateSampleCount(STEPFloat start,STEPFloat end) const;

    // intelligently sample the curve based on the current settings
    // and append the result to the mesh
    virtual void SampleDiscrete(TempMesh& out,STEPFloat start,STEPFloat end) const;

#ifdef ASSIMP_BUILD_DEBUG
    // check if a particular parameter value lies within the well-defined range
    bool InRange(STEPFloat) const;
#endif

public:

    static Curve* Convert(const STEP::STEPCurve&,ConversionData& conv);

protected:

    const STEPCurve& base_entity;
    ConversionData& conv;
};


// --------------------------------------------------------------------------------
// A BoundedCurve always holds the invariant that GetParametricRange()
// never returns infinite values.
// --------------------------------------------------------------------------------
class BoundedCurve : public Curve
{
public:

    BoundedCurve(const STEPBounded_Curve& entity, ConversionData& conv)
        : Curve(entity,conv)
    {}

public:

    bool IsClosed() const;

public:

    // sample the entire curve
    void SampleDiscrete(TempMesh& out) const;
    using Curve::SampleDiscrete;
};
}
}

#endif
