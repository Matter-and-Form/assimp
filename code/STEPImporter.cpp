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

/** @file  STEPImporter.cpp
 *  @brief Implementation of the STEP loader.
 */


#ifndef ASSIMP_BUILD_NO_STEP_IMPORTER

#include <iterator>
#include <limits>
#include <tuple>

#include "STEPFile.h"
#include "STEPImporter.h"
#include "STEPFileReader.h"
#include "STEPUtil.h"

#include "MemoryIOWrapper.h"
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/importerdesc.h>


namespace Assimp {
    template<> const std::string LogFunctions<STEPImporter>::log_prefix = "STEP: ";
}

using namespace Assimp;
using namespace Assimp::Formatter;
using namespace Assimp::STEP;

/* DO NOT REMOVE this comment block. The genentitylist.sh script
 * just looks for names adhering to the IfcSomething naming scheme
 * and includes all matches in the whitelist for code-generation. Thus,
 * all entity classes that are only indirectly referenced need to be
 * mentioned explicitly.

  STEPRepresentation_Map
  STEPAdvanced_Face

 */

static const aiImporterDesc desc = {
    "STEP Importer",
    "",
    "",
    "",
    0,
    0,
    0,
    0,
    0,
    "stp step"
};

namespace {

// forward declarations
//void SetUnits(ConversionData& conv);
//void SetCoordinateSpace(ConversionData& conv);
void ProcessShapes(ConversionData &conv);
//void MakeTreeRelative(ConversionData& conv);
//void ConvertUnit(const EXPRESS::DataType& dt,ConversionData& conv);

}

// ------------------------------------------------------------------------------------------------
// Constructor to be privately used by Importer
STEPImporter::STEPImporter()
{}

// ------------------------------------------------------------------------------------------------
// Destructor, private as well
STEPImporter::~STEPImporter()
{
}

// ------------------------------------------------------------------------------------------------
// Returns whether the class can handle the format of the given file.
bool STEPImporter::CanRead( const std::string& pFile, IOSystem* pIOHandler, bool checkSig) const
{
    const std::string& extension = GetExtension(pFile);
    if (extension == "stp" || extension == "step" ) {
        return true;
    } else if ((!extension.length() || checkSig) && pIOHandler)   {
        // note: this is the common identification for STEP-encoded files, so
        // it is only unambiguous as long as we don't support any further
        // file formats with STEP as their encoding.
        const char* tokens[] = {"ISO-10303-21"};
        return SearchFileHeaderForToken(pIOHandler,pFile,tokens,1);
    }
    return false;
}

// ------------------------------------------------------------------------------------------------
// List all extensions handled by this loader
const aiImporterDesc* STEPImporter::GetInfo () const
{
    return &desc;
}


// ------------------------------------------------------------------------------------------------
// Setup configuration properties for the loader
void STEPImporter::SetupProperties(const Importer* pImp)
{

}

// ------------------------------------------------------------------------------------------------
// Imports the given file into the given scene structure.
void STEPImporter::InternReadFile( const std::string& pFile,
    aiScene* pScene, IOSystem* pIOHandler)
{
    std::shared_ptr<IOStream> stream(pIOHandler->Open(pFile));
    if (!stream) {
        ThrowException("Could not open file for reading");
    }

    std::unique_ptr<STEP::DB> db(STEP::ReadFileHeader(stream));
    const STEP::HeaderInfo& head = static_cast<const STEP::DB&>(*db).GetHeader();

    if(!head.fileSchema.size()) {
        ThrowException("Unrecognized file schema: " + head.fileSchema);
    }

    if (!DefaultLogger::isNullLogger()) {
        LogDebug("File schema is \'" + head.fileSchema + '\'');
        if (head.timestamp.length()) {
            LogDebug("Timestamp \'" + head.timestamp + '\'');
        }
        if (head.app.length()) {
            LogDebug("Application/Exporter identline is \'" + head.app  + '\'');
        }
    }

    // obtain a copy of the machine-generated EXPRESS scheme
    EXPRESS::ConversionSchema schema;
    GetSchema(schema);

    // tell the reader to index "shape_definition_representation"s by type instead of index
    static const char* const types_to_track[] = {
        "shape_definition_representation"
    };

    // feed the schema into the reader and pre-parse all lines
    STEP::ReadFile(*db, schema, types_to_track, 1, NULL, 0);

    const STEP::LazyObject *shapeDef = db->GetObject("shape_definition_representation");
    if (!shapeDef)
        ThrowException("Missing shape definition");

    ConversionData conv(*db, pScene, settings);
    //SetUnits(conv);
    //SetCoordinateSpace(conv);
    ProcessShapes(conv);
    //MakeTreeRelative(conv);

    // do final data copying
    if (conv.meshes.size()) {
        pScene->mNumMeshes = static_cast<unsigned int>(conv.meshes.size());
        pScene->mMeshes = new aiMesh*[pScene->mNumMeshes]();
        std::copy(conv.meshes.begin(),conv.meshes.end(),pScene->mMeshes);

        // needed to keep the d'tor from burning us
        conv.meshes.clear();
    }

    if (conv.materials.size()) {
        pScene->mNumMaterials = static_cast<unsigned int>(conv.materials.size());
        pScene->mMaterials = new aiMaterial*[pScene->mNumMaterials]();
        std::copy(conv.materials.begin(),conv.materials.end(),pScene->mMaterials);

        // needed to keep the d'tor from burning us
        conv.materials.clear();
    }

    // apply world coordinate system (which includes the scaling to convert to meters and a -90 degrees rotation around x)
    aiMatrix4x4 scale, rot;
    aiMatrix4x4::Scaling(static_cast<aiVector3D>(STEPVector3(conv.len_scale)),scale);
    aiMatrix4x4::RotationX(-AI_MATH_HALF_PI_F,rot);

    pScene->mRootNode->mTransformation = rot * scale * conv.wcs * pScene->mRootNode->mTransformation;

    // this must be last because objects are evaluated lazily as we process them
    if ( !DefaultLogger::isNullLogger() ){
        LogDebug((Formatter::format(),"STEP: evaluated ",db->GetEvaluatedObjectCount()," object records"));
    }
}

namespace {


/*
// ------------------------------------------------------------------------------------------------
void ConvertUnit(const STEPNamed_Unit& unit,ConversionData& conv)
{
    if(const STEPSi_Unit* const si = unit.ToPtr<STEPSi_Unit>()) {

        if(si->UnitType == "LENGTHUNIT") {
            conv.len_scale = si->Prefix ? ConvertSIPrefix(si->Prefix) : 1.f;
            STEPImporter::LogDebug("got units used for lengths");
        }
        if(si->UnitType == "PLANEANGLEUNIT") {
            if (si->Name != "RADIAN") {
                STEPImporter::LogWarn("expected base unit for angles to be radian");
            }
        }
    }
    else if(const STEPConversion_Based_Unit* const convu = unit.ToPtr<STEPConversion_Based_Unit>()) {

        if(convu->UnitType == "PLANEANGLEUNIT") {
            try {
                conv.angle_scale = convu->ConversionFactor->ValueComponent->To<EXPRESS::REAL>();
                ConvertUnit(*convu->ConversionFactor->UnitComponent,conv);
                STEPImporter::LogDebug("got units used for angles");
            }
            catch(std::bad_cast&) {
                STEPImporter::LogError("skipping unknown STEPConversion_Based_Unit.ValueComponent entry - expected REAL");
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------
void ConvertUnit(const EXPRESS::DataType& dt,ConversionData& conv)
{
    try {
        const EXPRESS::ENTITY& e = dt.To<ENTITY>();

        const STEPNamed_Unit& unit = e.ResolveSelect<STEPNamed_Unit>(conv.db);
        if(unit.UnitType != "LENGTHUNIT" && unit.UnitType != "PLANEANGLEUNIT") {
            return;
        }

        ConvertUnit(unit,conv);
    }
    catch(std::bad_cast&) {
        // not entity, somehow
        STEPImporter::LogError("skipping unknown STEPUnit entry - expected entity");
    }
}

// ------------------------------------------------------------------------------------------------
void SetUnits(ConversionData& conv)
{
    // see if we can determine the coordinate space used to express.
    for(size_t i = 0; i <  conv.proj.UnitsInContext->Units.size(); ++i ) {
        ConvertUnit(*conv.proj.UnitsInContext->Units[i],conv);
    }
}

// ------------------------------------------------------------------------------------------------
void SetCoordinateSpace(ConversionData& conv)
{
    const STEPRepresentation_Context* fav = NULL;
    for(const STEPRepresentation_Context& v : conv.proj.RepresentationContexts) {
        fav = &v;
        // Model should be the most suitable type of context, hence ignore the others
        if (v.ContextType && v.ContextType.Get() == "Model") {
            break;
        }
    }
    if (fav) {
        if(const STEPGeometric_Representation_Context* const geo = fav->ToPtr<STEPGeometric_Representation_Context>()) {
            ConvertAxisPlacement(conv.wcs, *geo->WorldCoordinateSystem, conv);
            STEPImporter::LogDebug("got world coordinate system");
        }
    }
}

// ------------------------------------------------------------------------------------------------
void ResolveObjectPlacement(aiMatrix4x4& m, const STEPObject_Placement& place, ConversionData& conv)
{
    if (const STEPLocal_Placement* const local = place.ToPtr<STEPLocal_Placement>()){
        STEPMatrix4 tmp;
        ConvertAxisPlacement(tmp, *local->RelativePlacement, conv);

        m = static_cast<aiMatrix4x4>(tmp);

        if (local->PlacementRelTo) {
            aiMatrix4x4 tmp;
            ResolveObjectPlacement(tmp,local->PlacementRelTo.Get(),conv);
            m = tmp * m;
        }
    }
    else {
        STEPImporter::LogWarn("skipping unknown STEPObject_Placement entity, type is " + place.GetClassName());
    }
}
*/

// ------------------------------------------------------------------------------------------------
void ProcessShapes(ConversionData& conv)
{
    const STEP::DB::ObjectMapByType& objectsByType = conv.db.GetObjectsByType();

    ai_assert(objectsByType.find("shape_definition_representation") != objectsByType.end());

    const STEP::DB::ObjectSet* shapeDefReps = &objectsByType.find("shape_definition_representation")->second;

    if (shapeDefReps->empty())
        STEPImporter::ThrowException("No shape definitions found");

    conv.out->mRootNode = new aiNode();

    for(const STEP::LazyObject* lz : *shapeDefReps) {
        auto shapeDefRep = lz->ToPtr<STEPShape_Definition_Representation>();
        for (auto repItem : shapeDefRep->UsedRepresentation->Items) {
            std::vector<unsigned int> meshes;
            if (ProcessRepresentationItem(repItem, 0, meshes, conv))
                AssignAddedMeshes(meshes, conv.out->mRootNode, conv);
        }
    }
}

// ------------------------------------------------------------------------------------------------
bool ProcessMappedItem(const STEPMapped_Item& mapped, aiNode* nd_src, std::vector< aiNode* >& subnodes_src, unsigned int matid, ConversionData& conv)
{
    // insert a custom node here, the cartesian transform operator is simply a conventional transformation matrix
    std::unique_ptr<aiNode> nd(new aiNode());
    nd->mName.Set("STEPMapped_Item");

    // handle the Cartesian operator
    STEPMatrix4 m;
    //ConvertTransformOperator(m, *mapped.MappingTarget);

    STEPMatrix4 msrc;
    //ConvertAxisPlacement(msrc,*mapped.MappingSource->MappingOrigin,conv);

    msrc = m*msrc;

    std::vector<unsigned int> meshes;

    unsigned int localmatid = ProcessMaterials(mapped.GetID(),matid,conv,false);
    const STEPRepresentation& repr = mapped.MappingSource->MappedRepresentation;

    bool got = false;
    for(const STEPRepresentation_Item& item : repr.Items) {
        if(!ProcessRepresentationItem(item,localmatid,meshes,conv)) {
            STEPImporter::LogWarn("skipping mapped entity of type " + item.GetClassName() + ", no representations could be generated");
        }
        else got = true;
    }

    if (!got) {
        return false;
    }

    AssignAddedMeshes(meshes,nd.get(),conv);

    nd->mTransformation =  nd_src->mTransformation * static_cast<aiMatrix4x4>( msrc );
    subnodes_src.push_back(nd.release());

    return true;
}

// ------------------------------------------------------------------------------------------------
struct RateRepresentationPredicate {

    int Rate(const STEPRepresentation* r) const {
        // the smaller, the better

        if (r->Name.empty()) {
            // neutral choice if no extra information is specified
            return 0;
        }


        const std::string& name = r->Name;
        if (name == "MappedRepresentation") {
            if (!r->Items.empty()) {
                // take the first item and base our choice on it
                const STEPMapped_Item* const m = r->Items.front()->ToPtr<STEPMapped_Item>();
                if (m) {
                    return Rate(m->MappingSource->MappedRepresentation);
                }
            }
            return 100;
        }

        return Rate(name);
    }

    int Rate(const std::string& r) const {


        if (r == "SolidModel") {
            return -3;
        }

        // give strong preference to extruded geometry.
        if (r == "SweptSolid") {
            return -10;
        }

        if (r == "Clipping") {
            return -5;
        }

        // 'Brep' is difficult to get right due to possible voids in the
        // polygon boundaries, so take it only if we are forced to (i.e.
        // if the only alternative is (non-clipping) boolean operations,
        // which are not supported at all).
        if (r == "Brep") {
            return -2;
        }

        // Curves, bounding boxes - those will most likely not be loaded
        // as we can't make any use out of this data. So consider them
        // last.
        if (r == "BoundingBox" || r == "Curve2D") {
            return 100;
        }
        return 0;
    }

    bool operator() (const STEPRepresentation* a, const STEPRepresentation* b) const {
        return Rate(a) < Rate(b);
    }
};

// ------------------------------------------------------------------------------------------------
void MakeTreeRelative(aiNode* start, const aiMatrix4x4& combined)
{
    // combined is the parent's absolute transformation matrix
    const aiMatrix4x4 old = start->mTransformation;

    if (!combined.IsIdentity()) {
        start->mTransformation = aiMatrix4x4(combined).Inverse() * start->mTransformation;
    }

    // All nodes store absolute transformations right now, so we need to make them relative
    for (unsigned int i = 0; i < start->mNumChildren; ++i) {
        MakeTreeRelative(start->mChildren[i],old);
    }
}

// ------------------------------------------------------------------------------------------------
void MakeTreeRelative(ConversionData& conv)
{
    MakeTreeRelative(conv.out->mRootNode,STEPMatrix4());
}

} // !anon



#endif
