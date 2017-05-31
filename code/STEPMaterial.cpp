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

/** @file  STEPMaterial.cpp
 *  @brief Implementation of conversion routines to convert STEP materials to aiMaterial
 */



#ifndef ASSIMP_BUILD_NO_STEP_IMPORTER
#include "STEPUtil.h"
#include <limits>
#include <assimp/material.h>

namespace Assimp {
    namespace STEP {

// ------------------------------------------------------------------------------------------------
int ConvertShadingMode(const std::string& name)
{
    if (name == "BLINN") {
        return aiShadingMode_Blinn;
    }
    else if (name == "FLAT" || name == "NOTDEFINED") {
        return aiShadingMode_NoShading;
    }
    else if (name == "PHONG") {
        return aiShadingMode_Phong;
    }
    STEPImporter::LogWarn("shading mode "+name+" not recognized by Assimp, using Phong instead");
    return aiShadingMode_Phong;
}

// ------------------------------------------------------------------------------------------------
void FillMaterial(aiMaterial* mat, const STEP::STEPSurface_Side_Style* style, ConversionData& conv)
{
    aiString name;
    name.Set((style->Name.empty() ? "STEPSurfaceStyle_Unnamed" : style->Name));
    mat->AddProperty(&name,AI_MATKEY_NAME);

    for(std::shared_ptr< const STEP::STEPSurface_Style_Element_Select > el : style->Styles) {
        if (auto surfFill = el->ResolveSelectPtr<STEP::STEPSurface_Style_Fill_Area>(conv.db)) {
            const STEP::STEPFill_Area_Style* const fillAreaStyle = surfFill->FillArea;
            for (std::shared_ptr<const STEP::STEPFill_Style_Select> styleSel : fillAreaStyle->FillStyles) {
                if (auto fillStyleColour = styleSel->ResolveSelectPtr<STEP::STEPFill_Area_Style_Colour>(conv.db)) {
                    aiColor4D colour;
                    auto fillColour = fillStyleColour->FillColour->ToPtr<STEP::STEPColour_Rgb>();
                    if (fillColour) {
                        ConvertColor(colour, *fillColour);
                        mat->AddProperty(&colour, 1, AI_MATKEY_COLOR_DIFFUSE);
                    }
                }
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------
unsigned int ProcessMaterials(uint64_t id, unsigned int prevMatId, ConversionData& conv, bool forceDefaultMat)
{
    STEP::DB::RefMapRange range = conv.db.GetRefs().equal_range(id);
    for(;range.first != range.second; ++range.first) {
        if(const STEP::STEPStyled_Item* const styled = conv.db.GetObject((*range.first).second)->ToPtr<STEP::STEPStyled_Item>()) {
            for(const STEP::STEPPresentation_Style_Assignment& as : styled->Styles) {
                for(std::shared_ptr<const STEP::STEPPresentation_Style_Select> sel : as.Styles) {

                    if (const STEP::STEPSurface_Style_Usage* const usage = sel->ResolveSelectPtr<STEP::STEPSurface_Style_Usage>(conv.db)) {
                        // Create new material
                        const std::string side = static_cast<std::string>(usage->Side);
                        if (side != "BOTH")
                            STEPImporter::LogWarn("Ignoring surface side marker on surface_style_usage: " + side);

                        auto sideStyleSel = usage->Style;
                        if (auto predefined = sideStyleSel->ResolveSelectPtr<STEP::STEPPre_Defined_Surface_Side_Style>(conv.db)) {
                            STEPImporter::LogWarn("Ignoring predefined surface side style: " + predefined->Name.empty() ? "Unnamed" : predefined->Name);
                            return std::numeric_limits<uint32_t>::max();
                        }

                        const STEP::STEPSurface_Side_Style* const style = sideStyleSel->ResolveSelectPtr<STEP::STEPSurface_Side_Style>(conv.db);

                        ConversionData::MaterialCache::iterator mit = conv.cached_materials.find(style);
                        if (mit != conv.cached_materials.end())
                            return mit->second;

                        std::unique_ptr<aiMaterial> mat(new aiMaterial());

                        FillMaterial(mat.get(), style, conv);

                        conv.materials.push_back(mat.release());

                        unsigned int idx = static_cast<unsigned int>(conv.materials.size() - 1);
                        conv.cached_materials[style] = idx;
                        return idx;
                    }
                }
            }
        }
    }

    // no local material defined. If there's global one, use that instead
    if( prevMatId != std::numeric_limits<uint32_t>::max() )
        return prevMatId;

    // we're still here - create an default material if required, or simply fail otherwise
    if( !forceDefaultMat )
        return std::numeric_limits<uint32_t>::max();

    aiString name;
    name.Set("<STEPDefault>");
    //  ConvertColorToString( color, name);

    // look if there's already a default material with this base color
    for( size_t a = 0; a < conv.materials.size(); ++a )
    {
        aiString mname;
        conv.materials[a]->Get(AI_MATKEY_NAME, mname);
        if( name == mname )
            return (unsigned int)a;
    }

    // we're here, yet - no default material with suitable color available. Generate one
    std::unique_ptr<aiMaterial> mat(new aiMaterial());
    mat->AddProperty(&name,AI_MATKEY_NAME);

    const aiColor4D col = aiColor4D( 0.6f, 0.6f, 0.6f, 1.0f); // aiColor4D( color.r, color.g, color.b, 1.0f);
    mat->AddProperty(&col,1, AI_MATKEY_COLOR_DIFFUSE);

    conv.materials.push_back(mat.release());
    return (unsigned int) conv.materials.size() - 1;
}

} // ! STEP
} // ! Assimp

#endif
