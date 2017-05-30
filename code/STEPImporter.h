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

/** @file  STEPImporter.h
 *  @brief Declaration of the STEP loader main class
 */
#ifndef INCLUDED_AI_STEP_IMPORTER_H
#define INCLUDED_AI_STEP_IMPORTER_H

#include "BaseImporter.h"
#include "LogAux.h"

namespace Assimp    {

    // TinyFormatter.h
    namespace Formatter {
        template <typename T,typename TR, typename A> class basic_formatter;
        typedef class basic_formatter< char, std::char_traits<char>, std::allocator<char> > format;
    }

    namespace STEP {
        class DB;
    }


class STEPImporter : public BaseImporter, public LogFunctions<STEPImporter>
{
public:
    STEPImporter();
    ~STEPImporter();


public:

    // --------------------
    bool CanRead( const std::string& pFile,
        IOSystem* pIOHandler,
        bool checkSig
    ) const;

protected:

    // --------------------
    const aiImporterDesc* GetInfo () const;

    // --------------------
    void SetupProperties(const Importer* pImp);

    // --------------------
    void InternReadFile( const std::string& pFile,
        aiScene* pScene,
        IOSystem* pIOHandler
    );

private:


public:


    // loader settings, publicly accessible via their corresponding AI_CONFIG constants
    struct Settings
    {
        Settings()
            : skipSpaceRepresentations()
            , useCustomTriangulation()
            , skipAnnotations()
            , conicSamplingAngle(10.f)
            , cylindricalTessellation(32)
        {}


        bool skipSpaceRepresentations;
        bool useCustomTriangulation;
        bool skipAnnotations;
        float conicSamplingAngle;
        int cylindricalTessellation;
    };


private:

    Settings settings;

}; // !class STEPImporter

} // end of namespace Assimp
#endif // !INCLUDED_AI_STEP_IMPORTER_H
