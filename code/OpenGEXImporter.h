/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2014, assimp team
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
#ifndef AI_OPENGEX_IMPORTER_H
#define AI_OPENGEX_IMPORTER_H

#ifndef ASSIMP_BUILD_NO_OPENGEX_IMPORTER

#include "BaseImporter.h"
#include "LogAux.h"

namespace Assimp {
namespace OpenGEX {

/** @brief  This class is used to implement the OpenGEX importer
 *
 *  See http://opengex.org/OpenGEX.pdf for spec.
 */
class OpenGEXImporter : public BaseImporter, public LogFunctions<OpenGEXImporter> {
public:
    /// The class constructor.
    OpenGEXImporter();

    /// The class destructor.
    virtual ~OpenGEXImporter();

    /// BaseImporter override.
    virtual bool CanRead( const std::string &file, IOSystem *pIOHandler, bool checkSig ) const;

    /// BaseImporter override.
    virtual void InternReadFile( const std::string &file, aiScene *pScene, IOSystem *pIOHandler );

    /// BaseImporter override.
    virtual const aiImporterDesc *GetInfo() const;

    /// BaseImporter override.
    virtual void SetupProperties( const Importer *pImp );
};

} // Namespace OpenGEX
} // Namespace Assimp

#endif // ASSIMP_BUILD_NO_OPENGEX_IMPORTER

#endif // AI_OPENGEX_IMPORTER_H
