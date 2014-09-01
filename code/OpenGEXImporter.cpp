/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2015, assimp team
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
#ifndef ASSIMP_BUILD_NO_OPENGEX_IMPORTER

#include "AssimpPCH.h"
#include "OpenGEXImporter.h"

#include "./../contrib/OpenGEX/OpenGEX.h"

namespace Assimp {
    template<> const std::string LogFunctions<OpenGEX::OpenGEXImporter>::log_prefix = "OpenGEX: ";
}

static const aiImporterDesc desc = {
    "Open Game Engine Exchange",
    "",
    "",
    "",
    aiImporterFlags_SupportTextFlavour,
    0,
    0,
    0,
    0,
    "ogex"
};

using namespace ODDL;
using namespace OGEX;

using std::move;
using std::unique_ptr;

namespace Assimp {
namespace OpenGEX {

namespace {

bool IsObject(const Structure* structure) {
    const StructureType type = structure.GetStructureType();
    return type == kStructureLightObject ||
           type == kStructureGeometryObject ||
           type == kStructureCameraObject ||
           type == kStructureMaterial;
}

bool IsNode(const Structure* structure) {
    const StructureType type = structure.GetStructureType();
    return type == kStructureNode ||
           type == kStructureBoneNode ||
           type == kStructureLightNode ||
           type == kStructureCameraNode ||
           type == kStructureGeometryNode;
}

template <typename Type>
SafeAllocateAndMove(std::vector< std::unique_ptr<T> >& safeElements, Type*** unsafeElements, unsigned int* unsafeElementCount) {
    if (safeElements.empty()) return;
    *unsafeElements = new Type*[safe_elements.size()]();
    for (size_t i = 0; i < safe_elements.size(); ++i)
        *unsafeElements[i] = safe_elements[i].release();
    *unsafeElementCount = static_cast<unsigned int>(safe_elements.size());
}

/** Dummy class to encapsulate the conversion process */
class Converter {
public:

    Converter(aiScene* out, const OpenGexDataDescription& desc)
        : defaultMaterialIndex()
        , out(out)
        , desc(desc)
    {
        ConvertRootNode();

        TransferDataToScene();

        // if we didn't read any meshes set the AI_SCENE_FLAGS_INCOMPLETE
        // to make sure the scene passes assimp's validation. FBX files
        // need not contain geometry (i.e. camera animations, raw armatures).
        if (out->mNumMeshes == 0) {
            out->mFlags |= AI_SCENE_FLAGS_INCOMPLETE;
        }
    }


private:

    void ConvertRootNode() {
        out->mRootNode = new aiNode();
        out->mRootNode->mName.Set("RootNode");
        ConvertObjects(*desc.GetRootStructure());
        ConvertNodes(*desc.GetRootStructure(), *out->mRootNode);
    }

    // Objects are root-level structures (flat in the global hierarchy).
    void ConvertObjects(const Structure& rootStructure) {
        const Structure* structure = rootStructure.GetFirstSubnode();
        while (structure) {
            if (!IsObject(*structure)) continue;

            ConvertObject(*structure);
            structure = structure->Next();
        }
    }

    // Nodes are hierarchical structures.
    void ConvertNodes(const Structure& parentStructure, aiNode& parent) {
        const Structure* structure = parentStructure.GetFirstSubnode();

        std::vector< std::unique_ptr<aiNode> > nodes;
        while (structure) {
            if (!IsNode(*structure)) continue;

            unique_ptr<aiNode> node = ConvertNode(*structure);
            ai_assert(node);
            ConvertNodes(*structure, *node);
            nodes.push_back(move(node));

            structure = structure->Next();
        }

        SafeAllocateAndMove(nodes, &parent.mChildren, &parent.mNumChildren);
    }

    unique_ptr<aiNode> ConvertNode(const Structure* structureNode) {
        if (type == kStructureNode) {
            return ConvertNode(static_cast<const NodeStructure&>(structure));
        }

        if (type == kStructureBoneNode) {
            return ConvertBoneNode(static_cast<const BoneNodeStructure&>(structure));
        }

        if (type == kStructureGeometryNode) {
            return ConvertGeometryNode(static_cast<const GeometryNodeStructure&>(structure));
        }

        if (type == kStructureLightNode) {
            return ConvertLightNode(static_cast<const LightNodeStructure&>(structure));
        }

        if (type == kStructureCameraNode) {
            return ConvertCameraNode(static_cast<const CameraNodeStructure&>(structure));
        }

        ai_assert(false && "Invalid node type");
    }

    void ConvertStructure(const Structure& structure) {
        StructureType type = structure.GetStructureType();

        if (type == kStructureMetric) {
            return ConvertStructureImpl(static_cast<const MetricStructure&>(structure));
        }

        if (type == kStructureVertexArray) {
            return ConvertStructureImpl(static_cast<const VertexArrayStructure&>(structure));
        }

        if (type == kStructureIndexArray) {
            return ConvertStructureImpl(static_cast<const IndexArrayStructure&>(structure));
        }

        if (type == kStructureMesh) {
            return ConvertStructureImpl(static_cast<const MeshStructure&>(structure));
        }

        if (type == kStructureGeometryObject) {
            return ConvertStructureImpl(static_cast<const GeometryObjectStructure&>(structure));
        }

        if (type == kStructureLightObject) {
            return ConvertStructureImpl(static_cast<const LightObjectStructure&>(structure));
        }

        if (type == kStructureCameraObject) {
            return ConvertStructureImpl(static_cast<const CameraObjectStructure&>(structure));
        }

        if (type == kStructureTransform) {
            return ConvertStructureImpl(static_cast<const TransformStructure&>(structure));
        }

        if (type == kStructureTranslation) {
            return ConvertStructureImpl(static_cast<const TranslationStructure&>(structure));
        }

        if (type == kStructureRotation) {
            return ConvertStructureImpl(static_cast<const RotationStructure&>(structure));
        }

        if (type == kStructureScale) {
            return ConvertStructureImpl(static_cast<const ScaleStructure&>(structure));
        }

        if (type == kStructureName) {
            return ConvertStructureImpl(static_cast<const NameStructure&>(structure));
        }

        if (type == kStructureObjectRef) {
            return ConvertStructureImpl(static_cast<const ObjectRefStructure&>(structure));
        }

        if (type == kStructureMaterialRef) {
            return ConvertStructureImpl(static_cast<const MaterialRefStructure&>(structure));
        }

        if (type == kStructureMorph) {
            return ConvertStructureImpl(static_cast<const MorphStructure&>(structure));
        }

        if (type == kStructureBoneRefArray) {
            return ConvertStructureImpl(static_cast<const BoneRefArrayStructure&>(structure));
        }

        if (type == kStructureBoneCountArray) {
            return ConvertStructureImpl(static_cast<const BoneCountArrayStructure&>(structure));
        }

        if (type == kStructureBoneIndexArray) {
            return ConvertStructureImpl(static_cast<const BoneIndexArrayStructure&>(structure));
        }

        if (type == kStructureBoneWeightArray) {
            return ConvertStructureImpl(static_cast<const BoneWeightArrayStructure&>(structure));
        }

        if (type == kStructureSkeleton) {
            return ConvertStructureImpl(static_cast<const SkeletonStructure&>(structure));
        }

        if (type == kStructureSkin) {
            return ConvertStructureImpl(static_cast<const SkinStructure&>(structure));
        }

        if (type == kStructureMaterial) {
            return ConvertStructureImpl(static_cast<const MaterialStructure&>(structure));
        }

        if (type == kStructureParam) {
            return ConvertStructureImpl(static_cast<const ParamStructure&>(structure));
        }

        if (type == kStructureColor) {
            return ConvertStructureImpl(static_cast<const ColorStructure&>(structure));
        }

        if (type == kStructureTexture) {
            return ConvertStructureImpl(static_cast<const TextureStructure&>(structure));
        }

        if (type == kStructureAtten) {
            return ConvertStructureImpl(static_cast<const AttenStructure&>(structure));
        }

        if (type == kStructureKey) {
            return ConvertStructureImpl(static_cast<const KeyStructure&>(structure));
        }

        if (type == kStructureTime) {
            return ConvertStructureImpl(static_cast<const TimeStructure&>(structure));
        }

        if (type == kStructureValue) {
            return ConvertStructureImpl(static_cast<const ValueStructure&>(structure));
        }

        if (type == kStructureTrack) {
            return ConvertStructureImpl(static_cast<const TrackStructure&>(structure));
        }

        if (type == kStructureAnimation) {
            return ConvertStructureImpl(static_cast<const AnimationStructure&>(structure));
        }

    }

    void ConvertObject(const Structure& structure) {
        StructureType type = structure.GetStructureType();

        if (type == kStructureGeometryObject) {
            meshes.push_back(ConvertGeometryObject(static_cast<const GeometryObjectStructure&>(structure)));
        }

        if (type == kStructureLightObject) {
            lights.push_back(ConvertLightObject(static_cast<const LightObjectStructure&>(structure)));
        }

        if (type == kStructureCameraObject) {
            cameras.push_back(ConvertCamerObject(static_cast<const CameraObjectStructure&>(structure)));
        }

        if (type == kStructureMaterial) {
            materials.push_back(ConvertMaterial(static_cast<const MaterialStructure&>(structure)));
        }

        ai_assert(false && "Invalid structure type");
    }

    unique_ptr<aiMesh> ConvertGeometryObject(const GeometryObjectStructure& structure) {
        unique_ptr<aiMesh> mesh = new aiMesh();

        return move(mesh);
    }

    unique_ptr<aiLight> ConvertLightObject(const LightObjectStructure& structure) {
        unique_ptr<aiLight> light = new aiLight();

        return move(light);
    }

    unique_ptr<aiCamera> ConvertCameraObject(const CameraObjectStructure& structure) {
        unique_ptr<aiCamera> camera = new aiCamera();

        return move(camera);
    }

    unique_ptr<aiMaterial> ConvertMaterial(const MaterialStructure& structure) {
        unique_ptr<aiMaterial> material = new aiMaterial();

        return move(material);
    }

    unique_ptr<aiNode> ConvertNode(const NodeStructure& structure) {
        return unique_ptr<aiNode>();
    }

    unique_ptr<aiNode> ConvertBoneNode(const BoneNodeStructure& structure) {
        return unique_ptr<aiNode>();
    }

    unique_ptr<aiNode> ConvertGeometryNode(const GeometryNodeStructure& structure) {
        return unique_ptr<aiNode>();
    }

    unique_ptr<aiNode> ConvertLightNode(const LightNodeStructure& structure) {
        return unique_ptr<aiNode>();
    }

    unique_ptr<aiNode> ConvertCameraNode(const CameraNodeStructure& structure) {
        return unique_ptr<aiNode>();
    }
  
    void ConvertStructureImpl(const Structure& structure) {
        // No implemented.
    }

    void ConvertStructureImpl(const VertexArrayStructure& structure) {}
    void ConvertStructureImpl(const IndexArrayStructure& structure) {}
    void ConvertStructureImpl(const MeshStructure& structure) {}
    void ConvertStructureImpl(const TransformStructure& structure) {}
    void ConvertStructureImpl(const TranslationStructure& structure) {}
    void ConvertStructureImpl(const RotationStructure& structure) {}
    void ConvertStructureImpl(const ScaleStructure& structure) {}
    void ConvertStructureImpl(const NameStructure& structure) {}

    // Move all generated meshes, animations, lights, cameras and textures to the output scene.
    void TransferDataToScene() {
        ai_assert(!out->mMeshes && !out->mNumMeshes);
        SafeAllocateAndMove(mesh, &out->mMeshes, &out->mNumMeshes);
        SafeAllocateAndMove(materials, &out->mMaterials, &out->mNumMaterials);
        SafeAllocateAndMove(animations, &out->mAnimations, &out->mNumAnimations);
        SafeAllocateAndMove(lights, &out->mLights, &out->mNumLights);
        SafeAllocateAndMove(cameras, &out->mCameras, &out->mNumCameras);
    }


private:

    // 0: not assigned yet, others: index is value - 1
    unsigned int defaultMaterialIndex;

    std::vector< std::unique_ptr<aiMesh> > meshes;
    std::vector< std::unique_ptr<aiMaterial> > materials;
    std::vector< std::unique_ptr<aiAnimation> > animations;
    std::vector< std::unique_ptr<aiLight> > lights;
    std::vector< std::unique_ptr<aiCamera> > cameras;

    typedef std::map<const MaterialRefStructure*, unsigned int> MaterialMap;
    MaterialMap materials_converted;

    typedef std::map<const GeometryObjectStructure*, std::vector<unsigned int> > MeshMap;
    MeshMap meshes_converted;

    // name -> has had its prefix_stripped?
    typedef std::map<std::string, bool> NodeNameMap;
    NodeNameMap node_names;

    typedef std::map<std::string, std::string> NameNameMap;
    NameNameMap renamed_nodes;

    aiScene* const out;
    const OpenGexDataDescription& desc;
};

void ConvertToAssimpScene( aiScene *pScene, const OpenGexDataDescription& desc ) {
    Converter conv(pScene, desc);
}

} // namespace

//------------------------------------------------------------------------------------------------
OpenGEXImporter::OpenGEXImporter() {

}

//------------------------------------------------------------------------------------------------
OpenGEXImporter::~OpenGEXImporter() {

}

//------------------------------------------------------------------------------------------------
bool OpenGEXImporter::CanRead( const std::string &file, IOSystem *pIOHandler, bool checkSig ) const {
    if (!checkSig) {
        return SimpleExtensionCheck(file, "ogex");
    }

    static const char *pTokens[] = { "Metric", "GeometryNode" };
    return BaseImporter::SearchFileHeaderForToken(pIOHandler, file, pTokens, 2 );
}

//------------------------------------------------------------------------------------------------
void OpenGEXImporter::InternReadFile( const std::string &file, aiScene *pScene, IOSystem *pIOHandler ) {
    boost::scoped_ptr<IOStream> stream(pIOHandler->Open(file, "rb"));
    if (!stream) {
        ThrowException("Could not open file for reading");
    }

    if (!stream->FileSize()) {
        ThrowException("File is empty");
    }

    const size_t sizeBytes = stream->FileSize();
    std::vector<char> contents(sizeBytes + 1);
    stream->Read(&*contents.begin(), sizeBytes, 1);
    contents[sizeBytes] = 0;

    OpenGexDataDescription openGexDataDescription;
    DataResult result = openGexDataDescription.ProcessText(&*contents.begin());
    if (result != kDataOkay) {
        ThrowException("Failed to load OpenGEX file");
    }

    ConvertToAssimpScene(pScene, openGexDataDescription);
}

//------------------------------------------------------------------------------------------------
const aiImporterDesc *OpenGEXImporter::GetInfo() const {
    return &desc;
}

//------------------------------------------------------------------------------------------------
void OpenGEXImporter::SetupProperties( const Importer *pImp ) {

}

//------------------------------------------------------------------------------------------------

} // Namespace OpenGEX
} // Namespace Assimp

#endif // ASSIMP_BUILD_NO_OPENGEX_IMPORTER
