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

namespace Assimp {
namespace OpenGEX {

namespace {

template<typename T>
struct delete_fun
{
    void operator()(const volatile T* del) {
        delete del;
    }
};

/** Dummy class to encapsulate the conversion process */
class Converter
{
public:

    /** the different parts that make up the final local transformation of a fbx node */
    enum TransformationComp
    {
        TransformationComp_Translation = 0,
        TransformationComp_RotationOffset,
        TransformationComp_RotationPivot,
        TransformationComp_PreRotation,
        TransformationComp_Rotation,
        TransformationComp_PostRotation,
        TransformationComp_RotationPivotInverse,
        TransformationComp_ScalingOffset,
        TransformationComp_ScalingPivot,
        TransformationComp_Scaling,
        TransformationComp_ScalingPivotInverse,
        TransformationComp_GeometricTranslation,
        TransformationComp_GeometricRotation,
        TransformationComp_GeometricScaling,

        TransformationComp_MAXIMUM
    };

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

    ~Converter()
    {
        std::for_each(meshes.begin(), meshes.end(), delete_fun<aiMesh>());
        std::for_each(materials.begin(), materials.end(), delete_fun<aiMaterial>());
        std::for_each(animations.begin(), animations.end(), delete_fun<aiAnimation>());
        std::for_each(lights.begin(), lights.end(), delete_fun<aiLight>());
        std::for_each(cameras.begin(), cameras.end(), delete_fun<aiCamera>());
    }


private:

    // ------------------------------------------------------------------------------------------------
    // find scene root and trigger recursive scene conversion
    void ConvertRootNode()
    {
        out->mRootNode = new aiNode();
        out->mRootNode->mName.Set("RootNode");
        ConvertChildren(desc.GetRootStructure(), *out->mRootNode);
    }

    // collect and assign child nodes
    void ConvertChildren(const Structure* structureParent, aiNode& parent, const aiMatrix4x4& parent_transform = aiMatrix4x4())
    {
        ai_assert(structureParent);
        const Structure* structure = structureParent->GetFirstSubnode();

        std::vector<aiNode*> nodes;
        std::vector<aiNode*> nodes_chain;

        try {
            while (structure) {
                ConvertChildren(structure, parent);
                ConvertStructure(*structure);
                /*
                ConvertChildren(structure, *nodes_chain.back(), new_abs_transform);
                const Model* const model = dynamic_cast<const Model*>(object);

                if(model) {
                    nodes_chain.clear();

                    aiMatrix4x4 new_abs_transform = parent_transform;

                    // even though there is only a single input node, the design of
                    // assimp (or rather: the complicated transformation chain that
                    // is employed by fbx) means that we may need multiple aiNode's
                    // to represent a fbx node's transformation.
                    GenerateTransformationNodeChain(*model,nodes_chain);

                    ai_assert(nodes_chain.size());

                    const std::string& original_name = FixNodeName(model->Name());

                    // check if any of the nodes in the chain has the name the fbx node
                    // is supposed to have. If there is none, add another node to 
                    // preserve the name - people might have scripts etc. that rely
                    // on specific node names.
                    aiNode* name_carrier = NULL;
                    BOOST_FOREACH(aiNode* prenode, nodes_chain) {
                        if ( !strcmp(prenode->mName.C_Str(), original_name.c_str()) ) {
                            name_carrier = prenode;
                            break;
                        }
                    }

                    if(!name_carrier) {
                        nodes_chain.push_back(new aiNode(original_name));
                        name_carrier = nodes_chain.back();
                    }

                    //setup metadata on newest node
                    SetupNodeMetadata(*model, *nodes_chain.back());

                    // link all nodes in a row
                    aiNode* last_parent = &parent;
                    BOOST_FOREACH(aiNode* prenode, nodes_chain) {
                        ai_assert(prenode);

                        if(last_parent != &parent) {
                            last_parent->mNumChildren = 1;
                            last_parent->mChildren = new aiNode*[1];
                            last_parent->mChildren[0] = prenode;
                        }

                        prenode->mParent = last_parent;
                        last_parent = prenode;

                        new_abs_transform *= prenode->mTransformation;
                    }

                    // attach geometry
                    ConvertModel(*model, *nodes_chain.back(), new_abs_transform);

                    // attach sub-nodes
                    ConvertNodes(model->ID(), *nodes_chain.back(), new_abs_transform);

                    if(doc.Settings().readLights) {
                        ConvertLights(*model);
                    }

                    if(doc.Settings().readCameras) {
                        ConvertCameras(*model);
                    }

                    nodes.push_back(nodes_chain.front());
                    nodes_chain.clear();
                }
                */
                structure = structure->Next();
            }

            if(nodes.size()) {
                parent.mChildren = new aiNode*[nodes.size()]();
                parent.mNumChildren = static_cast<unsigned int>(nodes.size());

                std::swap_ranges(nodes.begin(),nodes.end(),parent.mChildren);
            }
        }
        catch(std::exception&)  {
            delete_fun<aiNode> deleter;
            std::for_each(nodes.begin(), nodes.end(), deleter);
            std::for_each(nodes_chain.begin(), nodes_chain.end(), deleter);
        }
    }


    void ConvertStructure(const Structure& structure) {
        StructureType type = structure.GetStructureType();

        if (type == kStructureMetric)
        {
            return ConvertStructureImpl(static_cast<const MetricStructure&>(structure));
        }

        if (type == kStructureVertexArray)
        {
            return ConvertStructureImpl(static_cast<const VertexArrayStructure&>(structure));
        }

        if (type == kStructureIndexArray)
        {
            return ConvertStructureImpl(static_cast<const IndexArrayStructure&>(structure));
        }

        if (type == kStructureMesh)
        {
            return ConvertStructureImpl(static_cast<const MeshStructure&>(structure));
        }

        if (type == kStructureNode)
        {
            return ConvertStructureImpl(static_cast<const NodeStructure&>(structure));
        }

        if (type == kStructureBoneNode)
        {
            return ConvertStructureImpl(static_cast<const BoneNodeStructure&>(structure));
        }

        if (type == kStructureGeometryNode)
        {
            return ConvertStructureImpl(static_cast<const GeometryNodeStructure&>(structure));
        }

        if (type == kStructureLightNode)
        {
            return ConvertStructureImpl(static_cast<const LightNodeStructure&>(structure));
        }

        if (type == kStructureCameraNode)
        {
            return ConvertStructureImpl(static_cast<const CameraNodeStructure&>(structure));
        }

        if (type == kStructureGeometryObject)
        {
            return ConvertStructureImpl(static_cast<const GeometryObjectStructure&>(structure));
        }

        if (type == kStructureLightObject)
        {
            return ConvertStructureImpl(static_cast<const LightObjectStructure&>(structure));
        }

        if (type == kStructureCameraObject)
        {
            return ConvertStructureImpl(static_cast<const CameraObjectStructure&>(structure));
        }

        if (type == kStructureTransform)
        {
            return ConvertStructureImpl(static_cast<const TransformStructure&>(structure));
        }

        if (type == kStructureTranslation)
        {
            return ConvertStructureImpl(static_cast<const TranslationStructure&>(structure));
        }

        if (type == kStructureRotation)
        {
            return ConvertStructureImpl(static_cast<const RotationStructure&>(structure));
        }

        if (type == kStructureScale)
        {
            return ConvertStructureImpl(static_cast<const ScaleStructure&>(structure));
        }

        if (type == kStructureName)
        {
            return ConvertStructureImpl(static_cast<const NameStructure&>(structure));
        }

        if (type == kStructureObjectRef)
        {
            return ConvertStructureImpl(static_cast<const ObjectRefStructure&>(structure));
        }

        if (type == kStructureMaterialRef)
        {
            return ConvertStructureImpl(static_cast<const MaterialRefStructure&>(structure));
        }

        if (type == kStructureMorph)
        {
            return ConvertStructureImpl(static_cast<const MorphStructure&>(structure));
        }

        if (type == kStructureBoneRefArray)
        {
            return ConvertStructureImpl(static_cast<const BoneRefArrayStructure&>(structure));
        }

        if (type == kStructureBoneCountArray)
        {
            return ConvertStructureImpl(static_cast<const BoneCountArrayStructure&>(structure));
        }

        if (type == kStructureBoneIndexArray)
        {
            return ConvertStructureImpl(static_cast<const BoneIndexArrayStructure&>(structure));
        }

        if (type == kStructureBoneWeightArray)
        {
            return ConvertStructureImpl(static_cast<const BoneWeightArrayStructure&>(structure));
        }

        if (type == kStructureSkeleton)
        {
            return ConvertStructureImpl(static_cast<const SkeletonStructure&>(structure));
        }

        if (type == kStructureSkin)
        {
            return ConvertStructureImpl(static_cast<const SkinStructure&>(structure));
        }

        if (type == kStructureMaterial)
        {
            return ConvertStructureImpl(static_cast<const MaterialStructure&>(structure));
        }

        if (type == kStructureParam)
        {
            return ConvertStructureImpl(static_cast<const ParamStructure&>(structure));
        }

        if (type == kStructureColor)
        {
            return ConvertStructureImpl(static_cast<const ColorStructure&>(structure));
        }

        if (type == kStructureTexture)
        {
            return ConvertStructureImpl(static_cast<const TextureStructure&>(structure));
        }

        if (type == kStructureAtten)
        {
            return ConvertStructureImpl(static_cast<const AttenStructure&>(structure));
        }

        if (type == kStructureKey)
        {
            return ConvertStructureImpl(static_cast<const KeyStructure&>(structure));
        }

        if (type == kStructureTime)
        {
            return ConvertStructureImpl(static_cast<const TimeStructure&>(structure));
        }

        if (type == kStructureValue)
        {
            return ConvertStructureImpl(static_cast<const ValueStructure&>(structure));
        }

        if (type == kStructureTrack)
        {
            return ConvertStructureImpl(static_cast<const TrackStructure&>(structure));
        }

        if (type == kStructureAnimation)
        {
            return ConvertStructureImpl(static_cast<const AnimationStructure&>(structure));
        }

    }


    void ConvertStructureImpl(const Structure& structure) {
        // No implemented.
    }

    void ConvertStructureImpl(const VertexArrayStructure& structure) {}
    void ConvertStructureImpl(const IndexArrayStructure& structure) {}
    void ConvertStructureImpl(const MeshStructure& structure) {}
    void ConvertStructureImpl(const NodeStructure& structure) {}
    void ConvertStructureImpl(const BoneNodeStructure& structure) {}
    void ConvertStructureImpl(const GeometryNodeStructure& structure) {}
    void ConvertStructureImpl(const LightNodeStructure& structure) {}
    void ConvertStructureImpl(const CameraNodeStructure& structure) {}
    void ConvertStructureImpl(const GeometryObjectStructure& structure) {}
    void ConvertStructureImpl(const LightObjectStructure& structure) {}
    void ConvertStructureImpl(const CameraObjectStructure& structure) {}
    void ConvertStructureImpl(const TransformStructure& structure) {}
    void ConvertStructureImpl(const TranslationStructure& structure) {}
    void ConvertStructureImpl(const RotationStructure& structure) {}
    void ConvertStructureImpl(const ScaleStructure& structure) {}
    void ConvertStructureImpl(const NameStructure& structure) {}

    /*
    // ------------------------------------------------------------------------------------------------
    void ConvertLights(const Model& model)
    {
        const std::vector<const NodeAttribute*>& node_attrs = model.GetAttributes();
        BOOST_FOREACH(const NodeAttribute* attr, node_attrs) {
            const Light* const light = dynamic_cast<const Light*>(attr);
            if(light) {
                ConvertLight(model, *light);
            }
        }
    }


    // ------------------------------------------------------------------------------------------------
    void ConvertCameras(const Model& model)
    {
        const std::vector<const NodeAttribute*>& node_attrs = model.GetAttributes();
        BOOST_FOREACH(const NodeAttribute* attr, node_attrs) {
            const Camera* const cam = dynamic_cast<const Camera*>(attr);
            if(cam) {
                ConvertCamera(model, *cam);
            }
        }
    }


    // ------------------------------------------------------------------------------------------------
    void ConvertLight(const Model& model, const Light& light)
    {
        lights.push_back(new aiLight());
        aiLight* const out_light = lights.back();

        out_light->mName.Set(FixNodeName(model.Name()));

        const float intensity = light.Intensity();
        const aiVector3D& col = light.Color();

        out_light->mColorDiffuse = aiColor3D(col.x,col.y,col.z);
        out_light->mColorDiffuse.r *= intensity;
        out_light->mColorDiffuse.g *= intensity;
        out_light->mColorDiffuse.b *= intensity;

        out_light->mColorSpecular = out_light->mColorDiffuse;

        switch(light.LightType())
        {
        case Light::Type_Point:
            out_light->mType = aiLightSource_POINT;
            break;

        case Light::Type_Directional:
            out_light->mType = aiLightSource_DIRECTIONAL;
            break;

        case Light::Type_Spot:
            out_light->mType = aiLightSource_SPOT;
            out_light->mAngleOuterCone = AI_DEG_TO_RAD(light.OuterAngle());
            out_light->mAngleInnerCone = AI_DEG_TO_RAD(light.InnerAngle());
            break;

        case Light::Type_Area:
            FBXImporter::LogWarn("cannot represent area light, set to UNDEFINED");
            out_light->mType = aiLightSource_UNDEFINED;
            break;

        case Light::Type_Volume:
            FBXImporter::LogWarn("cannot represent volume light, set to UNDEFINED");
            out_light->mType = aiLightSource_UNDEFINED;
            break;
        default:
            ai_assert(false);
        }

        // XXX: how to best convert the near and far decay ranges?
        switch(light.DecayType())
        {
        case Light::Decay_None:
            out_light->mAttenuationConstant = 1.0f;
            break;
        case Light::Decay_Linear:
            out_light->mAttenuationLinear = 1.0f;
            break;
        case Light::Decay_Quadratic:
            out_light->mAttenuationQuadratic = 1.0f;
            break;
        case Light::Decay_Cubic:
            FBXImporter::LogWarn("cannot represent cubic attenuation, set to Quadratic");
            out_light->mAttenuationQuadratic = 1.0f;
            break;
        default:
            ai_assert(false);
        }
    }


    // ------------------------------------------------------------------------------------------------
    void ConvertCamera(const Model& model, const Camera& cam)
    {
        cameras.push_back(new aiCamera());
        aiCamera* const out_camera = cameras.back();

        out_camera->mName.Set(FixNodeName(model.Name()));

        out_camera->mAspect = cam.AspectWidth() / cam.AspectHeight();
        out_camera->mPosition = cam.Position();
        out_camera->mLookAt = cam.InterestPosition() - out_camera->mPosition;

        // BUG HERE cam.FieldOfView() returns 1.0f every time.  1.0f is default value.
        out_camera->mHorizontalFOV = AI_DEG_TO_RAD(cam.FieldOfView());
    }

    // ------------------------------------------------------------------------------------------------
    aiVector3D TransformationCompDefaultValue(TransformationComp comp)
    {
        // XXX a neat way to solve the never-ending special cases for scaling 
        // would be to do everything in log space!
        return comp == TransformationComp_Scaling ? aiVector3D(1.f,1.f,1.f) : aiVector3D();
    }


    // ------------------------------------------------------------------------------------------------
    void GetRotationMatrix(Model::RotOrder mode, const aiVector3D& rotation, aiMatrix4x4& out)
    {
        if(mode == Model::RotOrder_SphericXYZ) {
            FBXImporter::LogError("Unsupported RotationMode: SphericXYZ");
            out = aiMatrix4x4();
            return;
        }

        const float angle_epsilon = 1e-6f;

        out = aiMatrix4x4();

        bool is_id[3] = { true, true, true };

        aiMatrix4x4 temp[3];
        if(fabs(rotation.z) > angle_epsilon) {
            aiMatrix4x4::RotationZ(AI_DEG_TO_RAD(rotation.z),temp[2]);
            is_id[2] = false;
        }
        if(fabs(rotation.y) > angle_epsilon) {
            aiMatrix4x4::RotationY(AI_DEG_TO_RAD(rotation.y),temp[1]);
            is_id[1] = false;
        }
        if(fabs(rotation.x) > angle_epsilon) {
            aiMatrix4x4::RotationX(AI_DEG_TO_RAD(rotation.x),temp[0]);
            is_id[0] = false;
        }

        int order[3] = {-1, -1, -1};

        // note: rotation order is inverted since we're left multiplying as is usual in assimp
        switch(mode)
        {
        case Model::RotOrder_EulerXYZ:
            order[0] = 2;
            order[1] = 1;
            order[2] = 0;
            break;

        case Model::RotOrder_EulerXZY: 
            order[0] = 1;
            order[1] = 2;
            order[2] = 0;
            break;

        case Model::RotOrder_EulerYZX:
            order[0] = 0;
            order[1] = 2;
            order[2] = 1;
            break;

        case Model::RotOrder_EulerYXZ: 
            order[0] = 2;
            order[1] = 0;
            order[2] = 1;
            break;

        case Model::RotOrder_EulerZXY: 
            order[0] = 1;
            order[1] = 0;
            order[2] = 2;
            break;

        case Model::RotOrder_EulerZYX:
            order[0] = 0;
            order[1] = 1;
            order[2] = 2;
            break;

            default:
                ai_assert(false);
        }
        
        ai_assert((order[0] >= 0) && (order[0] <= 2));
        ai_assert((order[1] >= 0) && (order[1] <= 2));
        ai_assert((order[2] >= 0) && (order[2] <= 2));

        if(!is_id[order[0]]) {
            out = temp[order[0]];
        }

        if(!is_id[order[1]]) {
            out = out * temp[order[1]];
        }

        if(!is_id[order[2]]) {
            out = out * temp[order[2]];
        }
    }


    // ------------------------------------------------------------------------------------------------
    // checks if a node has more than just scaling, rotation and translation components
    bool NeedsComplexTransformationChain(const Model& model)
    {
        const PropertyTable& props = model.Props();
        bool ok;

        const float zero_epsilon = 1e-6f;
        for (size_t i = 0; i < TransformationComp_MAXIMUM; ++i) {
            const TransformationComp comp = static_cast<TransformationComp>(i);

            if( comp == TransformationComp_Rotation || comp == TransformationComp_Scaling || comp == TransformationComp_Translation ||
                comp == TransformationComp_GeometricScaling || comp == TransformationComp_GeometricRotation || comp == TransformationComp_GeometricTranslation ) { 
                continue;
            }

            const aiVector3D& v = PropertyGet<aiVector3D>(props,NameTransformationCompProperty(comp),ok);
            if(ok && v.SquareLength() > zero_epsilon) {
                return true;
            }
        }

        return false;
    }


    // ------------------------------------------------------------------------------------------------
    // note: name must be a FixNodeName() result
    std::string NameTransformationChainNode(const std::string& name, TransformationComp comp)
    {
        return name + std::string(MAGIC_NODE_TAG) + "_" + NameTransformationComp(comp);
    }


    // ------------------------------------------------------------------------------------------------
    // note: memory for output_nodes will be managed by the caller
    void GenerateTransformationNodeChain(const Model& model,
        std::vector<aiNode*>& output_nodes)
    {
        const PropertyTable& props = model.Props();
        const Model::RotOrder rot = model.RotationOrder();

        bool ok;

        aiMatrix4x4 chain[TransformationComp_MAXIMUM];
        std::fill_n(chain, static_cast<unsigned int>(TransformationComp_MAXIMUM), aiMatrix4x4());

        // generate transformation matrices for all the different transformation components
        const float zero_epsilon = 1e-6f;
        bool is_complex = false;

        const aiVector3D& PreRotation = PropertyGet<aiVector3D>(props,"PreRotation",ok);
        if(ok && PreRotation.SquareLength() > zero_epsilon) {
            is_complex = true;

            GetRotationMatrix(rot, PreRotation, chain[TransformationComp_PreRotation]);
        }

        const aiVector3D& PostRotation = PropertyGet<aiVector3D>(props,"PostRotation",ok);
        if(ok && PostRotation.SquareLength() > zero_epsilon) {
            is_complex = true;

            GetRotationMatrix(rot, PostRotation, chain[TransformationComp_PostRotation]);
        }

        const aiVector3D& RotationPivot = PropertyGet<aiVector3D>(props,"RotationPivot",ok);
        if(ok && RotationPivot.SquareLength() > zero_epsilon) {
            is_complex = true;
            
            aiMatrix4x4::Translation(RotationPivot,chain[TransformationComp_RotationPivot]);
            aiMatrix4x4::Translation(-RotationPivot,chain[TransformationComp_RotationPivotInverse]);
        }

        const aiVector3D& RotationOffset = PropertyGet<aiVector3D>(props,"RotationOffset",ok);
        if(ok && RotationOffset.SquareLength() > zero_epsilon) {
            is_complex = true;

            aiMatrix4x4::Translation(RotationOffset,chain[TransformationComp_RotationOffset]);
        }

        const aiVector3D& ScalingOffset = PropertyGet<aiVector3D>(props,"ScalingOffset",ok);
        if(ok && ScalingOffset.SquareLength() > zero_epsilon) {
            is_complex = true;
            
            aiMatrix4x4::Translation(ScalingOffset,chain[TransformationComp_ScalingOffset]);
        }

        const aiVector3D& ScalingPivot = PropertyGet<aiVector3D>(props,"ScalingPivot",ok);
        if(ok && ScalingPivot.SquareLength() > zero_epsilon) {
            is_complex = true;

            aiMatrix4x4::Translation(ScalingPivot,chain[TransformationComp_ScalingPivot]);
            aiMatrix4x4::Translation(-ScalingPivot,chain[TransformationComp_ScalingPivotInverse]);
        }

        const aiVector3D& Translation = PropertyGet<aiVector3D>(props,"Lcl Translation",ok);
        if(ok && Translation.SquareLength() > zero_epsilon) {
            aiMatrix4x4::Translation(Translation,chain[TransformationComp_Translation]);
        }

        const aiVector3D& Scaling = PropertyGet<aiVector3D>(props,"Lcl Scaling",ok);
        if(ok && fabs(Scaling.SquareLength()-1.0f) > zero_epsilon) {
            aiMatrix4x4::Scaling(Scaling,chain[TransformationComp_Scaling]);
        }

        const aiVector3D& Rotation = PropertyGet<aiVector3D>(props,"Lcl Rotation",ok);
        if(ok && Rotation.SquareLength() > zero_epsilon) {
            GetRotationMatrix(rot, Rotation, chain[TransformationComp_Rotation]);
        }
        
        const aiVector3D& GeometricScaling = PropertyGet<aiVector3D>(props, "GeometricScaling", ok);
        if (ok && fabs(GeometricScaling.SquareLength() - 1.0f) > zero_epsilon) {
            aiMatrix4x4::Scaling(GeometricScaling, chain[TransformationComp_GeometricScaling]);
        }
        
        const aiVector3D& GeometricRotation = PropertyGet<aiVector3D>(props, "GeometricRotation", ok);
        if (ok && GeometricRotation.SquareLength() > zero_epsilon) {
            GetRotationMatrix(rot, GeometricRotation, chain[TransformationComp_GeometricRotation]);
        }

        const aiVector3D& GeometricTranslation = PropertyGet<aiVector3D>(props, "GeometricTranslation", ok);
        if (ok && GeometricTranslation.SquareLength() > zero_epsilon){
            aiMatrix4x4::Translation(GeometricTranslation, chain[TransformationComp_GeometricTranslation]);
        }

        // is_complex needs to be consistent with NeedsComplexTransformationChain()
        // or the interplay between this code and the animation converter would
        // not be guaranteed.
        ai_assert(NeedsComplexTransformationChain(model) == is_complex);

        const std::string& name = FixNodeName(model.Name());

        // now, if we have more than just Translation, Scaling and Rotation,
        // we need to generate a full node chain to accommodate for assimp's
        // lack to express pivots and offsets.
        if(is_complex && doc.Settings().preservePivots) {
            FBXImporter::LogInfo("generating full transformation chain for node: " + name);

            // query the anim_chain_bits dictionary to find out which chain elements
            // have associated node animation channels. These can not be dropped 
            // even if they have identity transform in bind pose.
            NodeAnimBitMap::const_iterator it = node_anim_chain_bits.find(name);
            const unsigned int anim_chain_bitmask = (it == node_anim_chain_bits.end() ? 0 : (*it).second);

            unsigned int bit = 0x1;
            for (size_t i = 0; i < TransformationComp_MAXIMUM; ++i, bit <<= 1) {
                const TransformationComp comp = static_cast<TransformationComp>(i);
                
                if (chain[i].IsIdentity() && (anim_chain_bitmask & bit) == 0) {
                    continue;
                }

                aiNode* nd = new aiNode();
                output_nodes.push_back(nd);
                
                nd->mName.Set(NameTransformationChainNode(name, comp));
                nd->mTransformation = chain[i];
            }

            ai_assert(output_nodes.size());
            return;
        }

        // else, we can just multiply the matrices together
        aiNode* nd = new aiNode();
        output_nodes.push_back(nd);

        nd->mName.Set(name);

        for (size_t i = 0; i < TransformationComp_MAXIMUM; ++i) {
            nd->mTransformation = nd->mTransformation * chain[i];
        }
    }
    
    // ------------------------------------------------------------------------------------------------

    void SetupNodeMetadata(const Model& model, aiNode& nd)
    {
        const PropertyTable& props = model.Props();
        DirectPropertyMap unparsedProperties = props.GetUnparsedProperties();

        // create metadata on node
        std::size_t numStaticMetaData = 2;
        aiMetadata* data = new aiMetadata();
        data->mNumProperties = unparsedProperties.size() + numStaticMetaData;
        data->mKeys = new aiString[data->mNumProperties]();
        data->mValues = new aiMetadataEntry[data->mNumProperties]();
        nd.mMetaData = data;
        int index = 0;

        // find user defined properties (3ds Max)
        data->Set(index++, "UserProperties", aiString(PropertyGet<std::string>(props, "UDP3DSMAX", "")));
        // preserve the info that a node was marked as Null node in the original file.
        data->Set(index++, "IsNull", model.IsNull() ? true : false);

        // add unparsed properties to the node's metadata
        BOOST_FOREACH(const DirectPropertyMap::value_type& prop, unparsedProperties) {

            // Interpret the property as a concrete type
            if (const TypedProperty<bool>* interpreted = prop.second->As<TypedProperty<bool> >())
                data->Set(index++, prop.first, interpreted->Value());
            else if (const TypedProperty<int>* interpreted = prop.second->As<TypedProperty<int> >())
                data->Set(index++, prop.first, interpreted->Value());
            else if (const TypedProperty<uint64_t>* interpreted = prop.second->As<TypedProperty<uint64_t> >())
                data->Set(index++, prop.first, interpreted->Value());
            else if (const TypedProperty<float>* interpreted = prop.second->As<TypedProperty<float> >())
                data->Set(index++, prop.first, interpreted->Value());
            else if (const TypedProperty<std::string>* interpreted = prop.second->As<TypedProperty<std::string> >())
                data->Set(index++, prop.first, aiString(interpreted->Value()));
            else if (const TypedProperty<aiVector3D>* interpreted = prop.second->As<TypedProperty<aiVector3D> >())
                data->Set(index++, prop.first, interpreted->Value());
            else
                assert(false);
        }
    }

    // ------------------------------------------------------------------------------------------------
    void ConvertModel(const Model& model, aiNode& nd, const aiMatrix4x4& node_global_transform)
    {
        const std::vector<const Geometry*>& geos = model.GetGeometry();

        std::vector<unsigned int> meshes;
        meshes.reserve(geos.size());

        BOOST_FOREACH(const Geometry* geo, geos) {

            const MeshGeometry* const mesh = dynamic_cast<const MeshGeometry*>(geo);
            if(mesh) {
                const std::vector<unsigned int>& indices = ConvertMesh(*mesh, model, node_global_transform);
                std::copy(indices.begin(),indices.end(),std::back_inserter(meshes) );
            }
            else {
                FBXImporter::LogWarn("ignoring unrecognized geometry: " + geo->Name());
            }
        }

        if(meshes.size()) {
            nd.mMeshes = new unsigned int[meshes.size()]();
            nd.mNumMeshes = static_cast<unsigned int>(meshes.size());

            std::swap_ranges(meshes.begin(),meshes.end(),nd.mMeshes);
        }
    }


    // ------------------------------------------------------------------------------------------------
    // MeshGeometry -> aiMesh, return mesh index + 1 or 0 if the conversion failed
    std::vector<unsigned int> ConvertMesh(const MeshGeometry& mesh,const Model& model, 
        const aiMatrix4x4& node_global_transform)
    {
        std::vector<unsigned int> temp; 

        MeshMap::const_iterator it = meshes_converted.find(&mesh);
        if (it != meshes_converted.end()) {
            std::copy((*it).second.begin(),(*it).second.end(),std::back_inserter(temp));
            return temp;
        }

        const std::vector<aiVector3D>& vertices = mesh.GetVertices();
        const std::vector<unsigned int>& faces = mesh.GetFaceIndexCounts();
        if(vertices.empty() || faces.empty()) {
            FBXImporter::LogWarn("ignoring empty geometry: " + mesh.Name());
            return temp;
        }

        // one material per mesh maps easily to aiMesh. Multiple material 
        // meshes need to be split.
        const MatIndexArray& mindices = mesh.GetMaterialIndices();
        if (doc.Settings().readMaterials && !mindices.empty()) {
            const MatIndexArray::value_type base = mindices[0];
            BOOST_FOREACH(MatIndexArray::value_type index, mindices) {
                if(index != base) {
                    return ConvertMeshMultiMaterial(mesh, model, node_global_transform);
                }
            }
        }

        // faster codepath, just copy the data
        temp.push_back(ConvertMeshSingleMaterial(mesh, model, node_global_transform));
        return temp;
    }


    // ------------------------------------------------------------------------------------------------
    aiMesh* SetupEmptyMesh(const MeshGeometry& mesh)
    {
        aiMesh* const out_mesh = new aiMesh();
        meshes.push_back(out_mesh);
        meshes_converted[&mesh].push_back(static_cast<unsigned int>(meshes.size()-1));

        // set name
        std::string name = mesh.Name();
        if (name.substr(0,10) == "Geometry::") {
            name = name.substr(10);
        }

        if(name.length()) {
            out_mesh->mName.Set(name);
        }

        return out_mesh;
    }


    // ------------------------------------------------------------------------------------------------
    unsigned int ConvertMeshSingleMaterial(const MeshGeometry& mesh, const Model& model, 
        const aiMatrix4x4& node_global_transform)   
    {
        const MatIndexArray& mindices = mesh.GetMaterialIndices();
        aiMesh* const out_mesh = SetupEmptyMesh(mesh); 

        const std::vector<aiVector3D>& vertices = mesh.GetVertices();
        const std::vector<unsigned int>& faces = mesh.GetFaceIndexCounts();

        // copy vertices
        out_mesh->mNumVertices = static_cast<unsigned int>(vertices.size());
        out_mesh->mVertices = new aiVector3D[vertices.size()];
        std::copy(vertices.begin(),vertices.end(),out_mesh->mVertices);

        // generate dummy faces
        out_mesh->mNumFaces = static_cast<unsigned int>(faces.size());
        aiFace* fac = out_mesh->mFaces = new aiFace[faces.size()]();

        unsigned int cursor = 0;
        BOOST_FOREACH(unsigned int pcount, faces) {
            aiFace& f = *fac++;
            f.mNumIndices = pcount;
            f.mIndices = new unsigned int[pcount];
            switch(pcount) 
            {
            case 1:
                out_mesh->mPrimitiveTypes |= aiPrimitiveType_POINT;
                break;
            case 2:
                out_mesh->mPrimitiveTypes |= aiPrimitiveType_LINE;
                break;
            case 3:
                out_mesh->mPrimitiveTypes |= aiPrimitiveType_TRIANGLE;
                break;
            default:
                out_mesh->mPrimitiveTypes |= aiPrimitiveType_POLYGON;
                break;
            }
            for (unsigned int i = 0; i < pcount; ++i) {
                f.mIndices[i] = cursor++;
            }
        }

        // copy normals
        const std::vector<aiVector3D>& normals = mesh.GetNormals();
        if(normals.size()) {
            ai_assert(normals.size() == vertices.size());

            out_mesh->mNormals = new aiVector3D[vertices.size()];
            std::copy(normals.begin(),normals.end(),out_mesh->mNormals);
        }

        // copy tangents - assimp requires both tangents and bitangents (binormals)
        // to be present, or neither of them. Compute binormals from normals
        // and tangents if needed.
        const std::vector<aiVector3D>& tangents = mesh.GetTangents();
        const std::vector<aiVector3D>* binormals = &mesh.GetBinormals();

        if(tangents.size()) {
            std::vector<aiVector3D> tempBinormals;
            if (!binormals->size()) {
                if (normals.size()) {
                    tempBinormals.resize(normals.size());
                    for (unsigned int i = 0; i < tangents.size(); ++i) {
                        tempBinormals[i] = normals[i] ^ tangents[i];
                    }

                    binormals = &tempBinormals;
                }
                else {
                    binormals = NULL;   
                }
            }

            if(binormals) {
                ai_assert(tangents.size() == vertices.size() && binormals->size() == vertices.size());

                out_mesh->mTangents = new aiVector3D[vertices.size()];
                std::copy(tangents.begin(),tangents.end(),out_mesh->mTangents);

                out_mesh->mBitangents = new aiVector3D[vertices.size()];
                std::copy(binormals->begin(),binormals->end(),out_mesh->mBitangents);
            }
        }

        // copy texture coords
        for (unsigned int i = 0; i < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++i) {
            const std::vector<aiVector2D>& uvs = mesh.GetTextureCoords(i);
            if(uvs.empty()) {
                break;
            }

            aiVector3D* out_uv = out_mesh->mTextureCoords[i] = new aiVector3D[vertices.size()];
            BOOST_FOREACH(const aiVector2D& v, uvs) {
                *out_uv++ = aiVector3D(v.x,v.y,0.0f);
            }

            out_mesh->mNumUVComponents[i] = 2;
        }

        // copy vertex colors
        for (unsigned int i = 0; i < AI_MAX_NUMBER_OF_COLOR_SETS; ++i) {
            const std::vector<aiColor4D>& colors = mesh.GetVertexColors(i);
            if(colors.empty()) {
                break;
            }

            out_mesh->mColors[i] = new aiColor4D[vertices.size()];
            std::copy(colors.begin(),colors.end(),out_mesh->mColors[i]);
        }

        if(!doc.Settings().readMaterials || mindices.empty()) {
            FBXImporter::LogError("no material assigned to mesh, setting default material");
            out_mesh->mMaterialIndex = GetDefaultMaterial();
        }
        else {
            ConvertMaterialForMesh(out_mesh,model,mesh,mindices[0]);
        }

        if(doc.Settings().readWeights && mesh.DeformerSkin() != NULL) {
            ConvertWeights(out_mesh, model, mesh, node_global_transform, NO_MATERIAL_SEPARATION);
        }

        return static_cast<unsigned int>(meshes.size() - 1);
    }


    // ------------------------------------------------------------------------------------------------
    std::vector<unsigned int> ConvertMeshMultiMaterial(const MeshGeometry& mesh, const Model& model, 
        const aiMatrix4x4& node_global_transform)   
    {
        const MatIndexArray& mindices = mesh.GetMaterialIndices();
        ai_assert(mindices.size());
    
        std::set<MatIndexArray::value_type> had;
        std::vector<unsigned int> indices;

        BOOST_FOREACH(MatIndexArray::value_type index, mindices) {
            if(had.find(index) == had.end()) {

                indices.push_back(ConvertMeshMultiMaterial(mesh, model, index, node_global_transform));
                had.insert(index);
            }
        }

        return indices;
    }


    // ------------------------------------------------------------------------------------------------
    unsigned int ConvertMeshMultiMaterial(const MeshGeometry& mesh, const Model& model, 
        MatIndexArray::value_type index, 
        const aiMatrix4x4& node_global_transform)   
    {
        aiMesh* const out_mesh = SetupEmptyMesh(mesh);

        const MatIndexArray& mindices = mesh.GetMaterialIndices();
        const std::vector<aiVector3D>& vertices = mesh.GetVertices();
        const std::vector<unsigned int>& faces = mesh.GetFaceIndexCounts();

        const bool process_weights = doc.Settings().readWeights && mesh.DeformerSkin() != NULL;

        unsigned int count_faces = 0;
        unsigned int count_vertices = 0;

        // count faces
        std::vector<unsigned int>::const_iterator itf = faces.begin();
        for(MatIndexArray::const_iterator it = mindices.begin(), 
            end = mindices.end(); it != end; ++it, ++itf) 
        {   
            if ((*it) != index) {
                continue;
            }
            ++count_faces;
            count_vertices += *itf;
        }

        ai_assert(count_faces);
        ai_assert(count_vertices);

        // mapping from output indices to DOM indexing, needed to resolve weights
        std::vector<unsigned int> reverseMapping;

        if (process_weights) {
            reverseMapping.resize(count_vertices);
        }

        // allocate output data arrays, but don't fill them yet
        out_mesh->mNumVertices = count_vertices;
        out_mesh->mVertices = new aiVector3D[count_vertices];

        out_mesh->mNumFaces = count_faces;
        aiFace* fac = out_mesh->mFaces = new aiFace[count_faces]();


        // allocate normals
        const std::vector<aiVector3D>& normals = mesh.GetNormals();
        if(normals.size()) {
            ai_assert(normals.size() == vertices.size());
            out_mesh->mNormals = new aiVector3D[vertices.size()];
        }

        // allocate tangents, binormals. 
        const std::vector<aiVector3D>& tangents = mesh.GetTangents();
        const std::vector<aiVector3D>* binormals = &mesh.GetBinormals();

        if(tangents.size()) {
            std::vector<aiVector3D> tempBinormals;
            if (!binormals->size()) {
                if (normals.size()) {
                    // XXX this computes the binormals for the entire mesh, not only 
                    // the part for which we need them.
                    tempBinormals.resize(normals.size());
                    for (unsigned int i = 0; i < tangents.size(); ++i) {
                        tempBinormals[i] = normals[i] ^ tangents[i];
                    }

                    binormals = &tempBinormals;
                }
                else {
                    binormals = NULL;   
                }
            }

            if(binormals) {
                ai_assert(tangents.size() == vertices.size() && binormals->size() == vertices.size());

                out_mesh->mTangents = new aiVector3D[vertices.size()];
                out_mesh->mBitangents = new aiVector3D[vertices.size()];
            }
        }

        // allocate texture coords
        unsigned int num_uvs = 0;
        for (unsigned int i = 0; i < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++i, ++num_uvs) {
            const std::vector<aiVector2D>& uvs = mesh.GetTextureCoords(i);
            if(uvs.empty()) {
                break;
            }

            out_mesh->mTextureCoords[i] = new aiVector3D[vertices.size()];
            out_mesh->mNumUVComponents[i] = 2;
        }

        // allocate vertex colors
        unsigned int num_vcs = 0;
        for (unsigned int i = 0; i < AI_MAX_NUMBER_OF_COLOR_SETS; ++i, ++num_vcs) {
            const std::vector<aiColor4D>& colors = mesh.GetVertexColors(i);
            if(colors.empty()) {
                break;
            }

            out_mesh->mColors[i] = new aiColor4D[vertices.size()];
        }

        unsigned int cursor = 0, in_cursor = 0;

        itf = faces.begin();
        for(MatIndexArray::const_iterator it = mindices.begin(), 
            end = mindices.end(); it != end; ++it, ++itf) 
        {   
            const unsigned int pcount = *itf;
            if ((*it) != index) {
                in_cursor += pcount;
                continue;
            }

            aiFace& f = *fac++;

            f.mNumIndices = pcount;
            f.mIndices = new unsigned int[pcount];
            switch(pcount) 
            {
            case 1:
                out_mesh->mPrimitiveTypes |= aiPrimitiveType_POINT;
                break;
            case 2:
                out_mesh->mPrimitiveTypes |= aiPrimitiveType_LINE;
                break;
            case 3:
                out_mesh->mPrimitiveTypes |= aiPrimitiveType_TRIANGLE;
                break;
            default:
                out_mesh->mPrimitiveTypes |= aiPrimitiveType_POLYGON;
                break;
            }
            for (unsigned int i = 0; i < pcount; ++i, ++cursor, ++in_cursor) {
                f.mIndices[i] = cursor;

                if(reverseMapping.size()) {
                    reverseMapping[cursor] = in_cursor;
                }

                out_mesh->mVertices[cursor] = vertices[in_cursor];

                if(out_mesh->mNormals) {
                    out_mesh->mNormals[cursor] = normals[in_cursor];
                }

                if(out_mesh->mTangents) {
                    out_mesh->mTangents[cursor] = tangents[in_cursor];
                    out_mesh->mBitangents[cursor] = (*binormals)[in_cursor];
                }

                for (unsigned int i = 0; i < num_uvs; ++i) {
                    const std::vector<aiVector2D>& uvs = mesh.GetTextureCoords(i);
                    out_mesh->mTextureCoords[i][cursor] = aiVector3D(uvs[in_cursor].x,uvs[in_cursor].y, 0.0f);
                }

                for (unsigned int i = 0; i < num_vcs; ++i) {
                    const std::vector<aiColor4D>& cols = mesh.GetVertexColors(i);
                    out_mesh->mColors[i][cursor] = cols[in_cursor];
                }
            }
        }
    
        ConvertMaterialForMesh(out_mesh,model,mesh,index);

        if(process_weights) {
            ConvertWeights(out_mesh, model, mesh, node_global_transform, index, &reverseMapping);
        }

        return static_cast<unsigned int>(meshes.size() - 1);
    }

    static const unsigned int NO_MATERIAL_SEPARATION = // std::numeric_limits<unsigned int>::max()
        static_cast<unsigned int>(-1);

    // ------------------------------------------------------------------------------------------------
    void ConvertMaterialForMesh(aiMesh* out, const Model& model, const MeshGeometry& geo, 
        MatIndexArray::value_type materialIndex)
    {
        // locate source materials for this mesh
        const std::vector<const Material*>& mats = model.GetMaterials();
        if (static_cast<unsigned int>(materialIndex) >= mats.size() || materialIndex < 0) {
            FBXImporter::LogError("material index out of bounds, setting default material");
            out->mMaterialIndex = GetDefaultMaterial();
            return;
        }

        const Material* const mat = mats[materialIndex];
        MaterialMap::const_iterator it = materials_converted.find(mat);
        if (it != materials_converted.end()) {
            out->mMaterialIndex = (*it).second;
            return;
        }

        out->mMaterialIndex = ConvertMaterial(*mat, &geo);  
        materials_converted[mat] = out->mMaterialIndex;
    }


    // ------------------------------------------------------------------------------------------------
    unsigned int GetDefaultMaterial()
    {
        if (defaultMaterialIndex) {
            return defaultMaterialIndex - 1; 
        }

        aiMaterial* out_mat = new aiMaterial();
        materials.push_back(out_mat);

        const aiColor3D diffuse = aiColor3D(0.8f,0.8f,0.8f);
        out_mat->AddProperty(&diffuse,1,AI_MATKEY_COLOR_DIFFUSE);

        aiString s;
        s.Set(AI_DEFAULT_MATERIAL_NAME);

        out_mat->AddProperty(&s,AI_MATKEY_NAME);

        defaultMaterialIndex = static_cast<unsigned int>(materials.size());
        return defaultMaterialIndex - 1;
    }


    // ------------------------------------------------------------------------------------------------
    // Material -> aiMaterial
    unsigned int ConvertMaterial(const Material& material, const MeshGeometry* const mesh)
    {
        const PropertyTable& props = material.Props();

        // generate empty output material
        aiMaterial* out_mat = new aiMaterial();
        materials_converted[&material] = static_cast<unsigned int>(materials.size());

        materials.push_back(out_mat);

        aiString str;

        // stip Material:: prefix
        std::string name = material.Name();
        if(name.substr(0,10) == "Material::") {
            name = name.substr(10);
        }

        // set material name if not empty - this could happen
        // and there should be no key for it in this case.
        if(name.length()) {
            str.Set(name);
            out_mat->AddProperty(&str,AI_MATKEY_NAME);
        }

        // shading stuff and colors
        SetShadingPropertiesCommon(out_mat,props);
    
        // texture assignments
        SetTextureProperties(out_mat,material.Textures(), mesh);
        SetTextureProperties(out_mat,material.LayeredTextures(), mesh);

        return static_cast<unsigned int>(materials.size() - 1);
    }


    // ------------------------------------------------------------------------------------------------
    void TrySetTextureProperties(aiMaterial* out_mat, const TextureMap& textures, 
        const std::string& propName, 
        aiTextureType target, const MeshGeometry* const mesh)
    {
        TextureMap::const_iterator it = textures.find(propName);
        if(it == textures.end()) {
            return;
        }

        const Texture* const tex = (*it).second;
        if(tex !=0 )
        {
            aiString path;
            path.Set(tex->RelativeFilename());

            out_mat->AddProperty(&path,_AI_MATKEY_TEXTURE_BASE,target,0);

            aiUVTransform uvTrafo;
            // XXX handle all kinds of UV transformations
            uvTrafo.mScaling = tex->UVScaling();
            uvTrafo.mTranslation = tex->UVTranslation();
            out_mat->AddProperty(&uvTrafo,1,_AI_MATKEY_UVTRANSFORM_BASE,target,0);

            const PropertyTable& props = tex->Props();

            int uvIndex = 0;

            bool ok;
            const std::string& uvSet = PropertyGet<std::string>(props,"UVSet",ok);
            if(ok) {
                // "default" is the name which usually appears in the FbxFileTexture template
                if(uvSet != "default" && uvSet.length()) {
                    // this is a bit awkward - we need to find a mesh that uses this
                    // material and scan its UV channels for the given UV name because
                    // assimp references UV channels by index, not by name.

                    // XXX: the case that UV channels may appear in different orders
                    // in meshes is unhandled. A possible solution would be to sort
                    // the UV channels alphabetically, but this would have the side
                    // effect that the primary (first) UV channel would sometimes
                    // be moved, causing trouble when users read only the first
                    // UV channel and ignore UV channel assignments altogether.

                    const unsigned int matIndex = static_cast<unsigned int>(std::distance(materials.begin(), 
                        std::find(materials.begin(),materials.end(),out_mat)
                    ));

                  uvIndex = -1;
                  if (!mesh) {
                      BOOST_FOREACH(const MeshMap::value_type& v,meshes_converted) {
                          const MeshGeometry* const mesh = dynamic_cast<const MeshGeometry*> (v.first);
                          if(!mesh) {
                              continue;
                          }

                          const MatIndexArray& mats = mesh->GetMaterialIndices();
                          if(std::find(mats.begin(),mats.end(),matIndex) == mats.end()) {
                              continue;
                          }

                          int index = -1;
                          for (unsigned int i = 0; i < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++i) {
                              if(mesh->GetTextureCoords(i).empty()) {
                                  break;
                              }
                              const std::string& name = mesh->GetTextureCoordChannelName(i);
                              if(name == uvSet) {
                                  index = static_cast<int>(i);
                                  break;
                              }
                          }
                          if(index == -1) {
                              FBXImporter::LogWarn("did not find UV channel named " + uvSet + " in a mesh using this material");
                              continue;
                          }

                          if(uvIndex == -1) {
                              uvIndex = index;
                          }
                          else {
                              FBXImporter::LogWarn("the UV channel named " + uvSet + 
                                  " appears at different positions in meshes, results will be wrong");
                          }
                      }
                  } else {
                        int index = -1;
                        for (unsigned int i = 0; i < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++i) {
                            if(mesh->GetTextureCoords(i).empty()) {
                                break;
                            }
                            const std::string& name = mesh->GetTextureCoordChannelName(i);
                            if(name == uvSet) {
                                index = static_cast<int>(i);
                                break;
                            }
                        }
                        if(index == -1) {
                            FBXImporter::LogWarn("did not find UV channel named " + uvSet + " in a mesh using this material");
                        }

                        if(uvIndex == -1) {
                            uvIndex = index;
                        }
                  }

                    if(uvIndex == -1) {
                        FBXImporter::LogWarn("failed to resolve UV channel " + uvSet + ", using first UV channel");
                        uvIndex = 0;
                    }
                }
            }

            out_mat->AddProperty(&uvIndex,1,_AI_MATKEY_UVWSRC_BASE,target,0);
        }
    }

    // ------------------------------------------------------------------------------------------------
    void TrySetTextureProperties(aiMaterial* out_mat, const LayeredTextureMap& layeredTextures, 
        const std::string& propName, 
        aiTextureType target, const MeshGeometry* const mesh)
    {
        LayeredTextureMap::const_iterator it = layeredTextures.find(propName);
        if(it == layeredTextures.end()) {
            return;
        }

        const Texture* const tex = (*it).second->getTexture();

        aiString path;
        path.Set(tex->RelativeFilename());

        out_mat->AddProperty(&path,_AI_MATKEY_TEXTURE_BASE,target,0);

        aiUVTransform uvTrafo;
        // XXX handle all kinds of UV transformations
        uvTrafo.mScaling = tex->UVScaling();
        uvTrafo.mTranslation = tex->UVTranslation();
        out_mat->AddProperty(&uvTrafo,1,_AI_MATKEY_UVTRANSFORM_BASE,target,0);

        const PropertyTable& props = tex->Props();

        int uvIndex = 0;

        bool ok;
        const std::string& uvSet = PropertyGet<std::string>(props,"UVSet",ok);
        if(ok) {
            // "default" is the name which usually appears in the FbxFileTexture template
            if(uvSet != "default" && uvSet.length()) {
                // this is a bit awkward - we need to find a mesh that uses this
                // material and scan its UV channels for the given UV name because
                // assimp references UV channels by index, not by name.

                // XXX: the case that UV channels may appear in different orders
                // in meshes is unhandled. A possible solution would be to sort
                // the UV channels alphabetically, but this would have the side
                // effect that the primary (first) UV channel would sometimes
                // be moved, causing trouble when users read only the first
                // UV channel and ignore UV channel assignments altogether.

                const unsigned int matIndex = static_cast<unsigned int>(std::distance(materials.begin(), 
                    std::find(materials.begin(),materials.end(),out_mat)
                    ));

              uvIndex = -1;
        if (!mesh)
        {                   
                    BOOST_FOREACH(const MeshMap::value_type& v,meshes_converted) {
                        const MeshGeometry* const mesh = dynamic_cast<const MeshGeometry*> (v.first);
                        if(!mesh) {
                            continue;
                        }

                        const MatIndexArray& mats = mesh->GetMaterialIndices();
                        if(std::find(mats.begin(),mats.end(),matIndex) == mats.end()) {
                            continue;
                        }

                        int index = -1;
                        for (unsigned int i = 0; i < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++i) {
                            if(mesh->GetTextureCoords(i).empty()) {
                                break;
                            }
                            const std::string& name = mesh->GetTextureCoordChannelName(i);
                            if(name == uvSet) {
                                index = static_cast<int>(i);
                                break;
                            }
                        }
                        if(index == -1) {
                            FBXImporter::LogWarn("did not find UV channel named " + uvSet + " in a mesh using this material");
                            continue;
                        }

                        if(uvIndex == -1) {
                            uvIndex = index;
                        }
                        else {
                            FBXImporter::LogWarn("the UV channel named " + uvSet + 
                                " appears at different positions in meshes, results will be wrong");
                        }
                    }
        }
        else
        {
                    int index = -1;
                    for (unsigned int i = 0; i < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++i) {
                        if(mesh->GetTextureCoords(i).empty()) {
                            break;
                        }
                        const std::string& name = mesh->GetTextureCoordChannelName(i);
                        if(name == uvSet) {
                            index = static_cast<int>(i);
                            break;
                        }
                    }
                    if(index == -1) {
                        FBXImporter::LogWarn("did not find UV channel named " + uvSet + " in a mesh using this material");
                    }

                    if(uvIndex == -1) {
                        uvIndex = index;
                    }
        }

                if(uvIndex == -1) {
                    FBXImporter::LogWarn("failed to resolve UV channel " + uvSet + ", using first UV channel");
                    uvIndex = 0;
                }
            }
        }

        out_mat->AddProperty(&uvIndex,1,_AI_MATKEY_UVWSRC_BASE,target,0);
    }

    // ------------------------------------------------------------------------------------------------
    void SetTextureProperties(aiMaterial* out_mat, const TextureMap& textures, const MeshGeometry* const mesh)
    {
        TrySetTextureProperties(out_mat, textures, "DiffuseColor", aiTextureType_DIFFUSE, mesh);
        TrySetTextureProperties(out_mat, textures, "AmbientColor", aiTextureType_AMBIENT, mesh);
        TrySetTextureProperties(out_mat, textures, "EmissiveColor", aiTextureType_EMISSIVE, mesh);
        TrySetTextureProperties(out_mat, textures, "SpecularColor", aiTextureType_SPECULAR, mesh);
        TrySetTextureProperties(out_mat, textures, "TransparentColor", aiTextureType_OPACITY, mesh);
        TrySetTextureProperties(out_mat, textures, "ReflectionColor", aiTextureType_REFLECTION, mesh);
        TrySetTextureProperties(out_mat, textures, "DisplacementColor", aiTextureType_DISPLACEMENT, mesh);
        TrySetTextureProperties(out_mat, textures, "NormalMap", aiTextureType_NORMALS, mesh);
        TrySetTextureProperties(out_mat, textures, "Bump", aiTextureType_HEIGHT, mesh);
        TrySetTextureProperties(out_mat, textures, "ShininessExponent", aiTextureType_SHININESS, mesh);
    }

    // ------------------------------------------------------------------------------------------------
    void SetTextureProperties(aiMaterial* out_mat, const LayeredTextureMap& layeredTextures, const MeshGeometry* const mesh)
    {
        TrySetTextureProperties(out_mat, layeredTextures, "DiffuseColor", aiTextureType_DIFFUSE, mesh);
        TrySetTextureProperties(out_mat, layeredTextures, "AmbientColor", aiTextureType_AMBIENT, mesh);
        TrySetTextureProperties(out_mat, layeredTextures, "EmissiveColor", aiTextureType_EMISSIVE, mesh);
        TrySetTextureProperties(out_mat, layeredTextures, "SpecularColor", aiTextureType_SPECULAR, mesh);
        TrySetTextureProperties(out_mat, layeredTextures, "TransparentColor", aiTextureType_OPACITY, mesh);
        TrySetTextureProperties(out_mat, layeredTextures, "ReflectionColor", aiTextureType_REFLECTION, mesh);
        TrySetTextureProperties(out_mat, layeredTextures, "DisplacementColor", aiTextureType_DISPLACEMENT, mesh);
        TrySetTextureProperties(out_mat, layeredTextures, "NormalMap", aiTextureType_NORMALS, mesh);
        TrySetTextureProperties(out_mat, layeredTextures, "Bump", aiTextureType_HEIGHT, mesh);
        TrySetTextureProperties(out_mat, layeredTextures, "ShininessExponent", aiTextureType_SHININESS, mesh);
    }


    // ------------------------------------------------------------------------------------------------
    aiColor3D GetColorPropertyFromMaterial(const PropertyTable& props, const std::string& baseName, 
        bool& result)
    {
        result = true;

        bool ok;
        const aiVector3D& Diffuse = PropertyGet<aiVector3D>(props,baseName,ok);
        if(ok) {
            return aiColor3D(Diffuse.x,Diffuse.y,Diffuse.z);
        }
        else {
            aiVector3D DiffuseColor = PropertyGet<aiVector3D>(props,baseName + "Color",ok);
            if(ok) {
                float DiffuseFactor = PropertyGet<float>(props,baseName + "Factor",ok);
                if(ok) {
                    DiffuseColor *= DiffuseFactor;
                }

                return aiColor3D(DiffuseColor.x,DiffuseColor.y,DiffuseColor.z);
            }
        }
        result = false;
        return aiColor3D(0.0f,0.0f,0.0f);
    }


    // ------------------------------------------------------------------------------------------------
    void SetShadingPropertiesCommon(aiMaterial* out_mat, const PropertyTable& props)
    {
        // set shading properties. There are various, redundant ways in which FBX materials
        // specify their shading settings (depending on shading models, prop
        // template etc.). No idea which one is right in a particular context. 
        // Just try to make sense of it - there's no spec to verify this against, 
        // so why should we.
        bool ok;
        const aiColor3D& Diffuse = GetColorPropertyFromMaterial(props,"Diffuse",ok);
        if(ok) {
            out_mat->AddProperty(&Diffuse,1,AI_MATKEY_COLOR_DIFFUSE);
        }

        const aiColor3D& Emissive = GetColorPropertyFromMaterial(props,"Emissive",ok);
        if(ok) {
            out_mat->AddProperty(&Emissive,1,AI_MATKEY_COLOR_EMISSIVE);
        }

        const aiColor3D& Ambient = GetColorPropertyFromMaterial(props,"Ambient",ok);
        if(ok) {
            out_mat->AddProperty(&Ambient,1,AI_MATKEY_COLOR_AMBIENT);
        }

        const aiColor3D& Specular = GetColorPropertyFromMaterial(props,"Specular",ok);
        if(ok) {
            out_mat->AddProperty(&Specular,1,AI_MATKEY_COLOR_SPECULAR);
        }

        const float Opacity = PropertyGet<float>(props,"Opacity",ok);
        if(ok) {
            out_mat->AddProperty(&Opacity,1,AI_MATKEY_OPACITY);
        }

        const float Reflectivity = PropertyGet<float>(props,"Reflectivity",ok);
        if(ok) {
            out_mat->AddProperty(&Reflectivity,1,AI_MATKEY_REFLECTIVITY);
        }

        const float Shininess = PropertyGet<float>(props,"Shininess",ok);
        if(ok) {
            out_mat->AddProperty(&Shininess,1,AI_MATKEY_SHININESS_STRENGTH);
        }

        const float ShininessExponent = PropertyGet<float>(props,"ShininessExponent",ok);
        if(ok) {
            out_mat->AddProperty(&ShininessExponent,1,AI_MATKEY_SHININESS);
        }
    }


    // ------------------------------------------------------------------------------------------------
    // convert animation data to aiAnimation et al
    void ConvertAnimations() 
    {
        // TODO
    }


    // ------------------------------------------------------------------------------------------------
    // rename a node already partially converted. fixed_name is a string previously returned by 
    // FixNodeName, new_name specifies the string FixNodeName should return on all further invocations 
    // which would previously have returned the old value.
    //
    // this also updates names in node animations, cameras and light sources and is thus slow.
    //
    // NOTE: the caller is responsible for ensuring that the new name is unique and does
    // not collide with any other identifiers. The best way to ensure this is to only
    // append to the old name, which is guaranteed to match these requirements.
    void RenameNode(const std::string& fixed_name, const std::string& new_name)
    {
        ai_assert(node_names.find(fixed_name) != node_names.end());
        ai_assert(node_names.find(new_name) == node_names.end());

        renamed_nodes[fixed_name] = new_name;

        const aiString fn(fixed_name);

        BOOST_FOREACH(aiCamera* cam, cameras) {
            if (cam->mName == fn) {
                cam->mName.Set(new_name);
                break;
            }
        }

        BOOST_FOREACH(aiLight* light, lights) {
            if (light->mName == fn) {
                light->mName.Set(new_name);
                break;
            }
        }

        BOOST_FOREACH(aiAnimation* anim, animations) {
            for (unsigned int i = 0; i < anim->mNumChannels; ++i) {
                aiNodeAnim* const na = anim->mChannels[i];
                if (na->mNodeName == fn) {
                    na->mNodeName.Set(new_name);
                    break;
                }
            }
        }
    }


    // ------------------------------------------------------------------------------------------------
    // takes a fbx node name and returns the identifier to be used in the assimp output scene.
    // the function is guaranteed to provide consistent results over multiple invocations
    // UNLESS RenameNode() is called for a particular node name.
    std::string FixNodeName(const std::string& name)
    {
        // strip Model:: prefix, avoiding ambiguities (i.e. don't strip if 
        // this causes ambiguities, well possible between empty identifiers,
        // such as "Model::" and ""). Make sure the behaviour is consistent
        // across multiple calls to FixNodeName().
        if(name.substr(0,7) == "Model::") {
            std::string temp = name.substr(7);

            const NodeNameMap::const_iterator it = node_names.find(temp);
            if (it != node_names.end()) {
                if (!(*it).second) {
                    return FixNodeName(name + "_");
                }
            }
            node_names[temp] = true;

            const NameNameMap::const_iterator rit = renamed_nodes.find(temp);
            return rit == renamed_nodes.end() ? temp : (*rit).second;
        }

        const NodeNameMap::const_iterator it = node_names.find(name);
        if (it != node_names.end()) {
            if ((*it).second) {
                return FixNodeName(name + "_");
            }
        }
        node_names[name] = false;

        const NameNameMap::const_iterator rit = renamed_nodes.find(name);
        return rit == renamed_nodes.end() ? name : (*rit).second;
    }

    */

    // ------------------------------------------------------------------------------------------------
    // copy generated meshes, animations, lights, cameras and textures to the output scene
    void TransferDataToScene()
    {
        ai_assert(!out->mMeshes && !out->mNumMeshes);

        // note: the trailing () ensures initialization with NULL - not
        // many C++ users seem to know this, so pointing it out to avoid
        // confusion why this code works.

        if(meshes.size()) {
            out->mMeshes = new aiMesh*[meshes.size()]();
            out->mNumMeshes = static_cast<unsigned int>(meshes.size());

            std::swap_ranges(meshes.begin(),meshes.end(),out->mMeshes);
        }

        if(materials.size()) {
            out->mMaterials = new aiMaterial*[materials.size()]();
            out->mNumMaterials = static_cast<unsigned int>(materials.size());

            std::swap_ranges(materials.begin(),materials.end(),out->mMaterials);
        }

        if(animations.size()) {
            out->mAnimations = new aiAnimation*[animations.size()]();
            out->mNumAnimations = static_cast<unsigned int>(animations.size());

            std::swap_ranges(animations.begin(),animations.end(),out->mAnimations);
        }

        if(lights.size()) {
            out->mLights = new aiLight*[lights.size()]();
            out->mNumLights = static_cast<unsigned int>(lights.size());

            std::swap_ranges(lights.begin(),lights.end(),out->mLights);
        }

        if(cameras.size()) {
            out->mCameras = new aiCamera*[cameras.size()]();
            out->mNumCameras = static_cast<unsigned int>(cameras.size());

            std::swap_ranges(cameras.begin(),cameras.end(),out->mCameras);
        }
    }


private:

    // 0: not assigned yet, others: index is value - 1
    unsigned int defaultMaterialIndex;

    std::vector<aiMesh*> meshes;
    std::vector<aiMaterial*> materials;
    std::vector<aiAnimation*> animations;
    std::vector<aiLight*> lights;
    std::vector<aiCamera*> cameras;

    typedef std::map<const MaterialRefStructure*, unsigned int> MaterialMap;
    MaterialMap materials_converted;

    typedef std::map<const GeometryObjectStructure*, std::vector<unsigned int> > MeshMap;
    MeshMap meshes_converted;

    // name -> has had its prefix_stripped?
    typedef std::map<std::string, bool> NodeNameMap;
    NodeNameMap node_names;

    typedef std::map<std::string, std::string> NameNameMap;
    NameNameMap renamed_nodes;

    double anim_fps;

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
    if (result == kDataOkay) {
        ConvertToAssimpScene(pScene, openGexDataDescription);
    } else {
        ThrowException("Failed to load OpenGEX file");
    }
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
