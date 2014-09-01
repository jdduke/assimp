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

#include <unordered_map>

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

bool IsObject(const Structure& structure) {
	const StructureType type = structure.GetStructureType();
	return type == kStructureLightObject ||
	       type == kStructureGeometryObject ||
	       type == kStructureCameraObject ||
	       type == kStructureMaterial;
}

bool IsNode(const Structure& structure) {
	const StructureType type = structure.GetStructureType();
	return type == kStructureNode ||
	       type == kStructureBoneNode ||
	       type == kStructureLightNode ||
	       type == kStructureCameraNode ||
	       type == kStructureGeometryNode;
}

template <typename Type>
void SafeAllocateAndMove(std::vector< std::unique_ptr<Type> >& source, Type*** pDest, unsigned int* pCount) {
	if (source.empty()) return;
	*pDest = new Type*[source.size()]();
	for (size_t i = 0; i < source.size(); ++i)
		*pDest[i] = source[i].release();
	*pCount = static_cast<unsigned int>(source.size());
}

struct Visitor {
	virtual void Visit(const Structure&) {}
	virtual void Visit(const NodeStructure&) {}
	virtual void Visit(const BoneNodeStructure&) {}
	virtual void Visit(const GeometryNodeStructure&) {}
	virtual void Visit(const LightNodeStructure&) {}
	virtual void Visit(const CameraNodeStructure&) {}
	virtual void Visit(const MetricStructure&) {}
	virtual void Visit(const VertexArrayStructure&) {}
	virtual void Visit(const IndexArrayStructure&) {}
	virtual void Visit(const MeshStructure&) {}
	virtual void Visit(const GeometryObjectStructure&) {}
	virtual void Visit(const LightObjectStructure&) {}
	virtual void Visit(const CameraObjectStructure&) {}
	virtual void Visit(const TransformStructure&) {}
	virtual void Visit(const TranslationStructure&) {}
	virtual void Visit(const RotationStructure&) {}
	virtual void Visit(const ScaleStructure&) {}
	virtual void Visit(const NameStructure&) {}
	virtual void Visit(const ObjectRefStructure&) {}
	virtual void Visit(const MaterialRefStructure&) {}
	virtual void Visit(const MorphStructure&) {}
	virtual void Visit(const BoneRefArrayStructure&) {}
	virtual void Visit(const BoneCountArrayStructure&) {}
	virtual void Visit(const BoneIndexArrayStructure&) {}
	virtual void Visit(const BoneWeightArrayStructure&) {}
	virtual void Visit(const SkeletonStructure&) {}
	virtual void Visit(const SkinStructure&) {}
	virtual void Visit(const MaterialStructure&) {}
	virtual void Visit(const ParamStructure&) {}
	virtual void Visit(const ColorStructure&) {}
	virtual void Visit(const TextureStructure&) {}
	virtual void Visit(const AttenStructure&) {}
	virtual void Visit(const KeyStructure&) {}
	virtual void Visit(const TimeStructure&) {}
	virtual void Visit(const ValueStructure&) {}
	virtual void Visit(const TrackStructure&) {}
	virtual void Visit(const AnimationStructure&) {}
};

void Visit(const Structure& structure, Visitor& visitor) {
	switch (structure.GetStructureType()) {
		case kStructureNode: return visitor.Visit(static_cast<const NodeStructure&>(structure));
		case kStructureBoneNode: return visitor.Visit(static_cast<const BoneNodeStructure&>(structure));
		case kStructureGeometryNode: return visitor.Visit(static_cast<const GeometryNodeStructure&>(structure));
		case kStructureLightNode: return visitor.Visit(static_cast<const LightNodeStructure&>(structure));
		case kStructureCameraNode: return visitor.Visit(static_cast<const CameraNodeStructure&>(structure));
		case kStructureMetric: return visitor.Visit(static_cast<const MetricStructure&>(structure));
		case kStructureVertexArray: return visitor.Visit(static_cast<const VertexArrayStructure&>(structure));
		case kStructureIndexArray: return visitor.Visit(static_cast<const IndexArrayStructure&>(structure));
		case kStructureMesh: return visitor.Visit(static_cast<const MeshStructure&>(structure));
		case kStructureGeometryObject: return visitor.Visit(static_cast<const GeometryObjectStructure&>(structure));
		case kStructureLightObject: return visitor.Visit(static_cast<const LightObjectStructure&>(structure));
		case kStructureCameraObject: return visitor.Visit(static_cast<const CameraObjectStructure&>(structure));
		case kStructureTransform: return visitor.Visit(static_cast<const TransformStructure&>(structure));
		case kStructureTranslation: return visitor.Visit(static_cast<const TranslationStructure&>(structure));
		case kStructureRotation: return visitor.Visit(static_cast<const RotationStructure&>(structure));
		case kStructureScale: return visitor.Visit(static_cast<const ScaleStructure&>(structure));
		case kStructureName: return visitor.Visit(static_cast<const NameStructure&>(structure));
		case kStructureObjectRef: return visitor.Visit(static_cast<const ObjectRefStructure&>(structure));
		case kStructureMaterialRef: return visitor.Visit(static_cast<const MaterialRefStructure&>(structure));
		case kStructureMorph: return visitor.Visit(static_cast<const MorphStructure&>(structure));
		case kStructureBoneRefArray: return visitor.Visit(static_cast<const BoneRefArrayStructure&>(structure));
		case kStructureBoneCountArray: return visitor.Visit(static_cast<const BoneCountArrayStructure&>(structure));
		case kStructureBoneIndexArray: return visitor.Visit(static_cast<const BoneIndexArrayStructure&>(structure));
		case kStructureBoneWeightArray: return visitor.Visit(static_cast<const BoneWeightArrayStructure&>(structure));
		case kStructureSkeleton: return visitor.Visit(static_cast<const SkeletonStructure&>(structure));
		case kStructureSkin: return visitor.Visit(static_cast<const SkinStructure&>(structure));
		case kStructureMaterial: return visitor.Visit(static_cast<const MaterialStructure&>(structure));
		case kStructureParam: return visitor.Visit(static_cast<const ParamStructure&>(structure));
		case kStructureColor: return visitor.Visit(static_cast<const ColorStructure&>(structure));
		case kStructureTexture: return visitor.Visit(static_cast<const TextureStructure&>(structure));
		case kStructureAtten: return visitor.Visit(static_cast<const AttenStructure&>(structure));
		case kStructureKey: return visitor.Visit(static_cast<const KeyStructure&>(structure));
		case kStructureTime: return visitor.Visit(static_cast<const TimeStructure&>(structure));
		case kStructureValue: return visitor.Visit(static_cast<const ValueStructure&>(structure));
		case kStructureTrack: return visitor.Visit(static_cast<const TrackStructure&>(structure));
		case kStructureAnimation: return visitor.Visit(static_cast<const AnimationStructure&>(structure));
		default: break;
	}
	return visitor.Visit(structure);
}

void VisitRecursive(const Structure& structure, Visitor& visitor) {
	Visit(structure, visitor);
	const Structure* child = structure.GetFirstSubnode();
	while (child) {
		VisitRecursive(*child, visitor);
		child = child->Next();
	}
}

void VisitSubnodes(const Structure& structure, Visitor& visitor) {
	const Structure* child = structure.GetFirstSubnode();
	while (child) {
		Visit(*child, visitor);
		child = child->Next();
	}
}

struct TransformVisitor : public Visitor {
	explicit TransformVisitor(aiMatrix4x4* result) : result(result) {}

	virtual void Visit(const TransformStructure& transform) override {
		if (transform.GetObjectFlag()) return;
		const float* data = transform.GetTransform(0);
		*result *= aiMatrix4x4(data[0], data[1], data[2], data[3],
		                       data[4], data[5], data[6], data[7],
		                       data[8], data[9], data[10], data[11],
		                       data[12], data[13], data[14], data[15]);
	}

	virtual void Visit(const TranslationStructure& transform) override {
		if (transform.GetObjectFlag()) return;
		const String& kind = transform.GetTranslationKind();
		const float* data = transform.GetData();

		aiMatrix4x4 translation;
		if (kind == "xyz")    aiMatrix4x4::Translation(aiVector3D(data[0], data[1], data[2]), translation);
		else if (kind == "x") aiMatrix4x4::Translation(aiVector3D(data[0], 0, 0), translation);
		else if (kind == "y") aiMatrix4x4::Translation(aiVector3D(0, data[0], 0), translation);
		else if (kind == "z") aiMatrix4x4::Translation(aiVector3D(0, 0, data[0]), translation);
		*result *= translation;
	}

	virtual void Visit(const RotationStructure& transform) override {
		if (transform.GetObjectFlag()) return;
		const String& kind = transform.GetRotationKind();
		const float* data = transform.GetData();

		aiMatrix4x4 rotation;
		if (kind == "quaternion") rotation = aiMatrix4x4(aiVector3D(), aiQuaternion(data[0], data[1], data[2], data[3]), aiVector3D());
		else if (kind == "axis")  aiMatrix4x4::Rotation(data[0], aiVector3D(data[1], data[2], data[3]), rotation);
		else if (kind == "x")     aiMatrix4x4::RotationX(data[0], rotation);
		else if (kind == "y")     aiMatrix4x4::RotationY(data[0], rotation);
		else if (kind == "z")     aiMatrix4x4::RotationZ(data[0], rotation);
		*result *= rotation;
	}

	virtual void Visit(const ScaleStructure& transform) override {
		if (transform.GetObjectFlag()) return;
		const String& kind = transform.GetScaleKind();
		const float* data = transform.GetData();

		aiMatrix4x4 scaling;
		if (kind == "xyz")    aiMatrix4x4::Scaling(aiVector3D(data[0], data[1], data[2]), scaling);
		else if (kind == "x") aiMatrix4x4::Scaling(aiVector3D(data[0], 0, 0), scaling);
		else if (kind == "y") aiMatrix4x4::Scaling(aiVector3D(0, data[0], 0), scaling);
		else if (kind == "z") aiMatrix4x4::Scaling(aiVector3D(0, 0, data[0]), scaling);
		*result *= scaling;
	}

	aiMatrix4x4* const result;
};

class OpenGEXConverter : public LogFunctions<OpenGEXConverter> {
public:

	OpenGEXConverter(aiScene* out, const OpenGexDataDescription& desc)
		: defaultMaterialIndex()
		, out(out)
		, desc(desc) {
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

			ConvertObjectStructure(*structure);
			structure = structure->Next();
		}
	}

	// Nodes are hierarchical structures.
	void ConvertNodes(const Structure& parentStructure, aiNode& parentNode) {
		const Structure* structure = parentStructure.GetFirstSubnode();

		std::vector< std::unique_ptr<aiNode> > nodes;
		while (structure) {
			if (!IsNode(*structure)) continue;

			unique_ptr<aiNode> node(ConvertNodeStructure(*static_cast<const NodeStructure*>(structure)));
			ai_assert(node);
			node->mParent = &parentNode;
			ConvertNodes(*structure, *node);
			nodes.push_back(move(node));

			structure = structure->Next();
		}

		SafeAllocateAndMove(nodes, &parentNode.mChildren, &parentNode.mNumChildren);
	}

	unique_ptr<aiNode> ConvertNodeStructure(const NodeStructure& nodeStructure) {
		ai_assert(IsNode(nodeStructure));
		StructureType type = nodeStructure.GetStructureType();
		switch (nodeStructure.GetStructureType()) {
			case kStructureNode: return ConvertNode(static_cast<const NodeStructure&>(nodeStructure));
			case kStructureBoneNode: return ConvertBoneNode(static_cast<const BoneNodeStructure&>(nodeStructure));
			case kStructureGeometryNode: return ConvertGeometryNode(static_cast<const GeometryNodeStructure&>(nodeStructure));
			case kStructureLightNode: return ConvertLightNode(static_cast<const LightNodeStructure&>(nodeStructure));
			case kStructureCameraNode: return ConvertCameraNode(static_cast<const CameraNodeStructure&>(nodeStructure));
			default:
				ai_assert(false && "Invalid node structure type");
				break;
		}
		return unique_ptr<aiNode>();
	}

	void ConvertObjectStructure(const Structure& structure) {
		ai_assert(IsObject(structure));
		switch (structure.GetStructureType()) {
			case kStructureGeometryObject:
				meshMap[&structure] = static_cast<unsigned int>(meshes.size());
				meshes.push_back(ConvertGeometryObject(static_cast<const GeometryObjectStructure&>(structure)));
				break;

			case kStructureLightObject:
				lightMap[&structure] = static_cast<unsigned int>(lights.size());
				lights.push_back(ConvertLightObject(static_cast<const LightObjectStructure&>(structure)));
				break;

			case kStructureCameraObject:
				cameraMap[&structure] = static_cast<unsigned int>(cameras.size());
				cameras.push_back(ConvertCameraObject(static_cast<const CameraObjectStructure&>(structure)));
				break;

			case kStructureMaterial:
				materialMap[&structure] = static_cast<unsigned int>(materials.size());
				materials.push_back(ConvertMaterial(static_cast<const MaterialStructure&>(structure)));
				break;

			default:
				ai_assert(false && "Invalid object structure type");
				break;
		}
	}

	unique_ptr<aiMesh> ConvertGeometryObject(const GeometryObjectStructure& structure) {
		const Map<MeshStructure>* meshMap = structure.GetMeshMap();
		ai_assert(meshMap && "Geometry object lacks a valid mesh map");
		ai_assert(!meshMap->Empty() && "Geometry object lacks a valid mesh");

		const MeshStructure* meshStructure = meshMap->First();

		// TODO: Handle multiple meshes, visibility, shadow and motion blur.
		while (meshStructure) {
			if (meshStructure->GetKey() == 0) break;
			meshStructure = meshStructure->Next();
		}

		return ConvertMesh(*meshStructure);
	}

	unique_ptr<aiLight> ConvertLightObject(const LightObjectStructure& structure) {
		unique_ptr<aiLight> light(new aiLight());

		return move(light);
	}

	unique_ptr<aiCamera> ConvertCameraObject(const CameraObjectStructure& structure) {
		unique_ptr<aiCamera> camera(new aiCamera());

		return move(camera);
	}

	unique_ptr<aiMaterial> ConvertMaterial(const MaterialStructure& structure) {
		unique_ptr<aiMaterial> material(new aiMaterial());

		return move(material);
	}

	unique_ptr<aiNode> ConvertNode(const NodeStructure& structure) {
		unique_ptr<aiNode> node(new aiNode);

		node->mName = structure.GetNodeName();

		TransformVisitor transformVisitor(&node->mTransformation);
		VisitSubnodes(structure, transformVisitor);

		return move(node);
	}

	unique_ptr<aiNode> ConvertBoneNode(const BoneNodeStructure& structure) {
		unique_ptr<aiNode> node = ConvertNode(structure);

		return move(node);
	}

	unique_ptr<aiNode> ConvertGeometryNode(const GeometryNodeStructure& structure) {
		unique_ptr<aiNode> node = ConvertNode(structure);

		const GeometryObjectStructure* geometry = structure.GetGeometryObjectStructure();
		ai_assert(geometry && "Geometry node must contain exactly 1 geometry object");
		node->mMeshes = new unsigned int[1];
		node->mMeshes[0] = meshMap.at(geometry);
		node->mNumMeshes = 1;

		// TODO: Materials, visibility, shadow, motion blur.
		// const Array<const MaterialStructure*, 4>& materials = structure.GetMaterialStructureArray();

		return move(node);
	}

	unique_ptr<aiNode> ConvertLightNode(const LightNodeStructure& structure) {
		unique_ptr<aiNode> node = ConvertNode(structure);

		return move(node);
	}

	unique_ptr<aiNode> ConvertCameraNode(const CameraNodeStructure& structure) {
		unique_ptr<aiNode> node = ConvertNode(structure);

		return move(node);
	}

	unique_ptr<aiMesh> ConvertMesh(const MeshStructure& structure) {
		unique_ptr<aiMesh> mesh(new aiMesh());

		mesh->mPrimitiveTypes = ConvertPrimitiveType(structure.GetMeshPrimitive());

		const Structure *subStructure = structure.GetFirstSubnode();
		while (subStructure) {
			switch (subStructure->GetStructureType()) {
				case kStructureVertexArray: {
					const VertexArrayStructure& vertexArrayStructure = *static_cast<const VertexArrayStructure *>(subStructure);
					const String& arrayAttrib = vertexArrayStructure.GetArrayAttrib();
					if (arrayAttrib == "position") {

					} else if (arrayAttrib == "normal") {

					} else if (arrayAttrib == "tangent") {

					} else if (arrayAttrib == "bitangent") {

					} else if (arrayAttrib == "color") {

					} else if (arrayAttrib == "texcoord") {

					}
				} break;

				case kStructureIndexArray: {
					ai_assert(!mesh->mNumFaces && "Only a single index array per mesh is allowed");
					const IndexArrayStructure& indexArrayStructure = *static_cast<const IndexArrayStructure *>(subStructure);
					ConvertIndexArray(indexArrayStructure, &mesh->mFaces, &mesh->mNumFaces);
					// TODO: Process index array here.
				} break;

				default: break;
			}
		}

		// TODO: Handle no index array.

		// TODO: Handle skin structure.

		return move(mesh);
	}

	unsigned int ConvertPrimitiveType(const String& primitive) {
		if (primitive == "points") return aiPrimitiveType_POINT;
		if (primitive == "lines") return aiPrimitiveType_LINE;
		if (primitive == "line_strip") ThrowException("Line strip not yet supported");
		if (primitive == "triangles") return aiPrimitiveType_TRIANGLE;
		if (primitive == "triangle_strip") ThrowException("Triangle strip not yet supported");
		if (primitive == "quads") return aiPrimitiveType_POLYGON;
		ThrowException("Invalid primitive type");
		return 0;
	}

	template <typename Type>
	void ConvertVertexArray(const VertexArrayStructure& structure, Type** pDest, unsigned int* pNumVertices) {

	}

	void ConvertIndexArray(const IndexArrayStructure& structure, aiFace** pDest, unsigned int* pNumFaces) {

	}

	// Move all generated meshes, animations, lights, cameras and textures to the output scene.
	void TransferDataToScene() {
		ai_assert(!out->mMeshes && !out->mNumMeshes);
		SafeAllocateAndMove(meshes, &out->mMeshes, &out->mNumMeshes);
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

	typedef std::unordered_map<const Structure*, unsigned int> StructureMap;
	StructureMap meshMap;
	StructureMap materialMap;
	StructureMap animationMap;
	StructureMap lightMap;
	StructureMap cameraMap;

	aiScene* const out;
	const OpenGexDataDescription& desc;
};

void ConvertToAssimpScene( aiScene *pScene, const OpenGexDataDescription& desc ) {
	OpenGEXConverter conv(pScene, desc);
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
