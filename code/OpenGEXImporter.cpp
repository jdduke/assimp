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

namespace std {
template<typename a, typename b>
struct hash< std::pair<a, b> > {
private:
	const hash<a> ah;
	const hash<b> bh;
public:
	hash() : ah(), bh() {}
	size_t operator()(const std::pair<a, b>& p) const {
		return ah(p.first) ^ bh(p.second);
	}
};
}
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

void ConvertFloatArray(const float* data, aiVector3D& v) {
	v.x = data[0];
	v.y = data[1];
	v.z = data[2];
}

void ConvertFloatArray(const float* data, aiColor4D& c) {
	c.r = data[0];
	c.g = data[1];
	c.b = data[2];
	c.a = data[3];
}

void ConvertFloatArray(const float* data, aiMatrix4x4& m) {
	m.a1 = data[0];
	m.a2 = data[1];
	m.a3 = data[2];
	m.a4 = data[3];
	m.b1 = data[4];
	m.b2 = data[5];
	m.b3 = data[6];
	m.b4 = data[7];
	m.c1 = data[8];
	m.c2 = data[9];
	m.c3 = data[10];
	m.c4 = data[11];
	m.d1 = data[12];
	m.d2 = data[13];
	m.d3 = data[14];
	m.d4 = data[15];
}

size_t GetArrayAttribIndex(const String& attrib) {
	const char* attribCstr = static_cast<const char*>(attrib);

	const char* bracketBegin = strchr(attribCstr, '[');
	if (!bracketBegin) return 0;

	const char* bracketEnd = strchr(bracketBegin, ']');
	if (!bracketEnd) return 0;

	size_t n = bracketEnd - bracketBegin - 1;
	if (!n) return 0;

	std::string attribIndexStr(bracketBegin + 1, n);
	return std::stoul(attribIndexStr);
}

template <typename Type>
void ConvertVertexArray(const VertexArrayStructure& structure, Type** pVertexArray, unsigned int* pNumVertices) {
	const DataStructure<FloatDataType>& dataStructure = *structure.GetDataStructure();
	int32 arraySize = dataStructure.GetArraySize();
	int32 elementCount = dataStructure.GetDataElementCount();
	int32 vertexCount = elementCount / arraySize;
	const float *data = &dataStructure.GetDataElement(0);

	*pVertexArray = new Type[vertexCount];
	if (*pNumVertices) {
		ai_assert(*pNumVertices == vertexCount);
	}
	*pNumVertices = vertexCount;

	for (int32 i = 0; i < vertexCount; ++i, data += arraySize) {
		ConvertFloatArray(data, (*pVertexArray)[i]);
	}
}

template <typename DataType>
void ConvertTypedIndexArray(const PrimitiveStructure& primitiveStructure, aiFace** pFaceArray, unsigned int* pNumFaces) {
	const DataStructure<DataType>& dataStructure = static_cast<const DataStructure<DataType>&>(primitiveStructure);
	const auto* data = &dataStructure.GetDataElement(0);

	int32 arraySize = dataStructure.GetArraySize();
	int32 elementCount = dataStructure.GetDataElementCount();
	int32 faceCount = elementCount / arraySize;

	*pFaceArray = new aiFace[faceCount];
	*pNumFaces = faceCount;

	for (int32 i = 0; i < faceCount; ++i, data += arraySize) {
		(*pFaceArray)[i].mNumIndices = arraySize;
		(*pFaceArray)[i].mIndices = new unsigned int[arraySize];
		for (int32 j = 0; j < arraySize; ++j) {
			(*pFaceArray)[i].mIndices[j] = static_cast<unsigned int>(data[j]);
		}
	}
}

void ConvertIndexArray(const IndexArrayStructure& structure, aiFace** pFaceArray, unsigned int* pNumFaces) {
	const PrimitiveStructure* primitiveStructure = structure.GetPrimitiveStructure();
	StructureType type = primitiveStructure->GetStructureType();
	if (type == kDataUnsignedInt8) {
		return ConvertTypedIndexArray<UnsignedInt8DataType>(*primitiveStructure, pFaceArray, pNumFaces);
	} else if (type == kDataUnsignedInt16) {
		return ConvertTypedIndexArray<UnsignedInt16DataType>(*primitiveStructure, pFaceArray, pNumFaces);
	} else if (type == kDataUnsignedInt32) {
		return ConvertTypedIndexArray<UnsignedInt32DataType>(*primitiveStructure, pFaceArray, pNumFaces);
	} else {
		return ConvertTypedIndexArray<UnsignedInt64DataType>(*primitiveStructure, pFaceArray, pNumFaces);
	}
}

template <typename Type>
void SafeAllocateAndMove(std::vector< std::unique_ptr<Type> >& source, Type*** pDest, unsigned int* pCount) {
	if (source.empty()) {
		*pCount = 0;
		*pDest = nullptr;
		return;
	}
	*pCount = static_cast<unsigned int>(source.size());
	*pDest = new Type*[source.size()]();
	for (size_t i = 0; i < source.size(); ++i) {
		(*pDest)[i] = source[i].release();
	}
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
		: out(out)
		, desc(desc) {
		ConvertRootNode();

		TransferDataToScene();

		if (out->mNumMeshes == 0) {
			out->mFlags |= AI_SCENE_FLAGS_INCOMPLETE;
		} else {
			out->mFlags |= AI_SCENE_FLAGS_NON_VERBOSE_FORMAT;
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
			if (IsObject(*structure)) {
				ConvertObjectStructure(*structure);
			}
			structure = structure->Next();
		}
	}

	// Nodes are hierarchical structures.
	void ConvertNodes(const Structure& parentStructure, aiNode& parentNode) {
		const Structure* structure = parentStructure.GetFirstSubnode();

		std::vector< std::unique_ptr<aiNode> > nodes;
		while (structure) {
			if (IsNode(*structure)) {
				unique_ptr<aiNode> node(ConvertNodeStructure(*static_cast<const NodeStructure*>(structure)));
				ai_assert(node);
				node->mParent = &parentNode;
				ConvertNodes(*structure, *node);
				nodes.push_back(move(node));
			}

			structure = structure->Next();
		}

		SafeAllocateAndMove(nodes, &parentNode.mChildren, &parentNode.mNumChildren);
	}

	unique_ptr<aiNode> ConvertNodeStructure(const NodeStructure& nodeStructure) {
		ai_assert(IsNode(nodeStructure));
		StructureType type = nodeStructure.GetStructureType();
		switch (nodeStructure.GetStructureType()) {
			case kStructureNode:         return ConvertNode(static_cast<const NodeStructure&>(nodeStructure));
			case kStructureBoneNode:     return ConvertBoneNode(static_cast<const BoneNodeStructure&>(nodeStructure));
			case kStructureGeometryNode: return ConvertGeometryNode(static_cast<const GeometryNodeStructure&>(nodeStructure));
			case kStructureLightNode:    return ConvertLightNode(static_cast<const LightNodeStructure&>(nodeStructure));
			case kStructureCameraNode:   return ConvertCameraNode(static_cast<const CameraNodeStructure&>(nodeStructure));
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
				rawMeshMap[&structure] = static_cast<unsigned int>(meshes.size());
				rawMeshes.push_back(ConvertGeometryObject(static_cast<const GeometryObjectStructure&>(structure)));
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

	void EnsureDefaultMaterial() {
		materialMap[nullptr] = static_cast<unsigned int>(materials.size());
		materials.push_back(CreateDefaultMaterial());
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
		return CreateDefaultMaterial();

		/* TODO: Convert material
		unique_ptr<aiMaterial> material(new aiMaterial());

		return move(material);
		*/
	}

	static unique_ptr<aiMaterial> CreateDefaultMaterial() {
		unique_ptr<aiMaterial> material(new aiMaterial());

		aiString s;
		s.Set(AI_DEFAULT_MATERIAL_NAME);
		material->AddProperty(&s, AI_MATKEY_NAME);
		aiColor4D clrDiffuse(0.6f, 0.6f, 0.6f, 1.0f);
		material->AddProperty(&clrDiffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
		material->AddProperty(&clrDiffuse, 1, AI_MATKEY_COLOR_SPECULAR);
		clrDiffuse = aiColor4D(0.05f, 0.05f, 0.05f, 1.0f);
		material->AddProperty(&clrDiffuse, 1, AI_MATKEY_COLOR_AMBIENT);

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

	unique_ptr<aiMesh> CloneMesh(const aiMesh& mesh) {
		unique_ptr<aiMesh> clone(new aiMesh());
		// TODO: Handle multiple mesh references.
		return move(clone);
	}

	unsigned int FindOrInsertMeshWithMaterial(const GeometryObjectStructure* geometry, const MaterialStructure* material) {
		if (!material) {
			EnsureDefaultMaterial();
		}

		const auto meshMaterialKey = std::make_pair(geometry, material);
		const auto meshMaterialIt = meshMaterialMap.find(meshMaterialKey);
		if (meshMaterialIt != meshMaterialMap.end()) {
			return meshMaterialIt->second;
		}

		// First look at the raw meshes container. If it contains a mesh for the provided |geometry|, "consume" that mesh.
		// Otherwise, clone the already consumed mesh, as it's already been used for a different material.
		unique_ptr<aiMesh> mesh;
		const auto rawMeshIt = rawMeshMap.find(geometry);
		if (rawMeshIt != rawMeshMap.end()) {
			mesh = move(rawMeshes[rawMeshIt->second]);
			rawMeshMap.erase(rawMeshIt);
		} else {
			mesh = CloneMesh(*meshes[meshMap.at(geometry)]);
		}
		ai_assert(mesh && "Invalid mesh");

		mesh->mMaterialIndex = materialMap.at(material);
		unsigned int meshIndex = static_cast<unsigned int>(meshes.size());
		meshes.push_back(move(mesh));
		meshMaterialMap[meshMaterialKey] = meshIndex;
		meshMap[geometry] = meshIndex;
		return meshIndex;
	}

	unique_ptr<aiNode> ConvertGeometryNode(const GeometryNodeStructure& structure) {
		unique_ptr<aiNode> node = ConvertNode(structure);

		const GeometryObjectStructure* geometry = structure.GetGeometryObjectStructure();
		ai_assert(geometry && "Geometry node must contain exactly 1 geometry object");

		// TODO: Optimize case where a given mesh is only referenced once.
		const Array<const MaterialStructure*, 4>& materials = structure.GetMaterialStructureArray();
		if (materials.Empty()) {
			node->mMeshes = new unsigned int[1];
			node->mMeshes[0] = FindOrInsertMeshWithMaterial(geometry, nullptr);
			node->mNumMeshes = 1;
		} else {
			node->mMeshes = new unsigned int[materials.GetElementCount()];
			node->mNumMeshes = materials.GetElementCount();
			const MaterialStructure** it = materials.begin();
			for (size_t i = 0; it != materials.end(); ++it, ++i) {
				node->mMeshes[i] = FindOrInsertMeshWithMaterial(geometry, *it);
			}
		}

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

	unique_ptr<aiMesh> ConvertMesh(const MeshStructure& structure) {
		unique_ptr<aiMesh> mesh(new aiMesh());

		mesh->mPrimitiveTypes = ConvertPrimitiveType(structure.GetMeshPrimitive());

		const Structure *subStructure = structure.GetFirstSubnode();
		while (subStructure) {
			switch (subStructure->GetStructureType()) {
				case kStructureVertexArray: {
					const VertexArrayStructure& vertexArrayStructure = *static_cast<const VertexArrayStructure *>(subStructure);
					const String& arrayAttrib = vertexArrayStructure.GetArrayAttrib();
					size_t attribIndex = GetArrayAttribIndex(arrayAttrib);
					if (arrayAttrib == "position") {
						if (attribIndex > 0) continue;
						ConvertVertexArray(vertexArrayStructure, &mesh->mVertices, &mesh->mNumVertices);
					} else if (arrayAttrib == "normal") {
						if (attribIndex > 0) continue;
						ConvertVertexArray(vertexArrayStructure, &mesh->mNormals, &mesh->mNumVertices);
					} else if (arrayAttrib == "tangent") {
						if (attribIndex > 0) continue;
						ConvertVertexArray(vertexArrayStructure, &mesh->mTangents, &mesh->mNumVertices);
					} else if (arrayAttrib == "bitangent") {
						if (attribIndex > 0) continue;
						ConvertVertexArray(vertexArrayStructure, &mesh->mBitangents, &mesh->mNumVertices);
					} else if (arrayAttrib == "color") {
						if (attribIndex >= AI_MAX_NUMBER_OF_COLOR_SETS) continue;
						ConvertVertexArray(vertexArrayStructure, &(mesh->mColors[attribIndex]), &mesh->mNumVertices);
					} else if (arrayAttrib == "texcoord") {
						if (attribIndex >= AI_MAX_NUMBER_OF_TEXTURECOORDS) continue;
						ConvertVertexArray(vertexArrayStructure, &(mesh->mTextureCoords[attribIndex]), &mesh->mNumVertices);
						mesh->mNumUVComponents[attribIndex] = vertexArrayStructure.GetDataStructure()->GetArraySize();
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
			subStructure = subStructure->Next();
		}

		// TODO: Handle no index array.

		// TODO: Handle skin structure.

		return move(mesh);
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

	typedef std::unordered_map<const Structure*, unsigned int> StructureMap;

	std::vector< std::unique_ptr<aiMesh> > rawMeshes;
	std::vector< std::unique_ptr<aiMesh> > meshes;
	std::vector< std::unique_ptr<aiMaterial> > materials;
	std::vector< std::unique_ptr<aiAnimation> > animations;
	std::vector< std::unique_ptr<aiLight> > lights;
	std::vector< std::unique_ptr<aiCamera> > cameras;

	StructureMap rawMeshMap;
	StructureMap materialMap;
	StructureMap animationMap;
	StructureMap lightMap;
	StructureMap cameraMap;

	typedef std::unordered_map<std::pair<const GeometryObjectStructure*, const MaterialStructure*>, unsigned int> MeshMaterialMap;
	StructureMap meshMap;
	MeshMaterialMap meshMaterialMap;

	aiScene* const out;
	const OpenGexDataDescription& desc;
};

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

	OpenGEXConverter conv(pScene, openGexDataDescription);
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

template<> const std::string Assimp::LogFunctions<Assimp::OpenGEX::OpenGEXConverter>::log_prefix = "OpenGEX: ";
template<> const std::string Assimp::LogFunctions<Assimp::OpenGEX::OpenGEXImporter>::log_prefix = "OpenGEX: ";

#endif // ASSIMP_BUILD_NO_OPENGEX_IMPORTER
