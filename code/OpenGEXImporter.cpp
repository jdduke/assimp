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

#include "MakeVerboseFormat.h"
#include "./../contrib/OpenGEX/OpenGEX.h"

#include <map>
#include <memory>

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

using std::auto_ptr;

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

const char* DataResultToString(DataResult result) {
#define CASE_DATA_RESULT(NAME)	case kDataOpenGex ## NAME: return #NAME;
	switch (result) {
		CASE_DATA_RESULT(InvalidUpDirection);
		CASE_DATA_RESULT(InvalidTranslationKind);
		CASE_DATA_RESULT(InvalidRotationKind);
		CASE_DATA_RESULT(InvalidScaleKind);
		CASE_DATA_RESULT(DuplicateLod);
		CASE_DATA_RESULT(MissingLodSkin);
		CASE_DATA_RESULT(MissingLodMorph);
		CASE_DATA_RESULT(DuplicateMorph);
		CASE_DATA_RESULT(UndefinedLightType);
		CASE_DATA_RESULT(UndefinedCurve);
		CASE_DATA_RESULT(UndefinedAtten);
		CASE_DATA_RESULT(DuplicateVertexArray);
		CASE_DATA_RESULT(PositionArrayRequired);
		CASE_DATA_RESULT(VertexCountUnsupported);
		CASE_DATA_RESULT(IndexValueUnsupported);
		CASE_DATA_RESULT(IndexArrayRequired);
		CASE_DATA_RESULT(VertexCountMismatch);
		CASE_DATA_RESULT(BoneCountMismatch);
		CASE_DATA_RESULT(BoneWeightCountMismatch);
		CASE_DATA_RESULT(InvalidBoneRef);
		CASE_DATA_RESULT(InvalidObjectRef);
		CASE_DATA_RESULT(InvalidMaterialRef);
		CASE_DATA_RESULT(MaterialIndexUnsupported);
		CASE_DATA_RESULT(DuplicateMaterialRef);
		CASE_DATA_RESULT(MissingMaterialRef);
		CASE_DATA_RESULT(TargetRefNotLocal);
		CASE_DATA_RESULT(InvalidTargetStruct);
		CASE_DATA_RESULT(InvalidKeyKind);
		CASE_DATA_RESULT(InvalidCurveType);
		CASE_DATA_RESULT(KeyCountMismatch);
		CASE_DATA_RESULT(EmptyKeyStructure);
		default:
			return "Unknown result";
	}
#undef CASE_DATA_RESULT
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

// Returns whether vertices are uniquely indexed (i.e., the mesh indexing is "verbose").
void ConvertIndexArray(const IndexArrayStructure& structure, aiFace** pFaceArray, unsigned int* pNumFaces) {
	// TODO: Account for index winding order (structure.GetFrontFace()).
	const PrimitiveStructure* primitiveStructure = structure.GetPrimitiveStructure();
	ai_assert(primitiveStructure &&	"Invalid primitive structure handle");
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

template <typename T>
void CloneArray(const T* source, unsigned int count, T** dest) {
	if (!source || !count) {
		*dest = NULL;
		return;
	}

	*dest = new T[count];
	// Don't use memcpy as we might need assignment operator semantics (e.g., aiFace);
	std::copy(source, source + count, *dest);
}

void GenerateDefaultIndexArray(aiMesh* mesh) {
	unsigned int stride = 3;
	switch (mesh->mPrimitiveTypes) {
		case aiPrimitiveType_POINT:
			stride = 1;
			break;
		case aiPrimitiveType_LINE:
			stride = 2;
			break;
		case aiPrimitiveType_TRIANGLE:
			stride = 3;
			break;
		case aiPrimitiveType_POLYGON:
			stride = mesh->mNumVertices;
			break;
		default:
			ai_assert(false && "Invalid primitive type.");
			break;
	}

	unsigned int faceCount = mesh->mNumVertices / stride;
	mesh->mFaces = new aiFace[faceCount];
	mesh->mNumFaces = faceCount;

	for (unsigned int fi = 0, vi = 0; fi < faceCount; ++fi, vi += stride) {
		mesh->mFaces[fi].mNumIndices = stride;
		mesh->mFaces[fi].mIndices = new unsigned int[stride];
		for (unsigned int j = 0; j < stride; ++j) {
			mesh->mFaces[fi].mIndices[j] = vi + j;
		}
	}
}

template <typename T>
void SafeAllocateAndMove(std::vector< T* >& source, T*** pDest, unsigned int* pCount) {
	if (source.empty()) {
		*pCount = 0;
		*pDest = nullptr;
		return;
	}
	*pCount = static_cast<unsigned int>(source.size());
	*pDest = new T*[source.size()]();
	for (size_t i = 0; i < source.size(); ++i) {
		(*pDest)[i] = source[i];
	}
	source.clear();
}

template <typename T>
void SafeDelete(std::vector< T* >& source) {
	for (size_t i = 0; i < source.size(); ++i)
		delete source[i];
	source.clear();
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

struct LightVisitor : public Visitor {
	explicit LightVisitor(aiLight* light) : light(light) {}

	virtual void Visit(const ColorStructure& color) {
		if (color.GetAttribString() == "light") {
			light->mColorDiffuse =
				aiColor3D(color.GetColor()[0], color.GetColor()[1], color.GetColor()[2]);
		}
		// TODO: Handle specular/ambient colors?
	}

	virtual void Visit(const ParamStructure& param) {
		if (param.GetAttribString() == "intensity") {
			light->mAttenuationConstant = param.GetParam();
		}
	}

	virtual void Visit(const TextureStructure& texture) {
		if (texture.GetAttribString() == "projection") {

		}
	}

	virtual void Visit(const AttenStructure& atten) {
		const String& attenKind = atten.GetAttenKind();
		const String& curveType = atten.GetCurveType();

		if (attenKind == "distance") {
			if ((curveType == "linear") || (curveType == "smooth"))
			{
				float beginParam = atten.GetBeginParam();
				float endParam = atten.GetEndParam();

				// TODO: Process linear or smooth attenuation here.
			}
			else if (curveType == "inverse")
			{
				float scaleParam = atten.GetScaleParam();
				float linearParam = atten.GetLinearParam();

				// TODO: Process inverse attenuation here.
			}
			else if (curveType == "inverse_square")
			{
				float scaleParam = atten.GetScaleParam();
				float quadraticParam = atten.GetQuadraticParam();

				// TODO: Process inverse square attenuation here.
			}
			else
			{
				ai_assert(false && "Invalid light curve type");
			}
		}
		else if (attenKind == "angle")
		{
			float endParam = atten.GetEndParam();

			// TODO: Process angular attenutation here.
		}
		else if (attenKind == "cos_angle")
		{
			float endParam = atten.GetEndParam();

		}
	}

private:
	aiLight* const light;
};

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

private:
	aiMatrix4x4* const result;
};

aiLight* CloneLight(const aiLight& light) {
	auto_ptr<aiLight> clone(new aiLight());
	*clone = light;
	return clone.release();
}

aiCamera* CloneCamera(const aiCamera& camera) {
	auto_ptr<aiCamera> clone(new aiCamera());
	*clone = camera;
	return clone.release();
}

aiMesh* CloneMesh(const aiMesh& mesh) {
	auto_ptr<aiMesh> clone(new aiMesh());

	clone->mPrimitiveTypes = mesh.mPrimitiveTypes;
	clone->mNumVertices = mesh.mNumVertices;
	clone->mNumFaces = mesh.mNumFaces;
	clone->mNumBones = mesh.mNumBones;
	clone->mMaterialIndex = mesh.mMaterialIndex;
	clone->mNumAnimMeshes = mesh.mNumAnimMeshes;

	CloneArray(mesh.mVertices, mesh.mNumVertices, &clone->mVertices);
	CloneArray(mesh.mNormals, mesh.mNumVertices, &clone->mNormals);
	CloneArray(mesh.mTangents, mesh.mNumVertices, &clone->mTangents);
	CloneArray(mesh.mBitangents, mesh.mNumVertices, &clone->mBitangents);
	CloneArray(mesh.mFaces, mesh.mNumFaces , &clone->mFaces);
	CloneArray(mesh.mBones, mesh.mNumBones, &clone->mBones);
	CloneArray(mesh.mAnimMeshes, mesh.mNumAnimMeshes, &clone->mAnimMeshes);

	return clone.release();
}

class OpenGEXConverter : public LogFunctions<OpenGEXConverter> {
public:

	OpenGEXConverter(aiScene* out, const OpenGexDataDescription& desc)
		: out(out)
		, desc(desc) {
		ConvertRootNode();

		TransferDataToScene();

		if (out->mNumMeshes == 0) {
			out->mFlags |= AI_SCENE_FLAGS_INCOMPLETE;
		}
	}

	~OpenGEXConverter() {
		SafeDelete(rawMeshes);
		// Pointer containers should all be empty if import is successful.
		SafeDelete(meshes);
		SafeDelete(materials);
		SafeDelete(animations);
		SafeDelete(lights);
		SafeDelete(cameras);
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

		std::vector< aiNode* > nodes;
		while (structure) {
			if (IsNode(*structure)) {
				aiNode* node(ConvertNodeStructure(*static_cast<const NodeStructure*>(structure)));
				ai_assert(node);
				node->mParent = &parentNode;
				ConvertNodes(*structure, *node);
				nodes.push_back(node);
			}

			structure = structure->Next();
		}

		SafeAllocateAndMove(nodes, &parentNode.mChildren, &parentNode.mNumChildren);
	}

	aiNode* ConvertNodeStructure(const NodeStructure& nodeStructure) {
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
		return NULL;
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

	aiMesh* ConvertGeometryObject(const GeometryObjectStructure& structure) {
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

	aiLight* ConvertLightObject(const LightObjectStructure& structure) {
		auto_ptr<aiLight> light(new aiLight());

		if (structure.GetTypeString() == "infinite") {
			light->mType = aiLightSource_DIRECTIONAL;
		} else if (structure.GetTypeString() == "point") {
			light->mType = aiLightSource_POINT;
		} else if (structure.GetTypeString() == "spot") {
			light->mType = aiLightSource_SPOT;
		} else {
			ai_assert(false && "Invalid light type");
		}

		LightVisitor lightVisitor(light.get());
		VisitSubnodes(structure, lightVisitor);

		return light.release();
	}

	aiCamera* ConvertCameraObject(const CameraObjectStructure& structure) {
		auto_ptr<aiCamera> camera(new aiCamera());
		if (structure.GetFocalLength()) {
			camera->mHorizontalFOV = 2.f * std::atan2(1.f, structure.GetFocalLength());
		}
		camera->mClipPlaneNear = structure.GetNearDepth();
		camera->mClipPlaneFar = structure.GetFarDepth();
		return camera.release();
	}

	aiMaterial* ConvertMaterial(const MaterialStructure& structure) {
		return CreateDefaultMaterial();

		/* TODO: Implement
		aiMaterial* material(new aiMaterial());

		return material;
		*/
	}

	static aiMaterial* CreateDefaultMaterial() {
		auto_ptr<aiMaterial> material(new aiMaterial());

		aiString s;
		s.Set(AI_DEFAULT_MATERIAL_NAME);
		material->AddProperty(&s, AI_MATKEY_NAME);
		aiColor4D clrDiffuse(0.6f, 0.6f, 0.6f, 1.0f);
		material->AddProperty(&clrDiffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
		material->AddProperty(&clrDiffuse, 1, AI_MATKEY_COLOR_SPECULAR);
		clrDiffuse = aiColor4D(0.05f, 0.05f, 0.05f, 1.0f);
		material->AddProperty(&clrDiffuse, 1, AI_MATKEY_COLOR_AMBIENT);

		return material.release();
	}

	aiNode* ConvertNode(const NodeStructure& structure) {
		auto_ptr<aiNode> node(new aiNode);

		TransformVisitor transformVisitor(&node->mTransformation);
		VisitSubnodes(structure, transformVisitor);

		return node.release();
	}

	aiNode* ConvertBoneNode(const BoneNodeStructure& structure) {
		auto_ptr<aiNode> node(ConvertNode(structure));
		// TODO: Implement;
		return node.release();
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
		aiMesh* mesh = NULL;
		const auto rawMeshIt = rawMeshMap.find(geometry);
		if (rawMeshIt != rawMeshMap.end()) {
			mesh = rawMeshes[rawMeshIt->second];
			rawMeshes.erase(rawMeshes.begin() +	rawMeshIt->second);
			rawMeshMap.erase(rawMeshIt);
		} else {
			mesh = CloneMesh(*meshes[meshMap.at(geometry)]);
		}
		ai_assert(mesh && "Invalid mesh");

		mesh->mMaterialIndex = materialMap.at(material);
		unsigned int meshIndex = static_cast<unsigned int>(meshes.size());
		meshes.push_back(mesh);
		meshMaterialMap[meshMaterialKey] = meshIndex;
		meshMap[geometry] = meshIndex;
		return meshIndex;
	}

	aiNode* ConvertGeometryNode(const GeometryNodeStructure& structure) {
		auto_ptr<aiNode> node(ConvertNode(structure));

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

		return node.release();
	}

	aiNode* ConvertLightNode(const LightNodeStructure& structure) {
		const auto lightIt = lightMap.find(structure.GetObjectStructure());
		if (lightIt == lightMap.end())
			ThrowException("Invalid light reference");

		auto_ptr<aiNode> node(ConvertNode(structure));
		ai_assert(node->mName.length && "Invalid (empty) light node name");

		// If the light has already been assigned to a node, clone it. Otherwise
		// assign it to this node.
		aiLight* light = lights[lightIt->second];
		if (light->mName.length) {
			lights.push_back(CloneLight(*light));
			lights.back()->mName = node->mName;
		} else {
			light->mName = node->mName;
		}
		return node.release();
	}

	aiNode* ConvertCameraNode(const CameraNodeStructure& structure) {
		const auto cameraIt = cameraMap.find(structure.GetObjectStructure());
		if (cameraIt == cameraMap.end())
			ThrowException("Invalid camera reference");

		auto_ptr<aiNode> node(ConvertNode(structure));
		ai_assert(node->mName.length && "Invalid (empty) camera node name");

		// If the camera has already been assigned to a node, clone it. Otherwise
		// assign it to this node.
		aiCamera* camera = cameras[cameraIt->second];
		if (camera->mName.length) {
			cameras.push_back(CloneCamera(*camera));
			cameras.back()->mName = node->mName;
		} else {
			camera->mName = node->mName;
		}
		return node.release();
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

	aiMesh* ConvertMesh(const MeshStructure& structure) {
		auto_ptr<aiMesh> mesh(new aiMesh());
		mesh->mPrimitiveTypes = ConvertPrimitiveType(structure.GetMeshPrimitive());

		const Structure *subStructure = structure.GetFirstSubnode();
		bool meshIsVerbose = false;
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
					ai_assert(!mesh->mNumFaces && !mesh->mFaces && "Only a single index array per mesh is allowed");
					const IndexArrayStructure& indexArrayStructure = *static_cast<const IndexArrayStructure *>(subStructure);
					ConvertIndexArray(indexArrayStructure, &mesh->mFaces, &mesh->mNumFaces);
				} break;

				default: break;
			}
			subStructure = subStructure->Next();
		}
		ai_assert(mesh->mVertices);
		ai_assert(mesh->mNumVertices);

		if (mesh->mFaces) {
			// Ensure pseudo, "verbose" indexing.
			if (!MakeVerboseFormatProcess::HasVerboseFormat(mesh.get()))
				MakeVerboseFormatProcess::MakeVerboseFormat(mesh.get());
		} else {
			// Generate a default face indexing if none has been provided.
			GenerateDefaultIndexArray(mesh.get());
		}

		// TODO: Handle skin structure.

		return mesh.release();
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

	typedef std::map<const Structure*, unsigned int> StructureMap;

	std::vector< aiMesh* > rawMeshes;
	std::vector< aiMesh* > meshes;
	std::vector< aiMaterial* > materials;
	std::vector< aiAnimation* > animations;
	std::vector< aiLight* > lights;
	std::vector< aiCamera* > cameras;

	StructureMap rawMeshMap;
	StructureMap materialMap;
	StructureMap animationMap;
	StructureMap lightMap;
	StructureMap cameraMap;

	typedef std::map<std::pair<const GeometryObjectStructure*, const MaterialStructure*>, unsigned int> MeshMaterialMap;
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
		ThrowException(std::string("Failed to load OpenGEX file: ") + DataResultToString(result));
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
