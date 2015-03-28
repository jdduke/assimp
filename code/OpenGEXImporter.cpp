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

#include "fast_atof.h"
#include "MakeVerboseFormat.h"
#include "TinyFormatter.h"
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

using Assimp::Formatter::format;
using std::auto_ptr;

namespace Assimp {
namespace OpenGEX {

namespace {

LogFunctions<OpenGEXImporter>& Logger() {
	static LogFunctions<OpenGEXImporter> sLogger;
	return sLogger;
}

template <typename T>
void CloneArray(const T* source, unsigned int count, T** dest) {
	if (!source || !count) {
		*dest = NULL;
		return;
	}

	*dest = new T[count];
	// Don't use memcpy as we might need assignment operator semantics
	// (e.g., for aiFace).
	std::copy(source, source + count, *dest);
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

// TODO: Remove the visitor pattern, it's used sparingly and is probably
// unnecessary boilerplate.
struct Visitor {
	
};

template <class Visitor>
void Visit(const Structure& structure, Visitor& visitor) {
#define CASE_VISIT_STRUCTURE(TYPE) case kStructure ## TYPE: return visitor.Visit(static_cast<const TYPE ## Structure&>(structure))
	switch (structure.GetStructureType()) {
		CASE_VISIT_STRUCTURE(Metric);
		CASE_VISIT_STRUCTURE(Name);
		CASE_VISIT_STRUCTURE(ObjectRef);
		CASE_VISIT_STRUCTURE(MaterialRef);
		CASE_VISIT_STRUCTURE(Matrix);
		CASE_VISIT_STRUCTURE(Transform);
		CASE_VISIT_STRUCTURE(Translation);
		CASE_VISIT_STRUCTURE(Rotation);
		CASE_VISIT_STRUCTURE(Scale);
		CASE_VISIT_STRUCTURE(MorphWeight);
		CASE_VISIT_STRUCTURE(Node);
		CASE_VISIT_STRUCTURE(BoneNode);
		CASE_VISIT_STRUCTURE(GeometryNode);
		CASE_VISIT_STRUCTURE(LightNode);
		CASE_VISIT_STRUCTURE(CameraNode);
		CASE_VISIT_STRUCTURE(VertexArray);
		CASE_VISIT_STRUCTURE(IndexArray);
		CASE_VISIT_STRUCTURE(BoneRefArray);
		CASE_VISIT_STRUCTURE(BoneCountArray);
		CASE_VISIT_STRUCTURE(BoneIndexArray);
		CASE_VISIT_STRUCTURE(BoneWeightArray);
		CASE_VISIT_STRUCTURE(Skeleton);
		CASE_VISIT_STRUCTURE(Skin);
		CASE_VISIT_STRUCTURE(Morph);
		CASE_VISIT_STRUCTURE(Mesh);
		CASE_VISIT_STRUCTURE(Object);
		CASE_VISIT_STRUCTURE(GeometryObject);
		CASE_VISIT_STRUCTURE(LightObject);
		CASE_VISIT_STRUCTURE(CameraObject);
		CASE_VISIT_STRUCTURE(Attrib);
		CASE_VISIT_STRUCTURE(Param);
		CASE_VISIT_STRUCTURE(Color);
		CASE_VISIT_STRUCTURE(Texture);
		CASE_VISIT_STRUCTURE(Atten);
		CASE_VISIT_STRUCTURE(Material);
		CASE_VISIT_STRUCTURE(Key);
		CASE_VISIT_STRUCTURE(Curve);
		CASE_VISIT_STRUCTURE(Time);
		CASE_VISIT_STRUCTURE(Value);
		CASE_VISIT_STRUCTURE(Track);
		CASE_VISIT_STRUCTURE(Animation);
		CASE_VISIT_STRUCTURE(Clip);
		CASE_VISIT_STRUCTURE(Extension);
		default: break;
	}
#undef CASE_VISIT_STRUCTURE
	return visitor.Visit(structure);
}

template <class Visitor>
void VisitSubnodes(const Structure& structure, Visitor& visitor) {
	const Structure* child = structure.GetFirstSubnode();
	while (child) {
		Visit(*child, visitor);
		child = child->Next();
	}
}

struct LightVisitor : public Visitor {
	explicit LightVisitor(aiLight* light) : light(light) {}

	void Visit(const Structure&) {}

	void Visit(const ColorStructure& color) {
		if (color.GetAttribString() == "light") {
			light->mColorDiffuse =
				aiColor3D(color.GetColor()[0], color.GetColor()[1], color.GetColor()[2]);
		}
		// TODO: Handle specular/ambient colors?
	}

	void Visit(const ParamStructure& param) {
		if (param.GetAttribString() == "intensity") {
			light->mAttenuationConstant = param.GetParam();
		}
	}

	void Visit(const TextureStructure& texture) {
		if (texture.GetAttribString() == "projection") {
			// TODO: Handle	projection texture?
		}
	}

	void Visit(const AttenStructure& atten) {
		const String& attenKind = atten.GetAttenKind();
		const String& curveType = atten.GetCurveType();

		if (attenKind == "distance") {
			if ((curveType == "linear") || (curveType == "smooth")) {
				float beginParam = atten.GetBeginParam();
				float endParam = atten.GetEndParam();

				// TODO: Process linear or smooth attenuation here.
			} else if (curveType == "inverse") {
				float scaleParam = atten.GetScaleParam();
				float linearParam = atten.GetLinearParam();
				// TODO: Process inverse attenuation here.
			} else if (curveType == "inverse_square") {
				float scaleParam = atten.GetScaleParam();
				float quadraticParam = atten.GetQuadraticParam();
				// TODO: Process inverse square attenuation here.
			} else {
				ai_assert(false && "Invalid light curve type");
			}
		} else if (attenKind == "angle") {
			float endParam = atten.GetEndParam();
			// TODO: Process angular attenutation here.
		} else if (attenKind == "cos_angle") {
			float endParam = atten.GetEndParam();
			// TODO: Process attentuation angle here.
		}
	}

private:
	aiLight* const light;
};

struct TransformVisitor : public Visitor {
	explicit TransformVisitor(aiMatrix4x4* result) : result(result) {}

	void Visit(const Structure&) {}

	void Visit(const TransformStructure& transform) {
		if (transform.GetObjectFlag()) return;
		const float* data = transform.GetTransform(0);
		// OpenGEX stores transforms in column-major order.
		*result *= aiMatrix4x4(data[0], data[4], data[8],  data[12],
		                       data[1], data[5], data[9],  data[13],
		                       data[2], data[6], data[10], data[14],
		                       data[3], data[7], data[11], data[15]);
	}

	void Visit(const TranslationStructure& transform) {
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

	void Visit(const RotationStructure& transform) {
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

	void Visit(const ScaleStructure& transform) {
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


struct MaterialVisitor : public Visitor {
	explicit MaterialVisitor(aiMaterial* material) : material(material) {}

	void Visit(const Structure&) {}

	void Visit(const ColorStructure& color) {
		const String& attrib = color.GetAttribString();
		aiColor4D value;
		ConvertFloatArray(color.GetColor(), value);
		if (attrib == "diffuse") {
			material->AddProperty(&value, 1, AI_MATKEY_COLOR_DIFFUSE);
		} else if (attrib == "specular") {
			material->AddProperty(&value, 1, AI_MATKEY_COLOR_SPECULAR);
		} else if (attrib == "emission") {
			material->AddProperty(&value, 1, AI_MATKEY_COLOR_EMISSIVE);
		} else if (attrib == "opacity") {
			Logger().LogWarn("Opacity material color attribute unsupported");
		} else if (attrib == "transparency") {
			material->AddProperty(&value, 1, AI_MATKEY_COLOR_TRANSPARENT);
		} else {
			Logger().LogWarn(format() << "Invalid material color attribute: " << attrib);
		}
	}

	void Visit(const ParamStructure& param) {
		const String& attrib = param.GetAttribString();
		if (attrib == "specular_power") {
			float specularPower = param.GetParam();
			material->AddProperty(&specularPower, 1, AI_MATKEY_SHININESS);
		} else {
			Logger().LogWarn(format() << "Invalid material parameter attribute: " << attrib);
		}
	}

	void Visit(const TextureStructure& texture) {
		// TODO: Convert texture transforms.
		const String& attrib = texture.GetAttribString();
		aiString textureName(static_cast<const char*>(texture.GetTextureName()));
		unsigned int textureCoordIndex = texture.GetTexcoordIndex();
		if (attrib == "diffuse") {
			material->AddProperty(&textureName, AI_MATKEY_TEXTURE_DIFFUSE(textureCoordIndex));
		} else if (attrib == "specular") {
			material->AddProperty(&textureName, AI_MATKEY_TEXTURE_SPECULAR(textureCoordIndex));
		} else if (attrib == "emission") {
			material->AddProperty(&textureName, AI_MATKEY_TEXTURE_EMISSIVE(textureCoordIndex));
		} else if (attrib == "opacity") {
			material->AddProperty(&textureName, AI_MATKEY_TEXTURE_OPACITY(textureCoordIndex));
		} else if (attrib == "transparency") {
			Logger().LogWarn("Transparency material texture attribute unsupported");
		} else if (attrib == "normal") {
			material->AddProperty(&textureName, AI_MATKEY_TEXTURE_NORMALS(textureCoordIndex));
		} else {
			Logger().LogWarn(format() << "Invalid material texture attribute: " << attrib);
		}
	}

	private:
		aiMaterial* const material;
};

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

unsigned int GetArrayAttribIndex(const String& attrib) {
	const char* attribCstr = static_cast<const char*>(attrib);

	const char* bracketBegin = strchr(attribCstr, '[');
	if (!bracketBegin) return 0;

	const char* bracketEnd = strchr(bracketBegin, ']');
	if (!bracketEnd) return 0;

	ptrdiff_t n = bracketEnd - bracketBegin - 1;
	if (n <= 0) return 0;

	std::string attribIndexStr(bracketBegin + 1, n);
	return strtoul_cppstyle(attribIndexStr.c_str());
}

template <typename Type>
void ConvertVertexArray(const VertexArrayStructure& structure, Type** pVertexArray, unsigned int* pNumVertices) {
	const DataStructure<FloatDataType>& dataStructure = *structure.GetDataStructure();
	const int32 arraySize = dataStructure.GetArraySize();
	const int32 elementCount = dataStructure.GetDataElementCount();
	const int32 vertexCount = elementCount / arraySize;
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
void ConvertTypedIndexArray(const PrimitiveStructure& primitiveStructure,
                            bool flipWindingOrder,
                            aiFace** ppFaceArray,
                            unsigned int* pNumFaces) {
	const DataStructure<DataType>& dataStructure =static_cast<const DataStructure<DataType>&>(primitiveStructure);
	const typename DataType::PrimType* pData = &dataStructure.GetDataElement(0);

	// Assimp assumes a counter-clockwise winding order by default.
	const int32 arraySize = dataStructure.GetArraySize();
	const int32 elementCount = dataStructure.GetDataElementCount();
	const int32 faceCount = elementCount / arraySize;

	*ppFaceArray = new aiFace[faceCount];
	*pNumFaces = faceCount;

	for (int32 i = 0; i < faceCount; ++i, pData += arraySize) {
		aiFace& pFace = (*ppFaceArray)[i];
		pFace.mNumIndices = arraySize;
		pFace.mIndices = new unsigned int[arraySize];
		for (int32 j = 0; j < arraySize; ++j) {
			if (flipWindingOrder) {
				pFace.mIndices[j] = static_cast<unsigned int>(pData[arraySize - j - 1]);
			} else {
				pFace.mIndices[j] = static_cast<unsigned int>(pData[j]);
			}
		}
	}
}

void CreateDefaultIndexArray(aiMesh* mesh) {
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
			ai_assert(false && "Invalid primitive type");
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

void ConvertIndexArray(const IndexArrayStructure& structure, aiFace** pFaceArray, unsigned int* pNumFaces) {
	// TODO: Account for material index and restart index.
	const PrimitiveStructure* primitiveStructure = structure.GetPrimitiveStructure();
	ai_assert(primitiveStructure &&	"Invalid primitive structure handle");

	const bool flipWindingOrder = structure.GetFrontFace() == "cw";
	const StructureType type = primitiveStructure->GetStructureType();
	if (type == kDataUnsignedInt8) {
		ConvertTypedIndexArray<UnsignedInt8DataType>(*primitiveStructure, flipWindingOrder, pFaceArray, pNumFaces);
	} else if (type == kDataUnsignedInt16) {
		ConvertTypedIndexArray<UnsignedInt16DataType>(*primitiveStructure, flipWindingOrder, pFaceArray, pNumFaces);
	} else if (type == kDataUnsignedInt32) {
		ConvertTypedIndexArray<UnsignedInt32DataType>(*primitiveStructure, flipWindingOrder, pFaceArray, pNumFaces);
	} else if (type == kDataUnsignedInt64) {
		ConvertTypedIndexArray<UnsignedInt64DataType>(*primitiveStructure, flipWindingOrder, pFaceArray, pNumFaces);
	} else {
		Logger().ThrowException(format() << "Invalid index array primitive type: " << type);
	}
}

unsigned int ConvertPrimitiveType(const String& primitive) {
	if (primitive == "points") return aiPrimitiveType_POINT;
	if (primitive == "lines") return aiPrimitiveType_LINE;
	if (primitive == "line_strip") Logger().ThrowException("Line strip primitives not yet supported");
	if (primitive == "triangles") return aiPrimitiveType_TRIANGLE;
	if (primitive == "triangle_strip") Logger().ThrowException("Triangle strip primitives not yet supported");
	if (primitive == "quads") return aiPrimitiveType_POLYGON;
	Logger().ThrowException(format() << "Invalid primitive type: " << primitive);
	return 0;
}

aiMesh* ConvertMesh(const MeshStructure& structure) {
	auto_ptr<aiMesh> mesh(new aiMesh());
	mesh->mPrimitiveTypes = ConvertPrimitiveType(structure.GetMeshPrimitive());

	const Structure *subStructure = structure.GetFirstSubnode();
	while (subStructure) {
		switch (subStructure->GetStructureType()) {
			case kStructureVertexArray: {
				const VertexArrayStructure& vertexArrayStructure = *static_cast<const VertexArrayStructure *>(subStructure);
				const String& arrayAttrib = vertexArrayStructure.GetArrayAttrib();
				const unsigned int attribIndex = GetArrayAttribIndex(arrayAttrib);
				// TODO: Warn for unsupported attribute indices?
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
		if (!MakeVerboseFormatProcess::HasVerboseFormat(mesh.get())) {
			MakeVerboseFormatProcess::MakeVerboseFormat(mesh.get());
		}
	} else {
		CreateDefaultIndexArray(mesh.get());
	}

	// TODO: Handle skin structure.

	return mesh.release();
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

aiMesh* ConvertGeometryObject(const GeometryObjectStructure& structure) {
	const Map<MeshStructure>* meshMap = structure.GetMeshMap();
	ai_assert(meshMap && "Geometry object lacks a valid mesh map");
	ai_assert(!meshMap->Empty() && "Geometry object lacks a valid mesh");

	// TODO: Handle morph structures.
	const MeshStructure* meshStructure = meshMap->First();
	while (meshStructure) {
		// The geometry structure may contain a mesh for each LOD, but we only care
		// about the first/highest LOD.
		if (meshStructure->GetKey() == 0) break;
		meshStructure = meshStructure->Next();
	}
	ai_assert(meshStructure && "Geometry object lacks a valid mesh");

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

aiLight* CloneLight(const aiLight& light) {
	auto_ptr<aiLight> clone(new aiLight());
	*clone = light;
	return clone.release();
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

aiCamera* CloneCamera(const aiCamera& camera) {
	auto_ptr<aiCamera> clone(new aiCamera());
	*clone = camera;
	return clone.release();
}

aiMaterial* CreateDefaultMaterial() {
	auto_ptr<aiMaterial> material(new aiMaterial());

	aiString s;
	s.Set(AI_DEFAULT_MATERIAL_NAME);
	material->AddProperty(&s, AI_MATKEY_NAME);
	aiColor4D clrDiffuse(0.6f, 0.6f, 0.6f, 1.0f);
	material->AddProperty(&clrDiffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
	material->AddProperty(&clrDiffuse, 1, AI_MATKEY_COLOR_SPECULAR);
	aiColor4D clrAmbient = aiColor4D(0.05f, 0.05f, 0.05f, 1.0f);
	material->AddProperty(&clrAmbient, 1, AI_MATKEY_COLOR_AMBIENT);

	return material.release();
}

aiMaterial* ConvertMaterial(const MaterialStructure& structure) {
	auto_ptr<aiMaterial> material(new aiMaterial());

	aiString s;
	if (structure.GetMaterialName()) {
		s.Set(structure.GetMaterialName());
		material->AddProperty(&s, AI_MATKEY_NAME);
	}

	bool twoSided = structure.GetTwoSidedFlag();
	material->AddProperty(&twoSided, 1, AI_MATKEY_TWOSIDED);

	MaterialVisitor visitor(material.get());
	VisitSubnodes(structure, visitor);

	return material.release();
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

class OpenGEXConverter : public LogFunctions<OpenGEXConverter> {
public:

	OpenGEXConverter() { }
	~OpenGEXConverter() { Cleanup(); }

	void ConvertScene(const OpenGexDataDescription& desc, aiScene* out) {
		ai_assert(desc.GetRootStructure() && "Invalid OpenGEX data description");
		out->mRootNode = new aiNode();
		out->mRootNode->mName.Set("RootNode");
		ConvertObjects(*desc.GetRootStructure());
		ConvertNodes(*desc.GetRootStructure(), *out->mRootNode);
		TransferObjectsToScene(out);
		if (out->mNumMeshes == 0) {
			out->mFlags |= AI_SCENE_FLAGS_INCOMPLETE;
		}
	}

private:
	static bool IsObject(const Structure& structure) {
		const StructureType type = structure.GetStructureType();
		return type == kStructureLightObject ||
		       type == kStructureGeometryObject ||
		       type == kStructureCameraObject ||
		       type == kStructureMaterial;
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

	static bool IsNode(const Structure& structure) {
		const StructureType type = structure.GetStructureType();
		return type == kStructureNode ||
		       type == kStructureBoneNode ||
		       type == kStructureLightNode ||
		       type == kStructureCameraNode ||
		       type == kStructureGeometryNode;
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

	aiNode* ConvertNode(const NodeStructure& structure) {
		auto_ptr<aiNode> node(new aiNode);
		node->mName.Set(structure.GetNodeName());

		TransformVisitor transformVisitor(&node->mTransformation);
		VisitSubnodes(structure, transformVisitor);

		return node.release();
	}

	aiNode* ConvertBoneNode(const BoneNodeStructure& structure) {
		auto_ptr<aiNode> node(ConvertNode(structure));
		// TODO: Implement.
		return node.release();
	}

	aiNode* ConvertGeometryNode(const GeometryNodeStructure& structure) {
		auto_ptr<aiNode> node(ConvertNode(structure));

		const GeometryObjectStructure* geometry = structure.GetGeometryObjectStructure();
		ai_assert(geometry && "Geometry node must contain exactly 1 geometry object");

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
		return node.release();
	}

	aiNode* ConvertLightNode(const LightNodeStructure& structure) {
		StructureMapConstIt lightIt = lightMap.find(structure.GetObjectStructure());
		if (lightIt == lightMap.end()) ThrowException("Invalid light reference");

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
		StructureMapConstIt cameraIt = cameraMap.find(structure.GetObjectStructure());
		if (cameraIt == cameraMap.end()) ThrowException("Invalid camera reference");

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

	unsigned int FindOrInsertMeshWithMaterial(const GeometryObjectStructure* geometry, const MaterialStructure* material) {
		typedef MeshMaterialMap::key_type MeshMaterialKey;
		typedef MeshMaterialMap::const_iterator MeshMaterialConstIt;

		if (!material) {
			materialMap[nullptr] = static_cast<unsigned int>(materials.size());
			materials.push_back(CreateDefaultMaterial());
		}

		const MeshMaterialKey meshMaterialKey = std::make_pair(geometry, material);
		const MeshMaterialConstIt meshMaterialIt = meshMaterialMap.find(meshMaterialKey);
		if (meshMaterialIt != meshMaterialMap.end()) {
			return meshMaterialIt->second;
		}

		// First look at the raw meshes container. If it contains a mesh for the
		// provided geometry, "consume" that mesh. Otherwise, clone the already
		// consumed mesh; it's already been used for a different material.
		aiMesh* mesh = NULL;
		StructureMapIt rawMeshIt = rawMeshMap.find(geometry);
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

	// Move all generated meshes, animations, lights, cameras and textures to the output scene.
	void TransferObjectsToScene(aiScene* out) {
		ai_assert(!out->mMeshes && !out->mNumMeshes);
		SafeAllocateAndMove(meshes, &out->mMeshes, &out->mNumMeshes);
		SafeAllocateAndMove(materials, &out->mMaterials, &out->mNumMaterials);
		SafeAllocateAndMove(animations, &out->mAnimations, &out->mNumAnimations);
		SafeAllocateAndMove(lights, &out->mLights, &out->mNumLights);
		SafeAllocateAndMove(cameras, &out->mCameras, &out->mNumCameras);
	}

	void Cleanup() {
		SafeDelete(rawMeshes);
		// Pointer containers below should all be empty if import is successful.
		SafeDelete(meshes);
		SafeDelete(materials);
		SafeDelete(animations);
		SafeDelete(lights);
		SafeDelete(cameras);
	}

private:

	typedef std::map<const Structure*, unsigned int> StructureMap;
	typedef StructureMap::iterator StructureMapIt;
	typedef StructureMap::const_iterator StructureMapConstIt;

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
		ThrowException(format() << "Failed to load OpenGEX file: " << DataResultToString(result));
	}

	OpenGEXConverter().ConvertScene(openGexDataDescription, pScene);
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
