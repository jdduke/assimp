#include "UnitTestPCH.h"

#include <assimp/mesh.h>

using namespace std;
using namespace Assimp;


TEST(FaceTest, testInitialize)
{
	aiFace face;
	EXPECT_EQ(0U, face.mNumIndices);
	EXPECT_TRUE(!face.mIndices);

	// Faces with <=4 indices should use stack allocation.
	face.Initialize(1);
	EXPECT_EQ(1U, face.mNumIndices);
	EXPECT_TRUE(face.mIndicesStorage == face.mIndices);

	face.Initialize(2);
	EXPECT_EQ(2U, face.mNumIndices);
	EXPECT_TRUE(face.mIndicesStorage == face.mIndices);

	face.InitializeFace3(1, 2, 3);
	EXPECT_TRUE(face.mIndicesStorage == face.mIndices);
	EXPECT_EQ(3U, face.mNumIndices);
	EXPECT_EQ(1U, face.mIndices[0]);
	EXPECT_EQ(2U, face.mIndices[1]);
	EXPECT_EQ(3U, face.mIndices[2]);

	face.InitializeFace4(4, 5, 6, 7);
	EXPECT_TRUE(face.mIndicesStorage == face.mIndices);
	EXPECT_EQ(4U, face.mNumIndices);
	EXPECT_EQ(4U, face.mIndices[0]);
	EXPECT_EQ(5U, face.mIndices[1]);
	EXPECT_EQ(6U, face.mIndices[2]);
	EXPECT_EQ(7U, face.mIndices[3]);

	// Faces with >4 indices should use heap allocation.
	unsigned int indices[5] = {8, 9, 10, 11, 12};
	face.Initialize(5, indices);
	EXPECT_TRUE(face.mIndicesStorage != face.mIndices);
	EXPECT_EQ(5U, face.mNumIndices);
	EXPECT_EQ(8U, face.mIndices[0]);
	EXPECT_EQ(9U, face.mIndices[1]);
	EXPECT_EQ(10U, face.mIndices[2]);
	EXPECT_EQ(11U, face.mIndices[3]);
	EXPECT_EQ(12U, face.mIndices[4]);

	face.Initialize(500);
	EXPECT_EQ(500U, face.mNumIndices);
	EXPECT_TRUE(face.mIndicesStorage != face.mIndices);
}

TEST(FaceTest, testEquality)
{
	aiFace face0, face1;
	EXPECT_TRUE(face0 == face0);
	EXPECT_TRUE(face0 == face1);

	face0.InitializeFace3(1, 2, 3);
	EXPECT_TRUE(face0 == face0);
	EXPECT_TRUE(face0 != face1);

	face1.InitializeFace3(1, 2, 3);
	EXPECT_TRUE(face0 == face0);
	EXPECT_TRUE(face0 == face1);

	face0.InitializeFace4(1, 2, 3, 4);
	EXPECT_TRUE(face0 == face0);
	EXPECT_TRUE(face0 != face1);

	unsigned int indices[5] = {2, 3, 4, 5, 6};
	face0.Initialize(5, indices);
	EXPECT_TRUE(face0 == face0);
	EXPECT_TRUE(face0 != face1);
	face1.Initialize(5, indices);
	EXPECT_TRUE(face0 == face1);
}

TEST(FaceTest, testAssign)
{
	aiFace face0, face1;
	face0.InitializeFace3(1, 2, 3);
	EXPECT_TRUE(face0 != face1);
	face1 = face0;
	EXPECT_TRUE(face0 == face1);
	EXPECT_EQ(3U, face1.mNumIndices);
	EXPECT_EQ(1U, face1.mIndices[0]);
	EXPECT_EQ(2U, face1.mIndices[1]);
	EXPECT_EQ(3U, face1.mIndices[2]);

	face0.Initialize(5);
	face0.mIndices[0] = 1;
	face0.mIndices[4] = 2;
	EXPECT_EQ(5U, face0.mNumIndices);
	EXPECT_TRUE(face0 != face1);

	face1 = face0;
	EXPECT_TRUE(face0 == face1);
	EXPECT_EQ(5U, face1.mNumIndices);
	EXPECT_EQ(1U, face1.mIndices[0]);
	EXPECT_EQ(2U, face1.mIndices[4]);
}

TEST(FaceTest, testCopy)
{
	aiFace face0;
	face0.InitializeFace3(1, 2, 3);

	aiFace face1(face0);
	EXPECT_TRUE(face0 == face1);
	EXPECT_EQ(3U, face1.mNumIndices);
	EXPECT_EQ(1U, face1.mIndices[0]);
	EXPECT_EQ(2U, face1.mIndices[1]);
	EXPECT_EQ(3U, face1.mIndices[2]);

	face0.Initialize(5);
	face0.mIndices[0] = 1;
	face0.mIndices[4] = 2;

	aiFace face2(face0);
	EXPECT_TRUE(face0 == face2);
	EXPECT_EQ(5U, face2.mNumIndices);
	EXPECT_EQ(1U, face2.mIndices[0]);
	EXPECT_EQ(2U, face2.mIndices[4]);
}

TEST(FaceTest, testSwap)
{
	aiFace face0;
	face0.InitializeFace3(1, 2, 3);
	aiFace face0Copy(face0);

	aiFace face1;
	face1.Initialize(5);
	face1.mIndices[0] = 1;
	face1.mIndices[4] = 2;
	aiFace face1Copy(face1);

	// Swap with one heap allocated, one stack allocated.
	face0.Swap(face1);
	EXPECT_TRUE(face0 == face1Copy);
	EXPECT_TRUE(face1 == face0Copy);
	face0.Swap(face1);
	EXPECT_TRUE(face0 == face0Copy);
	EXPECT_TRUE(face1 == face1Copy);

	// Swap with both stack allocated.
	face0.InitializeFace3(1, 2, 3);
	face1.InitializeFace4(2, 3, 4, 5);
	face0Copy = face0;
	face1Copy = face1;

	face0.Swap(face1);
	EXPECT_TRUE(face0 == face1Copy);
	EXPECT_TRUE(face1 == face0Copy);
	face0.Swap(face1);
	EXPECT_TRUE(face0 == face0Copy);
	EXPECT_TRUE(face1 == face1Copy);

	// Swap with both heap allocated.
	unsigned int indices0[5] = {2, 3, 4, 5, 6};
	unsigned int indices1[6] = {3, 4, 5, 6, 7, 8};
	face0.Initialize(5, indices0);
	face1.Initialize(6, indices1);
	face0Copy = face0;
	face1Copy = face1;

	face0.Swap(face1);
	EXPECT_TRUE(face0 == face1Copy);
	EXPECT_TRUE(face1 == face0Copy);
	face0.Swap(face1);
	EXPECT_TRUE(face0 == face0Copy);
	EXPECT_TRUE(face1 == face1Copy);
}

TEST(FaceTest, testDestroy)
{
	aiFace face;
	face.Initialize(10);
	EXPECT_TRUE(face.mIndices != NULL);
	face.~aiFace();
	EXPECT_TRUE(face.mIndices == NULL);

	face.Initialize(10);
	EXPECT_TRUE(face.mIndices != NULL);
	face = aiFace();
	EXPECT_TRUE(face.mIndices == NULL);
}
