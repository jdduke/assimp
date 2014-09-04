#include "UnitTestPCH.h"
#include "utFace.h"

CPPUNIT_TEST_SUITE_REGISTRATION (FaceTest);

void FaceTest::testInitialize(void)
{
	aiFace face;
	CPPUNIT_ASSERT_EQUAL(0U, face.mNumIndices);
	CPPUNIT_ASSERT(!face.mIndices);

	// Faces with <=4 indices should use stack allocation.
	face.Initialize(1);
	CPPUNIT_ASSERT_EQUAL(1U, face.mNumIndices);
	CPPUNIT_ASSERT(face.mIndicesStorage == face.mIndices);

	face.Initialize(2);
	CPPUNIT_ASSERT_EQUAL(2U, face.mNumIndices);
	CPPUNIT_ASSERT(face.mIndicesStorage == face.mIndices);

	face.InitializeFace3(1, 2, 3);
	CPPUNIT_ASSERT(face.mIndicesStorage == face.mIndices);
	CPPUNIT_ASSERT_EQUAL(3U, face.mNumIndices);
	CPPUNIT_ASSERT_EQUAL(1U, face.mIndices[0]);
	CPPUNIT_ASSERT_EQUAL(2U, face.mIndices[1]);
	CPPUNIT_ASSERT_EQUAL(3U, face.mIndices[2]);

	face.InitializeFace4(4, 5, 6, 7);
	CPPUNIT_ASSERT(face.mIndicesStorage == face.mIndices);
	CPPUNIT_ASSERT_EQUAL(4U, face.mNumIndices);
	CPPUNIT_ASSERT_EQUAL(4U, face.mIndices[0]);
	CPPUNIT_ASSERT_EQUAL(5U, face.mIndices[1]);
	CPPUNIT_ASSERT_EQUAL(6U, face.mIndices[2]);
	CPPUNIT_ASSERT_EQUAL(7U, face.mIndices[3]);

	// Faces with >4 indices should use heap allocation.
	unsigned int indices[5] = {8, 9, 10, 11, 12};
	face.Initialize(5, indices);
	CPPUNIT_ASSERT(face.mIndicesStorage != face.mIndices);
	CPPUNIT_ASSERT_EQUAL(5U, face.mNumIndices);
	CPPUNIT_ASSERT_EQUAL(8U, face.mIndices[0]);
	CPPUNIT_ASSERT_EQUAL(9U, face.mIndices[1]);
	CPPUNIT_ASSERT_EQUAL(10U, face.mIndices[2]);
	CPPUNIT_ASSERT_EQUAL(11U, face.mIndices[3]);
	CPPUNIT_ASSERT_EQUAL(12U, face.mIndices[4]);

	face.Initialize(500);
	CPPUNIT_ASSERT_EQUAL(500U, face.mNumIndices);
	CPPUNIT_ASSERT(face.mIndicesStorage != face.mIndices);
}

void FaceTest::testEquality(void)
{
	aiFace face0, face1;
	CPPUNIT_ASSERT(face0 == face0);
	CPPUNIT_ASSERT(face0 == face1);

	face0.InitializeFace3(1, 2, 3);
	CPPUNIT_ASSERT(face0 == face0);
	CPPUNIT_ASSERT(face0 != face1);

	face1.InitializeFace3(1, 2, 3);
	CPPUNIT_ASSERT(face0 == face0);
	CPPUNIT_ASSERT(face0 == face1);

	face0.InitializeFace4(1, 2, 3, 4);
	CPPUNIT_ASSERT(face0 == face0);
	CPPUNIT_ASSERT(face0 != face1);

	unsigned int indices[5] = {2, 3, 4, 5, 6};
	face0.Initialize(5, indices);
	CPPUNIT_ASSERT(face0 == face0);
	CPPUNIT_ASSERT(face0 != face1);
	face1.Initialize(5, indices);
	CPPUNIT_ASSERT(face0 == face1);
}

void FaceTest::testAssign(void)
{
	aiFace face0, face1;
	face0.InitializeFace3(1, 2, 3);
	CPPUNIT_ASSERT(face0 != face1);
	face1 = face0;
	CPPUNIT_ASSERT(face0 == face1);
	CPPUNIT_ASSERT_EQUAL(3U, face1.mNumIndices);
	CPPUNIT_ASSERT_EQUAL(1U, face1.mIndices[0]);
	CPPUNIT_ASSERT_EQUAL(2U, face1.mIndices[1]);
	CPPUNIT_ASSERT_EQUAL(3U, face1.mIndices[2]);

	face0.Initialize(5);
	face0.mIndices[0] = 1;
	face0.mIndices[4] = 2;
	CPPUNIT_ASSERT_EQUAL(5U, face0.mNumIndices);
	CPPUNIT_ASSERT(face0 != face1);

	face1 = face0;
	CPPUNIT_ASSERT(face0 == face1);
	CPPUNIT_ASSERT_EQUAL(5U, face1.mNumIndices);
	CPPUNIT_ASSERT_EQUAL(1U, face1.mIndices[0]);
	CPPUNIT_ASSERT_EQUAL(2U, face1.mIndices[4]);
}

void FaceTest::testCopy(void)
{
	aiFace face0;
	face0.InitializeFace3(1, 2, 3);

	aiFace face1(face0);
	CPPUNIT_ASSERT(face0 == face1);
	CPPUNIT_ASSERT_EQUAL(3U, face1.mNumIndices);
	CPPUNIT_ASSERT_EQUAL(1U, face1.mIndices[0]);
	CPPUNIT_ASSERT_EQUAL(2U, face1.mIndices[1]);
	CPPUNIT_ASSERT_EQUAL(3U, face1.mIndices[2]);

	face0.Initialize(5);
	face0.mIndices[0] = 1;
	face0.mIndices[4] = 2;

	aiFace face2(face0);
	CPPUNIT_ASSERT(face0 == face2);
	CPPUNIT_ASSERT_EQUAL(5U, face2.mNumIndices);
	CPPUNIT_ASSERT_EQUAL(1U, face2.mIndices[0]);
	CPPUNIT_ASSERT_EQUAL(2U, face2.mIndices[4]);
}

void FaceTest::testSwap(void)
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
	CPPUNIT_ASSERT(face0 == face1Copy);
	CPPUNIT_ASSERT(face1 == face0Copy);
	face0.Swap(face1);
	CPPUNIT_ASSERT(face0 == face0Copy);
	CPPUNIT_ASSERT(face1 == face1Copy);

	// Swap with both stack allocated.
	face0.InitializeFace3(1, 2, 3);
	face1.InitializeFace4(2, 3, 4, 5);
	face0Copy = face0;
	face1Copy = face1;

	face0.Swap(face1);
	CPPUNIT_ASSERT(face0 == face1Copy);
	CPPUNIT_ASSERT(face1 == face0Copy);
	face0.Swap(face1);
	CPPUNIT_ASSERT(face0 == face0Copy);
	CPPUNIT_ASSERT(face1 == face1Copy);

	// Swap with both heap allocated.
	unsigned int indices0[5] = {2, 3, 4, 5, 6};
	unsigned int indices1[6] = {3, 4, 5, 6, 7, 8};
	face0.Initialize(5, indices0);
	face1.Initialize(6, indices1);
	face0Copy = face0;
	face1Copy = face1;

	face0.Swap(face1);
	CPPUNIT_ASSERT(face0 == face1Copy);
	CPPUNIT_ASSERT(face1 == face0Copy);
	face0.Swap(face1);
	CPPUNIT_ASSERT(face0 == face0Copy);
	CPPUNIT_ASSERT(face1 == face1Copy);
}

void FaceTest::testDestroy(void)
{
	aiFace face;
	face.Initialize(10);
	CPPUNIT_ASSERT(face.mIndices != NULL);
	face.~aiFace();
	CPPUNIT_ASSERT(face.mIndices == NULL);

	face.Initialize(10);
	CPPUNIT_ASSERT(face.mIndices != NULL);
	face = aiFace();
	CPPUNIT_ASSERT(face.mIndices == NULL);
}
