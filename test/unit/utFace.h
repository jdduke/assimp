#ifndef TESTFACE_H
#define TESTFACE_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

class FaceTest : public CPPUNIT_NS :: TestFixture
{
	CPPUNIT_TEST_SUITE (FaceTest);
	CPPUNIT_TEST (testInitialize);
	CPPUNIT_TEST (testEquality);
	CPPUNIT_TEST (testAssign);
	CPPUNIT_TEST (testCopy);
	CPPUNIT_TEST (testSwap);
	CPPUNIT_TEST (testDestroy);
	CPPUNIT_TEST_SUITE_END ();

	void testInitialize(void);
	void testEquality(void);
	void testAssign(void);
	void testCopy(void);
	void testSwap(void);
	void testDestroy(void);
};

#endif
