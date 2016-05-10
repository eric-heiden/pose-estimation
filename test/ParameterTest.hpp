#pragma once

#include "../parameter.h"

#include <cppunit/extensions/HelperMacros.h>

using namespace PoseEstimation;

class ParameterTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(ParameterTest);
    CPPUNIT_TEST(testConstructor);
    CPPUNIT_TEST_SUITE_END();

public:
    void setUp()
    {}

    void tearDown()
    {}

    void testConstructor()
    {
        Parameter p1("category", "name", (int)1, "description");

        CPPUNIT_ASSERT(p1.isNumber());

        CPPUNIT_ASSERT_EQUAL(Parameter::getOrDefault("category_name", 0), 1);

        p1.constraints().push_back(std::make_shared<ConstantConstraint>(ParameterConstraintType::GreaterThan, 1));
        CPPUNIT_ASSERT(!p1.isValid());

        p1.constraints().clear();
        CPPUNIT_ASSERT(p1.isValid());
    }
};

CPPUNIT_TEST_SUITE_REGISTRATION(ParameterTest);
