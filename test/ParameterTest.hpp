#pragma once

#include "../parameter.h"

#include <cppunit/extensions/HelperMacros.h>

using namespace PoseEstimation;

class ParameterTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(ParameterTest);
    CPPUNIT_TEST(EnumTest);
    CPPUNIT_TEST(EnumParameterTest);
    CPPUNIT_TEST(SimpleParameterTest);
    CPPUNIT_TEST_SUITE_END();

public:
    void setUp()
    {}

    void tearDown()
    {}

    void EnumTest()
    {
        Enum e = Enum::define({"a", "b", "c"});
        CPPUNIT_ASSERT_EQUAL((size_t)3, e.size());
        CPPUNIT_ASSERT_EQUAL((std::string)"a", e.valueName());
        std::string b;
        CPPUNIT_ASSERT(e.get(1, b));
        CPPUNIT_ASSERT_EQUAL((std::string)"b", b);
        int idx;
        CPPUNIT_ASSERT(e.get(b, idx));
        CPPUNIT_ASSERT_EQUAL(1, idx);
    }

    void EnumParameterTest()
    {
        EnumParameter ep("category", "enumparameter", {"a", "b", "c"});
        CPPUNIT_ASSERT(ep.setValue("b"));
        std::cout << "Enum Parameter Initialized" << std::endl;
        Parameter::displayAll();
    }

    void SimpleParameterTest()
    {
        Parameter p1("category", "name", (int)1, "description");

        CPPUNIT_ASSERT(p1.isNumber());

        CPPUNIT_ASSERT_EQUAL(1, Parameter::getOrDefault("category_name", 0));

        p1.constraints().push_back(std::make_shared<ConstantConstraint>(
                                       ParameterConstraintType::GreaterThan, 1));
        CPPUNIT_ASSERT(!p1.isValid());

        p1.constraints().clear();
        CPPUNIT_ASSERT(p1.isValid());        
    }
};

CPPUNIT_TEST_SUITE_REGISTRATION(ParameterTest);
