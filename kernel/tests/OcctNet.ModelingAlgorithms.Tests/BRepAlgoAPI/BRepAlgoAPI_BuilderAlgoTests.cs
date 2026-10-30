// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;

namespace OcctNet.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// OCCT's TKBO GTests BRepAlgoAPI_BuilderAlgo_Test.cxx and BRepAlgoAPI_Common_Test.cxx.
/// </summary>
/// <remarks>
/// BRepAlgoAPI_BuilderAlgo_Test checks that the algorithms aren't copyable or movable (std::is_copy_constructible and
/// the like), because OCCT's hold raw pointers to their tools. The port's algorithms are classes, which C# never copies;
/// the tests check that each is a reference type with no constructor taking an instance of itself (a copy constructor).
/// </remarks>
public static class BRepAlgoAPI_BuilderAlgoTests
{
    private static bool IsCopyConstructible(Type theType) =>
        theType.IsValueType
        || theType.GetConstructors(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance)
                  .Any(c => c.GetParameters() is [ParameterInfo p] && p.ParameterType.IsAssignableFrom(theType));

    // C# has no assignment operator: a reference type can't be assigned by value.
    private static bool IsCopyAssignable(Type theType) => theType.IsValueType;

    /// <summary>BRepAlgoAPI_BuilderAlgo Non-Copyability Tests.</summary>
    public class BRepAlgoAPI_BuilderAlgoTest
    {
        [Fact]
        public void NotCopyConstructible() => Assert.False(IsCopyConstructible(typeof(BRepAlgoAPI_BuilderAlgo)));

        [Fact]
        public void NotCopyAssignable() => Assert.False(IsCopyAssignable(typeof(BRepAlgoAPI_BuilderAlgo)));

        [Fact]
        public void NotMoveConstructible() => Assert.False(IsCopyConstructible(typeof(BRepAlgoAPI_BuilderAlgo)));

        [Fact]
        public void NotMoveAssignable() => Assert.False(IsCopyAssignable(typeof(BRepAlgoAPI_BuilderAlgo)));
    }

    public class BRepAlgoAPI_BooleanOperationTest
    {
        [Fact]
        public void NotCopyConstructible() => Assert.False(IsCopyConstructible(typeof(BRepAlgoAPI_BooleanOperation)));
    }

    public class BRepAlgoAPI_CutTest
    {
        [Fact]
        public void NotCopyConstructible() => Assert.False(IsCopyConstructible(typeof(BRepAlgoAPI_Cut)));
    }

    public class BRepAlgoAPI_FuseTest
    {
        [Fact]
        public void NotCopyConstructible() => Assert.False(IsCopyConstructible(typeof(BRepAlgoAPI_Fuse)));
    }

    public class BRepAlgoAPI_CommonTest
    {
        [Fact]
        public void NotCopyConstructible() => Assert.False(IsCopyConstructible(typeof(BRepAlgoAPI_Common)));
    }

    public class BRepAlgoAPI_SectionTest
    {
        [Fact]
        public void NotCopyConstructible() => Assert.False(IsCopyConstructible(typeof(BRepAlgoAPI_Section)));
    }

    public class BRepAlgoAPI_SplitterTest
    {
        [Fact]
        public void NotCopyConstructible() => Assert.False(IsCopyConstructible(typeof(BRepAlgoAPI_Splitter)));
    }

    /// <summary>BOPCommon Simple Tests - migrating from /tests/boolean/bopcommon_simple/ (BRepAlgoAPI_Common_Test.cxx).</summary>
    public class BOPCommonSimpleTest : BRepAlgoAPI_TestBase
    {
        // Test bopcommon_simple/A1: box b1 0 0 0 1 1 1; box b2 0 0 0 1 1 1; bop b1 b2; bopcommon result;
        // checkprops result -s 6
        [Fact]
        public void IdenticalBoxes_A1()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new gp_Pnt(0, 0, 0), 1.0, 1.0, 1.0);
            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new gp_Pnt(0, 0, 0), 1.0, 1.0, 1.0);

            TopoDS_Shape aResult = PerformCommon(aBox1, aBox2);
            ValidateResult(aResult, 6.0);
        }
    }
}
