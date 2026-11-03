// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntCurveSurface_IntersectionPoint_Test.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntCurveSurface_InterUtils_Test.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntCurveSurface_ThePolygonOfHInter_Test.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntCurveSurface_ThePolyhedronOfHInter_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.CurveSurface.TransitionOnCurve;
using static Kerf.Engine.Intersection.Polygons.PIType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's IntCurveSurface_IntersectionPoint GTest.</summary>
public class IntCurveSurface_IntersectionPointTest
{
    // Regression test for bug #63: default constructor must initialize transition enum.
    [Fact]
    public void DefaultConstructor_TransitionInitialized()
    {
        CurveSurfaceIntersectionPoint aPt = new();

        // Transition must be deterministic (initialized to IntCurveSurface_Tangent).
        Assert.Equal(TransitionOnCurve.Tangent, aPt.Transition());

        // Scalar fields must be zero-initialized.
        Assert.Equal(0.0, aPt.U());
        Assert.Equal(0.0, aPt.V());
        Assert.Equal(0.0, aPt.W());
    }

    // Value constructor initializes all fields correctly.
    [Fact]
    public void ValueConstructor_AllFieldsSet()
    {
        Point3D aP = new(1.0, 2.0, 3.0);
        CurveSurfaceIntersectionPoint aPt = new(aP, 0.5, 0.6, 0.7, TransitionOnCurve.In);

        Assert.Equal(1.0, aPt.Pnt().X(), 1e-15);
        Assert.Equal(2.0, aPt.Pnt().Y(), 1e-15);
        Assert.Equal(3.0, aPt.Pnt().Z(), 1e-15);
        Assert.Equal(0.5, aPt.U());
        Assert.Equal(0.6, aPt.V());
        Assert.Equal(0.7, aPt.W());
        Assert.Equal(TransitionOnCurve.In, aPt.Transition());
    }

    // SetValues overwrites all fields including transition.
    [Fact]
    public void SetValues_OverwritesTransition()
    {
        CurveSurfaceIntersectionPoint aPt = new();
        Assert.Equal(TransitionOnCurve.Tangent, aPt.Transition());

        Point3D aP = new(5.0, 6.0, 7.0);
        aPt.SetValues(aP, 1.0, 2.0, 3.0, TransitionOnCurve.Out);
        Assert.Equal(TransitionOnCurve.Out, aPt.Transition());
    }
}

/// <summary>OCCT's IntCurveSurface_InterUtils GTest.</summary>
public class IntCurveSurface_InterUtilsTest
{
    private sealed class TestPolyhedron : IPolyhedron
    {
        private readonly Point3D[] myPoints = [new(0.0, 0.0, 0.0), new(2.0, 0.0, 0.0), new(2.0, 0.0, 0.0)];
        private readonly double[] myU = [0.0, 1.0, 2.0];
        private readonly double[] myV = [0.0, 0.0, 0.0];

        public void Triangle(int Index, out int theP1, out int theP2, out int theP3)
        {
            theP1 = 1;
            theP2 = 2;
            theP3 = 3;
        }

        public Point3D Point(int theIndex) => myPoints[theIndex - 1];

        public void Parameters(int theIndex, out double theU, out double theV)
        {
            theU = myU[theIndex - 1];
            theV = myV[theIndex - 1];
        }

        // Not used by SectionPointToParameters
        public Box Bounding() => throw new NotSupportedException();

        public Point3D Point(int Index, out double U, out double V) => throw new NotSupportedException();
    }

    private sealed class TestPolygon : IPolygon
    {
        public double ApproxParamOnCurve(int theIndex, double theParamOnLine) => 10.0 * theIndex + theParamOnLine;

        // Not used by SectionPointToParameters
        public double InfParameter() => throw new NotSupportedException();

        public double SupParameter() => throw new NotSupportedException();
    }

    [Fact]
    public void SectionPointToParameters_DegenerateFaceFallsBackToLongestEdge()
    {
        TestPolyhedron aPolyhedron = new();
        TestPolygon aPolygon = new();

        SectionPoint aSectionPoint = new(new Point3D(0.5, 0.0, 0.0), PIType.Edge, 0, 2, 0.4, PIType.Face, 1, 0, 0.0, 0.0);

        InterUtils.SectionPointToParameters(aSectionPoint, aPolyhedron, aPolygon,
                                                                 out double aU, out double aV, out double aW);

        Assert.Equal(0.25, aU, 1.0e-12);
        Assert.Equal(0.0, aV, 1.0e-12);
        Assert.Equal(20.4, aW, 1.0e-12);
    }
}

/// <summary>OCCT's IntCurveSurface_ThePolygonOfHInter GTest.</summary>
public class IntCurveSurface_ThePolygonOfHInter_Test
{
    [Fact]
    public void ClosedFlagReflectsStoredState()
    {
        LineCurve3D aLine = new(new Axis3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0)));
        CurveAdaptor anAdaptor = new(aLine);
        ThePolygonOfHInter aPolygon = new(anAdaptor, 6);

        Assert.False(aPolygon.Closed());
        aPolygon.Closed(true);
        Assert.True(aPolygon.Closed());
    }
}

/// <summary>OCCT's IntCurveSurface_ThePolyhedronOfHInter GTest.</summary>
public class IntCurveSurface_ThePolyhedronOfHInterTest
{
    private static SurfaceAdaptor makeTestPlane()
    {
        CoordinateSystem anAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        PlaneSurface aPlane = new(anAxis);
        return new SurfaceAdaptor(aPlane, -1.0, 1.0, -1.0, 1.0);
    }

    // Regression test for bug #55: singularity flags must be initialized to false.
    [Fact]
    public void SingularityFlags_InitializedFalse()
    {
        SurfaceAdaptor aSurf = makeTestPlane();
        ThePolyhedronOfHInter aPoly = new(aSurf, 3, 3, -1.0, -1.0, 1.0, 1.0);

        Assert.False(aPoly.HasUMinSingularity);
        Assert.False(aPoly.HasUMaxSingularity);
        Assert.False(aPoly.HasVMinSingularity);
        Assert.False(aPoly.HasVMaxSingularity);
    }

    // Singularity setters should update the flags.
    [Fact]
    public void SingularitySetters_UpdateFlags()
    {
        SurfaceAdaptor aSurf = makeTestPlane();
        ThePolyhedronOfHInter aPoly = new(aSurf, 3, 3, -1.0, -1.0, 1.0, 1.0);

        aPoly.UMinSingularity(true);
        Assert.True(aPoly.HasUMinSingularity);
        Assert.False(aPoly.HasUMaxSingularity);

        aPoly.VMaxSingularity(true);
        Assert.True(aPoly.HasVMaxSingularity);
        Assert.False(aPoly.HasVMinSingularity);
    }

    // Basic construction produces valid polyhedron.
    [Fact]
    public void BasicConstruction_ValidMesh()
    {
        SurfaceAdaptor aSurf = makeTestPlane();
        ThePolyhedronOfHInter aPoly = new(aSurf, 4, 4, -1.0, -1.0, 1.0, 1.0);

        aPoly.Size(out int nbU, out int nbV);
        Assert.Equal(4, nbU);
        Assert.Equal(4, nbV);
        Assert.Equal(4 * 4 * 2, aPoly.TriangleCount);
        Assert.Equal((4 + 1) * (4 + 1), aPoly.PointCount);
    }

    // Regression test for bug #58: parameter-array constructor with minimum-size arrays
    // must not cause division by zero (arrays of length 2 give nbdelta = 1).
    [Fact]
    public void ParamArrayConstructor_MinimumSize()
    {
        SurfaceAdaptor aSurf = makeTestPlane();

        NCollection_Array1<double> aUpars = new(1, 2);
        aUpars[1] = -1.0;
        aUpars[2] = 1.0;

        NCollection_Array1<double> aVpars = new(1, 2);
        aVpars[1] = -1.0;
        aVpars[2] = 1.0;

        ThePolyhedronOfHInter aPoly = new(aSurf, aUpars, aVpars);

        aPoly.Size(out int nbU, out int nbV);
        Assert.True(nbU >= 1);
        Assert.True(nbV >= 1);
        Assert.True(aPoly.TriangleCount > 0);
    }

    // Regression test for constructor validation added for invalid explicit parameter arrays.
    [Fact]
    public void ParamArrayConstructor_RejectsSingleValueArrays()
    {
        SurfaceAdaptor aSurf = makeTestPlane();

        NCollection_Array1<double> aUpars = new(1, 1);
        aUpars[1] = 0.0;

        NCollection_Array1<double> aVpars = new(1, 2);
        aVpars[1] = -1.0;
        aVpars[2] = 1.0;

        Assert.ThrowsAny<OutOfRangeException>(() => new ThePolyhedronOfHInter(aSurf, aUpars, aVpars));

        NCollection_Array1<double> aValidUpars = new(1, 2);
        aValidUpars[1] = -1.0;
        aValidUpars[2] = 1.0;

        NCollection_Array1<double> aSingleVpars = new(1, 1);
        aSingleVpars[1] = 0.0;

        Assert.ThrowsAny<OutOfRangeException>(() => new ThePolyhedronOfHInter(aSurf, aValidUpars, aSingleVpars));
    }

    // PlaneEquation for a valid triangle must produce finite results.
    [Fact]
    public void PlaneEquation_FiniteResults()
    {
        SurfaceAdaptor aSurf = makeTestPlane();
        ThePolyhedronOfHInter aPoly = new(aSurf, 3, 3, -1.0, -1.0, 1.0, 1.0);

        aPoly.PlaneEquation(1, out Xyz aNormal, out double aPolarDist);

        Assert.True(double.IsFinite(aNormal.X()));
        Assert.True(double.IsFinite(aNormal.Y()));
        Assert.True(double.IsFinite(aNormal.Z()));
        Assert.True(double.IsFinite(aPolarDist));
    }
}
