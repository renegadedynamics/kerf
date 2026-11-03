// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Kernel.ModelingAlgorithms.Tests.LocationLawTestSupport;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>GeomFill_UniformSection and GeomFill_SectionPlacement.</summary>
public class GeomFill_SectionsTests
{
    [Fact]
    public void UniformSection_OfACircle()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 2);
        var aLaw = new UniformSection(aCircle, 0, 10);
        aLaw.SectionShape(out int aNbPoles, out int aNbKnots, out int aDeg);
        BSplineCurve3D aBS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aCircle, ParameterisationType.QuasiAngular);
        Assert.Equal(aBS.IsPeriodic, aLaw.IsUPeriodic);
        Assert.True(aLaw.IsRational);
        Assert.True(aLaw.IsVPeriodic);
        Assert.Equal(aDeg, aBS.Degree);

        var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
        var aWeights = new NCollection_Array1<double>(1, aNbPoles);
        Assert.True(aLaw.D0(3.3, aPoles, aWeights));
        var aKnots = new NCollection_Array1<double>(1, aNbKnots);
        var aMults = new NCollection_Array1<int>(1, aNbKnots);
        aLaw.Knots(aKnots);
        aLaw.Mults(aMults);
        var aRebuilt = new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, aDeg, aLaw.IsUPeriodic);
        for (double u = 0; u < 2 * M_PI; u += 0.4)
        {
            Assert.True(aRebuilt.Value(u).DistanceTo(aBS.Value(u)) < 1e-12);
            Assert.Equal(2.0, aRebuilt.Value(u).DistanceTo(new Point3D(0, 0, 0)), 1e-12);
        }

        var aDPoles = new NCollection_Array1<Vector3D>(1, aNbPoles);
        var aDWeights = new NCollection_Array1<double>(1, aNbPoles);
        aDWeights.Init(7);
        Assert.True(aLaw.D1(3.3, aPoles, aDPoles, aWeights, aDWeights));
        Assert.Equal(new Vector3D(0, 0, 0), aDPoles[2]);
        Assert.Equal(0.0, aDWeights[1]);

        double anError = -1;
        Assert.True(aLaw.IsConstant(ref anError));
        Assert.Equal(0.0, anError);
        Curve3D aConst = aLaw.ConstantSection();
        Assert.NotSame(aCircle, aConst);
        Assert.False(aConst.IsFrozen);
        Assert.NotSame(aConst, aLaw.ConstantSection());
        Assert.Equal(4 * M_PI, aLaw.MaximalSection(), 1e-9);

        var T = new NCollection_Array1<double>(1, 2);
        Assert.Equal(1, aLaw.NbIntervals(Continuity.CN));
        aLaw.Intervals(T, Continuity.CN);
        Assert.Equal((0.0, 10.0), (T[1], T[2]));
        aLaw.GetDomain(out double f, out double l);
        Assert.Equal((0.0, 10.0), (f, l));

        var aTol = new NCollection_Array1<double>(1, aNbPoles);
        aLaw.GetTolerance(1e-5, 1e-3, 0.1, aTol);
        Assert.Equal(1e-5, aTol[1]);
        Assert.Equal(1e-3, aTol[2]);
        Assert.Equal(1e-5, aTol[aNbPoles]);
    }

    [Fact]
    public void UniformSection_BSplineSurfaceIsTheExtrusionInV()
    {
        BSplineCurve3D aCurve = WavyCurve();
        var aLaw = new UniformSection(aCurve, 2, 5);
        BSplineSurface? aSurf = aLaw.BSplineSurface();
        Assert.NotNull(aSurf);
        Assert.False(aSurf!.IsFrozen);
        Assert.Equal(1, aSurf.VDegree);
        Assert.Equal(aCurve.Value(0.3), aSurf.Value(0.3, 4));
        Assert.False(aCurve.IsFrozen); // the caller's B-spline is only read
        Assert.False(aLaw.IsRational);
        Point3D aBary = aLaw.BarycentreOfSurf();
        Assert.True(aBary.X > 0 && aBary.X < 4);
    }

    private static CurveAndTrihedron XLaw(double theFirst, double theLast)
    {
        // the path: the X axis from x = -2, a fixed frame (T = X, N = Y)
        var aLaw = new CurveAndTrihedron(new TestFixedTrihedron(new Vector3D(1, 0, 0), new Vector3D(0, 1, 0)));
        aLaw.SetCurve(new CurveAdaptor(new LineCurve3D(new Point3D(-2, 0, 0), new Direction3D(1, 0, 0)), theFirst, theLast));
        return aLaw;
    }

    [Fact]
    public void SectionPlacement_CircleCrossingThePath()
    {
        // a circle of radius 1 about the origin, in the plane x = 0: the path crosses its plane at parameter 2
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 1);
        var aPlace = new SurfaceSectionPlacement(XLaw(0, 10), aCircle);
        Assert.False(aPlace.IsDone);
        aPlace.Perform(1e-7);
        Assert.True(aPlace.IsDone);
        Assert.Equal(2.0, aPlace.ParameterOnPath(), 1e-12);
        Assert.Equal(M_PI / 2, aPlace.Angle(), 1e-12);
        Assert.Equal(Math.Sqrt(5.0), aPlace.Distance(), 1e-12); // from the path's start point to the circle

        Curve3D aSection = aPlace.Section(false);
        Assert.NotSame(aCircle, aSection);
        Assert.False(aSection.IsFrozen);
        var aPlaced = Assert.IsType<CircleCurve3D>(aSection);
        Assert.Equal(1.0, aPlaced.Radius, 1e-15);
        Curve3D aModified = aPlace.ModifiedSection(false);
        Assert.IsType<CircleCurve3D>(aModified);
        // the caller's circle is unchanged
        Assert.Equal(new Direction3D(1, 0, 0), aCircle.Axis.Direction);
    }

    [Fact]
    public void SectionPlacement_TransformationMapsTheSectionPointOntoThePath()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 3, 0), new Direction3D(1, 0, 0)), 1);
        CurveAndTrihedron aLaw = XLaw(0, 10);
        var aPlace = new SurfaceSectionPlacement(aLaw, aCircle);
        aPlace.Perform(4.0, 1e-7);
        Assert.Equal(4.0, aPlace.ParameterOnPath());
        // with the translation, the section's point lands on the path once the law's M and V are applied
        Transform3D aTrsf = aPlace.Transformation(true);
        Point3D aOnSection = aCircle.Value(aPlace.ParameterOnSection());
        Point3D aOnPath = aLaw.GetCurve()!.Value(4.0);
        Matrix3x3 M = new();
        Vector3D V = new();
        aLaw.D0(4.0, ref M, ref V);
        Assert.True(Swept(M, V, aOnSection.Transformed(aTrsf)).DistanceTo(aOnPath) < 1e-12);
        Curve3D aSection = aPlace.Section(true);
        Assert.True(Swept(M, V, aSection.Value(aPlace.ParameterOnSection())).DistanceTo(aOnPath) < 1e-12);
    }

    private static Point3D Swept(in Matrix3x3 M, in Vector3D V, in Point3D P) => new(P.XYZ().Multiplied(M).Added(V.XYZ()));

    [Fact]
    public void SectionPlacement_GeneralSection()
    {
        // a B-spline section near the path's end (its mean plane is used)
        BSplineCurve3D aCurve = WavyCurve();
        var aMoved = (BSplineCurve3D)aCurve.Translated(new Vector3D(8, 1, 0));
        var aPlace = new SurfaceSectionPlacement(XLaw(0, 10), aMoved);
        aPlace.Perform(1e-7);
        Assert.True(aPlace.IsDone);
        Assert.InRange(aPlace.ParameterOnPath(), 0.0, 10.0);
        Assert.False(aMoved.IsFrozen);
        Assert.NotSame(aMoved, aPlace.Section(true));

        // a line has no plane: the corrected transformation is refused
        var aLine = new SurfaceSectionPlacement(XLaw(0, 10), new TrimmedCurve3D(new LineCurve3D(new Point3D(3, 1, 1), new Direction3D(0, 1, 1)), 0, 2));
        aLine.Perform(1e-7);
        Assert.True(aLine.IsDone);
        Assert.Throws<FailureException>(() => aLine.Transformation(false, true));
    }

    [Fact]
    public void SectionPlacement_Point()
    {
        var aPlace = new SurfaceSectionPlacement(XLaw(0, 10), new CartesianPoint(3, 4, 0));
        aPlace.Perform(1e-7);
        Assert.Equal(5.0, aPlace.ParameterOnPath(), 1e-12);
        Assert.Equal(4.0, aPlace.Distance(), 1e-12);
        Assert.Equal(M_PI / 2, aPlace.Angle());
        CurveAndTrihedron aLaw = XLaw(0, 10);
        Matrix3x3 M = new();
        Vector3D V = new();
        aLaw.D0(5.0, ref M, ref V);
        Transform3D aTrsf = aPlace.Transformation(true);
        Assert.True(Swept(M, V, new Point3D(3, 4, 0).Transformed(aTrsf)).DistanceTo(new Point3D(3, 0, 0)) < 1e-12);

        aPlace.Perform(1.0, 1e-7);
        Assert.Equal(1.0, aPlace.ParameterOnPath());
        Assert.Equal(Math.Sqrt(16 + 16), aPlace.Distance(), 1e-12);
    }
}
