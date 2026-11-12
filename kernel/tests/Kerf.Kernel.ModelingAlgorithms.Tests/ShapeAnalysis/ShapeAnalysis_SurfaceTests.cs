// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Healing.Analysis;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeAnalysis_Surface: projections, singularities, closure, iso-curves and their boxes on every kind of surface:
/// plane, cylinder, cone, sphere, torus, revolution, extrusion, Bezier, B-spline (one with a degenerated edge),
/// offset and rectangular trimmed.
/// </summary>
public class ShapeAnalysis_SurfaceTests
{
    private const double Preci = 1e-7;

    private static readonly CoordinateSystem XOY = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));

    private static void AssertUV(double theU, double theV, Point2D theUV, double theTol = 1e-9)
    {
        Assert.Equal(theU, theUV.X, theTol);
        Assert.Equal(theV, theUV.Y, theTol);
    }

    private static NCollection_Array1<T> Array<T>(params T[] theValues)
    {
        var anArr = new NCollection_Array1<T>(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            anArr[i + 1] = theValues[i];
        }

        return anArr;
    }

    /// <summary>The saddle z = x·y on [0, 1]², as a Bezier surface of degree 2 x 2 scaled by <paramref name="theL"/>.</summary>
    private static BezierSurface Saddle(double theL)
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                double x = (i - 1) * 0.5, y = (j - 1) * 0.5;
                // bilinear (x, y) and the Bezier control net of x*y: z_ij = x_i*y_j
                aPoles[i, j] = new Point3D(theL * x, theL * y, theL * x * y);
            }
        }

        return new BezierSurface(aPoles);
    }

    /// <summary>
    /// A quadratic x linear B-spline patch whose U-first row collapses to the point (0, 0, 1): a cone-like sheet with a
    /// degenerated edge.
    /// </summary>
    private static BSplineSurface Degenerated()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 3, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 1);
        aPoles[1, 2] = new Point3D(0, 0, 1);
        aPoles[2, 1] = new Point3D(1, 0, 0.5);
        aPoles[2, 2] = new Point3D(1, 1, 0.5);
        aPoles[3, 1] = new Point3D(2, 0, 0);
        aPoles[3, 2] = new Point3D(2, 2, 0);
        return new BSplineSurface(aPoles, Array(0.0, 1.0), Array(0.0, 1.0), Array(3, 3), Array(2, 2), 2, 1);
    }

    [Fact]
    public void A_surface_with_no_bound_along_V_has_no_singular_bound_and_isnt_evaluated_there()
    {
        // #1365: a line turned about an axis it doesn't meet, and that surface offset: V runs to 2e100 each way. OCCT
        // evaluates the bounds' ends there, where the offset surface has no normal, and fixshape threw (decision 33002).
        var aRevolution = new RevolutionSurface(new LineCurve3D(new Point3D(4000, 0, 0), new Direction3D(0.6, -0.8, 0)),
                                                new Axis3D(new Point3D(-10000, 0, 0), new Direction3D(0, 1, 0)));
        foreach (Surface aSurface in new Surface[] { new OffsetSurface(aRevolution, 10), aRevolution })
        {
            var aSAS = new SurfaceAnalysis(aSurface);
            Assert.Equal(0, aSAS.NbSingularities(1e10));
            Assert.False(aSAS.HasSingularities(1));
            Assert.False(aSAS.IsDegenerated(new Point3D(4000, 0, 0), 1e-3));
        }
    }

    [Fact]
    public void Plane()
    {
        var aSAS = new SurfaceAnalysis(new PlaneSurface(XOY));
        AssertUV(3, 4, aSAS.ValueOfUV(new Point3D(3, 4, 5), Preci));
        Assert.Equal(5.0, aSAS.Gap(), 1e-12);
        Assert.Equal(0, aSAS.NbSingularities(1e10));
        Assert.False(aSAS.HasSingularities(1));
        Assert.False(aSAS.IsUClosed());
        Assert.False(aSAS.IsVClosed());
        Assert.Equal(Math.Sqrt(RealLast()), aSAS.UCloseVal()); // the square root of RealLast, as the distances
        double aDummy = 0;
        Point3D aP = new Point3D();
        Point2D aF = new Point2D(), aL = new Point2D();
        bool aUIso = false;
        Assert.False(aSAS.Singularity(1, ref aDummy, ref aP, ref aF, ref aL, ref aDummy, ref aDummy, ref aUIso));
        // NextValueOfUV on a plane is ValueOfUV
        AssertUV(-1, 2, aSAS.NextValueOfUV(new Point2D(0, 0), new Point3D(-1, 2, 0.5), Preci));
        // a straight pcurve is never degenerated on a plane
        Assert.False(aSAS.IsDegenerated(new Point2D(0, 0), new Point2D(0, 1), 1e-3, 10));
    }

    [Fact]
    public void Cylinder()
    {
        var aSAS = new SurfaceAnalysis(new CylindricalSurface(XOY, 2));
        AssertUV(0, 3, aSAS.ValueOfUV(new Point3D(2, 0, 3), Preci));
        AssertUV(1.5 * M_PI, 1, aSAS.ValueOfUV(new Point3D(0, -2, 1), Preci));
        Assert.Equal(0.0, aSAS.Gap(), 1e-15);
        AssertUV(0.5 * M_PI, -1, aSAS.ValueOfUV(new Point3D(0, 5, -1), Preci));
        Assert.Equal(3.0, aSAS.Gap(), 1e-12);
        Assert.True(aSAS.IsUClosed());
        Assert.Equal(0.0, aSAS.UCloseVal());
        Assert.False(aSAS.IsVClosed());
        Assert.Equal(0, aSAS.NbSingularities(1e10));

        // iso-curves: a line along V and a circle; bound isos are cached, all are frozen
        Curve3D? anIso = aSAS.UIso(0);
        Assert.IsType<LineCurve3D>(anIso);
        Assert.True(anIso!.IsFrozen);
        Assert.Same(anIso, aSAS.UIso(0));
        Assert.IsType<CircleCurve3D>(aSAS.VIso(1));
        Assert.NotSame(aSAS.VIso(1), aSAS.VIso(1));
    }

    [Fact]
    public void Cone_apex_is_a_singularity()
    {
        var aCone = new ConicalSurface(XOY, M_PI / 6, 1);
        var aSAS = new SurfaceAnalysis(aCone);
        Assert.Equal(1, aSAS.NbSingularities(0));
        double aPreci = -1, aFirstPar = 0, aLastPar = 0;
        Point3D aP = new Point3D();
        Point2D aF = new Point2D(), aL = new Point2D();
        bool aUIso = true;
        Assert.True(aSAS.Singularity(1, ref aPreci, ref aP, ref aF, ref aL, ref aFirstPar, ref aLastPar, ref aUIso));
        Assert.Equal(0.0, aPreci);
        Assert.True(aP.IsEqual(aCone.Apex, 1e-15));
        Assert.False(aUIso);
        Assert.Equal(-1 / Math.Sin(M_PI / 6), aF.Y);
        Assert.Equal(0.0, aF.X);
        Assert.Equal(2 * M_PI, aL.X);

        Assert.True(aSAS.IsDegenerated(aCone.Apex, Preci));
        Assert.False(aSAS.IsDegenerated(new Point3D(1, 0, 0), Preci));

        // ProjectDegenerated sets U from the neighbour
        var aResult = new Point2D(0, -2);
        Assert.True(aSAS.ProjectDegenerated(aCone.Apex, Preci, new Point2D(1.25, -1.5), ref aResult));
        AssertUV(1.25, -2, aResult, 0);

        // the straight pcurve along the apex is degenerated
        Assert.True(aSAS.IsDegenerated(new Point2D(0, -2), new Point2D(3, -2), 1e-3, 10));
        Assert.False(aSAS.IsDegenerated(new Point2D(0, 0), new Point2D(3, 0), 1e-3, 10));

        Point2D aUV = aSAS.ValueOfUV(aCone.Value(2.5, 0.75), Preci);
        AssertUV(2.5, 0.75, aUV);
        Assert.True(aSAS.IsUClosed());
        Assert.False(aSAS.IsVClosed());
    }

    [Fact]
    public void Sphere_poles_are_singularities()
    {
        var aSphere = new SphericalSurface(XOY, 1);
        var aSAS = new SurfaceAnalysis(aSphere);
        Assert.Equal(2, aSAS.NbSingularities(0));
        Assert.True(aSAS.HasSingularities(0));

        double aPreci = -1, aFirstPar = 0, aLastPar = 0;
        Point3D aP = new Point3D();
        Point2D aF = new Point2D(), aL = new Point2D();
        bool aUIso = true;
        Assert.True(aSAS.Singularity(1, ref aPreci, ref aP, ref aF, ref aL, ref aFirstPar, ref aLastPar, ref aUIso));
        Assert.Equal(1.0, aP.Z, 1e-15); // the northern pole first
        AssertUV(2 * M_PI, M_PI / 2, aF, 0);
        AssertUV(0, M_PI / 2, aL, 0);

        var aNorth = new Point3D(0, 0, 1.0000001);
        Point2D aDF = new Point2D(), aDL = new Point2D();
        Assert.True(aSAS.DegeneratedValues(aNorth, 1e-6, ref aDF, ref aDL, ref aFirstPar, ref aLastPar));
        AssertUV(2 * M_PI, M_PI / 2, aDF, 0);
        Assert.Equal(1e-7, aSAS.Gap(), 1e-12);
        Assert.False(aSAS.DegeneratedValues(new Point3D(1, 0, 0), 1e-6, ref aDF, ref aDL, ref aFirstPar, ref aLastPar));

        AssertUV(M_PI / 2, M_PI / 4, aSAS.ValueOfUV(aSphere.Value(M_PI / 2, M_PI / 4), Preci));
        Assert.True(aSAS.IsUClosed());
        Assert.False(aSAS.IsVClosed());

        // the multi-point form: points at the start lying in the south pole take their U from the first other point
        var aPoints = new NCollection_Sequence<Point3D>();
        var aUVs = new NCollection_Sequence<Point2D>();
        aPoints.Append(new Point3D(0, 0, -1));
        aUVs.Append(new Point2D(0, -M_PI / 2));
        aPoints.Append(aSphere.Value(1.0, -1.0));
        aUVs.Append(new Point2D(1.0, -1.0));
        aPoints.Append(aSphere.Value(1.0, 0.0));
        aUVs.Append(new Point2D(1.0, 0.0));
        Assert.True(aSAS.ProjectDegenerated(3, aPoints, aUVs, Preci, true));
        AssertUV(1.0, -M_PI / 2, aUVs[1], 0);
        Assert.False(aSAS.ProjectDegenerated(3, aPoints, aUVs, Preci, false));

        // all points degenerated: distributed evenly between the ends
        aPoints = new NCollection_Sequence<Point3D>();
        aUVs = new NCollection_Sequence<Point2D>();
        for (int i = 0; i < 3; i++)
        {
            aPoints.Append(new Point3D(0, 0, 1));
            aUVs.Append(new Point2D(i == 2 ? 2.0 : 0.0, M_PI / 2));
        }

        Assert.True(aSAS.ProjectDegenerated(3, aPoints, aUVs, Preci, true));
        Assert.Equal(1.0, aUVs[2].X, 1e-15);
    }

    [Fact]
    public void Torus_singularities_depend_on_the_radii()
    {
        var aTorus = new ToroidalSurface(XOY, 3, 1);
        var aSAS = new SurfaceAnalysis(aTorus);
        Assert.Equal(0, aSAS.NbSingularities(Preci));
        Assert.Equal(1, aSAS.NbSingularities(2));
        AssertUV(1.0, 2.0, aSAS.ValueOfUV(aTorus.Value(1.0, 2.0), Preci));
        AssertUV(5.0, 4.0, aSAS.ValueOfUV(aTorus.Value(5.0, 4.0), Preci));
        Assert.True(aSAS.IsUClosed());
        Assert.True(aSAS.IsVClosed());

        // a spindle torus (minor radius larger): two singular points
        var aSpindle = new SurfaceAnalysis(new ToroidalSurface(XOY, 1, 2));
        Assert.Equal(2, aSpindle.NbSingularities(0));
    }

    [Fact]
    public void Surface_of_revolution()
    {
        var aProfile = new TrimmedCurve3D(new LineCurve3D(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)), 0, 2);
        var aRev = new RevolutionSurface(aProfile, new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        var aSAS = new SurfaceAnalysis(aRev);
        AssertUV(M_PI / 2, 1, aSAS.ValueOfUV(new Point3D(0, 1, 1), Preci), 1e-7);
        Assert.Equal(0.0, aSAS.Gap(), 1e-7);
        AssertUV(M_PI / 2, 1, aSAS.ValueOfUV(new Point3D(0, 3, 1), Preci), 1e-7);
        Assert.Equal(2.0, aSAS.Gap(), 1e-7);
        Assert.True(aSAS.IsUClosed());
        Assert.False(aSAS.IsVClosed());
        Assert.Equal(4, aSAS.NbSingularities(1e10));
        Assert.Equal(0, aSAS.NbSingularities(1));

        // NextValueOfUV by Newton from a neighbour
        Point2D aNext = aSAS.NextValueOfUV(new Point2D(1.5, 0.9), aRev.Value(1.6, 1.1), Preci);
        AssertUV(1.6, 1.1, aNext, 1e-9);

        // a revolved line through the axis: a cone with its apex as a degenerated boundary
        var aSlant = new TrimmedCurve3D(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 1)), 0, 2);
        var aConeRev = new SurfaceAnalysis(new RevolutionSurface(aSlant, new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))));
        Assert.Equal(1, aConeRev.NbSingularities(Preci));
        Assert.True(aConeRev.IsDegenerated(new Point3D(0, 0, 0), Preci));
    }

    [Fact]
    public void Surface_of_linear_extrusion()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2);
        var anExtr = new ExtrusionSurface(aCircle, new Direction3D(0, 0, 1));
        var aSAS = new SurfaceAnalysis(anExtr);
        Assert.True(aSAS.IsUClosed());
        Assert.False(aSAS.IsVClosed());
        AssertUV(1.0, 3.0, aSAS.ValueOfUV(anExtr.Value(1.0, 3.0), Preci), 1e-7);

        // an open profile (an arc of half a circle): U isn't closed
        var anArc = new TrimmedCurve3D(aCircle, 0, M_PI);
        var aSAS2 = new SurfaceAnalysis(new ExtrusionSurface(anArc, new Direction3D(0, 0, 1)));
        Assert.False(aSAS2.IsUClosed());
        Assert.Equal(4.0, aSAS2.Gap(), 1e-12);
        Assert.Equal(RealLast(), aSAS2.UCloseVal()); // the ends are farther apart than the start and the middle
        // an almost closed profile is closed with a large enough precision
        var anAlmost = new TrimmedCurve3D(aCircle, 0, 2 * M_PI - 1e-4);
        var aSAS3 = new SurfaceAnalysis(new ExtrusionSurface(anAlmost, new Direction3D(0, 0, 1)));
        Assert.False(aSAS3.IsUClosed());
        Assert.True(aSAS3.IsUClosed(1e-3));
    }

    [Fact]
    public void Bezier_surface()
    {
        BezierSurface aSaddle = Saddle(2);
        var aSAS = new SurfaceAnalysis(aSaddle);
        AssertUV(0.3, 0.6, aSAS.ValueOfUV(aSaddle.Value(0.3, 0.6), Preci), 1e-8);
        Assert.True(aSAS.Gap() < 1e-8);
        // a point off the surface near a corner
        Point2D aUV = aSAS.ValueOfUV(new Point3D(-0.1, -0.1, 0), Preci);
        Assert.Equal(aSaddle.Value(aUV.X, aUV.Y).DistanceTo(new Point3D(-0.1, -0.1, 0)), aSAS.Gap(), 1e-12);
        Assert.False(aSAS.IsUClosed());
        Assert.False(aSAS.IsVClosed());
        Assert.Equal(4, aSAS.NbSingularities(1e10));
        Assert.Equal(0, aSAS.NbSingularities(1));
        AssertUV(0.35, 0.55, aSAS.NextValueOfUV(new Point2D(0.3, 0.6), aSaddle.Value(0.35, 0.55), Preci), 1e-9);

        // the iso boxes
        Box aBox = aSAS.GetBoxUF();
        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);
        Assert.Equal(0.0, aXmin, 1e-6);
        Assert.Equal(0.0, aXmax, 1e-6);
        Assert.Equal(2.0, aYmax, 1e-6);
        Assert.False(aSAS.GetBoxVL().IsVoid);
    }

    [Fact]
    public void BSpline_with_a_degenerated_edge()
    {
        BSplineSurface aBS = Degenerated();
        var aSAS = new SurfaceAnalysis(aBS);
        Assert.Equal(1, aSAS.NbSingularities(Preci));
        Assert.True(aSAS.IsDegenerated(new Point3D(0, 0, 1), Preci));
        double aPreci = -1, aFirstPar = 0, aLastPar = 0;
        Point3D aP = new Point3D();
        Point2D aF = new Point2D(), aL = new Point2D();
        bool aUIso = false;
        Assert.True(aSAS.Singularity(1, ref aPreci, ref aP, ref aF, ref aL, ref aFirstPar, ref aLastPar, ref aUIso));
        Assert.Equal(0.0, aPreci);
        Assert.True(aUIso); // the U-first boundary
        AssertUV(0, 1, aF, 0);
        AssertUV(0, 0, aL, 0);

        Assert.True(aSAS.IsDegenerated(new Point2D(0, 0), new Point2D(0, 1), 1e-3, 10));
        Assert.False(aSAS.IsDegenerated(new Point2D(1, 0), new Point2D(1, 1), 1e-3, 10));

        // projecting the apex: ValueOfUV finds U = 0, any V
        Point2D aUV = aSAS.ValueOfUV(new Point3D(0, 0, 1), Preci);
        Assert.Equal(0.0, aUV.X, 1e-7);
        Assert.True(aSAS.Gap() < 1e-7);
        var aResult = aUV;
        Assert.True(aSAS.ProjectDegenerated(new Point3D(0, 0, 1), Preci, new Point2D(0.3, 0.7), ref aResult));
        Assert.Equal(0.7, aResult.Y);

        AssertUV(0.4, 0.3, aSAS.ValueOfUV(aBS.Value(0.4, 0.3), Preci), 1e-8);
        AssertUV(0.45, 0.35, aSAS.NextValueOfUV(new Point2D(0.4, 0.3), aBS.Value(0.45, 0.35), Preci), 1e-9);

        Assert.False(aSAS.IsUClosed());
        Assert.False(aSAS.IsVClosed());
    }

    [Fact]
    public void Closed_BSpline_surface()
    {
        // a cylinder-like B-spline: four rows of poles around a square, the last equal to the first
        var aPoles = new NCollection_Array2<Point3D>(1, 5, 1, 2);
        double[] aXs = [1, 0, -1, 0, 1], aYs = [0, 1, 0, -1, 0];
        for (int i = 1; i <= 5; i++)
        {
            aPoles[i, 1] = new Point3D(aXs[i - 1], aYs[i - 1], 0);
            aPoles[i, 2] = new Point3D(aXs[i - 1], aYs[i - 1], 1);
        }

        var aBS = new BSplineSurface(aPoles, Array(0.0, 1.0, 2.0, 3.0, 4.0), Array(0.0, 1.0), Array(2, 1, 1, 1, 2), Array(2, 2), 1, 1);
        var aSAS = new SurfaceAnalysis(aBS);
        Assert.True(aSAS.IsUClosed());
        Assert.False(aSAS.IsVClosed());
        AssertUV(1.5, 0.25, aSAS.ValueOfUV(aBS.Value(1.5, 0.25), Preci), 1e-8);
        // near a C0 knot, NextValueOfUV goes through ValueOfUV
        AssertUV(2.0, 0.5, aSAS.NextValueOfUV(new Point2D(2.0, 0.4), aBS.Value(2.0, 0.5), Preci), 1e-8);
    }

    [Fact]
    public void Offset_surface()
    {
        var anOffset = new OffsetSurface(Saddle(2), 0.5);
        var aSAS = new SurfaceAnalysis(anOffset);
        AssertUV(0.4, 0.7, aSAS.ValueOfUV(anOffset.Value(0.4, 0.7), Preci), 1e-7);
        Assert.Equal(4, aSAS.NbSingularities(1e10));
        Assert.False(aSAS.IsUClosed());
        double aU = 0.5, aV = 0.5;
        double aDist = aSAS.UVFromIso(anOffset.Value(0.0, 0.25), Preci, ref aU, ref aV);
        Assert.True(aDist < 1e-6, $"distance {aDist}");
        Assert.Equal(0.0, aU, 1e-6);
        Assert.Equal(0.25, aV, 1e-6);

        // an offset of a cylinder is (for GeomAdaptor) a cylinder
        var aCylOffset = new SurfaceAnalysis(new OffsetSurface(new CylindricalSurface(XOY, 2), 1));
        AssertUV(M_PI / 2, 4, aCylOffset.ValueOfUV(new Point3D(0, 3, 4), Preci));
    }

    [Fact]
    public void Rectangular_trimmed_surface()
    {
        var aTrimmed = new TrimmedSurface(new CylindricalSurface(XOY, 1), 0, M_PI, 0, 2);
        var aSAS = new SurfaceAnalysis(aTrimmed);
        Assert.False(aSAS.IsUClosed());
        Assert.Equal(RealLast(), aSAS.UCloseVal()); // the ends are farther apart than the start and the middle
        Assert.Equal(2.0, aSAS.Gap(), 1e-12);
        Assert.False(aSAS.IsVClosed());
        aSAS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(M_PI, aU2);
        Assert.Equal(2.0, aV2);
        Assert.Equal(0.0, aU1);
        Assert.Equal(0.0, aV1);
        // trimmed surfaces are bounded: 4 boundaries, none degenerated
        Assert.Equal(0, aSAS.NbSingularities(Preci));
        AssertUV(M_PI / 2, 1, aSAS.ValueOfUV(new Point3D(0, 1, 1), Preci));

        // boxes of the bound isos
        Box aBox = aSAS.GetBoxUL();
        aBox.Get(out double aXmin, out _, out double aZmin, out double aXmax, out _, out double aZmax);
        Assert.Equal(-1.0, aXmin, 1e-6);
        Assert.Equal(-1.0, aXmax, 1e-6);
        Assert.Equal(0.0, aZmin, 1e-6);
        Assert.Equal(2.0, aZmax, 1e-6);
        aBox = aSAS.GetBoxVF();
        aBox.Get(out _, out _, out aZmin, out _, out _, out aZmax);
        Assert.Equal(0.0, aZmax, 1e-6);

        // a full trimmed cylinder is U-closed by the 101-point check
        var aFull = new SurfaceAnalysis(new TrimmedSurface(new CylindricalSurface(XOY, 1), 0, 2 * M_PI, 0, 2));
        Assert.True(aFull.IsUClosed());
    }

    [Fact]
    public void UVFromIso_refines_a_rough_solution()
    {
        var aTrimmed = new TrimmedSurface(Saddle(2), 0, 1, 0, 1);
        var aSAS = new SurfaceAnalysis(aTrimmed);
        double aU = 0.1, aV = 0.9;
        double aDist = aSAS.UVFromIso(aTrimmed.Value(1.0, 0.3), Preci, ref aU, ref aV);
        Assert.True(aDist < 1e-7, $"distance {aDist}");
        Assert.Equal(1.0, aU, 1e-7);
        Assert.Equal(0.3, aV, 1e-7);

        // already close enough: unchanged
        aU = 0.5;
        aV = 0.5;
        aDist = aSAS.UVFromIso(aTrimmed.Value(0.5, 0.5), Preci, ref aU, ref aV);
        Assert.Equal(0.0, aDist);
        Assert.Equal(0.5, aU);
    }

    [Fact]
    public void Init_and_SetDomain()
    {
        var aSAS = new SurfaceAnalysis(new SphericalSurface(XOY, 1));
        Assert.Equal(2, aSAS.NbSingularities(0));
        var aCyl = new CylindricalSurface(XOY, 1);
        aSAS.Init(aCyl);
        Assert.Same(aCyl, aSAS.Surface());
        Assert.Equal(0, aSAS.NbSingularities(0));

        // a copy of another analyzer takes its singularities and adaptor
        var aSphere = new SurfaceAnalysis(new SphericalSurface(XOY, 1));
        aSphere.NbSingularities(0);
        var aCopy = new SurfaceAnalysis(new PlaneSurface(XOY));
        aCopy.Init(aSphere);
        Assert.Equal(2, aCopy.NbSingularities(0));
        Assert.Same(aSphere.TrueAdaptor3d(), aCopy.Adaptor3d());

        aCopy.SetDomain(0, 1, -0.5, 0.5);
        aCopy.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((0.0, 1.0, -0.5, 0.5), (aU1, aU2, aV1, aV2));

        // a null surface: an empty analyzer, as OCCT (which warns)
        var aNull = new SurfaceAnalysis(null);
        Assert.Null(aNull.Surface());
        Assert.Null(aNull.Adaptor3d());
        Assert.Equal(0, aNull.NbSingularities(1));
    }

    [Fact]
    public void Value_evaluates_the_adaptor()
    {
        var aSAS = new SurfaceAnalysis(new CylindricalSurface(XOY, 2));
        Assert.True(aSAS.Value(M_PI / 2, 3).IsEqual(new Point3D(0, 2, 3), 1e-15));
        Assert.True(aSAS.Value(new Point2D(0, 1)).IsEqual(new Point3D(2, 0, 1), 1e-15));
    }
}
