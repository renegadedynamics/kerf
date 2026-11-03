// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of GeomFill_GordonUtilities, GeomFill_NetworkSurface and GeomFill_Gordon beyond OCCT's GTest: the
/// preparation helpers, frozen inputs and results, determinism.
/// </summary>
public class GeomFill_GordonTests
{
    private static BSplineCurve3D Line(Point3D theP1, Point3D theP2) => GeomFill_Gordon_Test.makeLinearBSpline(theP1, theP2);

    private static NCollection_Array1<BSplineCurve3D?> BSplines(params BSplineCurve3D[] theCurves)
    {
        NCollection_Array1<BSplineCurve3D?> anArray = new(1, theCurves.Length);
        for (int i = 0; i < theCurves.Length; ++i)
        {
            anArray[i + 1] = theCurves[i];
        }
        return anArray;
    }

    private static string Snapshot(BSplineCurve3D theCurve) =>
        string.Join(";", theCurve.Poles().AsSpan().ToArray().Select(p => p.ToString()))
        + "|" + string.Join(";", theCurve.Knots().AsSpan().ToArray()) + "|" + theCurve.Degree() + "|" + theCurve.IsPeriodic;

    [Fact]
    public void CompactParameters_sorts_and_merges_values_within_the_resolution()
    {
        NCollection_LinearVector<double> aParams = new();
        foreach (double aValue in new[] { 0.5, 0.1, 0.9, 0.5 + 1.0e-12, 0.1, 0.0 })
        {
            aParams.Append(aValue);
        }
        GordonUtilities.compactParameters(aParams);
        Assert.Equal([0.0, 0.1, 0.5, 0.9], aParams.AsSpan().ToArray());
    }

    [Fact]
    public void AddIntervalMidpoints_splits_the_widest_intervals()
    {
        NCollection_LinearVector<double> aParams = new();
        aParams.Append(0.0);
        aParams.Append(1.0);
        GordonUtilities.addIntervalMidpoints(aParams, 5);
        Assert.Equal([0.0, 0.25, 0.5, 0.75, 1.0], aParams.AsSpan().ToArray());
    }

    [Fact]
    public void ToMatrix_shares_the_table_storage()
    {
        NCollection_Array2<double> aTable = new(1, 2, 1, 3);
        Matrix aMatrix = GordonUtilities.toMatrix(aTable);
        aMatrix[2, 3] = 7.0;
        aTable[1, 2] = 5.0;
        Assert.Equal(7.0, aTable[2, 3]);
        Assert.Equal(5.0, aMatrix[1, 2]);
        Assert.Equal(2, aMatrix.RowNumber());
        Assert.Equal(3, aMatrix.ColNumber());

        math_Vector aColumns = GordonUtilities.columnMeans(aMatrix);
        math_Vector aRows = GordonUtilities.rowMeans(aMatrix);
        Assert.Equal(2.5, aColumns[2]);
        Assert.Equal(3.5, aColumns[3]);
        Assert.Equal(5.0 / 3.0, aRows[1], 1.0e-15);
        Assert.Equal(7.0 / 3.0, aRows[2], 1.0e-15);
    }

    [Fact]
    public void CollectContacts_finds_the_crossing_of_two_segments()
    {
        BSplineCurve3D aProfile = Line(new Point3D(0, 0.25, 0), new Point3D(1, 0.25, 0));
        BSplineCurve3D aGuide = Line(new Point3D(0.75, 0, 0), new Point3D(0.75, 1, 0));
        GordonUtilities.ContactSet aSet = GordonUtilities.collectContacts(aProfile, aGuide, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1, aSet.Candidates.Size());
        Assert.True(aSet.Candidates.First().IsValid);
        Assert.Equal(0.75, aSet.Candidates.First().ProfileParam, 1.0e-12);
        Assert.Equal(0.25, aSet.Candidates.First().GuideParam, 1.0e-12);

        // curved: the extrema and boundary searches, sorted by gap
        BSplineCurve3D aCurved = GeomFill_Gordon_Test.makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0.5, 0),
                                                                              new Point3D(1, 0, 0));
        BSplineCurve3D aCross = Line(new Point3D(0.5, -1, 0), new Point3D(0.5, 1, 0));
        GordonUtilities.ContactSet aCurvedSet = GordonUtilities.collectContacts(aCurved, aCross, 1.0e-6);
        Assert.NotEqual(0, aCurvedSet.Candidates.Size());
        Assert.Equal(0.5, aCurvedSet.Candidates.First().ProfileParam, 1.0e-6);
        for (int i = 1; i < aCurvedSet.Candidates.Size(); ++i)
        {
            Assert.True(aCurvedSet.Candidates.Value(i - 1).Gap <= aCurvedSet.Candidates.Value(i).Gap);
        }

        // disjoint: nothing
        BSplineCurve3D aFar = Line(new Point3D(3, 0, 0), new Point3D(3, 1, 0));
        Assert.Equal(0, GordonUtilities.collectContacts(aProfile, aFar, global::Kerf.Engine.Foundation.Precision.Confusion()).Candidates.Size());
    }

    [Fact]
    public void NetworkPreparation_orders_and_orients_a_shuffled_network()
    {
        // profiles at y = 1, 0, 0.5 (the middle one reversed), guides at x = 1, 0
        NCollection_Array1<BSplineCurve3D?> aProfiles =
          BSplines(Line(new Point3D(0, 1, 0), new Point3D(1, 1, 0)), Line(new Point3D(0, 0, 0), new Point3D(1, 0, 0)),
                   Line(new Point3D(1, 0.5, 0), new Point3D(0, 0.5, 0)));
        NCollection_Array1<BSplineCurve3D?> aGuides =
          BSplines(Line(new Point3D(1, 0, 0), new Point3D(1, 1, 0)), Line(new Point3D(0, 0, 0), new Point3D(0, 1, 0)));
        NCollection_Array2<double> aProfileParams = new(1, 3, 1, 2);
        NCollection_Array2<double> aGuideParams = new(1, 3, 1, 2);
        GordonUtilities.NetworkPreparation aNetwork = new(aProfiles, aGuides, aProfileParams, aGuideParams,
                                                          global::Kerf.Engine.Foundation.Precision.Confusion(), false);
        Assert.True(aNetwork.ConvertCurvesToUnitInterval());
        Assert.True(aNetwork.FillIntersectionParameters());
        Assert.True(aNetwork.ReorderNetwork());
        bool isUClosed = false, isVClosed = false;
        Assert.True(aNetwork.NormalizeIntersectionParameters(ref isUClosed, ref isVClosed));
        Assert.False(isUClosed);
        Assert.False(isVClosed);

        // profiles ordered by y, each running in +x; guides ordered by x, each in +y
        Assert.Equal(0.0, aProfiles[1]!.StartPoint().Y(), 1.0e-12);
        Assert.Equal(0.5, aProfiles[2]!.StartPoint().Y(), 1.0e-12);
        Assert.Equal(1.0, aProfiles[3]!.StartPoint().Y(), 1.0e-12);
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(0.0, aProfiles[i]!.StartPoint().X(), 1.0e-12);
        }
        Assert.Equal(0.0, aGuides[1]!.StartPoint().X(), 1.0e-12);
        Assert.Equal(1.0, aGuides[2]!.StartPoint().X(), 1.0e-12);
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(0.0, aProfileParams[i, 1], 1.0e-12);
            Assert.Equal(1.0, aProfileParams[i, 2], 1.0e-12);
        }
        Assert.Equal(0.0, aGuideParams[1, 1], 1.0e-12);
        Assert.Equal(0.5, aGuideParams[2, 1], 1.0e-12);
        Assert.Equal(1.0, aGuideParams[3, 1], 1.0e-12);

        NCollection_Array2<Point3D> aGrid = aNetwork.IntersectionPoints();
        Assert.Equal(2, aGrid.ColLength());
        Assert.Equal(3, aGrid.RowLength());
        Assert.True(aGrid[2, 3].Distance(new Point3D(1, 1, 0)) < 1.0e-12);
        NCollection_Array2<double> aWeights = aNetwork.IntersectionWeights();
        Assert.All(aWeights.AsSpan().ToArray(), w => Assert.Equal(1.0, w));
    }

    [Fact]
    public void ScalarLaw_is_a_monotone_piecewise_linear_map()
    {
        GordonUtilities.ScalarLaw aLaw = new();
        math_Vector aSource = new(1, 3);
        math_Vector aTarget = new(1, 3);
        aSource[1] = 0.0; aSource[2] = 0.2; aSource[3] = 1.0;
        aTarget[1] = 0.0; aTarget[2] = 0.5; aTarget[3] = 1.0;
        Assert.True(aLaw.Init(aSource, aTarget, 3));
        Assert.Equal(0.1, aLaw.Value(0.25), 1.0e-15);
        Assert.Equal(0.6, aLaw.Value(0.75), 1.0e-15);
        Assert.Equal(0.0, aLaw.Value(-1.0));
        Assert.Equal(1.0, aLaw.Value(2.0));
        aLaw.Evaluate(1, default, 0.25, out double aDerivative, out int anError);
        Assert.Equal(0, anError);
        Assert.Equal(0.4, aDerivative, 1.0e-15);
        aLaw.Evaluate(1, default, 0.75, out aDerivative, out _);
        Assert.Equal(1.6, aDerivative, 1.0e-15);

        aTarget[2] = 0.0; // not increasing
        Assert.False(new GordonUtilities.ScalarLaw().Init(aSource, aTarget, 3));
        Assert.False(new GordonUtilities.ScalarLaw().Init(aSource, aTarget, 1));
    }

    [Fact]
    public void ReparametrizeExactly_composes_the_curve_with_the_law()
    {
        BSplineCurve3D aCurve = GeomFill_Gordon_Test.makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 1, 0),
                                                                             new Point3D(1, 0, 0.5));
        math_Vector aSource = new(1, 3);
        math_Vector aTarget = new(1, 3);
        aSource[1] = 0.0; aSource[2] = 0.3; aSource[3] = 1.0;
        aTarget[1] = 0.0; aTarget[2] = 0.6; aTarget[3] = 1.0;
        GordonUtilities.ScalarLaw aLaw = new();
        Assert.True(aLaw.Init(aSource, aTarget, 3));
        BSplineCurve3D? aResult = null;
        Assert.True(GordonUtilities.reparametrizeExactly(aCurve, aSource, aTarget, 3, aLaw, ref aResult));
        Assert.NotNull(aResult);
        for (int i = 0; i <= 20; ++i)
        {
            double t = i / 20.0;
            Assert.True(aResult!.Value(t).Distance(aCurve.Value(aLaw.Value(t))) < 1.0e-12);
        }
        // the curve itself is not changed
        Assert.Equal(1.0, aCurve.LastParameter());
        Assert.Equal(3, aCurve.PoleCount);
    }

    [Fact]
    public void Gordon_leaves_frozen_input_curves_unchanged_and_freezes_its_result()
    {
        BSplineCurve3D[] aProfileCurves =
        [
            GeomFill_Gordon_Test.makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0.4), new Point3D(1, 0, 0)),
            GeomFill_Gordon_Test.makeQuadraticBSpline(new Point3D(0, 0.5, 0), new Point3D(0.5, 0.5, 0.6), new Point3D(1, 0.5, 0)),
            GeomFill_Gordon_Test.makeQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(0.5, 1, 0.4), new Point3D(1, 1, 0)),
        ];
        BSplineCurve3D[] aGuideCurves = [Line(new Point3D(0, 0, 0), new Point3D(0, 1, 0)), Line(new Point3D(1, 0, 0), new Point3D(1, 1, 0))];
        NCollection_Array1<Curve3D> aProfiles = new(1, 3);
        NCollection_Array1<Curve3D> aGuides = new(1, 2);
        List<string> aBefore = [];
        for (int i = 0; i < 3; ++i)
        {
            aProfileCurves[i].Freeze();
            aProfiles[i + 1] = aProfileCurves[i];
            aBefore.Add(Snapshot(aProfileCurves[i]));
        }
        for (int i = 0; i < 2; ++i)
        {
            aGuideCurves[i].Freeze();
            aGuides[i + 1] = aGuideCurves[i];
            aBefore.Add(Snapshot(aGuideCurves[i]));
        }

        GeomFill_Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone(), aGordon.Status().ToString());
        BSplineSurface aSurface = aGordon.Surface();
        Assert.True(aSurface.IsFrozen);
        Assert.Same(aSurface, aGordon.Surface());

        List<string> anAfter = [.. aProfileCurves.Select(Snapshot), .. aGuideCurves.Select(Snapshot)];
        Assert.Equal(aBefore, anAfter);
        Assert.Same(aProfileCurves[0], aProfiles[1]);

        // a second Perform from the same seeds gives the same poles
        BSplineSurface aCopy = (BSplineSurface)aSurface.Copy();
        aGordon.Perform();
        Assert.Equal(aCopy.Poles().AsSpan().ToArray(), aGordon.Surface().Poles().AsSpan().ToArray());
    }

    [Fact]
    public void NetworkSurface_keeps_frozen_curves_and_interpolates_them()
    {
        BSplineCurve3D[] aProfileCurves =
        [
            GeomFill_Gordon_Test.makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0.4), new Point3D(1, 0, 0)),
            GeomFill_Gordon_Test.makeQuadraticBSpline(new Point3D(0, 0.5, 0), new Point3D(0.5, 0.5, 0.6), new Point3D(1, 0.5, 0)),
            GeomFill_Gordon_Test.makeQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(0.5, 1, 0.4), new Point3D(1, 1, 0)),
        ];
        // guides through the profiles' points at u = 0, 0.5, 1
        NCollection_Array1<double> aV = new(1, 3);
        aV[1] = 0.0; aV[2] = 0.5; aV[3] = 1.0;
        NCollection_Array1<double> aU = new(1, 3);
        aU[1] = 0.0; aU[2] = 0.5; aU[3] = 1.0;
        BSplineCurve3D[] aGuideCurves = new BSplineCurve3D[3];
        for (int j = 0; j < 3; ++j)
        {
            NCollection_HArray1<Point3D> aPoints = new(1, 3);
            NCollection_HArray1<double> aParams = new(1, 3);
            for (int i = 0; i < 3; ++i)
            {
                aPoints.SetValue(i + 1, aProfileCurves[i].Value(aU[j + 1]));
                aParams.SetValue(i + 1, aV[i + 1]);
            }
            GeomAPI_Interpolate anInterpolator = new(aPoints, aParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
            anInterpolator.Perform();
            aGuideCurves[j] = anInterpolator.Curve(); // frozen
        }
        foreach (BSplineCurve3D aCurve in aProfileCurves)
        {
            aCurve.Freeze();
        }
        NCollection_Array1<BSplineCurve3D?> aProfiles = BSplines(aProfileCurves);
        NCollection_Array1<BSplineCurve3D?> aGuides = BSplines(aGuideCurves);
        NCollection_Array2<Point3D> aGrid = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        aWeights.Init(1.0);
        for (int j = 1; j <= 3; ++j)
        {
            for (int i = 1; i <= 3; ++i)
            {
                aGrid[j, i] = aProfileCurves[i - 1].Value(aU[j]);
            }
        }
        List<string> aBefore = [.. aProfileCurves.Select(Snapshot), .. aGuideCurves.Select(Snapshot)];

        GeomFill_NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, aV, aU, aGrid, aWeights, global::Kerf.Engine.Foundation.Precision.Confusion(), false, false);
        aNetwork.Perform();
        Assert.True(aNetwork.IsDone(), aNetwork.Status().ToString());
        BSplineSurface aSurface = aNetwork.Surface();
        Assert.True(aSurface.IsFrozen);

        Assert.Equal(aBefore, [.. aProfileCurves.Select(Snapshot), .. aGuideCurves.Select(Snapshot)]);
        Assert.Same(aProfileCurves[1], aProfiles[2]);

        // the surface (reparametrised to [0, 1]^2) passes through every profile and guide
        for (int k = 0; k <= 10; ++k)
        {
            double t = k / 10.0;
            for (int i = 0; i < 3; ++i)
            {
                Assert.True(aSurface.Value(t, aV[i + 1]).Distance(aProfileCurves[i].Value(t)) < 1.0e-9);
                Assert.True(aSurface.Value(aU[i + 1], t).Distance(aGuideCurves[i].Value(t)) < 1.0e-9);
            }
        }
    }

    [Fact]
    public void Gordon_report_is_a_value()
    {
        GeomFill_Gordon.BuildReport aReport = new();
        Assert.Equal(GeomFill_Gordon.ResultStatus.NotStarted, aReport.Status);
        Assert.Equal(aReport, new GeomFill_Gordon.BuildReport());
        GeomFill_Gordon.BuildReport aChanged = aReport;
        aChanged.MaxContactGap = 1.0;
        Assert.NotEqual(aReport, aChanged);
        Assert.Equal(0.0, aReport.MaxContactGap);
    }
}
