// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Topology;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// ShapeUpgrade's package functions: the conversion of a C0 B-spline curve into a sequence of C1 B-spline curves.
/// </summary>
/// <remarks>
/// OCCT's commented-out debug switch (SetDebug, Debug) is left out.
/// </remarks>
internal static class ShapeUpgrade
{
    /// <summary>
    /// Converts a C0 B-spline curve into a sequence of C1 B-spline curves, splitting it at the knots whose
    /// multiplicity equals the degree: unlike GeomConvert's C0BSplineToArrayOfC1BSplineCurve it uses no tolerance and
    /// doesn't change the geometry. False (and <paramref name="seqBS"/> unchanged) if the curve is null or already C1.
    /// </summary>
    /// <remarks>
    /// OCCT makes the curve non-periodic in place before splitting it. The port does so only for a curve ported code
    /// owns; a frozen curve (any a caller holds) is left as it is and a non-periodic copy is split (decision 998).
    /// </remarks>
    public static bool C0BSplineToSequenceOfC1BSplineCurve(BSplineCurve3D? bs,
                                                           ref NCollection_HSequence<BoundedCurve3D>? seqBS)
    {
        if (bs is null || (bs.IsCN(1)))
        {
            return false;
        }

        seqBS = new NCollection_HSequence<BoundedCurve3D>();
        // OCCT.NET: copy on write of a frozen curve (decision 998)
        if (bs.IsFrozen)
        {
            bs = (BSplineCurve3D)bs.Copy();
        }
        bs.SetNotPeriodic(); // to have equation NbPoles = NbKnots with Multiplicities - degree - 1

        int deg = bs.Degree;
        NCollection_Array1<Point3D> poles = bs.InternalPoles();
        NCollection_Array1<double> weights = bs.InternalWeightsArray();
        NCollection_Array1<double> knots = bs.InternalKnots();
        NCollection_Array1<int> mults = bs.InternalMultiplicities();
        NCollection_Array1<double> knotSequence = bs.InternalKnotSequence();

        int startKnotIndex, endKnotIndex, j;

        startKnotIndex = bs.FirstUKnotIndex();
        for (endKnotIndex = startKnotIndex + 1; endKnotIndex <= bs.LastUKnotIndex(); endKnotIndex++)
        {
            if ((mults[endKnotIndex] < deg) && (endKnotIndex < bs.LastUKnotIndex()))
            {
                continue;
            }

            int startFlatIndex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(deg, startKnotIndex, mults, false);
            //    StartFlatIndex += Mults (StartKnotIndex) - 1;
            int endFlatIndex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(deg, endKnotIndex, mults, false);
            endFlatIndex -= mults[endKnotIndex] - 1;

            var tempKnots = new NCollection_Array1<double>(1, knots.Length());
            var tempMults = new NCollection_Array1<int>(1, knots.Length());
            tempMults.Init(1);
            int tempKnotIndex = 1;
            tempKnots[tempKnotIndex] = knotSequence[startFlatIndex - deg];

            for (j = startFlatIndex - deg + 1; j <= endFlatIndex + deg; j++)
            {
                if (Math.Abs(knotSequence[j] - knotSequence[j - 1]) <= Constants.Resolution())
                {
                    tempMults[tempKnotIndex]++;
                }
                else
                {
                    tempKnots[++tempKnotIndex] = knotSequence[j];
                }
            }

            int tempStartIndex = 1, tempEndIndex = tempKnotIndex;
            if (tempMults[tempStartIndex] == 1)
            {
                tempMults[++tempStartIndex]++;
            }
            if (tempMults[tempEndIndex] == 1)
            {
                tempMults[--tempEndIndex]++;
            }

            int newNbKnots = tempEndIndex - tempStartIndex + 1;
            var newMults = new NCollection_Array1<int>(1, newNbKnots);
            var newKnots = new NCollection_Array1<double>(1, newNbKnots);
            for (j = 1; j <= newNbKnots; j++)
            {
                newMults[j] = tempMults[j + tempStartIndex - 1];
                newKnots[j] = tempKnots[j + tempStartIndex - 1];
            }

            int newNbPoles = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(deg, false, newMults);
            var newPoles = new NCollection_Array1<Point3D>(1, newNbPoles);
            var newWeights = new NCollection_Array1<double>(1, newNbPoles);
            int poleIndex = startFlatIndex - deg; // Index of starting pole when splitting B-Spline is an index of starting knot
            for (j = 1; j <= newNbPoles; j++)
            {
                newWeights[j] = weights[j + poleIndex - 1];
                newPoles[j] = poles[j + poleIndex - 1];
            }

            var newC = new BSplineCurve3D(newPoles, newWeights, newKnots, newMults, deg);
            seqBS.Append(newC);

            startKnotIndex = endKnotIndex;
        }
        return true;
    }

    private static BSplineCurve3D BSplineCurve2dTo3d(BSplineCurve2D bs)
    {
        int deg = bs.Degree;
        int nbPoles = bs.PoleCount;
        NCollection_Array1<Point2D> poles2d = bs.InternalPoles();
        NCollection_Array1<double> weights = bs.InternalWeightsArray();
        NCollection_Array1<double> knots = bs.InternalKnots();
        NCollection_Array1<int> mults = bs.InternalMultiplicities();

        var poles3d = new NCollection_Array1<Point3D>(1, nbPoles);
        for (int i = 1; i <= nbPoles; i++)
        {
            poles3d[i] = new Point3D(poles2d[i].X, poles2d[i].Y, 0);
        }

        var bs3d = new BSplineCurve3D(poles3d, weights, knots, mults, deg, bs.IsPeriodic);
        return bs3d;
    }

    private static BSplineCurve2D BSplineCurve3dTo2d(BSplineCurve3D bs)
    {
        int deg = bs.Degree;
        int nbPoles = bs.PoleCount;
        NCollection_Array1<Point3D> poles3d = bs.InternalPoles();
        NCollection_Array1<double> weights = bs.InternalWeightsArray();
        NCollection_Array1<double> knots = bs.InternalKnots();
        NCollection_Array1<int> mults = bs.InternalMultiplicities();

        var poles2d = new NCollection_Array1<Point2D>(1, nbPoles);
        for (int i = 1; i <= nbPoles; i++)
        {
            poles2d[i] = new Point2D(poles3d[i].X, poles3d[i].Y);
        }

        var bs2d = new BSplineCurve2D(poles2d, weights, knots, mults, deg, bs.IsPeriodic);
        return bs2d;
    }

    /// <summary>
    /// The 2D form of <see cref="C0BSplineToSequenceOfC1BSplineCurve(BSplineCurve3D?, ref NCollection_HSequence{BoundedCurve3D}?)"/>,
    /// through a 3D copy of the curve (so the curve given is never changed).
    /// </summary>
    public static bool C0BSplineToSequenceOfC1BSplineCurve(BSplineCurve2D? bs,
                                                           ref NCollection_HSequence<BoundedCurve2D>? seqBS)
    {
        if (bs is null || (bs.IsCN(1)))
        {
            return false;
        }

        BSplineCurve3D bs3d = BSplineCurve2dTo3d(bs);
        NCollection_HSequence<BoundedCurve3D>? seqBS3d = null;
        bool result = C0BSplineToSequenceOfC1BSplineCurve(bs3d, ref seqBS3d);
        if (result)
        {
            seqBS = new NCollection_HSequence<BoundedCurve2D>();
            for (int i = 1; i <= seqBS3d!.Length(); i++)
            {
                seqBS.Append(BSplineCurve3dTo2d((BSplineCurve3D)seqBS3d.Value(i)));
            }
        }
        return result;
    }

    /// <summary>
    /// OCCT.NET: what ShapeUpgrade's <c>OCC_CATCH_SIGNALS</c> blocks catch: a Standard_Failure, or a
    /// NullReferenceException where OCCT dereferences a null handle (an access violation natively, decision 977);
    /// never a write into a frozen object (Standard_ImmutableObject, TopoDS_LockedShape: decision 423).
    /// </summary>
    internal static bool IsCaughtFailure(Exception exception)
        => exception is not (ImmutableObjectException or LockedShapeException)
           && (Refusal.IsFailure(exception) || exception is NullReferenceException);
}
