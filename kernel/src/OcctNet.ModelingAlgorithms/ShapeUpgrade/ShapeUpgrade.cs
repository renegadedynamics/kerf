// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// ShapeUpgrade's package functions: the conversion of a C0 B-spline curve into a sequence of C1 B-spline curves.
/// </summary>
/// <remarks>
/// OCCT's commented-out debug switch (SetDebug, Debug) is left out.
/// </remarks>
public static class ShapeUpgrade
{
    /// <summary>
    /// Converts a C0 B-spline curve into a sequence of C1 B-spline curves, splitting it at the knots whose
    /// multiplicity equals the degree: unlike GeomConvert's C0BSplineToArrayOfC1BSplineCurve it uses no tolerance and
    /// doesn't change the geometry. False (and <paramref name="seqBS"/> unchanged) if the curve is null or already C1.
    /// </summary>
    /// <remarks>
    /// OCCT makes the curve non-periodic in place before splitting it. The port does so only for a curve ported code
    /// owns; a frozen curve (any a caller holds) is left as it is and a non-periodic copy is split (decision 8401).
    /// </remarks>
    public static bool C0BSplineToSequenceOfC1BSplineCurve(Geom_BSplineCurve? BS,
                                                           ref NCollection_HSequence<Geom_BoundedCurve>? seqBS)
    {
        if (BS is null || (BS.IsCN(1)))
        {
            return false;
        }

        seqBS = new NCollection_HSequence<Geom_BoundedCurve>();
        // OCCT.NET: copy on write of a frozen curve (decision 8401)
        if (BS.IsFrozen())
        {
            BS = (Geom_BSplineCurve)BS.Copy();
        }
        BS.SetNotPeriodic(); // to have equation NbPoles = NbKnots with Multiplicities - degree - 1

        int deg = BS.Degree();
        NCollection_Array1<gp_Pnt> Poles = BS.InternalPoles();
        NCollection_Array1<double> Weights = BS.InternalWeightsArray();
        NCollection_Array1<double> Knots = BS.InternalKnots();
        NCollection_Array1<int> Mults = BS.InternalMultiplicities();
        NCollection_Array1<double> KnotSequence = BS.InternalKnotSequence();

        int StartKnotIndex, EndKnotIndex, j;

        StartKnotIndex = BS.FirstUKnotIndex();
        for (EndKnotIndex = StartKnotIndex + 1; EndKnotIndex <= BS.LastUKnotIndex(); EndKnotIndex++)
        {
            if ((Mults[EndKnotIndex] < deg) && (EndKnotIndex < BS.LastUKnotIndex()))
            {
                continue;
            }

            int StartFlatIndex = BSplCLib.FlatIndex(deg, StartKnotIndex, Mults, false);
            //    StartFlatIndex += Mults (StartKnotIndex) - 1;
            int EndFlatIndex = BSplCLib.FlatIndex(deg, EndKnotIndex, Mults, false);
            EndFlatIndex -= Mults[EndKnotIndex] - 1;

            var TempKnots = new NCollection_Array1<double>(1, Knots.Length());
            var TempMults = new NCollection_Array1<int>(1, Knots.Length());
            TempMults.Init(1);
            int TempKnotIndex = 1;
            TempKnots[TempKnotIndex] = KnotSequence[StartFlatIndex - deg];

            for (j = StartFlatIndex - deg + 1; j <= EndFlatIndex + deg; j++)
            {
                if (Math.Abs(KnotSequence[j] - KnotSequence[j - 1]) <= gp.Resolution())
                {
                    TempMults[TempKnotIndex]++;
                }
                else
                {
                    TempKnots[++TempKnotIndex] = KnotSequence[j];
                }
            }

            int TempStartIndex = 1, TempEndIndex = TempKnotIndex;
            if (TempMults[TempStartIndex] == 1)
            {
                TempMults[++TempStartIndex]++;
            }
            if (TempMults[TempEndIndex] == 1)
            {
                TempMults[--TempEndIndex]++;
            }

            int NewNbKnots = TempEndIndex - TempStartIndex + 1;
            var newMults = new NCollection_Array1<int>(1, NewNbKnots);
            var newKnots = new NCollection_Array1<double>(1, NewNbKnots);
            for (j = 1; j <= NewNbKnots; j++)
            {
                newMults[j] = TempMults[j + TempStartIndex - 1];
                newKnots[j] = TempKnots[j + TempStartIndex - 1];
            }

            int NewNbPoles = BSplCLib.NbPoles(deg, false, newMults);
            var newPoles = new NCollection_Array1<gp_Pnt>(1, NewNbPoles);
            var newWeights = new NCollection_Array1<double>(1, NewNbPoles);
            int PoleIndex = StartFlatIndex - deg; // Index of starting pole when splitting B-Spline is an index of starting knot
            for (j = 1; j <= NewNbPoles; j++)
            {
                newWeights[j] = Weights[j + PoleIndex - 1];
                newPoles[j] = Poles[j + PoleIndex - 1];
            }

            var newC = new Geom_BSplineCurve(newPoles, newWeights, newKnots, newMults, deg);
            seqBS.Append(newC);

            StartKnotIndex = EndKnotIndex;
        }
        return true;
    }

    private static Geom_BSplineCurve BSplineCurve2dTo3d(Geom2d_BSplineCurve BS)
    {
        int deg = BS.Degree();
        int NbPoles = BS.NbPoles();
        NCollection_Array1<gp_Pnt2d> Poles2d = BS.InternalPoles();
        NCollection_Array1<double> Weights = BS.InternalWeightsArray();
        NCollection_Array1<double> Knots = BS.InternalKnots();
        NCollection_Array1<int> Mults = BS.InternalMultiplicities();

        var Poles3d = new NCollection_Array1<gp_Pnt>(1, NbPoles);
        for (int i = 1; i <= NbPoles; i++)
        {
            Poles3d[i] = new gp_Pnt(Poles2d[i].X(), Poles2d[i].Y(), 0);
        }

        var BS3d = new Geom_BSplineCurve(Poles3d, Weights, Knots, Mults, deg, BS.IsPeriodic());
        return BS3d;
    }

    private static Geom2d_BSplineCurve BSplineCurve3dTo2d(Geom_BSplineCurve BS)
    {
        int deg = BS.Degree();
        int NbPoles = BS.NbPoles();
        NCollection_Array1<gp_Pnt> Poles3d = BS.InternalPoles();
        NCollection_Array1<double> Weights = BS.InternalWeightsArray();
        NCollection_Array1<double> Knots = BS.InternalKnots();
        NCollection_Array1<int> Mults = BS.InternalMultiplicities();

        var Poles2d = new NCollection_Array1<gp_Pnt2d>(1, NbPoles);
        for (int i = 1; i <= NbPoles; i++)
        {
            Poles2d[i] = new gp_Pnt2d(Poles3d[i].X(), Poles3d[i].Y());
        }

        var BS2d = new Geom2d_BSplineCurve(Poles2d, Weights, Knots, Mults, deg, BS.IsPeriodic());
        return BS2d;
    }

    /// <summary>
    /// The 2D form of <see cref="C0BSplineToSequenceOfC1BSplineCurve(Geom_BSplineCurve?, ref NCollection_HSequence{Geom_BoundedCurve}?)"/>,
    /// through a 3D copy of the curve (so the curve given is never changed).
    /// </summary>
    public static bool C0BSplineToSequenceOfC1BSplineCurve(Geom2d_BSplineCurve? BS,
                                                           ref NCollection_HSequence<Geom2d_BoundedCurve>? seqBS)
    {
        if (BS is null || (BS.IsCN(1)))
        {
            return false;
        }

        Geom_BSplineCurve BS3d = BSplineCurve2dTo3d(BS);
        NCollection_HSequence<Geom_BoundedCurve>? seqBS3d = null;
        bool result = C0BSplineToSequenceOfC1BSplineCurve(BS3d, ref seqBS3d);
        if (result)
        {
            seqBS = new NCollection_HSequence<Geom2d_BoundedCurve>();
            for (int i = 1; i <= seqBS3d!.Length(); i++)
            {
                seqBS.Append(BSplineCurve3dTo2d((Geom_BSplineCurve)seqBS3d.Value(i)));
            }
        }
        return result;
    }

    /// <summary>
    /// OCCT.NET: what ShapeUpgrade's <c>OCC_CATCH_SIGNALS</c> blocks catch: a Standard_Failure, or a
    /// NullReferenceException where OCCT dereferences a null handle (an access violation natively, decision 977);
    /// never a write into a frozen object (Standard_ImmutableObject, TopoDS_LockedShape: decision 423).
    /// </summary>
    internal static bool IsCaughtFailure(Exception theException)
        => theException is not (Standard_ImmutableObject or TopoDS_LockedShape)
           && theException is Standard_Failure or NullReferenceException;
}
