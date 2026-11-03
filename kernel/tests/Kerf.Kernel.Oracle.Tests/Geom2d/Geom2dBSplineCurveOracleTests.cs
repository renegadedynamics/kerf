// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Engine.Geometry.BSplKnotDistribution;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>Geom2d_BSplineCurve against native OCCT: evaluation, queries and every bound edit, on random curves.</summary>
public class Geom2dBSplineCurveOracleTests(ITestOutputHelper output)
{
    private static readonly int Cases = Tape.Cases(300, 70);

    /// <summary>Structure, then evaluation at a few parameters (and slightly outside the range).</summary>
    private static void CheckAll(Oracle theOracle, N.Geom2d_BSplineCurve theNative, BSplineCurve2D thePort, string theWhat)
    {
        Geom2dOracle.SameStructure(theOracle, theNative, thePort, theWhat);
        for (int k = 0; k < 3; k++)
        {
            double U = Geom2dOracle.Param(theOracle, thePort, k == 2 ? 0.2 : 0.0);
            Geom2dOracle.SameEval(theOracle, theNative, thePort, U, theWhat, 5);
        }
    }

    [Fact]
    public void Evaluation_and_queries()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            BSplineSpec aSpec = Geom2dOracle.RandomBSpline(anOracle);
            using var aNative = Geom2dOracle.ToNative(aSpec);
            BSplineCurve2D aPort = Geom2dOracle.ToPort(aSpec);
            CheckAll(anOracle, aNative, aPort, "BSpline");

            for (int n = 0; n <= 6; n++)
            {
                anOracle.Same(aNative.IsCN(n), aPort.IsCN(n), $"IsCN({n})");
            }
            anOracle.Same(aNative.IsG1(aPort.FirstParameter(), aPort.LastParameter(), 1e-6),
                          aPort.IsG1(aPort.FirstParameter(), aPort.LastParameter(), 1e-6), "IsG1");

            double aTol = anOracle.Real(1e-7, 1e-2);
            double aNRes = 0.0;
            aNative.Resolution(aTol, ref aNRes);
            aPort.Resolution(aTol, out double aRes);
            anOracle.Same(aNRes, aRes, "Resolution");

            // ReversedParameter, PeriodicNormalization
            double U = Geom2dOracle.Param(anOracle, aPort, 1.0);
            anOracle.Same(aNative.ReversedParameter(U), aPort.ReversedParameter(U), "ReversedParameter");
            double aNU = U, aPU = U;
            aNative.PeriodicNormalization(ref aNU);
            aPort.PeriodicNormalization(ref aPU);
            anOracle.Same(aNU, aPU, "PeriodicNormalization");

            // LocateU, on and off knots, with and without knot repetition
            for (int k = 0; k < 4; k++)
            {
                double aU = k == 0 ? aPort.Knot(anOracle.Int(1, aPort.KnotCount)) : Geom2dOracle.Param(anOracle, aPort, 0.3);
                bool aRep = anOracle.Bool();
                double aPTol = anOracle.Bool() ? 0.0 : 1e-9;
                int aNI1 = 0, aNI2 = 0;
                aNative.LocateU(aU, aPTol, ref aNI1, ref aNI2, aRep);
                aPort.LocateU(aU, aPTol, out int aI1, out int aI2, aRep);
                anOracle.Same(aNI1, aI1, "LocateU I1");
                anOracle.Same(aNI2, aI2, "LocateU I2");
            }

            // Local evaluation on a span between two consecutive knots
            int aK1 = anOracle.Int(aPort.FirstUKnotIndex(), aPort.LastUKnotIndex() - 1);
            double aLU = anOracle.Real(aPort.Knot(aK1), aPort.Knot(aK1 + 1));
            using (var aNLV = aNative.LocalValue(aLU, aK1, aK1 + 1))
            using (var aNP = new N.gp_Pnt2d())
            using (var aNV1 = new N.gp_Vec2d())
            using (var aNV2 = new N.gp_Vec2d())
            using (var aNV3 = new N.gp_Vec2d())
            {
                anOracle.Same(Oracle.FromNative(aNLV), aPort.LocalValue(aLU, aK1, aK1 + 1).XY(), "LocalValue");
                aNative.LocalD1(aLU, aK1, aK1 + 1, aNP, aNV1);
                aPort.LocalD1(aLU, aK1, aK1 + 1, out Point2D aP, out Vector2D aV1);
                anOracle.Same(Oracle.FromNative(aNP), aP.XY(), "LocalD1 P");
                anOracle.Same(Oracle.FromNative(aNV1), aV1.XY(), "LocalD1 V1");
                aNative.LocalD3(aLU, aK1, aK1 + 1, aNP, aNV1, aNV2, aNV3);
                aPort.LocalD3(aLU, aK1, aK1 + 1, out aP, out aV1, out Vector2D aV2, out Vector2D aV3);
                anOracle.Same(Oracle.FromNative(aNP), aP.XY(), "LocalD3 P");
                anOracle.Same(Oracle.FromNative(aNV1), aV1.XY(), "LocalD3 V1");
                anOracle.Same(Oracle.FromNative(aNV2), aV2.XY(), "LocalD3 V2");
                anOracle.Same(Oracle.FromNative(aNV3), aV3.XY(), "LocalD3 V3");
                using var aNDN = aNative.LocalDN(aLU, aK1, aK1 + 1, 2);
                anOracle.Same(Oracle.FromNative(aNDN), aPort.LocalDN(aLU, aK1, aK1 + 1, 2).XY(), "LocalDN");
            }

            // Reversed copy and transformed copy
            using (var aNRev = aNative.Reversed())
            using (var aNRevB = N.Geom2d_BSplineCurve.DownCast(aNRev))
            {
                CheckAll(anOracle, aNRevB!, (BSplineCurve2D)aPort.Reversed(), "Reversed");
            }
            var (aT, aNT) = Geom2dOracle.Trsf(anOracle);
            using (aNT)
            using (var aNTr = aNative.Transformed(aNT))
            using (var aNTrB = N.Geom2d_BSplineCurve.DownCast(aNTr))
            {
                CheckAll(anOracle, aNTrB!, (BSplineCurve2D)aPort.Transformed(aT), "Transformed");
            }
        }

        anOracle.Report("Geom2d_BSplineCurve evaluation and queries");
    }

    /// <summary>
    /// One random edit of the kind theKind, chosen from thePort's state: the same operation on each side, each
    /// returning its extra results (RemoveKnot's flag, MovePoint's indexes, ...). False if no valid edit of that
    /// kind exists for the curve.
    /// </summary>
    private static bool Edit(Oracle theOracle, int theKind, BSplineCurve2D thePort, out string theWhat,
                             out Func<N.Geom2d_BSplineCurve, double[]> theNativeEdit,
                             out Func<BSplineCurve2D, double[]> thePortEdit)
    {
        int aDeg = thePort.Degree();
        int aNbKnots = thePort.KnotCount;
        double aF = thePort.FirstParameter(), aL = thePort.LastParameter();
        theNativeEdit = _ => [];
        thePortEdit = _ => [];
        switch (theKind)
        {
            case 0:
            {
                theWhat = "InsertKnot";
                double U = theOracle.Real(aF, aL);
                int M = theOracle.Int(1, aDeg);
                theNativeEdit = c => { c.InsertKnot(U, M, 0.0); return []; };
                thePortEdit = c => { c.InsertKnot(U, M, 0.0); return []; };
                return true;
            }
            case 1:
            case 2:
            {
                // Geom2d_BSplineCurve::InsertKnots defaults to Add = false; both values are passed explicitly.
                bool anAdd = theKind == 2;
                theWhat = $"InsertKnots(Add = {anAdd})";
                // A new knot, and an existing interior knot whose multiplicity can still grow by one
                int anExisting = 0;
                for (int k = 2; k < aNbKnots; k++)
                {
                    if (thePort.Multiplicity(k) < aDeg)
                    {
                        anExisting = k;
                        break;
                    }
                }
                double aNew = theOracle.Real(aF, aL);
                int aNewMult = theOracle.Int(1, aDeg);
                double[] aKnots;
                int[] aMults;
                if (anExisting != 0)
                {
                    double anOld = thePort.Knot(anExisting);
                    if (Math.Abs(anOld - aNew) < 1e-3)
                    {
                        return false;
                    }
                    int anOldMult = anAdd ? 1 : thePort.Multiplicity(anExisting) + 1;
                    aKnots = anOld < aNew ? [anOld, aNew] : [aNew, anOld];
                    aMults = anOld < aNew ? [anOldMult, aNewMult] : [aNewMult, anOldMult];
                }
                else
                {
                    aKnots = [aNew];
                    aMults = [aNewMult];
                }
                double aTol = theOracle.Bool() ? 0.0 : 1e-9;
                theNativeEdit = c =>
                {
                    using var aNK = Oracle.ToNative(NCollection_Array1<double>.From(aKnots));
                    using var aNM = Oracle.ToNative(NCollection_Array1<int>.From(aMults));
                    c.InsertKnots(aNK, aNM, aTol, anAdd);
                    return [];
                };
                thePortEdit = c =>
                {
                    c.InsertKnots(NCollection_Array1<double>.From(aKnots), NCollection_Array1<int>.From(aMults), aTol, anAdd);
                    return [];
                };
                return true;
            }
            case 3:
            {
                theWhat = "RemoveKnot";
                int aFirst = thePort.FirstUKnotIndex(), aLast = thePort.LastUKnotIndex();
                if (aLast - aFirst < 2)
                {
                    return false;
                }
                int anIndex = theOracle.Int(aFirst + 1, aLast - 1);
                int M = theOracle.Int(0, thePort.Multiplicity(anIndex) - 1);
                double aTol = Math.Pow(10.0, theOracle.Real(-6, 1));
                theNativeEdit = c => [c.RemoveKnot(anIndex, M, aTol) ? 1 : 0];
                thePortEdit = c => [c.RemoveKnot(anIndex, M, aTol) ? 1 : 0];
                return true;
            }
            case 4:
            {
                theWhat = "IncreaseDegree";
                int aNewDeg = aDeg + theOracle.Int(1, 2);
                theNativeEdit = c => { c.IncreaseDegree(aNewDeg); return []; };
                thePortEdit = c => { c.IncreaseDegree(aNewDeg); return []; };
                return true;
            }
            case 5:
            {
                theWhat = "IncreaseMultiplicity";
                if (aNbKnots < 3)
                {
                    return false;
                }
                if (theOracle.Bool())
                {
                    int anIndex = theOracle.Int(2, aNbKnots - 1);
                    int M = theOracle.Int(thePort.Multiplicity(anIndex), aDeg);
                    theNativeEdit = c => { c.IncreaseMultiplicity(anIndex, M); return []; };
                    thePortEdit = c => { c.IncreaseMultiplicity(anIndex, M); return []; };
                }
                else
                {
                    int aMax = 0;
                    for (int k = 2; k < aNbKnots; k++)
                    {
                        aMax = Math.Max(aMax, thePort.Multiplicity(k));
                    }
                    int M = theOracle.Int(aMax, aDeg);
                    theNativeEdit = c => { c.IncreaseMultiplicity(2, aNbKnots - 1, M); return []; };
                    thePortEdit = c => { c.IncreaseMultiplicity(2, aNbKnots - 1, M); return []; };
                }
                return true;
            }
            case 6:
            {
                theWhat = "Segment";
                double U1 = theOracle.Real(aF, aL), U2 = theOracle.Real(aF, aL);
                if (Math.Abs(U1 - U2) < 1e-2 * (aL - aF))
                {
                    return false;
                }
                double aLo = Math.Min(U1, U2), aHi = Math.Max(U1, U2);
                theNativeEdit = c => { c.Segment(aLo, aHi, 1e-9); return []; };
                thePortEdit = c => { c.Segment(aLo, aHi, 1e-9); return []; };
                return true;
            }
            case 7:
            {
                if (thePort.IsPeriodic)
                {
                    if (theOracle.Bool())
                    {
                        theWhat = "SetOrigin";
                        int anIndex = theOracle.Int(thePort.FirstUKnotIndex(), thePort.LastUKnotIndex());
                        theNativeEdit = c => { c.SetOrigin(anIndex); return []; };
                        thePortEdit = c => { c.SetOrigin(anIndex); return []; };
                    }
                    else
                    {
                        theWhat = "SetNotPeriodic";
                        theNativeEdit = c => { c.SetNotPeriodic(); return []; };
                        thePortEdit = c => { c.SetNotPeriodic(); return []; };
                    }
                }
                else
                {
                    // OCCT requires a closed curve; take a clamped one and close it by moving its last pole (and
                    // weight) onto the first.
                    if (thePort.Multiplicity(1) != aDeg + 1 || thePort.Multiplicity(aNbKnots) != aDeg + 1)
                    {
                        theWhat = "";
                        return false;
                    }
                    theWhat = "SetPeriodic";
                    int aLastPole = thePort.PoleCount;
                    Point2D aFirstPole = thePort.Pole(1);
                    double aFirstWeight = thePort.Weight(1);
                    theNativeEdit = c =>
                    {
                        using var aNP = Oracle.ToNative(aFirstPole);
                        c.SetPole(aLastPole, aNP, aFirstWeight);
                        c.SetPeriodic();
                        return [];
                    };
                    thePortEdit = c =>
                    {
                        c.SetPole(aLastPole, aFirstPole, aFirstWeight);
                        c.SetPeriodic();
                        return [];
                    };
                }
                return true;
            }
            case 8:
            {
                theWhat = "Reverse";
                theNativeEdit = c => { c.Reverse(); return []; };
                thePortEdit = c => { c.Reverse(); return []; };
                return true;
            }
            case 9:
            {
                int anIndex = theOracle.Int(1, thePort.PoleCount);
                Point2D aP = new Point2D(theOracle.Real(-10, 10), theOracle.Real(-10, 10));
                double aW = theOracle.Real(0.5, 2.0);
                switch (theOracle.Int(0, 3))
                {
                    case 0:
                        theWhat = "SetPole";
                        theNativeEdit = c => { using var aNP = Oracle.ToNative(aP); c.SetPole(anIndex, aNP); return []; };
                        thePortEdit = c => { c.SetPole(anIndex, aP); return []; };
                        break;
                    case 1:
                        theWhat = "SetPole with weight";
                        theNativeEdit = c => { using var aNP = Oracle.ToNative(aP); c.SetPole(anIndex, aNP, aW); return []; };
                        thePortEdit = c => { c.SetPole(anIndex, aP, aW); return []; };
                        break;
                    case 2:
                        theWhat = "SetWeight";
                        theNativeEdit = c => { c.SetWeight(anIndex, aW); return []; };
                        thePortEdit = c => { c.SetWeight(anIndex, aW); return []; };
                        break;
                    default:
                        // May turn a rational curve non-rational again
                        theWhat = "SetWeight 1";
                        theNativeEdit = c => { c.SetWeight(anIndex, 1.0); return []; };
                        thePortEdit = c => { c.SetWeight(anIndex, 1.0); return []; };
                        break;
                }
                return true;
            }
            case 10:
            {
                BSplKnotDistribution aDist = thePort.KnotDistribution();
                if (aDist != Uniform && aDist != QuasiUniform)
                {
                    theWhat = "";
                    return false;
                }
                if (theOracle.Bool() || thePort.PoleCount <= 2 || aNbKnots <= 2)
                {
                    theWhat = "InsertPoleAfter";
                    int anIndex = theOracle.Int(0, thePort.PoleCount);
                    Point2D aP = new Point2D(theOracle.Real(-10, 10), theOracle.Real(-10, 10));
                    double aW = theOracle.Bool() ? 1.0 : theOracle.Real(0.5, 2.0);
                    theNativeEdit = c => { using var aNP = Oracle.ToNative(aP); c.InsertPoleAfter(anIndex, aNP, aW); return []; };
                    thePortEdit = c => { c.InsertPoleAfter(anIndex, aP, aW); return []; };
                }
                else
                {
                    theWhat = "RemovePole";
                    int anIndex = theOracle.Int(1, thePort.PoleCount);
                    theNativeEdit = c => { c.RemovePole(anIndex); return []; };
                    thePortEdit = c => { c.RemovePole(anIndex); return []; };
                }
                return true;
            }
            case 11:
            {
                theWhat = "MovePoint";
                double U = theOracle.Real(aF, aL);
                int I1 = theOracle.Int(1, thePort.PoleCount), I2 = theOracle.Int(1, thePort.PoleCount);
                int aLo = Math.Min(I1, I2), aHi = Math.Max(I1, I2);
                Point2D aP = new Point2D(theOracle.Real(-10, 10), theOracle.Real(-10, 10));
                theNativeEdit = c =>
                {
                    using var aNP = Oracle.ToNative(aP);
                    int aFirst = 0, aLast = 0;
                    c.MovePoint(U, aNP, aLo, aHi, ref aFirst, ref aLast);
                    return [aFirst, aLast];
                };
                thePortEdit = c =>
                {
                    c.MovePoint(U, aP, aLo, aHi, out int aFirst, out int aLast);
                    return [aFirst, aLast];
                };
                return true;
            }
            case 12:
            {
                theWhat = "MovePointAndTangent";
                double U = theOracle.Real(aF, aL);
                Point2D aP = thePort.Value(U).Translated(new Vector2D(theOracle.Real(-1, 1), theOracle.Real(-1, 1)));
                Vector2D aTan = new Vector2D(theOracle.Real(-10, 10), theOracle.Real(-10, 10));
                int aStart = theOracle.Int(-1, 1), anEnd = theOracle.Int(-1, 1);
                theNativeEdit = c =>
                {
                    using var aNP = Oracle.ToNative(aP);
                    using var aNTan = Oracle.ToNative(aTan);
                    int anErr = 0;
                    c.MovePointAndTangent(U, aNP, aNTan, 1e-7, aStart, anEnd, ref anErr);
                    return [anErr];
                };
                thePortEdit = c =>
                {
                    c.MovePointAndTangent(U, aP, aTan, 1e-7, aStart, anEnd, out int anErr);
                    return [anErr];
                };
                return true;
            }
            default:
            {
                theWhat = "Transform";
                // Rotations included: native and port sine and cosine agree on these inputs, and the edit chains
                // compare bit-identical with them (decision 175).
                var (aT, aNT) = Geom2dOracle.Trsf(theOracle);
                theNativeEdit = c => { using (aNT) { c.Transform(aNT); } return []; };
                thePortEdit = c => { c.Transform(aT); return []; };
                return true;
            }
        }
    }

    [Fact]
    public void Edits()
    {
        var anOracle = new Oracle(output);
        const int aKinds = 14;
        int[] aDone = new int[aKinds];
        int aBothThrew = 0, aPortOnlyThrew = 0;
        var anInconsistent = new Dictionary<string, int>();
        for (int i = 0; i < Cases * 2; i++)
        {
            int aKind = i % aKinds;
            // Uniform knots for the pole insertion and removal, which need them
            BSplineSpec aSpec = Geom2dOracle.RandomBSpline(anOracle, null, aKind == 10);
            using var aNative = Geom2dOracle.ToNative(aSpec);
            BSplineCurve2D aPort = Geom2dOracle.ToPort(aSpec);

            // Up to three successive edits, each compared. Some random edits are invalid for some curves (raising
            // the degree of an unclamped curve, for instance): then both sides must throw, and the curve is dropped.
            for (int e = 0; e < 3; e++)
            {
                int aThisKind = e == 0 ? aKind : anOracle.Int(0, aKinds - 1);
                if (!Edit(anOracle, aThisKind, aPort, out string aWhat, out var aNativeEdit, out var aPortEdit))
                {
                    break;
                }

                BSplineCurve2D aEdited = new BSplineCurve2D(aPort);
                double[]? aPortRes = null, aNativeRes = null;
                Exception? aPortExc = null, aNativeExc = null;
                try
                {
                    aPortRes = aPortEdit(aEdited);
                }
                catch (Exception anExc)
                {
                    // Standard_Failure, or IndexOutOfRangeException where native OCCT raises Standard_OutOfRange
                    aPortExc = anExc;
                }
                if (aPortExc is ConstructionErrorException && aThisKind == 12)
                {
                    // MovePointAndTangent's 2 x 2 system is singular (degree 1 mostly): OCCT's release build divides
                    // by the zero determinant (gp_Mat2d::Invert's check is compiled out) where the port throws.
                    aPortOnlyThrew++;
                    break;
                }
                try
                {
                    aNativeRes = aNativeEdit(aNative);
                }
                catch (N.OcctException anExc)
                {
                    aNativeExc = anExc;
                }
                Assert.True((aPortExc is null) == (aNativeExc is null),
                            $"{aWhat}: native {aNativeExc?.Message ?? "succeeded"}, port {aPortExc?.Message ?? "succeeded"}");
                if (aPortExc is not null)
                {
                    aBothThrew++;
                    break;
                }

                for (int k = 0; k < aPortRes!.Length; k++)
                {
                    anOracle.Same(aNativeRes![k], aPortRes[k], $"{aWhat} result {k}");
                }
                aPort = aEdited;
                aDone[aThisKind]++;
                CheckAll(anOracle, aNative, aPort, aWhat);

                // Some edits of unclamped curves leave a pole count that doesn't match the multiplicities, natively
                // as in the port (compared just above); later edits would then read past the arrays. Stop there.
                if (aPort.PoleCount != global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aPort.Degree(), aPort.IsPeriodic, aPort.InternalMultiplicities()))
                {
                    anInconsistent[aWhat] = anInconsistent.GetValueOrDefault(aWhat) + 1;
                    break;
                }
            }
        }

        output.WriteLine("Edits per kind (InsertKnot, InsertKnots(Add=false), InsertKnots(Add=true), RemoveKnot, "
                         + "IncreaseDegree, IncreaseMultiplicity, Segment, Set(Not)Periodic/SetOrigin, Reverse, "
                         + "SetPole/SetWeight, InsertPoleAfter/RemovePole, MovePoint, MovePointAndTangent, Transform): "
                         + string.Join(", ", aDone));
        output.WriteLine($"Invalid edits thrown by both sides: {aBothThrew}; singular MovePointAndTangent skipped: {aPortOnlyThrew}");
        output.WriteLine("Edits leaving inconsistent curves (both sides): "
                         + string.Join(", ", anInconsistent.Select(p => $"{p.Key} {p.Value}")));
        anOracle.Report("Geom2d_BSplineCurve edits");
    }
}
