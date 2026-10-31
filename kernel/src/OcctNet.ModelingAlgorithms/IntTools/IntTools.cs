// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.gce_ErrorType;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>The statics of the intersection and classification package: roots, edge lengths, curve parameters.</summary>
public static class IntTools
{
    /// <summary>
    /// The radius of the curve between <paramref name="t1"/> and <paramref name="t3"/>: the circle's radius, or the
    /// radius of the circle through the points at t1, (t1 + t3) / 2 and t3. Returns 0 if computed, 1 for a line, 2 for
    /// confused or colinear points, -1 for another failure.
    /// </summary>
    public static int GetRadius(BRepAdaptor_Curve C, double t1, double t3, out double aR)
    {
        aR = 0.0;
        GeomAbs_CurveType aType = C.GetType();
        if (aType == GeomAbs_Line)
        {
            return 1;
        }

        if (aType == GeomAbs_Circle)
        {
            gp_Circ aCrc = C.Circle();
            aR = aCrc.Radius();
            return 0;
        }

        double t2;
        gp_Pnt P1, P2, P3;

        t2 = 0.5 * (t1 + t3);

        P1 = C.Value(t1);
        P2 = C.Value(t2);
        P3 = C.Value(t3);
        //
        //
        gce_MakeCirc aMakeCirc = new(P1, P2, P3);
        gce_ErrorType anErrorType;

        anErrorType = aMakeCirc.Status();

        if (!aMakeCirc.IsDone())
        {
            if (anErrorType == gce_ConfusedPoints || anErrorType == gce_IntersectionError
                || anErrorType == gce_ColinearPoints)
            {
                return 2;
            }
            return -1;
        }
        //
        //
        gp_Circ aCirc = aMakeCirc.Value();
        aR = aCirc.Radius();

        return 0;
    }

    /// <summary>
    /// The sample parameters of the curve on [<paramref name="Tmin"/>, <paramref name="Tmax"/>]: <paramref
    /// name="Discret"/> equal steps, refined on lines, circles, parabolas and hyperbolas by a quasi-uniform deflection
    /// of <paramref name="Deflection"/> times the local radius. Returns 0 on success, 1 or 2 on failure.
    /// </summary>
    /// <remarks>
    /// <paramref name="anArgs"/> is resized to [0, NbArgs - 1]. OCCT's C is a non-const reference only because
    /// GCPnts takes one; the adaptor isn't changed.
    /// </remarks>
    public static int PrepareArgs(BRepAdaptor_Curve C, double Tmax, double Tmin, int Discret, double Deflection,
                                  NCollection_Array1<double> anArgs)
    {
        NCollection_List<double> aPars = new();
        double dt, tCurrent, tNext, aR, anAbsDeflection;
        int ip, i, j, aNbDeflectionPoints;
        bool aRFlag;

        GeomAbs_CurveType aCurveType;
        aCurveType = C.GetType();

        dt = (Tmax - Tmin) / Discret;
        aRFlag = dt > 1.0e-5;
        for (i = 1; i <= Discret; i++)
        {
            tCurrent = Tmin + (i - 1) * dt;
            aPars.Append(tCurrent);
            tNext = tCurrent + dt;
            if (i == Discret)
            {
                tNext = Tmax;
            }
            ///////////////////////////////////////////////////
            if (!aRFlag)
            {
                continue;
            }
            if (aCurveType == GeomAbs_BSplineCurve || aCurveType == GeomAbs_BezierCurve
                || aCurveType == GeomAbs_OffsetCurve || aCurveType == GeomAbs_Ellipse
                || aCurveType == GeomAbs_OtherCurve)
            {
                continue;
            }
            //
            ip = GetRadius(C, tCurrent, tNext, out aR);
            if (ip < 0)
            {
                return 1;
            }
            //
            if (ip == 0)
            {
                anAbsDeflection = Deflection * aR;
                GCPnts_QuasiUniformDeflection anUD = new();
                anUD.Initialize(C, anAbsDeflection, tCurrent, tNext);
                if (!anUD.IsDone())
                {
                    return 2;
                }

                aNbDeflectionPoints = anUD.NbPoints();
                if (aNbDeflectionPoints > 2)
                {
                    aNbDeflectionPoints--;
                    for (j = 2; j <= aNbDeflectionPoints; j++)
                    {
                        tCurrent = anUD.Parameter(j);
                        aPars.Append(tCurrent);
                    }
                }
            }
        }

        aPars.Append(Tmax);
        int aDiscretBis = aPars.Extent();
        anArgs.Resize(0, aDiscretBis - 1, false);
        i = 0;
        for (NCollection_List<double>.Iterator anIt = new(aPars); anIt.More(); anIt.Next(), i++)
        {
            anArgs.SetValue(i, anIt.Value());
        }
        return 0;
    }

    /// <summary>The length of the edge (0 for a degenerated or non-geometric edge).</summary>
    public static double Length(in TopoDS_Edge anEdge)
    {
        double aLength = 0;

        if (!BRep_Tool.Degenerated(anEdge) && BRep_Tool.IsGeometric(anEdge))
        {
            GProp_GProps Temp = new();
            BRepGProp.LinearProperties(anEdge, Temp);
            aLength = Temp.Mass();
        }
        return aLength;
    }

    /// <summary>Removes from the sequence the roots closer than <paramref name="anEpsT"/> / 2 to an earlier one.</summary>
    /// <remarks>
    /// OCCT: after removing the k-th root the loop goes on at k + 1, so the root moved into place k isn't compared with
    /// the j-th one: of three roots within the tolerance, the third stays (occt-issues.md, unit 6A).
    /// </remarks>
    public static void RemoveIdenticalRoots(NCollection_Sequence<IntTools_Root> aSR, double anEpsT)
    {
        int aNbRoots, j, k;
        double anEpsT2 = 0.5 * anEpsT;
        aNbRoots = aSR.Length();
        for (j = 1; j <= aNbRoots; j++)
        {
            IntTools_Root aRj = aSR.Value(j);
            for (k = j + 1; k <= aNbRoots; k++)
            {
                IntTools_Root aRk = aSR.Value(k);
                if (Math.Abs(aRj.Root() - aRk.Root()) < anEpsT2)
                {
                    aSR.Remove(k); // OCCT: k isn't decremented, the next root is skipped
                    aNbRoots = aSR.Length();
                }
            }
        }
    }

    // OCCT's IntTools_RootComparator: by the root's value only, so roots with equal values tie.
    private readonly struct IntTools_RootComparator : std_heap.ILess<IntTools_Root>
    {
        public bool Less(IntTools_Root theLeft, IntTools_Root theRight) => theLeft.Root() < theRight.Root();
    }

    /// <summary>
    /// Sorts the roots in increasing order of their values (<paramref name="myEpsT"/> is unused). Ties are ordered as
    /// MSVC's <c>std::sort</c> orders them (<see cref="std_sort"/>, decision 491): in their order for up to 32 roots.
    /// </summary>
    public static void SortRoots(NCollection_Sequence<IntTools_Root> mySequenceOfRoots, double myEpsT)
    {
        _ = myEpsT;
        int j, aNbRoots;

        aNbRoots = mySequenceOfRoots.Length();

        IntTools_Root[] anArray1OfRoots = new IntTools_Root[aNbRoots];

        for (j = 1; j <= aNbRoots; j++)
        {
            anArray1OfRoots[j - 1] = mySequenceOfRoots.Value(j);
        }

        std_sort.sort(anArray1OfRoots.AsSpan(), new IntTools_RootComparator());

        mySequenceOfRoots.Clear();
        for (j = 1; j <= aNbRoots; j++)
        {
            mySequenceOfRoots.Append(anArray1OfRoots[j - 1]);
        }
    }

    /// <summary>Sets the states before and after each root from its type and the function's values on its interval.</summary>
    public static void FindRootStates(NCollection_Sequence<IntTools_Root> mySequenceOfRoots, double myEpsNull)
    {
        int aType, j, aNbRoots;
        double t1, t2, f1, f2, absf2;

        aNbRoots = mySequenceOfRoots.Length();

        for (j = 1; j <= aNbRoots; j++)
        {
            ref IntTools_Root aR = ref mySequenceOfRoots.ChangeValue(j);

            aR.Interval(out t1, out t2, out f1, out f2);

            aType = aR.Type();
            switch (aType)
            {
                case 0: // Simple Root
                    if (f1 > 0.0 && f2 < 0.0)
                    {
                        aR.SetStateBefore(TopAbs_OUT);
                        aR.SetStateAfter(TopAbs_IN);
                    }
                    else
                    {
                        aR.SetStateBefore(TopAbs_IN);
                        aR.SetStateAfter(TopAbs_OUT);
                    }
                    break;

                case 1: // Complete 0;
                    aR.SetStateBefore(TopAbs_ON);
                    aR.SetStateAfter(TopAbs_ON);
                    break;

                case 2: // Smart;
                    absf2 = Math.Abs(f2);
                    if (absf2 < myEpsNull)
                    {
                        aR.SetStateAfter(TopAbs_ON);
                        if (f1 > 0.0)
                        {
                            aR.SetStateBefore(TopAbs_OUT);
                        }
                        else
                        {
                            aR.SetStateBefore(TopAbs_IN);
                        }
                    }
                    else
                    {
                        aR.SetStateBefore(TopAbs_ON);
                        if (f2 > 0.0)
                        {
                            aR.SetStateAfter(TopAbs_OUT);
                        }
                        else
                        {
                            aR.SetStateAfter(TopAbs_IN);
                        }
                    }
                    break; // OCCT falls through to "default: break;", the same

                default:
                    break;
            }
        }
    }

    /// <summary>
    /// The parameter of the point on the curve: exact on conics, by projection on Bézier and B-spline curves. Returns
    /// 0 if computed, 2 if the projection found nothing, 1 for another curve type (the parameter left as it was).
    /// </summary>
    public static int Parameter(in gp_Pnt aP, Geom_Curve aCurve, ref double aParameter)
    {
        double aFirst, aLast;
        GeomAbs_CurveType aCurveType;

        aFirst = aCurve.FirstParameter();
        aLast = aCurve.LastParameter();

        GeomAdaptor_Curve aGAC = new();
        aGAC.Load(aCurve, aFirst, aLast);
        aCurveType = aGAC.GetType();

        switch (aCurveType)
        {
            case GeomAbs_Line:
            {
                gp_Lin aLin = aGAC.Line();
                aParameter = ElCLib.Parameter(aLin, aP);
                return 0;
            }
            case GeomAbs_Circle:
            {
                gp_Circ aCircle = aGAC.Circle();
                aParameter = ElCLib.Parameter(aCircle, aP);
                return 0;
            }
            case GeomAbs_Ellipse:
            {
                gp_Elips aElips = aGAC.Ellipse();
                aParameter = ElCLib.Parameter(aElips, aP);
                return 0;
            }
            case GeomAbs_Hyperbola:
            {
                gp_Hypr aHypr = aGAC.Hyperbola();
                aParameter = ElCLib.Parameter(aHypr, aP);
                return 0;
            }
            case GeomAbs_Parabola:
            {
                gp_Parab aParab = aGAC.Parabola();
                aParameter = ElCLib.Parameter(aParab, aP);
                return 0;
            }
            case GeomAbs_BezierCurve:
            case GeomAbs_BSplineCurve:
            {
                GeomAPI_ProjectPointOnCurve aProjector = new();

                aProjector.Init(aP, aCurve, aFirst, aLast);
                int aNbPoints = aProjector.NbPoints();
                if (aNbPoints != 0)
                {
                    aParameter = aProjector.LowerDistanceParameter();
                    return 0;
                }
                else
                {
                    return 2;
                }
            }
            default:
                break;
        }
        return 1;
    }
}
