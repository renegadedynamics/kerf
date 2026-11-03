// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Construction.Values.ErrorType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>The statics of the intersection and classification package: roots, edge lengths, curve parameters.</summary>
public static class IntTools
{
    /// <summary>
    /// The radius of the curve between <paramref name="t1"/> and <paramref name="t3"/>: the circle's radius, or the
    /// radius of the circle through the points at t1, (t1 + t3) / 2 and t3. Returns 0 if computed, 1 for a line, 2 for
    /// confused or colinear points, -1 for another failure.
    /// </summary>
    public static int GetRadius(EdgeCurveAdaptor c, double t1, double t3, out double aR)
    {
        aR = 0.0;
        CurveKind type = c.GetType();
        if (type == Line)
        {
            return 1;
        }

        if (type == Circle)
        {
            Circle3D crc = c.Circle();
            aR = crc.Radius();
            return 0;
        }

        double t2;
        Point3D p1, p2, p3;

        t2 = 0.5 * (t1 + t3);

        p1 = c.Value(t1);
        p2 = c.Value(t2);
        p3 = c.Value(t3);
        //
        //
        MakeCirc makeCirc = new(p1, p2, p3);
        ErrorType errorType;

        errorType = makeCirc.Status();

        if (!makeCirc.IsDone)
        {
            if (errorType == ConfusedPoints || errorType == IntersectionError
                || errorType == ColinearPoints)
            {
                return 2;
            }
            return -1;
        }
        //
        //
        Circle3D circ = makeCirc.Value();
        aR = circ.Radius();

        return 0;
    }

    /// <summary>
    /// The sample parameters of the curve on [<paramref name="tmin"/>, <paramref name="tmax"/>]: <paramref
    /// name="discret"/> equal steps, refined on lines, circles, parabolas and hyperbolas by a quasi-uniform deflection
    /// of <paramref name="deflection"/> times the local radius. Returns 0 on success, 1 or 2 on failure.
    /// </summary>
    /// <remarks>
    /// <paramref name="anArgs"/> is resized to [0, NbArgs - 1]. OCCT's C is a non-const reference only because
    /// GCPnts takes one; the adaptor isn't changed.
    /// </remarks>
    public static int PrepareArgs(EdgeCurveAdaptor c, double tmax, double tmin, int discret, double deflection,
                                  NCollection_Array1<double> anArgs)
    {
        NCollection_List<double> pars = new();
        double dt, tCurrent, tNext, r, absDeflection;
        int ip, i, j, nbDeflectionPoints;
        bool rFlag;

        CurveKind curveType;
        curveType = c.GetType();

        dt = (tmax - tmin) / discret;
        rFlag = dt > 1.0e-5;
        for (i = 1; i <= discret; i++)
        {
            tCurrent = tmin + (i - 1) * dt;
            pars.Append(tCurrent);
            tNext = tCurrent + dt;
            if (i == discret)
            {
                tNext = tmax;
            }
            ///////////////////////////////////////////////////
            if (!rFlag)
            {
                continue;
            }
            if (curveType == BSplineCurve || curveType == BezierCurve
                || curveType == OffsetCurve || curveType == Ellipse
                || curveType == OtherCurve)
            {
                continue;
            }
            //
            ip = GetRadius(c, tCurrent, tNext, out r);
            if (ip < 0)
            {
                return 1;
            }
            //
            if (ip == 0)
            {
                absDeflection = deflection * r;
                QuasiUniformDeflection ud = new();
                ud.Initialize(c, absDeflection, tCurrent, tNext);
                if (!ud.IsDone)
                {
                    return 2;
                }

                nbDeflectionPoints = ud.PointCount;
                if (nbDeflectionPoints > 2)
                {
                    nbDeflectionPoints--;
                    for (j = 2; j <= nbDeflectionPoints; j++)
                    {
                        tCurrent = ud.Parameter(j);
                        pars.Append(tCurrent);
                    }
                }
            }
        }

        pars.Append(tmax);
        int discretBis = pars.Extent();
        anArgs.Resize(0, discretBis - 1, false);
        i = 0;
        for (NCollection_List<double>.Iterator it = new(pars); it.More(); it.Next(), i++)
        {
            anArgs.SetValue(i, it.Value());
        }
        return 0;
    }

    /// <summary>The length of the edge (0 for a degenerated or non-geometric edge).</summary>
    public static double Length(in Edge anEdge)
    {
        double length = 0;

        if (!Tool.Degenerated(anEdge) && Tool.IsGeometric(anEdge))
        {
            Properties temp = new();
            MassPropertiesBuilder.LinearProperties(anEdge, temp);
            length = temp.Mass();
        }
        return length;
    }

    /// <summary>Removes from the sequence the roots closer than <paramref name="anEpsT"/> / 2 to an earlier one.</summary>
    /// <remarks>
    /// OCCT: after removing the k-th root the loop goes on at k + 1, so the root moved into place k isn't compared with
    /// the j-th one: of three roots within the tolerance, the third stays (occt-issues.md, unit 6A).
    /// </remarks>
    public static void RemoveIdenticalRoots(NCollection_Sequence<IntersectionRoot> aSR, double anEpsT)
    {
        int nbRoots, j, k;
        double epsT2 = 0.5 * anEpsT;
        nbRoots = aSR.Length();
        for (j = 1; j <= nbRoots; j++)
        {
            IntersectionRoot rj = aSR.Value(j);
            for (k = j + 1; k <= nbRoots; k++)
            {
                IntersectionRoot rk = aSR.Value(k);
                if (Math.Abs(rj.Root() - rk.Root()) < epsT2)
                {
                    aSR.Remove(k); // OCCT: k isn't decremented, the next root is skipped
                    nbRoots = aSR.Length();
                }
            }
        }
    }

    // OCCT's IntTools_RootComparator: by the root's value only, so roots with equal values tie.
    private readonly struct IntTools_RootComparator : Heap.ILess<IntersectionRoot>
    {
        public bool Less(IntersectionRoot left, IntersectionRoot right) => left.Root() < right.Root();
    }

    /// <summary>
    /// Sorts the roots in increasing order of their values (<paramref name="myEpsT"/> is unused). Ties are ordered as
    /// MSVC's <c>std::sort</c> orders them (<see cref="Sorting"/>, decision 491): in their order for up to 32 roots.
    /// </summary>
    public static void SortRoots(NCollection_Sequence<IntersectionRoot> mySequenceOfRoots, double myEpsT)
    {
        _ = myEpsT;
        int j, nbRoots;

        nbRoots = mySequenceOfRoots.Length();

        IntersectionRoot[] array1OfRoots = new IntersectionRoot[nbRoots];

        for (j = 1; j <= nbRoots; j++)
        {
            array1OfRoots[j - 1] = mySequenceOfRoots.Value(j);
        }

        Sorting.Sort(array1OfRoots.AsSpan(), new IntTools_RootComparator());

        mySequenceOfRoots.Clear();
        for (j = 1; j <= nbRoots; j++)
        {
            mySequenceOfRoots.Append(array1OfRoots[j - 1]);
        }
    }

    /// <summary>Sets the states before and after each root from its type and the function's values on its interval.</summary>
    public static void FindRootStates(NCollection_Sequence<IntersectionRoot> mySequenceOfRoots, double myEpsNull)
    {
        int type, j, nbRoots;
        double t1, t2, f1, f2, absf2;

        nbRoots = mySequenceOfRoots.Length();

        for (j = 1; j <= nbRoots; j++)
        {
            ref IntersectionRoot r = ref mySequenceOfRoots.ChangeValue(j);

            r.Interval(out t1, out t2, out f1, out f2);

            type = r.Type();
            switch (type)
            {
                case 0: // Simple Root
                    if (f1 > 0.0 && f2 < 0.0)
                    {
                        r.SetStateBefore(Out);
                        r.SetStateAfter(In);
                    }
                    else
                    {
                        r.SetStateBefore(In);
                        r.SetStateAfter(Out);
                    }
                    break;

                case 1: // Complete 0;
                    r.SetStateBefore(On);
                    r.SetStateAfter(On);
                    break;

                case 2: // Smart;
                    absf2 = Math.Abs(f2);
                    if (absf2 < myEpsNull)
                    {
                        r.SetStateAfter(On);
                        if (f1 > 0.0)
                        {
                            r.SetStateBefore(Out);
                        }
                        else
                        {
                            r.SetStateBefore(In);
                        }
                    }
                    else
                    {
                        r.SetStateBefore(On);
                        if (f2 > 0.0)
                        {
                            r.SetStateAfter(Out);
                        }
                        else
                        {
                            r.SetStateAfter(In);
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
    public static int Parameter(in Point3D aP, Curve3D aCurve, ref double aParameter)
    {
        double first, last;
        CurveKind curveType;

        first = aCurve.FirstParameter();
        last = aCurve.LastParameter();

        CurveAdaptor gac = new();
        gac.Load(aCurve, first, last);
        curveType = gac.GetType();

        switch (curveType)
        {
            case Line:
            {
                Line3D lin = gac.Line();
                aParameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, aP);
                return 0;
            }
            case Circle:
            {
                Circle3D circle = gac.Circle();
                aParameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(circle, aP);
                return 0;
            }
            case Ellipse:
            {
                Ellipse3D elips = gac.Ellipse();
                aParameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(elips, aP);
                return 0;
            }
            case Hyperbola:
            {
                Hyperbola3D hypr = gac.Hyperbola();
                aParameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(hypr, aP);
                return 0;
            }
            case Parabola:
            {
                Parabola3D parab = gac.Parabola();
                aParameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(parab, aP);
                return 0;
            }
            case BezierCurve:
            case BSplineCurve:
            {
                ProjectPointOnCurve projector = new();

                projector.Init(aP, aCurve, first, last);
                int nbPoints = projector.NbPoints();
                if (nbPoints != 0)
                {
                    aParameter = projector.LowerDistanceParameter();
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
