// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The result of a curve/surface intersection: points and segments, and whether the curve is "parallel".</summary>
/// <remarks>
/// An abstract class (OCCT's constructor is protected), with OCCT's protected fields and methods. A point equal to
/// one already stored (parameters within 1e-8, same transition) is not appended. OCCT's <c>Dump</c> (to std::cout)
/// is <see cref="ToString"/>.
/// </remarks>
public abstract class IntCurveSurface_Intersection
{
    protected bool done;
    protected bool myIsParallel; // Curve is "parallel" surface
    private readonly NCollection_Sequence<IntCurveSurface_IntersectionPoint> lpnt = new();
    private readonly NCollection_Sequence<IntCurveSurface_IntersectionSegment> lseg = new();

    protected IntCurveSurface_Intersection()
    {
        done = false;
        myIsParallel = false;
    }

    private static bool PARAMEQUAL(double a, double b) => Math.Abs((a) - (b)) < (1e-8);

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => (done);

    /// <summary>True if the curve is on or parallel to the surface (detected in the analytic cases only).</summary>
    public bool IsParallel() => (myIsParallel);

    /// <summary>The number of intersection points.</summary>
    public int NbPoints()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return lpnt.Length();
    }

    /// <summary>The number of intersection segments.</summary>
    public int NbSegments()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return lseg.Length();
    }

    /// <summary>The point of range <paramref name="N"/>.</summary>
    public IntCurveSurface_IntersectionPoint Point(int N)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return lpnt.Value(N);
    }

    /// <summary>The segment of range <paramref name="N"/>.</summary>
    public IntCurveSurface_IntersectionSegment Segment(int N)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return lseg.Value(N);
    }

    /// <summary>Copies the results of <paramref name="Other"/>.</summary>
    protected void SetValues(IntCurveSurface_Intersection Other)
    {
        if (Other.done)
        {
            lseg.Clear();
            lpnt.Clear();
            int N = Other.lpnt.Length();
            int i;
            for (i = 1; i <= N; i++)
            {
                lpnt.Append(Other.lpnt.Value(i));
            }
            N = Other.lseg.Length();
            for (i = 1; i <= N; i++)
            {
                lseg.Append(Other.lseg.Value(i));
            }
            done = true;
            myIsParallel = Other.myIsParallel;
        }
        else
        {
            done = false;
            myIsParallel = false;
        }
    }

    /// <summary>Appends the points and segments of <paramref name="Other"/> (the parameters are unused, as in OCCT).</summary>
    protected void Append(IntCurveSurface_Intersection Other, double FirstParamOnCurve, double LastParamOnCurve)
    {
        _ = FirstParamOnCurve;
        _ = LastParamOnCurve;
        int i, ni;
        if (Other.done)
        {
            ni = Other.lpnt.Length();
            for (i = 1; i <= ni; i++)
            {
                Append(Other.Point(i));
            }
            ni = Other.lseg.Length();
            for (i = 1; i <= ni; i++)
            {
                Append(Other.Segment(i));
            }
        }
    }

    /// <summary>Appends a point, unless one with the same parameters and transition is already stored.</summary>
    protected void Append(in IntCurveSurface_IntersectionPoint OtherPoint)
    {
        int i, ni;
        ni = lpnt.Length();
        for (i = 1; i <= ni; i++)
        {
            OtherPoint.Values(out _, out double u, out double v, out double w,
                              out IntCurveSurface_TransitionOnCurve TrOnCurve);
            lpnt[i].Values(out _, out double anu, out double anv, out double anw,
                           out IntCurveSurface_TransitionOnCurve anTrOnCurve);
            if (PARAMEQUAL(u, anu))
            {
                if (PARAMEQUAL(v, anv))
                {
                    if (PARAMEQUAL(w, anw))
                    {
                        if (anTrOnCurve == TrOnCurve)
                        {
                            return;
                        }
                    }
                }
            }
        }
        lpnt.Append(OtherPoint);
    }

    protected void Append(in IntCurveSurface_IntersectionSegment OtherSegment) => lseg.Append(OtherSegment);

    /// <summary>Forgets the results.</summary>
    protected void ResetFields()
    {
        if (done)
        {
            lseg.Clear();
            lpnt.Clear();
            done = false;
            myIsParallel = false;
        }
    }

    public override string ToString()
    {
        if (!done)
        {
            return " Intersection NotDone";
        }
        var aText = new System.Text.StringBuilder();
        int i, ni;
        ni = lpnt.Length();
        for (i = 1; i <= ni; i++)
        {
            aText.AppendLine(Point(i).ToString());
        }
        ni = lseg.Length();
        for (i = 1; i <= ni; i++)
        {
            aText.AppendLine(Segment(i).ToString());
        }
        return aText.ToString();
    }
}
