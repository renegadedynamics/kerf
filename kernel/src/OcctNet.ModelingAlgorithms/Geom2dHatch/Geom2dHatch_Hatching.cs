// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatching.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatching.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.HatchGen_ErrorStatus;

namespace Occt;

/// <summary>
/// A hatching of a 2D hatcher: its curve, its sorted intersection points with the elements, its domains, and the
/// flags and status of their computation.
/// </summary>
/// <remarks>
/// A working object (decisions 21 and 5804): a class, since the hatcher changes it in place, with OCCT's public API.
/// The constructor keeps a shallow copy of the caller's adaptor, as OCCT copies it.
/// </remarks>
public sealed class Geom2dHatch_Hatching
{
    private readonly Geom2dAdaptor_Curve myCurve;
    private bool myTrimDone;
    private bool myTrimFailed;
    private readonly NCollection_Sequence<HatchGen_PointOnHatching> myPoints = new();
    private bool myIsDone;
    private HatchGen_ErrorStatus myStatus;
    private readonly NCollection_Sequence<HatchGen_Domain> myDomains = new();

    /// <summary>A hatching on an empty curve.</summary>
    public Geom2dHatch_Hatching()
    {
        myCurve = new Geom2dAdaptor_Curve();
        myTrimDone = false;
        myTrimFailed = false;
        myIsDone = false;
        myStatus = HatchGen_NoProblem;
    }

    /// <summary>A hatching on (a copy of) <paramref name="Curve"/>.</summary>
    public Geom2dHatch_Hatching(Geom2dAdaptor_Curve Curve)
    {
        myCurve = (Geom2dAdaptor_Curve)Curve.ShallowCopy();
        myTrimDone = false;
        myTrimFailed = false;
        myIsDone = false;
        myStatus = HatchGen_NoProblem;
    }

    /// <summary>The curve of the hatching.</summary>
    public Geom2dAdaptor_Curve Curve() => myCurve;

    /// <summary>The curve of the hatching (OCCT's non-const reference: the same object).</summary>
    public Geom2dAdaptor_Curve ChangeCurve() => myCurve;

    /// <summary>Sets the flag that the trimmings were computed.</summary>
    public void TrimDone(bool Flag)
    {
        myTrimDone = Flag;
    }

    /// <summary>True if the trimmings were computed.</summary>
    public bool TrimDone() => myTrimDone;

    /// <summary>Sets the flag that a trimming failed (and then the status TrimFailure).</summary>
    public void TrimFailed(bool Flag)
    {
        myTrimFailed = Flag;
        if (myTrimFailed)
        {
            myStatus = HatchGen_TrimFailure;
        }
    }

    /// <summary>True if a trimming failed.</summary>
    public bool TrimFailed() => myTrimFailed;

    /// <summary>Sets the flag that the domains were computed.</summary>
    public void IsDone(bool Flag)
    {
        myIsDone = Flag;
    }

    /// <summary>True if the domains were computed.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>Sets the error status.</summary>
    public void Status(HatchGen_ErrorStatus theStatus)
    {
        myStatus = theStatus;
    }

    /// <summary>The error status.</summary>
    public HatchGen_ErrorStatus Status() => myStatus;

    /// <summary>
    /// Adds an intersection point, in parameter order; a point within <paramref name="Confusion"/> of an existing
    /// one gives that one its points on element instead.
    /// </summary>
    public void AddPoint(HatchGen_PointOnHatching Point, double Confusion)
    {
        int NbPoints = myPoints.Length();
        // for (int IPntH = 1 ; IPntH <= NbPoints ; IPntH++) {
        int IPntH;
        for (IPntH = 1; IPntH <= NbPoints; IPntH++)
        {
            HatchGen_PointOnHatching PntH = myPoints.Value(IPntH);
            if (!PntH.IsLower(Point, Confusion))
            {
                break;
            }
        }
        if (IPntH > NbPoints)
        {
            myPoints.Append(new HatchGen_PointOnHatching(Point));
        }
        else
        {
            HatchGen_PointOnHatching PntH = myPoints.ChangeValue(IPntH);
            if (PntH.IsGreater(Point, Confusion))
            {
                myPoints.InsertBefore(IPntH, new HatchGen_PointOnHatching(Point));
            }
            else
            {
                for (int IPntE = 1; IPntE <= Point.NbPoints(); IPntE++)
                {
                    HatchGen_PointOnElement PntE = Point.Point(IPntE);
                    PntH.AddPoint(PntE, Confusion);
                }
            }
        }
        if (myIsDone)
        {
            ClrDomains();
        }
    }

    /// <summary>The number of intersection points.</summary>
    public int NbPoints() => myPoints.Length();

    /// <summary>The <paramref name="Index"/>-th intersection point.</summary>
    /// <exception cref="Standard_OutOfRange"><paramref name="Index"/> is out of range.</exception>
    public HatchGen_PointOnHatching Point(int Index) => myPoints.Value(Index);

    /// <summary>The <paramref name="Index"/>-th intersection point, for writing (the same object).</summary>
    /// <exception cref="Standard_OutOfRange"><paramref name="Index"/> is out of range.</exception>
    public HatchGen_PointOnHatching ChangePoint(int Index) => myPoints.ChangeValue(Index);

    /// <summary>Removes the <paramref name="Index"/>-th intersection point (and the domains, if computed).</summary>
    public void RemPoint(int Index)
    {
        if (myIsDone)
        {
            ClrDomains();
        }
        myPoints.Remove(Index);
    }

    /// <summary>Removes all the intersection points (and the domains, if computed), and resets the trimming flags.</summary>
    public void ClrPoints()
    {
        if (myIsDone)
        {
            ClrDomains();
        }
        for (int IPntH = 1; IPntH <= myPoints.Length(); IPntH++)
        {
            HatchGen_PointOnHatching Point = myPoints.ChangeValue(IPntH);
            Point.ClrPoints();
        }
        myPoints.Clear();
        myTrimDone = false;
        myTrimFailed = false;
    }

    /// <summary>Adds a domain.</summary>
    public void AddDomain(in HatchGen_Domain Domain)
    {
        myDomains.Append(Domain);
    }

    /// <summary>The number of domains.</summary>
    public int NbDomains() => myDomains.Length();

    /// <summary>The <paramref name="Index"/>-th domain.</summary>
    /// <exception cref="Standard_OutOfRange"><paramref name="Index"/> is out of range.</exception>
    public HatchGen_Domain Domain(int Index) => myDomains.Value(Index);

    /// <summary>Removes the <paramref name="Index"/>-th domain.</summary>
    public void RemDomain(int Index)
    {
        myDomains.Remove(Index);
    }

    /// <summary>Removes all the domains; they are then not computed.</summary>
    public void ClrDomains()
    {
        myDomains.Clear();
        myIsDone = false;
    }

    /// <summary>
    /// A point of the curve for the classification: at its last parameter, else its first, else 0.
    /// </summary>
    public gp_Pnt2d ClassificationPoint()
    {
        double t, a, b;
        a = myCurve.FirstParameter();
        b = myCurve.LastParameter();
        if (b >= Precision.Infinite())
        {
            if (a <= -Precision.Infinite())
            {
                t = 0;
            }
            else
            {
                t = a;
            }
        }
        else
        {
            t = b;
        }
        return (myCurve.Value(t));
    }
}
