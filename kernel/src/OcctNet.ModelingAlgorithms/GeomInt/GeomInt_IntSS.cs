// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The intersection of two Geom surfaces: IntPatch_Intersection's lines made into Geom curves (conics trimmed to
/// the domains, walking lines approximated by B-splines, restriction lines approximated on their surface), with
/// their 2D curves on either surface on request.
/// </summary>
/// <remarks>
/// <para>
/// A working object, as OCCT's: each <c>Perform</c> computes a new result. The curves <see cref="Line"/>,
/// <see cref="LineOnS1"/>, <see cref="LineOnS2"/> and <see cref="Boundary"/> return are frozen (design 4.2): they
/// are the algorithm's results handed to the caller. The static helpers (<see cref="BuildPCurves(double, double, ref
/// double, Surface, Curve3D, ref Curve2D?)"/>, <see cref="MakeBSpline"/>, <see cref="TreatRLine"/>, ...)
/// return new, unfrozen curves, as IntTools_FaceFace expects.
/// </para>
/// <para>
/// The curve building (OCCT's GeomInt_IntSS_1.cxx) is in GeomInt_IntSS.MakeCurve.cs.
/// </para>
/// </remarks>
public partial class GeomInt_IntSS
{
    private readonly IntPatch_Intersection myIntersector = new();
    private readonly GeomInt_LineConstructor myLConstruct = new();
    private SurfaceAdaptor? myHS1;
    private SurfaceAdaptor? myHS2;
    private int myNbrestr;
    private readonly NCollection_Sequence<Curve3D> sline = new();
    private readonly NCollection_Sequence<Curve2D?> slineS1 = new();
    private readonly NCollection_Sequence<Curve2D?> slineS2 = new();
    private double myTolReached2d;
    private double myTolReached3d;
    private double myTolCheck;
    private double myTolAngCheck;

    public GeomInt_IntSS()
    {
        myNbrestr = 0;
        myTolReached2d = 0.0;
        myTolReached3d = 0.0;
        myTolCheck = 1.0e-7;
        myTolAngCheck = 1.0e-6;
    }

    /// <summary>Intersects S1 and S2 with the tolerance Tol (see <see cref="Perform(Surface, Surface,
    /// double, bool, bool, bool)"/>).</summary>
    public GeomInt_IntSS(Surface S1, Surface S2, double Tol, bool Approx = true, bool ApproxS1 = false,
                         bool ApproxS2 = false)
        : this()
    {
        Perform(S1, S2, Tol, Approx, ApproxS1, ApproxS2);
    }

    /// <summary>
    /// Intersects S1 and S2 with the tolerance Tol. Approx approximates the walking lines (otherwise they are
    /// polylines); ApproxS1 and ApproxS2 compute the 2D curves on S1 and S2.
    /// </summary>
    public void Perform(Surface S1, Surface S2, double Tol, bool Approx = true, bool ApproxS1 = false,
                        bool ApproxS2 = false)
    {
        myHS1 = new SurfaceAdaptor(S1);
        if (S1 == S2)
        {
            myHS2 = myHS1;
        }
        else
        {
            myHS2 = new SurfaceAdaptor(S2);
        }
        InternalPerform(Tol, Approx, ApproxS1, ApproxS2, false, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>Intersects two surface adaptors (see <see cref="Perform(Surface, Surface, double, bool,
    /// bool, bool)"/>). The adaptors are kept, as OCCT keeps the handles.</summary>
    public void Perform(SurfaceAdaptor HS1, SurfaceAdaptor HS2, double Tol, bool Approx = true,
                        bool ApproxS1 = false, bool ApproxS2 = false)
    {
        myHS1 = HS1;
        myHS2 = HS2;
        InternalPerform(Tol, Approx, ApproxS1, ApproxS2, false, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>Intersects S1 and S2 from the starting point (U1, V1) on S1 and (U2, V2) on S2.</summary>
    public void Perform(Surface S1, Surface S2, double Tol, double U1, double V1, double U2, double V2,
                        bool Approx = true, bool ApproxS1 = false, bool ApproxS2 = false)
    {
        myHS1 = new SurfaceAdaptor(S1);
        if (S1 == S2)
        {
            myHS2 = myHS1;
        }
        else
        {
            myHS2 = new SurfaceAdaptor(S2);
        }
        InternalPerform(Tol, Approx, ApproxS1, ApproxS2, true, U1, V1, U2, V2);
    }

    /// <summary>Intersects two surface adaptors from a starting point.</summary>
    public void Perform(SurfaceAdaptor HS1, SurfaceAdaptor HS2, double Tol, double U1, double V1, double U2,
                        double V2, bool Approx = true, bool ApproxS1 = false, bool ApproxS2 = false)
    {
        myHS1 = HS1;
        myHS2 = HS2;
        InternalPerform(Tol, Approx, ApproxS1, ApproxS2, true, U1, V1, U2, V2);
    }

    /// <summary>True if the intersection was computed.</summary>
    public bool IsDone() => myIntersector.IsDone();

    /// <summary>The largest tolerance reached by the 3D curves.</summary>
    public double TolReached3d() => myTolReached3d;

    /// <summary>The largest tolerance reached by the 2D curves.</summary>
    public double TolReached2d() => myTolReached2d;

    /// <summary>The number of intersection lines.</summary>
    public int NbLines() => sline.Length() - myNbrestr;

    /// <summary>The intersection line of index Index, frozen. Throws StdFail_NotDone if not done.</summary>
    public Curve3D Line(int Index)
    {
        StdFail_NotDone_Raise_if(!myIntersector.IsDone(), "GeomInt_IntSS::Line");
        Curve3D aC = sline.Value(Index + myNbrestr);
        aC.Freeze();
        return aC;
    }

    /// <summary>True if the line of index Index has a 2D curve on the first surface. Throws StdFail_NotDone if not
    /// done.</summary>
    public bool HasLineOnS1(int index)
    {
        StdFail_NotDone_Raise_if(!myIntersector.IsDone(), "GeomInt_IntSS::HasLineOnS1");
        return (slineS1.Value(index) is not null);
    }

    /// <summary>The 2D curve on the first surface of the line of index Index, frozen, or null. Throws
    /// StdFail_NotDone if not done.</summary>
    public Curve2D? LineOnS1(int Index)
    {
        StdFail_NotDone_Raise_if(!myIntersector.IsDone(), "GeomInt_IntSS::LineOnS1");
        Curve2D? aC = slineS1.Value(Index);
        aC?.Freeze();
        return aC;
    }

    /// <summary>True if the line of index Index has a 2D curve on the second surface. Throws StdFail_NotDone if not
    /// done.</summary>
    public bool HasLineOnS2(int index)
    {
        StdFail_NotDone_Raise_if(!myIntersector.IsDone(), "GeomInt_IntSS::HasLineOnS2");
        return (slineS2.Value(index) is not null);
    }

    /// <summary>The 2D curve on the second surface of the line of index Index, frozen, or null. Throws
    /// StdFail_NotDone if not done.</summary>
    public Curve2D? LineOnS2(int Index)
    {
        StdFail_NotDone_Raise_if(!myIntersector.IsDone(), "GeomInt_IntSS::LineOnS2");
        Curve2D? aC = slineS2.Value(Index);
        aC?.Freeze();
        return aC;
    }

    /// <summary>The number of boundary lines (always 0: OCCT never records any). Throws StdFail_NotDone if not
    /// done.</summary>
    public int NbBoundaries()
    {
        StdFail_NotDone_Raise_if(!myIntersector.IsDone(), "GeomInt_IntSS::NbBoundaries() - no result");
        return myNbrestr;
    }

    /// <summary>The boundary line of index Index, frozen. Throws StdFail_NotDone if not done, Standard_OutOfRange if
    /// Index is out of range.</summary>
    public Curve3D Boundary(int Index)
    {
        StdFail_NotDone_Raise_if(!myIntersector.IsDone(), "GeomInt_IntSS::Line");
        Standard_OutOfRange_Raise_if(Index <= 0 || Index > myNbrestr, "GeomInt_IntSS::Boundary");
        Curve3D aC = sline.Value(Index);
        aC.Freeze();
        return aC;
    }

    /// <summary>The number of isolated intersection points.</summary>
    public int NbPoints() => myIntersector.NbPnts();

    /// <summary>The isolated intersection point of index Index.</summary>
    public Point3D Point(int Index) => myIntersector.Point(Index).Value();

    /// <summary>The parameters on the first (OnFirst) or second surface of the point of index Index.</summary>
    public Point2D Pnt2d(int Index, bool OnFirst)
    {
        IntPatch_Point thept = myIntersector.Point(Index);
        double U, V;
        if (OnFirst)
        {
            thept.ParametersOnS1(out U, out V);
        }
        else
        {
            thept.ParametersOnS2(out U, out V);
        }
        return new Point2D(U, V);
    }

    /// <summary>Sets the tolerances of GeomLib_CheckBSplineCurve's tangent fixes of the approximated curves.</summary>
    public void SetTolFixTangents(double aTolCheck, double aTolAngCheck)
    {
        myTolCheck = aTolCheck;
        myTolAngCheck = aTolAngCheck;
    }

    /// <summary>The tolerances of the tangent fixes.</summary>
    public void TolFixTangents(out double aTolCheck, out double aTolAngCheck)
    {
        aTolCheck = myTolCheck;
        aTolAngCheck = myTolAngCheck;
    }

    protected void InternalPerform(double Tol, bool Approx, bool ApproxS1, bool ApproxS2, bool useStart, double U1,
                                   double V1, double U2, double V2)
    {
        myTolReached2d = myTolReached3d = 0.0;
        myNbrestr = 0;
        sline.Clear();
        // OCCT: slineS1 and slineS2 aren't cleared, so after a second Perform LineOnS1/S2(i) and HasLineOnS1/S2(i)
        // read the first Perform's 2D curves (the index isn't shifted as Line's is).
        double TolArc = Tol;
        double TolTang = Tol;
        double Deflection = 0.1;
        if (myHS1!.GetType() == SurfaceKind.BSplineSurface && myHS2!.GetType() == SurfaceKind.BSplineSurface)
        {
            Deflection /= 10.0;
        }

        TopolTool dom1 = new(myHS1);
        TopolTool dom2 = new(myHS2!);

        NCollection_DynamicArray<SurfaceAdaptorBase> aVecHS1 = new();
        NCollection_DynamicArray<SurfaceAdaptorBase> aVecHS2 = new();

        if (myHS1 == myHS2)
        {
            aVecHS1.Append(myHS1);
            aVecHS2.Append(myHS2);
        }
        else
        {
            IntPatch_Intersection.PrepareSurfaces(myHS1, dom1, myHS2!, dom2, Tol, aVecHS1, aVecHS2);
        }

        for (int aNumOfHS1 = 0; aNumOfHS1 < aVecHS1.Length(); aNumOfHS1++)
        {
            SurfaceAdaptorBase aHS1 = aVecHS1.Value(aNumOfHS1);
            for (int aNumOfHS2 = 0; aNumOfHS2 < aVecHS2.Length(); aNumOfHS2++)
            {
                SurfaceAdaptorBase aHS2 = aVecHS2.Value(aNumOfHS2);

                TopolTool aDom1 = new(aHS1);
                TopolTool aDom2 = new(aHS2);

                myLConstruct.Load(aDom1, aDom2, aHS1 as SurfaceAdaptor, aHS2 as SurfaceAdaptor);

                double UVMaxStep = IntPatch_Intersection.DefineUVMaxStep(aHS1, aDom1, aHS2, aDom2);

                myIntersector.SetTolerances(TolArc, TolTang, UVMaxStep, Deflection);

                if (aHS1 == aHS2)
                {
                    myIntersector.Perform(aHS1, aDom1, TolArc, TolTang);
                }
                else if (!useStart)
                {
                    myIntersector.Perform(aHS1, aDom1, aHS2, aDom2, TolArc, TolTang);
                }
                else
                {
                    Containment aState1 = aDom1.Classify(new Point2D(U1, V1), Tol);
                    Containment aState2 = aDom2.Classify(new Point2D(U2, V2), Tol);

                    if ((aState1 == In || aState1 == On) && (aState2 == In || aState2 == On))
                    {
                        myIntersector.Perform(aHS1, aDom1, aHS2, aDom2, U1, V1, U2, V2, TolArc, TolTang);
                    }
                }

                // ============================================================
                if (myIntersector.IsDone())
                {
                    int nblin = myIntersector.NbLines();
                    for (int i = 1; i <= nblin; i++)
                    {
                        MakeCurve(i, aDom1, aDom2, Tol, Approx, ApproxS1, ApproxS2);
                    }
                }
            }
        }
    }
}
