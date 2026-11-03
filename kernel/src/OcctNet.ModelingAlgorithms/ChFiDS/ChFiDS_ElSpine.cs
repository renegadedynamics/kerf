// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ElSpine.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ElSpine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// An elementary spine: the smooth guide curve (C2) of one part of a spine, which the walking and the
/// approximation of a fillet follow, with its end points and tangents, its vertices with tangents and the fillet
/// surfaces before and after it.
/// </summary>
/// <remarks>
/// A working object of the fillet builders (decision 21): they set its curve, bounds and neighbours through the
/// internal mutators. As an <see cref="Adaptor3d_Curve"/> it evaluates its curve between its own bounds.
/// </remarks>
public sealed class ChFiDS_ElSpine : Adaptor3d_Curve
{
    private GeomAdaptor_Curve curve = new();
    private Point3D ptfirst = new();
    private Point3D ptlast = new();
    private Vector3D tgfirst = new();
    private Vector3D tglast = new();
    private NCollection_Sequence<Axis3D> VerticesWithTangents = new();
    private ChFiDS_SurfData? previous;
    private ChFiDS_SurfData? next;
    private double pfirst;
    private double plast;
    private double period;
    private bool periodic;
    private double pfirstsav;
    private double plastsav;

    public ChFiDS_ElSpine()
    {
        pfirst = 0.0;
        plast = 0.0;
        period = 0.0;
        periodic = false;
        pfirstsav = global::Kerf.Engine.Foundation.Precision.Infinite();
        plastsav = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>Shallow copy of the adaptor.</summary>
    public override Adaptor3d_Curve ShallowCopy()
    {
        ChFiDS_ElSpine aCopy = new();

        Adaptor3d_Curve aCurve = curve.ShallowCopy();
        aCopy.curve = (GeomAdaptor_Curve)aCurve;

        aCopy.ptfirst = ptfirst;
        aCopy.ptlast = ptlast;
        aCopy.tgfirst = tgfirst;
        aCopy.tglast = tglast;
        aCopy.VerticesWithTangents = new NCollection_Sequence<Axis3D>(VerticesWithTangents);
        aCopy.previous = previous;
        aCopy.next = next;
        aCopy.pfirst = pfirst;
        aCopy.plast = plast;
        aCopy.period = period;
        aCopy.periodic = periodic;
        aCopy.pfirstsav = pfirstsav;
        aCopy.plastsav = plastsav;

        return aCopy;
    }

    public override double FirstParameter()
    {
        return pfirst;
    }

    public override double LastParameter()
    {
        return plast;
    }

    public double GetSavedFirstParameter()
    {
        return pfirstsav;
    }

    public double GetSavedLastParameter()
    {
        return plastsav;
    }

    public override Continuity Continuity()
    {
        return curve.Continuity();
    }

    public override int NbIntervals(Continuity S)
    {
        return curve.NbIntervals(S);
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        curve.Intervals(T, S);
    }

    /// <summary>The curve between <paramref name="First"/> and <paramref name="Last"/>.</summary>
    public override Adaptor3d_Curve Trim(double First, double Last, double Tol)
    {
        return curve.Trim(First, Last, Tol);
    }

    public override double Resolution(double R3d)
    {
        return curve.Resolution(R3d);
    }

    public override CurveKind GetType()
    {
        return curve.GetType();
    }

    public override bool IsPeriodic()
    {
        return periodic;
    }

    internal void SetPeriodic(bool I)
    {
        periodic = I;
        period = plast - pfirst;
    }

    public override double Period()
    {
        if (!periodic)
        {
            throw new FailureException("ElSpine non periodique");
        }

        return period;
    }

    public override Point3D EvalD0(double theAbsC)
    {
        return curve.EvalD0(theAbsC);
    }

    public override Geom_Curve.ResD1 EvalD1(double theAbsC)
    {
        return curve.EvalD1(theAbsC);
    }

    public override Geom_Curve.ResD2 EvalD2(double theAbsC)
    {
        return curve.EvalD2(theAbsC);
    }

    public override Geom_Curve.ResD3 EvalD3(double theAbsC)
    {
        return curve.EvalD3(theAbsC);
    }

    internal void FirstParameter(double P)
    {
        pfirst = P;
    }

    internal void LastParameter(double P)
    {
        plast = P;
    }

    internal void SaveFirstParameter()
    {
        pfirstsav = pfirst;
    }

    internal void SaveLastParameter()
    {
        plastsav = plast;
    }

    /// <summary>
    /// Moves the origin of the periodic B-spline curve to <paramref name="O"/>. OCCT edits the curve in place: the
    /// builders' own approximated curve, never a shape's (a frozen curve throws).
    /// </summary>
    internal void SetOrigin(double O)
    {
        if (!periodic)
        {
            throw new FailureException("Elspine non periodique");
        }

        Geom_BSplineCurve? bs = curve.Curve() as Geom_BSplineCurve;
        if (bs is not null)
        {
            bs.SetOrigin(O, global::Kerf.Engine.Foundation.Precision.PConfusion());
            curve.Load(bs);
        }
    }

    internal void SetFirstPointAndTgt(in Point3D P, in Vector3D T)
    {
        ptfirst = P;
        tgfirst = T;
    }

    internal void SetLastPointAndTgt(in Point3D P, in Vector3D T)
    {
        ptlast = P;
        tglast = T;
    }

    internal void AddVertexWithTangent(in Axis3D anAx1)
    {
        VerticesWithTangents.Append(anAx1);
    }

    public void FirstPointAndTgt(out Point3D P, out Vector3D T)
    {
        P = ptfirst;
        T = tgfirst;
    }

    public void LastPointAndTgt(out Point3D P, out Vector3D T)
    {
        P = ptlast;
        T = tglast;
    }

    public int NbVertices()
    {
        return VerticesWithTangents.Length();
    }

    public Axis3D VertexWithTangent(int Index)
    {
        return VerticesWithTangents[Index];
    }

    internal void SetCurve(Geom_Curve C)
    {
        curve.Load(C);
    }

    public ChFiDS_SurfData? Previous()
    {
        return previous;
    }

    internal ref ChFiDS_SurfData? ChangePrevious() => ref previous;

    public ChFiDS_SurfData? Next()
    {
        return next;
    }

    internal ref ChFiDS_SurfData? ChangeNext() => ref next;

    public override Line3D Line()
    {
        return curve.Line();
    }

    public override Circle3D Circle()
    {
        return curve.Circle();
    }

    public override Ellipse3D Ellipse()
    {
        return curve.Ellipse();
    }

    public override Hyperbola3D Hyperbola()
    {
        return curve.Hyperbola();
    }

    public override Parabola3D Parabola()
    {
        return curve.Parabola();
    }

    public override Geom_BezierCurve Bezier()
    {
        return curve.Bezier();
    }

    public override Geom_BSplineCurve BSpline()
    {
        return curve.BSpline();
    }
}
