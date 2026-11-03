// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Projector.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Projector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The base of the projectors of elementary curves on elementary surfaces: holds the 2D result (a line, a conic,
/// a Bezier or a B-spline curve) and its type.
/// </summary>
/// <remarks>A working object (decision 21): its setters are public, as in OCCT.</remarks>
public class ProjLib_Projector
{
    protected CurveKind myType;
    protected Line2D myLin = new();
    protected Circle2D myCirc = new();
    protected Ellipse2D myElips = new();
    protected Hyperbola2D myHypr = new();
    protected Parabola2D myParab = new();
    protected Geom2d_BSplineCurve? myBSpline;
    protected Geom2d_BezierCurve? myBezier;
    protected bool myIsPeriodic;
    protected bool isDone;

    /// <summary>Sets the type to BSplineCurve (OCCT's comment says OtherCurve).</summary>
    public ProjLib_Projector()
    {
        myIsPeriodic = false;
        isDone = false;
        myType = BSplineCurve; // OCCT: documented as OtherCurve
    }

    public bool IsDone() => isDone;

    /// <summary>Sets isDone = true.</summary>
    public void Done()
    {
        isDone = true;
    }

    /// <summary>The type of the projected curve. Hides <see cref="object.GetType"/> (decision 98).</summary>
    public new CurveKind GetType() => myType;

    public void SetType(CurveKind Type)
    {
        myType = Type;
    }

    public bool IsPeriodic() => myIsPeriodic;

    public void SetPeriodic()
    {
        myIsPeriodic = true;
    }

    /// <exception cref="NoSuchObjectException">The type is not Line.</exception>
    public Line2D Line()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Line)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Line");
        }
        return myLin;
    }

    /// <exception cref="NoSuchObjectException">The type is not Circle.</exception>
    public Circle2D Circle()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Circle)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Circle");
        }
        return myCirc;
    }

    /// <exception cref="NoSuchObjectException">The type is not Ellipse.</exception>
    public Ellipse2D Ellipse()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Ellipse)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Ellipse");
        }
        return myElips;
    }

    /// <exception cref="NoSuchObjectException">The type is not Hyperbola.</exception>
    public Hyperbola2D Hyperbola()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Hyperbola)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Hyperbola");
        }
        return myHypr;
    }

    /// <exception cref="NoSuchObjectException">The type is not Parabola.</exception>
    public Parabola2D Parabola()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Parabola)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Parabola");
        }
        return myParab;
    }

    public virtual void Project(in Line3D L)
    {
        myType = OtherCurve;
    }

    public virtual void Project(in Circle3D C)
    {
        myType = OtherCurve;
    }

    public virtual void Project(in Ellipse3D E)
    {
        myType = OtherCurve;
    }

    public virtual void Project(in Parabola3D P)
    {
        myType = OtherCurve;
    }

    public virtual void Project(in Hyperbola3D H)
    {
        myType = OtherCurve;
    }

    /// <summary>
    /// Translates the 2d curve to set the part of the curve [CFirst, CLast] in the range [UFirst, UFirst + Period[.
    /// Only lines are translated; CLast is not used.
    /// </summary>
    public void UFrame(double CFirst,
                       //			       double CLast,
                       double CLast,
                       double UFirst,
                       double Period)
    {
        _ = CLast;
        if (myType == global::Kerf.Geometry.CurveKind.Line)
        {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 Begin
            //     if ( myLin.Direction().Y() == 0.) {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 End
            //      gp_Pnt2d PFirst, PLast;
            Point2D PFirst;
            PFirst = global::Kerf.Engine.Geometry.ElCLib.Value(CFirst, myLin);
            // PLast  = ElCLib::Value(CLast ,myLin);
            // double U = std::min( PFirst.X(), PLast.X());
            double U = PFirst.X();
            double NewU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(U, UFirst, UFirst + Period);
            myLin.Translate(new Vector2D(NewU - U, 0.0));
        }
    }

    /// <summary>
    /// Translates the 2d curve to set the part of the curve [CFirst, CLast] in the range [VFirst, VFirst + Period[.
    /// Only lines are translated; CLast is not used.
    /// </summary>
    public void VFrame(double CFirst,
                       //			       double CLast,
                       double CLast,
                       double VFirst,
                       double Period)
    {
        _ = CLast;
        if (myType == global::Kerf.Geometry.CurveKind.Line)
        {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 Begin
            //     if ( myLin.Direction().X() == 0.) {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 End
            //      gp_Pnt2d PFirst, PLast;
            Point2D PFirst;
            PFirst = global::Kerf.Engine.Geometry.ElCLib.Value(CFirst, myLin);
            // PLast  = ElCLib::Value(CLast ,myLin);
            // double V = std::min( PFirst.Y(), PLast.Y());
            double V = PFirst.Y();
            double NewV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(V, VFirst, VFirst + Period);
            myLin.Translate(new Vector2D(0.0, NewV - V));
        }
    }

    public void SetBezier(Geom2d_BezierCurve? C)
    {
        myBezier = C;
    }

    /// <summary>The Bezier curve set by <see cref="SetBezier"/>, frozen when handed out (unit 6A): the same object
    /// at each call, shared as in OCCT.</summary>
    public Geom2d_BezierCurve? Bezier()
    {
        myBezier?.Freeze();
        return myBezier;
    }

    /// <summary>The stored Bezier curve, unfrozen, for ported code that is still building it.</summary>
    internal Geom2d_BezierCurve? InternalBezier() => myBezier;

    public void SetBSpline(Geom2d_BSplineCurve? C)
    {
        myBSpline = C;
    }

    /// <summary>The B-spline curve set by <see cref="SetBSpline"/>, frozen when handed out (unit 6A): the same object
    /// at each call, shared as in OCCT.</summary>
    public Geom2d_BSplineCurve? BSpline()
    {
        myBSpline?.Freeze();
        return myBSpline;
    }

    /// <summary>The stored B-spline curve, unfrozen, for ported code that is still building it
    /// (ProjLib_ProjectedCurve translates it into the surface's period).</summary>
    internal Geom2d_BSplineCurve? InternalBSpline() => myBSpline;

    /// <summary>
    /// C++'s implicit copy assignment (<c>myResult = P;</c>, slicing a subclass to a ProjLib_Projector): copies the
    /// fields, sharing the Bezier and B-spline curves as OCCT shares the handles.
    /// </summary>
    internal void Assign(ProjLib_Projector theOther)
    {
        myType = theOther.myType;
        myLin = theOther.myLin;
        myCirc = theOther.myCirc;
        myElips = theOther.myElips;
        myHypr = theOther.myHypr;
        myParab = theOther.myParab;
        myBSpline = theOther.myBSpline;
        myBezier = theOther.myBezier;
        myIsPeriodic = theOther.myIsPeriodic;
        isDone = theOther.isDone;
    }
}
