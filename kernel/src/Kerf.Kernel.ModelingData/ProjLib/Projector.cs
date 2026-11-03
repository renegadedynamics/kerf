// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Projector.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Projector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// The base of the projectors of elementary curves on elementary surfaces: holds the 2D result (a line, a conic,
/// a Bezier or a B-spline curve) and its type.
/// </summary>
/// <remarks>A working object (decision 21): its setters are public, as in OCCT.</remarks>
public class Projector
{
    protected CurveKind _type;
    protected Line2D _lin = new();
    protected Circle2D _circ = new();
    protected Ellipse2D _elips = new();
    protected Hyperbola2D _hypr = new();
    protected Parabola2D _parab = new();
    protected BSplineCurve2D? _bSpline;
    protected BezierCurve2D? _bezier;
    protected bool _isPeriodic;
    protected bool _isDone;

    /// <summary>Sets the type to BSplineCurve (OCCT's comment says OtherCurve).</summary>
    public Projector()
    {
        _isPeriodic = false;
        _isDone = false;
        _type = BSplineCurve; // OCCT: documented as OtherCurve
    }

    public bool IsDone => _isDone;

    /// <summary>Sets isDone = true.</summary>
    public void Done()
    {
        _isDone = true;
    }

    /// <summary>The type of the projected curve. Hides <see cref="object.GetType"/> (decision 98).</summary>
    public new CurveKind GetType() => _type;

    public void SetType(CurveKind type)
    {
        _type = type;
    }

    public bool IsPeriodic => _isPeriodic;

    public void SetPeriodic()
    {
        _isPeriodic = true;
    }

    /// <exception cref="NoSuchObjectException">The type is not Line.</exception>
    public Line2D Line()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Line)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Line");
        }
        return _lin;
    }

    /// <exception cref="NoSuchObjectException">The type is not Circle.</exception>
    public Circle2D Circle()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Circle)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Circle");
        }
        return _circ;
    }

    /// <exception cref="NoSuchObjectException">The type is not Ellipse.</exception>
    public Ellipse2D Ellipse()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Ellipse)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Ellipse");
        }
        return _elips;
    }

    /// <exception cref="NoSuchObjectException">The type is not Hyperbola.</exception>
    public Hyperbola2D Hyperbola()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Hyperbola)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Hyperbola");
        }
        return _hypr;
    }

    /// <exception cref="NoSuchObjectException">The type is not Parabola.</exception>
    public Parabola2D Parabola()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Parabola)
        {
            throw new NoSuchObjectException("ProjLib_Projector::Parabola");
        }
        return _parab;
    }

    public virtual void Project(in Line3D l)
    {
        _type = OtherCurve;
    }

    public virtual void Project(in Circle3D c)
    {
        _type = OtherCurve;
    }

    public virtual void Project(in Ellipse3D e)
    {
        _type = OtherCurve;
    }

    public virtual void Project(in Parabola3D p)
    {
        _type = OtherCurve;
    }

    public virtual void Project(in Hyperbola3D h)
    {
        _type = OtherCurve;
    }

    /// <summary>
    /// Translates the 2d curve to set the part of the curve [CFirst, CLast] in the range [UFirst, UFirst + Period[.
    /// Only lines are translated; CLast is not used.
    /// </summary>
    public void UFrame(double cFirst,
                       //			       double CLast,
                       double cLast,
                       double uFirst,
                       double period)
    {
        _ = cLast;
        if (_type == global::Kerf.Geometry.CurveKind.Line)
        {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 Begin
            //     if ( myLin.Direction().Y() == 0.) {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 End
            //      gp_Pnt2d PFirst, PLast;
            Point2D pFirst;
            pFirst = global::Kerf.Engine.Geometry.ElCLib.Value(cFirst, _lin);
            // PLast  = ElCLib::Value(CLast ,myLin);
            // double U = std::min( PFirst.X(), PLast.X());
            double u = pFirst.X();
            double newU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uFirst, uFirst + period);
            _lin = _lin.Translated(new Vector2D(newU - u, 0.0));
        }
    }

    /// <summary>
    /// Translates the 2d curve to set the part of the curve [CFirst, CLast] in the range [VFirst, VFirst + Period[.
    /// Only lines are translated; CLast is not used.
    /// </summary>
    public void VFrame(double cFirst,
                       //			       double CLast,
                       double cLast,
                       double vFirst,
                       double period)
    {
        _ = cLast;
        if (_type == global::Kerf.Geometry.CurveKind.Line)
        {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 Begin
            //     if ( myLin.Direction().X() == 0.) {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 End
            //      gp_Pnt2d PFirst, PLast;
            Point2D pFirst;
            pFirst = global::Kerf.Engine.Geometry.ElCLib.Value(cFirst, _lin);
            // PLast  = ElCLib::Value(CLast ,myLin);
            // double V = std::min( PFirst.Y(), PLast.Y());
            double v = pFirst.Y();
            double newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, vFirst, vFirst + period);
            _lin = _lin.Translated(new Vector2D(0.0, newV - v));
        }
    }

    public void SetBezier(BezierCurve2D? c)
    {
        _bezier = c;
    }

    /// <summary>The Bezier curve set by <see cref="SetBezier"/>, frozen when handed out (unit 6A): the same object
    /// at each call, shared as in OCCT.</summary>
    public BezierCurve2D? Bezier()
    {
        _bezier?.Freeze();
        return _bezier;
    }

    /// <summary>The stored Bezier curve, unfrozen, for ported code that is still building it.</summary>
    internal BezierCurve2D? InternalBezier() => _bezier;

    public void SetBSpline(BSplineCurve2D? c)
    {
        _bSpline = c;
    }

    /// <summary>The B-spline curve set by <see cref="SetBSpline"/>, frozen when handed out (unit 6A): the same object
    /// at each call, shared as in OCCT.</summary>
    public BSplineCurve2D? BSpline()
    {
        _bSpline?.Freeze();
        return _bSpline;
    }

    /// <summary>The stored B-spline curve, unfrozen, for ported code that is still building it
    /// (ProjLib_ProjectedCurve translates it into the surface's period).</summary>
    internal BSplineCurve2D? InternalBSpline() => _bSpline;

    /// <summary>
    /// C++'s implicit copy assignment (<c>myResult = P;</c>, slicing a subclass to a ProjLib_Projector): copies the
    /// fields, sharing the Bezier and B-spline curves as OCCT shares the handles.
    /// </summary>
    internal void Assign(Projector other)
    {
        _type = other._type;
        _lin = other._lin;
        _circ = other._circ;
        _elips = other._elips;
        _hypr = other._hypr;
        _parab = other._parab;
        _bSpline = other._bSpline;
        _bezier = other._bezier;
        _isPeriodic = other._isPeriodic;
        _isDone = other._isDone;
    }
}
