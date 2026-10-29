// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Projector.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Projector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The base of the projectors of elementary curves on elementary surfaces: holds the 2D result (a line, a conic,
/// a Bezier or a B-spline curve) and its type.
/// </summary>
/// <remarks>A working object (decision 21): its setters are public, as in OCCT.</remarks>
public class ProjLib_Projector
{
    protected GeomAbs_CurveType myType;
    protected gp_Lin2d myLin = new();
    protected gp_Circ2d myCirc = new();
    protected gp_Elips2d myElips = new();
    protected gp_Hypr2d myHypr = new();
    protected gp_Parab2d myParab = new();
    protected Geom2d_BSplineCurve? myBSpline;
    protected Geom2d_BezierCurve? myBezier;
    protected bool myIsPeriodic;
    protected bool isDone;

    /// <summary>Sets the type to BSplineCurve (OCCT's comment says OtherCurve).</summary>
    public ProjLib_Projector()
    {
        myIsPeriodic = false;
        isDone = false;
        myType = GeomAbs_BSplineCurve; // OCCT: documented as OtherCurve
    }

    public bool IsDone() => isDone;

    /// <summary>Sets isDone = true.</summary>
    public void Done()
    {
        isDone = true;
    }

    /// <summary>The type of the projected curve. Hides <see cref="object.GetType"/> (decision 98).</summary>
    public new GeomAbs_CurveType GetType() => myType;

    public void SetType(GeomAbs_CurveType Type)
    {
        myType = Type;
    }

    public bool IsPeriodic() => myIsPeriodic;

    public void SetPeriodic()
    {
        myIsPeriodic = true;
    }

    /// <exception cref="Standard_NoSuchObject">The type is not Line.</exception>
    public gp_Lin2d Line()
    {
        if (myType != GeomAbs_Line)
        {
            throw new Standard_NoSuchObject("ProjLib_Projector::Line");
        }
        return myLin;
    }

    /// <exception cref="Standard_NoSuchObject">The type is not Circle.</exception>
    public gp_Circ2d Circle()
    {
        if (myType != GeomAbs_Circle)
        {
            throw new Standard_NoSuchObject("ProjLib_Projector::Circle");
        }
        return myCirc;
    }

    /// <exception cref="Standard_NoSuchObject">The type is not Ellipse.</exception>
    public gp_Elips2d Ellipse()
    {
        if (myType != GeomAbs_Ellipse)
        {
            throw new Standard_NoSuchObject("ProjLib_Projector::Ellipse");
        }
        return myElips;
    }

    /// <exception cref="Standard_NoSuchObject">The type is not Hyperbola.</exception>
    public gp_Hypr2d Hyperbola()
    {
        if (myType != GeomAbs_Hyperbola)
        {
            throw new Standard_NoSuchObject("ProjLib_Projector::Hyperbola");
        }
        return myHypr;
    }

    /// <exception cref="Standard_NoSuchObject">The type is not Parabola.</exception>
    public gp_Parab2d Parabola()
    {
        if (myType != GeomAbs_Parabola)
        {
            throw new Standard_NoSuchObject("ProjLib_Projector::Parabola");
        }
        return myParab;
    }

    public virtual void Project(in gp_Lin L)
    {
        myType = GeomAbs_OtherCurve;
    }

    public virtual void Project(in gp_Circ C)
    {
        myType = GeomAbs_OtherCurve;
    }

    public virtual void Project(in gp_Elips E)
    {
        myType = GeomAbs_OtherCurve;
    }

    public virtual void Project(in gp_Parab P)
    {
        myType = GeomAbs_OtherCurve;
    }

    public virtual void Project(in gp_Hypr H)
    {
        myType = GeomAbs_OtherCurve;
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
        if (myType == GeomAbs_Line)
        {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 Begin
            //     if ( myLin.Direction().Y() == 0.) {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 End
            //      gp_Pnt2d PFirst, PLast;
            gp_Pnt2d PFirst;
            PFirst = ElCLib.Value(CFirst, myLin);
            // PLast  = ElCLib::Value(CLast ,myLin);
            // double U = std::min( PFirst.X(), PLast.X());
            double U = PFirst.X();
            double NewU = ElCLib.InPeriod(U, UFirst, UFirst + Period);
            myLin.Translate(new gp_Vec2d(NewU - U, 0.0));
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
        if (myType == GeomAbs_Line)
        {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 Begin
            //     if ( myLin.Direction().X() == 0.) {
            //  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272 End
            //      gp_Pnt2d PFirst, PLast;
            gp_Pnt2d PFirst;
            PFirst = ElCLib.Value(CFirst, myLin);
            // PLast  = ElCLib::Value(CLast ,myLin);
            // double V = std::min( PFirst.Y(), PLast.Y());
            double V = PFirst.Y();
            double NewV = ElCLib.InPeriod(V, VFirst, VFirst + Period);
            myLin.Translate(new gp_Vec2d(0.0, NewV - V));
        }
    }

    public void SetBezier(Geom2d_BezierCurve? C)
    {
        myBezier = C;
    }

    /// <summary>The Bezier curve set by <see cref="SetBezier"/> (shared, as in OCCT).</summary>
    public Geom2d_BezierCurve? Bezier() => myBezier;

    public void SetBSpline(Geom2d_BSplineCurve? C)
    {
        myBSpline = C;
    }

    /// <summary>The B-spline curve set by <see cref="SetBSpline"/> (shared, as in OCCT).</summary>
    public Geom2d_BSplineCurve? BSpline() => myBSpline;
}
