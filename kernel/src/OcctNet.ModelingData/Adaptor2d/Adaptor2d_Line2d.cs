// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_Line2d.hxx
// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_Line2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>A 2D line segment as an adapted curve; used by Adaptor3d_TopolTool to trim a surface.</summary>
/// <remarks>An adaptor is a working object: <see cref="Load(in gp_Lin2d)"/> is public, as in OCCT.</remarks>
public class Adaptor2d_Line2d : Adaptor2d_Curve2d
{
    private double myUfirst;
    private double myUlast;
    private gp_Ax2d myAx2d = new gp_Ax2d();

    /// <summary>An empty adaptor (the X axis, parameters 0 to 0).</summary>
    public Adaptor2d_Line2d()
    {
        myUfirst = 0.0;
        myUlast = 0.0;
    }

    /// <summary>The line through P with direction D, between UFirst and ULast.</summary>
    public Adaptor2d_Line2d(in gp_Pnt2d P, in gp_Dir2d D, double UFirst, double ULast)
    {
        myUfirst = UFirst;
        myUlast = ULast;
        myAx2d = new gp_Ax2d(P, D);
    }

    public override Adaptor2d_Curve2d ShallowCopy()
    {
        Adaptor2d_Line2d aCopy = new Adaptor2d_Line2d();
        aCopy.myUfirst = myUfirst;
        aCopy.myUlast = myUlast;
        aCopy.myAx2d = myAx2d;
        return aCopy;
    }

    /// <summary>Loads the infinite line L.</summary>
    public void Load(in gp_Lin2d L)
    {
        myAx2d = L.Position();
        myUfirst = -Precision.Infinite();
        myUlast = Precision.Infinite();
    }

    /// <summary>Loads the line L between Fi and La.</summary>
    public void Load(in gp_Lin2d L, double Fi, double La)
    {
        myAx2d = L.Position();
        myUfirst = Fi;
        myUlast = La;
    }

    public override double FirstParameter() => myUfirst;

    public override double LastParameter() => myUlast;

    public override GeomAbs_Shape Continuity() => GeomAbs_CN;

    public override int NbIntervals(GeomAbs_Shape S) => 1;

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        T[T.Lower()] = myUfirst;
        T[T.Lower() + 1] = myUlast;
    }

    public override Adaptor2d_Curve2d Trim(double First, double Last, double Tol)
    {
        Adaptor2d_Line2d HL = new Adaptor2d_Line2d();
        HL.Load(new gp_Lin2d(myAx2d), First, Last);
        return HL;
    }

    public override bool IsClosed() => false;

    public override bool IsPeriodic() => false;

    public override double Period() => throw new Standard_NoSuchObject();

    public override gp_Pnt2d Value(double X) => ElCLib.LineValue(X, myAx2d);

    public override void D0(double X, out gp_Pnt2d P) => P = ElCLib.LineValue(X, myAx2d);

    public override void D1(double X, out gp_Pnt2d P, out gp_Vec2d V) => ElCLib.LineD1(X, myAx2d, out P, out V);

    public override void D2(double X, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        ElCLib.LineD1(X, myAx2d, out P, out V1);
        V2 = new gp_Vec2d(0.0, 0.0);
    }

    public override void D3(double X, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
    {
        ElCLib.LineD1(X, myAx2d, out P, out V1);
        V2 = new gp_Vec2d(0.0, 0.0);
        V3 = new gp_Vec2d(0.0, 0.0);
    }

    /// <summary>The direction for N = 1, a null vector for N &gt; 1.</summary>
    /// <exception cref="Standard_OutOfRange">N &lt;= 0.</exception>
    public override gp_Vec2d DN(double U, int N)
    {
        if (N <= 0)
        {
            throw new Standard_OutOfRange();
        }
        if (N == 1)
        {
            return new gp_Vec2d(myAx2d.Direction());
        }
        return new gp_Vec2d(0.0, 0.0);
    }

    public override double Resolution(double R3d) => R3d; // nul !!!!

    public override GeomAbs_CurveType GetType() => GeomAbs_Line;

    public override gp_Lin2d Line() => new gp_Lin2d(myAx2d);

    public override gp_Circ2d Circle() => throw new Standard_NoSuchObject();

    public override gp_Elips2d Ellipse() => throw new Standard_NoSuchObject();

    public override gp_Hypr2d Hyperbola() => throw new Standard_NoSuchObject();

    public override gp_Parab2d Parabola() => throw new Standard_NoSuchObject();

    public override int Degree() => throw new Standard_NoSuchObject();

    public override bool IsRational() => throw new Standard_NoSuchObject();

    public override int NbPoles() => throw new Standard_NoSuchObject();

    public override int NbKnots() => throw new Standard_NoSuchObject();

    public override Geom2d_BezierCurve Bezier() => throw new Standard_NoSuchObject();

    public override Geom2d_BSplineCurve BSpline() => throw new Standard_NoSuchObject();
}
