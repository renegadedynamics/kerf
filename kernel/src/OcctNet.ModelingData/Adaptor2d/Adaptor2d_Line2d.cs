// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_Line2d.hxx
// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_Line2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>A 2D line segment as an adapted curve; used by Adaptor3d_TopolTool to trim a surface.</summary>
/// <remarks>An adaptor is a working object: <see cref="Load(in Line2D)"/> is public, as in OCCT.</remarks>
public class Adaptor2d_Line2d : Adaptor2d_Curve2d
{
    private double myUfirst;
    private double myUlast;
    private Axis2D myAx2d = new Axis2D();

    /// <summary>An empty adaptor (the X axis, parameters 0 to 0).</summary>
    public Adaptor2d_Line2d()
    {
        myUfirst = 0.0;
        myUlast = 0.0;
    }

    /// <summary>The line through P with direction D, between UFirst and ULast.</summary>
    public Adaptor2d_Line2d(in Point2D P, in Direction2D D, double UFirst, double ULast)
    {
        myUfirst = UFirst;
        myUlast = ULast;
        myAx2d = new Axis2D(P, D);
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
    public void Load(in Line2D L)
    {
        myAx2d = L.Position();
        myUfirst = -global::Kerf.Engine.Foundation.Precision.Infinite();
        myUlast = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>Loads the line L between Fi and La.</summary>
    public void Load(in Line2D L, double Fi, double La)
    {
        myAx2d = L.Position();
        myUfirst = Fi;
        myUlast = La;
    }

    public override double FirstParameter() => myUfirst;

    public override double LastParameter() => myUlast;

    public override Continuity Continuity() => CN;

    public override int NbIntervals(Continuity S) => 1;

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        T[T.Lower()] = myUfirst;
        T[T.Lower() + 1] = myUlast;
    }

    public override Adaptor2d_Curve2d Trim(double First, double Last, double Tol)
    {
        Adaptor2d_Line2d HL = new Adaptor2d_Line2d();
        HL.Load(new Line2D(myAx2d), First, Last);
        return HL;
    }

    public override bool IsClosed() => false;

    public override bool IsPeriodic() => false;

    public override double Period() => throw new NoSuchObjectException();

    public override Point2D Value(double X) => global::Kerf.Engine.Geometry.ElCLib.LineValue(X, myAx2d);

    public override void D0(double X, out Point2D P) => P = global::Kerf.Engine.Geometry.ElCLib.LineValue(X, myAx2d);

    public override void D1(double X, out Point2D P, out Vector2D V) => global::Kerf.Engine.Geometry.ElCLib.LineD1(X, myAx2d, out P, out V);

    public override void D2(double X, out Point2D P, out Vector2D V1, out Vector2D V2)
    {
        global::Kerf.Engine.Geometry.ElCLib.LineD1(X, myAx2d, out P, out V1);
        V2 = new Vector2D(0.0, 0.0);
    }

    public override void D3(double X, out Point2D P, out Vector2D V1, out Vector2D V2, out Vector2D V3)
    {
        global::Kerf.Engine.Geometry.ElCLib.LineD1(X, myAx2d, out P, out V1);
        V2 = new Vector2D(0.0, 0.0);
        V3 = new Vector2D(0.0, 0.0);
    }

    /// <summary>The direction for N = 1, a null vector for N &gt; 1.</summary>
    /// <exception cref="OutOfRangeException">N &lt;= 0.</exception>
    public override Vector2D DN(double U, int N)
    {
        if (N <= 0)
        {
            throw new OutOfRangeException();
        }
        if (N == 1)
        {
            return new Vector2D(myAx2d.Direction());
        }
        return new Vector2D(0.0, 0.0);
    }

    public override double Resolution(double R3d) => R3d; // nul !!!!

    public override CurveKind GetType() => global::Kerf.Geometry.CurveKind.Line;

    public override Line2D Line() => new Line2D(myAx2d);

    public override Circle2D Circle() => throw new NoSuchObjectException();

    public override Ellipse2D Ellipse() => throw new NoSuchObjectException();

    public override Hyperbola2D Hyperbola() => throw new NoSuchObjectException();

    public override Parabola2D Parabola() => throw new NoSuchObjectException();

    public override int Degree() => throw new NoSuchObjectException();

    public override bool IsRational() => throw new NoSuchObjectException();

    public override int NbPoles() => throw new NoSuchObjectException();

    public override int NbKnots() => throw new NoSuchObjectException();

    public override Geom2d_BezierCurve Bezier() => throw new NoSuchObjectException();

    public override Geom2d_BSplineCurve BSpline() => throw new NoSuchObjectException();
}
