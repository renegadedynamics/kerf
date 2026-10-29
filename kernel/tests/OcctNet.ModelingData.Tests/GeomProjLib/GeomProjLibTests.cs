// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// GeomProjLib: Curve2d (every overload, trimmed curves, the reached tolerance), Project on planes and on other
/// surfaces, ProjectOnPlane; results are new curves the caller owns (unfrozen) and the caller's curves are unchanged.
/// </summary>
public class GeomProjLibTests
{
    private static readonly gp_Ax3 Std = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0));

    private static Geom_BSplineCurve Helix(double theR, double theT0, double theT1)
    {
        // A helix-like curve on the cylinder of radius theR, interpolated through its points.
        int aNb = 9;
        var aPoles = new NCollection_Array1<gp_Pnt>(1, aNb);
        for (int i = 1; i <= aNb; i++)
        {
            double t = theT0 + (theT1 - theT0) * (i - 1) / (aNb - 1);
            aPoles[i] = new gp_Pnt(theR * Math.Cos(t), theR * Math.Sin(t), 0.3 * t);
        }
        var aKnots = new NCollection_Array1<double>(1, aNb - 2);
        var aMults = new NCollection_Array1<int>(1, aNb - 2);
        for (int i = 1; i <= aNb - 2; i++)
        {
            aKnots[i] = i - 1;
            aMults[i] = (i == 1 || i == aNb - 2) ? 4 : 1;
        }
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    [Fact]
    public void Curve2d_LineOnPlane_TrimmedAndNot()
    {
        var aPln = new Geom_Plane(Std);
        var aLine = new Geom_Line(new gp_Lin(new gp_Pnt(1, 2, 0), new gp_Dir(1, 1, 0)));
        Geom2d_Curve? aC2d = GeomProjLib.Curve2d(aLine, 0.0, 2.0, aPln);
        var aL2d = Assert.IsType<Geom2d_Line>(aC2d);
        Assert.Equal(1.0, aL2d.Location().X(), 1.0e-15);
        Assert.Equal(2.0, aL2d.Location().Y(), 1.0e-15);

        var aTrimmed = new Geom_TrimmedCurve(aLine, 0.5, 1.5);
        Geom2d_Curve? aT2d = GeomProjLib.Curve2d(aTrimmed, aPln);
        var aT = Assert.IsType<Geom2d_TrimmedCurve>(aT2d);
        Assert.Equal(0.5, aT.FirstParameter());
        Assert.Equal(1.5, aT.LastParameter());
        Assert.False(aT.IsFrozen());
    }

    [Fact]
    public void Curve2d_HelixOnCylinder_ToleranceAndOwnership()
    {
        var aCyl = new Geom_CylindricalSurface(Std, 2.0);
        Geom_BSplineCurve aHelix = Helix(2.0, 0.2, 2.8);
        double aTol = 1.0e-4;
        // The interpolating B-spline lies near the cylinder only; the projection approximates it.
        Geom2d_Curve? aC2d = GeomProjLib.Curve2d(aHelix, aHelix.FirstParameter(), aHelix.LastParameter(), aCyl, ref aTol);
        var aBS = Assert.IsType<Geom2d_BSplineCurve>(aC2d);
        Assert.False(aBS.IsFrozen());
        Assert.True(aTol > 0.0 && aTol < 1.0);
        for (int i = 0; i <= 20; i++)
        {
            double t = aHelix.FirstParameter() + (aHelix.LastParameter() - aHelix.FirstParameter()) * i / 20;
            gp_Pnt2d aUV = aBS.Value(t);
            gp_Pnt aOnS = aCyl.Value(aUV.X(), aUV.Y());
            gp_Pnt aOnC = aHelix.Value(t);
            // The projection of the curve point on the cylinder.
            var aRad = new gp_Vec(aOnC.X(), aOnC.Y(), 0.0);
            gp_Pnt aExpected = new(2.0 * aRad.X() / aRad.Magnitude(), 2.0 * aRad.Y() / aRad.Magnitude(), aOnC.Z());
            Assert.True(aOnS.Distance(aExpected) < 1.0e-3, $"t = {t}: {aOnS.Distance(aExpected)}");
        }
    }

    [Fact]
    public void Curve2d_Overloads_Agree()
    {
        var aCyl = new Geom_CylindricalSurface(Std, 1.0);
        var aCirc = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0.7), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 1.0);
        double aTol = 1.0e-7;
        Geom2d_Curve? a1 = GeomProjLib.Curve2d(aCirc, aCyl);
        Geom2d_Curve? a2 = GeomProjLib.Curve2d(aCirc, aCyl, 0.0, 2.0 * M_PI, -10.0, 10.0);
        Geom2d_Curve? a3 = GeomProjLib.Curve2d(aCirc, aCyl, 0.0, 2.0 * M_PI, -10.0, 10.0, ref aTol);
        Geom2d_Curve? a4 = GeomProjLib.Curve2d(aCirc, 0.0, 1.0, aCyl, ref aTol);
        foreach (Geom2d_Curve? aC in new[] { a1, a2, a3, a4 })
        {
            var aLine = Assert.IsType<Geom2d_Line>(aC);
            Assert.Equal(0.7, aLine.Value(0.3).Y(), 1.0e-14);
            Assert.Equal(0.3, aLine.Value(0.3).X(), 1.0e-14);
        }
    }

    [Fact]
    public void Project_OnPlane_ConicsAndBSpline()
    {
        var aPln = new Geom_Plane(Std);
        var aCirc = new Geom_Circle(new gp_Ax2(new gp_Pnt(1, 2, 5), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 2.0);
        var aRes = Assert.IsType<Geom_Circle>(GeomProjLib.Project(aCirc, aPln));
        Assert.Equal(0.0, aRes.Location().Z(), 1.0e-15);
        Assert.Equal(2.0, aRes.Radius(), 1.0e-15);

        var aTrimmed = new Geom_TrimmedCurve(aCirc, 0.5, 2.0);
        var aT = Assert.IsType<Geom_TrimmedCurve>(GeomProjLib.Project(aTrimmed, aPln));
        Assert.Equal(0.5, aT.FirstParameter());
        Assert.Equal(2.0, aT.LastParameter());

        Geom_BSplineCurve aHelix = Helix(2.0, 0.2, 2.8);
        var aBS = Assert.IsType<Geom_BSplineCurve>(GeomProjLib.Project(aHelix, aPln));
        Assert.False(aBS.IsFrozen());
        for (int i = 0; i <= 10; i++)
        {
            double t = aHelix.FirstParameter() + (aHelix.LastParameter() - aHelix.FirstParameter()) * i / 10;
            gp_Pnt aP = aHelix.Value(t);
            Assert.True(aBS.Value(t).Distance(new gp_Pnt(aP.X(), aP.Y(), 0.0)) < 1.0e-9);
        }
    }

    [Fact]
    public void Project_OnCylinder_Approximated3dCurve()
    {
        var aCyl = new Geom_CylindricalSurface(Std, 1.0);
        // A segment outside the cylinder: its normal projection is an arc of a helix-like curve.
        var aLine = new Geom_TrimmedCurve(new Geom_Line(new gp_Lin(new gp_Pnt(2, -1, 0), new gp_Dir(0, 1, 0.5))), 0.0, 2.0);
        Geom_Curve? aRes = GeomProjLib.Project(aLine, aCyl);
        var aBS = Assert.IsType<Geom_BSplineCurve>(aRes);
        Assert.False(aBS.IsFrozen());
        for (int i = 0; i <= 10; i++)
        {
            double t = aBS.FirstParameter() + (aBS.LastParameter() - aBS.FirstParameter()) * i / 10;
            gp_Pnt aP = aBS.Value(t);
            // On the cylinder ...
            Assert.Equal(1.0, Math.Sqrt(aP.X() * aP.X() + aP.Y() * aP.Y()), 1.0e-3);
            // ... at the foot of a point of the segment: the radial direction through aP meets the line.
            var anExt = new GeomAPI_ProjectPointOnCurveLike(aLine, aP);
            Assert.True(anExt.Ok, $"t = {t}");
        }
    }

    // The foot of the projection: some point of the segment projects normally on the cylinder at aP.
    private sealed class GeomAPI_ProjectPointOnCurveLike
    {
        public bool Ok { get; }

        public GeomAPI_ProjectPointOnCurveLike(Geom_Curve theLine, in gp_Pnt theP)
        {
            double aBest = double.MaxValue;
            for (int i = 0; i <= 2000; i++)
            {
                double s = theLine.FirstParameter() + (theLine.LastParameter() - theLine.FirstParameter()) * i / 2000;
                gp_Pnt aQ = theLine.Value(s);
                double aR = Math.Sqrt(aQ.X() * aQ.X() + aQ.Y() * aQ.Y());
                var aFoot = new gp_Pnt(aQ.X() / aR, aQ.Y() / aR, aQ.Z());
                aBest = Math.Min(aBest, aFoot.Distance(theP));
            }
            Ok = aBest < 5.0e-3;
        }
    }

    [Fact]
    public void ProjectOnPlane_CircleAlongOblique_IsEllipse()
    {
        var aPln = new Geom_Plane(Std);
        var aCirc = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 3), new gp_Dir(0, 1, 1), new gp_Dir(1, 0, 0)), 1.0);
        Geom_Curve? aRes = GeomProjLib.ProjectOnPlane(aCirc, aPln, new gp_Dir(0, 0, 1), true);
        var anEl = Assert.IsType<Geom_Ellipse>(aRes);
        Assert.Equal(1.0, anEl.MajorRadius(), 1.0e-12);
        Assert.Equal(Math.Sqrt(0.5), anEl.MinorRadius(), 1.0e-12);
        for (int i = 0; i <= 8; i++)
        {
            double t = 2.0 * M_PI * i / 8;
            gp_Pnt aP = aCirc.Value(t);
            Assert.True(anEl.Value(t).Distance(new gp_Pnt(aP.X(), aP.Y(), 0.0)) < 1.0e-12);
        }

        var aTrimmed = new Geom_TrimmedCurve(aCirc, 0.5, 2.0);
        var aT = Assert.IsType<Geom_TrimmedCurve>(GeomProjLib.ProjectOnPlane(aTrimmed, aPln, new gp_Dir(0, 0, 1), true));
        Assert.Equal(0.5, aT.FirstParameter(), 1.0e-15);
        Assert.Equal(2.0, aT.LastParameter(), 1.0e-15);
    }

    [Fact]
    public void ProjectOnPlane_BSpline_UnfrozenAndCallerUnchanged()
    {
        var aPln = new Geom_Plane(Std);
        Geom_BSplineCurve aHelix = Helix(2.0, 0.2, 2.8);
        gp_Pnt aPole = aHelix.Pole(3);
        Geom_Curve? aRes = GeomProjLib.ProjectOnPlane(aHelix, aPln, new gp_Dir(0.2, 0.0, 1.0), true);
        var aBS = Assert.IsType<Geom_BSplineCurve>(aRes);
        Assert.False(aBS.IsFrozen());
        Assert.NotSame(aHelix, aBS);
        Assert.Equal(aPole, aHelix.Pole(3));
        Assert.Equal(0.0, aBS.Value(1.3).Z(), 1.0e-12);
    }
}
