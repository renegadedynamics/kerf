// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_TransformedCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_TransformedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// An adaptor for curves with an applied transformation: it wraps a <see cref="GeomAdaptor_Curve"/> (or a curve on
/// a surface) and applies a gp_Trsf to every point and derivative it evaluates. The base class of
/// BRepAdaptor_Curve.
/// </summary>
/// <remarks>
/// The curve-on-surface adaptor is typed <see cref="Adaptor3d_Curve"/> until unit 2B's Adaptor3d_CurveOnSurface is
/// merged (<c>TODO(2B)</c>); only <see cref="LoadCurveOnSurface"/> and <see cref="CurveOnSurface"/> name the type.
/// </remarks>
public class GeomAdaptor_TransformedCurve : Adaptor3d_Curve
{
    protected GeomAdaptor_Curve myCurve;

    // TODO(2B): Adaptor3d_CurveOnSurface (OCCT: occ::handle<Adaptor3d_CurveOnSurface> myConSurf).
    protected Adaptor3d_Curve? myConSurf;

    protected gp_Trsf myTrsf;

    /// <summary>An undefined curve with the identity transformation.</summary>
    public GeomAdaptor_TransformedCurve()
    {
        myCurve = new GeomAdaptor_Curve();
        myTrsf = new gp_Trsf();
    }

    /// <summary>theCurve transformed by theTrsf.</summary>
    public GeomAdaptor_TransformedCurve(Geom_Curve theCurve, in gp_Trsf theTrsf)
    {
        myCurve = new GeomAdaptor_Curve(theCurve);
        myTrsf = theTrsf;
    }

    /// <summary>theCurve between theFirst and theLast, transformed by theTrsf.</summary>
    public GeomAdaptor_TransformedCurve(Geom_Curve theCurve, double theFirst, double theLast, in gp_Trsf theTrsf)
    {
        myCurve = new GeomAdaptor_Curve(theCurve, theFirst, theLast);
        myTrsf = theTrsf;
    }

    /// <summary>A shallow copy (shallow copies of the inner adaptors).</summary>
    public override Adaptor3d_Curve ShallowCopy()
    {
        GeomAdaptor_TransformedCurve aCopy = new GeomAdaptor_TransformedCurve();
        aCopy.myCurve = (GeomAdaptor_Curve)myCurve.ShallowCopy();
        if (myConSurf is not null)
        {
            aCopy.myConSurf = myConSurf.ShallowCopy();
        }
        aCopy.myTrsf = myTrsf;
        return aCopy;
    }

    /// <summary>Loads the curve geometry.</summary>
    public void Load(Geom_Curve theCurve) => myCurve.Load(theCurve);

    /// <summary>Loads the curve geometry with parameter bounds.</summary>
    public void Load(Geom_Curve theCurve, double theFirst, double theLast) => myCurve.Load(theCurve, theFirst, theLast);

    /// <summary>Sets the curve on surface adaptor (an Adaptor3d_CurveOnSurface; TODO(2B): typed so once merged).</summary>
    public void LoadCurveOnSurface(Adaptor3d_Curve theConSurf) => myConSurf = theConSurf;

    /// <summary>Sets the transformation.</summary>
    public void SetTrsf(in gp_Trsf theTrsf) => myTrsf = theTrsf;

    /// <summary>The transformation.</summary>
    public gp_Trsf Trsf() => myTrsf;

    /// <summary>True if the geometry is a 3D curve (not curve on surface).</summary>
    public bool Is3DCurve() => myConSurf is null;

    /// <summary>True if the geometry is a curve on surface.</summary>
    public bool IsCurveOnSurface() => myConSurf is not null;

    /// <summary>The underlying GeomAdaptor_Curve (the adaptor itself: a working object).</summary>
    public GeomAdaptor_Curve Curve() => myCurve;

    /// <summary>The underlying GeomAdaptor_Curve for modification (the same object as <see cref="Curve"/>).</summary>
    public GeomAdaptor_Curve ChangeCurve() => myCurve;

    /// <summary>The curve-on-surface adaptor (TODO(2B): Adaptor3d_CurveOnSurface).</summary>
    public Adaptor3d_Curve CurveOnSurface() => myConSurf!;

    /// <summary>The underlying Geom_Curve.</summary>
    public Geom_Curve? GeomCurve() => myCurve.Curve();

    // Parameter range methods - delegate to underlying curve or COS

    public override double FirstParameter() => myConSurf is null ? myCurve.FirstParameter() : myConSurf.FirstParameter();

    public override double LastParameter() => myConSurf is null ? myCurve.LastParameter() : myConSurf.LastParameter();

    public override GeomAbs_Shape Continuity() => myConSurf is null ? myCurve.Continuity() : myConSurf.Continuity();

    public override int NbIntervals(GeomAbs_Shape theS) =>
        myConSurf is null ? myCurve.NbIntervals(theS) : myConSurf.NbIntervals(theS);

    public override void Intervals(NCollection_Array1<double> theT, GeomAbs_Shape theS)
    {
        if (myConSurf is null)
        {
            myCurve.Intervals(theT, theS);
        }
        else
        {
            myConSurf.Intervals(theT, theS);
        }
    }

    /// <summary>A new adaptor of the curve between theFirst and theLast, with the same transformation.</summary>
    public override Adaptor3d_Curve Trim(double theFirst, double theLast, double theTol)
    {
        GeomAdaptor_TransformedCurve aCopy = new GeomAdaptor_TransformedCurve();
        if (myConSurf is null)
        {
            aCopy.myCurve.Load(myCurve.Curve()!, theFirst, theLast);
        }
        else
        {
            aCopy.myConSurf = myConSurf.Trim(theFirst, theLast, theTol);
        }
        aCopy.myTrsf = myTrsf;
        return aCopy;
    }

    public override bool IsClosed() => myConSurf is null ? myCurve.IsClosed() : myConSurf.IsClosed();

    public override bool IsPeriodic() => myConSurf is null ? myCurve.IsPeriodic() : myConSurf.IsPeriodic();

    public override double Period() => myConSurf is null ? myCurve.Period() : myConSurf.Period();

    /// <summary>Point evaluation. Applies transformation after evaluation.</summary>
    public sealed override gp_Pnt EvalD0(double theU)
    {
        gp_Pnt aP;
        if (myConSurf is null)
        {
            aP = myCurve.EvalD0(theU);
        }
        else
        {
            myConSurf.D0(theU, out aP);
        }
        aP.Transform(myTrsf);
        return aP;
    }

    /// <summary>D1 evaluation. Applies transformation after evaluation.</summary>
    public sealed override Geom_Curve.ResD1 EvalD1(double theU)
    {
        Geom_Curve.ResD1 aRes;
        if (myConSurf is null)
        {
            aRes = myCurve.EvalD1(theU);
        }
        else
        {
            myConSurf.D1(theU, out aRes.Point, out aRes.D1);
        }
        aRes.Point.Transform(myTrsf);
        aRes.D1.Transform(myTrsf);
        return aRes;
    }

    /// <summary>D2 evaluation. Applies transformation after evaluation.</summary>
    public sealed override Geom_Curve.ResD2 EvalD2(double theU)
    {
        Geom_Curve.ResD2 aRes;
        if (myConSurf is null)
        {
            aRes = myCurve.EvalD2(theU);
        }
        else
        {
            myConSurf.D2(theU, out aRes.Point, out aRes.D1, out aRes.D2);
        }
        aRes.Point.Transform(myTrsf);
        aRes.D1.Transform(myTrsf);
        aRes.D2.Transform(myTrsf);
        return aRes;
    }

    /// <summary>D3 evaluation. Applies transformation after evaluation.</summary>
    public sealed override Geom_Curve.ResD3 EvalD3(double theU)
    {
        Geom_Curve.ResD3 aRes;
        if (myConSurf is null)
        {
            aRes = myCurve.EvalD3(theU);
        }
        else
        {
            myConSurf.D3(theU, out aRes.Point, out aRes.D1, out aRes.D2, out aRes.D3);
        }
        aRes.Point.Transform(myTrsf);
        aRes.D1.Transform(myTrsf);
        aRes.D2.Transform(myTrsf);
        aRes.D3.Transform(myTrsf);
        return aRes;
    }

    /// <summary>DN evaluation. Applies transformation after evaluation.</summary>
    public sealed override gp_Vec EvalDN(double theU, int theN)
    {
        gp_Vec aV;
        if (myConSurf is null)
        {
            aV = myCurve.EvalDN(theU, theN);
        }
        else
        {
            aV = myConSurf.DN(theU, theN);
        }
        aV.Transform(myTrsf);
        return aV;
    }

    public override double Resolution(double theR3d) =>
        myConSurf is null ? myCurve.Resolution(theR3d) : myConSurf.Resolution(theR3d);

    public override GeomAbs_CurveType GetType() => myConSurf is null ? myCurve.GetType() : myConSurf.GetType();

    /// <summary>The line, transformed.</summary>
    public override gp_Lin Line()
    {
        gp_Lin aL;
        if (myConSurf is null)
        {
            aL = myCurve.Line();
        }
        else
        {
            aL = myConSurf.Line();
        }
        aL.Transform(myTrsf);
        return aL;
    }

    /// <summary>The circle, transformed.</summary>
    public override gp_Circ Circle()
    {
        gp_Circ aC;
        if (myConSurf is null)
        {
            aC = myCurve.Circle();
        }
        else
        {
            aC = myConSurf.Circle();
        }
        aC.Transform(myTrsf);
        return aC;
    }

    /// <summary>The ellipse, transformed.</summary>
    public override gp_Elips Ellipse()
    {
        gp_Elips aE;
        if (myConSurf is null)
        {
            aE = myCurve.Ellipse();
        }
        else
        {
            aE = myConSurf.Ellipse();
        }
        aE.Transform(myTrsf);
        return aE;
    }

    /// <summary>The hyperbola, transformed.</summary>
    public override gp_Hypr Hyperbola()
    {
        gp_Hypr aH;
        if (myConSurf is null)
        {
            aH = myCurve.Hyperbola();
        }
        else
        {
            aH = myConSurf.Hyperbola();
        }
        aH.Transform(myTrsf);
        return aH;
    }

    /// <summary>The parabola, transformed.</summary>
    public override gp_Parab Parabola()
    {
        gp_Parab aP;
        if (myConSurf is null)
        {
            aP = myCurve.Parabola();
        }
        else
        {
            aP = myConSurf.Parabola();
        }
        aP.Transform(myTrsf);
        return aP;
    }

    public override int Degree() => myConSurf is null ? myCurve.Degree() : myConSurf.Degree();

    public override bool IsRational() => myConSurf is null ? myCurve.IsRational() : myConSurf.IsRational();

    public override int NbPoles() => myConSurf is null ? myCurve.NbPoles() : myConSurf.NbPoles();

    public override int NbKnots() => myConSurf is null ? myCurve.NbKnots() : myConSurf.NbKnots();

    /// <summary>The Bezier curve: the shared one for the identity, a transformed copy otherwise.</summary>
    public override Geom_BezierCurve Bezier()
    {
        Geom_BezierCurve aBC;
        if (myConSurf is null)
        {
            aBC = myCurve.Bezier();
        }
        else
        {
            aBC = myConSurf.Bezier();
        }
        return myTrsf.Form() == gp_Identity ? aBC : (Geom_BezierCurve)aBC.Transformed(myTrsf);
    }

    /// <summary>The B-spline curve: the shared one for the identity, a transformed copy otherwise.</summary>
    public override Geom_BSplineCurve BSpline()
    {
        Geom_BSplineCurve aBS;
        if (myConSurf is null)
        {
            aBS = myCurve.BSpline();
        }
        else
        {
            aBS = myConSurf.BSpline();
        }
        return myTrsf.Form() == gp_Identity ? aBS : (Geom_BSplineCurve)aBS.Transformed(myTrsf);
    }

    /// <summary>The offset curve: the shared one for the identity, a transformed copy otherwise.</summary>
    /// <exception cref="Standard_NoSuchObject">Not a 3D offset curve.</exception>
    public override Geom_OffsetCurve OffsetCurve()
    {
        if (!Is3DCurve() || myCurve.GetType() != GeomAbs_OffsetCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_TransformedCurve::OffsetCurve");
        }
        Geom_OffsetCurve anOffC = myCurve.OffsetCurve();
        return myTrsf.Form() == gp_Identity ? anOffC : (Geom_OffsetCurve)anOffC.Transformed(myTrsf);
    }
}
