// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_TransformedSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_TransformedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// An adaptor for surfaces with an applied transformation: it wraps a <see cref="GeomAdaptor_Surface"/> and
/// applies a gp_Trsf to every point and derivative it evaluates. The base class of BRepAdaptor_Surface.
/// </summary>
/// <remarks>
/// The queries that return geometry (<see cref="Plane"/>, <see cref="BSpline"/>, <see cref="BasisSurface"/>, ...)
/// answer from an adaptor of the transformed surface, built on first use and kept until the surface or the
/// transformation changes.
/// </remarks>
public class GeomAdaptor_TransformedSurface : Adaptor3d_Surface
{
    protected GeomAdaptor_Surface mySurf;
    protected gp_Trsf myTrsf;
    private protected GeomAdaptor_Surface? myTransformedAdaptor;

    /// <summary>An undefined surface with the identity transformation.</summary>
    public GeomAdaptor_TransformedSurface()
    {
        mySurf = new GeomAdaptor_Surface();
        myTrsf = new gp_Trsf();
    }

    /// <summary>theSurface transformed by theTrsf.</summary>
    public GeomAdaptor_TransformedSurface(Geom_Surface theSurface, in gp_Trsf theTrsf)
    {
        mySurf = new GeomAdaptor_Surface(theSurface);
        myTrsf = theTrsf;
        invalidateTransformedCache();
    }

    /// <summary>theSurface over the given bounds (and snapping tolerances), transformed by theTrsf.</summary>
    public GeomAdaptor_TransformedSurface(Geom_Surface theSurface,
                                          double theUFirst,
                                          double theULast,
                                          double theVFirst,
                                          double theVLast,
                                          in gp_Trsf theTrsf,
                                          double theTolU = 0.0,
                                          double theTolV = 0.0)
    {
        mySurf = new GeomAdaptor_Surface(theSurface, theUFirst, theULast, theVFirst, theVLast, theTolU, theTolV);
        myTrsf = theTrsf;
        invalidateTransformedCache();
    }

    /// <summary>A shallow copy (shallow copies of the inner adaptors).</summary>
    public override Adaptor3d_Surface ShallowCopy()
    {
        GeomAdaptor_TransformedSurface aCopy = new GeomAdaptor_TransformedSurface();
        aCopy.mySurf = (GeomAdaptor_Surface)mySurf.ShallowCopy();
        aCopy.myTrsf = myTrsf;
        // OCCT copies the optional adaptor by value (sharing its cache); the port gives the copy its own.
        aCopy.myTransformedAdaptor = (GeomAdaptor_Surface?)myTransformedAdaptor?.ShallowCopy();
        return aCopy;
    }

    /// <summary>Loads theSurface over its bounds, with theTrsf.</summary>
    public void Load(Geom_Surface theSurface, in gp_Trsf theTrsf)
    {
        mySurf.Load(theSurface);
        myTrsf = theTrsf;
        invalidateTransformedCache();
    }

    /// <summary>Loads theSurface over the given bounds, with theTrsf.</summary>
    public void Load(Geom_Surface theSurface,
                     double theUFirst,
                     double theULast,
                     double theVFirst,
                     double theVLast,
                     in gp_Trsf theTrsf,
                     double theTolU = 0.0,
                     double theTolV = 0.0)
    {
        mySurf.Load(theSurface, theUFirst, theULast, theVFirst, theVLast, theTolU, theTolV);
        myTrsf = theTrsf;
        invalidateTransformedCache();
    }

    /// <summary>Sets the transformation.</summary>
    public void SetTrsf(in gp_Trsf theTrsf)
    {
        myTrsf = theTrsf;
        invalidateTransformedCache();
    }

    /// <summary>True if the transformation is not the identity.</summary>
    public bool HasTrsf() => myTrsf.Form() != gp_Identity;

    /// <summary>The transformation.</summary>
    public gp_Trsf Trsf() => myTrsf;

    /// <summary>The adaptor of the original surface (the adaptor itself: a working object).</summary>
    [Obsolete("Use AdaptorSurfaceOriginal() instead to get the original surface without transformation")]
    public GeomAdaptor_Surface Surface() => mySurf;

    /// <summary>The adaptor of the original surface, without transformation.</summary>
    public GeomAdaptor_Surface AdaptorSurfaceOriginal() => mySurf;

    /// <summary>The adaptor of the transformed surface (the original one for the identity).</summary>
    public GeomAdaptor_Surface AdaptorSurfaceTransformed() => transformedAdaptor();

    /// <summary>The original Geom surface (shared).</summary>
    public Geom_Surface? GeomSurfaceOriginal() => mySurf.Surface();

    /// <summary>The transformed Geom surface (the original one for the identity).</summary>
    public Geom_Surface? GeomSurfaceTransformed()
    {
        if (myTrsf.Form() == gp_Identity)
        {
            return mySurf.Surface();
        }
        ensureTransformedCache();
        return myTransformedAdaptor!.Surface();
    }

    /// <summary>The original Geom surface.</summary>
    [Obsolete("Use GeomSurfaceOriginal() or GeomSurfaceTransformed() instead")]
    public Geom_Surface? GeomSurface() => GeomSurfaceOriginal();

    public override double FirstUParameter() => mySurf.FirstUParameter();

    public override double LastUParameter() => mySurf.LastUParameter();

    public override double FirstVParameter() => mySurf.FirstVParameter();

    public override double LastVParameter() => mySurf.LastVParameter();

    public override GeomAbs_Shape UContinuity() => mySurf.UContinuity();

    public override GeomAbs_Shape VContinuity() => mySurf.VContinuity();

    public override int NbUIntervals(GeomAbs_Shape theS) => mySurf.NbUIntervals(theS);

    public override int NbVIntervals(GeomAbs_Shape theS) => mySurf.NbVIntervals(theS);

    public override void UIntervals(NCollection_Array1<double> theT, GeomAbs_Shape theS) => mySurf.UIntervals(theT, theS);

    public override void VIntervals(NCollection_Array1<double> theT, GeomAbs_Shape theS) => mySurf.VIntervals(theT, theS);

    /// <summary>The transformed surface's adaptor, trimmed in U.</summary>
    public override Adaptor3d_Surface UTrim(double theFirst, double theLast, double theTol) =>
        transformedAdaptor().UTrim(theFirst, theLast, theTol);

    /// <summary>The transformed surface's adaptor, trimmed in V.</summary>
    public override Adaptor3d_Surface VTrim(double theFirst, double theLast, double theTol) =>
        transformedAdaptor().VTrim(theFirst, theLast, theTol);

    public override bool IsUClosed() => mySurf.IsUClosed();

    public override bool IsVClosed() => mySurf.IsVClosed();

    public override bool IsUPeriodic() => mySurf.IsUPeriodic();

    public override double UPeriod() => mySurf.UPeriod();

    public override bool IsVPeriodic() => mySurf.IsVPeriodic();

    public override double VPeriod() => mySurf.VPeriod();

    /// <summary>The tolerance within which a U parameter is snapped to a bound.</summary>
    public double ToleranceU() => mySurf.ToleranceU();

    /// <summary>The tolerance within which a V parameter is snapped to a bound.</summary>
    public double ToleranceV() => mySurf.ToleranceV();

    /// <summary>Point evaluation. Applies transformation after evaluation.</summary>
    public sealed override gp_Pnt EvalD0(double theU, double theV)
    {
        if (myTrsf.Form() == gp_Identity)
        {
            return mySurf.EvalD0(theU, theV);
        }
        return mySurf.EvalD0(theU, theV).Transformed(myTrsf);
    }

    /// <summary>D1 evaluation. Applies transformation after evaluation.</summary>
    public sealed override Geom_Surface.ResD1 EvalD1(double theU, double theV)
    {
        if (myTrsf.Form() == gp_Identity)
        {
            return mySurf.EvalD1(theU, theV);
        }
        Geom_Surface.ResD1 aRes = mySurf.EvalD1(theU, theV);
        aRes.Point.Transform(myTrsf);
        aRes.D1U.Transform(myTrsf);
        aRes.D1V.Transform(myTrsf);
        return aRes;
    }

    /// <summary>D2 evaluation. Applies transformation after evaluation.</summary>
    public sealed override Geom_Surface.ResD2 EvalD2(double theU, double theV)
    {
        if (myTrsf.Form() == gp_Identity)
        {
            return mySurf.EvalD2(theU, theV);
        }
        Geom_Surface.ResD2 aRes = mySurf.EvalD2(theU, theV);
        aRes.Point.Transform(myTrsf);
        aRes.D1U.Transform(myTrsf);
        aRes.D1V.Transform(myTrsf);
        aRes.D2U.Transform(myTrsf);
        aRes.D2V.Transform(myTrsf);
        aRes.D2UV.Transform(myTrsf);
        return aRes;
    }

    /// <summary>D3 evaluation. Applies transformation after evaluation.</summary>
    public sealed override Geom_Surface.ResD3 EvalD3(double theU, double theV)
    {
        if (myTrsf.Form() == gp_Identity)
        {
            return mySurf.EvalD3(theU, theV);
        }
        Geom_Surface.ResD3 aRes = mySurf.EvalD3(theU, theV);
        aRes.Point.Transform(myTrsf);
        aRes.D1U.Transform(myTrsf);
        aRes.D1V.Transform(myTrsf);
        aRes.D2U.Transform(myTrsf);
        aRes.D2V.Transform(myTrsf);
        aRes.D2UV.Transform(myTrsf);
        aRes.D3U.Transform(myTrsf);
        aRes.D3V.Transform(myTrsf);
        aRes.D3UUV.Transform(myTrsf);
        aRes.D3UVV.Transform(myTrsf);
        return aRes;
    }

    /// <summary>DN evaluation. Applies transformation after evaluation.</summary>
    public sealed override gp_Vec EvalDN(double theU, double theV, int theNu, int theNv)
    {
        if (myTrsf.Form() == gp_Identity)
        {
            return mySurf.EvalDN(theU, theV, theNu, theNv);
        }
        return mySurf.EvalDN(theU, theV, theNu, theNv).Transformed(myTrsf);
    }

    public override double UResolution(double theR3d) => mySurf.UResolution(theR3d);

    public override double VResolution(double theR3d) => mySurf.VResolution(theR3d);

    public override GeomAbs_SurfaceType GetType() => mySurf.GetType();

    public override gp_Pln Plane() => transformedAdaptor().Plane();

    public override gp_Cylinder Cylinder() => transformedAdaptor().Cylinder();

    public override gp_Cone Cone() => transformedAdaptor().Cone();

    public override gp_Sphere Sphere() => transformedAdaptor().Sphere();

    public override gp_Torus Torus() => transformedAdaptor().Torus();

    public override int UDegree() => mySurf.UDegree();

    public override int NbUPoles() => mySurf.NbUPoles();

    public override int VDegree() => mySurf.VDegree();

    public override int NbVPoles() => mySurf.NbVPoles();

    public override int NbUKnots() => mySurf.NbUKnots();

    public override int NbVKnots() => mySurf.NbVKnots();

    public override bool IsURational() => mySurf.IsURational();

    public override bool IsVRational() => mySurf.IsVRational();

    public override Geom_BezierSurface Bezier() => transformedAdaptor().Bezier();

    public override Geom_BSplineSurface BSpline() => transformedAdaptor().BSpline();

    public override gp_Ax1 AxeOfRevolution() => transformedAdaptor().AxeOfRevolution();

    public override gp_Dir Direction() => transformedAdaptor().Direction();

    public override Adaptor3d_Curve BasisCurve() => transformedAdaptor().BasisCurve();

    public override Adaptor3d_Surface BasisSurface() => transformedAdaptor().BasisSurface();

    public override double OffsetValue() => transformedAdaptor().OffsetValue();

    protected void invalidateTransformedCache()
    {
        myTransformedAdaptor = null;
    }

    protected void ensureTransformedCache()
    {
        if (myTrsf.Form() == gp_Identity || myTransformedAdaptor is not null)
        {
            return;
        }
        initTransformedCache();
    }

    protected void initTransformedCache()
    {
        if (mySurf.Surface() is null)
        {
            myTransformedAdaptor = new GeomAdaptor_Surface();
            return;
        }
        bool isIdentity = myTrsf.Form() == gp_Identity;
        gp_Trsf aTrsf = myTrsf;
        Geom_Surface aSurface;
        switch (mySurf.GetType())
        {
            case GeomAbs_Plane:
                aSurface = isIdentity ? mySurf.Surface()! : new Geom_Plane(mySurf.Plane().Transformed(aTrsf));
                break;
            case GeomAbs_Cylinder:
                aSurface = isIdentity
                    ? mySurf.Surface()!
                    : new Geom_CylindricalSurface(mySurf.Cylinder().Transformed(aTrsf));
                break;
            case GeomAbs_Cone:
                aSurface = isIdentity ? mySurf.Surface()! : new Geom_ConicalSurface(mySurf.Cone().Transformed(aTrsf));
                break;
            case GeomAbs_Sphere:
                aSurface = isIdentity
                    ? mySurf.Surface()!
                    : new Geom_SphericalSurface(mySurf.Sphere().Transformed(aTrsf));
                break;
            case GeomAbs_Torus:
                aSurface = isIdentity ? mySurf.Surface()! : new Geom_ToroidalSurface(mySurf.Torus().Transformed(aTrsf));
                break;
            default:
                aSurface = isIdentity ? mySurf.Surface()! : (Geom_Surface)mySurf.Surface()!.Transformed(aTrsf);
                break;
        }
        myTransformedAdaptor = new GeomAdaptor_Surface(aSurface,
                                                       mySurf.FirstUParameter(),
                                                       mySurf.LastUParameter(),
                                                       mySurf.FirstVParameter(),
                                                       mySurf.LastVParameter(),
                                                       mySurf.ToleranceU(),
                                                       mySurf.ToleranceV());
    }

    protected GeomAdaptor_Surface transformedAdaptor()
    {
        if (myTrsf.Form() == gp_Identity)
        {
            return mySurf;
        }
        ensureTransformedCache();
        return myTransformedAdaptor!;
    }
}
