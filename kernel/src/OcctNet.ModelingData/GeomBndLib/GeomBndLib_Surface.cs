// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Surface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// Bounding box of a surface: detects the surface type once and dispatches to the evaluator of that type
/// (GeomBndLib_Plane, _Cylinder, ..., _OtherSurface).
/// </summary>
/// <remarks>
/// Built from an adaptor, it keeps a reference to it (as OCCT does); built from a Geom_Surface, it owns a
/// GeomAdaptor_Surface for a rectangular trimmed surface or an unknown type. OCCT's <c>std::variant</c> of
/// evaluators is an interface field, null for <c>std::monostate</c> (decision 236's pattern).
/// </remarks>
public sealed class GeomBndLib_Surface
{
    /// <summary>The members of the per-type evaluators that the dispatcher calls through OCCT's <c>std::visit</c>.</summary>
    internal interface ISurfaceEvaluator
    {
        Box Box(double theTol);

        Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol);

        Box BoxOptimal(double theTol);

        Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol);
    }

    private readonly Adaptor3d_Surface? myAdaptorRef;
    private Adaptor3d_Surface? myAdaptorOwned;
    private ISurfaceEvaluator? myEvaluator;
    private SurfaceKind mySurfType;

    private static Geom_Surface? unwrapSurface(Geom_Surface? theSurf)
    {
        Geom_RectangularTrimmedSurface? aTrim = theSurf as Geom_RectangularTrimmedSurface;
        return aTrim is null ? theSurf : unwrapSurface(aTrim.BasisSurface());
    }

    /// <summary>Construct from an adaptor surface.</summary>
    public GeomBndLib_Surface(Adaptor3d_Surface theSurf)
    {
        mySurfType = theSurf.GetType();
        myAdaptorRef = theSurf;

        // Detect the adaptor type ONCE and extract the underlying Geom_Surface handle
        // (for GeomAdaptor_TransformedSurface, the transformed geometry).
        Geom_Surface? aGeomSurf = null;
        if (theSurf is GeomAdaptor_Surface aGA)
        {
            aGeomSurf = aGA.Surface();
        }
        else if (theSurf is GeomAdaptor_TransformedSurface aGTS)
        {
            aGeomSurf = aGTS.GeomSurfaceTransformed();
        }

        Geom_Surface? aBaseGeom = unwrapSurface(aGeomSurf);
        if (aBaseGeom is not null && initFromSurfaceType(mySurfType, aBaseGeom))
        {
            return;
        }

        switch (mySurfType)
        {
            case Plane:
                myEvaluator = new GeomBndLib_Plane(new Geom_Plane(myAdaptorRef.Plane()));
                break;
            case SurfaceKind.Cylinder:
                myEvaluator = new GeomBndLib_Cylinder(new Geom_CylindricalSurface(myAdaptorRef.Cylinder()));
                break;
            case SurfaceKind.Cone:
                myEvaluator = new GeomBndLib_Cone(new Geom_ConicalSurface(myAdaptorRef.Cone()));
                break;
            case SurfaceKind.Sphere:
                myEvaluator = new GeomBndLib_Sphere(new Geom_SphericalSurface(myAdaptorRef.Sphere()));
                break;
            case SurfaceKind.Torus:
                myEvaluator = new GeomBndLib_Torus(new Geom_ToroidalSurface(myAdaptorRef.Torus()));
                break;
            case BezierSurface:
                myEvaluator = new GeomBndLib_BezierSurface(myAdaptorRef.Bezier());
                break;
            case BSplineSurface:
                myEvaluator = new GeomBndLib_BSplineSurface(myAdaptorRef.BSpline());
                break;
            default:
                initOtherSurface(myAdaptorRef);
                break;
        }
    }

    /// <summary>Construct from a Geom_Surface.</summary>
    public GeomBndLib_Surface(Geom_Surface theSurf)
    {
        mySurfType = OtherSurface;

        if (theSurf is Geom_RectangularTrimmedSurface aTrim)
        {
            myAdaptorOwned = new GeomAdaptor_Surface(theSurf);
            if (!initFromSurface(aTrim.BasisSurface()))
            {
                myEvaluator = new GeomBndLib_OtherSurface(myAdaptorOwned);
            }
            return;
        }

        if (!initFromSurface(theSurf))
        {
            initOtherSurface(theSurf);
        }
    }

    private bool initFromSurface(Geom_Surface? theSurf)
    {
        if (theSurf is null)
        {
            return false;
        }

        // OCCT compares the dynamic type exactly (STANDARD_TYPE), not IsKind.
        Type aType = ((object)theSurf).GetType();
        SurfaceKind aSurfType;
        if (aType == typeof(Geom_Plane))
        {
            aSurfType = Plane;
        }
        else if (aType == typeof(Geom_CylindricalSurface))
        {
            aSurfType = SurfaceKind.Cylinder;
        }
        else if (aType == typeof(Geom_ConicalSurface))
        {
            aSurfType = SurfaceKind.Cone;
        }
        else if (aType == typeof(Geom_SphericalSurface))
        {
            aSurfType = SurfaceKind.Sphere;
        }
        else if (aType == typeof(Geom_ToroidalSurface))
        {
            aSurfType = SurfaceKind.Torus;
        }
        else if (aType == typeof(Geom_BezierSurface))
        {
            aSurfType = BezierSurface;
        }
        else if (aType == typeof(Geom_BSplineSurface))
        {
            aSurfType = BSplineSurface;
        }
        else if (aType == typeof(Geom_SurfaceOfRevolution))
        {
            aSurfType = SurfaceOfRevolution;
        }
        else if (aType == typeof(Geom_SurfaceOfLinearExtrusion))
        {
            aSurfType = SurfaceOfExtrusion;
        }
        else if (aType == typeof(Geom_OffsetSurface))
        {
            aSurfType = OffsetSurface;
        }
        else
        {
            return false;
        }

        mySurfType = aSurfType;
        return initFromSurfaceType(aSurfType, theSurf);
    }

    // OCCT's down_cast yields a null handle when the adaptor's type does not match its surface; the port's
    // evaluators then hold null, as OCCT's do, and fail when used.
    private bool initFromSurfaceType(SurfaceKind theType, Geom_Surface theSurf)
    {
        switch (theType)
        {
            case Plane:
                myEvaluator = new GeomBndLib_Plane((theSurf as Geom_Plane)!);
                return true;
            case SurfaceKind.Cylinder:
                myEvaluator = new GeomBndLib_Cylinder((theSurf as Geom_CylindricalSurface)!);
                return true;
            case SurfaceKind.Cone:
                myEvaluator = new GeomBndLib_Cone((theSurf as Geom_ConicalSurface)!);
                return true;
            case SurfaceKind.Sphere:
                myEvaluator = new GeomBndLib_Sphere((theSurf as Geom_SphericalSurface)!);
                return true;
            case SurfaceKind.Torus:
                myEvaluator = new GeomBndLib_Torus((theSurf as Geom_ToroidalSurface)!);
                return true;
            case BezierSurface:
                myEvaluator = new GeomBndLib_BezierSurface((theSurf as Geom_BezierSurface)!);
                return true;
            case BSplineSurface:
                myEvaluator = new GeomBndLib_BSplineSurface((theSurf as Geom_BSplineSurface)!);
                return true;
            case SurfaceOfRevolution:
                myEvaluator = new GeomBndLib_SurfaceOfRevolution((theSurf as Geom_SurfaceOfRevolution)!);
                return true;
            case SurfaceOfExtrusion:
                myEvaluator = new GeomBndLib_SurfaceOfExtrusion((theSurf as Geom_SurfaceOfLinearExtrusion)!);
                return true;
            case OffsetSurface:
                myEvaluator = new GeomBndLib_OffsetSurface((theSurf as Geom_OffsetSurface)!);
                return true;
            default:
                return false;
        }
    }

    private void initOtherSurface(Adaptor3d_Surface theSurf)
    {
        mySurfType = OtherSurface;
        myEvaluator = new GeomBndLib_OtherSurface(theSurf);
    }

    private void initOtherSurface(Geom_Surface theSurf)
    {
        myAdaptorOwned = new GeomAdaptor_Surface(theSurf);
        mySurfType = OtherSurface;
        myEvaluator = new GeomBndLib_OtherSurface(myAdaptorOwned);
    }

    /// <summary>Return detected surface type.</summary>
    public new SurfaceKind GetType() => mySurfType;

    private Adaptor3d_Surface? adaptor() => myAdaptorRef ?? myAdaptorOwned;

    private double effectiveU1()
    {
        Adaptor3d_Surface? anAdaptor = adaptor();
        return anAdaptor is not null ? anAdaptor.FirstUParameter() : 0.0;
    }

    private double effectiveU2()
    {
        Adaptor3d_Surface? anAdaptor = adaptor();
        return anAdaptor is not null ? anAdaptor.LastUParameter() : 0.0;
    }

    private double effectiveV1()
    {
        Adaptor3d_Surface? anAdaptor = adaptor();
        return anAdaptor is not null ? anAdaptor.FirstVParameter() : 0.0;
    }

    private double effectiveV2()
    {
        Adaptor3d_Surface? anAdaptor = adaptor();
        return anAdaptor is not null ? anAdaptor.LastVParameter() : 0.0;
    }

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double theTol)
    {
        if (adaptor() is not null)
        {
            return Box(effectiveU1(), effectiveU2(), effectiveV1(), effectiveV2(), theTol);
        }

        return myEvaluator is null ? new Box() : myEvaluator.Box(theTol);
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
        => myEvaluator is null ? new Box() : myEvaluator.Box(theUMin, theUMax, theVMin, theVMax, theTol);

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Box BoxOptimal(double theTol)
    {
        if (adaptor() is not null)
        {
            return BoxOptimal(effectiveU1(), effectiveU2(), effectiveV1(), effectiveV2(), theTol);
        }

        return myEvaluator is null ? new Box() : myEvaluator.BoxOptimal(theTol);
    }

    /// <summary>Compute precise bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
        => myEvaluator is null ? new Box() : myEvaluator.BoxOptimal(theUMin, theUMax, theVMin, theVMax, theTol);

    /// <summary>Add bounding box for full surface.</summary>
    public void Add(double theTol, ref Box theBox) => theBox.Add(Box(theTol));

    /// <summary>Add bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public void Add(double theUMin, double theUMax, double theVMin, double theVMax, double theTol, ref Box theBox)
        => theBox.Add(Box(theUMin, theUMax, theVMin, theVMax, theTol));

    /// <summary>Add precise bounding box for full surface.</summary>
    public void AddOptimal(double theTol, ref Box theBox) => theBox.Add(BoxOptimal(theTol));

    /// <summary>Add precise bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public void AddOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol, ref Box theBox)
        => theBox.Add(BoxOptimal(theUMin, theUMax, theVMin, theVMax, theTol));
}
