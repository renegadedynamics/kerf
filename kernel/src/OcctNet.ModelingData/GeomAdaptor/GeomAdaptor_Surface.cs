// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Surface.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// An interface between the services provided by any surface from the package Geom and those required of the
/// surface by algorithms which use it. Creation of the loaded surface: the surface is C1 by piece.
/// </summary>
/// <remarks>
/// <para>Polynomial coefficients of B-spline and Bezier surfaces are cached in the adaptor (a
/// <see cref="SurfaceCache"/>, rebuilt when the evaluated span changes), so evaluating one adaptor from several
/// threads at once is not safe; give each thread its own (<see cref="ShallowCopy"/> doesn't copy the cache).</para>
/// <para>A rectangular trimmed surface is replaced by its basis surface (with the given bounds); the surface and
/// the surfaces returned by <see cref="BSpline"/>, <see cref="Bezier"/> and <see cref="Surface"/> are the shared
/// geometry: the adaptor never changes it.</para>
/// </remarks>
public class GeomAdaptor_Surface : Adaptor3d_Surface
{
    private static readonly double PosTol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.5;

    /// <summary>Evaluation data of a surface of linear extrusion.</summary>
    internal sealed class ExtrusionData
    {
        /// <summary>Adaptor for basis curve.</summary>
        public Adaptor3d_Curve BasisCurve = null!;

        /// <summary>Extrusion direction XYZ (normalized).</summary>
        public Xyz Direction;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public GeomEval_RepSurfaceDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a surface of revolution.</summary>
    internal sealed class RevolutionData
    {
        /// <summary>Adaptor for basis curve.</summary>
        public Adaptor3d_Curve BasisCurve = null!;

        /// <summary>Revolution axis.</summary>
        public Axis3D Axis;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public GeomEval_RepSurfaceDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of an offset surface.</summary>
    internal sealed class OffsetData
    {
        /// <summary>Adaptor for basis surface.</summary>
        public GeomAdaptor_Surface BasisAdaptor = null!;

        /// <summary>Adaptor for equivalent surface (if exists).</summary>
        public GeomAdaptor_Surface? EquivalentAdaptor;

        /// <summary>Original offset surface for osculating queries.</summary>
        public Geom_OffsetSurface OffsetSurface = null!;

        /// <summary>Offset distance.</summary>
        public double Offset;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public GeomEval_RepSurfaceDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a Bezier surface.</summary>
    internal sealed class BezierData
    {
        /// <summary>Bezier surface to prevent downcasts.</summary>
        public Geom_BezierSurface Surface = null!;

        /// <summary>Cached data for evaluation.</summary>
        public SurfaceCache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public GeomEval_RepSurfaceDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a B-spline surface.</summary>
    internal sealed class BSplineData
    {
        /// <summary>BSpline surface to prevent downcasts.</summary>
        public Geom_BSplineSurface Surface = null!;

        /// <summary>Cached data for evaluation.</summary>
        public SurfaceCache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the surface: don't modify).</summary>
        public GeomEval_RepSurfaceDesc.Base? EvalRep;
    }

    protected Geom_Surface? mySurface;
    protected double myUFirst;
    protected double myULast;
    protected double myVFirst;
    protected double myVLast;
    protected double myTolU;
    protected double myTolV;
    protected SurfaceKind mySurfaceType;

    // OCCT's std::variant<monostate, gp_Pln, gp_Cylinder, gp_Cone, gp_Sphere, gp_Torus, ExtrusionData,
    // RevolutionData, OffsetData, BezierData, BSplineData>: null, a boxed gp_* value or one of the data classes.
    private protected object? mySurfaceData;

    /// <summary>An adaptor without a surface.</summary>
    public GeomAdaptor_Surface()
    {
        myUFirst = 0.0;
        myULast = 0.0;
        myVFirst = 0.0;
        myVLast = 0.0;
        myTolU = 0.0;
        myTolV = 0.0;
        mySurfaceType = OtherSurface;
    }

    /// <summary>An adaptor of theSurf over its bounds.</summary>
    /// <exception cref="NullObjectException">theSurf is null.</exception>
    public GeomAdaptor_Surface(Geom_Surface theSurf)
    {
        myTolU = 0.0;
        myTolV = 0.0;
        Load(theSurf);
    }

    /// <summary>
    /// An adaptor of theSurf over [theUFirst, theULast] x [theVFirst, theVLast]. Evaluation snaps parameters
    /// within theTolU (theTolV) of a bound to the bound.
    /// </summary>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast or theVFirst &gt; theVLast.</exception>
    public GeomAdaptor_Surface(Geom_Surface theSurf,
                               double theUFirst,
                               double theULast,
                               double theVFirst,
                               double theVLast,
                               double theTolU = 0.0,
                               double theTolV = 0.0)
    {
        Load(theSurf, theUFirst, theULast, theVFirst, theVLast, theTolU, theTolV);
    }

    private static bool hasEvalRep(object? theData)
    {
        if (theData is BezierData aBezierData)
        {
            return aBezierData.EvalRep is not null;
        }
        if (theData is BSplineData aBSplineData)
        {
            return aBSplineData.EvalRep is not null;
        }
        if (theData is ExtrusionData anExtrusionData)
        {
            return anExtrusionData.EvalRep is not null;
        }
        if (theData is RevolutionData aRevolutionData)
        {
            return aRevolutionData.EvalRep is not null;
        }
        if (theData is OffsetData anOffsetData)
        {
            return anOffsetData.EvalRep is not null;
        }
        return false;
    }

    private static Continuity LocalContinuity(int Degree,
                                                 int Nb,
                                                 NCollection_Array1<double> TK,
                                                 NCollection_Array1<int> TM,
                                                 double PFirst,
                                                 double PLast,
                                                 bool IsPeriodic)
    {
        Standard_DomainError_Raise_if((TK.Length() != Nb || TM.Length() != Nb), " ");
        int Index1;
        int Index2;
        double newFirst, newLast;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(Degree, TK, TM, PFirst, IsPeriodic, 1, Nb, out Index1, out newFirst);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(Degree, TK, TM, PLast, IsPeriodic, 1, Nb, out Index2, out newLast);
        double EpsKnot = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (Math.Abs(newFirst - TK[Index1 + 1]) < EpsKnot)
        {
            Index1++;
        }
        if (Math.Abs(newLast - TK[Index2]) < EpsKnot)
        {
            Index2--;
        }
        // Handle periodic curves.
        if ((IsPeriodic) && (Index1 == Nb))
        {
            Index1 = 1;
        }

        if (Index2 != Index1)
        {
            int i, Multmax = TM[Index1 + 1];
            for (i = Index1 + 1; i <= Index2; i++)
            {
                if (TM[i] > Multmax)
                {
                    Multmax = TM[i];
                }
            }
            Multmax = Degree - Multmax;
            if (Multmax <= 0)
            {
                return C0;
            }
            switch (Multmax)
            {
                case 1:
                    return C1;
                case 2:
                    return C2;
                case 3:
                    return C3;
            }
        }
        return CN;
    }

    // Offset surface evaluation with retry mechanism for singular points; uses the equivalent surface adaptor
    // for faster evaluation when available. Return true if evaluation succeeded.
    private static bool offsetD0(double theU, double theV, OffsetData theData, out Point3D theValue)
    {
        if (theData.EquivalentAdaptor is not null)
        {
            theValue = theData.EquivalentAdaptor.EvalD0(theU, theV);
            return true;
        }
        return Geom_OffsetSurfaceUtils.EvaluateD0(theU,
                                                  theV,
                                                  new Geom_OffsetSurfaceUtils.AdaptorBasis(theData.BasisAdaptor),
                                                  theData.Offset,
                                                  theData.OffsetSurface.OsculatingSurface(),
                                                  out theValue);
    }

    private static bool offsetD1(double theU, double theV, OffsetData theData, out Geom_Surface.ResD1 theResult)
    {
        if (theData.EquivalentAdaptor is not null)
        {
            theResult = theData.EquivalentAdaptor.EvalD1(theU, theV);
            return true;
        }
        return Geom_OffsetSurfaceUtils.EvaluateD1(theU,
                                                  theV,
                                                  new Geom_OffsetSurfaceUtils.AdaptorBasis(theData.BasisAdaptor),
                                                  theData.Offset,
                                                  theData.OffsetSurface.OsculatingSurface(),
                                                  out theResult.Point,
                                                  out theResult.D1U,
                                                  out theResult.D1V);
    }

    private static bool offsetD2(double theU, double theV, OffsetData theData, out Geom_Surface.ResD2 theResult)
    {
        if (theData.EquivalentAdaptor is not null)
        {
            theResult = theData.EquivalentAdaptor.EvalD2(theU, theV);
            return true;
        }
        return Geom_OffsetSurfaceUtils.EvaluateD2(theU,
                                                  theV,
                                                  new Geom_OffsetSurfaceUtils.AdaptorBasis(theData.BasisAdaptor),
                                                  theData.Offset,
                                                  theData.OffsetSurface.OsculatingSurface(),
                                                  out theResult.Point,
                                                  out theResult.D1U,
                                                  out theResult.D1V,
                                                  out theResult.D2U,
                                                  out theResult.D2V,
                                                  out theResult.D2UV);
    }

    private static bool offsetD3(double theU, double theV, OffsetData theData, out Geom_Surface.ResD3 theResult)
    {
        if (theData.EquivalentAdaptor is not null)
        {
            theResult = theData.EquivalentAdaptor.EvalD3(theU, theV);
            return true;
        }
        return Geom_OffsetSurfaceUtils.EvaluateD3(theU,
                                                  theV,
                                                  new Geom_OffsetSurfaceUtils.AdaptorBasis(theData.BasisAdaptor),
                                                  theData.Offset,
                                                  theData.OffsetSurface.OsculatingSurface(),
                                                  out theResult.Point,
                                                  out theResult.D1U,
                                                  out theResult.D1V,
                                                  out theResult.D2U,
                                                  out theResult.D2V,
                                                  out theResult.D2UV,
                                                  out theResult.D3U,
                                                  out theResult.D3V,
                                                  out theResult.D3UUV,
                                                  out theResult.D3UVV);
    }

    private static bool offsetDN(double theU, double theV, OffsetData theData, int theNu, int theNv,
                                 out Vector3D theResult)
    {
        if (theData.EquivalentAdaptor is not null)
        {
            theResult = theData.EquivalentAdaptor.EvalDN(theU, theV, theNu, theNv);
            return true;
        }
        return Geom_OffsetSurfaceUtils.EvaluateDN(theU,
                                                  theV,
                                                  theNu,
                                                  theNv,
                                                  new Geom_OffsetSurfaceUtils.AdaptorBasis(theData.BasisAdaptor),
                                                  theData.Offset,
                                                  theData.OffsetSurface.OsculatingSurface(),
                                                  out theResult);
    }

    /// <summary>A shallow copy: the same surface, new evaluation caches (and copies of the inner adaptors).</summary>
    public override Adaptor3d_Surface ShallowCopy()
    {
        GeomAdaptor_Surface aCopy = new GeomAdaptor_Surface();

        aCopy.mySurface = mySurface;
        aCopy.myUFirst = myUFirst;
        aCopy.myULast = myULast;
        aCopy.myVFirst = myVFirst;
        aCopy.myVLast = myVLast;
        aCopy.myTolU = myTolU;
        aCopy.myTolV = myTolV;
        aCopy.mySurfaceType = mySurfaceType;

        // Copy surface-specific evaluation data
        if (mySurfaceData is ExtrusionData anExtrusionData)
        {
            ExtrusionData aNewData = new ExtrusionData();
            aNewData.BasisCurve = anExtrusionData.BasisCurve.ShallowCopy();
            aNewData.Direction = anExtrusionData.Direction;
            aNewData.EvalRep = anExtrusionData.EvalRep;
            aCopy.mySurfaceData = aNewData;
        }
        else if (mySurfaceData is RevolutionData aRevolutionData)
        {
            RevolutionData aNewData = new RevolutionData();
            aNewData.BasisCurve = aRevolutionData.BasisCurve.ShallowCopy();
            aNewData.Axis = aRevolutionData.Axis;
            aNewData.EvalRep = aRevolutionData.EvalRep;
            aCopy.mySurfaceData = aNewData;
        }
        else if (mySurfaceData is OffsetData anOffsetData)
        {
            OffsetData aNewData = new OffsetData();
            aNewData.BasisAdaptor = (GeomAdaptor_Surface)anOffsetData.BasisAdaptor.ShallowCopy();
            if (anOffsetData.EquivalentAdaptor is not null)
            {
                aNewData.EquivalentAdaptor = (GeomAdaptor_Surface)anOffsetData.EquivalentAdaptor.ShallowCopy();
            }
            aNewData.OffsetSurface = anOffsetData.OffsetSurface; // Shared handle to original surface
            aNewData.Offset = anOffsetData.Offset;
            aNewData.EvalRep = anOffsetData.EvalRep;
            aCopy.mySurfaceData = aNewData;
        }
        else if (mySurfaceData is BSplineData aBSplineData)
        {
            BSplineData aNewData = new BSplineData();
            aNewData.Surface = aBSplineData.Surface;
            aNewData.EvalRep = aBSplineData.EvalRep;
            // Cache is not copied - will be rebuilt on demand
            aCopy.mySurfaceData = aNewData;
        }
        else if (mySurfaceData is BezierData aBezierData)
        {
            BezierData aNewData = new BezierData();
            aNewData.Surface = aBezierData.Surface;
            aNewData.EvalRep = aBezierData.EvalRep;
            aCopy.mySurfaceData = aNewData;
        }
        else
        {
            // Elementary surface types (gp_Pln, gp_Cylinder, etc.) are value types - direct copy (boxed values are
            // never changed in place)
            aCopy.mySurfaceData = mySurfaceData;
        }

        return aCopy;
    }

    /// <summary>Loads theSurf over its bounds.</summary>
    /// <exception cref="NullObjectException">theSurf is null.</exception>
    public void Load(Geom_Surface theSurf)
    {
        if (theSurf is null)
        {
            throw new NullObjectException("GeomAdaptor_Surface::Load");
        }
        theSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        load(theSurf, aU1, aU2, aV1, aV2);
    }

    /// <summary>Loads theSurf over [theUFirst, theULast] x [theVFirst, theVLast].</summary>
    /// <exception cref="NullObjectException">theSurf is null.</exception>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast or theVFirst &gt; theVLast.</exception>
    public void Load(Geom_Surface theSurf,
                     double theUFirst,
                     double theULast,
                     double theVFirst,
                     double theVLast,
                     double theTolU = 0.0,
                     double theTolV = 0.0)
    {
        if (theSurf is null)
        {
            throw new NullObjectException("GeomAdaptor_Surface::Load");
        }
        if (theUFirst > theULast || theVFirst > theVLast)
        {
            throw new ConstructionErrorException("GeomAdaptor_Surface::Load");
        }
        load(theSurf, theUFirst, theULast, theVFirst, theVLast, theTolU, theTolV);
    }

    /// <summary>The adapted surface (the basis surface for a trimmed surface), shared: null before a Load.</summary>
    public Geom_Surface? Surface() => mySurface;

    public override double FirstUParameter() => myUFirst;

    public override double LastUParameter() => myULast;

    public override double FirstVParameter() => myVFirst;

    public override double LastVParameter() => myVLast;

    /// <summary>The parameter bounds (virtual First/Last parameters).</summary>
    public void Bounds(out double theU1, out double theU2, out double theV1, out double theV2)
    {
        theU1 = FirstUParameter();
        theU2 = LastUParameter();
        theV1 = FirstVParameter();
        theV2 = LastVParameter();
    }

    /// <summary>The tolerance within which a U parameter is snapped to a bound.</summary>
    public double ToleranceU() => myTolU;

    /// <summary>The tolerance within which a V parameter is snapped to a bound.</summary>
    public double ToleranceV() => myTolV;

    private void load(Geom_Surface S,
                      double UFirst,
                      double ULast,
                      double VFirst,
                      double VLast,
                      double TolU = 0.0,
                      double TolV = 0.0)
    {
        myTolU = TolU;
        myTolV = TolV;
        myUFirst = UFirst;
        myULast = ULast;
        myVFirst = VFirst;
        myVLast = VLast;

        if (!ReferenceEquals(mySurface, S))
        {
            mySurface = S;
            mySurfaceData = null;

            Type TheType = S.DynamicType();
            if (TheType == typeof(Geom_RectangularTrimmedSurface))
            {
                Load(((Geom_RectangularTrimmedSurface)S).BasisSurface(), UFirst, ULast, VFirst, VLast);
            }
            else if (TheType == typeof(Geom_Plane))
            {
                mySurfaceType = global::Kerf.Geometry.SurfaceKind.Plane;
                mySurfaceData = ((Geom_Plane)S).Pln();
            }
            else if (TheType == typeof(Geom_CylindricalSurface))
            {
                mySurfaceType = global::Kerf.Geometry.SurfaceKind.Cylinder;
                mySurfaceData = ((Geom_CylindricalSurface)S).Cylinder();
            }
            else if (TheType == typeof(Geom_ConicalSurface))
            {
                mySurfaceType = global::Kerf.Geometry.SurfaceKind.Cone;
                mySurfaceData = ((Geom_ConicalSurface)S).Cone();
            }
            else if (TheType == typeof(Geom_SphericalSurface))
            {
                mySurfaceType = global::Kerf.Geometry.SurfaceKind.Sphere;
                mySurfaceData = ((Geom_SphericalSurface)S).Sphere();
            }
            else if (TheType == typeof(Geom_ToroidalSurface))
            {
                mySurfaceType = global::Kerf.Geometry.SurfaceKind.Torus;
                mySurfaceData = ((Geom_ToroidalSurface)S).Torus();
            }
            else if (TheType == typeof(Geom_SurfaceOfRevolution))
            {
                mySurfaceType = SurfaceOfRevolution;
                Geom_SurfaceOfRevolution aRevSurf = (Geom_SurfaceOfRevolution)mySurface;
                // Populate revolution surface data
                RevolutionData aRevData = new RevolutionData();
                aRevData.BasisCurve = new GeomAdaptor_Curve(aRevSurf.BasisCurve(), myVFirst, myVLast);
                aRevData.Axis = aRevSurf.Axis();
                aRevData.EvalRep = aRevSurf.EvalRepresentation();
                mySurfaceData = aRevData;
            }
            else if (TheType == typeof(Geom_SurfaceOfLinearExtrusion))
            {
                mySurfaceType = SurfaceOfExtrusion;
                Geom_SurfaceOfLinearExtrusion anExtSurf = (Geom_SurfaceOfLinearExtrusion)mySurface;
                // Populate extrusion surface data with XYZ for fast evaluation
                ExtrusionData anExtData = new ExtrusionData();
                anExtData.BasisCurve = new GeomAdaptor_Curve(anExtSurf.BasisCurve(), myUFirst, myULast);
                anExtData.Direction = anExtSurf.Direction().XYZ();
                anExtData.EvalRep = anExtSurf.EvalRepresentation();
                mySurfaceData = anExtData;
            }
            else if (TheType == typeof(Geom_BezierSurface))
            {
                mySurfaceType = BezierSurface;
                BezierData aBezierData = new BezierData();
                aBezierData.Surface = (Geom_BezierSurface)mySurface;
                aBezierData.EvalRep = aBezierData.Surface.EvalRepresentation();
                mySurfaceData = aBezierData;
            }
            else if (TheType == typeof(Geom_BSplineSurface))
            {
                mySurfaceType = BSplineSurface;
                BSplineData aBSplineData = new BSplineData();
                aBSplineData.Surface = (Geom_BSplineSurface)mySurface;
                aBSplineData.EvalRep = aBSplineData.Surface.EvalRepresentation();
                mySurfaceData = aBSplineData;
            }
            else if (TheType == typeof(Geom_OffsetSurface))
            {
                mySurfaceType = OffsetSurface;
                Geom_OffsetSurface anOffSurf = (Geom_OffsetSurface)mySurface;
                // Populate offset surface data - reuse the original surface for osculating queries
                OffsetData anOffsetData = new OffsetData();
                anOffsetData.BasisAdaptor = new GeomAdaptor_Surface(anOffSurf.BasisSurface(),
                                                                    myUFirst,
                                                                    myULast,
                                                                    myVFirst,
                                                                    myVLast,
                                                                    myTolU,
                                                                    myTolV);
                anOffsetData.OffsetSurface = anOffSurf;
                anOffsetData.Offset = anOffSurf.Offset();
                anOffsetData.EvalRep = anOffSurf.EvalRepresentation();
                // Check if equivalent canonical surface exists for faster evaluation
                Geom_Surface? anEquivSurf = anOffSurf.Surface();
                if (anEquivSurf is not null)
                {
                    anOffsetData.EquivalentAdaptor = new GeomAdaptor_Surface(anEquivSurf,
                                                                             myUFirst,
                                                                             myULast,
                                                                             myVFirst,
                                                                             myVLast,
                                                                             myTolU,
                                                                             myTolV);
                }
                mySurfaceData = anOffsetData;
            }
            else
            {
                mySurfaceType = OtherSurface;
            }
        }
    }

    //    --
    //    --     Global methods - Apply to the whole Surface.
    //    --

    /// <exception cref="NoSuchObjectException">The surface is not a Geom surface this adaptor knows.</exception>
    public override Continuity UContinuity()
    {
        switch (mySurfaceType)
        {
            case BSplineSurface:
            {
                Geom_BSplineSurface aBSpl = ((BSplineData)mySurfaceData!).Surface;
                NCollection_Array1<double> TK = new(aBSpl.InternalUKnots());
                NCollection_Array1<int> TM = new(aBSpl.InternalUMultiplicities());
                return LocalContinuity(aBSpl.UDegree(), aBSpl.NbUKnots(), TK, TM, myUFirst, myULast, IsUPeriodic());
            }
            case OffsetSurface:
            {
                switch (BasisSurface().UContinuity())
                {
                    case CN:
                    case C3:
                        return CN;
                    case G2:
                    case C2:
                        return C1;
                    case G1:
                    case C1:
                    case C0:
                        return C0;
                }
                throw new NoSuchObjectException("GeomAdaptor_Surface::UContinuity");
            }
            case SurfaceOfExtrusion:
            {
                return ((ExtrusionData)mySurfaceData!).BasisCurve.Continuity();
            }
            case OtherSurface:
                throw new NoSuchObjectException("GeomAdaptor_Surface::UContinuity");
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case BezierSurface:
            case SurfaceOfRevolution:
                break;
        }
        return CN;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a Geom surface this adaptor knows.</exception>
    public override Continuity VContinuity()
    {
        switch (mySurfaceType)
        {
            case BSplineSurface:
            {
                Geom_BSplineSurface aBSpl = ((BSplineData)mySurfaceData!).Surface;
                NCollection_Array1<double> TK = new(aBSpl.InternalVKnots());
                NCollection_Array1<int> TM = new(aBSpl.InternalVMultiplicities());
                return LocalContinuity(aBSpl.VDegree(), aBSpl.NbVKnots(), TK, TM, myVFirst, myVLast, IsVPeriodic());
            }
            case OffsetSurface:
            {
                switch (BasisSurface().VContinuity())
                {
                    case CN:
                    case C3:
                        return CN;
                    case G2:
                    case C2:
                        return C1;
                    case G1:
                    case C1:
                    case C0:
                        return C0;
                }
                throw new NoSuchObjectException("GeomAdaptor_Surface::VContinuity");
            }
            case SurfaceOfRevolution:
            {
                return ((RevolutionData)mySurfaceData!).BasisCurve.Continuity();
            }
            case OtherSurface:
                throw new NoSuchObjectException("GeomAdaptor_Surface::VContinuity");
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case BezierSurface:
            case SurfaceOfExtrusion:
                break;
        }
        return CN;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on an offset surface.</exception>
    public override int NbUIntervals(Continuity S)
    {
        switch (mySurfaceType)
        {
            case BSplineSurface:
            {
                Geom_BSplineSurface aBSpl = ((BSplineData)mySurfaceData!).Surface;
                GeomAdaptor_Curve myBasisCurve = new(aBSpl.VIso(aBSpl.VKnot(aBSpl.FirstVKnotIndex())), myUFirst, myULast);
                return myBasisCurve.NbIntervals(S);
            }
            case SurfaceOfExtrusion:
            {
                Geom_SurfaceOfLinearExtrusion myExtSurf = (Geom_SurfaceOfLinearExtrusion)mySurface!;
                GeomAdaptor_Curve myBasisCurve = new(myExtSurf.BasisCurve(), myUFirst, myULast);
                if (myBasisCurve.GetType() == BSplineCurve)
                {
                    return myBasisCurve.NbIntervals(S);
                }
                break;
            }
            case OffsetSurface:
            {
                Continuity BaseS = CN;
                switch (S)
                {
                    case G1:
                    case G2:
                        throw new DomainErrorException("GeomAdaptor_Curve::NbUIntervals");
                    case C0:
                        BaseS = C1;
                        break;
                    case C1:
                        BaseS = C2;
                        break;
                    case C2:
                        BaseS = C3;
                        break;
                    case C3:
                    case CN:
                        break;
                }
                return ((OffsetData)mySurfaceData!).BasisAdaptor.NbUIntervals(BaseS);
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case BezierSurface:
            case OtherSurface:
            case SurfaceOfRevolution:
                break;
        }
        return 1;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on an offset surface.</exception>
    public override int NbVIntervals(Continuity S)
    {
        switch (mySurfaceType)
        {
            case BSplineSurface:
            {
                Geom_BSplineSurface aBSpl = ((BSplineData)mySurfaceData!).Surface;
                GeomAdaptor_Curve myBasisCurve = new(aBSpl.UIso(aBSpl.UKnot(aBSpl.FirstUKnotIndex())), myVFirst, myVLast);
                return myBasisCurve.NbIntervals(S);
            }
            case SurfaceOfRevolution:
            {
                Geom_SurfaceOfRevolution myRevSurf = (Geom_SurfaceOfRevolution)mySurface!;
                GeomAdaptor_Curve myBasisCurve = new(myRevSurf.BasisCurve(), myVFirst, myVLast);
                if (myBasisCurve.GetType() == BSplineCurve)
                {
                    return myBasisCurve.NbIntervals(S);
                }
                break;
            }
            case OffsetSurface:
            {
                Continuity BaseS = CN;
                switch (S)
                {
                    case G1:
                    case G2:
                        throw new DomainErrorException("GeomAdaptor_Curve::NbVIntervals");
                    case C0:
                        BaseS = C1;
                        break;
                    case C1:
                        BaseS = C2;
                        break;
                    case C2:
                        BaseS = C3;
                        break;
                    case C3:
                    case CN:
                        break;
                }
                return ((OffsetData)mySurfaceData!).BasisAdaptor.NbVIntervals(BaseS);
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case BezierSurface:
            case OtherSurface:
            case SurfaceOfExtrusion:
                break;
        }
        return 1;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on an offset surface.</exception>
    public override void UIntervals(NCollection_Array1<double> T, Continuity S)
    {
        switch (mySurfaceType)
        {
            case BSplineSurface:
            {
                Geom_BSplineSurface aBSpl = ((BSplineData)mySurfaceData!).Surface;
                GeomAdaptor_Curve myBasisCurve = new(aBSpl.VIso(aBSpl.VKnot(aBSpl.FirstVKnotIndex())), myUFirst, myULast);
                myBasisCurve.Intervals(T, S);
                return;
            }
            case SurfaceOfExtrusion:
            {
                GeomAdaptor_Curve myBasisCurve = new(((Geom_SurfaceOfLinearExtrusion)mySurface!).BasisCurve(),
                                                     myUFirst,
                                                     myULast);
                if (myBasisCurve.GetType() == BSplineCurve)
                {
                    myBasisCurve.Intervals(T, S);
                    return;
                }
                break;
            }
            case OffsetSurface:
            {
                Continuity BaseS = CN;
                switch (S)
                {
                    case G1:
                    case G2:
                        throw new DomainErrorException("GeomAdaptor_Curve::UIntervals");
                    case C0:
                        BaseS = C1;
                        break;
                    case C1:
                        BaseS = C2;
                        break;
                    case C2:
                        BaseS = C3;
                        break;
                    case C3:
                    case CN:
                        break;
                }
                ((OffsetData)mySurfaceData!).BasisAdaptor.UIntervals(T, BaseS);
                return;
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case BezierSurface:
            case OtherSurface:
            case SurfaceOfRevolution:
                break;
        }

        T[T.Lower()] = myUFirst;
        T[T.Lower() + 1] = myULast;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on an offset surface.</exception>
    public override void VIntervals(NCollection_Array1<double> T, Continuity S)
    {
        switch (mySurfaceType)
        {
            case BSplineSurface:
            {
                Geom_BSplineSurface aBSpl = ((BSplineData)mySurfaceData!).Surface;
                GeomAdaptor_Curve myBasisCurve = new(aBSpl.UIso(aBSpl.UKnot(aBSpl.FirstUKnotIndex())), myVFirst, myVLast);
                myBasisCurve.Intervals(T, S);
                return;
            }
            case SurfaceOfRevolution:
            {
                Geom_SurfaceOfRevolution myRevSurf = (Geom_SurfaceOfRevolution)mySurface!;
                GeomAdaptor_Curve myBasisCurve = new(myRevSurf.BasisCurve(), myVFirst, myVLast);
                if (myBasisCurve.GetType() == BSplineCurve)
                {
                    myBasisCurve.Intervals(T, S);
                    return;
                }
                break;
            }
            case OffsetSurface:
            {
                Continuity BaseS = CN;
                switch (S)
                {
                    case G1:
                    case G2:
                        throw new DomainErrorException("GeomAdaptor_Curve::VIntervals");
                    case C0:
                        BaseS = C1;
                        break;
                    case C1:
                        BaseS = C2;
                        break;
                    case C2:
                        BaseS = C3;
                        break;
                    case C3:
                    case CN:
                        break;
                }
                ((OffsetData)mySurfaceData!).BasisAdaptor.VIntervals(T, BaseS);
                return;
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
            case BezierSurface:
            case OtherSurface:
            case SurfaceOfExtrusion:
                break;
        }

        T[T.Lower()] = myVFirst;
        T[T.Lower() + 1] = myVLast;
    }

    /// <summary>A new adaptor of the same surface between First and Last in U.</summary>
    public override Adaptor3d_Surface UTrim(double First, double Last, double Tol)
    {
        return new GeomAdaptor_Surface(mySurface!, First, Last, myVFirst, myVLast, Tol, myTolV);
    }

    /// <summary>A new adaptor of the same surface between First and Last in V.</summary>
    public override Adaptor3d_Surface VTrim(double First, double Last, double Tol)
    {
        return new GeomAdaptor_Surface(mySurface!, myUFirst, myULast, First, Last, myTolU, Tol);
    }

    /// <summary>True if the surface is U-closed and the adaptor spans its whole U range (a period if periodic).</summary>
    public override bool IsUClosed()
    {
        if (!mySurface!.IsUClosed())
        {
            return false;
        }

        mySurface.Bounds(out double U1, out double U2, out _, out _);
        if (mySurface.IsUPeriodic())
        {
            return (Math.Abs(Math.Abs(U1 - U2) - Math.Abs(myUFirst - myULast)) < global::Kerf.Engine.Foundation.Precision.PConfusion());
        }

        return (Math.Abs(U1 - myUFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion() && Math.Abs(U2 - myULast) < global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    /// <summary>True if the surface is V-closed and the adaptor spans its whole V range (a period if periodic).</summary>
    public override bool IsVClosed()
    {
        if (!mySurface!.IsVClosed())
        {
            return false;
        }

        mySurface.Bounds(out _, out _, out double V1, out double V2);
        if (mySurface.IsVPeriodic())
        {
            return (Math.Abs(Math.Abs(V1 - V2) - Math.Abs(myVFirst - myVLast)) < global::Kerf.Engine.Foundation.Precision.PConfusion());
        }

        return (Math.Abs(V1 - myVFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion() && Math.Abs(V2 - myVLast) < global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override bool IsUPeriodic() => (mySurface!.IsUPeriodic());

    /// <exception cref="NoSuchObjectException">The surface is not U-periodic.</exception>
    public override double UPeriod()
    {
        Standard_NoSuchObject_Raise_if(!IsUPeriodic(), " ");
        return mySurface!.UPeriod();
    }

    public override bool IsVPeriodic() => (mySurface!.IsVPeriodic());

    /// <exception cref="NoSuchObjectException">The surface is not V-periodic.</exception>
    public override double VPeriod()
    {
        Standard_NoSuchObject_Raise_if(!IsVPeriodic(), " ");
        return mySurface!.VPeriod();
    }

    private void RebuildCache(double theU, double theV)
    {
        if (mySurfaceType == BezierSurface)
        {
            // Create cache for Bezier
            BezierData aBezData = (BezierData)mySurfaceData!;
            Geom_BezierSurface aBezier = (Geom_BezierSurface)mySurface!;
            int aDegU = aBezier.UDegree();
            int aDegV = aBezier.VDegree();
            NCollection_Array1<double> aFlatKnotsU = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(aDegU));
            NCollection_Array1<double> aFlatKnotsV = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(aDegV));
            if (aBezData.Cache is null)
            {
                aBezData.Cache = new SurfaceCache(aDegU,
                                                    aBezier.IsUPeriodic(),
                                                    aFlatKnotsU,
                                                    aDegV,
                                                    aBezier.IsVPeriodic(),
                                                    aFlatKnotsV,
                                                    aBezier.InternalWeights());
            }
            aBezData.Cache.BuildCache(theU,
                                      theV,
                                      aFlatKnotsU,
                                      aFlatKnotsV,
                                      aBezier.InternalPoles(),
                                      aBezier.InternalWeights());
        }
        else if (mySurfaceType == BSplineSurface)
        {
            // Create cache for B-spline
            BSplineData aBSplData = (BSplineData)mySurfaceData!;
            Geom_BSplineSurface aBSpl = aBSplData.Surface;
            if (aBSplData.Cache is null)
            {
                aBSplData.Cache = new SurfaceCache(aBSpl.UDegree(),
                                                     aBSpl.IsUPeriodic(),
                                                     aBSpl.InternalUKnotSequence(),
                                                     aBSpl.VDegree(),
                                                     aBSpl.IsVPeriodic(),
                                                     aBSpl.InternalVKnotSequence(),
                                                     aBSpl.InternalWeights());
            }
            aBSplData.Cache.BuildCache(theU,
                                       theV,
                                       aBSpl.InternalUKnotSequence(),
                                       aBSpl.InternalVKnotSequence(),
                                       aBSpl.InternalPoles(),
                                       aBSpl.InternalWeights());
        }
    }

    /// <exception cref="Geom_UndefinedValue">The point is undefined at (theU, theV) (swept or offset surfaces).</exception>
    public sealed override Point3D EvalD0(double theU, double theV)
    {
        double U = theU;
        double V = theV;
        Point3D P;
        switch (mySurfaceType)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Plane3D)mySurfaceData!, out P);
                return P;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Cylinder)mySurfaceData!, out P);
                return P;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Cone)mySurfaceData!, out P);
                return P;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Sphere)mySurfaceData!, out P);
                return P;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.D0(U, V, (Torus)mySurfaceData!, out P);
                return P;

            case BezierSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD0(U, V);
                }
                BezierData aBezData = (BezierData)mySurfaceData!;
                if (aBezData.Cache is null)
                {
                    RebuildCache(U, V);
                }
                aBezData.Cache!.D0(U, V, out P);
                return P;
            }
            case BSplineSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD0(U, V);
                }
                BSplineData aBSplData = (BSplineData)mySurfaceData!;
                if (aBSplData.Cache is null || !aBSplData.Cache.IsCacheValid(U, V))
                {
                    RebuildCache(U, V);
                }
                aBSplData.Cache!.D0(U, V, out P);
                return P;
            }

            case SurfaceOfExtrusion:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD0(U, V);
                }
                ExtrusionData anExtData = (ExtrusionData)mySurfaceData!;
                if (!Geom_ExtrusionUtils.D0(U, V, anExtData.BasisCurve, anExtData.Direction, out P))
                {
                    throw new Geom_UndefinedValue("GeomAdaptor_Surface::EvalD0: evaluation failed");
                }
                return P;
            }

            case SurfaceOfRevolution:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD0(U, V);
                }
                RevolutionData aRevData = (RevolutionData)mySurfaceData!;
                if (!Geom_RevolutionUtils.D0(U, V, aRevData.BasisCurve, aRevData.Axis, out P))
                {
                    throw new Geom_UndefinedValue("GeomAdaptor_Surface::EvalD0: evaluation failed");
                }
                return P;
            }

            case OffsetSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD0(U, V);
                }
                OffsetData anOffData = (OffsetData)mySurfaceData!;
                if (!offsetD0(U, V, anOffData, out P))
                {
                    throw new Geom_UndefinedValue("GeomAdaptor_Surface::EvalD0: evaluation failed");
                }
                return P;
            }

            default:
                return mySurface!.EvalD0(U, V);
        }
    }

    // Snaps the parameters within the tolerances of a bound to the bound, and tells which side (1 first, -1 last).
    private void boundSides(double U, double V, out double u, out double v, out int USide, out int VSide)
    {
        USide = 0;
        VSide = 0;
        u = U;
        v = V;
        if (Math.Abs(U - myUFirst) <= myTolU)
        {
            USide = 1;
            u = myUFirst;
        }
        else if (Math.Abs(U - myULast) <= myTolU)
        {
            USide = -1;
            u = myULast;
        }
        if (Math.Abs(V - myVFirst) <= myTolV)
        {
            VSide = 1;
            v = myVFirst;
        }
        else if (Math.Abs(V - myVLast) <= myTolV)
        {
            VSide = -1;
            v = myVLast;
        }
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives are undefined at (theU, theV) (swept or offset surfaces).</exception>
    public sealed override Geom_Surface.ResD1 EvalD1(double theU, double theV)
    {
        double U = theU;
        double V = theV;
        Geom_Surface.ResD1 aResult;
        boundSides(U, V, out double u, out double v, out int USide, out int VSide);

        switch (mySurfaceType)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Plane3D)mySurfaceData!, out aResult.Point, out aResult.D1U, out aResult.D1V);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Cylinder)mySurfaceData!, out aResult.Point, out aResult.D1U, out aResult.D1V);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Cone)mySurfaceData!, out aResult.Point, out aResult.D1U, out aResult.D1V);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Sphere)mySurfaceData!, out aResult.Point, out aResult.D1U, out aResult.D1V);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Torus)mySurfaceData!, out aResult.Point, out aResult.D1U, out aResult.D1V);
                return aResult;

            case BezierSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD1(U, V);
                }
                BezierData aBezData = (BezierData)mySurfaceData!;
                if (aBezData.Cache is null)
                {
                    RebuildCache(U, V);
                }
                aBezData.Cache!.D1(U, V, out aResult.Point, out aResult.D1U, out aResult.D1V);
                return aResult;
            }
            case BSplineSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD1(U, V);
                }
                BSplineData aBSplData = (BSplineData)mySurfaceData!;
                Geom_BSplineSurface aBSpl = aBSplData.Surface;
                int Ideb = 0, Ifin = 0, IVdeb = 0, IVfin = 0;
                if ((USide != 0 || VSide != 0) && IfUVBound(u, v, out Ideb, out Ifin, out IVdeb, out IVfin, USide, VSide))
                {
                    aBSpl.LocalD1(u, v, Ideb, Ifin, IVdeb, IVfin, out aResult.Point, out aResult.D1U, out aResult.D1V);
                }
                else
                {
                    if (aBSplData.Cache is null || !aBSplData.Cache.IsCacheValid(U, V))
                    {
                        RebuildCache(U, V);
                    }
                    aBSplData.Cache!.D1(U, V, out aResult.Point, out aResult.D1U, out aResult.D1V);
                }
                return aResult;
            }

            case SurfaceOfExtrusion:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD1(U, V);
                }
                ExtrusionData anExtData = (ExtrusionData)mySurfaceData!;
                if (!Geom_ExtrusionUtils.D1(u,
                                            v,
                                            anExtData.BasisCurve,
                                            anExtData.Direction,
                                            out aResult.Point,
                                            out aResult.D1U,
                                            out aResult.D1V))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalD1: evaluation failed");
                }
                return aResult;
            }

            case SurfaceOfRevolution:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD1(U, V);
                }
                RevolutionData aRevData = (RevolutionData)mySurfaceData!;
                if (!Geom_RevolutionUtils.D1(u,
                                             v,
                                             aRevData.BasisCurve,
                                             aRevData.Axis,
                                             out aResult.Point,
                                             out aResult.D1U,
                                             out aResult.D1V))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalD1: evaluation failed");
                }
                return aResult;
            }

            case OffsetSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD1(U, V);
                }
                OffsetData anOffData = (OffsetData)mySurfaceData!;
                if (!offsetD1(u, v, anOffData, out aResult))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalD1: evaluation failed");
                }
                return aResult;
            }

            default:
                return mySurface!.EvalD1(u, v);
        }
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives are undefined at (theU, theV) (swept or offset surfaces).</exception>
    public sealed override Geom_Surface.ResD2 EvalD2(double theU, double theV)
    {
        double U = theU;
        double V = theV;
        Geom_Surface.ResD2 aResult;
        boundSides(U, V, out double u, out double v, out int USide, out int VSide);

        switch (mySurfaceType)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Plane3D)mySurfaceData!, out aResult.Point, out aResult.D1U, out aResult.D1V);
                aResult.D2U = new Vector3D(0.0, 0.0, 0.0);
                aResult.D2V = new Vector3D(0.0, 0.0, 0.0);
                aResult.D2UV = new Vector3D(0.0, 0.0, 0.0);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.D2(u,
                          v,
                          (Cylinder)mySurfaceData!,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.D2(u,
                          v,
                          (Cone)mySurfaceData!,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.D2(u,
                          v,
                          (Sphere)mySurfaceData!,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.D2(u,
                          v,
                          (Torus)mySurfaceData!,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV);
                return aResult;

            case BezierSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD2(U, V);
                }
                BezierData aBezData = (BezierData)mySurfaceData!;
                if (aBezData.Cache is null)
                {
                    RebuildCache(U, V);
                }
                aBezData.Cache!.D2(U,
                                   V,
                                   out aResult.Point,
                                   out aResult.D1U,
                                   out aResult.D1V,
                                   out aResult.D2U,
                                   out aResult.D2V,
                                   out aResult.D2UV);
                return aResult;
            }
            case BSplineSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD2(U, V);
                }
                BSplineData aBSplData = (BSplineData)mySurfaceData!;
                Geom_BSplineSurface aBSpl = aBSplData.Surface;
                int Ideb = 0, Ifin = 0, IVdeb = 0, IVfin = 0;
                if ((USide != 0 || VSide != 0) && IfUVBound(u, v, out Ideb, out Ifin, out IVdeb, out IVfin, USide, VSide))
                {
                    aBSpl.LocalD2(u,
                                  v,
                                  Ideb,
                                  Ifin,
                                  IVdeb,
                                  IVfin,
                                  out aResult.Point,
                                  out aResult.D1U,
                                  out aResult.D1V,
                                  out aResult.D2U,
                                  out aResult.D2V,
                                  out aResult.D2UV);
                }
                else
                {
                    if (aBSplData.Cache is null || !aBSplData.Cache.IsCacheValid(U, V))
                    {
                        RebuildCache(U, V);
                    }
                    aBSplData.Cache!.D2(U,
                                        V,
                                        out aResult.Point,
                                        out aResult.D1U,
                                        out aResult.D1V,
                                        out aResult.D2U,
                                        out aResult.D2V,
                                        out aResult.D2UV);
                }
                return aResult;
            }

            case SurfaceOfExtrusion:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD2(U, V);
                }
                ExtrusionData anExtData = (ExtrusionData)mySurfaceData!;
                if (!Geom_ExtrusionUtils.D2(u,
                                            v,
                                            anExtData.BasisCurve,
                                            anExtData.Direction,
                                            out aResult.Point,
                                            out aResult.D1U,
                                            out aResult.D1V,
                                            out aResult.D2U,
                                            out aResult.D2V,
                                            out aResult.D2UV))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalD2: evaluation failed");
                }
                return aResult;
            }

            case SurfaceOfRevolution:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD2(U, V);
                }
                RevolutionData aRevData = (RevolutionData)mySurfaceData!;
                if (!Geom_RevolutionUtils.D2(u,
                                             v,
                                             aRevData.BasisCurve,
                                             aRevData.Axis,
                                             out aResult.Point,
                                             out aResult.D1U,
                                             out aResult.D1V,
                                             out aResult.D2U,
                                             out aResult.D2V,
                                             out aResult.D2UV))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalD2: evaluation failed");
                }
                return aResult;
            }

            case OffsetSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD2(U, V);
                }
                OffsetData anOffData = (OffsetData)mySurfaceData!;
                if (!offsetD2(u, v, anOffData, out aResult))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalD2: evaluation failed");
                }
                return aResult;
            }

            default:
                return mySurface!.EvalD2(u, v);
        }
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives are undefined at (theU, theV) (swept or offset surfaces).</exception>
    public sealed override Geom_Surface.ResD3 EvalD3(double theU, double theV)
    {
        double U = theU;
        double V = theV;
        Geom_Surface.ResD3 aResult;
        boundSides(U, V, out double u, out double v, out int USide, out int VSide);

        switch (mySurfaceType)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.D1(u, v, (Plane3D)mySurfaceData!, out Point3D aPlnP, out Vector3D aPlnD1U, out Vector3D aPlnD1V);
                aResult = new Geom_Surface.ResD3();
                aResult.Point = aPlnP;
                aResult.D1U = aPlnD1U;
                aResult.D1V = aPlnD1V;
                aResult.D2U = new Vector3D(0.0, 0.0, 0.0);
                aResult.D2V = new Vector3D(0.0, 0.0, 0.0);
                aResult.D2UV = new Vector3D(0.0, 0.0, 0.0);
                aResult.D3U = new Vector3D(0.0, 0.0, 0.0);
                aResult.D3V = new Vector3D(0.0, 0.0, 0.0);
                aResult.D3UUV = new Vector3D(0.0, 0.0, 0.0);
                aResult.D3UVV = new Vector3D(0.0, 0.0, 0.0);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.D3(u,
                          v,
                          (Cylinder)mySurfaceData!,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV,
                          out aResult.D3U,
                          out aResult.D3V,
                          out aResult.D3UUV,
                          out aResult.D3UVV);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.D3(u,
                          v,
                          (Cone)mySurfaceData!,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV,
                          out aResult.D3U,
                          out aResult.D3V,
                          out aResult.D3UUV,
                          out aResult.D3UVV);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.D3(u,
                          v,
                          (Sphere)mySurfaceData!,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV,
                          out aResult.D3U,
                          out aResult.D3V,
                          out aResult.D3UUV,
                          out aResult.D3UVV);
                return aResult;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.D3(u,
                          v,
                          (Torus)mySurfaceData!,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV,
                          out aResult.D3U,
                          out aResult.D3V,
                          out aResult.D3UUV,
                          out aResult.D3UVV);
                return aResult;

            case BSplineSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD3(U, V);
                }
                Geom_BSplineSurface aBSpl = ((BSplineData)mySurfaceData!).Surface;
                if ((USide == 0) && (VSide == 0))
                {
                    return aBSpl.EvalD3(u, v);
                }
                else
                {
                    if (IfUVBound(u, v, out int Ideb, out int Ifin, out int IVdeb, out int IVfin, USide, VSide))
                    {
                        aBSpl.LocalD3(u,
                                      v,
                                      Ideb,
                                      Ifin,
                                      IVdeb,
                                      IVfin,
                                      out aResult.Point,
                                      out aResult.D1U,
                                      out aResult.D1V,
                                      out aResult.D2U,
                                      out aResult.D2V,
                                      out aResult.D2UV,
                                      out aResult.D3U,
                                      out aResult.D3V,
                                      out aResult.D3UUV,
                                      out aResult.D3UVV);
                    }
                    else
                    {
                        return aBSpl.EvalD3(u, v);
                    }
                }
                return aResult;
            }

            case SurfaceOfExtrusion:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD3(U, V);
                }
                ExtrusionData anExtData = (ExtrusionData)mySurfaceData!;
                if (!Geom_ExtrusionUtils.D3(u,
                                            v,
                                            anExtData.BasisCurve,
                                            anExtData.Direction,
                                            out aResult.Point,
                                            out aResult.D1U,
                                            out aResult.D1V,
                                            out aResult.D2U,
                                            out aResult.D2V,
                                            out aResult.D2UV,
                                            out aResult.D3U,
                                            out aResult.D3V,
                                            out aResult.D3UUV,
                                            out aResult.D3UVV))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalD3: evaluation failed");
                }
                return aResult;
            }

            case SurfaceOfRevolution:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD3(U, V);
                }
                RevolutionData aRevData = (RevolutionData)mySurfaceData!;
                if (!Geom_RevolutionUtils.D3(u,
                                             v,
                                             aRevData.BasisCurve,
                                             aRevData.Axis,
                                             out aResult.Point,
                                             out aResult.D1U,
                                             out aResult.D1V,
                                             out aResult.D2U,
                                             out aResult.D2V,
                                             out aResult.D2UV,
                                             out aResult.D3U,
                                             out aResult.D3V,
                                             out aResult.D3UUV,
                                             out aResult.D3UVV))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalD3: evaluation failed");
                }
                return aResult;
            }

            case OffsetSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalD3(U, V);
                }
                OffsetData anOffData = (OffsetData)mySurfaceData!;
                if (!offsetD3(u, v, anOffData, out aResult))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalD3: evaluation failed");
                }
                return aResult;
            }

            default:
                return mySurface!.EvalD3(u, v);
        }
    }

    /// <exception cref="Geom_UndefinedDerivative">
    /// The derivative is undefined at (theU, theV), or the orders are invalid.
    /// </exception>
    public sealed override Vector3D EvalDN(double theU, double theV, int theNu, int theNv)
    {
        double U = theU;
        double V = theV;
        int Nu = theNu;
        int Nv = theNv;
        boundSides(U, V, out double u, out double v, out int USide, out int VSide);

        switch (mySurfaceType)
        {
            case BSplineSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalDN(U, V, Nu, Nv);
                }
                Geom_BSplineSurface aBSpl = ((BSplineData)mySurfaceData!).Surface;
                if ((USide == 0) && (VSide == 0))
                {
                    return aBSpl.EvalDN(u, v, Nu, Nv);
                }
                else
                {
                    if (IfUVBound(u, v, out int Ideb, out int Ifin, out int IVdeb, out int IVfin, USide, VSide))
                    {
                        return aBSpl.LocalDN(u, v, Ideb, Ifin, IVdeb, IVfin, Nu, Nv);
                    }
                    else
                    {
                        return aBSpl.EvalDN(u, v, Nu, Nv);
                    }
                }
            }

            case SurfaceOfExtrusion:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalDN(U, V, Nu, Nv);
                }
                ExtrusionData anExtData = (ExtrusionData)mySurfaceData!;
                if (!Geom_ExtrusionUtils.DN(u, anExtData.BasisCurve, anExtData.Direction, Nu, Nv, out Vector3D aDN))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalDN: evaluation failed");
                }
                return aDN;
            }

            case SurfaceOfRevolution:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalDN(U, V, Nu, Nv);
                }
                RevolutionData aRevData = (RevolutionData)mySurfaceData!;
                if (!Geom_RevolutionUtils.DN(u, v, aRevData.BasisCurve, aRevData.Axis, Nu, Nv, out Vector3D aDN))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalDN: evaluation failed");
                }
                return aDN;
            }

            case OffsetSurface:
            {
                if (hasEvalRep(mySurfaceData))
                {
                    return mySurface!.EvalDN(U, V, Nu, Nv);
                }
                OffsetData anOffData = (OffsetData)mySurfaceData!;
                if (!offsetDN(u, v, anOffData, Nu, Nv, out Vector3D aDN))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Surface::EvalDN: evaluation failed");
                }
                return aDN;
            }

            case global::Kerf.Geometry.SurfaceKind.Plane:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Plane3D)mySurfaceData!, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Cylinder)mySurfaceData!, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Cone:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Cone)mySurfaceData!, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Sphere)mySurfaceData!, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Torus:
                return global::Kerf.Engine.Geometry.ElSLib.DN(u, v, (Torus)mySurfaceData!, Nu, Nv);

            case BezierSurface:
            case OtherSurface:
            default:
                break;
        }

        return mySurface!.EvalDN(u, v, Nu, Nv);
    }

    /// <summary>The U resolution corresponding to the 3D resolution R3d.</summary>
    public override double UResolution(double R3d)
    {
        double Res = 0.0;

        switch (mySurfaceType)
        {
            case SurfaceOfExtrusion:
            {
                return ((ExtrusionData)mySurfaceData!).BasisCurve.Resolution(R3d);
            }
            case global::Kerf.Geometry.SurfaceKind.Torus:
            {
                Geom_ToroidalSurface S = (Geom_ToroidalSurface)mySurface!;
                double R = S.MajorRadius() + S.MinorRadius();
                if (R > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    Res = R3d / (2.0 * R);
                }
                break;
            }
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            {
                Geom_SphericalSurface S = (Geom_SphericalSurface)mySurface!;
                double R = S.Radius();
                if (R > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    Res = R3d / (2.0 * R);
                }
                break;
            }
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            {
                Geom_CylindricalSurface S = (Geom_CylindricalSurface)mySurface!;
                double R = S.Radius();
                if (R > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    Res = R3d / (2.0 * R);
                }
                break;
            }
            case global::Kerf.Geometry.SurfaceKind.Cone:
            {
                if (myVLast - myVFirst > 1.0e10)
                {
                    // Not truly bounded => unknown resolution
                    return global::Kerf.Engine.Foundation.Precision.Parametric(R3d);
                }
                Geom_ConicalSurface S = (Geom_ConicalSurface)mySurface!;
                Geom_Curve C = S.VIso(myVLast);
                double Rayon1 = ((Geom_Circle)C).Radius();
                C = S.VIso(myVFirst);
                double Rayon2 = ((Geom_Circle)C).Radius();
                double R = (Rayon1 > Rayon2) ? Rayon1 : Rayon2;
                return (R > global::Kerf.Engine.Foundation.Precision.Confusion() ? (R3d / R) : 0.0);
            }
            case global::Kerf.Geometry.SurfaceKind.Plane:
            {
                return R3d;
            }
            case BezierSurface:
            {
                ((Geom_BezierSurface)mySurface!).Resolution(R3d, out double Ures, out _);
                return Ures;
            }
            case BSplineSurface:
            {
                ((BSplineData)mySurfaceData!).Surface.Resolution(R3d, out double Ures, out _);
                return Ures;
            }
            case OffsetSurface:
            {
                return ((OffsetData)mySurfaceData!).BasisAdaptor.UResolution(R3d);
            }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(R3d);
        }

        if (Res <= 1.0)
        {
            return 2.0 * Math.Asin(Res);
        }

        return 2.0 * Math.PI;
    }

    /// <summary>The V resolution corresponding to the 3D resolution R3d.</summary>
    public override double VResolution(double R3d)
    {
        double Res = 0.0;

        switch (mySurfaceType)
        {
            case SurfaceOfRevolution:
            {
                return ((RevolutionData)mySurfaceData!).BasisCurve.Resolution(R3d);
            }
            case global::Kerf.Geometry.SurfaceKind.Torus:
            {
                Geom_ToroidalSurface S = (Geom_ToroidalSurface)mySurface!;
                double R = S.MinorRadius();
                if (R > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    Res = R3d / (2.0 * R);
                }
                break;
            }
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            {
                Geom_SphericalSurface S = (Geom_SphericalSurface)mySurface!;
                double R = S.Radius();
                if (R > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    Res = R3d / (2.0 * R);
                }
                break;
            }
            case SurfaceOfExtrusion:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Plane:
            {
                return R3d;
            }
            case BezierSurface:
            {
                ((Geom_BezierSurface)mySurface!).Resolution(R3d, out _, out double Vres);
                return Vres;
            }
            case BSplineSurface:
            {
                ((BSplineData)mySurfaceData!).Surface.Resolution(R3d, out _, out double Vres);
                return Vres;
            }
            case OffsetSurface:
            {
                return ((OffsetData)mySurfaceData!).BasisAdaptor.VResolution(R3d);
            }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(R3d);
        }

        if (Res <= 1.0)
        {
            return 2.0 * Math.Asin(Res);
        }

        return 2.0 * Math.PI;
    }

    /// <summary>The type of the surface (GeomAbs_OtherSurface before a Load).</summary>
    public override SurfaceKind GetType() => mySurfaceType;

    /// <exception cref="NoSuchObjectException">The surface is not a plane.</exception>
    public override Plane3D Plane()
    {
        if (mySurfaceType != global::Kerf.Geometry.SurfaceKind.Plane)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Plane");
        }
        return (Plane3D)mySurfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cylinder.</exception>
    public override Cylinder Cylinder()
    {
        if (mySurfaceType != global::Kerf.Geometry.SurfaceKind.Cylinder)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Cylinder");
        }
        return (Cylinder)mySurfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cone.</exception>
    public override Cone Cone()
    {
        if (mySurfaceType != global::Kerf.Geometry.SurfaceKind.Cone)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Cone");
        }
        return (Cone)mySurfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a sphere.</exception>
    public override Sphere Sphere()
    {
        if (mySurfaceType != global::Kerf.Geometry.SurfaceKind.Sphere)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Sphere");
        }
        return (Sphere)mySurfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a torus.</exception>
    public override Torus Torus()
    {
        if (mySurfaceType != global::Kerf.Geometry.SurfaceKind.Torus)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Torus");
        }
        return (Torus)mySurfaceData!;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline, Bezier or extrusion surface.</exception>
    public override int UDegree()
    {
        if (mySurfaceType == BSplineSurface)
        {
            return ((BSplineData)mySurfaceData!).Surface.UDegree();
        }
        if (mySurfaceType == BezierSurface)
        {
            return ((Geom_BezierSurface)mySurface!).UDegree();
        }
        if (mySurfaceType == SurfaceOfExtrusion)
        {
            return ((ExtrusionData)mySurfaceData!).BasisCurve.Degree();
        }
        throw new NoSuchObjectException("GeomAdaptor_Surface::UDegree");
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline, Bezier or extrusion surface.</exception>
    public override int NbUPoles()
    {
        if (mySurfaceType == BSplineSurface)
        {
            return ((BSplineData)mySurfaceData!).Surface.NbUPoles();
        }
        if (mySurfaceType == BezierSurface)
        {
            return ((Geom_BezierSurface)mySurface!).NbUPoles();
        }
        if (mySurfaceType == SurfaceOfExtrusion)
        {
            return ((ExtrusionData)mySurfaceData!).BasisCurve.NbPoles();
        }
        throw new NoSuchObjectException("GeomAdaptor_Surface::NbUPoles");
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline, Bezier or revolution surface.</exception>
    public override int VDegree()
    {
        if (mySurfaceType == BSplineSurface)
        {
            return ((BSplineData)mySurfaceData!).Surface.VDegree();
        }
        if (mySurfaceType == BezierSurface)
        {
            return ((Geom_BezierSurface)mySurface!).VDegree();
        }
        if (mySurfaceType == SurfaceOfRevolution)
        {
            return ((RevolutionData)mySurfaceData!).BasisCurve.Degree();
        }
        throw new NoSuchObjectException("GeomAdaptor_Surface::VDegree");
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline, Bezier or revolution surface.</exception>
    public override int NbVPoles()
    {
        if (mySurfaceType == BSplineSurface)
        {
            return ((BSplineData)mySurfaceData!).Surface.NbVPoles();
        }
        if (mySurfaceType == BezierSurface)
        {
            return ((Geom_BezierSurface)mySurface!).NbVPoles();
        }
        if (mySurfaceType == SurfaceOfRevolution)
        {
            return ((RevolutionData)mySurfaceData!).BasisCurve.NbPoles();
        }
        throw new NoSuchObjectException("GeomAdaptor_Surface::NbVPoles");
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline or extrusion surface.</exception>
    public override int NbUKnots()
    {
        if (mySurfaceType == BSplineSurface)
        {
            return ((BSplineData)mySurfaceData!).Surface.NbUKnots();
        }
        if (mySurfaceType == SurfaceOfExtrusion)
        {
            return ((ExtrusionData)mySurfaceData!).BasisCurve.NbKnots();
        }
        throw new NoSuchObjectException("GeomAdaptor_Surface::NbUKnots");
    }

    /// <exception cref="NoSuchObjectException">The surface is not a B-spline surface.</exception>
    public override int NbVKnots()
    {
        if (mySurfaceType == BSplineSurface)
        {
            return ((BSplineData)mySurfaceData!).Surface.NbVKnots();
        }
        throw new NoSuchObjectException("GeomAdaptor_Surface::NbVKnots");
    }

    /// <summary>True for a B-spline or Bezier surface rational in U; false for any other surface.</summary>
    public override bool IsURational()
    {
        if (mySurfaceType == BSplineSurface)
        {
            return ((BSplineData)mySurfaceData!).Surface.IsURational();
        }
        if (mySurfaceType == BezierSurface)
        {
            return ((Geom_BezierSurface)mySurface!).IsURational();
        }
        return false;
    }

    /// <summary>True for a B-spline or Bezier surface rational in V; false for any other surface.</summary>
    public override bool IsVRational()
    {
        if (mySurfaceType == BSplineSurface)
        {
            return ((BSplineData)mySurfaceData!).Surface.IsVRational();
        }
        if (mySurfaceType == BezierSurface)
        {
            return ((Geom_BezierSurface)mySurface!).IsVRational();
        }
        return false;
    }

    /// <summary>The Bezier surface (shared).</summary>
    /// <exception cref="NoSuchObjectException">The surface is not a Bezier surface.</exception>
    public override Geom_BezierSurface Bezier()
    {
        if (mySurfaceType != BezierSurface)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Bezier");
        }
        return (Geom_BezierSurface)mySurface!;
    }

    /// <summary>The B-spline surface (shared).</summary>
    /// <exception cref="NoSuchObjectException">The surface is not a B-spline surface.</exception>
    public override Geom_BSplineSurface BSpline()
    {
        if (mySurfaceType != BSplineSurface)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::BSpline");
        }
        return ((BSplineData)mySurfaceData!).Surface;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a surface of revolution.</exception>
    public override Axis3D AxeOfRevolution()
    {
        if (mySurfaceType != SurfaceOfRevolution)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::AxeOfRevolution");
        }
        return ((Geom_SurfaceOfRevolution)mySurface!).Axis();
    }

    /// <exception cref="NoSuchObjectException">The surface is not a surface of linear extrusion.</exception>
    public override Direction3D Direction()
    {
        if (mySurfaceType != SurfaceOfExtrusion)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::Direction");
        }
        return ((Geom_SurfaceOfLinearExtrusion)mySurface!).Direction();
    }

    /// <summary>A new adaptor of the basis curve of a swept surface (over the whole curve).</summary>
    /// <exception cref="NoSuchObjectException">The surface is not a swept surface.</exception>
    public override Adaptor3d_Curve BasisCurve()
    {
        Geom_Curve C;
        if (mySurfaceType == SurfaceOfExtrusion)
        {
            C = ((Geom_SurfaceOfLinearExtrusion)mySurface!).BasisCurve();
        }
        else if (mySurfaceType == SurfaceOfRevolution)
        {
            C = ((Geom_SurfaceOfRevolution)mySurface!).BasisCurve();
        }
        else
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::BasisCurve");
        }
        return new GeomAdaptor_Curve(C);
    }

    /// <summary>A new adaptor of the basis surface of an offset surface, over this adaptor's bounds.</summary>
    /// <exception cref="NoSuchObjectException">The surface is not an offset surface.</exception>
    public override Adaptor3d_Surface BasisSurface()
    {
        if (mySurfaceType != OffsetSurface)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::BasisSurface");
        }
        return new GeomAdaptor_Surface(((Geom_OffsetSurface)mySurface!).BasisSurface(),
                                       myUFirst,
                                       myULast,
                                       myVFirst,
                                       myVLast);
    }

    /// <exception cref="NoSuchObjectException">The surface is not an offset surface.</exception>
    public override double OffsetValue()
    {
        if (mySurfaceType != OffsetSurface)
        {
            throw new NoSuchObjectException("GeomAdaptor_Surface::BasisSurface");
        }
        return ((Geom_OffsetSurface)mySurface!).Offset();
    }

    // Locates U, V if they are a First or Last bound, processes the found span and returns the parameters for
    // LocalDi.
    private bool IfUVBound(double U,
                           double V,
                           out int IOutDeb,
                           out int IOutFin,
                           out int IOutVDeb,
                           out int IOutVFin,
                           int USide,
                           int VSide)
    {
        Geom_BSplineSurface aBSpl = ((BSplineData)mySurfaceData!).Surface;
        int Ideb, Ifin;
        int anUFKIndx = aBSpl.FirstUKnotIndex(), anULKIndx = aBSpl.LastUKnotIndex(),
            aVFKIndx = aBSpl.FirstVKnotIndex(), aVLKIndx = aBSpl.LastVKnotIndex();
        aBSpl.LocateU(U, PosTol, out Ideb, out Ifin, false);
        bool Local = (Ideb == Ifin);
        Span(USide, Ideb, Ifin, out Ideb, out Ifin, anUFKIndx, anULKIndx);
        int IVdeb, IVfin;
        aBSpl.LocateV(V, PosTol, out IVdeb, out IVfin, false);
        if (IVdeb == IVfin)
        {
            Local = true;
        }
        Span(VSide, IVdeb, IVfin, out IVdeb, out IVfin, aVFKIndx, aVLKIndx);

        IOutDeb = Ideb;
        IOutFin = Ifin;
        IOutVDeb = IVdeb;
        IOutVFin = IVfin;

        return Local;
    }

    // Locates U, V if U = UFirst or U = ULast, processes the found span and returns the parameters for LocalDi.
    private static void Span(int Side, int Ideb, int Ifin, out int OutIdeb, out int OutIfin, int theFKIndx,
                             int theLKIndx)
    {
        if (Ideb != Ifin) // not a knot
        {
            if (Ideb < theFKIndx)
            {
                OutIdeb = theFKIndx;
                OutIfin = theFKIndx + 1;
            }
            else if (Ifin > theLKIndx)
            {
                OutIdeb = theLKIndx - 1;
                OutIfin = theLKIndx;
            }
            else if (Ideb >= (theLKIndx - 1))
            {
                OutIdeb = theLKIndx - 1;
                OutIfin = theLKIndx;
            }
            else if (Ifin <= theFKIndx + 1)
            {
                OutIdeb = theFKIndx;
                OutIfin = theFKIndx + 1;
            }
            else if (Ideb > Ifin)
            {
                OutIdeb = Ifin - 1;
                OutIfin = Ifin;
            }
            else
            {
                OutIdeb = Ideb;
                OutIfin = Ifin;
            }
        }
        else
        {
            if (Ideb <= theFKIndx)
            {
                OutIdeb = theFKIndx;
                OutIfin = theFKIndx + 1;
            } // first knot
            else if (Ifin >= theLKIndx)
            {
                OutIdeb = theLKIndx - 1;
                OutIfin = theLKIndx;
            } // last knot
            else
            {
                if (Side == -1)
                {
                    OutIdeb = Ideb - 1;
                    OutIfin = Ifin;
                }
                else
                {
                    OutIdeb = Ideb;
                    OutIfin = Ifin + 1;
                }
            }
        }
    }
}
