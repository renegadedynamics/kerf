// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Face.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Face.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The surface tool of BRepGProp's surface and volume integrations: a face (its surface, placed and forward) and the
/// current arc of its boundary (a pcurve, or a side of the UV rectangle), with the integration orders and knots
/// chosen for each type of surface and curve.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: the Load methods change the current arc. OCCT copies it by value into
/// BRepGProp_TFunction and BRepGProp_UFunction; the port's copy is the internal copy constructor.
/// </remarks>
internal sealed class FaceIntegrand
{
    private static readonly double Epsilon1 = Epsilon(1.0);

    private readonly FaceSurfaceAdaptor _surface;
    private readonly CurveAdaptor2D _curve;
    private Surface? _faceSurface;
    private Placement _faceLocation;
    private bool _isFaceContextReady;
    private bool _sReverse;
    private readonly bool _isUseSpan;

    /// <summary>
    /// An empty face tool. <paramref name="isUseSpan"/> asks for the spans of a B-spline face
    /// (<see cref="GetUKnots"/>, <see cref="GetTKnots"/>); it has no effect on other faces.
    /// </summary>
    public FaceIntegrand(bool isUseSpan = false)
    {
        _surface = new FaceSurfaceAdaptor();
        _curve = new CurveAdaptor2D();
        _faceLocation = new Placement();
        _isFaceContextReady = false;
        _sReverse = false;
        _isUseSpan = isUseSpan;
    }

    /// <summary>The face tool of <paramref name="f"/>; see <see cref="FaceIntegrand(bool)"/> for <paramref name="isUseSpan"/>.</summary>
    public FaceIntegrand(in Face f, bool isUseSpan = false)
        : this(isUseSpan)
    {
        Load(f);
    }

    /// <summary>C++ copy construction: the adaptors are shallow-copied, so the copy can be reloaded on its own.</summary>
    internal FaceIntegrand(FaceIntegrand other)
    {
        _surface = (FaceSurfaceAdaptor)other._surface.ShallowCopy();
        _curve = (CurveAdaptor2D)other._curve.ShallowCopy();
        _faceSurface = other._faceSurface;
        _faceLocation = other._faceLocation;
        _isFaceContextReady = other._isFaceContextReady;
        _sReverse = other._sReverse;
        _isUseSpan = other._isUseSpan;
    }

    /// <summary>Loads the face: its surface, forward, and whether it is reversed.</summary>
    public void Load(in Face f)
    {
        Shape localShape = f.Oriented(Forward);
        _surface.Initialize(ShapeCasts.Face(localShape));
        _faceSurface = Tool.Surface(_surface.Face(), out _faceLocation);
        _isFaceContextReady = _faceSurface is not null;
        //  mySurface.Initialize(TopoDS::Face(F.Oriented(TopAbs_FORWARD)));
        _sReverse = (f.Orientation == Reversed);
    }

    /// <summary>The number of points for an accurate integration in the U direction.</summary>
    public int UIntegrationOrder()
    {
        int nu;
        switch (_surface.GetType())
        {
            case Plane:
                nu = 4;
                break;

            case SurfaceKind.BezierSurface:
            {
                nu = ((global::Kerf.Geometry.BezierSurface)_surface.GeomSurfaceOriginal()!).UDegree + 1;
                nu = Max(4, nu);
            }

                break;
            case SurfaceKind.BSplineSurface:
            {
                int a = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).UDegree + 1;
                int b = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).UKnotCount - 1;
                nu = Max(4, a * b);
            }

                break;
            default:
                nu = 9;
                break;
        }

        return Max(8, 2 * nu);
    }

    /// <summary>The number of points for an accurate integration in the V direction.</summary>
    public int VIntegrationOrder()
    {
        int nv;
        switch (_surface.GetType())
        {
            case Plane:
                nv = 4;
                break;

            case SurfaceKind.BezierSurface:
            {
                nv = ((global::Kerf.Geometry.BezierSurface)_surface.GeomSurfaceOriginal()!).VDegree + 1;
                nv = Max(4, nv);
            }

                break;
            case SurfaceKind.BSplineSurface:
            {
                int a = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).VDegree + 1;
                int b = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).VKnotCount - 1;
                nv = Max(4, a * b);
            }

                break;
            default:
                nv = 9;
                break;
        }

        return Max(8, 2 * nv);
    }

    /// <summary>True if the face is not trimmed.</summary>
    public bool NaturalRestriction() => Tool.NaturalRestriction(_surface.Face());

    /// <summary>The face, forward.</summary>
    public Face GetFace() => _surface.Face();

    /// <summary>The point of parameter U on the current boundary curve.</summary>
    public Point2D Value2d(double u) => _curve.Value(u);

    /// <summary>The number of points along the current boundary curve for an accurate integration.</summary>
    public int IntegrationOrder()
    {
        int n;

        switch (_curve.GetType())
        {
            case Line:
                n = 2;
                break;

            case Circle:
            case Ellipse:
            case Hyperbola:
                n = 9;
                break;

            case Parabola:
                n = 9;
                break;

            case BezierCurve:
            {
                n = ((BezierCurve2D)_curve.Curve()!).Degree + 1;
            }

                break;
            case BSplineCurve:
            {
                int a = ((BSplineCurve2D)_curve.Curve()!).Degree + 1;
                int b = ((BSplineCurve2D)_curve.Curve()!).KnotCount - 1;
                n = a * b;
            }

                break;
            default:
                n = 9;
                break;
        }

        return Max(4, 2 * n);
    }

    /// <summary>The parametric bounds of the face.</summary>
    public void Bounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = _surface.FirstUParameter();
        u2 = _surface.LastUParameter();
        v1 = _surface.FirstVParameter();
        v2 = _surface.LastVParameter();
    }

    /// <summary>Loads the boundary arc of the edge; false if the edge has no pcurve on the face.</summary>
    public bool Load(in Edge e)
    {
        double a, b;
        if (!_isFaceContextReady)
        {
            return false;
        }

        Curve2D? c = Tool.CurveOnSurface(e, _faceSurface, _faceLocation, out a, out b);
        if (c is null)
        {
            return false;
        }

        if (e.Orientation == Reversed)
        {
            double x = a;
            a = c.ReversedParameter(b);
            b = c.ReversedParameter(x);
            c = c.Reversed();
        }

        _curve.Load(c, a, b);
        return true;
    }

    /// <summary>The point of parameters (U, V) on the face and the normal there (D1U ^ D1V, reversed with the face).</summary>
    public void Normal(double u, double v, out Point3D p, out Vector3D vNor)
    {
        _surface.D1(u, v, out p, out Vector3D d1U, out Vector3D d1V);
        vNor = d1U.Crossed(d1V);
        if (_sReverse)
        {
            vNor = vNor.Reversed();
        }
    }

    //  APO 17.04.2002 (OCC104)
    // This is functions that calculate coeff. to optimize "integration order".
    // They had been produced experimentally for some hard example.
    private const double As = -0.15, Al = -0.50, B = 1.0, C = 0.75, D = 0.25;

    private static double SCoeff(double eps) => eps < 0.1 ? As * (B + Math.Log10(eps)) + C : C;

    private static double LCoeff(double eps) => eps < 0.1 ? Al * (B + Math.Log10(eps)) + D : D;

    /// <summary>The number of Gauss points over the surface for the relative precision <paramref name="eps"/>.</summary>
    public int SIntOrder(double eps)
    {
        int nv, nu;
        switch (_surface.GetType())
        {
            case Plane:
                nu = 1;
                nv = 1;
                break;
            case SurfaceKind.Cylinder:
                nu = 2;
                nv = 1;
                break;
            case SurfaceKind.Cone:
                nu = 2;
                nv = 1;
                break;
            case SurfaceKind.Sphere:
                nu = 2;
                nv = 2;
                break;
            case SurfaceKind.Torus:
                nu = 2;
                nv = 2;
                break;
            case SurfaceKind.BezierSurface:
                nv = ((global::Kerf.Geometry.BezierSurface)_surface.GeomSurfaceOriginal()!).VDegree;
                nu = ((global::Kerf.Geometry.BezierSurface)_surface.GeomSurfaceOriginal()!).UDegree;
                break;
            case SurfaceKind.BSplineSurface:
                nv = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).VDegree;
                nu = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).UDegree;
                break;
            default:
                nu = 2;
                nv = 2;
                break;
        }

        return Min(RealToInt(Math.Ceiling(SCoeff(eps) * Max((nu + 1), (nv + 1)))), NumericTables.GaussPointsMax());
    }

    /// <summary>The number of subintervals in U over the surface.</summary>
    public int SUIntSubs()
    {
        int n;
        switch (_surface.GetType())
        {
            case Plane:
                n = 2;
                break;
            case SurfaceKind.Cylinder:
                n = 4;
                break;
            case SurfaceKind.Cone:
                n = 4;
                break;
            case SurfaceKind.Sphere:
                n = 4;
                break;
            case SurfaceKind.Torus:
                n = 4;
                break;
            case SurfaceKind.BezierSurface:
                n = 2;
                break;
            case SurfaceKind.BSplineSurface:
                n = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).UKnotCount;
                break;
            default:
                n = 2;
                break;
        }

        return n - 1;
    }

    /// <summary>The number of subintervals in V over the surface.</summary>
    public int SVIntSubs()
    {
        int n;
        switch (_surface.GetType())
        {
            case Plane:
                n = 2;
                break;
            case SurfaceKind.Cylinder:
                n = 2;
                break;
            case SurfaceKind.Cone:
                n = 2;
                break;
            case SurfaceKind.Sphere:
                n = 3;
                break;
            case SurfaceKind.Torus:
                n = 4;
                break;
            case SurfaceKind.BezierSurface:
                n = 2;
                break;
            case SurfaceKind.BSplineSurface:
                n = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).VKnotCount;
                break;
            default:
                n = 2;
                break;
        }

        return n - 1;
    }

    /// <summary>Fills <paramref name="knots"/> (SUIntSubs() + 1 values) with the U bounds of the subintervals.</summary>
    public void UKnots(NCollection_Array1<double> knots)
    {
        switch (_surface.GetType())
        {
            case Plane:
                knots[1] = _surface.FirstUParameter();
                knots[2] = _surface.LastUParameter();
                break;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                knots[1] = 0.0;
                knots[2] = M_PI * 2.0 / 3.0;
                knots[3] = M_PI * 4.0 / 3.0;
                knots[4] = M_PI * 6.0 / 3.0;
                break;
            case SurfaceKind.BSplineSurface:
            {
                NCollection_Array1<double> srcKnots = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).InternalUKnots();
                for (int i = knots.Lower(); i <= knots.Upper(); i++)
                {
                    knots[i] = srcKnots[i];
                }
            }

                break;
            default:
                knots[1] = _surface.FirstUParameter();
                knots[2] = _surface.LastUParameter();
                break;
        }
    }

    /// <summary>Fills <paramref name="knots"/> (SVIntSubs() + 1 values) with the V bounds of the subintervals.</summary>
    public void VKnots(NCollection_Array1<double> knots)
    {
        switch (_surface.GetType())
        {
            case Plane:
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
                // the V bounds (OCCT took the U bounds, here and below: decision 1469)
                knots[1] = _surface.FirstVParameter();
                knots[2] = _surface.LastVParameter();
                break;
            case SurfaceKind.Sphere:
                knots[1] = -M_PI / 2.0;
                knots[2] = 0.0;
                knots[3] = +M_PI / 2.0;
                break;
            case SurfaceKind.Torus:
                knots[1] = 0.0;
                knots[2] = M_PI * 2.0 / 3.0;
                knots[3] = M_PI * 4.0 / 3.0;
                knots[4] = M_PI * 6.0 / 3.0;
                break;
            case SurfaceKind.BSplineSurface:
            {
                NCollection_Array1<double> srcKnots = ((global::Kerf.Geometry.BSplineSurface)_surface.GeomSurfaceOriginal()!).InternalVKnots();
                for (int i = knots.Lower(); i <= knots.Upper(); i++)
                {
                    knots[i] = srcKnots[i];
                }
            }

                break;
            default:
                knots[1] = _surface.FirstVParameter();
                knots[2] = _surface.LastVParameter();
                break;
        }
    }

    /// <summary>The number of Gauss points along the current boundary curve for the relative precision <paramref name="eps"/>.</summary>
    public int LIntOrder(double eps)
    {
        Box2d box = new();

        Add2dCurve.Add(_curve, 1.0e-7, ref box);
        double xmin, xmax, ymin, ymax;
        box.Get(out xmin, out ymin, out xmax, out ymax);
        double vmin = _surface.FirstVParameter();
        double vmax = _surface.LastVParameter();

        double dv = (vmax - vmin);
        double r = (dv > Epsilon1 ? Min((ymax - ymin) / dv, 1.0) : 1.0);

        int rInt = RealToInt(Math.Ceiling(SVIntSubs() * r));
        int lSubs = LIntSubs();

        //  double NL, NS = std::max(SIntOrder(1.0)*anRInt/LIntSubs(), 1);
        double nl, ns = Max(SIntOrder(1.0) * rInt / lSubs, 1);
        switch (_curve.GetType())
        {
            case Line:
                nl = 1;
                break;
            case Circle:
                nl = 2 * 3;
                break; // correction for the spans of converted curve
            case Ellipse:
                nl = 2 * 3;
                break; //
            case Parabola:
                nl = 2 * 3;
                break;
            case Hyperbola:
                nl = 3 * 3;
                break;
            case BezierCurve:
                nl = ((BezierCurve2D)_curve.Curve()!).Degree;
                break;
            case BSplineCurve:
                nl = ((BSplineCurve2D)_curve.Curve()!).Degree;
                break;
            default:
                nl = 3 * 3;
                break;
        }

        nl = Max(nl, ns);

        int nn = RealToInt(lSubs <= 4 ? Math.Ceiling(LCoeff(eps) * (nl + 1)) : nl + 1);

        return Min(nn, NumericTables.GaussPointsMax());
    }

    /// <summary>The number of subintervals of the current boundary curve.</summary>
    public int LIntSubs()
    {
        int n;
        switch (_curve.GetType())
        {
            case Line:
                n = 2;
                break;
            case Circle:
            case Ellipse:
                n = 4;
                break;
            case Parabola:
            case Hyperbola:
                n = 2;
                break;
            case BSplineCurve:
                n = ((BSplineCurve2D)_curve.Curve()!).KnotCount;
                break;
            default:
                n = 2;
                break;
        }

        return n - 1;
    }

    /// <summary>Fills <paramref name="knots"/> (LIntSubs() + 1 values) with the bounds of the subintervals of the current curve.</summary>
    public void LKnots(NCollection_Array1<double> knots)
    {
        switch (_curve.GetType())
        {
            case Line:
                knots[1] = _curve.FirstParameter();
                knots[2] = _curve.LastParameter();
                break;
            case Circle:
            case Ellipse:
                knots[1] = 0.0;
                knots[2] = M_PI * 2.0 / 3.0;
                knots[3] = M_PI * 4.0 / 3.0;
                knots[4] = M_PI * 6.0 / 3.0;
                break;
            case Parabola:
            case Hyperbola:
                knots[1] = _curve.FirstParameter();
                knots[2] = _curve.LastParameter();
                break;
            case BSplineCurve:
            {
                NCollection_Array1<double> srcKnots = ((BSplineCurve2D)_curve.Curve()!).InternalKnots();
                for (int i = knots.Lower(); i <= knots.Upper(); i++)
                {
                    knots[i] = srcKnots[i];
                }
            }

                break;
            default:
                knots[1] = _curve.FirstParameter();
                knots[2] = _curve.LastParameter();
                break;
        }
    }

    /// <summary>The parameter of the start point of the current arc.</summary>
    public double FirstParameter() => _curve.FirstParameter();

    /// <summary>The parameter of the end point of the current arc.</summary>
    public double LastParameter() => _curve.LastParameter();

    /// <summary>The point of parameter U of the current boundary curve and the first derivative there.</summary>
    public void D12d(double u, out Point2D p, out Vector2D v1) => _curve.D1(u, out p, out v1);

    /// <summary>
    /// Loads a side of the UV rectangle of the face as the current arc: with <see cref="IsoU"/> the left
    /// (<paramref name="isFirstParam"/>) or right side, with <see cref="IsoV"/> the bottom or top one.
    /// </summary>
    public void Load(bool isFirstParam, IsoType isoType)
    {
        double len;
        double u1;
        double u2;
        double v1;
        double v2;
        Point2D loc = new();
        Direction2D dir = new();

        Bounds(out u1, out u2, out v1, out v2);

        if (isoType == IsoU)
        {
            len = v2 - v1;

            if (isFirstParam)
            {
                loc = loc.WithCoord(u1, v2);
                dir = dir.WithCoord(0.0, -1.0);
            }
            else
            {
                loc = loc.WithCoord(u2, v1);
                dir = dir.WithCoord(0.0, 1.0);
            }
        }
        else if (isoType == IsoV)
        {
            len = u2 - u1;

            if (isFirstParam)
            {
                loc = loc.WithCoord(u1, v1);
                dir = dir.WithCoord(1.0, 0.0);
            }
            else
            {
                loc = loc.WithCoord(u2, v2);
                dir = dir.WithCoord(-1.0, 0.0);
            }
        }
        else
        {
            return;
        }

        Curve2D lin = new LineCurve2D(loc, dir);

        _curve.Load(lin, 0.0, len);
    }

    private static void GetRealKnots(double min,
                                     double max,
                                     NCollection_HArray1<double> knots,
                                     out NCollection_HArray1<double> realKnots)
    {
        int i = knots.Lower() - 1;
        int iU = knots.Upper();
        int startI = 0;
        int endI = 0;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        while (++i < iU)
        {
            if (startI == 0 && knots.Value(i) > min + tol)
            {
                startI = i;
            }

            if (endI == 0 && knots.Value(i + 1) > max - tol)
            {
                endI = i;
            }

            if (startI != 0 && endI != 0)
            {
                break;
            }
        }

        if (startI == 0)
        {
            startI = iU;
        }

        int nbNode = Max(0, endI - startI + 1) + 2;
        int j;

        realKnots = new NCollection_HArray1<double>(1, nbNode);
        realKnots.SetValue(1, min);
        realKnots.SetValue(nbNode, max);

        for (i = 2, j = startI; j <= endI; i++, j++)
        {
            realKnots.SetValue(i, knots.Value(j));
        }
    }

    private static void GetCurveKnots(double min,
                                      double max,
                                      CurveAdaptor2D curve,
                                      out NCollection_HArray1<double> knots)
    {
        bool isSBSpline = curve.GetType() == BSplineCurve;

        if (isSBSpline)
        {
            BSplineCurve2D crv;
            NCollection_HArray1<double> crvKnots;

            crv = (BSplineCurve2D)curve.Curve()!;
            crvKnots = new NCollection_HArray1<double>(crv.InternalKnots());
            GetRealKnots(min, max, crvKnots, out knots);
        }
        else
        {
            knots = new NCollection_HArray1<double>(1, 2);
            knots.SetValue(1, min);
            knots.SetValue(2, max);
        }
    }

    /// <summary>
    /// The U knots of the face in [theUMin, theUMax]: theUMin, the knots of a B-spline face (or of the B-spline basis
    /// curve of an extrusion) strictly inside, in increasing order, if the face uses spans, then theUMax.
    /// </summary>
    public NCollection_HArray1<double> GetUKnots(double uMin, double uMax)
    {
        NCollection_HArray1<double> theUKnots;
        bool isSBSpline = _surface.GetType() == SurfaceKind.BSplineSurface;
        bool isCBSpline = false;

        if (!isSBSpline)
        {
            if (_surface.GetType() == SurfaceOfExtrusion)
            {
                var curve = new CurveAdaptor();
                Surface surf = _surface.GeomSurfaceOriginal()!;

                curve.Load(((ExtrusionSurface)surf).BasisCurve);
                isCBSpline = curve.GetType() == BSplineCurve;
            }
        }

        if (_isUseSpan && (isSBSpline || isCBSpline))
        {
            NCollection_HArray1<double> knots;

            if (isSBSpline)
            {
                Surface surf = _surface.GeomSurfaceOriginal()!;
                global::Kerf.Geometry.BSplineSurface bSplSurf;

                bSplSurf = (global::Kerf.Geometry.BSplineSurface)surf;
                knots = new NCollection_HArray1<double>(bSplSurf.InternalUKnots());
            }
            else
            {
                var curve = new CurveAdaptor();
                Surface surf = _surface.GeomSurfaceOriginal()!;
                BSplineCurve3D bSplCurve;

                curve.Load(((ExtrusionSurface)surf).BasisCurve);
                bSplCurve = curve.BSpline();
                knots = new NCollection_HArray1<double>(bSplCurve.InternalKnots());
            }

            GetRealKnots(uMin, uMax, knots, out theUKnots);
        }
        else
        {
            theUKnots = new NCollection_HArray1<double>(1, 2);
            theUKnots.SetValue(1, uMin);
            theUKnots.SetValue(2, uMax);
        }

        return theUKnots;
    }

    /// <summary>
    /// The T knots of the current arc in [theTMin, theTMax]: theTMin, the knots of a B-spline arc strictly inside, in
    /// increasing order, if the face is a B-spline using spans, then theTMax.
    /// </summary>
    public NCollection_HArray1<double> GetTKnots(double tMin, double tMax)
    {
        NCollection_HArray1<double> theTKnots;
        bool isBSpline = _surface.GetType() == SurfaceKind.BSplineSurface;

        if (_isUseSpan && isBSpline)
        {
            // OCCT copies the V knots of the face here and doesn't use them
            NCollection_HArray1<double> surfKnots;
            Surface surf = _surface.GeomSurfaceOriginal()!;
            global::Kerf.Geometry.BSplineSurface bSplSurf;

            bSplSurf = (global::Kerf.Geometry.BSplineSurface)surf;
            surfKnots = new NCollection_HArray1<double>(bSplSurf.InternalVKnots());
            _ = surfKnots;
            GetCurveKnots(tMin, tMax, _curve, out theTKnots);
        }
        else
        {
            theTKnots = new NCollection_HArray1<double>(1, 2);
            theTKnots.SetValue(1, tMin);
            theTKnots.SetValue(2, tMax);
        }

        return theTKnots;
    }
}
