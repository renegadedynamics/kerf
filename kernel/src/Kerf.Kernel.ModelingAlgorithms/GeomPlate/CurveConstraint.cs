// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_CurveConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_CurveConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Measurement;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A curve constraint of a plate surface: a 3D curve the plate passes through (order 0, or -1 for a curve only
/// approached), or a curve on a surface (an <see cref="CurveOnSurface"/> over a
/// <see cref="SurfaceAdaptor"/>) whose tangent plane (order 1) or curvature (order 2) the plate must follow.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: its setters are public, and GeomPlate_BuildPlateSurface sets the
/// projected curve, the 2D curve and the order. It keeps the caller's adaptors and laws, read only.
/// </remarks>
internal class CurveConstraint : Transient
{
    protected CurveOnSurface? _frontiere;
    protected int _nbPoints;
    protected int _order;
    protected CurveAdaptorBase? _my3dCurve;
    protected int _tang;
    protected Curve2D? _my2dCurve;
    protected Curve2DAdaptorBase? _hCurve2d;
    protected LawsFunction? _g0Crit;
    protected LawsFunction? _g1Crit;
    protected LawsFunction? _g2Crit;
    protected bool _constG0;
    protected bool _constG1;
    protected bool _constG2;
    protected SLProps _lProp;
    protected double _tolDist;
    protected double _tolAng;
    protected double _tolCurv;
    protected double _tolU;
    protected double _tolV;

    public CurveConstraint()
    {
        _nbPoints = 0;
        _order = 0;
        _tang = 0;
        _constG0 = false;
        _constG1 = false;
        _constG2 = false;
        _lProp = new SLProps(2, 1.0e-4);
        _tolDist = 0.0;
        _tolAng = 0.0;
        _tolCurv = 0.0;
        _tolU = 0.0;
        _tolV = 0.0;
    }

    /// <summary>
    /// A constraint on <paramref name="boundary"/> of order <paramref name="tang"/> (-1 to 2), checked at
    /// <paramref name="nPt"/> points.
    /// </summary>
    /// <exception cref="FailureException">The order isn't -1 to 2, or a curve on a surface isn't on a
    /// GeomAdaptor_Surface.</exception>
    public CurveConstraint(CurveAdaptorBase boundary, int tang, int nPt = 10, double tolDist = 0.0001,
                                     double tolAng = 0.01, double tolCurv = 0.1)
    {
        _lProp = new SLProps(2, tolDist);
        _tolDist = tolDist;
        _tolAng = tolAng;
        _tolCurv = tolCurv;
        _order = tang;
        if ((tang < -1) || (tang > 2))
        {
            throw new FailureException("GeomPlate : The continuity is not G0 G1 or G2");
        }
        _nbPoints = nPt;
        _constG0 = true;
        _constG1 = true;
        _constG2 = true;

        _frontiere = boundary as CurveOnSurface;

        if (_frontiere is null)
        {
            _my3dCurve = boundary;
        }
        else
        {
            Surface? surf;
            SurfaceAdaptor? gs1 = _frontiere.GetSurface() as SurfaceAdaptor;

            if (gs1 is not null)
            {
                surf = gs1.Surface();
            }
            else
            {
                throw new FailureException("GeomPlate_CurveConstraint : Surface must be GeomAdaptor_Surface");
            }

            _lProp.SetSurface(surf!);
        }

        _my2dCurve = null;
        _hCurve2d = null;
        _tolU = 0.0;
        _tolV = 0.0;
        _g0Crit = null;
        _g1Crit = null;
        _g2Crit = null;
    }

    public double FirstParameter()
    {
        if (_hCurve2d is not null)
        {
            return _hCurve2d.FirstParameter();
        }
        else if (_my3dCurve is null)
        {
            return _frontiere!.FirstParameter();
        }
        else
        {
            return _my3dCurve.FirstParameter();
        }
    }

    public double LastParameter()
    {
        if (_hCurve2d is not null)
        {
            return _hCurve2d.LastParameter();
        }
        else if (_my3dCurve is null)
        {
            return _frontiere!.LastParameter();
        }
        else
        {
            return _my3dCurve.LastParameter();
        }
    }

    public double Length()
    {
        if (_my3dCurve is null)
        {
            return AbscissaPoint.Length(_frontiere!);
        }
        else
        {
            return AbscissaPoint.Length(_my3dCurve);
        }
    }

    public void D0(double u, out Point3D p)
    {
        Point2D p2d;

        if (_my3dCurve is null)
        {
            p2d = _frontiere!.GetCurve()!.Value(u);
            _frontiere.GetSurface()!.D0(p2d.Coord(1), p2d.Coord(2), out p);
        }
        else
        {
            _my3dCurve.D0(u, out p);
        }
    }

    /// <exception cref="FailureException">The constraint isn't on a surface.</exception>
    public void D1(double u, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        Point2D p2d;
        if (_my3dCurve is not null)
        {
            throw new FailureException("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }

        p2d = _frontiere!.GetCurve()!.Value(u);
        _frontiere.GetSurface()!.D1(p2d.Coord(1), p2d.Coord(2), out p, out v1, out v2);
    }

    /// <exception cref="FailureException">The constraint isn't on a surface.</exception>
    public void D2(double u, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3, out Vector3D v4, out Vector3D v5)
    {
        Point2D p2d;
        if (_my3dCurve is not null)
        {
            throw new FailureException("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }

        p2d = _frontiere!.GetCurve()!.Value(u);
        _frontiere.GetSurface()!.D2(p2d.Coord(1), p2d.Coord(2), out p, out v1, out v2, out v3, out v4, out v5);
    }

    public void SetG0Criterion(LawsFunction g0Crit)
    {
        _g0Crit = g0Crit;
        _constG0 = false;
    }

    public void SetG1Criterion(LawsFunction g1Crit)
    {
        if (_my3dCurve is not null)
        {
            throw new FailureException("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        _g1Crit = g1Crit;
        _constG1 = false;
    }

    public void SetG2Criterion(LawsFunction g2Crit)
    {
        if (_my3dCurve is not null)
        {
            throw new FailureException("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        _g2Crit = g2Crit;
        _constG2 = false;
    }

    public double G0Criterion(double u)
    {
        if (_constG0)
        {
            return _tolDist;
        }
        else
        {
            return _g0Crit!.Value(u);
        }
    }

    public double G1Criterion(double u)
    {
        if (_my3dCurve is not null)
        {
            throw new FailureException("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        if (_constG1)
        {
            return _tolAng;
        }
        else
        {
            return _g1Crit!.Value(u);
        }
    }

    public double G2Criterion(double u)
    {
        if (_my3dCurve is not null)
        {
            throw new FailureException("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        if (_constG2)
        {
            return _tolCurv;
        }
        else
        {
            return _g2Crit!.Value(u);
        }
    }

    /// <summary>
    /// The 2D curve on the surface: the one set, else an approximation of the projected curve (a new curve on each
    /// call), else null.
    /// </summary>
    public Curve2D? Curve2dOnSurf()
    {
        if (_my2dCurve is null && _hCurve2d is not null)
        {
            Curve2D? c2d;
            Continuity continuity = global::Kerf.Geometry.Continuity.C1;
            int maxDegree = 10;
            int maxSeg = 20 + _hCurve2d.NbIntervals(global::Kerf.Geometry.Continuity.C3);
            Curve2d appr = new Curve2d(_hCurve2d, _hCurve2d.FirstParameter(), _hCurve2d.LastParameter(),
                                                     _tolU, _tolV, continuity, maxDegree, maxSeg);
            c2d = appr.Curve();
            return c2d;
        }
        else
        {
            return _my2dCurve;
        }
    }

    public void SetCurve2dOnSurf(Curve2D? curve)
    {
        _my2dCurve = curve;
    }

    public Curve2DAdaptorBase? ProjectedCurve() => _hCurve2d;

    public void SetProjectedCurve(Curve2DAdaptorBase? curve, double tolU, double tolV)
    {
        _hCurve2d = curve;
        _tolU = tolU;
        _tolV = tolV;
    }

    public CurveAdaptorBase Curve3d()
    {
        if (_my3dCurve is null)
        {
            return _frontiere!;
        }
        else
        {
            return _my3dCurve;
        }
    }

    public int PointCount => _nbPoints;

    public int Order() => _order;

    public void SetNbPoints(int newNb)
    {
        _nbPoints = newNb;
    }

    public void SetOrder(int order)
    {
        _order = order;
    }

    /// <summary>The local properties of the support surface at the constraint's parameter U.</summary>
    /// <exception cref="FailureException">The constraint isn't on a surface.</exception>
    public SLProps LPropSurf(double u)
    {
        if (_frontiere is null)
        {
            throw new FailureException("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        Point2D p2d = _frontiere.GetCurve()!.Value(u);
        _lProp.SetParameters(p2d.X, p2d.Y);
        return _lProp;
    }
}
