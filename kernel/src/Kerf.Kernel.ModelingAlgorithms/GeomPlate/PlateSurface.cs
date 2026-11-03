// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_Surface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A plate surface: an initial surface plus the deformation of a solved plate, evaluated together. Built by
/// GeomPlate_BuildPlateSurface; usually approximated by GeomPlate_MakeApprox before use.
/// </summary>
/// <remarks>
/// Geometry (rules, section 8): the mutators (<see cref="SetBounds"/>, and OCCT's empty <c>UReverse</c>,
/// <c>VReverse</c> and <c>Transform</c>) are internal and check that the surface isn't frozen; the public
/// <see cref="WithBounds"/> changes a copy. The surface keeps its own copy of the plate, as OCCT's holds it by value,
/// and the caller's initial surface, read only. As in OCCT, <see cref="Copy"/> doesn't keep the bounds, the
/// parametric functions are placeholders (<c>UIso</c>/<c>VIso</c> return null, <c>Continuity</c> GeomAbs_C0,
/// <c>IsCNu</c>/<c>IsCNv</c> throw), and D3 and DN throw Geom_UndefinedDerivative.
/// </remarks>
public class PlateSurface : Surface
{
    private readonly PlateSolver _surfinter;
    private readonly Surface _surfinit;
    private double _umin;
    private double _umax;
    private double _vmin;
    private double _vmax;

    /// <summary>The surface <paramref name="surfinit"/> deformed by a copy of the plate <paramref name="surfinter"/>.</summary>
    public PlateSurface(Surface surfinit, PlateSolver surfinter)
    {
        _surfinter = new PlateSolver(surfinter);
        _surfinit = surfinit;
        _umin = 0.0;
        _umax = 0.0;
        _vmin = 0.0;
        _vmax = 0.0;
    }

    protected internal override void UReverse()
    {
        CheckMutable();
    }

    public override double UReversedParameter(double u)
    {
        return (-u);
    }

    protected internal override void VReverse()
    {
        CheckMutable();
    }

    public override double VReversedParameter(double v)
    {
        return (-v);
    }

    public override void TransformParameters(ref double u, ref double v, in Transform3D t)
    {
    }

    public override GeneralTransform2D ParametricTransformation(in Transform3D t)
    {
        return new GeneralTransform2D();
    }

    public override void Bounds(out double u1, out double u2, out double v1, out double v2)
    {
        if (_surfinit.DynamicType() == typeof(PlateSurface))
        {
            _surfinit.Bounds(out u1, out u2, out v1, out v2);
        }
        else
        {
            u1 = _umin;
            u2 = _umax;
            v1 = _vmin;
            v2 = _vmax;
        }
    }

    public override bool IsUClosed => false;

    public override bool IsVClosed => false;

    public override bool IsUPeriodic => false;

    public override double UPeriod() => 0.0; // OCCT returns false

    public override bool IsVPeriodic => false;

    public override double VPeriod() => 0.0; // OCCT returns false

    public override Curve3D UIso(double u) => null!; // OCCT returns a null handle

    public override Curve3D VIso(double v) => null!; // OCCT returns a null handle

    public override Continuity Continuity() => default; // GeomAbs_Shape(), GeomAbs_C0

    public override bool IsCNu(int n)
    {
        throw new FailureException("IsCNu");
    }

    public override bool IsCNv(int n)
    {
        throw new FailureException("IsCNv");
    }

    public override Point3D EvalD0(double u, double v)
    {
        Point3D surfP = _surfinit.EvalD0(u, v);
        Xy p1 = new Xy(u, v);
        Xyz p3 = _surfinter.Evaluate(p1);
        return new Point3D(p3 + surfP.XYZ());
    }

    public override ResD1 EvalD1(double u, double v)
    {
        ResD1 surfD1 = _surfinit.EvalD1(u, v);
        Xy p1 = new Xy(u, v);
        Xyz p3 = _surfinter.Evaluate(p1);
        Xyz v2U = _surfinter.EvaluateDerivative(p1, 1, 0);
        Xyz v2V = _surfinter.EvaluateDerivative(p1, 0, 1);
        ResD1 result;
        result.Point = new Point3D(p3 + surfD1.Point.XYZ());
        result.D1U = new Vector3D(surfD1.D1U.XYZ() + v2U);
        result.D1V = new Vector3D(surfD1.D1V.XYZ() + v2V);
        return result;
    }

    public override ResD2 EvalD2(double u, double v)
    {
        ResD2 surfD2 = _surfinit.EvalD2(u, v);
        Xy p1 = new Xy(u, v);
        Xyz p3 = _surfinter.Evaluate(p1);
        Xyz V2U_interp = _surfinter.EvaluateDerivative(p1, 1, 0);
        Xyz V2V_interp = _surfinter.EvaluateDerivative(p1, 0, 1);
        Xyz v2U = _surfinter.EvaluateDerivative(p1, 2, 0);
        Xyz v2V = _surfinter.EvaluateDerivative(p1, 0, 2);
        Xyz v2UV = _surfinter.EvaluateDerivative(p1, 1, 1);
        ResD2 result;
        result.Point = new Point3D(p3 + surfD2.Point.XYZ());
        result.D1U = new Vector3D(surfD2.D1U.XYZ() + V2U_interp);
        result.D1V = new Vector3D(surfD2.D1V.XYZ() + V2V_interp);
        result.D2U = new Vector3D(surfD2.D2U.XYZ() + v2U);
        result.D2V = new Vector3D(surfD2.D2V.XYZ() + v2V);
        result.D2UV = new Vector3D(surfD2.D2UV.XYZ() + v2UV);
        return result;
    }

    public override ResD3 EvalD3(double u, double v)
    {
        throw new UndefinedDerivative3DException("GeomPlate_Surface::EvalD3");
    }

    public override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        throw new UndefinedDerivative3DException("GeomPlate_Surface::EvalDN");
    }

    /// <summary>A new, unfrozen surface on the same initial surface and plate; as OCCT's, without the bounds.</summary>
    public override Geometry3D Copy()
    {
        PlateSurface gps = new PlateSurface(_surfinit, _surfinter);
        return gps;
    }

    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
    }

    /// <summary>The initial surface.</summary>
    public Surface CallSurfinit() => _surfinit;

    /// <exception cref="FailureException">The bounds are reversed or equal.</exception>
    internal void SetBounds(double umin, double umax, double vmin, double vmax)
    {
        CheckMutable();
        if ((umin > umax) || (vmin > vmax))
        {
            throw new FailureException("Bounds haven't the good sense");
        }
        if ((umin == umax) || (vmin == vmax))
        {
            throw new FailureException("Bounds are equal");
        }
        _umin = umin;
        _umax = umax;
        _vmin = vmin;
        _vmax = vmax;
    }

    /// <summary>A copy of the surface with the given bounds (the twin of OCCT's <c>SetBounds</c>).</summary>
    /// <exception cref="FailureException">The bounds are reversed or equal.</exception>
    public PlateSurface WithBounds(double umin, double umax, double vmin, double vmax)
    {
        PlateSurface copy = (PlateSurface)Copy();
        copy.SetBounds(umin, umax, vmin, vmax);
        return copy;
    }

    /// <summary>The UV box of the plate's constraints.</summary>
    public void RealBounds(out double u1, out double u2, out double v1, out double v2)
    {
        _surfinter.UVBox(out u1, out u2, out v1, out v2);
    }

    /// <summary>Appends the UV points of the plate's constraints to <paramref name="seq"/>.</summary>
    public void Constraints(NCollection_Sequence<Xy> seq)
    {
        _surfinter.UVConstraints(seq);
    }
}
