// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_Surface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public class GeomPlate_Surface : Geom_Surface
{
    private readonly Plate_Plate mySurfinter;
    private readonly Geom_Surface mySurfinit;
    private double myUmin;
    private double myUmax;
    private double myVmin;
    private double myVmax;

    /// <summary>The surface <paramref name="Surfinit"/> deformed by a copy of the plate <paramref name="Surfinter"/>.</summary>
    public GeomPlate_Surface(Geom_Surface Surfinit, Plate_Plate Surfinter)
    {
        mySurfinter = new Plate_Plate(Surfinter);
        mySurfinit = Surfinit;
        myUmin = 0.0;
        myUmax = 0.0;
        myVmin = 0.0;
        myVmax = 0.0;
    }

    protected internal override void UReverse()
    {
        CheckMutable();
    }

    public override double UReversedParameter(double U)
    {
        return (-U);
    }

    protected internal override void VReverse()
    {
        CheckMutable();
    }

    public override double VReversedParameter(double V)
    {
        return (-V);
    }

    public override void TransformParameters(ref double U, ref double V, in gp_Trsf T)
    {
    }

    public override gp_GTrsf2d ParametricTransformation(in gp_Trsf T)
    {
        return new gp_GTrsf2d();
    }

    public override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        if (mySurfinit.DynamicType() == typeof(GeomPlate_Surface))
        {
            mySurfinit.Bounds(out U1, out U2, out V1, out V2);
        }
        else
        {
            U1 = myUmin;
            U2 = myUmax;
            V1 = myVmin;
            V2 = myVmax;
        }
    }

    public override bool IsUClosed() => false;

    public override bool IsVClosed() => false;

    public override bool IsUPeriodic() => false;

    public override double UPeriod() => 0.0; // OCCT returns false

    public override bool IsVPeriodic() => false;

    public override double VPeriod() => 0.0; // OCCT returns false

    public override Geom_Curve UIso(double U) => null!; // OCCT returns a null handle

    public override Geom_Curve VIso(double V) => null!; // OCCT returns a null handle

    public override GeomAbs_Shape Continuity() => default; // GeomAbs_Shape(), GeomAbs_C0

    public override bool IsCNu(int N)
    {
        throw new Standard_Failure("IsCNu");
    }

    public override bool IsCNv(int N)
    {
        throw new Standard_Failure("IsCNv");
    }

    public override gp_Pnt EvalD0(double U, double V)
    {
        gp_Pnt aSurfP = mySurfinit.EvalD0(U, V);
        gp_XY P1 = new gp_XY(U, V);
        gp_XYZ P3 = mySurfinter.Evaluate(P1);
        return new gp_Pnt(P3 + aSurfP.XYZ());
    }

    public override ResD1 EvalD1(double U, double V)
    {
        ResD1 aSurfD1 = mySurfinit.EvalD1(U, V);
        gp_XY P1 = new gp_XY(U, V);
        gp_XYZ P3 = mySurfinter.Evaluate(P1);
        gp_XYZ V2U = mySurfinter.EvaluateDerivative(P1, 1, 0);
        gp_XYZ V2V = mySurfinter.EvaluateDerivative(P1, 0, 1);
        ResD1 aResult;
        aResult.Point = new gp_Pnt(P3 + aSurfD1.Point.XYZ());
        aResult.D1U = new gp_Vec(aSurfD1.D1U.XYZ() + V2U);
        aResult.D1V = new gp_Vec(aSurfD1.D1V.XYZ() + V2V);
        return aResult;
    }

    public override ResD2 EvalD2(double U, double V)
    {
        ResD2 aSurfD2 = mySurfinit.EvalD2(U, V);
        gp_XY P1 = new gp_XY(U, V);
        gp_XYZ P3 = mySurfinter.Evaluate(P1);
        gp_XYZ V2U_interp = mySurfinter.EvaluateDerivative(P1, 1, 0);
        gp_XYZ V2V_interp = mySurfinter.EvaluateDerivative(P1, 0, 1);
        gp_XYZ V2U = mySurfinter.EvaluateDerivative(P1, 2, 0);
        gp_XYZ V2V = mySurfinter.EvaluateDerivative(P1, 0, 2);
        gp_XYZ V2UV = mySurfinter.EvaluateDerivative(P1, 1, 1);
        ResD2 aResult;
        aResult.Point = new gp_Pnt(P3 + aSurfD2.Point.XYZ());
        aResult.D1U = new gp_Vec(aSurfD2.D1U.XYZ() + V2U_interp);
        aResult.D1V = new gp_Vec(aSurfD2.D1V.XYZ() + V2V_interp);
        aResult.D2U = new gp_Vec(aSurfD2.D2U.XYZ() + V2U);
        aResult.D2V = new gp_Vec(aSurfD2.D2V.XYZ() + V2V);
        aResult.D2UV = new gp_Vec(aSurfD2.D2UV.XYZ() + V2UV);
        return aResult;
    }

    public override ResD3 EvalD3(double U, double V)
    {
        throw new Geom_UndefinedDerivative("GeomPlate_Surface::EvalD3");
    }

    public override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        throw new Geom_UndefinedDerivative("GeomPlate_Surface::EvalDN");
    }

    /// <summary>A new, unfrozen surface on the same initial surface and plate; as OCCT's, without the bounds.</summary>
    public override Geom_Geometry Copy()
    {
        GeomPlate_Surface GPS = new GeomPlate_Surface(mySurfinit, mySurfinter);
        return GPS;
    }

    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
    }

    /// <summary>The initial surface.</summary>
    public Geom_Surface CallSurfinit() => mySurfinit;

    /// <exception cref="Standard_Failure">The bounds are reversed or equal.</exception>
    internal void SetBounds(double Umin, double Umax, double Vmin, double Vmax)
    {
        CheckMutable();
        if ((Umin > Umax) || (Vmin > Vmax))
        {
            throw new Standard_Failure("Bounds haven't the good sense");
        }
        if ((Umin == Umax) || (Vmin == Vmax))
        {
            throw new Standard_Failure("Bounds are equal");
        }
        myUmin = Umin;
        myUmax = Umax;
        myVmin = Vmin;
        myVmax = Vmax;
    }

    /// <summary>A copy of the surface with the given bounds (the twin of OCCT's <c>SetBounds</c>).</summary>
    /// <exception cref="Standard_Failure">The bounds are reversed or equal.</exception>
    public GeomPlate_Surface WithBounds(double Umin, double Umax, double Vmin, double Vmax)
    {
        GeomPlate_Surface aCopy = (GeomPlate_Surface)Copy();
        aCopy.SetBounds(Umin, Umax, Vmin, Vmax);
        return aCopy;
    }

    /// <summary>The UV box of the plate's constraints.</summary>
    public void RealBounds(out double U1, out double U2, out double V1, out double V2)
    {
        mySurfinter.UVBox(out U1, out U2, out V1, out V2);
    }

    /// <summary>Appends the UV points of the plate's constraints to <paramref name="Seq"/>.</summary>
    public void Constraints(NCollection_Sequence<gp_XY> Seq)
    {
        mySurfinter.UVConstraints(Seq);
    }
}
