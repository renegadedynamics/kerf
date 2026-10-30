// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_UniformSection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_UniformSection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;

namespace Occt;

/// <summary>A constant section law: the same B-spline section for every v in [First, Last].</summary>
/// <remarks>
/// A working object (decision 21). It keeps a copy of the section and, when the section is a B-spline curve, the
/// caller's curve itself, as OCCT keeps the handle (it only reads it). <see cref="ConstantSection"/> and
/// <see cref="BSplineSurface"/> build new, unfrozen geometry.
/// </remarks>
public class GeomFill_UniformSection : GeomFill_SectionLaw
{
    private readonly Geom_Curve mySection;
    private readonly Geom_BSplineCurve myCurve;
    private readonly double First;
    private readonly double Last;

    /// <summary>A constant section law of the section C, between the law's parameters FirstParameter and LastParameter.</summary>
    public GeomFill_UniformSection(Geom_Curve C, double FirstParameter = 0.0, double LastParameter = 1.0)
    {
        First = FirstParameter;
        Last = LastParameter;
        mySection = (Geom_Curve)C.Copy();
        Geom_BSplineCurve? aCurve = C as Geom_BSplineCurve;
        if (aCurve is null)
        {
            aCurve = GeomConvert.CurveToBSplineCurve(C, Convert_QuasiAngular);
            if (aCurve.IsPeriodic())
            {
                int M = aCurve.Degree() / 2 + 1;
                aCurve.RemoveKnot(1, M, Precision.Confusion());
            }
        }
        myCurve = aCurve;
    }

    public override bool D0(double Param, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> Weights)
    {
        Poles.Assign(myCurve.InternalPoles());
        Weights.Assign(myCurve.InternalWeightsArray());
        return true;
    }

    public override bool D1(double Param, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<gp_Vec> DPoles,
                            NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights)
    {
        Poles.Assign(myCurve.InternalPoles());
        Weights.Assign(myCurve.InternalWeightsArray());
        gp_Vec V0 = new gp_Vec(0, 0, 0);
        DPoles.Init(V0);
        DWeights.Init(0);
        return true;
    }

    public override bool D2(double Param, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<gp_Vec> DPoles,
                            NCollection_Array1<gp_Vec> D2Poles, NCollection_Array1<double> Weights,
                            NCollection_Array1<double> DWeights, NCollection_Array1<double> D2Weights)
    {
        Poles.Assign(myCurve.InternalPoles());
        Weights.Assign(myCurve.InternalWeightsArray());
        gp_Vec V0 = new gp_Vec(0, 0, 0);
        DPoles.Init(V0);
        DWeights.Init(0);
        D2Poles.Init(V0);
        D2Weights.Init(0);
        return true;
    }

    /// <summary>The surface swept by the section: a new, unfrozen surface of degree 1 in v.</summary>
    public override Geom_BSplineSurface? BSplineSurface()
    {
        int ii, NbPoles = myCurve.NbPoles();
        NCollection_Array2<gp_Pnt> Poles = new(1, NbPoles, 1, 2);
        NCollection_Array1<double> CurKnots = myCurve.InternalKnots();
        NCollection_Array1<double> UKnots = new(CurKnots);
        NCollection_Array1<double> VKnots = new(1, 2);
        NCollection_Array1<int> CurMults = myCurve.InternalMultiplicities();
        NCollection_Array1<int> UMults = new(CurMults);
        NCollection_Array1<int> VMults = new(1, 2);

        for (ii = 1; ii <= NbPoles; ii++)
        {
            Poles[ii, 1] = Poles[ii, 2] = myCurve.Pole(ii);
        }

        VKnots[1] = First;
        VKnots[2] = Last;

        VMults.Init(2);

        Geom_BSplineSurface BS = new Geom_BSplineSurface(Poles, UKnots, VKnots, UMults, VMults, myCurve.Degree(), 1,
                                                         myCurve.IsPeriodic());

        return BS;
    }

    public override void SectionShape(out int NbPoles, out int NbKnots, out int Degree)
    {
        NbPoles = myCurve.NbPoles();
        NbKnots = myCurve.NbKnots();
        Degree = myCurve.Degree();
    }

    public override void Knots(NCollection_Array1<double> TKnots)
    {
        TKnots.Assign(myCurve.InternalKnots());
    }

    public override void Mults(NCollection_Array1<int> TMults)
    {
        TMults.Assign(myCurve.InternalMultiplicities());
    }

    public override bool IsRational() => myCurve.IsRational();

    public override bool IsUPeriodic() => myCurve.IsPeriodic();

    public override bool IsVPeriodic() => true;

    public override int NbIntervals(GeomAbs_Shape S) => 1;

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        T[T.Lower()] = First;
        T[T.Upper()] = Last;
    }

    public override void SetInterval(double First, double Last)
    {
        // Ne fait Rien
    }

    public override void GetInterval(out double F, out double L)
    {
        F = First;
        L = Last;
    }

    public override void GetDomain(out double F, out double L)
    {
        F = First;
        L = Last;
    }

    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol, NCollection_Array1<double> Tol3d)
    {
        Tol3d.Init(SurfTol);
        if (BoundTol < SurfTol)
        {
            Tol3d[Tol3d.Lower()] = BoundTol;
            Tol3d[Tol3d.Upper()] = BoundTol;
        }
    }

    public override gp_Pnt BarycentreOfSurf()
    {
        // OCCT: starts at the section's first parameter but steps to the B-spline's last one.
        double U = mySection.FirstParameter(), Delta;
        gp_Pnt P;
        gp_Pnt Bary = new gp_Pnt();

        Delta = (myCurve.LastParameter() - U) / 20;
        Bary.SetCoord(0.0, 0.0, 0.0);
        for (int ii = 0; ii <= 20; ii++, U += Delta)
        {
            P = myCurve.Value(U);
            Bary.ChangeCoord() += P.XYZ();
        }

        Bary.ChangeCoord() /= 21.0;
        return Bary;
    }

    public override double MaximalSection()
    {
        GeomAdaptor_Curve AC = new GeomAdaptor_Curve(mySection);
        return GCPnts_AbscissaPoint.Length(AC);
    }

    public override void GetMinimalWeight(NCollection_Array1<double> Weights)
    {
        Weights.Assign(myCurve.InternalWeightsArray());
    }

    public override bool IsConstant(ref double Error)
    {
        Error = 0.0;
        return true;
    }

    /// <summary>A new copy of the section.</summary>
    public override Geom_Curve ConstantSection()
    {
        Geom_Curve C;
        C = (Geom_Curve)mySection.Copy();
        return C;
    }
}
