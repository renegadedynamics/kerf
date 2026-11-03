// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_EvolvedSection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_EvolvedSection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Occt;

/// <summary>A section law that scales one section by a law: section(v) = Law(v) * C, scaled about the origin.</summary>
/// <remarks>
/// A working object (decision 21), as <see cref="GeomFill_UniformSection"/>: it keeps a copy of the section and, when
/// the section is a B-spline curve, the caller's curve itself (read only, as OCCT keeps the handle), and a trimmed
/// copy of the law (<see cref="Law_Function.Trim"/>). <see cref="ConstantSection"/> builds new, unfrozen geometry.
/// </remarks>
public class GeomFill_EvolvedSection : GeomFill_SectionLaw
{
    private readonly double First;
    private readonly double Last;
    private readonly Geom_Curve mySection;
    private readonly Law_Function myLaw;
    private Law_Function TLaw;
    private readonly Geom_BSplineCurve myCurve;

    /// <summary>The section C scaled by the law L, over L's bounds.</summary>
    public GeomFill_EvolvedSection(Geom_Curve C, Law_Function L)
    {
        L.Bounds(out First, out Last);
        mySection = (Geom_Curve)C.Copy();
        myLaw = L.Trim(First, Last, 1.0e-20);
        TLaw = myLaw;
        Geom_BSplineCurve? aCurve = C as Geom_BSplineCurve;
        if (aCurve is null)
        {
            aCurve = GeomConvert.CurveToBSplineCurve(C, QuasiAngular);
            if (aCurve.IsPeriodic())
            {
                int M = aCurve.Degree() / 2 + 1;
                aCurve.RemoveKnot(1, M, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
        myCurve = aCurve;
    }

    public override bool D0(double U, NCollection_Array1<Point3D> Poles, NCollection_Array1<double> Weights)
    {
        double val;
        int ii, L = Poles.Length();
        val = TLaw.Value(U);
        Poles.Assign(myCurve.InternalPoles());
        for (ii = 1; ii <= L; ii++)
        {
            Poles[ii].ChangeCoord() *= val;
        }
        Weights.Assign(myCurve.InternalWeightsArray());

        return true;
    }

    public override bool D1(double U, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                            NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights)
    {
        double val, dval;
        int ii, L = Poles.Length();
        TLaw.D1(U, out val, out dval);

        Poles.Assign(myCurve.InternalPoles());
        Weights.Assign(myCurve.InternalWeightsArray());
        for (ii = 1; ii <= L; ii++)
        {
            DPoles[ii].SetXYZ(Poles[ii].XYZ());
            DPoles[ii] *= dval;
            Poles[ii].ChangeCoord() *= val;
        }
        DWeights.Init(0);

        return true;
    }

    public override bool D2(double U, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                            NCollection_Array1<Vector3D> D2Poles, NCollection_Array1<double> Weights,
                            NCollection_Array1<double> DWeights, NCollection_Array1<double> D2Weights)
    {
        double val, dval, d2val;
        int ii, L = Poles.Length();
        TLaw.D2(U, out val, out dval, out d2val);
        Poles.Assign(myCurve.InternalPoles());
        Weights.Assign(myCurve.InternalWeightsArray());

        for (ii = 1; ii <= L; ii++)
        {
            DPoles[ii].SetXYZ(Poles[ii].XYZ());
            D2Poles[ii] = DPoles[ii];
            D2Poles[ii] *= d2val;
            DPoles[ii] *= dval;
            Poles[ii].ChangeCoord() *= val;
        }

        DWeights.Init(0);
        D2Weights.Init(0);

        return true;
    }

    /// <summary>Null: OCCT's construction of the surface is commented out.</summary>
    public override Geom_BSplineSurface? BSplineSurface() => null;

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

    public override bool IsVPeriodic() => (Math.Abs(myLaw.Value(First) - myLaw.Value(Last)) < global::Kerf.Engine.Foundation.Precision.Confusion());

    public override int NbIntervals(Continuity S) => myLaw.NbIntervals(S);

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        myLaw.Intervals(T, S);
    }

    public override void SetInterval(double F, double L)
    {
        TLaw = myLaw.Trim(F, L, global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void GetInterval(out double F, out double L)
    {
        TLaw.Bounds(out F, out L);
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

    public override Point3D BarycentreOfSurf()
    {
        // OCCT: starts at the section's first parameter but steps to the B-spline's last one (as UniformSection).
        double U = mySection.FirstParameter(), Delta, b;
        int ii;
        Point3D P;
        Point3D Bary = new Point3D();

        Delta = (myCurve.LastParameter() - U) / 20;
        Bary.SetCoord(0.0, 0.0, 0.0);
        for (ii = 0; ii <= 20; ii++, U += Delta)
        {
            P = myCurve.Value(U);
            Bary.ChangeCoord() += P.XYZ();
        }

        U = First;
        Delta = (Last - First) / 20;
        for (ii = 0, b = 0.0; ii <= 20; ii++, U += Delta)
        {
            b += myLaw.Value(U);
        }
        Bary.ChangeCoord() *= b / (21 * 21);
        return Bary;
    }

    public override double MaximalSection()
    {
        double L, val, max, U, Delta;
        int ii;
        GeomAdaptor_Curve AC = new GeomAdaptor_Curve(mySection);
        L = GCPnts_AbscissaPoint.Length(AC);

        Delta = (Last - First) / 20;
        for (ii = 0, max = 0.0, U = First; ii <= 20; ii++, U += Delta)
        {
            val = myLaw.Value(U);
            if (val > max)
            {
                max = val;
            }
        }
        return L * max;
    }

    public override void GetMinimalWeight(NCollection_Array1<double> Weights)
    {
        Weights.Assign(myCurve.InternalWeightsArray());
    }

    public override bool IsConstant(ref double Error)
    {
        //  double isconst = false;
        bool isconst = false;
        Error = 0.0;
        return isconst;
    }

    /// <summary>Throws StdFail_NotDone: the law is never constant (<see cref="IsConstant"/>).</summary>
    public override Geom_Curve ConstantSection()
    {
        double Err = 0, scale;
        if (!IsConstant(ref Err))
        {
            throw new NotDoneException("The Law is not Constant!");
        }
        Transform3D T = new Transform3D();
        Point3D P = new Point3D(0, 0, 0);
        scale = myLaw.Value(First) + myLaw.Value((First + Last) / 2) + myLaw.Value(Last);
        T.SetScale(P, scale / 3);

        Geom_Curve C;
        C = (Geom_Curve)mySection.Copy();
        C.Transform(T);
        return C;
    }
}
