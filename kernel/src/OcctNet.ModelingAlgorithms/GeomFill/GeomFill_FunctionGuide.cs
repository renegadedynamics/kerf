// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionGuide.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionGuide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function G(w) - S(teta, v) of a sweep with a guide, G the guide and S the surface of revolution of the
/// section about the current axis, with its derivatives.
/// </summary>
/// <remarks>
/// A working object: <see cref="SetParam"/> rebuilds the surface of revolution. It keeps the caller's section law
/// and guide, and only evaluates them; the section it revolves is a copy or a curve it builds.
/// OCCT declares <c>Deriv2T</c> but its definition is commented out; it is not ported.
/// </remarks>
public sealed class GeomFill_FunctionGuide : FunctionSetWithDerivatives
{
    private readonly CurveAdaptorBase TheGuide;
    private readonly GeomFill_SectionLaw TheLaw;
    private readonly bool isconst;
    private Curve3D? TheCurve;
    private readonly Curve3D? TheConst;
    private Surface? TheSurface;
    private readonly double First;
    private readonly double Last;
    private readonly double TheUonS;
    private Xyz Centre;
    private Xyz Dir;

    /// <summary>Initialises the section and the guide.</summary>
    public GeomFill_FunctionGuide(GeomFill_SectionLaw S, CurveAdaptorBase Guide, double ParamOnLaw = 0.0)
    {
        TheGuide = Guide;
        TheLaw = S;
        isconst = false;
        First = 0.0;
        Last = 0.0;
        TheUonS = ParamOnLaw;

        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (TheLaw.IsConstant(ref Tol))
        {
            isconst = true;
            TheConst = TheLaw.ConstantSection();
            First = TheConst.FirstParameter();
            Last = TheConst.LastParameter();
        }
        else
        {
            isconst = false;
            TheConst = null;
        }
        TheCurve = null;
    }

    /// <summary>Builds the surface of revolution of the section about the axis (C, D), with DX as X direction.</summary>
    public void SetParam(double Param, in Point3D C, in Xyz D, in Xyz DX)
    {
        Centre = C.XYZ();
        Dir = D;

        // repere fixe
        CoordinateSystem Rep = new CoordinateSystem(Constants.Origin(), Constants.DZ(), Constants.DX());

        // calculer transfo entre triedre et Oxyz
        Direction3D B2 = new Direction3D(DX);
        CoordinateSystem RepTriedre = new CoordinateSystem(C, new Direction3D(D), B2);
        Transform3D Transfo = new Transform3D();
        Transfo.SetTransformation(RepTriedre, Rep);

        if (isconst)
        {
            TheCurve = new TrimmedCurve3D((Curve3D)TheConst!.Copy(), First, Last);
        }
        else
        {
            TheLaw.SectionShape(out int NbPoles, out int NbKnots, out int Deg);
            NCollection_Array1<int> Mult = new(1, NbKnots);
            TheLaw.Mults(Mult);
            NCollection_Array1<double> Knots = new(1, NbKnots);
            TheLaw.Knots(Knots);
            NCollection_Array1<Point3D> Poles = new(1, NbPoles);
            NCollection_Array1<double> Weights = new(1, NbPoles);
            TheLaw.D0(TheUonS, Poles, Weights);
            if (TheLaw.IsRational())
            {
                TheCurve = new BSplineCurve3D(Poles, Weights, Knots, Mult, Deg, TheLaw.IsUPeriodic());
            }
            else
            {
                TheCurve = new BSplineCurve3D(Poles, Knots, Mult, Deg, TheLaw.IsUPeriodic());
            }
        }

        Axis3D Axe = new Axis3D(C, new Direction3D(Dir));
        TheCurve.Transform(Transfo);
        TheSurface = new RevolutionSurface(TheCurve, Axe);
    }

    /// <summary>3: (w, u, v).</summary>
    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector X, math_Vector F)
    {
        TheGuide.D0(X[1], out Point3D P);
        TheSurface!.D0(X[2], X[3], out Point3D P1);

        F[1] = P.Coord(1) - P1.Coord(1);
        F[2] = P.Coord(2) - P1.Coord(2);
        F[3] = P.Coord(3) - P1.Coord(3);

        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        TheGuide.D1(X[1], out _, out Vector3D DP);
        TheSurface!.D1(X[2], X[3], out _, out Vector3D DP1U, out Vector3D DP1V);

        int i;
        for (i = 1; i <= 3; i++)
        {
            D[i, 1] = DP.Coord(i);
            D[i, 2] = -DP1U.Coord(i);
            D[i, 3] = -DP1V.Coord(i);
        } // for

        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        TheGuide.D1(X[1], out Point3D P, out Vector3D DP);                              // derivee de la generatrice
        TheSurface!.D1(X[2], X[3], out Point3D P1, out Vector3D DP1U, out Vector3D DP1V); // derivee de la new surface

        int i;
        for (i = 1; i <= 3; i++)
        {
            F[i] = P.Coord(i) - P1.Coord(i);

            D[i, 1] = DP.Coord(i);
            D[i, 2] = -DP1U.Coord(i);
            D[i, 3] = -DP1V.Coord(i);
        } // for

        return true;
    }

    /// <summary>The first derivative in t.</summary>
    public bool DerivT(math_Vector X, in Xyz DCentre, in Xyz DDir, math_Vector DFDT)
    {
        Vector3D DS = new Vector3D();
        DSDT(X[2], X[3], DCentre, DDir, ref DS);

        TheCurve!.D0(X[1], out Point3D P);

        DFDT[1] = P.Coord(1) - DS.Coord(1);
        DFDT[2] = P.Coord(2) - DS.Coord(2);
        DFDT[3] = P.Coord(3) - DS.Coord(3);

        return true;
    }

    // The derivative of the surface in t at (U, V).
    private void DSDT(double U, double V, in Xyz DC, in Xyz DDir, ref Vector3D DS)
    {
        // C origine sur l'axe de revolution
        // Vdir vecteur unitaire definissant la direction de l'axe de revolution
        // Q(v) point de parametre V sur la courbe de revolution
        // OM (u,v) = OC + CQ * std::cos(U) + (CQ.Vdir)(1-std::cos(U)) * Vdir +
        //            (Vdir^CQ)* std::sin(U)

        TheCurve!.D0(V, out Point3D Pc); // Q(v)
        // OCCT's Q is a reference into Pc, which is not read afterwards: a copy here.
        Xyz Q = Pc.XYZ(), DQ = new Xyz(0, 0, 0); // Q
        // OCCT prints "Not implemented" to std::cout here when the section is not constant; the port prints nothing.

        Q.Subtract(Centre); // CQ
        DQ -= DC;

        Xyz DVcrossCQ = new Xyz();
        DVcrossCQ.SetLinearForm(DDir.Crossed(Q), Dir.Crossed(DQ)); // Vdir^CQ
        DVcrossCQ.Multiply(Math.Sin(U));                           //(Vdir^CQ)*Sin(U)

        double CosU = Math.Cos(U);
        Xyz DVdotCQ = new Xyz();
        DVdotCQ.SetLinearForm(DDir.Dot(Q) + Dir.Dot(DQ), Dir, Dir.Dot(Q), DDir); //(CQ.Vdir)(1-std::cos(U))Vdir
        DVdotCQ.Add(DVcrossCQ);                                                    // addition des composantes

        DQ.Multiply(CosU);
        DQ.Add(DVdotCQ);
        DQ.Add(DC);
        DS.SetXYZ(DQ);
    }
}
