// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionGuide.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionGuide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
public sealed class GeomFill_FunctionGuide : math_FunctionSetWithDerivatives
{
    private readonly Adaptor3d_Curve TheGuide;
    private readonly GeomFill_SectionLaw TheLaw;
    private readonly bool isconst;
    private Geom_Curve? TheCurve;
    private readonly Geom_Curve? TheConst;
    private Geom_Surface? TheSurface;
    private readonly double First;
    private readonly double Last;
    private readonly double TheUonS;
    private gp_XYZ Centre;
    private gp_XYZ Dir;

    /// <summary>Initialises the section and the guide.</summary>
    public GeomFill_FunctionGuide(GeomFill_SectionLaw S, Adaptor3d_Curve Guide, double ParamOnLaw = 0.0)
    {
        TheGuide = Guide;
        TheLaw = S;
        isconst = false;
        First = 0.0;
        Last = 0.0;
        TheUonS = ParamOnLaw;

        double Tol = Precision.Confusion();
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
    public void SetParam(double Param, in gp_Pnt C, in gp_XYZ D, in gp_XYZ DX)
    {
        Centre = C.XYZ();
        Dir = D;

        // repere fixe
        gp_Ax3 Rep = new gp_Ax3(gp.Origin(), gp.DZ(), gp.DX());

        // calculer transfo entre triedre et Oxyz
        gp_Dir B2 = new gp_Dir(DX);
        gp_Ax3 RepTriedre = new gp_Ax3(C, new gp_Dir(D), B2);
        gp_Trsf Transfo = new gp_Trsf();
        Transfo.SetTransformation(RepTriedre, Rep);

        if (isconst)
        {
            TheCurve = new Geom_TrimmedCurve((Geom_Curve)TheConst!.Copy(), First, Last);
        }
        else
        {
            TheLaw.SectionShape(out int NbPoles, out int NbKnots, out int Deg);
            NCollection_Array1<int> Mult = new(1, NbKnots);
            TheLaw.Mults(Mult);
            NCollection_Array1<double> Knots = new(1, NbKnots);
            TheLaw.Knots(Knots);
            NCollection_Array1<gp_Pnt> Poles = new(1, NbPoles);
            NCollection_Array1<double> Weights = new(1, NbPoles);
            TheLaw.D0(TheUonS, Poles, Weights);
            if (TheLaw.IsRational())
            {
                TheCurve = new Geom_BSplineCurve(Poles, Weights, Knots, Mult, Deg, TheLaw.IsUPeriodic());
            }
            else
            {
                TheCurve = new Geom_BSplineCurve(Poles, Knots, Mult, Deg, TheLaw.IsUPeriodic());
            }
        }

        gp_Ax1 Axe = new gp_Ax1(C, new gp_Dir(Dir));
        TheCurve.Transform(Transfo);
        TheSurface = new Geom_SurfaceOfRevolution(TheCurve, Axe);
    }

    /// <summary>3: (w, u, v).</summary>
    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector X, math_Vector F)
    {
        TheGuide.D0(X[1], out gp_Pnt P);
        TheSurface!.D0(X[2], X[3], out gp_Pnt P1);

        F[1] = P.Coord(1) - P1.Coord(1);
        F[2] = P.Coord(2) - P1.Coord(2);
        F[3] = P.Coord(3) - P1.Coord(3);

        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        TheGuide.D1(X[1], out _, out gp_Vec DP);
        TheSurface!.D1(X[2], X[3], out _, out gp_Vec DP1U, out gp_Vec DP1V);

        int i;
        for (i = 1; i <= 3; i++)
        {
            D[i, 1] = DP.Coord(i);
            D[i, 2] = -DP1U.Coord(i);
            D[i, 3] = -DP1V.Coord(i);
        } // for

        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, math_Matrix D)
    {
        TheGuide.D1(X[1], out gp_Pnt P, out gp_Vec DP);                              // derivee de la generatrice
        TheSurface!.D1(X[2], X[3], out gp_Pnt P1, out gp_Vec DP1U, out gp_Vec DP1V); // derivee de la new surface

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
    public bool DerivT(math_Vector X, in gp_XYZ DCentre, in gp_XYZ DDir, math_Vector DFDT)
    {
        gp_Vec DS = new gp_Vec();
        DSDT(X[2], X[3], DCentre, DDir, ref DS);

        TheCurve!.D0(X[1], out gp_Pnt P);

        DFDT[1] = P.Coord(1) - DS.Coord(1);
        DFDT[2] = P.Coord(2) - DS.Coord(2);
        DFDT[3] = P.Coord(3) - DS.Coord(3);

        return true;
    }

    // The derivative of the surface in t at (U, V).
    private void DSDT(double U, double V, in gp_XYZ DC, in gp_XYZ DDir, ref gp_Vec DS)
    {
        // C origine sur l'axe de revolution
        // Vdir vecteur unitaire definissant la direction de l'axe de revolution
        // Q(v) point de parametre V sur la courbe de revolution
        // OM (u,v) = OC + CQ * std::cos(U) + (CQ.Vdir)(1-std::cos(U)) * Vdir +
        //            (Vdir^CQ)* std::sin(U)

        TheCurve!.D0(V, out gp_Pnt Pc); // Q(v)
        // OCCT's Q is a reference into Pc, which is not read afterwards: a copy here.
        gp_XYZ Q = Pc.XYZ(), DQ = new gp_XYZ(0, 0, 0); // Q
        // OCCT prints "Not implemented" to std::cout here when the section is not constant; the port prints nothing.

        Q.Subtract(Centre); // CQ
        DQ -= DC;

        gp_XYZ DVcrossCQ = new gp_XYZ();
        DVcrossCQ.SetLinearForm(DDir.Crossed(Q), Dir.Crossed(DQ)); // Vdir^CQ
        DVcrossCQ.Multiply(Math.Sin(U));                           //(Vdir^CQ)*Sin(U)

        double CosU = Math.Cos(U);
        gp_XYZ DVdotCQ = new gp_XYZ();
        DVdotCQ.SetLinearForm(DDir.Dot(Q) + Dir.Dot(DQ), Dir, Dir.Dot(Q), DDir); //(CQ.Vdir)(1-std::cos(U))Vdir
        DVdotCQ.Add(DVcrossCQ);                                                    // addition des composantes

        DQ.Multiply(CosU);
        DQ.Add(DVdotCQ);
        DQ.Add(DC);
        DS.SetXYZ(DQ);
    }
}
