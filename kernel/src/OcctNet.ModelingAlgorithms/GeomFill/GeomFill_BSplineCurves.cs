// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BSplineCurves.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BSplineCurves.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BSplineCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomFill_FillingStyle;

namespace Occt;

/// <summary>
/// A B-spline surface filling two, three or four contiguous B-spline curves, in the Stretch, Coons or Curved style.
/// The curves are arranged (and reversed if needed) to form the boundary; three curves are closed by a straight
/// segment, and two curves give a ruled surface (Stretch, Coons) or a translated one (Curved).
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Init</c> recomputes the surface. The caller's curves are copied, never
/// changed. <see cref="Surface"/> hands out the surface frozen; ported callers use <see cref="InternalSurface"/>.
/// </remarks>
public class GeomFill_BSplineCurves
{
    private Geom_BSplineSurface? mySurface;

    // Arrange: prepares the filling. The curves are arranged in this way:
    //
    //                      CC3
    //                  ----->-----
    //                 |           |
    //                 |           |
    //                 |           |
    //             CC4 ^           ^ CC2
    //                 |           |
    //                 |           |
    //                  ----->-----
    //                   CC1 = C1
    //
    // A curve degenerated to a point that matches the last end point is taken first.
    private static bool Arrange(Geom_BSplineCurve C1, Geom_BSplineCurve C2, Geom_BSplineCurve C3, Geom_BSplineCurve C4,
                                out Geom_BSplineCurve CC1, out Geom_BSplineCurve CC2, out Geom_BSplineCurve CC3,
                                out Geom_BSplineCurve CC4, double Tol)
    {
        Geom_BSplineCurve[] GC = new Geom_BSplineCurve[4];
        Geom_BSplineCurve Dummy;
        GC[0] = (Geom_BSplineCurve)C1.Copy();
        GC[1] = (Geom_BSplineCurve)C2.Copy();
        GC[2] = (Geom_BSplineCurve)C3.Copy();
        GC[3] = (Geom_BSplineCurve)C4.Copy();
        CC1 = CC2 = CC3 = CC4 = null!;

        int i, j;
        bool Trouve;

        for (i = 1; i <= 3; i++)
        {
            Trouve = false;
            // search for a degenerated curve = point, which would match first
            for (j = i; j <= 3 && !Trouve; j++)
            {
                if (GC[j].StartPoint().Distance(GC[j].EndPoint()) < Tol)
                {
                    // this is a degenerated line, does it match the last endpoint?
                    if (GC[j].StartPoint().Distance(GC[i - 1].EndPoint()) < Tol)
                    {
                        Dummy = GC[i];
                        GC[i] = GC[j];
                        GC[j] = Dummy;
                        Trouve = true;
                    }
                }
            }
            // if no degenerated curve matched, try an ordinary one as next curve
            if (!Trouve)
            {
                for (j = i; j <= 3 && !Trouve; j++)
                {
                    if (GC[j].StartPoint().Distance(GC[i - 1].EndPoint()) < Tol)
                    {
                        Dummy = GC[i];
                        GC[i] = GC[j];
                        GC[j] = Dummy;
                        Trouve = true;
                    }
                    else if (GC[j].EndPoint().Distance(GC[i - 1].EndPoint()) < Tol)
                    {
                        GC[j] = (Geom_BSplineCurve)GC[j].Reversed();
                        Dummy = GC[i];
                        GC[i] = GC[j];
                        GC[j] = Dummy;
                        Trouve = true;
                    }
                }
            }
            // if still non matched -> error, the algorithm cannot finish
            if (!Trouve)
            {
                return false;
            }
        }

        CC1 = GC[0];
        CC2 = GC[1];
        CC3 = (Geom_BSplineCurve)GC[2].Reversed();
        CC4 = (Geom_BSplineCurve)GC[3].Reversed();

        return true;
    }

    private static int SetSameDistribution(ref Geom_BSplineCurve C1, ref Geom_BSplineCurve C2)
    {
        NCollection_Array1<gp_Pnt> P1 = new(C1.InternalPoles());
        NCollection_Array1<double> W1 = C1.InternalWeightsArray();
        NCollection_Array1<double> K1 = new(C1.InternalKnots());
        NCollection_Array1<int> M1 = new(C1.InternalMultiplicities());
        NCollection_Array1<gp_Pnt> P2 = new(C2.InternalPoles());
        NCollection_Array1<double> W2 = C2.InternalWeightsArray();
        NCollection_Array1<double> K2 = new(C2.InternalKnots());
        NCollection_Array1<int> M2 = new(C2.InternalMultiplicities());

        double K11 = K1[1];
        double K12 = K1[K1.Upper()];
        double K21 = K2[1];
        double K22 = K2[K2.Upper()];

        if ((K12 - K11) > (K22 - K21))
        {
            BSplCLib.Reparametrize(K11, K12, K2);
            C2.SetKnots(K2);
        }
        else if ((K12 - K11) < (K22 - K21))
        {
            BSplCLib.Reparametrize(K21, K22, K1);
            C1.SetKnots(K1);
        }
        else if (Math.Abs(K12 - K11) > Precision.PConfusion())
        {
            BSplCLib.Reparametrize(K11, K12, K2);
            C2.SetKnots(K2);
        }

        int NP, NK;
        if (BSplCLib.PrepareInsertKnots(C1.Degree(), false, K1, M1, K2, M2, out NP, out NK, Precision.PConfusion(), false))
        {
            NCollection_Array1<gp_Pnt> NewP = new(1, NP);
            NCollection_Array1<double> NewW = new(1, NP);
            NCollection_Array1<double> NewK = new(1, NK);
            NCollection_Array1<int> NewM = new(1, NK);
            BSplCLib.InsertKnots(C1.Degree(), false, P1, W1, K1, M1, K2, M2, NewP, NewW, NewK, NewM,
                                 Precision.PConfusion(), false);
            if (C1.IsRational())
            {
                C1 = new Geom_BSplineCurve(NewP, NewW, NewK, NewM, C1.Degree());
            }
            else
            {
                C1 = new Geom_BSplineCurve(NewP, NewK, NewM, C1.Degree());
            }
            BSplCLib.InsertKnots(C2.Degree(), false, P2, W2, K2, M2, K1, M1, NewP, NewW, NewK, NewM,
                                 Precision.PConfusion(), false);
            if (C2.IsRational())
            {
                C2 = new Geom_BSplineCurve(NewP, NewW, NewK, NewM, C2.Degree());
            }
            else
            {
                C2 = new Geom_BSplineCurve(NewP, NewK, NewM, C2.Degree());
            }
        }
        else
        {
            throw new Standard_ConstructionError(" ");
        }

        return C1.NbPoles();
    }

    public GeomFill_BSplineCurves()
    {
    }

    /// <summary>The filling of four contiguous curves in the style <paramref name="Type"/>.</summary>
    public GeomFill_BSplineCurves(Geom_BSplineCurve C1, Geom_BSplineCurve C2, Geom_BSplineCurve C3,
                                  Geom_BSplineCurve C4, GeomFill_FillingStyle Type)
    {
        Init(C1, C2, C3, C4, Type);
    }

    /// <summary>The filling of three contiguous curves, closed by a segment, in the style <paramref name="Type"/>.</summary>
    public GeomFill_BSplineCurves(Geom_BSplineCurve C1, Geom_BSplineCurve C2, Geom_BSplineCurve C3,
                                  GeomFill_FillingStyle Type)
    {
        Init(C1, C2, C3, Type);
    }

    /// <summary>The filling between two curves in the style <paramref name="Type"/>.</summary>
    public GeomFill_BSplineCurves(Geom_BSplineCurve C1, Geom_BSplineCurve C2, GeomFill_FillingStyle Type)
    {
        Init(C1, C2, Type);
    }

    /// <summary>
    /// Fills four contiguous curves. Throws Standard_ConstructionError if they don't join, or for the Coons style
    /// with fewer than four poles on a side.
    /// </summary>
    public void Init(Geom_BSplineCurve C1, Geom_BSplineCurve C2, Geom_BSplineCurve C3, Geom_BSplineCurve C4,
                     GeomFill_FillingStyle Type)
    {
        // On ordonne les courbes
        Geom_BSplineCurve CC1, CC2, CC3, CC4;

        double Tol = Precision.Confusion();
        bool IsOK = Arrange(C1, C2, C3, C4, out CC1, out CC2, out CC3, out CC4, Tol);

        Standard_ConstructionError_Raise_if(!IsOK, " GeomFill_BSplineCurves: Courbes non jointives");

        // Mise en conformite des degres
        int Deg1 = CC1.Degree();
        int Deg2 = CC2.Degree();
        int Deg3 = CC3.Degree();
        int Deg4 = CC4.Degree();
        int DegU = Max(Deg1, Deg3);
        int DegV = Max(Deg2, Deg4);
        if (Deg1 < DegU)
        {
            CC1.IncreaseDegree(DegU);
        }
        if (Deg2 < DegV)
        {
            CC2.IncreaseDegree(DegV);
        }
        if (Deg3 < DegU)
        {
            CC3.IncreaseDegree(DegU);
        }
        if (Deg4 < DegV)
        {
            CC4.IncreaseDegree(DegV);
        }

        // Mise en conformite des distributions de noeuds
        int NbUPoles = SetSameDistribution(ref CC1, ref CC3);
        int NbVPoles = SetSameDistribution(ref CC2, ref CC4);

        if (Type == GeomFill_CoonsStyle)
        {
            if (NbUPoles < 4 || NbVPoles < 4)
            {
                throw new Standard_ConstructionError("GeomFill_BSplineCurves: invalid filling style");
            }
        }

        NCollection_Array1<gp_Pnt> P1 = CC1.InternalPoles();
        NCollection_Array1<gp_Pnt> P2 = CC2.InternalPoles();
        NCollection_Array1<gp_Pnt> P3 = CC3.InternalPoles();
        NCollection_Array1<gp_Pnt> P4 = CC4.InternalPoles();

        // Traitement des courbes rationelles
        bool isRat = (CC1.IsRational() || CC2.IsRational() || CC3.IsRational() || CC4.IsRational());

        NCollection_Array1<double> W1 = CC1.InternalWeightsArray();
        NCollection_Array1<double> W2 = CC2.InternalWeightsArray();
        NCollection_Array1<double> W3 = CC3.InternalWeightsArray();
        NCollection_Array1<double> W4 = CC4.InternalWeightsArray();

        GeomFill_Filling Caro = new();
        if (isRat)
        {
            switch (Type)
            {
                case GeomFill_StretchStyle:
                    Caro = new GeomFill_Stretch(P1, P2, P3, P4, W1, W2, W3, W4);
                    break;
                case GeomFill_CoonsStyle:
                    Caro = new GeomFill_Coons(P1, P4, P3, P2, W1, W4, W3, W2);
                    break;
                case GeomFill_CurvedStyle:
                    Caro = new GeomFill_Curved(P1, P2, P3, P4, W1, W2, W3, W4);
                    break;
            }
        }
        else
        {
            switch (Type)
            {
                case GeomFill_StretchStyle:
                    Caro = new GeomFill_Stretch(P1, P2, P3, P4);
                    break;
                case GeomFill_CoonsStyle:
                    Caro = new GeomFill_Coons(P1, P4, P3, P2);
                    break;
                case GeomFill_CurvedStyle:
                    Caro = new GeomFill_Curved(P1, P2, P3, P4);
                    break;
            }
        }

        NbUPoles = Caro.NbUPoles();
        NbVPoles = Caro.NbVPoles();
        NCollection_Array2<gp_Pnt> Poles = new(1, NbUPoles, 1, NbVPoles);

        // Creation de la surface
        NCollection_Array1<double> UKnots = CC1.InternalKnots();
        NCollection_Array1<int> UMults = CC1.InternalMultiplicities();
        NCollection_Array1<double> VKnots = CC2.InternalKnots();
        NCollection_Array1<int> VMults = CC2.InternalMultiplicities();

        Caro.Poles(Poles);

        if (Caro.isRational())
        {
            NCollection_Array2<double> Weights = new(1, NbUPoles, 1, NbVPoles);
            Caro.Weights(Weights);
            mySurface = new Geom_BSplineSurface(Poles, Weights, UKnots, VKnots, UMults, VMults, CC1.Degree(),
                                                CC2.Degree());
        }
        else
        {
            mySurface = new Geom_BSplineSurface(Poles, UKnots, VKnots, UMults, VMults, CC1.Degree(), CC2.Degree());
        }
    }

    /// <summary>Fills three contiguous curves, the fourth side being the segment between the free ends.</summary>
    public void Init(Geom_BSplineCurve C1, Geom_BSplineCurve C2, Geom_BSplineCurve C3, GeomFill_FillingStyle Type)
    {
        Geom_BSplineCurve C4;
        NCollection_Array1<gp_Pnt> Poles = new(1, 2);
        NCollection_Array1<double> Knots = new(1, 2);
        NCollection_Array1<int> Mults = new(1, 2);
        double Tol = Precision.Confusion();
        Tol = Tol * Tol;
        if (C1.StartPoint().SquareDistance(C2.StartPoint()) > Tol
            && C1.StartPoint().SquareDistance(C2.EndPoint()) > Tol)
        {
            Poles[1] = C1.StartPoint();
        }
        else
        {
            Poles[1] = C1.EndPoint();
        }

        if (C3.StartPoint().SquareDistance(C2.StartPoint()) > Tol
            && C3.StartPoint().SquareDistance(C2.EndPoint()) > Tol)
        {
            Poles[2] = C3.StartPoint();
        }
        else
        {
            Poles[2] = C3.EndPoint();
        }

        Knots[1] = C2.Knot(C2.FirstUKnotIndex());
        Knots[2] = C2.Knot(C2.LastUKnotIndex());
        Mults[1] = Mults[2] = 2;
        C4 = new Geom_BSplineCurve(Poles, Knots, Mults, 1);
        Init(C1, C2, C3, C4, Type);
    }

    /// <summary>
    /// Fills between two curves: a ruled surface for the Stretch and Coons styles, the first curve translated along
    /// the second for the Curved style (which needs the curves to share an end; else Standard_OutOfRange).
    /// </summary>
    public void Init(Geom_BSplineCurve C1, Geom_BSplineCurve C2, GeomFill_FillingStyle Type)
    {
        Geom_BSplineCurve CC1 = (Geom_BSplineCurve)C1.Copy();
        Geom_BSplineCurve CC2 = (Geom_BSplineCurve)C2.Copy();

        int Deg1 = CC1.Degree();
        int Deg2 = CC2.Degree();

        bool isRat = (CC1.IsRational() || CC2.IsRational());

        if (Type != GeomFill_CurvedStyle)
        {
            int DegU = Max(Deg1, Deg2);

            if (CC1.Degree() < DegU)
            {
                CC1.IncreaseDegree(DegU);
            }
            if (CC2.Degree() < DegU)
            {
                CC2.IncreaseDegree(DegU);
            }

            // Mise en conformite des distributions de noeuds
            int NbPoles = SetSameDistribution(ref CC1, ref CC2);
            NCollection_Array2<gp_Pnt> Poles = new(1, NbPoles, 1, 2);
            NCollection_Array1<gp_Pnt> P1 = CC1.InternalPoles();
            NCollection_Array1<gp_Pnt> P2 = CC2.InternalPoles();

            int i;
            for (i = 1; i <= NbPoles; i++)
            {
                Poles[i, 1] = P1[i];
                Poles[i, 2] = P2[i];
            }
            NCollection_Array1<double> UKnots = CC1.InternalKnots();
            NCollection_Array1<int> UMults = CC1.InternalMultiplicities();
            NCollection_Array1<double> VKnots = new(1, 2);
            NCollection_Array1<int> VMults = new(1, 2);
            VKnots[1] = 0;
            VKnots[2] = 1;
            VMults[1] = 2;
            VMults[2] = 2;

            // Traitement des courbes rationelles
            if (isRat)
            {
                NCollection_Array2<double> Weights = new(1, NbPoles, 1, 2);
                NCollection_Array1<double> W1 = CC1.InternalWeightsArray();
                NCollection_Array1<double> W2 = CC2.InternalWeightsArray();
                for (i = 1; i <= NbPoles; i++)
                {
                    Weights[i, 1] = W1[i];
                    Weights[i, 2] = W2[i];
                }
                mySurface = new Geom_BSplineSurface(Poles, Weights, UKnots, VKnots, UMults, VMults, CC1.Degree(), 1,
                                                    CC1.IsPeriodic(), false);
            }
            else
            {
                mySurface = new Geom_BSplineSurface(Poles, UKnots, VKnots, UMults, VMults, CC1.Degree(), 1);
            }
        }
        else
        {
            double Eps = Precision.Confusion();
            bool IsOK = false;
            if (CC1.StartPoint().IsEqual(CC2.StartPoint(), Eps))
            {
                IsOK = true;
            }
            else if (CC1.StartPoint().IsEqual(CC2.EndPoint(), Eps))
            {
                CC2.Reverse();
                IsOK = true;
            }
            else if (CC1.EndPoint().IsEqual(CC2.StartPoint(), Eps))
            {
                // OCCT: reverses the caller's C1, not its copy CC1, so the filling is built from the unreversed
                // copy. The port leaves the caller's curve unchanged (decision 846): the result is OCCT's
                // (occt-issues.md).
                IsOK = true;
            }
            else if (CC1.EndPoint().IsEqual(CC2.EndPoint(), Eps))
            {
                CC1.Reverse();
                CC2.Reverse();
                IsOK = true;
            }

            if (!IsOK)
            {
                throw new Standard_OutOfRange("GeomFill_BSplineCurves: Courbes non jointives");
            }

            int NbUPoles = CC1.NbPoles();
            int NbVPoles = CC2.NbPoles();
            NCollection_Array1<gp_Pnt> P1 = CC1.InternalPoles();
            NCollection_Array1<gp_Pnt> P2 = CC2.InternalPoles();

            NCollection_Array1<double> UKnots = CC1.InternalKnots();
            NCollection_Array1<int> UMults = CC1.InternalMultiplicities();
            NCollection_Array1<double> VKnots = CC2.InternalKnots();
            NCollection_Array1<int> VMults = CC2.InternalMultiplicities();

            GeomFill_Filling Caro;
            if (isRat)
            {
                NCollection_Array1<double> W1 = CC1.InternalWeightsArray();
                NCollection_Array1<double> W2 = CC2.InternalWeightsArray();
                Caro = new GeomFill_Curved(P1, P2, W1, W2);
            }
            else
            {
                Caro = new GeomFill_Curved(P1, P2);
            }

            NbUPoles = Caro.NbUPoles();
            NbVPoles = Caro.NbVPoles();
            NCollection_Array2<gp_Pnt> Poles = new(1, NbUPoles, 1, NbVPoles);
            Caro.Poles(Poles);

            if (Caro.isRational())
            {
                NCollection_Array2<double> Weights = new(1, NbUPoles, 1, NbVPoles);
                Caro.Weights(Weights);
                mySurface = new Geom_BSplineSurface(Poles, Weights, UKnots, VKnots, UMults, VMults, Deg1, Deg2, false,
                                                    false);
            }
            else
            {
                mySurface = new Geom_BSplineSurface(Poles, UKnots, VKnots, UMults, VMults, Deg1, Deg2, false, false);
            }
        }
    }

    /// <summary>The surface (null before <c>Init</c>), frozen.</summary>
    public Geom_BSplineSurface? Surface()
    {
        mySurface?.Freeze();
        return mySurface;
    }

    /// <summary>The surface, not frozen, for ported callers.</summary>
    internal Geom_BSplineSurface? InternalSurface() => mySurface;
}
