// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BezierCurves.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BezierCurves.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BezierCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomFill_FillingStyle;

namespace Occt;

/// <summary>
/// A Bezier surface filling two, three or four contiguous Bezier curves, in the Stretch, Coons or Curved style.
/// The curves are arranged (and reversed if needed) to form the boundary; three curves are closed by a straight
/// segment, and two curves give a ruled surface (Stretch, Coons) or a translated one (Curved).
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Init</c> recomputes the surface. The caller's curves are copied, never
/// changed. <see cref="Surface"/> hands out the surface frozen; ported callers use <see cref="InternalSurface"/>.
/// </remarks>
public class GeomFill_BezierCurves
{
    private Geom_BezierSurface? mySurface;

    // SetSameWeights uses the following property of rational Bezier curves: if Wi = Weight(i), Pi = Pole(i),
    // n = NbPoles, then for any a, b, c != 0 the transformation Wi = a * b^i * c^(n-i) doesn't modify the geometry
    // of the curve; only the lengths of the derivatives change.
    private static void SetSameWeights(NCollection_Array1<double> W1, NCollection_Array1<double> W2,
                                       NCollection_Array1<double> W3, NCollection_Array1<double> W4)
    {
        double Eps = Precision.Confusion();

        int NU = W1.Length();
        int NV = W2.Length();

        double A = (W1[1] * W2[1]) / (W1[NU] * W2[NV]);
        double B = (W3[1] * W4[1]) / (W3[NU] * W4[NV]);

        int i;
        double Alfa = W1[NU] / W2[1];
        for (i = 1; i <= NV; i++)
        {
            W2[i] *= Alfa;
        }
        double Beta = W2[NV] / W3[NU];
        for (i = 1; i <= NU; i++)
        {
            W3[i] *= Beta;
        }
        double Gamma = W3[1] / W4[NV];
        for (i = 1; i <= NV; i++)
        {
            W4[i] *= Gamma;
        }

        if (Math.Abs(A - B) > Eps)
        {
            double w = Math.Pow(W1[1] / W4[1], 1.0 / (double)(NV - 1));
            double x = w;
            for (i = NV - 1; i >= 1; i--)
            {
                W4[i] *= x;
                x *= w;
            }
        }
    }

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
    private static bool Arrange(Geom_BezierCurve C1, Geom_BezierCurve C2, Geom_BezierCurve C3, Geom_BezierCurve C4,
                                out Geom_BezierCurve CC1, out Geom_BezierCurve CC2, out Geom_BezierCurve CC3,
                                out Geom_BezierCurve CC4, double Tol)
    {
        Geom_BezierCurve[] GC = new Geom_BezierCurve[4];
        Geom_BezierCurve Dummy;
        GC[0] = (Geom_BezierCurve)C1.Copy();
        GC[1] = (Geom_BezierCurve)C2.Copy();
        GC[2] = (Geom_BezierCurve)C3.Copy();
        GC[3] = (Geom_BezierCurve)C4.Copy();
        CC1 = CC2 = CC3 = CC4 = null!;

        int i, j;
        bool Trouve;

        for (i = 1; i <= 3; i++)
        {
            Trouve = false;
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
                    GC[j] = (Geom_BezierCurve)GC[j].Reversed();
                    Dummy = GC[i];
                    GC[i] = GC[j];
                    GC[j] = Dummy;
                    Trouve = true;
                }
            }
            if (!Trouve)
            {
                return false;
            }
        }

        CC1 = GC[0];
        CC2 = GC[1];
        CC3 = (Geom_BezierCurve)GC[2].Reversed();
        CC4 = (Geom_BezierCurve)GC[3].Reversed();

        return true;
    }

    // OCCT's Poles() and WeightsArray() const references: copies of the values (the curves are this object's own).
    private static NCollection_Array1<gp_Pnt> PolesOf(Geom_BezierCurve C)
    {
        NCollection_Array1<gp_Pnt> P = new(1, C.NbPoles());
        C.Poles(P);
        return P;
    }

    private static NCollection_Array1<double> WeightsOf(Geom_BezierCurve C)
    {
        NCollection_Array1<double> W = new(1, C.NbPoles());
        C.Weights(W);
        return W;
    }

    public GeomFill_BezierCurves()
    {
    }

    /// <summary>The filling of four contiguous curves in the style <paramref name="Type"/>.</summary>
    public GeomFill_BezierCurves(Geom_BezierCurve C1, Geom_BezierCurve C2, Geom_BezierCurve C3, Geom_BezierCurve C4,
                                 GeomFill_FillingStyle Type)
    {
        Init(C1, C2, C3, C4, Type);
    }

    /// <summary>The filling of three contiguous curves, closed by a segment, in the style <paramref name="Type"/>.</summary>
    public GeomFill_BezierCurves(Geom_BezierCurve C1, Geom_BezierCurve C2, Geom_BezierCurve C3,
                                 GeomFill_FillingStyle Type)
    {
        Init(C1, C2, C3, Type);
    }

    /// <summary>The filling between two curves in the style <paramref name="Type"/>.</summary>
    public GeomFill_BezierCurves(Geom_BezierCurve C1, Geom_BezierCurve C2, GeomFill_FillingStyle Type)
    {
        Init(C1, C2, Type);
    }

    /// <summary>Fills four contiguous curves; throws Standard_ConstructionError if they don't join.</summary>
    public void Init(Geom_BezierCurve C1, Geom_BezierCurve C2, Geom_BezierCurve C3, Geom_BezierCurve C4,
                     GeomFill_FillingStyle Type)
    {
        // On ordonne les courbes
        Geom_BezierCurve CC1, CC2, CC3, CC4;

        double Tol = Precision.Confusion();
        // OCCT's release build ignores the result (No_Exception); the port checks it (decision 9).
        bool IsOK = Arrange(C1, C2, C3, C4, out CC1, out CC2, out CC3, out CC4, Tol);

        Standard_ConstructionError_Raise_if(!IsOK, " GeomFill_BezierCurves: Courbes non jointives");

        // Mise en conformite des degres
        int DegU = Max(CC1.Degree(), CC3.Degree());
        int DegV = Max(CC2.Degree(), CC4.Degree());

        if (Type == GeomFill_CoonsStyle)
        {
            DegU = Max(DegU, 3);
            DegV = Max(DegV, 3);
        }

        if (CC1.Degree() < DegU)
        {
            CC1.Increase(DegU);
        }
        if (CC2.Degree() < DegV)
        {
            CC2.Increase(DegV);
        }
        if (CC3.Degree() < DegU)
        {
            CC3.Increase(DegU);
        }
        if (CC4.Degree() < DegV)
        {
            CC4.Increase(DegV);
        }

        NCollection_Array1<gp_Pnt> P1 = PolesOf(CC1);
        NCollection_Array1<gp_Pnt> P2 = PolesOf(CC2);
        NCollection_Array1<gp_Pnt> P3 = PolesOf(CC3);
        NCollection_Array1<gp_Pnt> P4 = PolesOf(CC4);

        // Traitement des courbes rationelles
        bool isRat = (CC1.IsRational() || CC2.IsRational() || CC3.IsRational() || CC4.IsRational());

        GeomFill_Filling Caro = new();
        if (isRat)
        {
            NCollection_Array1<double> W1 = WeightsOf(CC1);
            NCollection_Array1<double> W2 = WeightsOf(CC2);
            NCollection_Array1<double> W3 = WeightsOf(CC3);
            NCollection_Array1<double> W4 = WeightsOf(CC4);

            // Mise en conformite des poids aux coins.
            SetSameWeights(W1, W2, W3, W4);

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

        int NbUPoles = Caro.NbUPoles();
        int NbVPoles = Caro.NbVPoles();
        NCollection_Array2<gp_Pnt> Poles = new(1, NbUPoles, 1, NbVPoles);
        Caro.Poles(Poles);

        if (Caro.isRational())
        {
            NCollection_Array2<double> Weights = new(1, NbUPoles, 1, NbVPoles);
            Caro.Weights(Weights);
            mySurface = new Geom_BezierSurface(Poles, Weights);
        }
        else
        {
            mySurface = new Geom_BezierSurface(Poles);
        }
    }

    /// <summary>Fills three contiguous curves, the fourth side being the segment between the free ends.</summary>
    public void Init(Geom_BezierCurve C1, Geom_BezierCurve C2, Geom_BezierCurve C3, GeomFill_FillingStyle Type)
    {
        Geom_BezierCurve C4;
        NCollection_Array1<gp_Pnt> Poles = new(1, 2);
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
        C4 = new Geom_BezierCurve(Poles);
        Init(C1, C2, C3, C4, Type);
    }

    /// <summary>
    /// Fills between two curves: a ruled surface for the Stretch and Coons styles, the first curve translated along
    /// the second for the Curved style (which needs the curves to share an end; else Standard_OutOfRange).
    /// </summary>
    public void Init(Geom_BezierCurve C1, Geom_BezierCurve C2, GeomFill_FillingStyle Type)
    {
        Geom_BezierCurve CC1 = (Geom_BezierCurve)C1.Copy();
        Geom_BezierCurve CC2 = (Geom_BezierCurve)C2.Copy();

        int Deg1 = CC1.Degree();
        int Deg2 = CC2.Degree();

        bool isRat = (CC1.IsRational() || CC2.IsRational());

        if (Type != GeomFill_CurvedStyle)
        {
            int DegU = Max(Deg1, Deg2);

            if (CC1.Degree() < DegU)
            {
                CC1.Increase(DegU);
            }
            if (CC2.Degree() < DegU)
            {
                CC2.Increase(DegU);
            }

            NCollection_Array2<gp_Pnt> Poles = new(1, DegU + 1, 1, 2);
            NCollection_Array1<gp_Pnt> P1 = PolesOf(CC1);
            NCollection_Array1<gp_Pnt> P2 = PolesOf(CC2);

            int i;
            for (i = 1; i <= DegU + 1; i++)
            {
                Poles[i, 1] = P1[i];
                Poles[i, 2] = P2[i];
            }
            if (isRat)
            {
                NCollection_Array1<double> W1 = WeightsOf(CC1);
                NCollection_Array1<double> W2 = WeightsOf(CC2);
                NCollection_Array2<double> Weights = new(1, DegU + 1, 1, 2);
                for (i = 1; i <= DegU + 1; i++)
                {
                    Weights[i, 1] = W1[i];
                    Weights[i, 2] = W2[i];
                }
                mySurface = new Geom_BezierSurface(Poles, Weights);
            }
            else
            {
                mySurface = new Geom_BezierSurface(Poles);
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
                throw new Standard_OutOfRange("GeomFill_BezierCurves: Courbes non jointives");
            }

            NCollection_Array1<gp_Pnt> P1 = PolesOf(CC1);
            NCollection_Array1<gp_Pnt> P2 = PolesOf(CC2);

            GeomFill_Filling Caro;
            if (isRat)
            {
                NCollection_Array1<double> W1 = WeightsOf(CC1);
                NCollection_Array1<double> W2 = WeightsOf(CC2);
                Caro = new GeomFill_Curved(P1, P2, W1, W2);
            }
            else
            {
                Caro = new GeomFill_Curved(P1, P2);
            }

            int NbUPoles = Caro.NbUPoles();
            int NbVPoles = Caro.NbVPoles();
            NCollection_Array2<gp_Pnt> Poles = new(1, NbUPoles, 1, NbVPoles);
            Caro.Poles(Poles);

            if (Caro.isRational())
            {
                NCollection_Array2<double> Weights = new(1, NbUPoles, 1, NbVPoles);
                Caro.Weights(Weights);
                mySurface = new Geom_BezierSurface(Poles, Weights);
            }
            else
            {
                mySurface = new Geom_BezierSurface(Poles);
            }
        }
    }

    /// <summary>The surface (null before <c>Init</c>), frozen.</summary>
    public Geom_BezierSurface? Surface()
    {
        mySurface?.Freeze();
        return mySurface;
    }

    /// <summary>The surface, not frozen, for ported callers.</summary>
    internal Geom_BezierSurface? InternalSurface() => mySurface;
}
