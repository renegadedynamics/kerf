// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_NLPlate.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_NLPlate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A non-linear plate: an initial surface deformed by a stack of linear plates, each solved against the residuals of
/// the constraints (positions, G1 to G3 targets) on the surface deformed by the plates before it.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: <c>Load</c>, the solvers and <c>Init</c> change it. It keeps the
/// caller's initial surface and constraints (OCCT's handles), reads the surface, and changes a constraint only to
/// move the UV point of a sliding one (<see cref="ConstraintsSliding"/>). OCCT's stack of plates (a list of
/// <see cref="Plate_Plate"/> values, the newest first) holds plates the solver makes, which nothing else sees.
/// </remarks>
public class NLPlate_NLPlate
{
    private readonly Geom_Surface myInitialSurface;
    private readonly NCollection_Sequence<NLPlate_HGPPConstraint> myHGPPConstraints = new();
    private readonly NCollection_List<Plate_Plate> mySOP = new();
    private bool OK;

    public NLPlate_NLPlate(Geom_Surface InitialSurface)
    {
        myInitialSurface = InitialSurface;
        OK = false;
    }

    public void Load(NLPlate_HGPPConstraint? GConst)
    {
        if (GConst is not null)
        {
            myHGPPConstraints.Append(GConst);
        }
        OK = false;
    }

    public void Solve(int ord = 2, int InitialConsraintOrder = 1)
    {
        int maxOrder = MaxActiveConstraintOrder();
        int ordre = ord;
        if (ordre < maxOrder + 2)
        {
            ordre = maxOrder + 2;
        }

        for (int iterOrder = InitialConsraintOrder; iterOrder <= maxOrder; iterOrder++)
        {
            if (!Iterate(iterOrder, ordre + iterOrder - maxOrder))
            {
                OK = false;
                break;
            }
        }
        OK = true; // OCCT: overwrites the failure above (occt-issues.md)
    }

    public void Solve2(int ord = 2, int InitialConsraintOrder = 1)
    {
        int maxOrder = MaxActiveConstraintOrder();
        int ordre = ord;
        if (ordre < maxOrder + 2)
        {
            ordre = maxOrder + 2;
        }
        if (Iterate(0, ord)) // OCCT: the resolution order is ord, not the raised ordre
        {
            mySOP.ChangeFirst().SetPolynomialPartOnly(true);
            ConstraintsSliding();
        }

        for (int iterOrder = InitialConsraintOrder; iterOrder <= maxOrder; iterOrder++)
        {
            if (!Iterate(iterOrder, ordre + iterOrder - maxOrder))
            {
                OK = false;
                break;
            }
        }
        OK = true; // OCCT: overwrites the failure above (occt-issues.md)
    }

    public void IncrementalSolve(int ord = 2, int InitialConsraintOrder = 1, int NbIncrements = 4, bool UVSliding = false)
    {
        _ = InitialConsraintOrder; // unused, as in OCCT
        int maxOrder = MaxActiveConstraintOrder();
        int ordre = ord;
        if (ordre < maxOrder + 2)
        {
            ordre = maxOrder + 2;
        }
        double IncrementalLoad = 1.0;

        for (int increment = 0; increment < NbIncrements; increment++)
        {
            IncrementalLoad = 1.0 / (double)(NbIncrements - increment);

            int iterOrder = maxOrder;
            {
                if (!Iterate(iterOrder, ordre + iterOrder - maxOrder, IncrementalLoad))
                {
                    OK = false;
                    return;
                }
            }
            if (UVSliding)
            {
                ConstraintsSliding();
            }
        }
        OK = true;
    }

    /// <summary>True if all has been correctly done.</summary>
    public bool IsDone() => OK;

    public void destroy()
    {
        Init();
    }

    /// <summary>Resets the plate to its initial state (the same as after construction, with the same surface).</summary>
    public void Init()
    {
        mySOP.Clear();
        myHGPPConstraints.Clear();
    }

    public gp_XYZ Evaluate(in gp_XY point2d) => EvaluateDerivative(point2d, 0, 0);

    public gp_XYZ EvaluateDerivative(in gp_XY point2d, int iu, int iv)
    {
        gp_XYZ Value = new(0.0, 0.0, 0.0);
        if ((iu == 0) && (iv == 0))
        {
            Value = myInitialSurface.Value(point2d.X(), point2d.Y()).XYZ();
        }
        else
        {
            Value = myInitialSurface.DN(point2d.X(), point2d.Y(), iu, iv).XYZ();
        }

        for (NCollection_List<Plate_Plate>.Iterator SI = new(mySOP); SI.More(); SI.Next())
        {
            if (SI.Value().IsDone())
            {
                Value += SI.Value().EvaluateDerivative(point2d, iu, iv);
            }
        }
        return Value;
    }

    public int Continuity()
    {
        int cont;
        for (cont = -1; cont < 10; cont++)
        {
            if (!(myInitialSurface.IsCNu(cont + 1) && myInitialSurface.IsCNv(cont + 1)))
            {
                break;
            }
        }
        for (NCollection_List<Plate_Plate>.Iterator SI = new(mySOP); SI.More(); SI.Next())
        {
            if ((SI.Value().IsDone()) && (cont > SI.Value().Continuity()))
            {
                cont = SI.Value().Continuity();
            }
        }
        return cont;
    }

    private bool Iterate(int ConstraintOrder, int ResolutionOrder, double IncrementalLoading = 1.0)
    {
        // OCCT prepends a copy of an empty plate and works on the stored one: the same as storing a new plate
        Plate_Plate EmptyPlate = new();
        mySOP.Prepend(EmptyPlate);
        Plate_Plate TopP = mySOP.First();
        for (int index = 1; index <= myHGPPConstraints.Length(); index++)
        {
            NLPlate_HGPPConstraint HGPP = myHGPPConstraints.Value(index);
            int Order = HGPP.ActiveOrder();
            if (ConstraintOrder < Order)
            {
                Order = ConstraintOrder;
            }
            gp_XY UV = HGPP.UV();

            if ((Order >= 0) && HGPP.IsG0())
            {
                if (HGPP.IncrementalLoadAllowed())
                {
                    TopP.Load(new Plate_PinpointConstraint(UV, (HGPP.G0Target() - Evaluate(UV)) * IncrementalLoading));
                }
                else
                {
                    TopP.Load(new Plate_PinpointConstraint(UV, HGPP.G0Target() - Evaluate(UV)));
                }
            }

            if ((IncrementalLoading != 1.0) && HGPP.IncrementalLoadAllowed() && (Order >= 1))
            {
                switch (Order)
                {
                    case 1:
                    {
                        Plate_D1 D1S = new(EvaluateDerivative(UV, 1, 0), EvaluateDerivative(UV, 0, 1));
                        TopP.Load(new Plate_FreeGtoCConstraint(UV, D1S, HGPP.G1Target(), IncrementalLoading, HGPP.Orientation()));
                    }
                    break;
                    case 2:
                    {
                        Plate_D1 D1S = new(EvaluateDerivative(UV, 1, 0), EvaluateDerivative(UV, 0, 1));
                        Plate_D2 D2S = new(EvaluateDerivative(UV, 2, 0), EvaluateDerivative(UV, 1, 1), EvaluateDerivative(UV, 0, 2));
                        TopP.Load(new Plate_FreeGtoCConstraint(UV, D1S, HGPP.G1Target(), D2S, HGPP.G2Target(), IncrementalLoading,
                                                               HGPP.Orientation()));
                    }
                    break;
                    case 3:
                    {
                        Plate_D1 D1S = new(EvaluateDerivative(UV, 1, 0), EvaluateDerivative(UV, 0, 1));
                        Plate_D2 D2S = new(EvaluateDerivative(UV, 2, 0), EvaluateDerivative(UV, 1, 1), EvaluateDerivative(UV, 0, 2));
                        Plate_D3 D3S = new(EvaluateDerivative(UV, 3, 0), EvaluateDerivative(UV, 2, 1), EvaluateDerivative(UV, 1, 2),
                                           EvaluateDerivative(UV, 0, 3));
                        TopP.Load(new Plate_FreeGtoCConstraint(UV, D1S, HGPP.G1Target(), D2S, HGPP.G2Target(), D3S, HGPP.G3Target(),
                                                               IncrementalLoading, HGPP.Orientation()));
                    }
                    break;
                    default:
                        break;
                }
            }
            else
            {
                switch (Order)
                {
                    case 1:
                    {
                        Plate_D1 D1S = new(EvaluateDerivative(UV, 1, 0), EvaluateDerivative(UV, 0, 1));
                        TopP.Load(new Plate_FreeGtoCConstraint(UV, D1S, HGPP.G1Target()));
                    }
                    break;
                    case 2:
                    {
                        Plate_D1 D1S = new(EvaluateDerivative(UV, 1, 0), EvaluateDerivative(UV, 0, 1));
                        Plate_D2 D2S = new(EvaluateDerivative(UV, 2, 0), EvaluateDerivative(UV, 1, 1), EvaluateDerivative(UV, 0, 2));
                        TopP.Load(new Plate_FreeGtoCConstraint(UV, D1S, HGPP.G1Target(), D2S, HGPP.G2Target()));
                    }
                    break;
                    case 3:
                    {
                        Plate_D1 D1S = new(EvaluateDerivative(UV, 1, 0), EvaluateDerivative(UV, 0, 1));
                        Plate_D2 D2S = new(EvaluateDerivative(UV, 2, 0), EvaluateDerivative(UV, 1, 1), EvaluateDerivative(UV, 0, 2));
                        Plate_D3 D3S = new(EvaluateDerivative(UV, 3, 0), EvaluateDerivative(UV, 2, 1), EvaluateDerivative(UV, 1, 2),
                                           EvaluateDerivative(UV, 0, 3));
                        TopP.Load(new Plate_FreeGtoCConstraint(UV, D1S, HGPP.G1Target(), D2S, HGPP.G2Target(), D3S, HGPP.G3Target()));
                    }
                    break;
                    default:
                        break;
                }
            }
        }

        TopP.SolveTI(ResolutionOrder);
        if (!TopP.IsDone())
        {
            mySOP.RemoveFirst();
            return false;
        }
        else
        {
            return true;
        }
    }

    public void ConstraintsSliding(int NbIterations = 3)
    {
        for (int index = 1; index <= myHGPPConstraints.Length(); index++)
        {
            NLPlate_HGPPConstraint HGPP = myHGPPConstraints.Value(index);
            if (HGPP.UVFreeSliding() && HGPP.IsG0())
            {
                gp_XY UV = HGPP.UV();
                gp_XYZ P0 = Evaluate(UV);
                gp_XYZ P1 = HGPP.G0Target();
                for (int iter = 1; iter <= NbIterations; iter++)
                {
                    // iterates at the first order, which is enough near the surface (OCCT's note)
                    gp_XYZ DP = P1 - P0;
                    gp_XYZ Du = EvaluateDerivative(UV, 1, 0);
                    gp_XYZ Dv = EvaluateDerivative(UV, 0, 1);
                    math_Matrix mat = new(0, 1, 0, 1);
                    mat[0, 0] = Du * Du;
                    mat[0, 1] = Du * Dv;
                    mat[1, 0] = Du * Dv;
                    mat[1, 1] = Dv * Dv;
                    math_Gauss gauss = new(mat);
                    if (!gauss.IsDone())
                    {
                        break;
                    }

                    math_Vector vec = new(0, 1);
                    vec[0] = Du * DP;
                    vec[1] = Dv * DP;
                    math_Vector sol = new(0, 1);
                    gauss.Solve(vec, sol);
                    UV.SetX(UV.X() + sol[0]);
                    UV.SetY(UV.Y() + sol[1]);
                    P0 = Evaluate(UV);
                }
                HGPP.SetUV(UV);
            }
        }
    }

    public int MaxActiveConstraintOrder()
    {
        int MaxOrder = -1;
        for (int index = 1; index <= myHGPPConstraints.Length(); index++)
        {
            int CAOrder = myHGPPConstraints.Value(index).ActiveOrder();
            if (CAOrder > MaxOrder)
            {
                MaxOrder = CAOrder;
            }
        }
        return MaxOrder;
    }
}
