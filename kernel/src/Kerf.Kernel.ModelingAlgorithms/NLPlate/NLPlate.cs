// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_NLPlate.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_NLPlate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A non-linear plate: an initial surface deformed by a stack of linear plates, each solved against the residuals of
/// the constraints (positions, G1 to G3 targets) on the surface deformed by the plates before it.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: <c>Load</c>, the solvers and <c>Init</c> change it. It keeps the
/// caller's initial surface and constraints (OCCT's handles), reads the surface, and changes a constraint only to
/// move the UV point of a sliding one (<see cref="ConstraintsSliding"/>). OCCT's stack of plates (a list of
/// <see cref="PlateSolver"/> values, the newest first) holds plates the solver makes, which nothing else sees.
/// </remarks>
internal class NLPlate
{
    private readonly Surface _initialSurface;
    private readonly NCollection_Sequence<HgppConstraint> _hgppConstraints = new();
    private readonly NCollection_List<PlateSolver> _sop = new();
    private bool _ok;

    public NLPlate(Surface initialSurface)
    {
        _initialSurface = initialSurface;
        _ok = false;
    }

    public void Load(HgppConstraint? gConst)
    {
        if (gConst is not null)
        {
            _hgppConstraints.Append(gConst);
        }
        _ok = false;
    }

    public void Solve(int ord = 2, int initialConsraintOrder = 1)
    {
        int maxOrder = MaxActiveConstraintOrder();
        int ordre = ord;
        if (ordre < maxOrder + 2)
        {
            ordre = maxOrder + 2;
        }

        for (int iterOrder = initialConsraintOrder; iterOrder <= maxOrder; iterOrder++)
        {
            if (!Iterate(iterOrder, ordre + iterOrder - maxOrder))
            {
                _ok = false;
                break;
            }
        }
        _ok = true; // OCCT: overwrites the failure above (occt-issues.md)
    }

    public void Solve2(int ord = 2, int initialConsraintOrder = 1)
    {
        int maxOrder = MaxActiveConstraintOrder();
        int ordre = ord;
        if (ordre < maxOrder + 2)
        {
            ordre = maxOrder + 2;
        }
        if (Iterate(0, ord)) // OCCT: the resolution order is ord, not the raised ordre
        {
            _sop.ChangeFirst().SetPolynomialPartOnly(true);
            ConstraintsSliding();
        }

        for (int iterOrder = initialConsraintOrder; iterOrder <= maxOrder; iterOrder++)
        {
            if (!Iterate(iterOrder, ordre + iterOrder - maxOrder))
            {
                _ok = false;
                break;
            }
        }
        _ok = true; // OCCT: overwrites the failure above (occt-issues.md)
    }

    public void IncrementalSolve(int ord = 2, int initialConsraintOrder = 1, int nbIncrements = 4, bool uvSliding = false)
    {
        _ = initialConsraintOrder; // unused, as in OCCT
        int maxOrder = MaxActiveConstraintOrder();
        int ordre = ord;
        if (ordre < maxOrder + 2)
        {
            ordre = maxOrder + 2;
        }
        double incrementalLoad = 1.0;

        for (int increment = 0; increment < nbIncrements; increment++)
        {
            incrementalLoad = 1.0 / (double)(nbIncrements - increment);

            int iterOrder = maxOrder;
            {
                if (!Iterate(iterOrder, ordre + iterOrder - maxOrder, incrementalLoad))
                {
                    _ok = false;
                    return;
                }
            }
            if (uvSliding)
            {
                ConstraintsSliding();
            }
        }
        _ok = true;
    }

    /// <summary>True if all has been correctly done.</summary>
    public bool IsDone => _ok;

    public void Destroy()
    {
        Init();
    }

    /// <summary>Resets the plate to its initial state (the same as after construction, with the same surface).</summary>
    public void Init()
    {
        _sop.Clear();
        _hgppConstraints.Clear();
    }

    public Xyz Evaluate(in Xy point2d) => EvaluateDerivative(point2d, 0, 0);

    public Xyz EvaluateDerivative(in Xy point2d, int iu, int iv)
    {
        Xyz value = new(0.0, 0.0, 0.0);
        if ((iu == 0) && (iv == 0))
        {
            value = _initialSurface.Value(point2d.X(), point2d.Y()).XYZ();
        }
        else
        {
            value = _initialSurface.DN(point2d.X(), point2d.Y(), iu, iv).XYZ();
        }

        for (NCollection_List<PlateSolver>.Iterator si = new(_sop); si.More(); si.Next())
        {
            if (si.Value().IsDone)
            {
                value += si.Value().EvaluateDerivative(point2d, iu, iv);
            }
        }
        return value;
    }

    public int Continuity()
    {
        int cont;
        for (cont = -1; cont < 10; cont++)
        {
            if (!(_initialSurface.IsCNu(cont + 1) && _initialSurface.IsCNv(cont + 1)))
            {
                break;
            }
        }
        for (NCollection_List<PlateSolver>.Iterator si = new(_sop); si.More(); si.Next())
        {
            if ((si.Value().IsDone) && (cont > si.Value().Continuity()))
            {
                cont = si.Value().Continuity();
            }
        }
        return cont;
    }

    private bool Iterate(int constraintOrder, int resolutionOrder, double incrementalLoading = 1.0)
    {
        // OCCT prepends a copy of an empty plate and works on the stored one: the same as storing a new plate
        PlateSolver emptyPlate = new();
        _sop.Prepend(emptyPlate);
        PlateSolver topP = _sop.First();
        for (int index = 1; index <= _hgppConstraints.Length(); index++)
        {
            HgppConstraint hgpp = _hgppConstraints.Value(index);
            int order = hgpp.ActiveOrder();
            if (constraintOrder < order)
            {
                order = constraintOrder;
            }
            Xy uv = hgpp.UV();

            if ((order >= 0) && hgpp.IsG0)
            {
                if (hgpp.IncrementalLoadAllowed())
                {
                    topP.Load(new PinpointConstraint(uv, (hgpp.G0Target() - Evaluate(uv)) * incrementalLoading));
                }
                else
                {
                    topP.Load(new PinpointConstraint(uv, hgpp.G0Target() - Evaluate(uv)));
                }
            }

            if ((incrementalLoading != 1.0) && hgpp.IncrementalLoadAllowed() && (order >= 1))
            {
                switch (order)
                {
                    case 1:
                    {
                        D1 d1S = new(EvaluateDerivative(uv, 1, 0), EvaluateDerivative(uv, 0, 1));
                        topP.Load(new FreeGtoCConstraint(uv, d1S, hgpp.G1Target(), incrementalLoading, hgpp.Orientation()));
                    }
                    break;
                    case 2:
                    {
                        D1 d1S = new(EvaluateDerivative(uv, 1, 0), EvaluateDerivative(uv, 0, 1));
                        D2 d2S = new(EvaluateDerivative(uv, 2, 0), EvaluateDerivative(uv, 1, 1), EvaluateDerivative(uv, 0, 2));
                        topP.Load(new FreeGtoCConstraint(uv, d1S, hgpp.G1Target(), d2S, hgpp.G2Target(), incrementalLoading,
                                                               hgpp.Orientation()));
                    }
                    break;
                    case 3:
                    {
                        D1 d1S = new(EvaluateDerivative(uv, 1, 0), EvaluateDerivative(uv, 0, 1));
                        D2 d2S = new(EvaluateDerivative(uv, 2, 0), EvaluateDerivative(uv, 1, 1), EvaluateDerivative(uv, 0, 2));
                        D3 d3S = new(EvaluateDerivative(uv, 3, 0), EvaluateDerivative(uv, 2, 1), EvaluateDerivative(uv, 1, 2),
                                           EvaluateDerivative(uv, 0, 3));
                        topP.Load(new FreeGtoCConstraint(uv, d1S, hgpp.G1Target(), d2S, hgpp.G2Target(), d3S, hgpp.G3Target(),
                                                               incrementalLoading, hgpp.Orientation()));
                    }
                    break;
                    default:
                        break;
                }
            }
            else
            {
                switch (order)
                {
                    case 1:
                    {
                        D1 d1S = new(EvaluateDerivative(uv, 1, 0), EvaluateDerivative(uv, 0, 1));
                        topP.Load(new FreeGtoCConstraint(uv, d1S, hgpp.G1Target()));
                    }
                    break;
                    case 2:
                    {
                        D1 d1S = new(EvaluateDerivative(uv, 1, 0), EvaluateDerivative(uv, 0, 1));
                        D2 d2S = new(EvaluateDerivative(uv, 2, 0), EvaluateDerivative(uv, 1, 1), EvaluateDerivative(uv, 0, 2));
                        topP.Load(new FreeGtoCConstraint(uv, d1S, hgpp.G1Target(), d2S, hgpp.G2Target()));
                    }
                    break;
                    case 3:
                    {
                        D1 d1S = new(EvaluateDerivative(uv, 1, 0), EvaluateDerivative(uv, 0, 1));
                        D2 d2S = new(EvaluateDerivative(uv, 2, 0), EvaluateDerivative(uv, 1, 1), EvaluateDerivative(uv, 0, 2));
                        D3 d3S = new(EvaluateDerivative(uv, 3, 0), EvaluateDerivative(uv, 2, 1), EvaluateDerivative(uv, 1, 2),
                                           EvaluateDerivative(uv, 0, 3));
                        topP.Load(new FreeGtoCConstraint(uv, d1S, hgpp.G1Target(), d2S, hgpp.G2Target(), d3S, hgpp.G3Target()));
                    }
                    break;
                    default:
                        break;
                }
            }
        }

        topP.SolveTI(resolutionOrder);
        if (!topP.IsDone)
        {
            _sop.RemoveFirst();
            return false;
        }
        else
        {
            return true;
        }
    }

    public void ConstraintsSliding(int nbIterations = 3)
    {
        for (int index = 1; index <= _hgppConstraints.Length(); index++)
        {
            HgppConstraint hgpp = _hgppConstraints.Value(index);
            if (hgpp.UVFreeSliding() && hgpp.IsG0)
            {
                Xy uv = hgpp.UV();
                Xyz p0 = Evaluate(uv);
                Xyz p1 = hgpp.G0Target();
                for (int iter = 1; iter <= nbIterations; iter++)
                {
                    // iterates at the first order, which is enough near the surface (OCCT's note)
                    Xyz dp = p1 - p0;
                    Xyz du = EvaluateDerivative(uv, 1, 0);
                    Xyz dv = EvaluateDerivative(uv, 0, 1);
                    Matrix mat = new(0, 1, 0, 1);
                    mat[0, 0] = du * du;
                    mat[0, 1] = du * dv;
                    mat[1, 0] = du * dv;
                    mat[1, 1] = dv * dv;
                    Gauss gauss = new(mat);
                    if (!gauss.IsDone)
                    {
                        break;
                    }

                    math_Vector vec = new(0, 1);
                    vec[0] = du * dp;
                    vec[1] = dv * dp;
                    math_Vector sol = new(0, 1);
                    gauss.Solve(vec, sol);
                    uv.SetX(uv.X() + sol[0]);
                    uv.SetY(uv.Y() + sol[1]);
                    p0 = Evaluate(uv);
                }
                hgpp.SetUV(uv);
            }
        }
    }

    public int MaxActiveConstraintOrder()
    {
        int maxOrder = -1;
        for (int index = 1; index <= _hgppConstraints.Length(); index++)
        {
            int caOrder = _hgppConstraints.Value(index).ActiveOrder();
            if (caOrder > maxOrder)
            {
                maxOrder = caOrder;
            }
        }
        return maxOrder;
    }
}
