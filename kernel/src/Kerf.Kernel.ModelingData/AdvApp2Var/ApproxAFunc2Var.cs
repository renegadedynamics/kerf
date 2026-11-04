// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_ApproxAFunc2Var.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_ApproxAFunc2Var.lxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_ApproxAFunc2Var.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Occt;
using System.Globalization;
using static Kerf.Engine.Approximation.Advanced.CriterionRepartition;
using static Kerf.Engine.Approximation.Advanced.CriterionType;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// Approximates a function F(U, V), made of 1D, 2D and 3D subspaces, by B-spline surfaces: polynomial patches on a
/// grid cut as needed, joined with the requested continuities, within the tolerances of each subspace (inside and
/// on the boundaries).
/// </summary>
/// <remarks>
/// <para>
/// The arguments: <c>Num1DSS</c>, <c>Num2DSS</c>, <c>Num3DSS</c>, the numbers of 1, 2 and 3-dimensional subspaces;
/// <c>OneDTol</c>, <c>TwoDTol</c>, <c>ThreeDTol</c>, their tolerances (one per subspace, 1-based; null for a
/// dimension without subspaces); <c>OneDTolFr</c>, ..., their tolerances on the four boundaries (subspace x 4);
/// [<c>FirstInU</c>, <c>LastInU</c>] x [<c>FirstInV</c>, <c>LastInV</c>], the domain; <c>FavorIso</c>, the iso-curves
/// to extract from F (which can help the evaluator); <c>ContInU</c>, <c>ContInV</c>, the continuities (C0, C1 or C2);
/// <c>PrecisCode</c>, the precision of the error measurement (1 fast, 2 average, 3 slow and precise);
/// <c>MaxDegInU</c>, <c>MaxDegInV</c>, the maximum degrees (at least 2 * order + 1, the order being 0, 1 or 2 for
/// C0, C1 or C2); <c>MaxPatch</c>, the maximum number of patches; <c>Func</c>, the function; <c>Crit</c>, an
/// optional criterion of convergence; <c>UChoice</c>, <c>VChoice</c>, where to cut.
/// </para>
/// <para>
/// For the moment the result is a 3D surface: <c>Num1DSS</c> and <c>Num2DSS</c> must be 0 and <c>Num3DSS</c> 1. The
/// evaluator writes, for each parameter, the 1D values, then the 2D ones, then the 3D ones.
/// </para>
/// <para>
/// The approximation is done in the constructor. <see cref="Surface"/> returns the stored surface, frozen
/// (decision 197); the error accessors return copies, and null where OCCT returns a null handle (decision 146).
/// The approximation is thread-safe: each instance has its own state, and the f2c layer none (decisions 113 and
/// 1901).
/// </para>
/// </remarks>
internal class ApproxAFunc2Var
{
    private readonly int[] _numSubSpaces = new int[3];
    private readonly NCollection_HArray1<double>? _my1DTolerances;
    private readonly NCollection_HArray1<double>? _my2DTolerances;
    private readonly NCollection_HArray1<double>? _my3DTolerances;
    private readonly NCollection_HArray2<double>? _my1DTolOnFront;
    private readonly NCollection_HArray2<double>? _my2DTolOnFront;
    private readonly NCollection_HArray2<double>? _my3DTolOnFront;
    private readonly double _firstParInU;
    private readonly double _lastParInU;
    private readonly double _firstParInV;
    private readonly double _lastParInV;
    private readonly IsoType _favoriteIso;
    private readonly Continuity _contInU;
    private readonly Continuity _contInV;
    private int _precisionCode;
    private readonly int _maxDegInU;
    private readonly int _maxDegInV;
    private readonly int _maxPatches;
    private Context _conditions = new();
    private Network _result = new();
    private Framework _constraints = new();
    private bool _done;
    private bool _hasResult;
    private NCollection_HArray1<Surface?>? _surfaces;
    private int _degreeInU;
    private int _degreeInV;
    private NCollection_HArray1<double>? _my1DMaxError = null;
    private NCollection_HArray1<double>? _my1DAverageError = null;
    private NCollection_HArray1<double>? _my1DUFrontError = null;
    private NCollection_HArray1<double>? _my1DVFrontError = null;
    private NCollection_HArray1<double>? _my2DMaxError = null;
    private NCollection_HArray1<double>? _my2DAverageError = null;
    private NCollection_HArray1<double>? _my2DUFrontError = null;
    private NCollection_HArray1<double>? _my2DVFrontError = null;
    private NCollection_HArray1<double>? _my3DMaxError;
    private NCollection_HArray1<double>? _my3DAverageError;
    private NCollection_HArray1<double>? _my3DUFrontError;
    private NCollection_HArray1<double>? _my3DVFrontError;
    private double _criterionError;

    /// <summary>Approximates <paramref name="func"/> (see the class remarks for the arguments).</summary>
    /// <exception cref="ConstructionErrorException">
    /// A continuity other than C0, C1 or C2, or an approximation that fails (discretisation, curve or surface
    /// approximation error).
    /// </exception>
    public ApproxAFunc2Var(int num1DSS,
                                      int num2DSS,
                                      int num3DSS,
                                      NCollection_HArray1<double>? oneDTol,
                                      NCollection_HArray1<double>? twoDTol,
                                      NCollection_HArray1<double>? threeDTol,
                                      NCollection_HArray2<double>? oneDTolFr,
                                      NCollection_HArray2<double>? twoDTolFr,
                                      NCollection_HArray2<double>? threeDTolFr,
                                      double firstInU,
                                      double lastInU,
                                      double firstInV,
                                      double lastInV,
                                      IsoType favorIso,
                                      Continuity contInU,
                                      Continuity contInV,
                                      int precisCode,
                                      int maxDegInU,
                                      int maxDegInV,
                                      int maxPatch,
                                      EvaluatorFunc2Var func,
                                      Cutting uChoice,
                                      Cutting vChoice)
    {
        _my1DTolerances = oneDTol;
        _my2DTolerances = twoDTol;
        _my3DTolerances = threeDTol;
        _my1DTolOnFront = oneDTolFr;
        _my2DTolOnFront = twoDTolFr;
        _my3DTolOnFront = threeDTolFr;
        _firstParInU = firstInU;
        _lastParInU = lastInU;
        _firstParInV = firstInV;
        _lastParInV = lastInV;
        _favoriteIso = favorIso;
        _contInU = contInU;
        _contInV = contInV;
        _precisionCode = precisCode;
        _maxDegInU = maxDegInU;
        _maxDegInV = maxDegInV;
        _maxPatches = maxPatch;
        _done = false;
        _hasResult = false;
        _degreeInU = 0;
        _degreeInV = 0;
        _criterionError = 0.0;
        _numSubSpaces[0] = num1DSS;
        _numSubSpaces[1] = num2DSS;
        _numSubSpaces[2] = num3DSS;
        Init();
        Perform(uChoice, vChoice, func);
        ConvertBS();
    }

    /// <summary>Approximates <paramref name="func"/> under the criterion <paramref name="crit"/>.</summary>
    /// <exception cref="ConstructionErrorException">
    /// A continuity other than C0, C1 or C2, or an approximation that fails.
    /// </exception>
    public ApproxAFunc2Var(int num1DSS,
                                      int num2DSS,
                                      int num3DSS,
                                      NCollection_HArray1<double>? oneDTol,
                                      NCollection_HArray1<double>? twoDTol,
                                      NCollection_HArray1<double>? threeDTol,
                                      NCollection_HArray2<double>? oneDTolFr,
                                      NCollection_HArray2<double>? twoDTolFr,
                                      NCollection_HArray2<double>? threeDTolFr,
                                      double firstInU,
                                      double lastInU,
                                      double firstInV,
                                      double lastInV,
                                      IsoType favorIso,
                                      Continuity contInU,
                                      Continuity contInV,
                                      int precisCode,
                                      int maxDegInU,
                                      int maxDegInV,
                                      int maxPatch,
                                      EvaluatorFunc2Var func,
                                      Criterion crit,
                                      Cutting uChoice,
                                      Cutting vChoice)
    {
        _my1DTolerances = oneDTol;
        _my2DTolerances = twoDTol;
        _my3DTolerances = threeDTol;
        _my1DTolOnFront = oneDTolFr;
        _my2DTolOnFront = twoDTolFr;
        _my3DTolOnFront = threeDTolFr;
        _firstParInU = firstInU;
        _lastParInU = lastInU;
        _firstParInV = firstInV;
        _lastParInV = lastInV;
        _favoriteIso = favorIso;
        _contInU = contInU;
        _contInV = contInV;
        _precisionCode = precisCode;
        _maxDegInU = maxDegInU;
        _maxDegInV = maxDegInV;
        _maxPatches = maxPatch;
        _done = false;
        _hasResult = false;
        _degreeInU = 0;
        _degreeInV = 0;
        _criterionError = 0.0;
        _numSubSpaces[0] = num1DSS;
        _numSubSpaces[1] = num2DSS;
        _numSubSpaces[2] = num3DSS;
        Init();
        Perform(uChoice, vChoice, func, crit);
        ConvertBS();
    }

    // Initialisation of the approximation ; used by Create
    private void Init()
    {
        int ifav, iu = 0, iv = 0, ndu, ndv;
        switch (_favoriteIso)
        {
            case IsoU:
                ifav = 1;
                break;
            case IsoV:
                ifav = 2;
                break;
            default:
                ifav = 2;
                break;
        }
        switch (_contInU)
        {
            case C0:
                iu = 0;
                break;
            case C1:
                iu = 1;
                break;
            case C2:
                iu = 2;
                break;
            default:
                throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : UContinuity Error");
        }
        switch (_contInV)
        {
            case C0:
                iv = 0;
                break;
            case C1:
                iv = 1;
                break;
            case C2:
                iv = 2;
                break;
            default:
                throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : VContinuity Error");
        }
        ndu = Max(_maxDegInU + 1, 2 * iu + 2);
        ndv = Max(_maxDegInV + 1, 2 * iv + 2);
        if (ndu < 2 * iu + 2)
        {
            throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : UMaxDegree Error");
        }
        if (ndv < 2 * iv + 2)
        {
            throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : VMaxDegree Error");
        }
        _precisionCode = Max(0, Min(_precisionCode, 3));
        Context conditions = new(ifav,
                                            iu,
                                            iv,
                                            ndu,
                                            ndv,
                                            _precisionCode,
                                            _numSubSpaces[0],
                                            _numSubSpaces[1],
                                            _numSubSpaces[2],
                                            _my1DTolerances,
                                            _my2DTolerances,
                                            _my3DTolerances,
                                            _my1DTolOnFront,
                                            _my2DTolOnFront,
                                            _my3DTolOnFront);
        _conditions = conditions;
        InitGrid(1);
    }

    // Initialisation of the approximation with a grid of regular cuttings ; used by Init and Perform
    private void InitGrid(int nbInt)
    {
        int iu = _conditions.UOrder(), iv = _conditions.VOrder(), iint;

        Patch m0 = new(_firstParInU, _lastParInU, _firstParInV, _lastParInV, iu, iv);

        NCollection_Sequence<Patch> net = new();
        net.Append(m0);

        NCollection_Sequence<double> theU = new(), theV = new();
        theU.Append(_firstParInU);
        theV.Append(_firstParInV);
        theU.Append(_lastParInU);
        theV.Append(_lastParInV);

        Network result = new(net, theU, theV);

        Xy uv1 = new(_firstParInU, _firstParInV);
        Node c1 = new(uv1, iu, iv);
        Xy uv2 = new(_lastParInU, _firstParInV);
        Node c2 = new(uv2, iu, iv);
        Xy uv4 = new(_lastParInU, _lastParInV);
        Node c4 = new(uv4, iu, iv);
        Xy uv3 = new(_firstParInU, _lastParInV);
        Node c3 = new(uv3, iu, iv);
        NCollection_Sequence<Node> bag = new();
        bag.Append(c1);
        bag.Append(c2);
        bag.Append(c3);
        bag.Append(c4);

        Iso v0 = new(IsoV,
                                _firstParInV,
                                _firstParInU,
                                _lastParInU,
                                _firstParInV,
                                _lastParInV,
                                1,
                                iu,
                                iv);
        Iso v1 = new(IsoV,
                                _lastParInV,
                                _firstParInU,
                                _lastParInU,
                                _firstParInV,
                                _lastParInV,
                                2,
                                iu,
                                iv);
        Iso u0 = new(IsoU,
                                _firstParInU,
                                _firstParInU,
                                _lastParInU,
                                _firstParInV,
                                _lastParInV,
                                3,
                                iu,
                                iv);
        Iso u1 = new(IsoU,
                                _lastParInU,
                                _firstParInU,
                                _lastParInU,
                                _firstParInV,
                                _lastParInV,
                                4,
                                iu,
                                iv);

        NCollection_Sequence<Iso> bu0 = new(), bv0 = new();
        bu0.Append(v0);
        bu0.Append(v1);
        bv0.Append(u0);
        bv0.Append(u1);

        NCollection_Sequence<NCollection_Sequence<Iso>> uStrip = new(), vStrip = new();
        uStrip.Append(bu0);
        vStrip.Append(bv0);

        Framework constraints = new(bag, uStrip, vStrip);

        // regular cutting if NbInt>1
        double deltu = (_lastParInU - _firstParInU) / nbInt,
               deltv = (_lastParInV - _firstParInV) / nbInt;
        for (iint = 1; iint <= nbInt - 1; iint++)
        {
            result.UpdateInU(_firstParInU + iint * deltu);
            constraints.UpdateInU(_firstParInU + iint * deltu);
            result.UpdateInV(_firstParInV + iint * deltv);
            constraints.UpdateInV(_firstParInV + iint * deltv);
        }
        _result = result;
        _constraints = constraints;
    }

    // Computation of the approximation result ; used by Create
    private void Perform(Cutting uChoice, Cutting vChoice, EvaluatorFunc2Var func)
    {
        ComputePatches(uChoice, vChoice, func);
        _hasResult = _done = true;
        Compute3DErrors();
    }

    private void Perform(Cutting uChoice,
                         Cutting vChoice,
                         EvaluatorFunc2Var func,
                         Criterion crit)
    {
        ComputePatches(uChoice, vChoice, func, crit);
        _hasResult = _done = true;
        Compute3DErrors();
        ComputeCritError();
    }

    // Computation of the polynomial approximations ; used by Perform
    private void ComputePatches(Cutting uChoice,
                                Cutting vChoice,
                                EvaluatorFunc2Var func)
    {
        double udec, vdec;
        bool umore, vmore;
        int nbPatch, nbU, nbV, numDec;
        int firstNA = 0;

        while (_result.FirstNotApprox(ref firstNA))
        {
            // complete the set of constraints
            ComputeConstraints(uChoice, vChoice, func);

            // discretization of constraints relative to the square
            _result[firstNA].Discretise(_conditions, _constraints, func);
            if (!_result[firstNA].IsDiscretised)
            {
                _hasResult = _done = false;
                throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : Surface Discretisation Error");
            }

            // calculate the number and the type of authorized cuts
            // depending on the max number of squares and the validity of next cuts.
            nbU = _result.PatchInUCount;
            nbV = _result.PatchInVCount;
            nbPatch = nbU * nbV;
            umore = uChoice.Value(_result[firstNA].U0(), _result[firstNA].U1(), out udec);
            vmore = vChoice.Value(_result[firstNA].V0(), _result[firstNA].V1(), out vdec);

            numDec = 0;
            if (((nbPatch + nbV) <= _maxPatches) && ((nbPatch + nbU) > _maxPatches) && umore)
            {
                numDec = 1;
            }
            if (((nbPatch + nbV) > _maxPatches) && ((nbPatch + nbU) <= _maxPatches) && vmore)
            {
                numDec = 2;
            }
            if (((nbPatch + nbV) <= _maxPatches) && ((nbPatch + nbU) <= _maxPatches))
            {
                if (umore)
                {
                    numDec = 3;
                }
                if ((nbV > nbU) && vmore)
                {
                    numDec = 4;
                }
            }
            if ((nbU + 1) * (nbV + 1) <= _maxPatches)
            {
                if (!umore && !vmore)
                {
                    numDec = 0;
                }
                if (umore && !vmore)
                {
                    numDec = 3;
                }
                if (!umore && vmore)
                {
                    numDec = 4;
                }
                if (umore && vmore)
                {
                    numDec = 5;
                }
            }

            // approximation of the square
            _result[firstNA].MakeApprox(_conditions, _constraints, numDec);

            if (!_result[firstNA].IsApproximated)
            {
                switch (_result[firstNA].CutSense())
                {
                    case 0:
                        //	It is not possible to cut : the result is preserved
                        if (_result[firstNA].HasResult)
                        {
                            _result[firstNA].OverwriteApprox();
                        }
                        else
                        {
                            _hasResult = _done = false;
                            throw new ConstructionErrorException(
                                "AdvApp2Var_ApproxAFunc2Var : Surface Approximation Error");
                        }
                        break;
                    case 1:
                        //      It is necessary to cut in U
                        _result.UpdateInU(udec);
                        _constraints.UpdateInU(udec);
                        break;
                    case 2:
                        //      It is necessary to cut in V
                        _result.UpdateInV(vdec);
                        _constraints.UpdateInV(vdec);
                        break;
                    case 3:
                        //      It is necessary to cut in U and V
                        _result.UpdateInU(udec);
                        _constraints.UpdateInU(udec);
                        _result.UpdateInV(vdec);
                        _constraints.UpdateInV(vdec);
                        break;
                    default:
                        _hasResult = _done = false;
                        throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : Surface Approximation Error");
                }
            }
        }
    }

    private void ComputePatches(Cutting uChoice,
                                Cutting vChoice,
                                EvaluatorFunc2Var func,
                                Criterion crit)
    {
        double udec, vdec, critValue, m1 = 0.0;
        bool umore, vmore, critAbs = crit.Type() == Absolute;
        int nbPatch, nbU, nbV, nbInt, numDec;
        int firstNA = 0, decision = 0;

        while (_result.FirstNotApprox(ref firstNA))
        {
            // complete the set of constraints
            ComputeConstraints(uChoice, vChoice, func, crit);
            if (decision > 0)
            {
                m1 = 0.0;
            }

            // discretize the constraints relative to the square
            _result[firstNA].Discretise(_conditions, _constraints, func);
            if (!_result[firstNA].IsDiscretised)
            {
                _hasResult = _done = false;
                throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : Surface Discretisation Error");
            }

            // calculate the number and type of authorized cuts
            // depending on the max number of squares and the validity of next cuts
            nbU = _result.PatchInUCount;
            nbV = _result.PatchInVCount;
            nbPatch = nbU * nbV;
            nbInt = nbU;
            umore = uChoice.Value(_result[firstNA].U0(), _result[firstNA].U1(), out udec);
            vmore = vChoice.Value(_result[firstNA].V0(), _result[firstNA].V1(), out vdec);

            numDec = 0;
            if (((nbPatch + nbV) <= _maxPatches) && ((nbPatch + nbU) > _maxPatches) && umore)
            {
                numDec = 1;
            }
            if (((nbPatch + nbV) > _maxPatches) && ((nbPatch + nbU) <= _maxPatches) && vmore)
            {
                numDec = 2;
            }
            if (((nbPatch + nbV) <= _maxPatches) && ((nbPatch + nbU) <= _maxPatches))
            {
                if (umore)
                {
                    numDec = 3;
                }
                if ((nbV > nbU) && vmore)
                {
                    numDec = 4;
                }
            }
            if ((nbU + 1) * (nbV + 1) <= _maxPatches)
            {
                if (!umore && !vmore)
                {
                    numDec = 0;
                }
                if (umore && !vmore)
                {
                    numDec = 1;
                }
                if (!umore && vmore)
                {
                    numDec = 2;
                }
                if (umore && vmore)
                {
                    numDec = 5;
                }
            }

            // approximation of the square
            if (critAbs)
            {
                _result[firstNA].MakeApprox(_conditions, _constraints, 0);
            }
            else
            {
                _result[firstNA].MakeApprox(_conditions, _constraints, numDec);
            }
            if (numDec >= 3)
            {
                numDec = numDec - 2;
            }

            // evaluation of the criterion on the square
            if (_result[firstNA].HasResult)
            {
                crit.Value(_result[firstNA], _conditions);
                critValue = _result[firstNA].CritValue();
                if (m1 < critValue)
                {
                    m1 = critValue;
                }
            }
            // is it necessary to cut ?
            decision = _result[firstNA].CutSense(crit, numDec);
            bool regular = crit.Repartition() == global::Kerf.Engine.Approximation.Advanced.CriterionRepartition.Regular;
            //    bool Regular = true;
            if (regular && decision > 0)
            {
                nbInt++;
                InitGrid(nbInt);
            }
            else
            {
                switch (decision)
                {
                    case 0:
                        //	Impossible to cut : the result is preserved
                        if (_result[firstNA].HasResult)
                        {
                            _result[firstNA].OverwriteApprox();
                        }
                        else
                        {
                            _hasResult = _done = false;
                            throw new ConstructionErrorException(
                                "AdvApp2Var_ApproxAFunc2Var : Surface Approximation Error");
                        }
                        break;
                    case 1:
                        //      It is necessary to cut in U
                        _result.UpdateInU(udec);
                        _constraints.UpdateInU(udec);
                        break;
                    case 2:
                        //      It is necessary to cut in V
                        _result.UpdateInV(vdec);
                        _constraints.UpdateInV(vdec);
                        break;
                    case 3:
                        //      It is necessary to cut in U and V
                        _result.UpdateInU(udec);
                        _constraints.UpdateInU(udec);
                        _result.UpdateInV(vdec);
                        _constraints.UpdateInV(vdec);
                        break;
                    default:
                        _hasResult = _done = false;
                        throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : Surface Approximation Error");
                }
            }
        }
    }

    // Approximation of the constraints ; used by ComputePatches
    private void ComputeConstraints(Cutting uChoice,
                                    Cutting vChoice,
                                    EvaluatorFunc2Var func)
    {
        double dec;
        bool more;
        int ind1 = 0, ind2 = 0, nbPatch, nbU, nbV;
        int iu = _conditions.UOrder(), iv = _conditions.VOrder();
        Node n1 = new(iu, iv), n2 = new(iu, iv);

        for (Iso? iso = _constraints.FirstNotApprox(ref ind1, ref ind2);
             iso is not null;
             iso = _constraints.FirstNotApprox(ref ind1, ref ind2))
        {
            // approximation of iso and calculation of constraints at extremities
            int indN1 = _constraints.FirstNode(iso.Type(), ind1, ind2);
            n1.Assign(_constraints.Node(indN1));
            int indN2 = _constraints.LastNode(iso.Type(), ind1, ind2);
            n2.Assign(_constraints.Node(indN2));

            // note that old code attempted to make copy of anIso here (but copy was incomplete)
            iso.MakeApprox(_conditions,
                             _firstParInU,
                             _lastParInU,
                             _firstParInV,
                             _lastParInV,
                             func,
                             n1,
                             n2);

            if (iso.IsApproximated)
            {
                // iso is approached at the required tolerance
                _constraints.ChangeIso(ind1, ind2, iso);
                _constraints.Node(indN1).Assign(n1);
                _constraints.Node(indN2).Assign(n2);
            }
            else
            {
                // Approximation is not satisfactory
                nbU = _result.PatchInUCount;
                nbV = _result.PatchInVCount;
                if (iso.Type() == IsoV)
                {
                    nbPatch = (nbU + 1) * nbV;
                    more = uChoice.Value(iso.T0(), iso.T1(), out dec);
                }
                else
                {
                    nbPatch = (nbV + 1) * nbU;
                    more = vChoice.Value(iso.T0(), iso.T1(), out dec);
                }

                if (nbPatch <= _maxPatches && more)
                {
                    // It is possible to cut iso
                    if (iso.Type() == IsoV)
                    {
                        _result.UpdateInU(dec);
                        _constraints.UpdateInU(dec);
                    }
                    else
                    {
                        _result.UpdateInV(dec);
                        _constraints.UpdateInV(dec);
                    }
                }
                else
                {
                    // It is not possible to cut : the result is preserved
                    if (iso.HasResult)
                    {
                        iso.OverwriteApprox();
                        _constraints.ChangeIso(ind1, ind2, iso);
                        _constraints.Node(indN1).Assign(n1);
                        _constraints.Node(indN2).Assign(n2);
                    }
                    else
                    {
                        _hasResult = _done = false;
                        throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : Curve Approximation Error");
                    }
                }
            }
        }
    }

    private void ComputeConstraints(Cutting uChoice,
                                    Cutting vChoice,
                                    EvaluatorFunc2Var func,
                                    Criterion crit)
    {
        double dec;
        bool more, critRel = crit.Type() == Relative;
        int ind1 = 0, ind2 = 0, nbPatch, nbU, nbV;
        int indN1, indN2;
        int iu = _conditions.UOrder(), iv = _conditions.VOrder();
        Node n1 = new(iu, iv), n2 = new(iu, iv);

        for (Iso? iso = _constraints.FirstNotApprox(ref ind1, ref ind2);
             iso is not null;
             iso = _constraints.FirstNotApprox(ref ind1, ref ind2))
        {
            // approximation of the iso and calculation of constraints at the extremities
            indN1 = _constraints.FirstNode(iso.Type(), ind1, ind2);
            n1.Assign(_constraints.Node(indN1));
            indN2 = _constraints.LastNode(iso.Type(), ind1, ind2);
            n2.Assign(_constraints.Node(indN2));

            // note that old code attempted to make copy of anIso here (but copy was incomplete)
            iso.MakeApprox(_conditions,
                             _firstParInU,
                             _lastParInU,
                             _firstParInV,
                             _lastParInV,
                             func,
                             n1,
                             n2);

            if (iso.IsApproximated)
            {
                // iso is approached at the required tolerance
                _constraints.ChangeIso(ind1, ind2, iso);
                _constraints.Node(indN1).Assign(n1);
                _constraints.Node(indN2).Assign(n2);
            }
            else
            {
                // Approximation is not satisfactory
                nbU = _result.PatchInUCount;
                nbV = _result.PatchInVCount;
                if (iso.Type() == IsoV)
                {
                    nbPatch = (nbU + 1) * nbV;
                    more = uChoice.Value(iso.T0(), iso.T1(), out dec);
                }
                else
                {
                    nbPatch = (nbV + 1) * nbU;
                    more = vChoice.Value(iso.T0(), iso.T1(), out dec);
                }

                // To force Overwrite if the criterion is Absolute
                more = more && critRel;

                if (nbPatch <= _maxPatches && more)
                {
                    // It is possible to cut iso
                    if (iso.Type() == IsoV)
                    {
                        _result.UpdateInU(dec);
                        _constraints.UpdateInU(dec);
                    }
                    else
                    {
                        _result.UpdateInV(dec);
                        _constraints.UpdateInV(dec);
                    }
                }
                else
                {
                    // It is not possible to cut: the result is preserved
                    if (iso.HasResult)
                    {
                        iso.OverwriteApprox();
                        _constraints.ChangeIso(ind1, ind2, iso);
                        _constraints.Node(indN1).Assign(n1);
                        _constraints.Node(indN2).Assign(n2);
                    }
                    else
                    {
                        _hasResult = _done = false;
                        throw new ConstructionErrorException("AdvApp2Var_ApproxAFunc2Var : Curve Approximation Error");
                    }
                }
            }
        }
    }

    // Computation of the 3D errors on the approximation result ; used by Perform
    private void Compute3DErrors()
    {
        int iesp, ipat;
        double error_max, error_moy, error_U0, error_V0, error_U1, error_V1;
        double tol, f1Tol, f2Tol, f3Tol, f4Tol;
        if (_numSubSpaces[2] > 0)
        {
            _my3DMaxError = new NCollection_HArray1<double>(1, _numSubSpaces[2]);
            _my3DAverageError = new NCollection_HArray1<double>(1, _numSubSpaces[2]);
            _my3DUFrontError = new NCollection_HArray1<double>(1, _numSubSpaces[2]);
            _my3DVFrontError = new NCollection_HArray1<double>(1, _numSubSpaces[2]);
            for (iesp = 1; iesp <= _numSubSpaces[2]; iesp++)
            {
                error_max = 0;
                error_moy = 0.0;
                error_U0 = 0.0;
                error_V0 = 0.0;
                error_U1 = 0.0;
                error_V1 = 0.0;
                tol = _my3DTolerances!.Value(iesp);
                f1Tol = _my3DTolOnFront!.Value(iesp, 1);
                f2Tol = _my3DTolOnFront.Value(iesp, 2);
                f3Tol = _my3DTolOnFront.Value(iesp, 3);
                f4Tol = _my3DTolOnFront.Value(iesp, 4);
                for (ipat = 1; ipat <= _result.PatchCount; ipat++)
                {
                    error_max = Max(_result[ipat].MaxErrors()!.Value(iesp), error_max);
                    error_U0 = Max(_result[ipat].IsoErrors()!.Value(iesp, 3), error_U0);
                    error_U1 = Max(_result[ipat].IsoErrors()!.Value(iesp, 4), error_U1);
                    error_V0 = Max(_result[ipat].IsoErrors()!.Value(iesp, 1), error_V0);
                    error_V1 = Max(_result[ipat].IsoErrors()!.Value(iesp, 2), error_V1);
                    error_moy += _result[ipat].AverageErrors()!.Value(iesp);
                }
                _my3DMaxError.SetValue(iesp, error_max);
                _my3DUFrontError.SetValue(iesp, Max(error_U0, error_U1));
                _my3DVFrontError.SetValue(iesp, Max(error_V0, error_V1));
                error_moy /= (double)_result.PatchCount;
                _my3DAverageError.SetValue(iesp, error_moy);
                if (error_max > tol || error_U0 > f3Tol || error_U1 > f4Tol || error_V0 > f1Tol || error_V1 > f2Tol)
                {
                    _done = false;
                }
            }
        }
    }

    // Computation of the max value of the criterion on the approximation result ; used by Perform
    private void ComputeCritError()
    {
        int iesp, ipat;
        double crit_max;
        if (_numSubSpaces[2] > 0)
        {
            for (iesp = 1; iesp <= _numSubSpaces[2]; iesp++)
            {
                crit_max = 0.0;
                for (ipat = 1; ipat <= _result.PatchCount; ipat++)
                {
                    crit_max = Max(_result[ipat].CritValue(), crit_max);
                }
                _criterionError = crit_max;
            }
        }
    }

    // Conversion of the approximation result in BSpline; used by Create
    private void ConvertBS()
    {
        // Homogeneization of degrees
        int iu = _conditions.UOrder(), iv = _conditions.VOrder();
        int ncfu = _conditions.ULimit(), ncfv = _conditions.VLimit();
        _result.SameDegree(iu, iv, out ncfu, out ncfv);
        _degreeInU = ncfu - 1;
        _degreeInV = ncfv - 1;

        // Calculate resulting surfaces
        _surfaces = new NCollection_HArray1<Surface?>(1, _numSubSpaces[2]);

        int j;
        NCollection_Array1<double> uKnots = new(1, _result.PatchInUCount + 1);
        for (j = 1; j <= uKnots.Length(); j++)
        {
            uKnots.SetValue(j, _result.UParameter(j));
        }

        NCollection_Array1<double> vKnots = new(1, _result.PatchInVCount + 1);
        for (j = 1; j <= vKnots.Length(); j++)
        {
            vKnots.SetValue(j, _result.VParameter(j));
        }

        // Prepare data for conversion grid of polynoms --> poles
        NCollection_HArray1<double> uint1 = new(1, 2);
        uint1.SetValue(1, -1);
        uint1.SetValue(2, 1);
        NCollection_HArray1<double> vint1 = new(1, 2);
        vint1.SetValue(1, -1);
        vint1.SetValue(2, 1);

        NCollection_HArray1<double> uint2 = new(1, _result.PatchInUCount + 1);
        for (j = 1; j <= uint2.Length(); j++)
        {
            uint2.SetValue(j, _result.UParameter(j));
        }
        NCollection_HArray1<double> vint2 = new(1, _result.PatchInVCount + 1);
        for (j = 1; j <= vint2.Length(); j++)
        {
            vint2.SetValue(j, _result.VParameter(j));
        }

        int nmax = _result.PatchInUCount * _result.PatchInVCount,
            Size_eq = _conditions.ULimit() * _conditions.VLimit() * 3;

        NCollection_HArray2<int> nbCoeff = new(1, nmax, 1, 2);
        NCollection_HArray1<double> poly = new(1, nmax * Size_eq);

        int ssp, i;
        for (ssp = 1; ssp <= _numSubSpaces[2]; ssp++)
        {
            // Creation of the grid of polynoms
            int n = 0, icf = 1, ieq;
            for (j = 1; j <= _result.PatchInVCount; j++)
            {
                for (i = 1; i <= _result.PatchInUCount; i++)
                {
                    n++;
                    nbCoeff.SetValue(n, 1, _result.Patch(i, j).CoeffInUCount);
                    nbCoeff.SetValue(n, 2, _result.Patch(i, j).CoeffInVCount);
                    for (ieq = 1; ieq <= Size_eq; ieq++)
                    {
                        poly.SetValue(icf, _result.Patch(i, j).Coefficients(ssp, _conditions)!.Value(ieq));
                        icf++;
                    }
                }
            }

            // Conversion into poles
            GridPolynomialToPoles cvP = new(_result.PatchInUCount,
                                                    _result.PatchInVCount,
                                                    iu,
                                                    iv,
                                                    _maxDegInU,
                                                    _maxDegInV,
                                                    nbCoeff.Array2(),
                                                    poly.Array1(),
                                                    uint1.Array1(),
                                                    vint1.Array1(),
                                                    uint2.Array1(),
                                                    vint2.Array1());
            if (!cvP.IsDone)
            {
                _done = false;
            }

            // Conversion into BSpline
            BSplineSurface surface = new(cvP.Poles(),
                                               cvP.UKnots(),
                                               cvP.VKnots(),
                                               cvP.UMultiplicities(),
                                               cvP.VMultiplicities(),
                                               cvP.UDegree(),
                                               cvP.VDegree());
            // The port: the result is handed to callers, so it is frozen (decision 197)
            surface.Freeze();
            _surfaces.ChangeValue(ssp) = surface;
        }
    }

    /// <summary>
    /// True if the approximation succeeded within the imposed tolerances and the wished continuities.
    /// </summary>
    public bool IsDone => _done;

    /// <summary>
    /// True if the approximation came out with a result, not necessarily within the required tolerance or with the
    /// wished continuities.
    /// </summary>
    public bool HasResult => _hasResult;

    /// <summary>The B-spline surface of the subspace <paramref name="sspIndex"/> (frozen; decision 197).</summary>
    public BSplineSurface Surface(int sspIndex) => (BSplineSurface)_surfaces!.Value(sspIndex)!;

    public int UDegree() => _degreeInU;

    public int VDegree() => _degreeInV;

    public int NumSubSpaces(int dimension) => _numSubSpaces[dimension - 1];

    private static NCollection_HArray1<double>? CopyOf(NCollection_HArray1<double>? array) =>
        array is null ? null : new NCollection_HArray1<double>(array);

    /// <summary>The maximum errors of the subspaces of dimension <paramref name="dimension"/>: a copy, or null.</summary>
    /// <exception cref="OutOfRangeException">A dimension other than 1, 2 or 3.</exception>
    public NCollection_HArray1<double>? MaxError(int dimension)
    {
        NCollection_HArray1<double>? ePtr = null;
        if (dimension < 1 || dimension > 3)
        {
            throw new OutOfRangeException(
                "AdvApp2Var_ApproxAFunc2Var::MaxError : Dimension must be equal to 1,2 or 3 !");
        }
        switch (dimension)
        {
            case 1:
                ePtr = _my1DMaxError;
                break;
            case 2:
                ePtr = _my2DMaxError;
                break;
            case 3:
                ePtr = _my3DMaxError;
                break;
        }
        return CopyOf(ePtr);
    }

    /// <summary>The average errors of the subspaces of dimension <paramref name="dimension"/>: a copy, or null.</summary>
    /// <exception cref="OutOfRangeException">A dimension other than 1, 2 or 3.</exception>
    public NCollection_HArray1<double>? AverageError(int dimension)
    {
        NCollection_HArray1<double>? ePtr = null;
        if (dimension < 1 || dimension > 3)
        {
            throw new OutOfRangeException(
                "AdvApp2Var_ApproxAFunc2Var::AverageError : Dimension must be equal to 1,2 or 3 !");
        }
        switch (dimension)
        {
            case 1:
                ePtr = _my1DAverageError;
                break;
            case 2:
                ePtr = _my2DAverageError;
                break;
            case 3:
                ePtr = _my3DAverageError;
                break;
        }
        return CopyOf(ePtr);
    }

    /// <summary>The maximum errors on the U frontiers (dimension 3 only is computed): a copy, or null.</summary>
    /// <exception cref="OutOfRangeException">A dimension other than 1, 2 or 3.</exception>
    public NCollection_HArray1<double>? UFrontError(int dimension)
    {
        NCollection_HArray1<double>? ePtr = null;
        if (dimension < 1 || dimension > 3)
        {
            throw new OutOfRangeException(
                "AdvApp2Var_ApproxAFunc2Var::UFrontError : Dimension must be equal to 1,2 or 3 !");
        }
        switch (dimension)
        {
            case 1:
                ePtr = _my1DUFrontError;
                break;
            case 2:
                ePtr = _my2DUFrontError;
                break;
            case 3:
                ePtr = _my3DUFrontError;
                break;
        }
        return CopyOf(ePtr);
    }

    /// <summary>The maximum errors on the V frontiers (dimension 3 only is computed): a copy, or null.</summary>
    /// <exception cref="OutOfRangeException">A dimension other than 1, 2 or 3.</exception>
    public NCollection_HArray1<double>? VFrontError(int dimension)
    {
        NCollection_HArray1<double>? ePtr = null;
        if (dimension <= 0 || dimension > 3)
        {
            throw new OutOfRangeException(
                "AdvApp2Var_ApproxAFunc2Var::VFrontError : Dimension must be equal to 1,2 or 3 !");
        }
        switch (dimension)
        {
            case 1:
                ePtr = _my1DVFrontError;
                break;
            case 2:
                ePtr = _my2DVFrontError;
                break;
            case 3:
                ePtr = _my3DVFrontError;
                break;
        }
        return CopyOf(ePtr);
    }

    /// <summary>The maximum error of the surface <paramref name="sspIndex"/>.</summary>
    /// <exception cref="OutOfRangeException">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double MaxError(int dimension, int sspIndex)
    {
        if (dimension != 3 || sspIndex != 1)
        {
            throw new OutOfRangeException("AdvApp2Var_ApproxAFunc2Var::MaxError: ONE Surface 3D only !");
        }
        return _my3DMaxError!.Value(sspIndex);
    }

    /// <summary>The average error of the surface <paramref name="sspIndex"/>.</summary>
    /// <exception cref="OutOfRangeException">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double AverageError(int dimension, int sspIndex)
    {
        if (dimension != 3 || sspIndex != 1)
        {
            throw new OutOfRangeException("AdvApp2Var_ApproxAFunc2Var::AverageError : ONE Surface 3D only !");
        }
        return _my3DAverageError!.Value(sspIndex);
    }

    /// <summary>The maximum error on the U frontiers of the surface <paramref name="sspIndex"/>.</summary>
    /// <exception cref="OutOfRangeException">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double UFrontError(int dimension, int sspIndex)
    {
        if (dimension != 3 || sspIndex != 1)
        {
            throw new OutOfRangeException("AdvApp2Var_ApproxAFunc2Var::UFrontError : ONE Surface 3D only !");
        }
        return _my3DUFrontError!.Value(sspIndex);
    }

    /// <summary>The maximum error on the V frontiers of the surface <paramref name="sspIndex"/>.</summary>
    /// <exception cref="OutOfRangeException">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double VFrontError(int dimension, int sspIndex)
    {
        if (dimension != 3 || sspIndex != 1)
        {
            throw new OutOfRangeException("AdvApp2Var_ApproxAFunc2Var::VFrontError : ONE Surface 3D only !");
        }
        return _my3DVFrontError!.Value(sspIndex);
    }

    /// <summary>The maximum value of the criterion over the patches.</summary>
    /// <exception cref="OutOfRangeException">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double CritError(int dimension, int sspIndex)
    {
        if (dimension != 3 || sspIndex != 1)
        {
            throw new OutOfRangeException("AdvApp2Var_ApproxAFunc2Var::CritError: ONE Surface 3D only !");
        }
        return _criterionError;
    }

    /// <summary>Writes information on the result (C++: <c>Dump(Standard_OStream&amp;)</c>).</summary>
    public void Dump(TextWriter o)
    {
        int iesp = 1, nbKU, nbKV, ik;
        CultureInfo c = CultureInfo.InvariantCulture;
        o.Write('\n');
        if (!_hasResult)
        {
            o.Write("No result" + '\n');
        }
        else
        {
            o.Write("There is a result");
            if (_done)
            {
                o.Write(" within the requested tolerance " + _my3DTolerances!.Value(iesp).ToString(c) + '\n');
            }
            else if (_my3DMaxError!.Value(iesp) > _my3DTolerances!.Value(iesp))
            {
                o.Write(" WITHOUT the requested tolerance " + _my3DTolerances.Value(iesp).ToString(c) + '\n');
            }
            else
            {
                o.Write(" WITHOUT the requested continuities " + '\n');
            }
            o.Write('\n');
            o.Write("Result max error :" + _my3DMaxError!.Value(iesp).ToString(c) + '\n');
            o.Write("Result average error :" + _my3DAverageError!.Value(iesp).ToString(c) + '\n');
            o.Write("Result max error on U frontiers :" + _my3DUFrontError!.Value(iesp).ToString(c) + '\n');
            o.Write("Result max error on V frontiers :" + _my3DVFrontError!.Value(iesp).ToString(c) + '\n');
            o.Write('\n');
            o.Write("Degree of Bezier patches in U : " + _degreeInU.ToString(c) + "  in V : "
                    + _degreeInV.ToString(c) + '\n');
            o.Write('\n');
            BSplineSurface s = (BSplineSurface)_surfaces!.Value(iesp)!;
            o.Write("Number of poles in U : " + s.UPoleCount.ToString(c) + "  in V : " + s.VPoleCount.ToString(c)
                    + '\n');
            o.Write('\n');
            nbKU = s.UKnotCount;
            nbKV = s.VKnotCount;
            o.Write("Number of knots in U : " + nbKU.ToString(c) + '\n');
            for (ik = 1; ik <= nbKU; ik++)
            {
                o.Write("   " + ik.ToString(c) + " : " + s.UKnot(ik).ToString(c) + "   mult : "
                        + s.UMultiplicity(ik).ToString(c) + '\n');
            }
            o.Write('\n');
            o.Write("Number of knots in V : " + nbKV.ToString(c) + '\n');
            for (ik = 1; ik <= nbKV; ik++)
            {
                o.Write("   " + ik.ToString(c) + " : " + s.VKnot(ik).ToString(c) + "   mult : "
                        + s.VMultiplicity(ik).ToString(c) + '\n');
            }
            o.Write('\n');
        }
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
