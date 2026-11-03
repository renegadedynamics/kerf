// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineSurface.cxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineSurface_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.BSplKnotDistribution;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

/// <summary>
/// A B-spline surface: uniform or non-uniform, rational or non-rational, periodic or non-periodic in each
/// parametric direction. It is defined by its degrees in U and V (1 to <see cref="MaxDegree"/>, 25), a
/// rectangular grid of poles (with weights when rational), and in each direction a strictly increasing knot
/// sequence with multiplicities, all indexed from 1. The rows of the pole grid follow U, the columns V.
/// </summary>
/// <remarks>
/// <para>
/// Arrays: <see cref="UKnots()"/>, <see cref="VKnots()"/>, <see cref="UMultiplicities()"/>,
/// <see cref="VMultiplicities()"/>, <see cref="UKnotSequence()"/> and <see cref="VKnotSequence()"/> return
/// immutable <see cref="PackedList{T}"/>s (index 0 is OCCT's index 1); <see cref="Poles()"/>,
/// <see cref="Weights()"/> and <see cref="WeightsArray"/> return new copies of the two-dimensional arrays (OCCT
/// returns references to the surface's arrays). Code inside OCCT.NET uses the internal <c>Internal*</c>
/// accessors, which return the stored arrays (shared: don't modify).
/// </para>
/// <para>
/// The evaluation representation (<see cref="HasEvalRepresentation"/>) is set by
/// <see cref="WithEvalRepresentation"/>, which stores a copy of the descriptor. Modifications drop it, as in OCCT.
/// </para>
/// </remarks>
public class BSplineSurface : BoundedSurface
{
    private NCollection_Array2<Point3D> _poles = new NCollection_Array2<Point3D>();
    private NCollection_Array2<double> _weights = new NCollection_Array2<double>();
    private NCollection_Array1<double> _uKnots = new NCollection_Array1<double>();
    private NCollection_Array1<double> _vKnots = new NCollection_Array1<double>();
    private NCollection_Array1<double> _uFlatKnots = new NCollection_Array1<double>();
    private NCollection_Array1<double> _vFlatKnots = new NCollection_Array1<double>();
    private NCollection_Array1<int> _uMults = new NCollection_Array1<int>();
    private NCollection_Array1<int> _vMults = new NCollection_Array1<int>();
    private RepSurfaceDesc.Base? _evalRep;
    private int _uDeg = 0;
    private int _vDeg = 0;
    private bool _uPeriodic = false;
    private bool _vPeriodic = false;
    private bool _uRational = false;
    private bool _vRational = false;
    private BSplKnotDistribution _uKnotSet = NonUniform;
    private BSplKnotDistribution _vKnotSet = NonUniform;
    private Continuity _uSmooth = C0;
    private Continuity _vSmooth = C0;
    private double _uMaxDerivInv = 0.0;
    private double _vMaxDerivInv = 0.0;
    private volatile bool _maxDerivInvOk = false;

    private const double Precision_PConfusion = 1.0e-7 * 0.01;

    //=============================================================================================
    // Geom_BSplineSurface.cxx
    //=============================================================================================

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation => _evalRep is not null;

    /// <summary>The evaluation representation descriptor (may be null). Stored, not copied: don't modify.</summary>
    internal RepSurfaceDesc.Base? EvalRepresentation() => _evalRep;

    /// <summary>Sets a new evaluation representation, after checking it (no circular reference).</summary>
    internal void SetEvalRepresentation(RepSurfaceDesc.Base? desc)
    {
        CheckMutable();
        RepUtils.ValidateSurfaceDesc(desc, this);
        _evalRep = desc;
    }

    /// <summary>Removes the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        _evalRep = null;
    }

    private static void CheckSurfaceData(NCollection_Array2<Point3D> sPoles,
                                         NCollection_Array1<double> suKnots,
                                         NCollection_Array1<double> svKnots,
                                         NCollection_Array1<int> suMults,
                                         NCollection_Array1<int> svMults,
                                         int uDegree,
                                         int vDegree,
                                         bool uPeriodic,
                                         bool vPeriodic)
    {
        if (uDegree < 1 || uDegree > BSplineSurface.MaxDegree() || vDegree < 1
            || vDegree > BSplineSurface.MaxDegree())
        {
            throw new ConstructionErrorException("Geom_BSplineSurface: invalid degree");
        }

        if (sPoles.ColLength() < 2 || sPoles.RowLength() < 2)
        {
            throw new ConstructionErrorException("Geom_BSplineSurface: at least 2 poles required");
        }

        if (suKnots.Length() != suMults.Length() || svKnots.Length() != svMults.Length())
        {
            throw new ConstructionErrorException("Geom_BSplineSurface: Knot and Mult array size mismatch");
        }

        int i;
        for (i = suKnots.Lower(); i < suKnots.Upper(); i++)
        {
            if (suKnots[i + 1] - suKnots[i] <= Epsilon(Math.Abs(suKnots[i])))
            {
                throw new ConstructionErrorException("Geom_BSplineSurface: UKnots interval values too close");
            }
        }

        for (i = svKnots.Lower(); i < svKnots.Upper(); i++)
        {
            if (svKnots[i + 1] - svKnots[i] <= Epsilon(Math.Abs(svKnots[i])))
            {
                throw new ConstructionErrorException("Geom_BSplineSurface: VKnots interval values too close");
            }
        }

        if (sPoles.ColLength() != global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(uDegree, uPeriodic, suMults))
        {
            throw new ConstructionErrorException("Geom_BSplineSurface: # U Poles and degree mismatch");
        }

        if (sPoles.RowLength() != global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(vDegree, vPeriodic, svMults))
        {
            throw new ConstructionErrorException("Geom_BSplineSurface: # V Poles and degree mismatch");
        }
    }

    /// <summary>Checks the rationality of an array of weights in each direction.</summary>
    internal static void Rational(NCollection_Array2<double> weights, out bool urational, out bool vrational)
    {
        int i, j;
        j = weights.LowerCol();
        vrational = false;
        while (!vrational && j <= weights.UpperCol())
        {
            i = weights.LowerRow();
            while (!vrational && i <= weights.UpperRow() - 1)
            {
                vrational = (Math.Abs(weights[i, j] - weights[i + 1, j]) > Epsilon(Math.Abs(weights[i, j])));
                i++;
            }
            j++;
        }

        i = weights.LowerRow();
        urational = false;
        while (!urational && i <= weights.UpperRow())
        {
            j = weights.LowerCol();
            while (!urational && j <= weights.UpperCol() - 1)
            {
                urational = (Math.Abs(weights[i, j] - weights[i, j + 1]) > Epsilon(Math.Abs(weights[i, j])));
                j++;
            }
            i++;
        }
    }

    /// <summary>A new, unfrozen surface equal to this one.</summary>
    public sealed override Geometry3D Copy() => new BSplineSurface(this);

    /// <summary>A copy of theOther, without validation (OCCT's copy constructor).</summary>
    public BSplineSurface(BSplineSurface other)
    {
        _poles = new NCollection_Array2<Point3D>(other._poles);
        _weights = (other._uRational || other._vRational)
                      ? new NCollection_Array2<double>(other._weights)
                      : SurfaceBSplSLib.UnitWeights(other._poles.ColLength(), other._poles.RowLength());
        _uKnots = new NCollection_Array1<double>(other._uKnots);
        _vKnots = new NCollection_Array1<double>(other._vKnots);
        _uFlatKnots = new NCollection_Array1<double>(other._uFlatKnots);
        _vFlatKnots = new NCollection_Array1<double>(other._vFlatKnots);
        _uMults = new NCollection_Array1<int>(other._uMults);
        _vMults = new NCollection_Array1<int>(other._vMults);
        _evalRep = RepUtils.CloneSurfaceDesc(other._evalRep);
        _uDeg = other._uDeg;
        _vDeg = other._vDeg;
        _uPeriodic = other._uPeriodic;
        _vPeriodic = other._vPeriodic;
        _uRational = other._uRational;
        _vRational = other._vRational;
        _uKnotSet = other._uKnotSet;
        _vKnotSet = other._vKnotSet;
        _uSmooth = other._uSmooth;
        _vSmooth = other._vSmooth;
        _uMaxDerivInv = other._uMaxDerivInv;
        _vMaxDerivInv = other._vMaxDerivInv;
        _maxDerivInvOk = false;
    }

    /// <summary>A non-rational B-spline surface.</summary>
    /// <exception cref="ConstructionErrorException">
    /// A degree not in [1, MaxDegree()], fewer than 2 poles in a direction, knots and multiplicities of different
    /// lengths, knots not increasing, or numbers of poles that don't match the multiplicities.
    /// </exception>
    public BSplineSurface(NCollection_Array2<Point3D> poles,
                               NCollection_Array1<double> uKnots,
                               NCollection_Array1<double> vKnots,
                               NCollection_Array1<int> uMults,
                               NCollection_Array1<int> vMults,
                               int uDegree,
                               int vDegree,
                               bool uPeriodic = false,
                               bool vPeriodic = false)
    {
        _uRational = false;
        _vRational = false;
        _maxDerivInvOk = false;

        // check
        CheckSurfaceData(poles, uKnots, vKnots, uMults, vMults, uDegree, vDegree, uPeriodic, vPeriodic);

        // set scalar fields
        _uDeg = uDegree;
        _vDeg = vDegree;
        _uPeriodic = uPeriodic;
        _vPeriodic = vPeriodic;

        // copy arrays
        _poles.Resize(1, poles.ColLength(), 1, poles.RowLength(), false);
        _poles.CopyValues(poles);
        _weights = SurfaceBSplSLib.UnitWeights(poles.ColLength(), poles.RowLength());

        _uKnots.Resize(1, uKnots.Length(), false);
        _uKnots.CopyValues(uKnots);
        _uMults.Resize(1, uMults.Length(), false);
        _uMults.CopyValues(uMults);
        _vKnots.Resize(1, vKnots.Length(), false);
        _vKnots.CopyValues(vKnots);
        _vMults.Resize(1, vMults.Length(), false);
        _vMults.CopyValues(vMults);

        UpdateUKnots();
        UpdateVKnots();
    }

    /// <summary>A rational B-spline surface (non-rational if the weights are all equal in each direction).</summary>
    /// <exception cref="ConstructionErrorException">
    /// As the non-rational constructor, or weights of a different size from the poles, or a weight not greater
    /// than gp::Resolution().
    /// </exception>
    public BSplineSurface(NCollection_Array2<Point3D> poles,
                               NCollection_Array2<double> weights,
                               NCollection_Array1<double> uKnots,
                               NCollection_Array1<double> vKnots,
                               NCollection_Array1<int> uMults,
                               NCollection_Array1<int> vMults,
                               int uDegree,
                               int vDegree,
                               bool uPeriodic = false,
                               bool vPeriodic = false)
    {
        _uRational = false;
        _vRational = false;
        _maxDerivInvOk = false;

        // check weights
        if (weights.ColLength() != poles.ColLength())
        {
            throw new ConstructionErrorException("Geom_BSplineSurface: U Weights and Poles array size mismatch");
        }

        if (weights.RowLength() != poles.RowLength())
        {
            throw new ConstructionErrorException("Geom_BSplineSurface: V Weights and Poles array size mismatch");
        }

        int i, j;
        for (i = weights.LowerRow(); i <= weights.UpperRow(); i++)
        {
            for (j = weights.LowerCol(); j <= weights.UpperCol(); j++)
            {
                if (weights[i, j] <= Constants.Resolution())
                {
                    throw new ConstructionErrorException("Geom_BSplineSurface: Weights values too small");
                }
            }
        }

        // check really rational
        Rational(weights, out _uRational, out _vRational);

        // check
        CheckSurfaceData(poles, uKnots, vKnots, uMults, vMults, uDegree, vDegree, uPeriodic, vPeriodic);

        // set scalar fields
        _uDeg = uDegree;
        _vDeg = vDegree;
        _uPeriodic = uPeriodic;
        _vPeriodic = vPeriodic;

        // copy arrays
        _poles.Resize(1, poles.ColLength(), 1, poles.RowLength(), false);
        _poles.CopyValues(poles);

        if (_uRational || _vRational)
        {
            _weights.Resize(1, poles.ColLength(), 1, poles.RowLength(), false);
            _weights.CopyValues(weights);
        }
        else
        {
            _weights = SurfaceBSplSLib.UnitWeights(poles.ColLength(), poles.RowLength());
        }

        _uKnots.Resize(1, uKnots.Length(), false);
        _uKnots.CopyValues(uKnots);
        _uMults.Resize(1, uMults.Length(), false);
        _uMults.CopyValues(uMults);
        _vKnots.Resize(1, vKnots.Length(), false);
        _vKnots.CopyValues(vKnots);
        _vMults.Resize(1, vMults.Length(), false);
        _vMults.CopyValues(vMults);

        UpdateUKnots();
        UpdateVKnots();
    }

    /// <summary>Exchanges the U and V parametric directions; the poles grid is transposed.</summary>
    internal void ExchangeUV()
    {
        CheckMutable();
        ClearEvalRepresentation();
        int lc = _poles.LowerCol();
        int uc = _poles.UpperCol();
        int lr = _poles.LowerRow();
        int ur = _poles.UpperRow();

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(lc, uc, lr, ur);
        NCollection_Array2<double> nweights = new NCollection_Array2<double>();
        if (_uRational || _vRational)
        {
            nweights = new NCollection_Array2<double>(lc, uc, lr, ur);
        }

        for (int i = lc; i <= uc; i++)
        {
            for (int j = lr; j <= ur; j++)
            {
                npoles[i, j] = _poles[j, i];
                if (_uRational || _vRational)
                {
                    nweights.ChangeValue(i, j) = _weights.Value(j, i);
                }
            }
        }

        _poles = npoles;
        if (_uRational || _vRational)
        {
            _weights = nweights;
        }
        else
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
        }

        (_uRational, _vRational) = (_vRational, _uRational);
        (_uPeriodic, _vPeriodic) = (_vPeriodic, _uPeriodic);
        (_uDeg, _vDeg) = (_vDeg, _uDeg);
        (_uKnots, _vKnots) = (_vKnots, _uKnots);
        (_uMults, _vMults) = (_vMults, _uMults);

        UpdateUKnots();
        UpdateVKnots();
    }

    /// <summary>Raises the degrees to UDegree and VDegree (no change where equal to the current degree).</summary>
    /// <exception cref="ConstructionErrorException">A degree lower than the current one or above MaxDegree().</exception>
    internal void IncreaseDegree(int uDegree, int vDegree)
    {
        CheckMutable();
        if (uDegree != _uDeg)
        {
            if (uDegree < _uDeg || uDegree > BSplineSurface.MaxDegree())
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::IncreaseDegree: bad U degree value");
            }

            ClearEvalRepresentation();

            int fromK1 = FirstUKnotIndex();
            int toK2 = LastUKnotIndex();

            int step = uDegree - _uDeg;

            NCollection_Array2<Point3D> npoles =
                new NCollection_Array2<Point3D>(1, _poles.ColLength() + step * (toK2 - fromK1), 1, _poles.RowLength());

            int nbknots = global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegreeCountKnots(_uDeg, uDegree, _uPeriodic, _uMults);

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

            if (_uRational || _vRational)
            {
                NCollection_Array2<double> nweights =
                    new NCollection_Array2<double>(1, npoles.ColLength(), 1, npoles.RowLength());

                SurfaceBSplSLib.IncreaseDegree(true,
                                        _uDeg,
                                        uDegree,
                                        _uPeriodic,
                                        _poles,
                                        _weights,
                                        _uKnots,
                                        _uMults,
                                        npoles,
                                        nweights,
                                        nknots,
                                        nmults);
                _weights = nweights;
            }
            else
            {
                SurfaceBSplSLib.IncreaseDegree(true,
                                        _uDeg,
                                        uDegree,
                                        _uPeriodic,
                                        _poles,
                                        SurfaceBSplSLib.NoWeights(),
                                        _uKnots,
                                        _uMults,
                                        npoles,
                                        SurfaceBSplSLib.NoWeights(),
                                        nknots,
                                        nmults);
                _weights = SurfaceBSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
            }
            _uDeg = uDegree;
            _poles = npoles;
            _uKnots = nknots;
            _uMults = nmults;
            UpdateUKnots();
        }

        if (vDegree != _vDeg)
        {
            if (vDegree < _vDeg || vDegree > BSplineSurface.MaxDegree())
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::IncreaseDegree: bad V degree value");
            }

            ClearEvalRepresentation();

            int fromK1 = FirstVKnotIndex();
            int toK2 = LastVKnotIndex();

            int step = vDegree - _vDeg;

            NCollection_Array2<Point3D> npoles =
                new NCollection_Array2<Point3D>(1, _poles.ColLength(), 1, _poles.RowLength() + step * (toK2 - fromK1));

            int nbknots = global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegreeCountKnots(_vDeg, vDegree, _vPeriodic, _vMults);

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

            if (_uRational || _vRational)
            {
                NCollection_Array2<double> nweights =
                    new NCollection_Array2<double>(1, npoles.ColLength(), 1, npoles.RowLength());

                SurfaceBSplSLib.IncreaseDegree(false,
                                        _vDeg,
                                        vDegree,
                                        _vPeriodic,
                                        _poles,
                                        _weights,
                                        _vKnots,
                                        _vMults,
                                        npoles,
                                        nweights,
                                        nknots,
                                        nmults);
                _weights = nweights;
            }
            else
            {
                SurfaceBSplSLib.IncreaseDegree(false,
                                        _vDeg,
                                        vDegree,
                                        _vPeriodic,
                                        _poles,
                                        SurfaceBSplSLib.NoWeights(),
                                        _vKnots,
                                        _vMults,
                                        npoles,
                                        SurfaceBSplSLib.NoWeights(),
                                        nknots,
                                        nmults);
                _weights = SurfaceBSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
            }
            _vDeg = vDegree;
            _poles = npoles;
            _vKnots = nknots;
            _vMults = nmults;
            UpdateVKnots();
        }
    }

    /// <summary>Raises the multiplicity of the U knot UIndex to M.</summary>
    internal void IncreaseUMultiplicity(int uIndex, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = _uKnots.Value(uIndex);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - _uMults.Value(uIndex);
        InsertUKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicities of the U knots FromI1 to ToI2 to M.</summary>
    internal void IncreaseUMultiplicity(int fromI1, int toI2, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(_uKnots, fromI1, fromI1, toI2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(fromI1, toI2);
        for (int i = fromI1; i <= toI2; i++)
        {
            m[i] = M - _uMults.Value(i);
        }
        InsertUKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicity of the V knot VIndex to M.</summary>
    internal void IncreaseVMultiplicity(int vIndex, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = _vKnots.Value(vIndex);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - _vMults.Value(vIndex);
        InsertVKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicities of the V knots FromI1 to ToI2 to M.</summary>
    internal void IncreaseVMultiplicity(int fromI1, int toI2, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(_vKnots, fromI1, fromI1, toI2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(fromI1, toI2);
        for (int i = fromI1; i <= toI2; i++)
        {
            m[i] = M - _vMults.Value(i);
        }
        InsertVKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>The common part of <see cref="Segment"/> and <see cref="CheckAndSegment"/>.</summary>
    protected void SegmentCore(double u1,
                           double u2,
                           double v1,
                           double v2,
                           double epsU,
                           double epsV,
                           bool segmentInU,
                           bool segmentInV)
    {
        CheckMutable();
        ClearEvalRepresentation();
        double deltaU = u2 - u1;
        if (_uPeriodic)
        {
            double uPeriod = _uKnots.Last() - _uKnots.First();
            if (deltaU - uPeriod > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                throw new DomainErrorException("Geom_BSplineSurface::Segment");
            }
            if (deltaU > uPeriod)
            {
                deltaU = uPeriod;
            }
        }

        double deltaV = v2 - v1;
        if (_vPeriodic)
        {
            double vPeriod = _vKnots.Last() - _vKnots.First();
            if (deltaV - vPeriod > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                throw new DomainErrorException("Geom_BSplineSurface::Segment");
            }
            if (deltaV > vPeriod)
            {
                deltaV = vPeriod;
            }
        }

        double newU1, newU2, newV1, newV2;
        double u, v;
        int indexU, indexV;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uKnots, _uMults, u1, _uPeriodic, _uKnots.Lower(), _uKnots.Upper(),
                                 out indexU, out newU1);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uKnots, _uMults, u2, _uPeriodic, _uKnots.Lower(), _uKnots.Upper(),
                                 out indexU, out newU2);
        if (segmentInU)
        {
            // inserting the UKnots
            NCollection_Array1<double> uKnots = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int> uMults = new NCollection_Array1<int>(1, 2);
            uKnots[1] = Min(newU1, newU2);
            uKnots[2] = Max(newU1, newU2);
            uMults[1] = uMults[2] = _uDeg;

            InsertUKnots(uKnots, uMults, epsU);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vKnots, _vMults, v1, _vPeriodic, _vKnots.Lower(), _vKnots.Upper(),
                                 out indexV, out newV1);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vKnots, _vMults, v2, _vPeriodic, _vKnots.Lower(), _vKnots.Upper(),
                                 out indexV, out newV2);
        if (segmentInV)
        {
            // Inserting the VKnots
            NCollection_Array1<double> vKnots = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int> vMults = new NCollection_Array1<int>(1, 2);

            vKnots[1] = Min(newV1, newV2);
            vKnots[2] = Max(newV1, newV2);
            vMults[1] = vMults[2] = _vDeg;
            InsertVKnots(vKnots, vMults, epsV);
        }

        if (_uPeriodic && segmentInU)
        { // set the origine at NewU1
            int index;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uKnots, _uMults, u1, _uPeriodic, _uKnots.Lower(),
                                     _uKnots.Upper(), out index, out u);
            if (Math.Abs(_uKnots.Value(index + 1) - u) <= epsU)
            {
                index++;
            }
            SetUOrigin(index);
            SetUNotPeriodic();
        }

        // compute index1 and index2 to set the new knots and mults
        // OCCT: for a periodic direction left whole (CheckAndSegment), NewU1 + deltaU locates on the first knot again,
        // so index2U == index1U + 1 and the direction keeps one span while staying periodic with all its poles: the
        // knots no longer match the poles (occt-issues.md)
        int index1U, index2U;
        int fromU1 = _uKnots.Lower();
        int toU2 = _uKnots.Upper();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uKnots, _uMults, newU1, _uPeriodic, fromU1, toU2, out index1U, out u);
        if (Math.Abs(_uKnots.Value(index1U + 1) - u) <= epsU)
        {
            index1U++;
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uKnots, _uMults, newU1 + deltaU, _uPeriodic, fromU1, toU2,
                                 out index2U, out u);
        if (Math.Abs(_uKnots.Value(index2U + 1) - u) <= epsU || index2U == index1U)
        {
            index2U++;
        }

        int nbuknots = index2U - index1U + 1;

        NCollection_Array1<double> nuknots = new NCollection_Array1<double>(1, nbuknots);
        NCollection_Array1<int> numults = new NCollection_Array1<int>(1, nbuknots);

        int i, k = 1;
        for (i = index1U; i <= index2U; i++)
        {
            nuknots.SetValue(k, _uKnots.Value(i));
            numults.SetValue(k, _uMults.Value(i));
            k++;
        }
        if (segmentInU)
        {
            numults.SetValue(1, _uDeg + 1);
            numults.SetValue(nbuknots, _uDeg + 1);
        }

        if (_vPeriodic && segmentInV)
        { // set the origine at NewV1
            int index;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vKnots, _vMults, v1, _vPeriodic, _vKnots.Lower(),
                                     _vKnots.Upper(), out index, out v);
            if (Math.Abs(_vKnots.Value(index + 1) - v) <= epsV)
            {
                index++;
            }
            SetVOrigin(index);
            SetVNotPeriodic();
        }

        // compute index1 and index2 to set the new knots and mults
        int index1V, index2V;
        int fromV1 = _vKnots.Lower();
        int toV2 = _vKnots.Upper();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vKnots, _vMults, newV1, _vPeriodic, fromV1, toV2, out index1V, out v);
        if (Math.Abs(_vKnots.Value(index1V + 1) - v) <= epsV)
        {
            index1V++;
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vKnots, _vMults, newV1 + deltaV, _vPeriodic, fromV1, toV2,
                                 out index2V, out v);
        if (Math.Abs(_vKnots.Value(index2V + 1) - v) <= epsV || index2V == index1V)
        {
            index2V++;
        }

        int nbvknots = index2V - index1V + 1;

        NCollection_Array1<double> nvknots = new NCollection_Array1<double>(1, nbvknots);
        NCollection_Array1<int> nvmults = new NCollection_Array1<int>(1, nbvknots);

        k = 1;
        for (i = index1V; i <= index2V; i++)
        {
            nvknots.SetValue(k, _vKnots.Value(i));
            nvmults.SetValue(k, _vMults.Value(i));
            k++;
        }
        if (segmentInV)
        {
            nvmults.SetValue(1, _vDeg + 1);
            nvmults.SetValue(nbvknots, _vDeg + 1);
        }

        // compute index1 and index2 to set the new poles and weights
        int pindex1U = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(_uDeg, index1U, _uPeriodic, _uMults);
        int pindex2U = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(_uDeg, index2U, _uPeriodic, _uMults);

        pindex1U++;
        pindex2U = Min(pindex2U + 1, _poles.ColLength());

        int nbupoles = pindex2U - pindex1U + 1;

        // compute index1 and index2 to set the new poles and weights
        int pindex1V = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(_vDeg, index1V, _vPeriodic, _vMults);
        int pindex2V = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(_vDeg, index2V, _vPeriodic, _vMults);

        pindex1V++;
        pindex2V = Min(pindex2V + 1, _poles.RowLength());

        int nbvpoles = pindex2V - pindex1V + 1;

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, nbupoles, 1, nbvpoles);

        k = 1;
        int j, l;
        if (_uRational || _vRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbupoles, 1, nbvpoles);
            for (i = pindex1U; i <= pindex2U; i++)
            {
                l = 1;
                for (j = pindex1V; j <= pindex2V; j++)
                {
                    npoles.SetValue(k, l, _poles.Value(i, j));
                    nweights.SetValue(k, l, _weights.Value(i, j));
                    l++;
                }
                k++;
            }
            _weights = nweights;
        }
        else
        {
            for (i = pindex1U; i <= pindex2U; i++)
            {
                l = 1;
                for (j = pindex1V; j <= pindex2V; j++)
                {
                    npoles.SetValue(k, l, _poles.Value(i, j));
                    l++;
                }
                k++;
            }
            _weights = SurfaceBSplSLib.UnitWeights(nbupoles, nbvpoles);
        }

        _uKnots = nuknots;
        _uMults = numults;
        _vKnots = nvknots;
        _vMults = nvmults;
        _poles = npoles;

        _maxDerivInvOk = false;
        UpdateUKnots();
        UpdateVKnots();
    }

    /// <summary>
    /// Restricts the surface to the parametric rectangle [U1, U2] x [V1, V2], inserting knots of multiplicity
    /// Degree at the bounds; a periodic direction becomes non-periodic.
    /// </summary>
    /// <exception cref="DomainErrorException">U2 &lt; U1 or V2 &lt; V1, or a segment longer than the period.</exception>
    internal void Segment(double u1,
                          double u2,
                          double v1,
                          double v2,
                          double uTolerance = Precision_PConfusion,
                          double vTolerance = Precision_PConfusion)
    {
        CheckMutable();
        if ((u2 < u1) || (v2 < v1))
        {
            throw new DomainErrorException("Geom_BSplineSurface::Segment");
        }

        double maxU = Max(Math.Abs(u2), Math.Abs(u1));
        double epsU = Max(Epsilon(maxU), uTolerance);

        double maxV = Max(Math.Abs(v2), Math.Abs(v1));
        double epsV = Max(Epsilon(maxV), vTolerance);

        SegmentCore(u1, u2, v1, v2, epsU, epsV, true, true);
    }

    /// <summary>As <see cref="Segment"/>, but segments only the directions whose bounds change.</summary>
    internal void CheckAndSegment(double u1,
                                  double u2,
                                  double v1,
                                  double v2,
                                  double uTolerance = Precision_PConfusion,
                                  double vTolerance = Precision_PConfusion)
    {
        CheckMutable();
        if ((u2 < u1) || (v2 < v1))
        {
            throw new DomainErrorException("Geom_BSplineSurface::CheckAndSegment");
        }

        double maxU = Max(Math.Abs(u2), Math.Abs(u1));
        double epsU = Max(Epsilon(maxU), uTolerance);

        double maxV = Max(Math.Abs(v2), Math.Abs(v1));
        double epsV = Max(Epsilon(maxV), vTolerance);

        bool segment_in_U = (Math.Abs(u1 - _uKnots.Value(_uKnots.Lower())) > epsU)
                            || (Math.Abs(u2 - _uKnots.Value(_uKnots.Upper())) > epsU);
        bool segment_in_V = (Math.Abs(v1 - _vKnots.Value(_vKnots.Lower())) > epsV)
                            || (Math.Abs(v2 - _vKnots.Value(_vKnots.Upper())) > epsV);

        SegmentCore(u1, u2, v1, v2, epsU, epsV, segment_in_U, segment_in_V);
    }

    /// <summary>Changes the value of the U knot UIndex.</summary>
    /// <exception cref="OutOfRangeException">UIndex out of [1, NbUKnots].</exception>
    /// <exception cref="ConstructionErrorException">K not strictly between the neighbouring knots.</exception>
    internal void SetUKnot(int uIndex, double k)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _uKnots.Length())
        {
            throw new OutOfRangeException("Geom_BSplineSurface::SetUKnot: Index and #knots mismatch");
        }

        int newIndex = uIndex;
        double du = Math.Abs(Epsilon(k));
        if (uIndex == 1)
        {
            if (k >= _uKnots.Value(2) - du)
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetUKnot: K out of range");
            }
        }
        else if (uIndex == _uKnots.Length())
        {
            if (k <= _uKnots.Value(_uKnots.Length() - 1) + du)
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetUKnot: K out of range");
            }
        }
        else
        {
            if (k <= _uKnots.Value(newIndex - 1) + du || k >= _uKnots.Value(newIndex + 1) - du)
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetUKnot: K out of range");
            }
        }

        if (k != _uKnots.Value(newIndex))
        {
            ClearEvalRepresentation();
            _uKnots.SetValue(newIndex, k);
            _maxDerivInvOk = false;
            UpdateUKnots();
        }
    }

    /// <summary>Changes the U knots UK.Lower() to UK.Upper().</summary>
    internal void SetUKnots(NCollection_Array1<double> uk)
    {
        CheckMutable();
        int lower = uk.Lower();
        int upper = uk.Upper();
        if (lower < 1 || lower > _uKnots.Length() || upper < 1 || upper > _uKnots.Length())
        {
            throw new OutOfRangeException("Geom_BSplineSurface::SetUKnots: invalid array dimension");
        }
        if (lower > 1)
        {
            if (Math.Abs(uk[lower] - _uKnots.Value(lower - 1)) <= Constants.Resolution())
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetUKnots: invalid knot value");
            }
        }
        if (upper < _uKnots.Length())
        {
            if (Math.Abs(uk[upper] - _uKnots.Value(upper + 1)) <= Constants.Resolution())
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetUKnots: invalid knot value");
            }
        }
        double k1 = uk[lower];
        ClearEvalRepresentation();
        for (int i = lower; i <= upper; i++)
        {
            _uKnots.SetValue(i, uk[i]);
            if (i != lower)
            {
                if (Math.Abs(uk[i] - k1) <= Constants.Resolution())
                {
                    throw new ConstructionErrorException("Geom_BSplineSurface::SetUKnots: invalid knot value");
                }
                k1 = uk[i];
            }
        }

        _maxDerivInvOk = false;
        UpdateUKnots();
    }

    /// <summary>Changes the value of the U knot UIndex and raises its multiplicity to M.</summary>
    internal void SetUKnot(int uIndex, double k, int m)
    {
        CheckMutable();
        IncreaseUMultiplicity(uIndex, m);
        SetUKnot(uIndex, k);
    }

    /// <summary>Changes the value of the V knot VIndex.</summary>
    /// <exception cref="OutOfRangeException">VIndex out of [1, NbVKnots].</exception>
    /// <exception cref="ConstructionErrorException">K not strictly between the neighbouring knots.</exception>
    internal void SetVKnot(int vIndex, double k)
    {
        CheckMutable();
        if (vIndex < 1 || vIndex > _vKnots.Length())
        {
            throw new OutOfRangeException("Geom_BSplineSurface::SetVKnot: Index and #knots mismatch");
        }
        int newIndex = vIndex + _vKnots.Lower() - 1;
        double dv = Math.Abs(Epsilon(k));
        if (vIndex == 1)
        {
            if (k >= _vKnots.Value(2) - dv)
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetVKnot: K out of range");
            }
        }
        else if (vIndex == _vKnots.Length())
        {
            if (k <= _vKnots.Value(_vKnots.Length() - 1) + dv)
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetVKnot: K out of range");
            }
        }
        else
        {
            if (k <= _vKnots.Value(newIndex - 1) + dv || k >= _vKnots.Value(newIndex + 1) - dv)
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetVKnot: K out of range");
            }
        }

        if (k != _vKnots.Value(newIndex))
        {
            ClearEvalRepresentation();
            _vKnots.SetValue(newIndex, k);
            _maxDerivInvOk = false;
            UpdateVKnots();
        }
    }

    /// <summary>Changes the V knots VK.Lower() to VK.Upper().</summary>
    internal void SetVKnots(NCollection_Array1<double> vk)
    {
        CheckMutable();
        int lower = vk.Lower();
        int upper = vk.Upper();
        if (lower < 1 || lower > _vKnots.Length() || upper < 1 || upper > _vKnots.Length())
        {
            throw new OutOfRangeException("Geom_BSplineSurface::SetVKnots: invalid array dimension");
        }
        if (lower > 1)
        {
            if (Math.Abs(vk[lower] - _vKnots.Value(lower - 1)) <= Constants.Resolution())
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetVKnots: invalid knot value");
            }
        }
        if (upper < _vKnots.Length())
        {
            if (Math.Abs(vk[upper] - _vKnots.Value(upper + 1)) <= Constants.Resolution())
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetVKnots: invalid knot value");
            }
        }
        double k1 = vk[lower];
        ClearEvalRepresentation();
        for (int i = lower; i <= upper; i++)
        {
            _vKnots.SetValue(i, vk[i]);
            if (i != lower)
            {
                if (Math.Abs(vk[i] - k1) <= Constants.Resolution())
                {
                    throw new ConstructionErrorException("Geom_BSplineSurface::SetVKnots: invalid knot value");
                }
                k1 = vk[i];
            }
        }

        _maxDerivInvOk = false;
        UpdateVKnots();
    }

    /// <summary>Changes the value of the V knot VIndex and raises its multiplicity to M.</summary>
    internal void SetVKnot(int vIndex, double k, int m)
    {
        CheckMutable();
        IncreaseVMultiplicity(vIndex, m);
        SetVKnot(vIndex, k);
    }

    /// <summary>Inserts the U knot U with multiplicity M (see <see cref="InsertUKnots"/>).</summary>
    internal void InsertUKnot(double u, int M, double parametricTolerance, bool add = true)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = u;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertUKnots(k, m, parametricTolerance, add);
    }

    /// <summary>Inserts the V knot V with multiplicity M (see <see cref="InsertVKnots"/>).</summary>
    internal void InsertVKnot(double v, int M, double parametricTolerance, bool add = true)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = v;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertVKnots(k, m, parametricTolerance, add);
    }

    /// <summary>Raises the multiplicities of the U knots FromI1 to ToI2 by Step.</summary>
    internal void IncrementUMultiplicity(int fromI1, int toI2, int step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(_uKnots, fromI1, fromI1, toI2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(fromI1, toI2);
        m.Init(step);
        InsertUKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Raises the multiplicities of the V knots FromI1 to ToI2 by Step.</summary>
    internal void IncrementVMultiplicity(int fromI1, int toI2, int step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(_vKnots, fromI1, fromI1, toI2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(fromI1, toI2);
        m.Init(step);
        InsertVKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Recomputes the U flat knots, the U knot distribution and the U continuity.</summary>
    protected void UpdateUKnots()
    {
        _maxDerivInvOk = false;
        int maxKnotMult;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotAnalysis(_uDeg, _uPeriodic, _uKnots, _uMults, out _uKnotSet, out maxKnotMult);

        if (_uKnotSet == Uniform && !_uPeriodic)
        {
            _uFlatKnots.Resize(_uKnots.Lower(), _uKnots.Upper(), false);
            _uFlatKnots.Assign(_uKnots);
        }
        else
        {
            _uFlatKnots.Resize(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(_uMults, _uDeg, _uPeriodic), false);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(_uKnots, _uMults, _uDeg, _uPeriodic, _uFlatKnots);
        }

        if (maxKnotMult == 0)
        {
            _uSmooth = CN;
        }
        else
        {
            switch (_uDeg - maxKnotMult)
            {
                case 0:
                    _uSmooth = C0;
                    break;
                case 1:
                    _uSmooth = C1;
                    break;
                case 2:
                    _uSmooth = C2;
                    break;
                case 3:
                    _uSmooth = C3;
                    break;
                default:
                    _uSmooth = C3;
                    break;
            }
        }
    }

    /// <summary>Recomputes the V flat knots, the V knot distribution and the V continuity.</summary>
    protected void UpdateVKnots()
    {
        _maxDerivInvOk = false;
        int maxKnotMult;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotAnalysis(_vDeg, _vPeriodic, _vKnots, _vMults, out _vKnotSet, out maxKnotMult);

        if (_vKnotSet == Uniform && !_vPeriodic)
        {
            _vFlatKnots.Resize(_vKnots.Lower(), _vKnots.Upper(), false);
            _vFlatKnots.Assign(_vKnots);
        }
        else
        {
            _vFlatKnots.Resize(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(_vMults, _vDeg, _vPeriodic), false);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(_vKnots, _vMults, _vDeg, _vPeriodic, _vFlatKnots);
        }

        if (maxKnotMult == 0)
        {
            _vSmooth = CN;
        }
        else
        {
            switch (_vDeg - maxKnotMult)
            {
                case 0:
                    _vSmooth = C0;
                    break;
                case 1:
                    _vSmooth = C1;
                    break;
                case 2:
                    _vSmooth = C2;
                    break;
                case 3:
                    _vSmooth = C3;
                    break;
                default:
                    _vSmooth = C3;
                    break;
            }
        }
    }

    /// <summary>
    /// Brings the parameters of a periodic direction into its period range by whole periods; the parameters of a
    /// non-periodic direction are unchanged.
    /// </summary>
    /// <exception cref="OutOfRangeException">The period is not greater than Epsilon of the parameter.</exception>
    public void PeriodicNormalization(ref double uparameter, ref double vparameter)
    {
        double period, maxVal, minVal;

        if (_uPeriodic)
        {
            maxVal = _uFlatKnots.Value(_uFlatKnots.Upper() - _uDeg);
            minVal = _uFlatKnots.Value(_uDeg + 1);
            double eps = Math.Abs(Epsilon(uparameter));
            period = maxVal - minVal;

            if (period <= eps)
            {
                throw new OutOfRangeException("Geom_BSplineSurface::PeriodicNormalization: Uparameter is too great number");
            }

            bool isLess, isGreater;
            isLess = minVal - uparameter > 0;
            isGreater = uparameter - maxVal > 0;
            if (isLess || isGreater)
            {
                double dPar, nbPer;
                dPar = (isLess) ? (maxVal - uparameter) : (minVal - uparameter);
                nbPer = Math.Truncate(dPar / period); // modf
                uparameter += nbPer * period;
            }
        }
        if (_vPeriodic)
        {
            maxVal = _vFlatKnots.Value(_vFlatKnots.Upper() - _vDeg);
            minVal = _vFlatKnots.Value(_vDeg + 1);
            double eps = Math.Abs(Epsilon(vparameter));
            period = maxVal - minVal;

            if (period <= eps)
            {
                throw new OutOfRangeException("Geom_BSplineSurface::PeriodicNormalization: Vparameter is too great number");
            }

            bool isLess, isGreater;
            isLess = minVal - vparameter > 0;
            isGreater = vparameter - maxVal > 0;
            if (isLess || isGreater)
            {
                double dPar, nbPer;
                dPar = (isLess) ? (maxVal - vparameter) : (minVal - vparameter);
                nbPer = Math.Truncate(dPar / period); // modf
                vparameter += nbPer * period;
            }
        }
    }

    /// <summary>Changes the weight of the pole (UIndex, VIndex); the rationality flags are recomputed.</summary>
    /// <exception cref="ConstructionErrorException">Weight not greater than gp::Resolution().</exception>
    /// <exception cref="OutOfRangeException">An index out of the pole grid.</exception>
    internal void SetWeight(int uIndex, int vIndex, double weight)
    {
        CheckMutable();
        if (weight <= Constants.Resolution())
        {
            throw new ConstructionErrorException("Geom_BSplineSurface::SetWeight: Weight too small");
        }

        if (uIndex < 1 || uIndex > _poles.ColLength() || vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException("Geom_BSplineSurface::SetWeight: Index and #pole mismatch");
        }

        ClearEvalRepresentation();
        if (!_uRational && !_vRational)
        {
            // Make an owned copy of the unit weights view before modifying.
            _weights = new NCollection_Array2<double>(_weights);
        }
        _weights[uIndex + _weights.LowerRow() - 1, vIndex + _weights.LowerCol() - 1] = weight;
        Rational(_weights, out _uRational, out _vRational);
        if (!_uRational && !_vRational)
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
        }

        _maxDerivInvOk = false;
    }

    /// <summary>Changes the weights of the column VIndex (poles CPoleWeights.Lower() to Upper() of it).</summary>
    internal void SetWeightCol(int vIndex, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        if (vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException("Geom_BSplineSurface::SetWeightCol: Index and #pole mismatch");
        }

        if (cPoleWeights.Lower() < 1 || cPoleWeights.Lower() > _poles.ColLength() || cPoleWeights.Upper() < 1
            || cPoleWeights.Upper() > _poles.ColLength())
        {
            throw new ConstructionErrorException("Geom_BSplineSurface::SetWeightCol: invalid array dimension");
        }

        ClearEvalRepresentation();
        if (!_uRational && !_vRational)
        {
            // Make an owned copy of the unit weights view before modifying.
            _weights = new NCollection_Array2<double>(_weights);
        }
        int i = cPoleWeights.Lower();
        while (i <= cPoleWeights.Upper())
        {
            if (cPoleWeights[i] <= Constants.Resolution())
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetWeightCol: Weight too small");
            }
            _weights[i + _weights.LowerRow() - 1, vIndex + _weights.LowerCol() - 1] = cPoleWeights[i];
            i++;
        }
        // Check if it is rational
        Rational(_weights, out _uRational, out _vRational);
        if (!_uRational && !_vRational)
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
        }

        _maxDerivInvOk = false;
    }

    /// <summary>Changes the weights of the row UIndex (poles CPoleWeights.Lower() to Upper() of it).</summary>
    internal void SetWeightRow(int uIndex, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _poles.ColLength())
        {
            throw new OutOfRangeException("Geom_BSplineSurface::SetWeightRow: Index and #pole mismatch");
        }

        if (cPoleWeights.Lower() < 1 || cPoleWeights.Lower() > _poles.RowLength() || cPoleWeights.Upper() < 1
            || cPoleWeights.Upper() > _poles.RowLength())
        {
            throw new ConstructionErrorException("Geom_BSplineSurface::SetWeightRow: invalid array dimension");
        }

        ClearEvalRepresentation();
        if (!_uRational && !_vRational)
        {
            // Make an owned copy of the unit weights view before modifying.
            _weights = new NCollection_Array2<double>(_weights);
        }
        int i = cPoleWeights.Lower();

        while (i <= cPoleWeights.Upper())
        {
            if (cPoleWeights[i] <= Constants.Resolution())
            {
                throw new ConstructionErrorException("Geom_BSplineSurface::SetWeightRow: Weight too small");
            }
            _weights[uIndex + _weights.LowerRow() - 1, i + _weights.LowerCol() - 1] = cPoleWeights[i];
            i++;
        }
        // Check if it is rational
        Rational(_weights, out _uRational, out _vRational);
        if (!_uRational && !_vRational)
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
        }

        _maxDerivInvOk = false;
    }

    //=============================================================================================
    // Geom_BSplineSurface_1.cxx
    //=============================================================================================

    /// <summary>True if the surface is at least CN in the U direction.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public sealed override bool IsCNu(int n)
    {
        Standard_RangeError_Raise_if(n < 0, " ");
        switch (_uSmooth)
        {
            case CN:
                return true;
            case C0:
                return n <= 0;
            case G1:
                return n <= 0;
            case C1:
                return n <= 1;
            case G2:
                return n <= 1;
            case C2:
                return n <= 2;
            case C3:
                return n <= 3 ? true
                              : n <= _uDeg - global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxKnotMult(_uMults, _uMults.Lower() + 1, _uMults.Upper() - 1);
            default:
                return false;
        }
    }

    /// <summary>True if the surface is at least CN in the V direction.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public sealed override bool IsCNv(int n)
    {
        Standard_RangeError_Raise_if(n < 0, " ");

        switch (_vSmooth)
        {
            case CN:
                return true;
            case C0:
                return n <= 0;
            case G1:
                return n <= 0;
            case C1:
                return n <= 1;
            case G2:
                return n <= 1;
            case C2:
                return n <= 2;
            case C3:
                return n <= 3 ? true
                              : n <= _vDeg - global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxKnotMult(_vMults, _vMults.Lower() + 1, _vMults.Upper() - 1);
            default:
                return false;
        }
    }

    /// <summary>The point of parameters (U, V); periodic parameters are normalised first.</summary>
    public sealed override Point3D EvalD0(double u, double v)
    {
        Point3D evalRepResult;
        if (RepUtils.TryEvalSurfaceD0(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        double newU = u;
        double newV = v;
        PeriodicNormalization(ref newU, ref newV);

        Point3D p;
        SurfaceBSplSLib.D0(newU,
                    newV,
                    0,
                    0,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out p);
        return p;
    }

    /// <summary>The point and first derivatives at (U, V).</summary>
    public sealed override ResD1 EvalD1(double u, double v)
    {
        ResD1 evalRepResult;
        if (RepUtils.TryEvalSurfaceD1(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        double newU = u;
        double newV = v;
        PeriodicNormalization(ref newU, ref newV);

        int uindex = 0, vindex = 0;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uKnots, _uMults, u, _uPeriodic, ref uindex, out newU);
        uindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_uDeg, uindex, _uMults, _uPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vKnots, _vMults, v, _vPeriodic, ref vindex, out newV);
        vindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_vDeg, vindex, _vMults, _vPeriodic);

        ResD1 result;
        SurfaceBSplSLib.D1(newU,
                    newV,
                    uindex,
                    vindex,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out result.Point,
                    out result.D1U,
                    out result.D1V);
        return result;
    }

    /// <summary>The point and derivatives up to the second order at (U, V).</summary>
    public sealed override ResD2 EvalD2(double u, double v)
    {
        ResD2 evalRepResult;
        if (RepUtils.TryEvalSurfaceD2(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        double newU = u;
        double newV = v;
        PeriodicNormalization(ref newU, ref newV);

        int uindex = 0, vindex = 0;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uKnots, _uMults, u, _uPeriodic, ref uindex, out newU);
        uindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_uDeg, uindex, _uMults, _uPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vKnots, _vMults, v, _vPeriodic, ref vindex, out newV);
        vindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_vDeg, vindex, _vMults, _vPeriodic);

        ResD2 result;
        SurfaceBSplSLib.D2(newU,
                    newV,
                    uindex,
                    vindex,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out result.Point,
                    out result.D1U,
                    out result.D1V,
                    out result.D2U,
                    out result.D2V,
                    out result.D2UV);
        return result;
    }

    /// <summary>The point and derivatives up to the third order at (U, V).</summary>
    public sealed override ResD3 EvalD3(double u, double v)
    {
        ResD3 evalRepResult;
        if (RepUtils.TryEvalSurfaceD3(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        ResD3 result;
        SurfaceBSplSLib.D3(u,
                    v,
                    0,
                    0,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out result.Point,
                    out result.D1U,
                    out result.D1V,
                    out result.D2U,
                    out result.D2V,
                    out result.D2UV,
                    out result.D3U,
                    out result.D3V,
                    out result.D3UUV,
                    out result.D3UVV);
        return result;
    }

    /// <summary>The derivative of order Nu in U and Nv in V at (U, V).</summary>
    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public sealed override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new UndefinedDerivative3DException();
        }

        Vector3D evalRepResult;
        if (RepUtils.TryEvalSurfaceDN(_evalRep, u, v, nu, nv, out evalRepResult))
        {
            return evalRepResult;
        }

        Vector3D vn;
        SurfaceBSplSLib.DN(u,
                    v,
                    nu,
                    nv,
                    0,
                    0,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out vn);
        return vn;
    }

    /// <summary>The point at (U, V), evaluated on the spans between the knots FromUK1-ToUK2 and FromVK1-ToVK2.</summary>
    public Point3D LocalValue(double u, double v, int fromUK1, int toUK2, int fromVK1, int toVK2)
    {
        Point3D p;
        LocalD0(u, v, fromUK1, toUK2, fromVK1, toVK2, out p);
        return p;
    }

    /// <summary>
    /// The point at (U, V), evaluated on the spans between the flat knots FromUK1 and ToUK2 in U, FromVK1 and
    /// ToVK2 in V (useful at a knot of a C0 surface).
    /// </summary>
    /// <exception cref="DomainErrorException">FromUK1 == ToUK2 or FromVK1 == ToVK2.</exception>
    public void LocalD0(double U, double V, int fromUK1, int toUK2, int fromVK1, int toVK2, out Point3D p)
    {
        Standard_DomainError_Raise_if(fromUK1 == toUK2 || fromVK1 == toVK2, "Geom_BSplineSurface::LocalD0");

        double u, v;
        int uindex, vindex;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uFlatKnots, U, _uPeriodic, fromUK1, toUK2, out uindex, out u);
        uindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_uDeg, uindex, _uMults, _uPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vFlatKnots, V, _vPeriodic, fromVK1, toVK2, out vindex, out v);
        vindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_vDeg, vindex, _vMults, _vPeriodic);

        SurfaceBSplSLib.D0(u,
                    v,
                    uindex,
                    vindex,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out p);
    }

    /// <summary>The point and first derivatives at (U, V), evaluated on the given spans (see <see cref="LocalD0"/>).</summary>
    public void LocalD1(double U,
                        double V,
                        int fromUK1,
                        int toUK2,
                        int fromVK1,
                        int toVK2,
                        out Point3D p,
                        out Vector3D d1U,
                        out Vector3D d1V)
    {
        Standard_DomainError_Raise_if(fromUK1 == toUK2 || fromVK1 == toVK2, "Geom_BSplineSurface::LocalD1");

        double u, v;
        int uindex, vindex;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uFlatKnots, U, _uPeriodic, fromUK1, toUK2, out uindex, out u);
        uindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_uDeg, uindex, _uMults, _uPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vFlatKnots, V, _vPeriodic, fromVK1, toVK2, out vindex, out v);
        vindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_vDeg, vindex, _vMults, _vPeriodic);

        SurfaceBSplSLib.D1(u,
                    v,
                    uindex,
                    vindex,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out p,
                    out d1U,
                    out d1V);
    }

    /// <summary>The point and derivatives up to the second order, evaluated on the given spans.</summary>
    public void LocalD2(double U,
                        double V,
                        int fromUK1,
                        int toUK2,
                        int fromVK1,
                        int toVK2,
                        out Point3D p,
                        out Vector3D d1U,
                        out Vector3D d1V,
                        out Vector3D d2U,
                        out Vector3D d2V,
                        out Vector3D d2UV)
    {
        Standard_DomainError_Raise_if(fromUK1 == toUK2 || fromVK1 == toVK2, "Geom_BSplineSurface::LocalD2");

        double u, v;
        int uindex, vindex;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uFlatKnots, U, _uPeriodic, fromUK1, toUK2, out uindex, out u);
        uindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_uDeg, uindex, _uMults, _uPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vFlatKnots, V, _vPeriodic, fromVK1, toVK2, out vindex, out v);
        vindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_vDeg, vindex, _vMults, _vPeriodic);

        SurfaceBSplSLib.D2(u,
                    v,
                    uindex,
                    vindex,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out p,
                    out d1U,
                    out d1V,
                    out d2U,
                    out d2V,
                    out d2UV);
    }

    /// <summary>The point and derivatives up to the third order, evaluated on the given spans.</summary>
    public void LocalD3(double U,
                        double V,
                        int fromUK1,
                        int toUK2,
                        int fromVK1,
                        int toVK2,
                        out Point3D p,
                        out Vector3D d1U,
                        out Vector3D d1V,
                        out Vector3D d2U,
                        out Vector3D d2V,
                        out Vector3D d2UV,
                        out Vector3D d3U,
                        out Vector3D d3V,
                        out Vector3D d3UUV,
                        out Vector3D d3UVV)
    {
        Standard_DomainError_Raise_if(fromUK1 == toUK2 || fromVK1 == toVK2, "Geom_BSplineSurface::LocalD3");

        double u, v;
        int uindex, vindex;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uFlatKnots, U, _uPeriodic, fromUK1, toUK2, out uindex, out u);
        uindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_uDeg, uindex, _uMults, _uPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vFlatKnots, V, _vPeriodic, fromVK1, toVK2, out vindex, out v);
        vindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_vDeg, vindex, _vMults, _vPeriodic);

        SurfaceBSplSLib.D3(u,
                    v,
                    uindex,
                    vindex,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out p,
                    out d1U,
                    out d1V,
                    out d2U,
                    out d2V,
                    out d2UV,
                    out d3U,
                    out d3V,
                    out d3UUV,
                    out d3UVV);
    }

    /// <summary>The derivative of order Nu in U and Nv in V, evaluated on the given spans.</summary>
    public Vector3D LocalDN(double U, double V, int fromUK1, int toUK2, int fromVK1, int toVK2, int nu, int nv)
    {
        Standard_DomainError_Raise_if(fromUK1 == toUK2 || fromVK1 == toVK2, "Geom_BSplineSurface::LocalDN");

        double u, v;
        int uindex, vindex;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_uDeg, _uFlatKnots, U, _uPeriodic, fromUK1, toUK2, out uindex, out u);
        uindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_uDeg, uindex, _uMults, _uPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_vDeg, _vFlatKnots, V, _vPeriodic, fromVK1, toVK2, out vindex, out v);
        vindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_vDeg, vindex, _vMults, _vPeriodic);

        Vector3D vn;
        SurfaceBSplSLib.DN(u,
                    v,
                    nu,
                    nv,
                    uindex,
                    vindex,
                    _poles,
                    InternalWeights(),
                    _uFlatKnots,
                    _vFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    _uDeg,
                    _vDeg,
                    _uRational,
                    _vRational,
                    _uPeriodic,
                    _vPeriodic,
                    out vn);
        return vn;
    }

    /// <summary>The pole (UIndex, VIndex).</summary>
    /// <exception cref="OutOfRangeException">An index out of the pole grid.</exception>
    public Point3D Pole(int uIndex, int vIndex)
    {
        Standard_OutOfRange_Raise_if(uIndex < 1 || uIndex > _poles.ColLength() || vIndex < 1
                                         || vIndex > _poles.RowLength(),
                                     " ");
        return _poles.Value(uIndex, vIndex);
    }

    /// <summary>Copies the poles into P (deprecated in OCCT: use <see cref="Poles()"/>).</summary>
    /// <exception cref="DimensionErrorException">P is not NbUPoles x NbVPoles.</exception>
    internal void Poles(NCollection_Array2<Point3D> p)
    {
        Standard_DimensionError_Raise_if(p.ColLength() != _poles.ColLength() || p.RowLength() != _poles.RowLength(),
                                         " ");
        p.Assign(_poles);
    }

    /// <summary>The poles, NbUPoles rows by NbVPoles columns, indexed from 0 (a copy).</summary>
    public Grid<Point3D> Poles() => _poles.ToGrid();

    /// <summary>The U iso-curve (a B-spline curve in V) at U.</summary>
    public sealed override Curve3D UIso(double u)
    {
        NCollection_Array1<Point3D> cpoles = new NCollection_Array1<Point3D>(1, _poles.RowLength());
        NCollection_Array1<double> cweights = new NCollection_Array1<double>(1, _poles.RowLength());

        BSplineCurve3D c;

        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.Iso(u, true, _poles, InternalWeights(), _uFlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), _uDeg, _uPeriodic,
                         cpoles, cweights);
            c = new BSplineCurve3D(cpoles, cweights, _vKnots, _vMults, _vDeg, _vPeriodic);
        }
        else
        {
            SurfaceBSplSLib.Iso(u, true, _poles, SurfaceBSplSLib.NoWeights(), _uFlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), _uDeg,
                         _uPeriodic, cpoles, cweights);
            c = new BSplineCurve3D(cpoles, _vKnots, _vMults, _vDeg, _vPeriodic);
        }
        return c;
    }

    /// <summary>
    /// The U iso-curve at U; with CheckRational, the curve is non-rational when its weights are all equal.
    /// </summary>
    public Curve3D UIso(double u, bool checkRational)
    {
        NCollection_Array1<Point3D> cpoles = new NCollection_Array1<Point3D>(1, _poles.RowLength());
        NCollection_Array1<double> cweights = new NCollection_Array1<double>(1, _poles.RowLength());

        BSplineCurve3D c;

        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.Iso(u, true, _poles, InternalWeights(), _uFlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), _uDeg, _uPeriodic,
                         cpoles, cweights);
            c = new BSplineCurve3D(cpoles, cweights, _vKnots, _vMults, _vDeg, _vPeriodic,
                                      checkRational);
        }
        else
        {
            SurfaceBSplSLib.Iso(u, true, _poles, SurfaceBSplSLib.NoWeights(), _uFlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), _uDeg,
                         _uPeriodic, cpoles, cweights);
            c = new BSplineCurve3D(cpoles, _vKnots, _vMults, _vDeg, _vPeriodic);
        }
        return c;
    }

    /// <summary>The value of the U knot UIndex.</summary>
    /// <exception cref="OutOfRangeException">UIndex out of [1, NbUKnots].</exception>
    public double UKnot(int uIndex)
    {
        Standard_OutOfRange_Raise_if(uIndex < 1 || uIndex > _uKnots.Length(), " ");
        return _uKnots.Value(uIndex);
    }

    /// <summary>The value of the V knot VIndex.</summary>
    /// <exception cref="OutOfRangeException">VIndex out of [1, NbVKnots].</exception>
    public double VKnot(int vIndex)
    {
        Standard_OutOfRange_Raise_if(vIndex < 1 || vIndex > _vKnots.Length(), " ");
        return _vKnots.Value(vIndex);
    }

    /// <summary>Copies the U knots into Ku (deprecated in OCCT: use <see cref="UKnots()"/>).</summary>
    public void UKnots(NCollection_Array1<double> ku)
    {
        Standard_DimensionError_Raise_if(ku.Length() != _uKnots.Length(), " ");
        ku.Assign(_uKnots);
    }

    /// <summary>The U knots (without repetition).</summary>
    public IReadOnlyList<double> UKnots() => _uKnots.ToPackedList();

    /// <summary>Copies the V knots into Kv (deprecated in OCCT: use <see cref="VKnots()"/>).</summary>
    public void VKnots(NCollection_Array1<double> kv)
    {
        Standard_DimensionError_Raise_if(kv.Length() != _vKnots.Length(), " ");
        kv.Assign(_vKnots);
    }

    /// <summary>The V knots (without repetition).</summary>
    public IReadOnlyList<double> VKnots() => _vKnots.ToPackedList();

    /// <summary>Copies the U flat knots into Ku (deprecated in OCCT: use <see cref="UKnotSequence()"/>).</summary>
    public void UKnotSequence(NCollection_Array1<double> ku)
    {
        Standard_DimensionError_Raise_if(ku.Length() != _uFlatKnots.Length(), " ");
        ku.Assign(_uFlatKnots);
    }

    /// <summary>The U knots repeated by their multiplicities (the flat knots).</summary>
    public IReadOnlyList<double> UKnotSequence() => _uFlatKnots.ToPackedList();

    /// <summary>Copies the V flat knots into Kv (deprecated in OCCT: use <see cref="VKnotSequence()"/>).</summary>
    public void VKnotSequence(NCollection_Array1<double> kv)
    {
        Standard_DimensionError_Raise_if(kv.Length() != _vFlatKnots.Length(), " ");
        kv.Assign(_vFlatKnots);
    }

    /// <summary>The V knots repeated by their multiplicities (the flat knots).</summary>
    public IReadOnlyList<double> VKnotSequence() => _vFlatKnots.ToPackedList();

    /// <summary>The multiplicity of the U knot UIndex.</summary>
    public int UMultiplicity(int uIndex)
    {
        Standard_OutOfRange_Raise_if(uIndex < 1 || uIndex > _uMults.Length(), " ");
        return _uMults.Value(uIndex);
    }

    /// <summary>Copies the U multiplicities into Mu (deprecated in OCCT: use <see cref="UMultiplicities()"/>).</summary>
    public void UMultiplicities(NCollection_Array1<int> mu)
    {
        Standard_DimensionError_Raise_if(mu.Length() != _uMults.Length(), " ");
        mu.Assign(_uMults);
    }

    /// <summary>The multiplicities of the U knots.</summary>
    public IReadOnlyList<int> UMultiplicities() => _uMults.ToPackedList();

    /// <summary>The V iso-curve (a B-spline curve in U) at V.</summary>
    public sealed override Curve3D VIso(double v)
    {
        NCollection_Array1<Point3D> cpoles = new NCollection_Array1<Point3D>(1, _poles.ColLength());
        NCollection_Array1<double> cweights = new NCollection_Array1<double>(1, _poles.ColLength());

        BSplineCurve3D c;

        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.Iso(v, false, _poles, InternalWeights(), _vFlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), _vDeg, _vPeriodic,
                         cpoles, cweights);
            c = new BSplineCurve3D(cpoles, cweights, _uKnots, _uMults, _uDeg, _uPeriodic);
        }
        else
        {
            SurfaceBSplSLib.Iso(v, false, _poles, SurfaceBSplSLib.NoWeights(), _vFlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), _vDeg,
                         _vPeriodic, cpoles, cweights);
            c = new BSplineCurve3D(cpoles, _uKnots, _uMults, _uDeg, _uPeriodic);
        }
        return c;
    }

    /// <summary>
    /// The V iso-curve at V; with CheckRational, the curve is non-rational when its weights are all equal.
    /// </summary>
    public Curve3D VIso(double v, bool checkRational)
    {
        NCollection_Array1<Point3D> cpoles = new NCollection_Array1<Point3D>(1, _poles.ColLength());
        NCollection_Array1<double> cweights = new NCollection_Array1<double>(1, _poles.ColLength());

        BSplineCurve3D c;

        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.Iso(v, false, _poles, InternalWeights(), _vFlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), _vDeg, _vPeriodic,
                         cpoles, cweights);
            c = new BSplineCurve3D(cpoles, cweights, _uKnots, _uMults, _uDeg, _uPeriodic,
                                      checkRational);
        }
        else
        {
            SurfaceBSplSLib.Iso(v, false, _poles, SurfaceBSplSLib.NoWeights(), _vFlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), _vDeg,
                         _vPeriodic, cpoles, cweights);
            c = new BSplineCurve3D(cpoles, _uKnots, _uMults, _uDeg, _uPeriodic);
        }
        return c;
    }

    /// <summary>The multiplicity of the V knot VIndex.</summary>
    public int VMultiplicity(int vIndex)
    {
        Standard_OutOfRange_Raise_if(vIndex < 1 || vIndex > _vMults.Length(), " ");
        return _vMults.Value(vIndex);
    }

    /// <summary>Copies the V multiplicities into Mv (deprecated in OCCT: use <see cref="VMultiplicities()"/>).</summary>
    public void VMultiplicities(NCollection_Array1<int> mv)
    {
        Standard_DimensionError_Raise_if(mv.Length() != _vMults.Length(), " ");
        mv.Assign(_vMults);
    }

    /// <summary>The multiplicities of the V knots.</summary>
    public IReadOnlyList<int> VMultiplicities() => _vMults.ToPackedList();

    /// <summary>The weight of the pole (UIndex, VIndex): 1 for a non-rational surface.</summary>
    public double Weight(int uIndex, int vIndex)
    {
        if (!(_uRational || _vRational))
        {
            return 1.0;
        }
        Standard_OutOfRange_Raise_if(uIndex < 1 || uIndex > _weights.ColLength() || vIndex < 1
                                         || vIndex > _weights.RowLength(),
                                     " ");
        return _weights.Value(uIndex, vIndex);
    }

    /// <summary>Copies the weights into W, all 1 for a non-rational surface (deprecated in OCCT).</summary>
    /// <exception cref="DimensionErrorException">W is not NbUPoles x NbVPoles.</exception>
    internal void Weights(NCollection_Array2<double> w)
    {
        if (!(_uRational || _vRational))
        {
            Standard_DimensionError_Raise_if(w.ColLength() != _poles.ColLength()
                                                 || w.RowLength() != _poles.RowLength(),
                                             " ");
            w.Init(1.0);
            return;
        }
        Standard_DimensionError_Raise_if(w.ColLength() != _weights.ColLength() || w.RowLength() != _weights.RowLength(),
                                         " ");
        w.Assign(_weights);
    }

    /// <summary>The weights, indexed from 0 as <see cref="Poles()"/> (a copy), or null if the surface is non-rational
    /// in both directions.</summary>
    public Grid<double>? Weights() => (_uRational || _vRational) ? _weights.ToGrid() : null;

    /// <summary>The weights, all 1 for a non-rational surface, indexed from 0 as <see cref="Poles()"/> (a copy).</summary>
    public Grid<double> WeightsArray() => _weights.ToGrid();

    /// <summary>Transforms the poles by T.</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        ClearEvalRepresentation();
        for (int j = _poles.LowerCol(); j <= _poles.UpperCol(); j++)
        {
            for (int i = _poles.LowerRow(); i <= _poles.UpperRow(); i++)
            {
                _poles[i, j] = _poles[i, j].Transformed(t);
            }
        }
        _maxDerivInvOk = false;
    }

    /// <summary>
    /// Makes the surface periodic in U: the knots outside [FirstUKnotIndex, LastUKnotIndex] are dropped and the
    /// poles trimmed to the periodic count.
    /// </summary>
    internal void SetUPeriodic()
    {
        CheckMutable();
        ClearEvalRepresentation();
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        NCollection_Array1<double> cknots = new NCollection_Array1<double>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cknots[k - first + 1] = _uKnots[k];
        }
        _uKnots = cknots;

        NCollection_Array1<int> cmults = new NCollection_Array1<int>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cmults[k - first + 1] = _uMults[k];
        }
        cmults[1] = cmults[cmults.Upper()] = Min(_uDeg, Max(cmults[1], cmults[cmults.Upper()]));
        _uMults = cmults;

        // compute new number of poles;
        int nbp = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(_uDeg, true, _uMults);

        _poles.ResizeWithTrim(1, nbp, _poles.LowerCol(), _poles.UpperCol(), true);
        if (_uRational || _vRational)
        {
            _weights.ResizeWithTrim(1, nbp, _weights.LowerCol(), _weights.UpperCol(), true);
        }
        else
        {
            _weights = SurfaceBSplSLib.UnitWeights(nbp, _poles.RowLength());
        }

        _uPeriodic = true;

        _maxDerivInvOk = false;
        UpdateUKnots();
    }

    /// <summary>Makes the surface periodic in V (see <see cref="SetUPeriodic"/>).</summary>
    internal void SetVPeriodic()
    {
        CheckMutable();
        ClearEvalRepresentation();
        int first = FirstVKnotIndex();
        int last = LastVKnotIndex();

        NCollection_Array1<double> cknots = new NCollection_Array1<double>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cknots[k - first + 1] = _vKnots[k];
        }
        _vKnots = cknots;

        NCollection_Array1<int> cmults = new NCollection_Array1<int>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cmults[k - first + 1] = _vMults[k];
        }
        cmults[1] = cmults[cmults.Upper()] = Min(_vDeg, Max(cmults[1], cmults[cmults.Upper()]));
        _vMults = cmults;

        // compute new number of poles;
        int nbp = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(_vDeg, true, _vMults);

        _poles.ResizeWithTrim(_poles.LowerRow(), _poles.UpperRow(), 1, nbp, true);
        if (_uRational || _vRational)
        {
            _weights.ResizeWithTrim(_weights.LowerRow(), _weights.UpperRow(), 1, nbp, true);
        }
        else
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), nbp);
        }

        _vPeriodic = true;

        _maxDerivInvOk = false;
        UpdateVKnots();
    }

    /// <summary>Makes the U knot Index the origin of a U-periodic surface.</summary>
    /// <exception cref="NoSuchObjectException">The surface is not U-periodic.</exception>
    /// <exception cref="DomainErrorException">Index out of [FirstUKnotIndex, LastUKnotIndex].</exception>
    internal void SetUOrigin(int Index)
    {
        CheckMutable();
        if (!_uPeriodic)
        {
            throw new NoSuchObjectException("Geom_BSplineSurface::SetUOrigin: surface is not U periodic");
        }

        ClearEvalRepresentation();
        int i, j, k;
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        if ((Index < first) || (Index > last))
        {
            throw new DomainErrorException("Geom_BSplineCurve::SetUOrigin: Index out of range");
        }

        int nbknots = _uKnots.Length();
        int nbpoles = _poles.ColLength();

        NCollection_Array1<double> newknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> newmults = new NCollection_Array1<int>(1, nbknots);

        // set the knots and mults
        double period = _uKnots.Value(last) - _uKnots.Value(first);
        k = 1;
        for (i = Index; i <= last; i++)
        {
            newknots[k] = _uKnots.Value(i);
            newmults[k] = _uMults.Value(i);
            k++;
        }
        for (i = first + 1; i <= Index; i++)
        {
            newknots[k] = _uKnots.Value(i) + period;
            newmults[k] = _uMults.Value(i);
            k++;
        }

        int index = 1;
        for (i = first + 1; i <= Index; i++)
        {
            index += _uMults.Value(i);
        }

        // set the poles and weights
        int nbvp = _poles.RowLength();
        NCollection_Array2<Point3D> newpoles = new NCollection_Array2<Point3D>(1, nbpoles, 1, nbvp);
        first = _poles.LowerRow();
        last = _poles.UpperRow();
        if (_uRational || _vRational)
        {
            NCollection_Array2<double> newweights = new NCollection_Array2<double>(1, nbpoles, 1, nbvp);
            k = 1;
            for (i = index; i <= last; i++)
            {
                for (j = 1; j <= nbvp; j++)
                {
                    newpoles[k, j] = _poles.Value(i, j);
                    newweights[k, j] = _weights.Value(i, j);
                }
                k++;
            }
            for (i = first; i < index; i++)
            {
                for (j = 1; j <= nbvp; j++)
                {
                    newpoles[k, j] = _poles.Value(i, j);
                    newweights[k, j] = _weights.Value(i, j);
                }
                k++;
            }
            _weights = newweights;
        }
        else
        {
            k = 1;
            for (i = index; i <= last; i++)
            {
                for (j = 1; j <= nbvp; j++)
                {
                    newpoles[k, j] = _poles.Value(i, j);
                }
                k++;
            }
            for (i = first; i < index; i++)
            {
                for (j = 1; j <= nbvp; j++)
                {
                    newpoles[k, j] = _poles.Value(i, j);
                }
                k++;
            }
            _weights = SurfaceBSplSLib.UnitWeights(nbpoles, nbvp);
        }

        _poles = newpoles;
        _uKnots = newknots;
        _uMults = newmults;
        UpdateUKnots();
    }

    /// <summary>Makes the V knot Index the origin of a V-periodic surface.</summary>
    /// <exception cref="NoSuchObjectException">The surface is not V-periodic.</exception>
    /// <exception cref="DomainErrorException">Index out of [FirstVKnotIndex, LastVKnotIndex].</exception>
    internal void SetVOrigin(int Index)
    {
        CheckMutable();
        if (!_vPeriodic)
        {
            throw new NoSuchObjectException("Geom_BSplineSurface::SetVOrigin: surface is not V periodic");
        }

        ClearEvalRepresentation();
        int i, j, k;
        int first = FirstVKnotIndex();
        int last = LastVKnotIndex();

        if ((Index < first) || (Index > last))
        {
            throw new DomainErrorException("Geom_BSplineCurve::SetVOrigin: Index out of range");
        }

        int nbknots = _vKnots.Length();
        int nbpoles = _poles.RowLength();

        NCollection_Array1<double> newknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> newmults = new NCollection_Array1<int>(1, nbknots);

        // set the knots and mults
        double period = _vKnots.Value(last) - _vKnots.Value(first);
        k = 1;
        for (i = Index; i <= last; i++)
        {
            newknots[k] = _vKnots.Value(i);
            newmults[k] = _vMults.Value(i);
            k++;
        }
        for (i = first + 1; i <= Index; i++)
        {
            newknots[k] = _vKnots.Value(i) + period;
            newmults[k] = _vMults.Value(i);
            k++;
        }

        int index = 1;
        for (i = first + 1; i <= Index; i++)
        {
            index += _vMults.Value(i);
        }

        // set the poles and weights
        int nbup = _poles.ColLength();
        NCollection_Array2<Point3D> newpoles = new NCollection_Array2<Point3D>(1, nbup, 1, nbpoles);
        first = _poles.LowerCol();
        last = _poles.UpperCol();
        if (_uRational || _vRational)
        {
            NCollection_Array2<double> newweights = new NCollection_Array2<double>(1, nbup, 1, nbpoles);
            k = 1;
            for (j = index; j <= last; j++)
            {
                for (i = 1; i <= nbup; i++)
                {
                    newpoles[i, k] = _poles.Value(i, j);
                    newweights[i, k] = _weights.Value(i, j);
                }
                k++;
            }
            for (j = first; j < index; j++)
            {
                for (i = 1; i <= nbup; i++)
                {
                    newpoles[i, k] = _poles.Value(i, j);
                    newweights[i, k] = _weights.Value(i, j);
                }
                k++;
            }
            _weights = newweights;
        }
        else
        {
            k = 1;
            for (j = index; j <= last; j++)
            {
                for (i = 1; i <= nbup; i++)
                {
                    newpoles[i, k] = _poles.Value(i, j);
                }
                k++;
            }
            for (j = first; j < index; j++)
            {
                for (i = 1; i <= nbup; i++)
                {
                    newpoles[i, k] = _poles.Value(i, j);
                }
                k++;
            }
            _weights = SurfaceBSplSLib.UnitWeights(nbup, nbpoles);
        }

        _poles = newpoles;
        _vKnots = newknots;
        _vMults = newmults;
        UpdateVKnots();
    }

    /// <summary>Makes a U-periodic surface non-periodic (the same geometry); does nothing otherwise.</summary>
    internal void SetUNotPeriodic()
    {
        CheckMutable();
        if (_uPeriodic)
        {
            ClearEvalRepresentation();
            int nbKnots, nbPoles;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(_uDeg, _uMults, out nbKnots, out nbPoles);

            NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, nbPoles, 1, _poles.RowLength());

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbKnots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbKnots);

            if (_uRational || _vRational)
            {
                NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbPoles, 1, _poles.RowLength());
                nweights.Init(0.0);

                SurfaceBSplSLib.Unperiodize(true,
                                     _uDeg,
                                     _uMults,
                                     _uKnots,
                                     _poles,
                                     InternalWeights(),
                                     nmults,
                                     nknots,
                                     npoles,
                                     nweights);
                _weights = nweights;
            }
            else
            {
                SurfaceBSplSLib.Unperiodize(true,
                                     _uDeg,
                                     _uMults,
                                     _uKnots,
                                     _poles,
                                     SurfaceBSplSLib.NoWeights(),
                                     nmults,
                                     nknots,
                                     npoles,
                                     SurfaceBSplSLib.NoWeights());
                _weights = SurfaceBSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
            }
            _poles = npoles;
            _uMults = nmults;
            _uKnots = nknots;
            _uPeriodic = false;

            _maxDerivInvOk = false;
            UpdateUKnots();
        }
    }

    /// <summary>Makes a V-periodic surface non-periodic (the same geometry); does nothing otherwise.</summary>
    internal void SetVNotPeriodic()
    {
        CheckMutable();
        if (_vPeriodic)
        {
            ClearEvalRepresentation();
            int nbKnots, nbPoles;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(_vDeg, _vMults, out nbKnots, out nbPoles);

            NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, _poles.ColLength(), 1, nbPoles);

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbKnots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbKnots);

            if (_uRational || _vRational)
            {
                NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, _poles.ColLength(), 1, nbPoles);
                nweights.Init(0.0);

                SurfaceBSplSLib.Unperiodize(false,
                                     _vDeg,
                                     _vMults,
                                     _vKnots,
                                     _poles,
                                     InternalWeights(),
                                     nmults,
                                     nknots,
                                     npoles,
                                     nweights);
                _weights = nweights;
            }
            else
            {
                SurfaceBSplSLib.Unperiodize(false,
                                     _vDeg,
                                     _vMults,
                                     _vKnots,
                                     _poles,
                                     SurfaceBSplSLib.NoWeights(),
                                     nmults,
                                     nknots,
                                     npoles,
                                     SurfaceBSplSLib.NoWeights());
                _weights = SurfaceBSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
            }
            _poles = npoles;
            _vMults = nmults;
            _vKnots = nknots;
            _vPeriodic = false;

            _maxDerivInvOk = false;
            UpdateVKnots();
        }
    }

    /// <summary>
    /// True if the surface is closed in U: periodic, or with equal first and last U iso-curves (within
    /// Precision::Confusion()).
    /// </summary>
    public sealed override bool IsUClosed
    {
        get
        {
            if (_uPeriodic)
            {
                return true;
            }

            double u1, u2, v1, v2;
            Bounds(out u1, out u2, out v1, out v2);
            Curve3D cuf = UIso(u1);
            Curve3D cul = UIso(u2);
            if (cuf is null || cul is null)
            {
                return false;
            }
            BSplineCurve3D? bsF = cuf as BSplineCurve3D;
            BSplineCurve3D? bsL = cul as BSplineCurve3D;
            return (bsF is not null && bsL is not null && bsF.IsEqual(bsL, global::Kerf.Engine.Foundation.Precision.Confusion()));
        }
    }

    /// <summary>
    /// True if the surface is closed in V: periodic, or with equal first and last V iso-curves (within
    /// Precision::Confusion()).
    /// </summary>
    public sealed override bool IsVClosed
    {
        get
        {
            if (_vPeriodic)
            {
                return true;
            }

            double u1, u2, v1, v2;
            Bounds(out u1, out u2, out v1, out v2);
            Curve3D cvf = VIso(v1);
            Curve3D cvl = VIso(v2);
            if (cvf is null || cvl is null)
            {
                return false;
            }
            BSplineCurve3D? bsF = cvf as BSplineCurve3D;
            BSplineCurve3D? bsL = cvl as BSplineCurve3D;
            return (bsF is not null && bsL is not null && bsF.IsEqual(bsL, global::Kerf.Engine.Foundation.Precision.Confusion()));
        }
    }

    /// <summary>True if the surface is periodic in U.</summary>
    public sealed override bool IsUPeriodic => _uPeriodic;

    /// <summary>True if the surface is periodic in V.</summary>
    public sealed override bool IsVPeriodic => _vPeriodic;

    /// <summary>The index of the first U knot of the parametric range (1 when periodic).</summary>
    public int FirstUKnotIndex()
    {
        if (_uPeriodic)
        {
            return 1;
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(_uDeg, _uMults);
        }
    }

    /// <summary>The index of the first V knot of the parametric range (1 when periodic).</summary>
    public int FirstVKnotIndex()
    {
        if (_vPeriodic)
        {
            return 1;
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(_vDeg, _vMults);
        }
    }

    /// <summary>The index of the last U knot of the parametric range (NbUKnots when periodic).</summary>
    public int LastUKnotIndex()
    {
        if (_uPeriodic)
        {
            return _uKnots.Length();
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(_uDeg, _uMults);
        }
    }

    /// <summary>The index of the last V knot of the parametric range (NbVKnots when periodic).</summary>
    public int LastVKnotIndex()
    {
        if (_vPeriodic)
        {
            return _vKnots.Length();
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(_vDeg, _vMults);
        }
    }

    /// <summary>
    /// Locates U among the U knots (with repetition if WithKnotRepetition): I1 = I2 on a knot (within
    /// ParametricTolerance), else UKnots(I1) &lt; U &lt; UKnots(I2); I1 = 0 below the first knot, I2 = Length + 1
    /// above the last.
    /// </summary>
    public void LocateU(double u, double parametricTolerance, out int i1, out int i2, bool withKnotRepetition = false)
    {
        double newU = u, vbid = _vKnots.Value(1);
        NCollection_Array1<double> theKnots = withKnotRepetition ? _uFlatKnots : _uKnots;

        PeriodicNormalization(ref newU, ref vbid); // Handle periodicity

        double uFirst = theKnots[1];
        double uLast = theKnots[theKnots.Length()];
        double pParametricTolerance = Math.Abs(parametricTolerance);
        if (Math.Abs(newU - uFirst) <= pParametricTolerance)
        {
            i1 = i2 = 1;
        }
        else if (Math.Abs(newU - uLast) <= pParametricTolerance)
        {
            i1 = i2 = theKnots.Length();
        }
        else if (newU < uFirst)
        {
            i2 = 1;
            i1 = 0;
        }
        else if (newU > uLast)
        {
            i1 = theKnots.Length();
            i2 = i1 + 1;
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(theKnots, newU, out i1);
            i1 = Max(Min(i1, theKnots.Upper()), theKnots.Lower());
            while (i1 + 1 <= theKnots.Upper() && Math.Abs(theKnots[i1 + 1] - newU) <= pParametricTolerance)
            {
                i1++;
            }
            if (Math.Abs(theKnots[i1] - newU) <= pParametricTolerance)
            {
                i2 = i1;
            }
            else
            {
                i2 = i1 + 1;
            }
        }
    }

    /// <summary>Locates V among the V knots (see <see cref="LocateU"/>).</summary>
    public void LocateV(double v, double parametricTolerance, out int i1, out int i2, bool withKnotRepetition = false)
    {
        double newV = v, ubid = _uKnots.Value(1);
        NCollection_Array1<double> theKnots = withKnotRepetition ? _vFlatKnots : _vKnots;

        PeriodicNormalization(ref ubid, ref newV); // Handle periodicity

        double vFirst = theKnots[1];
        double vLast = theKnots[theKnots.Length()];
        double pParametricTolerance = Math.Abs(parametricTolerance);
        if (Math.Abs(newV - vFirst) <= pParametricTolerance)
        {
            i1 = i2 = 1;
        }
        else if (Math.Abs(newV - vLast) <= pParametricTolerance)
        {
            i1 = i2 = theKnots.Length();
        }
        else if (newV < vFirst - pParametricTolerance)
        {
            i2 = 1;
            i1 = 0;
        }
        else if (newV > vLast + pParametricTolerance)
        {
            i1 = theKnots.Length();
            i2 = i1 + 1;
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(theKnots, newV, out i1);
            i1 = Max(Min(i1, theKnots.Upper()), theKnots.Lower());
            while (i1 + 1 <= theKnots.Upper() && Math.Abs(theKnots[i1 + 1] - newV) <= pParametricTolerance)
            {
                i1++;
            }
            if (Math.Abs(theKnots[i1] - newV) <= pParametricTolerance)
            {
                i2 = i1;
            }
            else
            {
                i2 = i1 + 1;
            }
        }
    }

    /// <summary>Reverses the U direction; the U knots become UKnots(1) + UKnots(NbUKnots) - UKnots(i).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_uMults);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_uKnots);
        int last;
        if (_uPeriodic)
        {
            last = _uFlatKnots.Upper() - _uDeg - 1;
        }
        else
        {
            last = _poles.UpperRow();
        }
        SurfaceBSplSLib.Reverse(_poles, last, true);
        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.Reverse(_weights, last, true);
        }
        UpdateUKnots();
    }

    /// <summary>UKnots(1) + UKnots(NbUKnots) - U.</summary>
    public sealed override double UReversedParameter(double u) => (_uKnots.Value(1) + _uKnots.Value(_uKnots.Length()) - u);

    /// <summary>Reverses the V direction; the V knots become VKnots(1) + VKnots(NbVKnots) - VKnots(i).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_vMults);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_vKnots);
        int last;
        if (_vPeriodic)
        {
            last = _vFlatKnots.Upper() - _vDeg - 1;
        }
        else
        {
            last = _poles.UpperCol();
        }
        SurfaceBSplSLib.Reverse(_poles, last, false);
        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.Reverse(_weights, last, false);
        }
        UpdateVKnots();
    }

    /// <summary>VKnots(1) + VKnots(NbVKnots) - V.</summary>
    public sealed override double VReversedParameter(double v) => (_vKnots.Value(1) + _vKnots.Value(_vKnots.Length()) - v);

    /// <summary>Changes the poles of the column VIndex (rows CPoles.Lower() to Upper()).</summary>
    internal void SetPoleCol(int vIndex, NCollection_Array1<Point3D> cPoles)
    {
        CheckMutable();
        if (vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException("Geom_BSplineSurface::SetPoleCol: VIndex out of range");
        }
        if (cPoles.Lower() < 1 || cPoles.Lower() > _poles.ColLength() || cPoles.Upper() < 1
            || cPoles.Upper() > _poles.ColLength())
        {
            throw new ConstructionErrorException("Geom_BSplineSurface::SetPoleCol: invalid array dimension");
        }

        ClearEvalRepresentation();
        for (int i = cPoles.Lower(); i <= cPoles.Upper(); i++)
        {
            _poles[i + _poles.LowerRow() - 1, vIndex + _poles.LowerCol() - 1] = cPoles[i];
        }

        _maxDerivInvOk = false;
    }

    /// <summary>Changes the poles and weights of the column VIndex.</summary>
    internal void SetPoleCol(int vIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        SetPoleCol(vIndex, cPoles);
        SetWeightCol(vIndex, cPoleWeights);
    }

    /// <summary>Changes the poles of the row UIndex (columns CPoles.Lower() to Upper()).</summary>
    internal void SetPoleRow(int uIndex, NCollection_Array1<Point3D> cPoles)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _poles.ColLength())
        {
            throw new OutOfRangeException("Geom_BSplineSurface::SetPoleRow: UIndex out of range");
        }
        if (cPoles.Lower() < 1 || cPoles.Lower() > _poles.RowLength() || cPoles.Upper() < 1
            || cPoles.Upper() > _poles.RowLength())
        {
            throw new ConstructionErrorException("Geom_BSplineSurface::SetPoleRow: invalid array dimension");
        }

        ClearEvalRepresentation();
        for (int i = cPoles.Lower(); i <= cPoles.Upper(); i++)
        {
            _poles[uIndex + _poles.LowerRow() - 1, i + _poles.LowerCol() - 1] = cPoles[i];
        }

        _maxDerivInvOk = false;
    }

    /// <summary>Changes the poles and weights of the row UIndex.</summary>
    internal void SetPoleRow(int uIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        SetPoleRow(uIndex, cPoles);
        SetWeightRow(uIndex, cPoleWeights);
    }

    /// <summary>Changes the pole (UIndex, VIndex).</summary>
    /// <remarks>OCCT checks no index (a bad one throws <see cref="IndexOutOfRangeException"/> here).</remarks>
    internal void SetPole(int uIndex, int vIndex, in Point3D p)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _poles.SetValue(uIndex + _poles.LowerRow() - 1, vIndex + _poles.LowerCol() - 1, p);
        _maxDerivInvOk = false;
    }

    /// <summary>Changes the pole (UIndex, VIndex) and its weight.</summary>
    internal void SetPole(int uIndex, int vIndex, in Point3D p, double weight)
    {
        CheckMutable();
        SetWeight(uIndex, vIndex, weight);
        SetPole(uIndex, vIndex, p);
    }

    /// <summary>
    /// Moves the point at (U, V) to P by moving the poles UIndex1-UIndex2 x VIndex1-VIndex2; returns the range of
    /// poles actually moved (UFirstModifiedPole = 0 if none: P is already the point, or the indices don't
    /// influence it).
    /// </summary>
    /// <exception cref="OutOfRangeException">Indices out of the pole grid or not increasing.</exception>
    internal void MovePoint(double u,
                            double v,
                            in Point3D p,
                            int uIndex1,
                            int uIndex2,
                            int vIndex1,
                            int vIndex2,
                            out int uFirstModifiedPole,
                            out int uLastmodifiedPole,
                            out int vFirstModifiedPole,
                            out int vLastmodifiedPole)
    {
        CheckMutable();
        if (uIndex1 < 1 || uIndex1 > _poles.UpperRow() || uIndex2 < 1 || uIndex2 > _poles.UpperRow()
            || uIndex1 > uIndex2 || vIndex1 < 1 || vIndex1 > _poles.UpperCol() || vIndex2 < 1
            || vIndex2 > _poles.UpperCol() || vIndex1 > vIndex2)
        {
            throw new OutOfRangeException("Geom_BSplineSurface::MovePoint: Index and #pole mismatch");
        }

        // OCCT: on a periodic surface, BSplSLib::MovePoint indexes the stored poles (and weights) with indices from the
        // periodic flat knots, which run past the arrays (occt-issues.md; the port throws Standard_OutOfRange)
        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, _poles.UpperRow(), 1, _poles.UpperCol());
        Point3D p0;
        D0(u, v, out p0);
        Vector3D displ = new Vector3D(p0, p);
        bool rational = (_uRational || _vRational);
        SurfaceBSplSLib.MovePoint(u,
                           v,
                           displ,
                           uIndex1,
                           uIndex2,
                           vIndex1,
                           vIndex2,
                           _uDeg,
                           _vDeg,
                           rational,
                           _poles,
                           _weights,
                           _uFlatKnots,
                           _vFlatKnots,
                           out uFirstModifiedPole,
                           out uLastmodifiedPole,
                           out vFirstModifiedPole,
                           out vLastmodifiedPole,
                           npoles);
        if (uFirstModifiedPole != 0)
        {
            ClearEvalRepresentation();
            _poles = npoles;
        }
        _maxDerivInvOk = false;
    }

    /// <summary>The parametric bounds: the knots at the ends of the parametric range in each direction.</summary>
    public sealed override void Bounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = _uFlatKnots.Value(_uDeg + 1);
        u2 = _uFlatKnots.Value(_uFlatKnots.Upper() - _uDeg);
        v1 = _vFlatKnots.Value(_vDeg + 1);
        v2 = _vFlatKnots.Value(_vFlatKnots.Upper() - _vDeg);
    }

    /// <summary>The maximum degree in each direction: 25.</summary>
    public static int MaxDegree() => global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    /// <summary>True if the weights vary in the U direction.</summary>
    public bool IsURational => _uRational;

    /// <summary>True if the weights vary in the V direction.</summary>
    public bool IsVRational => _vRational;

    /// <summary>The lower of the U and V continuities.</summary>
    public sealed override Continuity Continuity() => ((_uSmooth < _vSmooth) ? _uSmooth : _vSmooth);

    /// <summary>The number of U knots.</summary>
    public int UKnotCount => _uKnots.Length();

    /// <summary>The number of poles in U (rows of the pole grid).</summary>
    public int UPoleCount => _poles.ColLength();

    /// <summary>The number of V knots.</summary>
    public int VKnotCount => _vKnots.Length();

    /// <summary>The number of poles in V (columns of the pole grid).</summary>
    public int VPoleCount => _poles.RowLength();

    /// <summary>The degree in U.</summary>
    public int UDegree() => _uDeg;

    /// <summary>The degree in V.</summary>
    public int VDegree() => _vDeg;

    /// <summary>The distribution of the U knots (uniform, quasi-uniform, piecewise Bezier, non-uniform).</summary>
    public BSplKnotDistribution UKnotDistribution() => _uKnotSet;

    /// <summary>The distribution of the V knots.</summary>
    public BSplKnotDistribution VKnotDistribution() => _vKnotSet;

    /// <summary>
    /// Inserts the U knots with the multiplicities Mults (a knot within ParametricTolerance of an existing one
    /// raises its multiplicity: by Mults(i) when Add, to Mults(i) otherwise).
    /// </summary>
    /// <exception cref="ConstructionErrorException">The insertion is invalid (BSplCLib::PrepareInsertKnots).</exception>
    internal void InsertUKnots(NCollection_Array1<double> knots,
                               NCollection_Array1<int> mults,
                               double parametricTolerance = 0.0,
                               bool add = true)
    {
        CheckMutable();
        // Check and compute new sizes
        int nbpoles, nbknots;

        if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(_uDeg,
                                         _uPeriodic,
                                         _uKnots,
                                         _uMults,
                                         knots,
                                         mults,
                                         out nbpoles,
                                         out nbknots,
                                         parametricTolerance,
                                         add))
        {
            throw new ConstructionErrorException("Geom_BSplineSurface::InsertUKnots");
        }

        if (nbpoles == _poles.ColLength())
        {
            return;
        }

        ClearEvalRepresentation();
        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, nbpoles, 1, _poles.RowLength());
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        if (_uRational || _vRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbpoles, 1, _poles.RowLength());
            SurfaceBSplSLib.InsertKnots(true,
                                 _uDeg,
                                 _uPeriodic,
                                 _poles,
                                 InternalWeights(),
                                 _uKnots,
                                 _uMults,
                                 knots,
                                 mults,
                                 npoles,
                                 nweights,
                                 nknots,
                                 nmults,
                                 parametricTolerance,
                                 add);
            _weights = nweights;
        }
        else
        {
            SurfaceBSplSLib.InsertKnots(true,
                                 _uDeg,
                                 _uPeriodic,
                                 _poles,
                                 SurfaceBSplSLib.NoWeights(),
                                 _uKnots,
                                 _uMults,
                                 knots,
                                 mults,
                                 npoles,
                                 SurfaceBSplSLib.NoWeights(),
                                 nknots,
                                 nmults,
                                 parametricTolerance,
                                 add);
            _weights = SurfaceBSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
        }

        _poles = npoles;
        _uKnots = nknots;
        _uMults = nmults;
        UpdateUKnots();
    }

    /// <summary>Inserts the V knots (see <see cref="InsertUKnots"/>).</summary>
    /// <exception cref="ConstructionErrorException">The insertion is invalid (BSplCLib::PrepareInsertKnots).</exception>
    internal void InsertVKnots(NCollection_Array1<double> knots,
                               NCollection_Array1<int> mults,
                               double parametricTolerance = 0.0,
                               bool add = true)
    {
        CheckMutable();
        // Check and compute new sizes
        int nbpoles, nbknots;

        if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(_vDeg,
                                         _vPeriodic,
                                         _vKnots,
                                         _vMults,
                                         knots,
                                         mults,
                                         out nbpoles,
                                         out nbknots,
                                         parametricTolerance,
                                         add))
        {
            throw new ConstructionErrorException("Geom_BSplineSurface::InsertVKnots");
        }

        if (nbpoles == _poles.RowLength())
        {
            return;
        }

        ClearEvalRepresentation();
        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, _poles.ColLength(), 1, nbpoles);
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        if (_uRational || _vRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, _poles.ColLength(), 1, nbpoles);
            SurfaceBSplSLib.InsertKnots(false,
                                 _vDeg,
                                 _vPeriodic,
                                 _poles,
                                 InternalWeights(),
                                 _vKnots,
                                 _vMults,
                                 knots,
                                 mults,
                                 npoles,
                                 nweights,
                                 nknots,
                                 nmults,
                                 parametricTolerance,
                                 add);
            _weights = nweights;
        }
        else
        {
            SurfaceBSplSLib.InsertKnots(false,
                                 _vDeg,
                                 _vPeriodic,
                                 _poles,
                                 SurfaceBSplSLib.NoWeights(),
                                 _vKnots,
                                 _vMults,
                                 knots,
                                 mults,
                                 npoles,
                                 SurfaceBSplSLib.NoWeights(),
                                 nknots,
                                 nmults,
                                 parametricTolerance,
                                 add);
            _weights = SurfaceBSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
        }

        _poles = npoles;
        _vKnots = nknots;
        _vMults = nmults;
        UpdateVKnots();
    }

    /// <summary>
    /// Lowers the multiplicity of the U knot Index to M (0 removes it), if the surface moves by no more than
    /// Tolerance; returns false and leaves the surface unchanged otherwise.
    /// </summary>
    /// <exception cref="OutOfRangeException">Index is an end knot (non-periodic) or out of range.</exception>
    internal bool RemoveUKnot(int index, int m, double tolerance)
    {
        CheckMutable();
        if (m < 0)
        {
            return true;
        }

        int i1 = FirstUKnotIndex();
        int i2 = LastUKnotIndex();

        // OCCT: accepts the first and last knots of a periodic surface; removing one leaves knots and multiplicities
        // that BSplCLib::KnotSequence reads out of range (occt-issues.md; the port throws IndexOutOfRangeException)
        if (!_uPeriodic && (index <= i1 || index >= i2))
        {
            throw new OutOfRangeException("Geom_BSplineSurface::RemoveUKnot: invalid Index");
        }
        else if (_uPeriodic && (index < i1 || index > i2))
        {
            throw new OutOfRangeException("Geom_BSplineSurface::RemoveUKnot: invalid Index for periodic case");
        }

        int step = _uMults.Value(index) - m;
        if (step <= 0)
        {
            return true;
        }

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, _poles.ColLength() - step, 1, _poles.RowLength());
        int newUKnotLen = (m == 0) ? _uKnots.Length() - 1 : _uKnots.Length();
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, newUKnotLen);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, newUKnotLen);

        if (_uRational || _vRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, npoles.ColLength(), 1, npoles.RowLength());
            if (!SurfaceBSplSLib.RemoveKnot(true,
                                     index,
                                     m,
                                     _uDeg,
                                     _uPeriodic,
                                     _poles,
                                     InternalWeights(),
                                     _uKnots,
                                     _uMults,
                                     npoles,
                                     nweights,
                                     nknots,
                                     nmults,
                                     tolerance))
            {
                return false;
            }
            _weights = nweights;
        }
        else
        {
            if (!SurfaceBSplSLib.RemoveKnot(true,
                                     index,
                                     m,
                                     _uDeg,
                                     _uPeriodic,
                                     _poles,
                                     SurfaceBSplSLib.NoWeights(),
                                     _uKnots,
                                     _uMults,
                                     npoles,
                                     SurfaceBSplSLib.NoWeights(),
                                     nknots,
                                     nmults,
                                     tolerance))
            {
                return false;
            }
            _weights = SurfaceBSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
        }

        ClearEvalRepresentation();
        _poles = npoles;
        _uKnots = nknots;
        _uMults = nmults;

        _maxDerivInvOk = false;
        UpdateUKnots();
        return true;
    }

    /// <summary>Lowers the multiplicity of the V knot Index to M (see <see cref="RemoveUKnot"/>).</summary>
    internal bool RemoveVKnot(int index, int m, double tolerance)
    {
        CheckMutable();
        if (m < 0)
        {
            return true;
        }

        int i1 = FirstVKnotIndex();
        int i2 = LastVKnotIndex();

        // OCCT: the same as RemoveUKnot for the end knots of a periodic surface
        if (!_vPeriodic && (index <= i1 || index >= i2))
        {
            throw new OutOfRangeException("Geom_BSplineSurface::RemoveVKnot: invalid Index");
        }
        else if (_vPeriodic && (index < i1 || index > i2))
        {
            throw new OutOfRangeException("Geom_BSplineSurface::RemoveVKnot: invalid Index for periodic case");
        }

        int step = _vMults.Value(index) - m;
        if (step <= 0)
        {
            return true;
        }

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, _poles.ColLength(), 1, _poles.RowLength() - step);
        int newVKnotLen = (m == 0) ? _vKnots.Length() - 1 : _vKnots.Length();
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, newVKnotLen);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, newVKnotLen);

        if (_uRational || _vRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, npoles.ColLength(), 1, npoles.RowLength());
            if (!SurfaceBSplSLib.RemoveKnot(false,
                                     index,
                                     m,
                                     _vDeg,
                                     _vPeriodic,
                                     _poles,
                                     InternalWeights(),
                                     _vKnots,
                                     _vMults,
                                     npoles,
                                     nweights,
                                     nknots,
                                     nmults,
                                     tolerance))
            {
                return false;
            }
            _weights = nweights;
        }
        else
        {
            if (!SurfaceBSplSLib.RemoveKnot(false,
                                     index,
                                     m,
                                     _vDeg,
                                     _vPeriodic,
                                     _poles,
                                     SurfaceBSplSLib.NoWeights(),
                                     _vKnots,
                                     _vMults,
                                     npoles,
                                     SurfaceBSplSLib.NoWeights(),
                                     nknots,
                                     nmults,
                                     tolerance))
            {
                return false;
            }
            _weights = SurfaceBSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
        }

        ClearEvalRepresentation();
        _poles = npoles;
        _vKnots = nknots;
        _vMults = nmults;

        _maxDerivInvOk = false;
        UpdateVKnots();
        return true;
    }

    /// <summary>
    /// The parametric tolerances in U and V matching the 3D tolerance Tolerance3D (computed once and cached, as in
    /// OCCT; the cache is not part of the surface's value, so this works on a frozen surface).
    /// </summary>
    public void Resolution(double tolerance3D, out double uTolerance, out double vTolerance)
    {
        if (!_maxDerivInvOk)
        {
            double uMaxDerivInv, vMaxDerivInv;
            SurfaceBSplSLib.Resolution(_poles,
                                InternalWeights(),
                                _uKnots,
                                _vKnots,
                                _uMults,
                                _vMults,
                                _uDeg,
                                _vDeg,
                                _uRational,
                                _vRational,
                                _uPeriodic,
                                _vPeriodic,
                                1.0,
                                out uMaxDerivInv,
                                out vMaxDerivInv);
            _uMaxDerivInv = uMaxDerivInv;
            _vMaxDerivInv = vMaxDerivInv;
            _maxDerivInvOk = true;
        }
        uTolerance = tolerance3D * _uMaxDerivInv;
        vTolerance = tolerance3D * _vMaxDerivInv;
    }

    //=============================================================================================
    // Internal access to the stored arrays (OCCT.NET)
    //=============================================================================================

    /// <summary>The stored poles: shared, don't modify (OCCT's <c>Poles()</c> const reference).</summary>
    internal NCollection_Array2<Point3D> InternalPoles() => _poles;

    /// <summary>The stored weights, or null for a non-rational surface (OCCT's <c>Weights()</c> pointer). Don't modify.</summary>
    internal NCollection_Array2<double>? InternalWeights() => (_uRational || _vRational) ? _weights : SurfaceBSplSLib.NoWeights();

    /// <summary>The weights, NbUPoles x NbVPoles of them (OCCT's <c>WeightsArray()</c>): shared, don't modify.</summary>
    internal NCollection_Array2<double> InternalWeightsArray() => _weights;

    /// <summary>The stored U knots: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalUKnots() => _uKnots;

    /// <summary>The stored V knots: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalVKnots() => _vKnots;

    /// <summary>The stored U multiplicities: shared, don't modify.</summary>
    internal NCollection_Array1<int> InternalUMultiplicities() => _uMults;

    /// <summary>The stored V multiplicities: shared, don't modify.</summary>
    internal NCollection_Array1<int> InternalVMultiplicities() => _vMults;

    /// <summary>The stored U flat knots: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalUKnotSequence() => _uFlatKnots;

    /// <summary>The stored V flat knots: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalVKnotSequence() => _vFlatKnots;

    //=============================================================================================
    // Non-mutating twins (OCCT.NET)
    //=============================================================================================

    private BSplineSurface CopyBSpline() => (BSplineSurface)Copy();

    /// <summary>A copy with U and V exchanged (see <see cref="ExchangeUV"/>).</summary>
    public BSplineSurface ExchangedUV()
    {
        BSplineSurface copy = CopyBSpline();
        copy.ExchangeUV();
        return copy;
    }

    /// <summary>A copy made U-periodic (see <see cref="SetUPeriodic"/>).</summary>
    public BSplineSurface WithUPeriodic()
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetUPeriodic();
        return copy;
    }

    /// <summary>A copy made V-periodic (see <see cref="SetVPeriodic"/>).</summary>
    public BSplineSurface WithVPeriodic()
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetVPeriodic();
        return copy;
    }

    /// <summary>A copy whose U origin is the knot Index (see <see cref="SetUOrigin"/>).</summary>
    public BSplineSurface WithUOrigin(int index)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetUOrigin(index);
        return copy;
    }

    /// <summary>A copy whose V origin is the knot Index (see <see cref="SetVOrigin"/>).</summary>
    public BSplineSurface WithVOrigin(int index)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetVOrigin(index);
        return copy;
    }

    /// <summary>A copy made non-periodic in U (see <see cref="SetUNotPeriodic"/>).</summary>
    public BSplineSurface WithUNotPeriodic()
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetUNotPeriodic();
        return copy;
    }

    /// <summary>A copy made non-periodic in V (see <see cref="SetVNotPeriodic"/>).</summary>
    public BSplineSurface WithVNotPeriodic()
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetVNotPeriodic();
        return copy;
    }

    /// <summary>A copy with the degrees raised (see <see cref="IncreaseDegree"/>).</summary>
    public BSplineSurface DegreeIncreased(int uDegree, int vDegree)
    {
        BSplineSurface copy = CopyBSpline();
        copy.IncreaseDegree(uDegree, vDegree);
        return copy;
    }

    /// <summary>A copy with the U knots inserted (see <see cref="InsertUKnots"/>).</summary>
    public BSplineSurface UKnotsInserted(NCollection_Array1<double> knots,
                                              NCollection_Array1<int> mults,
                                              double parametricTolerance = 0.0,
                                              bool add = true)
    {
        BSplineSurface copy = CopyBSpline();
        copy.InsertUKnots(knots, mults, parametricTolerance, add);
        return copy;
    }

    /// <summary>A copy with the V knots inserted (see <see cref="InsertVKnots"/>).</summary>
    public BSplineSurface VKnotsInserted(NCollection_Array1<double> knots,
                                              NCollection_Array1<int> mults,
                                              double parametricTolerance = 0.0,
                                              bool add = true)
    {
        BSplineSurface copy = CopyBSpline();
        copy.InsertVKnots(knots, mults, parametricTolerance, add);
        return copy;
    }

    /// <summary>A copy with the U knot U inserted (see <see cref="InsertUKnot"/>).</summary>
    public BSplineSurface UKnotInserted(double u, int m, double parametricTolerance, bool add = true)
    {
        BSplineSurface copy = CopyBSpline();
        copy.InsertUKnot(u, m, parametricTolerance, add);
        return copy;
    }

    /// <summary>A copy with the V knot V inserted (see <see cref="InsertVKnot"/>).</summary>
    public BSplineSurface VKnotInserted(double v, int m, double parametricTolerance, bool add = true)
    {
        BSplineSurface copy = CopyBSpline();
        copy.InsertVKnot(v, m, parametricTolerance, add);
        return copy;
    }

    /// <summary>A copy with the U knot Index lowered to multiplicity M, or null if that fails (see <see cref="RemoveUKnot"/>).</summary>
    public BSplineSurface? UKnotRemoved(int index, int m, double tolerance)
    {
        BSplineSurface copy = CopyBSpline();
        return copy.RemoveUKnot(index, m, tolerance) ? copy : null;
    }

    /// <summary>A copy with the V knot Index lowered to multiplicity M, or null if that fails (see <see cref="RemoveVKnot"/>).</summary>
    public BSplineSurface? VKnotRemoved(int index, int m, double tolerance)
    {
        BSplineSurface copy = CopyBSpline();
        return copy.RemoveVKnot(index, m, tolerance) ? copy : null;
    }

    /// <summary>A copy with the multiplicity of the U knot UIndex raised to M.</summary>
    public BSplineSurface UMultiplicityIncreased(int uIndex, int m)
    {
        BSplineSurface copy = CopyBSpline();
        copy.IncreaseUMultiplicity(uIndex, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the U knots FromI1 to ToI2 raised to M.</summary>
    public BSplineSurface UMultiplicityIncreased(int fromI1, int toI2, int m)
    {
        BSplineSurface copy = CopyBSpline();
        copy.IncreaseUMultiplicity(fromI1, toI2, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the U knots FromI1 to ToI2 raised by Step.</summary>
    public BSplineSurface UMultiplicityIncremented(int fromI1, int toI2, int step)
    {
        BSplineSurface copy = CopyBSpline();
        copy.IncrementUMultiplicity(fromI1, toI2, step);
        return copy;
    }

    /// <summary>A copy with the multiplicity of the V knot VIndex raised to M.</summary>
    public BSplineSurface VMultiplicityIncreased(int vIndex, int m)
    {
        BSplineSurface copy = CopyBSpline();
        copy.IncreaseVMultiplicity(vIndex, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the V knots FromI1 to ToI2 raised to M.</summary>
    public BSplineSurface VMultiplicityIncreased(int fromI1, int toI2, int m)
    {
        BSplineSurface copy = CopyBSpline();
        copy.IncreaseVMultiplicity(fromI1, toI2, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the V knots FromI1 to ToI2 raised by Step.</summary>
    public BSplineSurface VMultiplicityIncremented(int fromI1, int toI2, int step)
    {
        BSplineSurface copy = CopyBSpline();
        copy.IncrementVMultiplicity(fromI1, toI2, step);
        return copy;
    }

    /// <summary>A copy restricted to [U1, U2] x [V1, V2] (see <see cref="Segment"/>).</summary>
    public BSplineSurface Segmented(double u1,
                                         double u2,
                                         double v1,
                                         double v2,
                                         double uTolerance = Precision_PConfusion,
                                         double vTolerance = Precision_PConfusion)
    {
        BSplineSurface copy = CopyBSpline();
        copy.Segment(u1, u2, v1, v2, uTolerance, vTolerance);
        return copy;
    }

    /// <summary>A copy restricted to [U1, U2] x [V1, V2] where the bounds change (see <see cref="CheckAndSegment"/>).</summary>
    public BSplineSurface CheckedAndSegmented(double u1,
                                                   double u2,
                                                   double v1,
                                                   double v2,
                                                   double uTolerance = Precision_PConfusion,
                                                   double vTolerance = Precision_PConfusion)
    {
        BSplineSurface copy = CopyBSpline();
        copy.CheckAndSegment(u1, u2, v1, v2, uTolerance, vTolerance);
        return copy;
    }

    public BSplineSurface WithUKnot(int uIndex, double k)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetUKnot(uIndex, k);
        return copy;
    }

    public BSplineSurface WithUKnots(NCollection_Array1<double> uk)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetUKnots(uk);
        return copy;
    }

    public BSplineSurface WithUKnot(int uIndex, double k, int m)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetUKnot(uIndex, k, m);
        return copy;
    }

    public BSplineSurface WithVKnot(int vIndex, double k)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetVKnot(vIndex, k);
        return copy;
    }

    public BSplineSurface WithVKnots(NCollection_Array1<double> vk)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetVKnots(vk);
        return copy;
    }

    public BSplineSurface WithVKnot(int vIndex, double k, int m)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetVKnot(vIndex, k, m);
        return copy;
    }

    public BSplineSurface WithPole(int uIndex, int vIndex, in Point3D p)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetPole(uIndex, vIndex, p);
        return copy;
    }

    public BSplineSurface WithPole(int uIndex, int vIndex, in Point3D p, double weight)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetPole(uIndex, vIndex, p, weight);
        return copy;
    }

    public BSplineSurface WithPoleCol(int vIndex, NCollection_Array1<Point3D> cPoles)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetPoleCol(vIndex, cPoles);
        return copy;
    }

    public BSplineSurface WithPoleCol(int vIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetPoleCol(vIndex, cPoles, cPoleWeights);
        return copy;
    }

    public BSplineSurface WithPoleRow(int uIndex, NCollection_Array1<Point3D> cPoles)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetPoleRow(uIndex, cPoles);
        return copy;
    }

    public BSplineSurface WithPoleRow(int uIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetPoleRow(uIndex, cPoles, cPoleWeights);
        return copy;
    }

    public BSplineSurface WithWeight(int uIndex, int vIndex, double weight)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetWeight(uIndex, vIndex, weight);
        return copy;
    }

    public BSplineSurface WithWeightCol(int vIndex, NCollection_Array1<double> cPoleWeights)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetWeightCol(vIndex, cPoleWeights);
        return copy;
    }

    public BSplineSurface WithWeightRow(int uIndex, NCollection_Array1<double> cPoleWeights)
    {
        BSplineSurface copy = CopyBSpline();
        copy.SetWeightRow(uIndex, cPoleWeights);
        return copy;
    }

    /// <summary>A copy with the point at (U, V) moved to P (see <see cref="MovePoint"/>).</summary>
    public BSplineSurface PointMoved(double u,
                                          double v,
                                          in Point3D p,
                                          int uIndex1,
                                          int uIndex2,
                                          int vIndex1,
                                          int vIndex2,
                                          out int uFirstIndex,
                                          out int uLastIndex,
                                          out int vFirstIndex,
                                          out int vLastIndex)
    {
        BSplineSurface copy = CopyBSpline();
        copy.MovePoint(u, v, p, uIndex1, uIndex2, vIndex1, vIndex2,
                        out uFirstIndex, out uLastIndex, out vFirstIndex, out vLastIndex);
        return copy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public BSplineSurface WithEvalRepresentation(RepSurfaceDesc.Base? desc)
    {
        RepUtils.ValidateSurfaceDesc(desc, this);
        BSplineSurface copy = CopyBSpline();
        copy.SetEvalRepresentation(RepUtils.CloneSurfaceDesc(desc));
        return copy;
    }
}
