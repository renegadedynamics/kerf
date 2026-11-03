// Port of: src/FoundationClasses/TKMath/Convert/Convert_GridPolynomialToPoles.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_GridPolynomialToPoles.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a grid of polynomial surfaces (3D) into the poles of an equivalent B-spline surface, by
/// interpolation at the Schoenberg points.
/// </summary>
/// <remarks>The batch accessors return copies, where OCCT returns references to its fields.</remarks>
public class GridPolynomialToPoles
{
    private NCollection_Array1<double> _uFlatKnots = new();
    private NCollection_Array1<double> _vFlatKnots = new();
    private NCollection_Array1<double> _uKnots = new();
    private NCollection_Array1<double> _vKnots = new();
    private NCollection_Array1<int> _uMults = new();
    private NCollection_Array1<int> _vMults = new();
    private NCollection_Array2<Point3D> _poles = new();
    private int _uDegree;
    private int _vDegree;
    private bool _done;

    /// <summary>
    /// Handle form of <see cref="GridPolynomialToPoles(int, int, NCollection_Array1{int}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{double})"/>.
    /// </summary>
    public GridPolynomialToPoles(int maxUDegree,
                                         int maxVDegree,
                                         NCollection_HArray1<int> numCoeff,
                                         NCollection_HArray1<double> coefficients,
                                         NCollection_HArray1<double> polynomialUIntervals,
                                         NCollection_HArray1<double> polynomialVIntervals)
        : this(maxUDegree,
               maxVDegree,
               numCoeff.Array1(),
               coefficients.Array1(),
               polynomialUIntervals.Array1(),
               polynomialVIntervals.Array1())
    {
    }

    /// <summary>
    /// Handle form of <see cref="GridPolynomialToPoles(int, int, int, int, int, int, NCollection_Array2{int}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{double})"/>.
    /// </summary>
    public GridPolynomialToPoles(int nbUSurfaces,
                                         int nbVSurfaces,
                                         int uContinuity,
                                         int vContinuity,
                                         int maxUDegree,
                                         int maxVDegree,
                                         NCollection_HArray2<int> numCoeffPerSurface,
                                         NCollection_HArray1<double> coefficients,
                                         NCollection_HArray1<double> polynomialUIntervals,
                                         NCollection_HArray1<double> polynomialVIntervals,
                                         NCollection_HArray1<double> trueUIntervals,
                                         NCollection_HArray1<double> trueVIntervals)
        : this(nbUSurfaces,
               nbVSurfaces,
               uContinuity,
               vContinuity,
               maxUDegree,
               maxVDegree,
               numCoeffPerSurface.Array2(),
               coefficients.Array1(),
               polynomialUIntervals.Array1(),
               polynomialVIntervals.Array1(),
               trueUIntervals.Array1(),
               trueVIntervals.Array1())
    {
    }

    /// <summary>
    /// Converts one polynomial surface. <paramref name="polynomialUIntervals"/> and
    /// <paramref name="polynomialVIntervals"/> (2 values each) are the domain of the polynomial;
    /// <paramref name="numCoeff"/> (bounds 1..2) the numbers of coefficients in U and V; the coefficients are a
    /// "C array" [MaxUDegree + 1][MaxVDegree + 1][3] (bounds from 1).
    /// </summary>
    public GridPolynomialToPoles(int maxUDegree,
                                         int maxVDegree,
                                         NCollection_Array1<int> numCoeff,
                                         NCollection_Array1<double> coefficients,
                                         NCollection_Array1<double> polynomialUIntervals,
                                         NCollection_Array1<double> polynomialVIntervals)
    {
        _uDegree = 0;
        _vDegree = 0;
        _done = false;
        if (numCoeff.Lower() != 1 || numCoeff.Upper() != 2)
        {
            throw new DomainErrorException("Convert : Wrong Coefficients");
        }
        if (coefficients.Lower() != 1
            || coefficients.Upper() != 3 * (maxUDegree + 1) * (maxVDegree + 1))
        {
            throw new DomainErrorException("Convert : Wrong Coefficients");
        }

        _uDegree = numCoeff.Value(1) - 1;
        _vDegree = numCoeff.Value(2) - 1;

        if (_uDegree > maxUDegree)
        {
            throw new DomainErrorException("Convert : Incoherence between NumCoeffPerSurface and MaxUDegree");
        }
        if (_vDegree > maxVDegree)
        {
            throw new DomainErrorException("Convert : Incoherence between NumCoeffPerSurface and MaxVDegree");
        }

        NCollection_Array2<int> numCoeff2D = new(1, 1, 1, 2);
        numCoeff2D.SetValue(1, 1, numCoeff.Value(1));
        numCoeff2D.SetValue(1, 2, numCoeff.Value(2));

        Perform(0,
                0,
                maxUDegree,
                maxVDegree,
                numCoeff2D,
                coefficients,
                polynomialUIntervals,
                polynomialVIntervals,
                polynomialUIntervals,
                polynomialVIntervals);
    }

    /// <summary>
    /// Converts a grid of NbUSurfaces x NbVSurfaces polynomial surfaces, with the given continuity in each
    /// direction (at most the maximum degree). <paramref name="trueUIntervals"/> and
    /// <paramref name="trueVIntervals"/> are the parametrisation of the result.
    /// <paramref name="numCoeffPerSurface"/> is a [1, NbVSurfaces * NbUSurfaces, 1, 2] array, the coefficients a
    /// "C array" [NbVSurfaces][NbUSurfaces][MaxUDegree + 1][MaxVDegree + 1][3] (bounds from 1);
    /// <see cref="DomainErrorException"/> otherwise.
    /// </summary>
    public GridPolynomialToPoles(int nbUSurfaces,
                                         int nbVSurfaces,
                                         int uContinuity,
                                         int vContinuity,
                                         int maxUDegree,
                                         int maxVDegree,
                                         NCollection_Array2<int> numCoeffPerSurface,
                                         NCollection_Array1<double> coefficients,
                                         NCollection_Array1<double> polynomialUIntervals,
                                         NCollection_Array1<double> polynomialVIntervals,
                                         NCollection_Array1<double> trueUIntervals,
                                         NCollection_Array1<double> trueVIntervals)
    {
        _uDegree = 0;
        _vDegree = 0;
        _done = false;
        int realUDegree = Max(maxUDegree, 2 * uContinuity + 1);
        int realVDegree = Max(maxVDegree, 2 * vContinuity + 1);

        if (numCoeffPerSurface.LowerRow() != 1
            || numCoeffPerSurface.UpperRow() != nbUSurfaces * nbVSurfaces
            || numCoeffPerSurface.LowerCol() != 1 || numCoeffPerSurface.UpperCol() != 2)
        {
            throw new DomainErrorException("Convert : Wrong NumCoeffPerSurface");
        }

        if (coefficients.Lower() != 1
            || coefficients.Upper()
                 != 3 * nbUSurfaces * nbVSurfaces * (realUDegree + 1) * (realVDegree + 1))
        {
            throw new DomainErrorException("Convert : Wrong Coefficients");
        }

        // Compute actual degrees from the supplied coefficient counts.
        for (int ii = 1; ii <= nbUSurfaces * nbVSurfaces; ++ii)
        {
            if (numCoeffPerSurface.Value(ii, 1) > _uDegree + 1)
            {
                _uDegree = numCoeffPerSurface.Value(ii, 1) - 1;
            }
            if (numCoeffPerSurface.Value(ii, 2) > _vDegree + 1)
            {
                _vDegree = numCoeffPerSurface.Value(ii, 2) - 1;
            }
        }

        if (_uDegree > realUDegree)
        {
            throw new DomainErrorException("Convert : Incoherence between NumCoeffPerSurface and MaxUDegree");
        }
        if (_vDegree > realVDegree)
        {
            throw new DomainErrorException("Convert : Incoherence between NumCoeffPerSurface and MaxVDegree");
        }

        Perform(uContinuity,
                vContinuity,
                realUDegree,
                realVDegree,
                numCoeffPerSurface,
                coefficients,
                polynomialUIntervals,
                polynomialVIntervals,
                trueUIntervals,
                trueVIntervals);
    }

    private void Perform(int uContinuity,
                         int vContinuity,
                         int maxUDegree,
                         int maxVDegree,
                         NCollection_Array2<int> numCoeffPerSurface,
                         NCollection_Array1<double> coefficients,
                         NCollection_Array1<double> polynomialUIntervals,
                         NCollection_Array1<double> polynomialVIntervals,
                         NCollection_Array1<double> trueUIntervals,
                         NCollection_Array1<double> trueVIntervals)
    {
        // (1) Build monodimensional knot/multiplicity/parameter tables.
        NCollection_Array1<double> uParameters;
        NCollection_Array1<double> vParameters;
        _uKnots = new NCollection_Array1<double>(trueUIntervals);
        _vKnots = new NCollection_Array1<double>(trueVIntervals);

        BuildArray(_uDegree, _uKnots, uContinuity, out _uFlatKnots, out _uMults, out uParameters);
        BuildArray(_vDegree, _vKnots, vContinuity, out _vFlatKnots, out _vMults, out vParameters);

        // (2) Digitalisation
        int dimension = 3 * (_vDegree + 1);
        int sizPatch = 3 * (maxUDegree + 1) * (maxVDegree + 1);
        _poles = new NCollection_Array2<Point3D>(1, uParameters.Length(), 1, vParameters.Length());

        NCollection_Array1<double> patch = new(1, (_uDegree + 1) * dimension);
        Span<double> digit = stackalloc double[3];
        ReadOnlySpan<double> coeffs = patch.AsSpan();

        int patchIndice = 0;
        int uIndex = 1;
        for (int ii = 1; ii <= uParameters.Length(); ++ii)
        {
            while (uParameters.Value(ii) > trueUIntervals.Value(uIndex + 1)
                   && uIndex < _uKnots.Length() - 1)
            {
                ++uIndex;
            }
            double nValue = (uParameters.Value(ii) - trueUIntervals.Value(uIndex))
                             / (trueUIntervals.Value(uIndex + 1) - trueUIntervals.Value(uIndex));
            double uValue = (1.0 - nValue) * polynomialUIntervals.Value(1)
                             + nValue * polynomialUIntervals.Value(2);

            int vIndex = 1;
            for (int jj = 1; jj <= vParameters.Length(); ++jj)
            {
                while (vParameters.Value(jj) > trueVIntervals.Value(vIndex + 1)
                       && vIndex < _vKnots.Length() - 1)
                {
                    ++vIndex;
                }
                nValue = (vParameters.Value(jj) - trueVIntervals.Value(vIndex))
                          / (trueVIntervals.Value(vIndex + 1) - trueVIntervals.Value(vIndex));
                double vValue = (1.0 - nValue) * polynomialVIntervals.Value(1)
                                 + nValue * polynomialVIntervals.Value(2);

                // (2.1) Extract the active patch coefficients.
                if (patchIndice != uIndex + (_uKnots.Length() - 1) * (vIndex - 1))
                {
                    patchIndice = uIndex + (_uKnots.Length() - 1) * (vIndex - 1);
                    int ll = 1;
                    for (int k1 = 1; k1 <= numCoeffPerSurface.Value(patchIndice, 1); ++k1)
                    {
                        int pos = sizPatch * (patchIndice - 1) + 3 * (maxVDegree + 1) * (k1 - 1) + 1;
                        for (int k2 = 1; k2 <= numCoeffPerSurface.Value(patchIndice, 2); ++k2, pos += 3)
                        {
                            patch[ll] = coefficients.Value(pos);
                            patch[ll + 1] = coefficients.Value(pos + 1);
                            patch[ll + 2] = coefficients.Value(pos + 2);
                            ll += 3;
                        }
                    }
                }

                // (2.2) Evaluate at (aUValue, aVValue).
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPoly2Var(uValue,
                                  vValue,
                                  0,
                                  0,
                                  numCoeffPerSurface.Value(patchIndice, 1) - 1,
                                  numCoeffPerSurface.Value(patchIndice, 2) - 1,
                                  3,
                                  coeffs,
                                  digit);

                _poles.SetValue(ii, jj, new Point3D(digit[0], digit[1], digit[2]));
            }
        }

        // (3)Interpolation --------------------------------------------------------------

        SurfaceBSplSLib.Interpolate(_uDegree,
                             _vDegree,
                             _uFlatKnots,
                             _vFlatKnots,
                             uParameters,
                             vParameters,
                             _poles,
                             out int inversionProblem);
        _done = (inversionProblem == 0);
    }

    /// <summary>The multiplicities, flat knots and Schoenberg parameters of one direction.</summary>
    /// <remarks>The arrays are outputs, where OCCT assigns new arrays to its reference parameters.</remarks>
    private static void BuildArray(int degree,
                                   NCollection_Array1<double> knots,
                                   int continuity,
                                   out NCollection_Array1<double> flatKnots,
                                   out NCollection_Array1<int> mults,
                                   out NCollection_Array1<double> parameters)
    {
        int numCurves = knots.Length() - 1;

        // Calcul des Multiplicites
        int multiplicities = degree - continuity;
        mults = new NCollection_Array1<int>(1, knots.Length());

        for (int ii = 2; ii < knots.Length(); ii++)
        {
            mults.SetValue(ii, multiplicities);
        }
        mults.SetValue(1, degree + 1);
        mults.SetValue(numCurves + 1, degree + 1);

        // Calcul des Noeuds Plats
        int num_flat_knots = multiplicities * (numCurves - 1) + 2 * degree + 2;
        flatKnots = new NCollection_Array1<double>(1, num_flat_knots);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(knots, mults, degree, false, flatKnots);

        // Calcul du nombre de Poles
        int num_poles = num_flat_knots - degree - 1;

        // Cacul des parametres d'interpolation
        parameters = new NCollection_Array1<double>(1, num_poles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(degree, flatKnots, parameters);
    }

    /// <summary>The number of poles in U.</summary>
    public int UPoleCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
            return _poles.ColLength();
        }
    }

    /// <summary>The number of poles in V.</summary>
    public int VPoleCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
            return _poles.RowLength();
        }
    }

    /// <summary>The poles of the B-spline surface: a copy.</summary>
    public NCollection_Array2<Point3D> Poles()
    {
        StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
        return new NCollection_Array2<Point3D>(_poles);
    }

    /// <summary>The degree in U.</summary>
    public int UDegree()
    {
        StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
        return _uDegree;
    }

    /// <summary>The degree in V.</summary>
    public int VDegree()
    {
        StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
        return _vDegree;
    }

    /// <summary>The number of knots in U.</summary>
    public int UKnotCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
            return _uKnots.Length();
        }
    }

    /// <summary>The number of knots in V.</summary>
    public int VKnotCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
            return _vKnots.Length();
        }
    }

    /// <summary>The knots in U: a copy.</summary>
    public NCollection_Array1<double> UKnots()
    {
        StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
        return new NCollection_Array1<double>(_uKnots);
    }

    /// <summary>The knots in V: a copy.</summary>
    public NCollection_Array1<double> VKnots()
    {
        StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
        return new NCollection_Array1<double>(_vKnots);
    }

    /// <summary>The multiplicities of the U knots: a copy.</summary>
    public NCollection_Array1<int> UMultiplicities()
    {
        StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
        return new NCollection_Array1<int>(_uMults);
    }

    /// <summary>The multiplicities of the V knots: a copy.</summary>
    public NCollection_Array1<int> VMultiplicities()
    {
        StdFail_NotDone_Raise_if(!_done, "GridPolynomialToPoles");
        return new NCollection_Array1<int>(_vMults);
    }

    /// <summary>True if the conversion succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _done;
        }
    }
}
