// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_PolyhedronUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The discretization of a surface into the polyhedra of the curve/surface intersections (OCCT's namespace
/// IntCurveSurface_PolyhedronUtils, the former IntCurveSurface_Polyhedron.gxx; IntPatch_Polyhedron uses it too).
/// </summary>
/// <remarks>
/// OCCT's raw arrays (<c>void*</c> cast to <c>gp_Pnt*</c>, <c>double*</c>, <c>bool*</c>, indexed from 1, of size
/// NbPoints + 1) are .NET arrays of the same size; <c>AllocateArrays</c> and <c>Destroy</c> create and drop them.
/// </remarks>
internal static class PolyhedronUtils
{
    /// <summary>Minimum edge length for valid triangles.</summary>
    public const double THE_MIN_EDGE_LENGTH_SQUARED = 1e-15;

    /// <summary>Samples the surface on a uniform UV grid with the grid evaluator, and adds the points to the box.</summary>
    public static void InitUniform(SurfaceGrid eval, double u0, double v0, double u1,
                                   double v1, int nbDeltaU, int nbDeltaV, Point3D[] pnts, double[] u,
                                   double[] v, bool[] isOnBounds, ref Box bnd)
    {
        double dU = (u1 - u0) / (double)(nbDeltaU);
        double dV = (v1 - v0) / (double)(nbDeltaV);

        // Build parameter arrays
        int nbU = nbDeltaU + 1;
        int nbV = nbDeltaV + 1;

        NCollection_Array1<double> uParams = new(1, nbU);
        NCollection_Array1<double> vParams = new(1, nbV);

        for (int i = 0; i < nbU; ++i)
        {
            uParams[i + 1] = u0 + i * dU;
        }
        for (int j = 0; j < nbV; ++j)
        {
            vParams[j + 1] = v0 + j * dV;
        }

        // Evaluate grid
        NCollection_Array2<Point3D> grid = eval.EvaluateGrid(uParams, vParams);

        // Copy results to output arrays (convert from 2D grid to 1D linear indexing)
        int index = 1;
        for (int i1 = 0; i1 < nbU; ++i1)
        {
            for (int i2 = 0; i2 < nbV; ++i2)
            {
                Point3D tp = grid[i1 + 1, i2 + 1];
                pnts[index] = tp;
                u[index] = uParams[i1 + 1];
                v[index] = vParams[i2 + 1];
                isOnBounds[index] = (i1 == 0 || i1 == nbDeltaU || i2 == 0 || i2 == nbDeltaV);
                bnd.Add(tp);
                ++index;
            }
        }
    }

    /// <summary>Samples the surface on the given UV parameters with the grid evaluator, and adds the points to the box.</summary>
    public static void InitWithParams(SurfaceGrid eval, NCollection_Array1<double> upars,
                                      NCollection_Array1<double> vpars, int nbDeltaU, int nbDeltaV,
                                      Point3D[] pnts, double[] u, double[] v, bool[] isOnBounds,
                                      ref Box bnd)
    {
        // Evaluate grid using provided parameters
        NCollection_Array2<Point3D> grid = eval.EvaluateGrid(upars, vpars);

        int i0 = upars.Lower();
        int j0 = vpars.Lower();
        int index = 1;

        for (int i1 = 0; i1 <= nbDeltaU; ++i1)
        {
            for (int i2 = 0; i2 <= nbDeltaV; ++i2)
            {
                Point3D tp = grid[i1 + 1, i2 + 1];
                pnts[index] = tp;
                u[index] = upars[i1 + i0];
                v[index] = vpars[i2 + j0];
                isOnBounds[index] = (i1 == 0 || i1 == nbDeltaU || i2 == 0 || i2 == nbDeltaV);
                bnd.Add(tp);
                ++index;
            }
        }
    }

    /// <summary>
    /// The maximum deflection along a boundary isoline (U-iso if <paramref name="isUIso"/>), from the boundary
    /// points and the midpoints evaluated with the grid evaluator.
    /// </summary>
    public static double ComputeBorderDeflection(SurfaceGrid eval, double parameter, double pMin,
                                                 double pMax, bool isUIso, int nbSamples)
    {
        if (nbSamples <= 0)
        {
            return 0.0;
        }

        // Build parameter array with both boundary points and midpoints
        // For N segments: boundary points at i*delta, midpoints at (i+0.5)*delta
        // Total: 2*N + 1 points (N+1 boundaries + N midpoints, interleaved)
        double delta = (pMax - pMin) / nbSamples;
        int nbPnts = 2 * nbSamples + 1;

        NCollection_Array1<double> varyingParams = new(1, nbPnts);
        NCollection_Array1<double> fixedParams = new(1, 1);
        fixedParams[1] = parameter;

        // Fill varying parameters: alternating boundary and midpoint values
        // Index 1, 3, 5, ... (odd) = boundary points (0, delta, 2*delta, ...)
        // Index 2, 4, 6, ... (even) = midpoints (delta/2, 3*delta/2, ...)
        for (int i = 0; i <= nbSamples; ++i)
        {
            varyingParams[2 * i + 1] = pMin + i * delta;
            if (i < nbSamples)
            {
                varyingParams[2 * i + 2] = pMin + (i + 0.5) * delta;
            }
        }

        // Evaluate grid: 1xN or Nx1 depending on isoline direction
        NCollection_Array2<Point3D> grid = isUIso ? eval.EvaluateGrid(fixedParams, varyingParams)
                                                     : eval.EvaluateGrid(varyingParams, fixedParams);

        // Compute max deflection from pre-evaluated points
        double deflection = RealFirst();

        for (int i = 0; i < nbSamples; ++i)
        {
            // Boundary points at indices 2*i+1 and 2*i+3 (1-based)
            // Midpoint at index 2*i+2 (1-based)
            Xyz p1, p2, pParMid;

            if (isUIso)
            {
                p1 = grid[1, 2 * i + 1].XYZ();
                p2 = grid[1, 2 * i + 3].XYZ();
                pParMid = grid[1, 2 * i + 2].XYZ();
            }
            else
            {
                p1 = grid[2 * i + 1, 1].XYZ();
                p2 = grid[2 * i + 3, 1].XYZ();
                pParMid = grid[2 * i + 2, 1].XYZ();
            }

            Xyz pMid = (p2 + p1) / 2.0;
            double dist = (pMid - pParMid).Modulus();

            if (dist > deflection)
            {
                deflection = dist;
            }
        }

        return deflection;
    }

    /// <summary>The number of triangles: 2 * nbdeltaU * nbdeltaV.</summary>
    public static int NbTriangles(int nbDeltaU, int nbDeltaV) => nbDeltaU * nbDeltaV * 2;

    /// <summary>The number of points: (nbdeltaU + 1) * (nbdeltaV + 1).</summary>
    public static int NbPoints(int nbDeltaU, int nbDeltaV) => (nbDeltaU + 1) * (nbDeltaV + 1);

    /// <summary>The three vertex indices of the triangle <paramref name="index"/> (1-based).</summary>
    public static void Triangle(int index, out int p1, out int p2, out int p3, int nbDeltaV)
    {
        int line = 1 + ((index - 1) / (nbDeltaV * 2));
        int colon = 1 + ((index - 1) % (nbDeltaV * 2));
        int colpnt = (colon + 1) / 2;

        p1 = (line - 1) * (nbDeltaV + 1) + colpnt;
        p2 = line * (nbDeltaV + 1) + colpnt + ((colon - 1) % 2);
        p3 = (line - 1 + (colon % 2)) * (nbDeltaV + 1) + colpnt + 1;
    }

    /// <summary>
    /// The triangle connected to <paramref name="triang"/> through the edge from <paramref name="pivot"/> to
    /// <paramref name="pedge"/> (0 on the boundary), and its other point.
    /// </summary>
    public static int TriConnex(int triang, int pivot, int pedge, out int triCon, out int otherP,
                                int nbDeltaU, int nbDeltaV)
    {
        int pivotm1 = pivot - 1;
        int nbdeltaVp1 = nbDeltaV + 1;
        int nbdeltaVm2 = nbDeltaV + nbDeltaV;

        int ligP = pivotm1 / nbdeltaVp1;
        int colP = pivotm1 - ligP * nbdeltaVp1;

        int ligE = 0, colE = 0, typE = 0;
        if (pedge != 0)
        {
            ligE = (pedge - 1) / nbdeltaVp1;
            colE = (pedge - 1) - (ligE * nbdeltaVp1);
            if (ligP == ligE)
            {
                typE = 1;
            }
            else if (colP == colE)
            {
                typE = 2;
            }
            else
            {
                typE = 3;
            }
        }

        int linT = 0, colT = 0;
        int linO = 0, colO = 0;
        int t = 0, tt = 0;

        if (triang != 0)
        {
            t = (triang - 1) / nbdeltaVm2;
            tt = (triang - 1) - t * nbdeltaVm2;
            linT = 1 + t;
            colT = 1 + tt;
            if (typE == 0)
            {
                if (ligP == linT)
                {
                    ligE = ligP - 1;
                    colE = colP - 1;
                    typE = 3;
                }
                else
                {
                    if (colT == ligP + ligP)
                    {
                        ligE = ligP;
                        colE = colP - 1;
                        typE = 1;
                    }
                    else
                    {
                        ligE = ligP + 1;
                        colE = colP + 1;
                        typE = 3;
                    }
                }
            }
            switch (typE)
            {
                case 1:
                    if (linT == ligP)
                    {
                        linT++;
                        linO = ligP + 1;
                        colO = (colP > colE) ? colP : colE;
                    }
                    else
                    {
                        linT--;
                        linO = ligP - 1;
                        colO = (colP < colE) ? colP : colE;
                    }
                    break;
                case 2:
                    if (colT == (colP + colP))
                    {
                        colT++;
                        linO = (ligP > ligE) ? ligP : ligE;
                        colO = colP + 1;
                    }
                    else
                    {
                        colT--;
                        linO = (ligP < ligE) ? ligP : ligE;
                        colO = colP - 1;
                    }
                    break;
                case 3:
                    if ((colT & 1) == 0)
                    {
                        colT--;
                        linO = (ligP > ligE) ? ligP : ligE;
                        colO = (colP < colE) ? colP : colE;
                    }
                    else
                    {
                        colT++;
                        linO = (ligP < ligE) ? ligP : ligE;
                        colO = (colP > colE) ? colP : colE;
                    }
                    break;
            }
        }
        else
        {
            if (pedge == 0)
            {
                linT = (1 > ligP) ? 1 : ligP;
                colT = (1 > (colP + colP)) ? 1 : (colP + colP);
                if (ligP == 0)
                {
                    linO = ligP + 1;
                }
                else
                {
                    linO = ligP - 1;
                }
                colO = colP;
            }
            else
            {
                switch (typE)
                {
                    case 1:
                        linT = ligP + 1;
                        colT = (colP > colE) ? colP : colE;
                        colT += colT;
                        linO = ligP + 1;
                        colO = (colP > colE) ? colP : colE;
                        break;
                    case 2:
                        linT = (ligP > ligE) ? ligP : ligE;
                        colT = colP + colP;
                        linO = (ligP < ligE) ? ligP : ligE;
                        colO = colP - 1;
                        break;
                    case 3:
                        linT = (ligP > ligE) ? ligP : ligE;
                        colT = colP + colE;
                        linO = (ligP > ligE) ? ligP : ligE;
                        colO = (colP < colE) ? colP : colE;
                        break;
                }
            }
        }

        triCon = (linT - 1) * nbdeltaVm2 + colT;

        if (linT < 1)
        {
            linO = 0;
            colO = colP + colP - colE;
            if (colO < 0)
            {
                colO = 0;
                linO = 1;
            }
            else if (colO > nbDeltaV)
            {
                colO = nbDeltaV;
                linO = 1;
            }
            triCon = 0;
        }
        else if (linT > nbDeltaU)
        {
            linO = nbDeltaU;
            colO = colP + colP - colE;
            if (colO < 0)
            {
                colO = 0;
                linO = nbDeltaU - 1;
            }
            else if (colO > nbDeltaV)
            {
                colO = nbDeltaV;
                linO = nbDeltaU - 1;
            }
            triCon = 0;
        }

        if (colT < 1)
        {
            colO = 0;
            linO = ligP + ligP - ligE;
            if (linO < 0)
            {
                linO = 0;
                colO = 1;
            }
            else if (linO > nbDeltaU)
            {
                linO = nbDeltaU;
                colO = 1;
            }
            triCon = 0;
        }
        else if (colT > nbDeltaV)
        {
            colO = nbDeltaV;
            linO = ligP + ligP - ligE;
            if (linO < 0)
            {
                linO = 0;
                colO = nbDeltaV - 1;
            }
            else if (linO > nbDeltaU)
            {
                linO = nbDeltaU;
                colO = nbDeltaV - 1;
            }
            triCon = 0;
        }

        otherP = linO * nbdeltaVp1 + colO + 1;

        return triCon;
    }

    /// <summary>The plane equation of a triangle (a default normal and zero distance for a degenerate one).</summary>
    public static void PlaneEquation(in Point3D p1, in Point3D p2, in Point3D p3, out Xyz normalVector,
                                     out double polarDistance)
    {
        Xyz v1 = p2.XYZ() - p1.XYZ();
        Xyz v2 = p3.XYZ() - p2.XYZ();
        Xyz v3 = p1.XYZ() - p3.XYZ();

        normalVector = new Xyz();
        if (v1.SquareModulus() <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            normalVector.SetCoord(1.0, 0.0, 0.0);
            polarDistance = 0.0;
            return;
        }
        if (v2.SquareModulus() <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            normalVector.SetCoord(1.0, 0.0, 0.0);
            polarDistance = 0.0;
            return;
        }
        if (v3.SquareModulus() <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            normalVector.SetCoord(1.0, 0.0, 0.0);
            polarDistance = 0.0;
            return;
        }

        normalVector = (v1 ^ v2) + (v2 ^ v3) + (v3 ^ v1);
        double normLen = normalVector.Modulus();
        if (normLen < Constants.Resolution())
        {
            polarDistance = 0.0;
        }
        else
        {
            normalVector.Divide(normLen);
            polarDistance = normalVector * p1.XYZ();
        }
    }

    /// <summary>True if the point is inside the triangle.</summary>
    public static bool Contain(in Point3D p1, in Point3D p2, in Point3D p3, in Point3D testPnt)
    {
        Xyz v1 = (p2.XYZ() - p1.XYZ()) ^ (testPnt.XYZ() - p1.XYZ());
        Xyz v2 = (p3.XYZ() - p2.XYZ()) ^ (testPnt.XYZ() - p2.XYZ());
        Xyz v3 = (p1.XYZ() - p3.XYZ()) ^ (testPnt.XYZ() - p3.XYZ());

        return (v1 * v2 >= 0.0 && v2 * v3 >= 0.0 && v3 * v1 >= 0.0);
    }

    /// <summary>Fills the boxes of all the triangles, enlarged by the deflection (void for degenerate triangles).</summary>
    public static void FillBounding(Point3D[] pnts, int nbDeltaU, int nbDeltaV, double deflection,
                                    NCollection_HArray1<Box> componentsBnd)
    {
        Box boite = new();
        int nbtriangles = NbTriangles(nbDeltaU, nbDeltaV);

        for (int iTri = 1; iTri <= nbtriangles; ++iTri)
        {
            Triangle(iTri, out int np1, out int np2, out int np3, nbDeltaV);
            Point3D p1 = pnts[np1];
            Point3D p2 = pnts[np2];
            Point3D p3 = pnts[np3];
            boite.SetVoid();
            if (p1.SquareDistanceTo(p2) > THE_MIN_EDGE_LENGTH_SQUARED)
            {
                if (p1.SquareDistanceTo(p3) > THE_MIN_EDGE_LENGTH_SQUARED)
                {
                    if (p2.SquareDistanceTo(p3) > THE_MIN_EDGE_LENGTH_SQUARED)
                    {
                        boite.Add(p1);
                        boite.Add(p2);
                        boite.Add(p3);
                        boite.Enlarge(deflection);
                    }
                }
            }
            boite.Enlarge(deflection);
            componentsBnd.SetValue(iTri, boite);
        }
    }

    /// <summary>The deflection of a triangle: the distance from its plane to the surface point at its center.</summary>
    public static double ComputeDeflectionWithCenter(in Point3D p1, in Point3D p2, in Point3D p3,
                                                     in Point3D center)
    {
        // Check for degenerate triangles
        if (p1.SquareDistanceTo(p2) <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            return 0.0;
        }
        if (p1.SquareDistanceTo(p3) <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            return 0.0;
        }
        if (p2.SquareDistanceTo(p3) <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            return 0.0;
        }

        // Compute normal vector
        Xyz xyz1 = p2.XYZ() - p1.XYZ();
        Xyz xyz2 = p3.XYZ() - p2.XYZ();
        Xyz xyz3 = p1.XYZ() - p3.XYZ();
        Vector3D normalVector = new((xyz1 ^ xyz2) + (xyz2 ^ xyz3) + (xyz3 ^ xyz1));

        double normLen = normalVector.Magnitude;
        if (normLen < Constants.Resolution())
        {
            return 0.0;
        }

        normalVector = normalVector.Divided(normLen);

        // Compute distance from center to triangle plane
        Vector3D p1P = new(p1, center);
        return Math.Abs(p1P.Dot(normalVector));
    }

    /// <summary>
    /// The maximum deflection over all the triangles, evaluating the surface at their centers one by one.
    /// </summary>
    public static double ComputeMaxDeflection<PolyhedronType>(SurfaceAdaptorBase surface, PolyhedronType polyhedron,
                                                              int nbTriangles)
        where PolyhedronType : IPolyhedron
    {
        if (nbTriangles <= 0)
        {
            return 0.0;
        }

        double tol = 0.0;
        for (int i = 1; i <= nbTriangles; ++i)
        {
            polyhedron.Triangle(i, out int i1, out int i2, out int i3);

            Point3D p1 = polyhedron.Point(i1, out double u1, out double v1);
            Point3D p2 = polyhedron.Point(i2, out double u2, out double v2);
            Point3D p3 = polyhedron.Point(i3, out double u3, out double v3);

            double uCenter = (u1 + u2 + u3) / 3.0;
            double vCenter = (v1 + v2 + v3) / 3.0;

            surface.D0(uCenter, vCenter, out Point3D center);

            double tol1 = ComputeDeflectionWithCenter(p1, p2, p3, center);
            if (tol1 > tol)
            {
                tol = tol1;
            }
        }
        return tol;
    }

    /// <summary>The maximum deflection over the four boundaries, evaluated with the grid evaluator.</summary>
    public static double ComputeMaxBorderDeflection(SurfaceGrid eval, double u0, double v0,
                                                    double u1, double v1, int nbDeltaU, int nbDeltaV)
    {
        double maxDeflection = RealFirst();

        // Lower bound (U-isoline at U=U0, V varies from V0 to V1)
        double deflection = ComputeBorderDeflection(eval, u0, v0, v1, true, nbDeltaV);
        if (deflection > maxDeflection)
        {
            maxDeflection = deflection;
        }

        // Upper bound (U-isoline at U=U1, V varies from V0 to V1)
        deflection = ComputeBorderDeflection(eval, u1, v0, v1, true, nbDeltaV);
        if (deflection > maxDeflection)
        {
            maxDeflection = deflection;
        }

        // Lower bound (V-isoline at V=V0, U varies from U0 to U1)
        deflection = ComputeBorderDeflection(eval, v0, u0, u1, false, nbDeltaU);
        if (deflection > maxDeflection)
        {
            maxDeflection = deflection;
        }

        // Upper bound (V-isoline at V=V1, U varies from U0 to U1)
        deflection = ComputeBorderDeflection(eval, v1, u0, u1, false, nbDeltaU);
        if (deflection > maxDeflection)
        {
            maxDeflection = deflection;
        }

        return maxDeflection;
    }

    /// <summary>True if the edge between the two points lies on the boundary of the grid.</summary>
    public static bool IsOnBound(int index1, int index2, bool[] isOnBounds, int nbDeltaU, int nbDeltaV)
    {
        int diff = Math.Abs(index1 - index2);

        if (diff != 1 && diff != nbDeltaV + 1)
        {
            return false;
        }

        for (int i = 0; i <= nbDeltaU; ++i)
        {
            if ((index1 == 1 + i * (nbDeltaV + 1)) && (index2 == index1 - 1))
            {
                return false;
            }

            if ((index1 == (1 + i) * (nbDeltaV + 1)) && (index2 == index1 + 1))
            {
                return false;
            }
        }

        return (isOnBounds[index1] && isOnBounds[index2]);
    }

    /// <summary>Drops the arrays.</summary>
    public static void Destroy(ref Point3D[]? pnts, ref double[]? u, ref double[]? v, ref bool[]? isOnBounds)
    {
        pnts = null;
        u = null;
        v = null;
        isOnBounds = null;
    }

    /// <summary>Allocates the arrays: (nbdeltaU + 1) * (nbdeltaV + 1) + 1 elements, indexed from 1.</summary>
    public static void AllocateArrays(int nbDeltaU, int nbDeltaV, out Point3D[] pnts, out double[] u,
                                      out double[] v, out bool[] isOnBounds)
    {
        int t = (nbDeltaU + 1) * (nbDeltaV + 1) + 1;
        pnts = new Point3D[t];
        u = new double[t];
        v = new double[t];
        isOnBounds = new bool[t];
    }

    /// <summary>Sets the deflection, at least 1e-4, and enlarges the box by it.</summary>
    public static void SetDeflectionOverEstimation(double flec, out double deflection, ref Box bnd)
    {
        const double THE_MIN_DEFLECTION = 0.0001;
        if (flec < THE_MIN_DEFLECTION)
        {
            deflection = THE_MIN_DEFLECTION;
            bnd.Enlarge(THE_MIN_DEFLECTION);
        }
        else
        {
            deflection = flec;
            bnd.Enlarge(flec);
        }
    }
}
