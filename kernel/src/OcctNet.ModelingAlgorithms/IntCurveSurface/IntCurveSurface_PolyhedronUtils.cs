// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_PolyhedronUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The discretization of a surface into the polyhedra of the curve/surface intersections (OCCT's namespace
/// IntCurveSurface_PolyhedronUtils, the former IntCurveSurface_Polyhedron.gxx; IntPatch_Polyhedron uses it too).
/// </summary>
/// <remarks>
/// OCCT's raw arrays (<c>void*</c> cast to <c>gp_Pnt*</c>, <c>double*</c>, <c>bool*</c>, indexed from 1, of size
/// NbPoints + 1) are .NET arrays of the same size; <c>AllocateArrays</c> and <c>Destroy</c> create and drop them.
/// </remarks>
internal static class IntCurveSurface_PolyhedronUtils
{
    /// <summary>Minimum edge length for valid triangles.</summary>
    public const double THE_MIN_EDGE_LENGTH_SQUARED = 1e-15;

    /// <summary>Samples the surface on a uniform UV grid with the grid evaluator, and adds the points to the box.</summary>
    public static void InitUniform(GeomGridEval_Surface theEval, double theU0, double theV0, double theU1,
                                   double theV1, int theNbDeltaU, int theNbDeltaV, Point3D[] thePnts, double[] theU,
                                   double[] theV, bool[] theIsOnBounds, ref Box theBnd)
    {
        double dU = (theU1 - theU0) / (double)(theNbDeltaU);
        double dV = (theV1 - theV0) / (double)(theNbDeltaV);

        // Build parameter arrays
        int aNbU = theNbDeltaU + 1;
        int aNbV = theNbDeltaV + 1;

        NCollection_Array1<double> aUParams = new(1, aNbU);
        NCollection_Array1<double> aVParams = new(1, aNbV);

        for (int i = 0; i < aNbU; ++i)
        {
            aUParams[i + 1] = theU0 + i * dU;
        }
        for (int j = 0; j < aNbV; ++j)
        {
            aVParams[j + 1] = theV0 + j * dV;
        }

        // Evaluate grid
        NCollection_Array2<Point3D> aGrid = theEval.EvaluateGrid(aUParams, aVParams);

        // Copy results to output arrays (convert from 2D grid to 1D linear indexing)
        int Index = 1;
        for (int i1 = 0; i1 < aNbU; ++i1)
        {
            for (int i2 = 0; i2 < aNbV; ++i2)
            {
                Point3D TP = aGrid[i1 + 1, i2 + 1];
                thePnts[Index] = TP;
                theU[Index] = aUParams[i1 + 1];
                theV[Index] = aVParams[i2 + 1];
                theIsOnBounds[Index] = (i1 == 0 || i1 == theNbDeltaU || i2 == 0 || i2 == theNbDeltaV);
                theBnd.Add(TP);
                ++Index;
            }
        }
    }

    /// <summary>Samples the surface on the given UV parameters with the grid evaluator, and adds the points to the box.</summary>
    public static void InitWithParams(GeomGridEval_Surface theEval, NCollection_Array1<double> theUpars,
                                      NCollection_Array1<double> theVpars, int theNbDeltaU, int theNbDeltaV,
                                      Point3D[] thePnts, double[] theU, double[] theV, bool[] theIsOnBounds,
                                      ref Box theBnd)
    {
        // Evaluate grid using provided parameters
        NCollection_Array2<Point3D> aGrid = theEval.EvaluateGrid(theUpars, theVpars);

        int i0 = theUpars.Lower();
        int j0 = theVpars.Lower();
        int Index = 1;

        for (int i1 = 0; i1 <= theNbDeltaU; ++i1)
        {
            for (int i2 = 0; i2 <= theNbDeltaV; ++i2)
            {
                Point3D TP = aGrid[i1 + 1, i2 + 1];
                thePnts[Index] = TP;
                theU[Index] = theUpars[i1 + i0];
                theV[Index] = theVpars[i2 + j0];
                theIsOnBounds[Index] = (i1 == 0 || i1 == theNbDeltaU || i2 == 0 || i2 == theNbDeltaV);
                theBnd.Add(TP);
                ++Index;
            }
        }
    }

    /// <summary>
    /// The maximum deflection along a boundary isoline (U-iso if <paramref name="theIsUIso"/>), from the boundary
    /// points and the midpoints evaluated with the grid evaluator.
    /// </summary>
    public static double ComputeBorderDeflection(GeomGridEval_Surface theEval, double theParameter, double thePMin,
                                                 double thePMax, bool theIsUIso, int theNbSamples)
    {
        if (theNbSamples <= 0)
        {
            return 0.0;
        }

        // Build parameter array with both boundary points and midpoints
        // For N segments: boundary points at i*delta, midpoints at (i+0.5)*delta
        // Total: 2*N + 1 points (N+1 boundaries + N midpoints, interleaved)
        double aDelta = (thePMax - thePMin) / theNbSamples;
        int aNbPnts = 2 * theNbSamples + 1;

        NCollection_Array1<double> aVaryingParams = new(1, aNbPnts);
        NCollection_Array1<double> aFixedParams = new(1, 1);
        aFixedParams[1] = theParameter;

        // Fill varying parameters: alternating boundary and midpoint values
        // Index 1, 3, 5, ... (odd) = boundary points (0, delta, 2*delta, ...)
        // Index 2, 4, 6, ... (even) = midpoints (delta/2, 3*delta/2, ...)
        for (int i = 0; i <= theNbSamples; ++i)
        {
            aVaryingParams[2 * i + 1] = thePMin + i * aDelta;
            if (i < theNbSamples)
            {
                aVaryingParams[2 * i + 2] = thePMin + (i + 0.5) * aDelta;
            }
        }

        // Evaluate grid: 1xN or Nx1 depending on isoline direction
        NCollection_Array2<Point3D> aGrid = theIsUIso ? theEval.EvaluateGrid(aFixedParams, aVaryingParams)
                                                     : theEval.EvaluateGrid(aVaryingParams, aFixedParams);

        // Compute max deflection from pre-evaluated points
        double aDeflection = RealFirst();

        for (int i = 0; i < theNbSamples; ++i)
        {
            // Boundary points at indices 2*i+1 and 2*i+3 (1-based)
            // Midpoint at index 2*i+2 (1-based)
            Xyz aP1, aP2, aPParMid;

            if (theIsUIso)
            {
                aP1 = aGrid[1, 2 * i + 1].XYZ();
                aP2 = aGrid[1, 2 * i + 3].XYZ();
                aPParMid = aGrid[1, 2 * i + 2].XYZ();
            }
            else
            {
                aP1 = aGrid[2 * i + 1, 1].XYZ();
                aP2 = aGrid[2 * i + 3, 1].XYZ();
                aPParMid = aGrid[2 * i + 2, 1].XYZ();
            }

            Xyz aPMid = (aP2 + aP1) / 2.0;
            double aDist = (aPMid - aPParMid).Modulus();

            if (aDist > aDeflection)
            {
                aDeflection = aDist;
            }
        }

        return aDeflection;
    }

    /// <summary>The number of triangles: 2 * nbdeltaU * nbdeltaV.</summary>
    public static int NbTriangles(int theNbDeltaU, int theNbDeltaV) => theNbDeltaU * theNbDeltaV * 2;

    /// <summary>The number of points: (nbdeltaU + 1) * (nbdeltaV + 1).</summary>
    public static int NbPoints(int theNbDeltaU, int theNbDeltaV) => (theNbDeltaU + 1) * (theNbDeltaV + 1);

    /// <summary>The three vertex indices of the triangle <paramref name="theIndex"/> (1-based).</summary>
    public static void Triangle(int theIndex, out int theP1, out int theP2, out int theP3, int theNbDeltaV)
    {
        int line = 1 + ((theIndex - 1) / (theNbDeltaV * 2));
        int colon = 1 + ((theIndex - 1) % (theNbDeltaV * 2));
        int colpnt = (colon + 1) / 2;

        theP1 = (line - 1) * (theNbDeltaV + 1) + colpnt;
        theP2 = line * (theNbDeltaV + 1) + colpnt + ((colon - 1) % 2);
        theP3 = (line - 1 + (colon % 2)) * (theNbDeltaV + 1) + colpnt + 1;
    }

    /// <summary>
    /// The triangle connected to <paramref name="theTriang"/> through the edge from <paramref name="thePivot"/> to
    /// <paramref name="thePedge"/> (0 on the boundary), and its other point.
    /// </summary>
    public static int TriConnex(int theTriang, int thePivot, int thePedge, out int theTriCon, out int theOtherP,
                                int theNbDeltaU, int theNbDeltaV)
    {
        int Pivotm1 = thePivot - 1;
        int nbdeltaVp1 = theNbDeltaV + 1;
        int nbdeltaVm2 = theNbDeltaV + theNbDeltaV;

        int ligP = Pivotm1 / nbdeltaVp1;
        int colP = Pivotm1 - ligP * nbdeltaVp1;

        int ligE = 0, colE = 0, typE = 0;
        if (thePedge != 0)
        {
            ligE = (thePedge - 1) / nbdeltaVp1;
            colE = (thePedge - 1) - (ligE * nbdeltaVp1);
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

        if (theTriang != 0)
        {
            t = (theTriang - 1) / nbdeltaVm2;
            tt = (theTriang - 1) - t * nbdeltaVm2;
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
            if (thePedge == 0)
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

        theTriCon = (linT - 1) * nbdeltaVm2 + colT;

        if (linT < 1)
        {
            linO = 0;
            colO = colP + colP - colE;
            if (colO < 0)
            {
                colO = 0;
                linO = 1;
            }
            else if (colO > theNbDeltaV)
            {
                colO = theNbDeltaV;
                linO = 1;
            }
            theTriCon = 0;
        }
        else if (linT > theNbDeltaU)
        {
            linO = theNbDeltaU;
            colO = colP + colP - colE;
            if (colO < 0)
            {
                colO = 0;
                linO = theNbDeltaU - 1;
            }
            else if (colO > theNbDeltaV)
            {
                colO = theNbDeltaV;
                linO = theNbDeltaU - 1;
            }
            theTriCon = 0;
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
            else if (linO > theNbDeltaU)
            {
                linO = theNbDeltaU;
                colO = 1;
            }
            theTriCon = 0;
        }
        else if (colT > theNbDeltaV)
        {
            colO = theNbDeltaV;
            linO = ligP + ligP - ligE;
            if (linO < 0)
            {
                linO = 0;
                colO = theNbDeltaV - 1;
            }
            else if (linO > theNbDeltaU)
            {
                linO = theNbDeltaU;
                colO = theNbDeltaV - 1;
            }
            theTriCon = 0;
        }

        theOtherP = linO * nbdeltaVp1 + colO + 1;

        return theTriCon;
    }

    /// <summary>The plane equation of a triangle (a default normal and zero distance for a degenerate one).</summary>
    public static void PlaneEquation(in Point3D theP1, in Point3D theP2, in Point3D theP3, out Xyz theNormalVector,
                                     out double thePolarDistance)
    {
        Xyz v1 = theP2.XYZ() - theP1.XYZ();
        Xyz v2 = theP3.XYZ() - theP2.XYZ();
        Xyz v3 = theP1.XYZ() - theP3.XYZ();

        theNormalVector = new Xyz();
        if (v1.SquareModulus() <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            theNormalVector.SetCoord(1.0, 0.0, 0.0);
            thePolarDistance = 0.0;
            return;
        }
        if (v2.SquareModulus() <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            theNormalVector.SetCoord(1.0, 0.0, 0.0);
            thePolarDistance = 0.0;
            return;
        }
        if (v3.SquareModulus() <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            theNormalVector.SetCoord(1.0, 0.0, 0.0);
            thePolarDistance = 0.0;
            return;
        }

        theNormalVector = (v1 ^ v2) + (v2 ^ v3) + (v3 ^ v1);
        double aNormLen = theNormalVector.Modulus();
        if (aNormLen < Constants.Resolution())
        {
            thePolarDistance = 0.0;
        }
        else
        {
            theNormalVector.Divide(aNormLen);
            thePolarDistance = theNormalVector * theP1.XYZ();
        }
    }

    /// <summary>True if the point is inside the triangle.</summary>
    public static bool Contain(in Point3D theP1, in Point3D theP2, in Point3D theP3, in Point3D theTestPnt)
    {
        Xyz v1 = (theP2.XYZ() - theP1.XYZ()) ^ (theTestPnt.XYZ() - theP1.XYZ());
        Xyz v2 = (theP3.XYZ() - theP2.XYZ()) ^ (theTestPnt.XYZ() - theP2.XYZ());
        Xyz v3 = (theP1.XYZ() - theP3.XYZ()) ^ (theTestPnt.XYZ() - theP3.XYZ());

        return (v1 * v2 >= 0.0 && v2 * v3 >= 0.0 && v3 * v1 >= 0.0);
    }

    /// <summary>Fills the boxes of all the triangles, enlarged by the deflection (void for degenerate triangles).</summary>
    public static void FillBounding(Point3D[] thePnts, int theNbDeltaU, int theNbDeltaV, double theDeflection,
                                    NCollection_HArray1<Box> theComponentsBnd)
    {
        Box Boite = new();
        int nbtriangles = NbTriangles(theNbDeltaU, theNbDeltaV);

        for (int iTri = 1; iTri <= nbtriangles; ++iTri)
        {
            Triangle(iTri, out int np1, out int np2, out int np3, theNbDeltaV);
            Point3D p1 = thePnts[np1];
            Point3D p2 = thePnts[np2];
            Point3D p3 = thePnts[np3];
            Boite.SetVoid();
            if (p1.SquareDistance(p2) > THE_MIN_EDGE_LENGTH_SQUARED)
            {
                if (p1.SquareDistance(p3) > THE_MIN_EDGE_LENGTH_SQUARED)
                {
                    if (p2.SquareDistance(p3) > THE_MIN_EDGE_LENGTH_SQUARED)
                    {
                        Boite.Add(p1);
                        Boite.Add(p2);
                        Boite.Add(p3);
                        Boite.Enlarge(theDeflection);
                    }
                }
            }
            Boite.Enlarge(theDeflection);
            theComponentsBnd.SetValue(iTri, Boite);
        }
    }

    /// <summary>The deflection of a triangle: the distance from its plane to the surface point at its center.</summary>
    public static double ComputeDeflectionWithCenter(in Point3D theP1, in Point3D theP2, in Point3D theP3,
                                                     in Point3D theCenter)
    {
        // Check for degenerate triangles
        if (theP1.SquareDistance(theP2) <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            return 0.0;
        }
        if (theP1.SquareDistance(theP3) <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            return 0.0;
        }
        if (theP2.SquareDistance(theP3) <= THE_MIN_EDGE_LENGTH_SQUARED)
        {
            return 0.0;
        }

        // Compute normal vector
        Xyz XYZ1 = theP2.XYZ() - theP1.XYZ();
        Xyz XYZ2 = theP3.XYZ() - theP2.XYZ();
        Xyz XYZ3 = theP1.XYZ() - theP3.XYZ();
        Vector3D NormalVector = new((XYZ1 ^ XYZ2) + (XYZ2 ^ XYZ3) + (XYZ3 ^ XYZ1));

        double aNormLen = NormalVector.Magnitude();
        if (aNormLen < Constants.Resolution())
        {
            return 0.0;
        }

        NormalVector.Divide(aNormLen);

        // Compute distance from center to triangle plane
        Vector3D P1P = new(theP1, theCenter);
        return Math.Abs(P1P.Dot(NormalVector));
    }

    /// <summary>
    /// The maximum deflection over all the triangles, evaluating the surface at their centers one by one.
    /// </summary>
    public static double ComputeMaxDeflection<PolyhedronType>(Adaptor3d_Surface theSurface, PolyhedronType thePolyhedron,
                                                              int theNbTriangles)
        where PolyhedronType : IIntCurveSurface_Polyhedron
    {
        if (theNbTriangles <= 0)
        {
            return 0.0;
        }

        double tol = 0.0;
        for (int i = 1; i <= theNbTriangles; ++i)
        {
            thePolyhedron.Triangle(i, out int i1, out int i2, out int i3);

            Point3D P1 = thePolyhedron.Point(i1, out double u1, out double v1);
            Point3D P2 = thePolyhedron.Point(i2, out double u2, out double v2);
            Point3D P3 = thePolyhedron.Point(i3, out double u3, out double v3);

            double uCenter = (u1 + u2 + u3) / 3.0;
            double vCenter = (v1 + v2 + v3) / 3.0;

            theSurface.D0(uCenter, vCenter, out Point3D aCenter);

            double tol1 = ComputeDeflectionWithCenter(P1, P2, P3, aCenter);
            if (tol1 > tol)
            {
                tol = tol1;
            }
        }
        return tol;
    }

    /// <summary>The maximum deflection over the four boundaries, evaluated with the grid evaluator.</summary>
    public static double ComputeMaxBorderDeflection(GeomGridEval_Surface theEval, double theU0, double theV0,
                                                    double theU1, double theV1, int theNbDeltaU, int theNbDeltaV)
    {
        double maxDeflection = RealFirst();

        // Lower bound (U-isoline at U=U0, V varies from V0 to V1)
        double aDeflection = ComputeBorderDeflection(theEval, theU0, theV0, theV1, true, theNbDeltaV);
        if (aDeflection > maxDeflection)
        {
            maxDeflection = aDeflection;
        }

        // Upper bound (U-isoline at U=U1, V varies from V0 to V1)
        aDeflection = ComputeBorderDeflection(theEval, theU1, theV0, theV1, true, theNbDeltaV);
        if (aDeflection > maxDeflection)
        {
            maxDeflection = aDeflection;
        }

        // Lower bound (V-isoline at V=V0, U varies from U0 to U1)
        aDeflection = ComputeBorderDeflection(theEval, theV0, theU0, theU1, false, theNbDeltaU);
        if (aDeflection > maxDeflection)
        {
            maxDeflection = aDeflection;
        }

        // Upper bound (V-isoline at V=V1, U varies from U0 to U1)
        aDeflection = ComputeBorderDeflection(theEval, theV1, theU0, theU1, false, theNbDeltaU);
        if (aDeflection > maxDeflection)
        {
            maxDeflection = aDeflection;
        }

        return maxDeflection;
    }

    /// <summary>True if the edge between the two points lies on the boundary of the grid.</summary>
    public static bool IsOnBound(int theIndex1, int theIndex2, bool[] theIsOnBounds, int theNbDeltaU, int theNbDeltaV)
    {
        int aDiff = Math.Abs(theIndex1 - theIndex2);

        if (aDiff != 1 && aDiff != theNbDeltaV + 1)
        {
            return false;
        }

        for (int i = 0; i <= theNbDeltaU; ++i)
        {
            if ((theIndex1 == 1 + i * (theNbDeltaV + 1)) && (theIndex2 == theIndex1 - 1))
            {
                return false;
            }

            if ((theIndex1 == (1 + i) * (theNbDeltaV + 1)) && (theIndex2 == theIndex1 + 1))
            {
                return false;
            }
        }

        return (theIsOnBounds[theIndex1] && theIsOnBounds[theIndex2]);
    }

    /// <summary>Drops the arrays.</summary>
    public static void Destroy(ref Point3D[]? thePnts, ref double[]? theU, ref double[]? theV, ref bool[]? theIsOnBounds)
    {
        thePnts = null;
        theU = null;
        theV = null;
        theIsOnBounds = null;
    }

    /// <summary>Allocates the arrays: (nbdeltaU + 1) * (nbdeltaV + 1) + 1 elements, indexed from 1.</summary>
    public static void AllocateArrays(int theNbDeltaU, int theNbDeltaV, out Point3D[] thePnts, out double[] theU,
                                      out double[] theV, out bool[] theIsOnBounds)
    {
        int t = (theNbDeltaU + 1) * (theNbDeltaV + 1) + 1;
        thePnts = new Point3D[t];
        theU = new double[t];
        theV = new double[t];
        theIsOnBounds = new bool[t];
    }

    /// <summary>Sets the deflection, at least 1e-4, and enlarges the box by it.</summary>
    public static void SetDeflectionOverEstimation(double theFlec, out double theDeflection, ref Box theBnd)
    {
        const double THE_MIN_DEFLECTION = 0.0001;
        if (theFlec < THE_MIN_DEFLECTION)
        {
            theDeflection = THE_MIN_DEFLECTION;
            theBnd.Enlarge(THE_MIN_DEFLECTION);
        }
        else
        {
            theDeflection = theFlec;
            theBnd.Enlarge(theFlec);
        }
    }
}
