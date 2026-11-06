// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polyhedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polyhedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// A polyhedron discretizing a surface for IntPatch_PrmPrmIntersection: a uniform grid of points in (U, V) over the
/// surface's bounds, cut into two triangles per cell, with the boxes of the triangles enlarged by the deflection.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>DeflectionOverEstimation(double)</c>, <see cref="FillBounding"/> and
/// <see cref="Destroy"/> are public, as in OCCT. OCCT's destructor calls <c>Destroy</c>, which drops the arrays; the
/// garbage collector does the rest. OCCT declares <c>Perform</c>, the four singularity setters
/// (<c>UMinSingularity</c>, ...) and their <c>Has...</c> getters but never defines them: they are left out. The setter
/// <c>Point(thePnt, lig, col, U, V)</c> and <c>Dump</c> are empty in OCCT and kept so.
/// </remarks>
internal sealed class Polyhedron : IPolyhedron
{
    private const double LONGUEUR_MINI_EDGE_TRIANGLE = 1e-14;
    private const double DEFLECTION_COEFF = 1.1;
    private const int Nbmaxuv = 30;

    private Box _theBnd = new();
    private NCollection_HArray1<Box>? _theComponentsBnd;
    private double _theDeflection;
    private readonly int _nbdeltaU;
    private readonly int _nbdeltaV;
    private Point3D[]? C_MyPnts;
    private double[]? C_MyU;
    private double[]? C_MyV;

    //================================================================================
    private static int NbPOnU(SurfaceAdaptorBase s)
    {
        double u0 = s.FirstUParameter();
        double u1 = s.LastUParameter();
        int nbpu = HInterTool.NbSamplesU(s, u0, u1);
        return (nbpu > Nbmaxuv ? Nbmaxuv : nbpu);
    }

    //================================================================================
    private static int NbPOnV(SurfaceAdaptorBase s)
    {
        double v0 = s.FirstVParameter();
        double v1 = s.LastVParameter();
        int nbpv = HInterTool.NbSamplesV(s, v0, v1);
        return (nbpv > Nbmaxuv ? Nbmaxuv : nbpv);
    }

    /// <summary>Drops the arrays of points and parameters (OCCT's destructor calls it).</summary>
    public void Destroy()
    {
        C_MyPnts = null;
        C_MyU = null;
        C_MyV = null;
    }

    /// <summary>
    /// A polyhedron of the surface with as many cells as IntPatch_HInterTool samples (at most 30 in each direction).
    /// </summary>
    public Polyhedron(SurfaceAdaptorBase surface)
    {
        _theDeflection = Epsilon(100.0);
        _nbdeltaU = NbPOnU(surface);
        _nbdeltaV = NbPOnV(surface);
        Init(surface);
    }

    /// <summary>A polyhedron of the surface with <paramref name="nbu"/> by <paramref name="nbv"/> cells (at least 1 by 1).</summary>
    public Polyhedron(SurfaceAdaptorBase surface, int nbu, int nbv)
    {
        _theDeflection = Epsilon(100.0);
        _nbdeltaU = (nbu < 1 ? 1 : nbu);
        _nbdeltaV = (nbv < 1 ? 1 : nbv);
        Init(surface);
    }

    // The body both of OCCT's constructors repeat verbatim after their member initialisers.
    private void Init(SurfaceAdaptorBase surface)
    {
        int t = (_nbdeltaU + 1) * (_nbdeltaV + 1) + 1;
        Point3D[] cMyPnts = new Point3D[t];
        double[] cMyU = new double[t];
        double[] cMyV = new double[t];
        C_MyPnts = cMyPnts;
        C_MyU = cMyU;
        C_MyV = cMyV;

        double u0 = surface.FirstUParameter();
        double u1 = surface.LastUParameter();
        double v0 = surface.FirstVParameter();
        double v1 = surface.LastVParameter();

        // Build UV parameter arrays
        NCollection_Array1<double> uParams = new(0, _nbdeltaU);
        NCollection_Array1<double> vParams = new(0, _nbdeltaV);
        double u1mU0sNbdeltaU = (u1 - u0) / (double)_nbdeltaU;
        double v1mV0sNbdeltaV = (v1 - v0) / (double)_nbdeltaV;

        for (int i = 0; i <= _nbdeltaU; ++i)
        {
            uParams.SetValue(i, u0 + i * u1mU0sNbdeltaU);
        }
        for (int j = 0; j <= _nbdeltaV; ++j)
        {
            vParams.SetValue(j, v0 + j * v1mV0sNbdeltaV);
        }

        // Use grid evaluator for batch point evaluation
        SurfaceGrid eval = new(surface);
        NCollection_Array2<Point3D> gridPnts = eval.EvaluateGrid(uParams, vParams);

        // Copy to internal arrays and build bounding box
        int index = 1;
        for (int i1 = 0; i1 <= _nbdeltaU; ++i1)
        {
            for (int i2 = 0; i2 <= _nbdeltaV; ++i2)
            {
                cMyPnts[index] = gridPnts.Value(i1 + 1, i2 + 1);
                cMyU[index] = uParams.Value(i1);
                cMyV[index] = vParams.Value(i2);
                _theBnd.Add(cMyPnts[index]);
                index++;
            }
        }

        // Compute max deflection using surface adaptor
        double tol = PolyhedronUtils.ComputeMaxDeflection(surface, this, TriangleCount);
        tol *= DEFLECTION_COEFF;

        DeflectionOverEstimation(tol);
        FillBounding();
    }

    /// <summary>The parameters of the point of index <paramref name="index"/>.</summary>
    public void Parameters(int index, out double u, out double v)
    {
        u = C_MyU![index];
        v = C_MyV![index];
    }

    /// <summary>Sets the deflection (at least 1e-4) and enlarges the box by it.</summary>
    public void DeflectionOverEstimation(double flec)
    {
        if (flec < 0.0001)
        {
            _theDeflection = 0.0001;
            _theBnd.Enlarge(0.0001);
        }
        else
        {
            _theDeflection = flec;
            _theBnd.Enlarge(flec);
        }
    }

    /// <summary>The deflection.</summary>
    public double DeflectionOverEstimation() => _theDeflection;

    /// <summary>The box of the polyhedron.</summary>
    public Box Bounding() => _theBnd;

    /// <summary>
    /// Computes the boxes of the triangles, enlarged by the deflection; a triangle with an edge shorter than 1e-7 has
    /// a void box.
    /// </summary>
    public void FillBounding()
    {
        _theComponentsBnd = new NCollection_HArray1<Box>(1, TriangleCount);
        Box boite = new();
        int nbtriangles = TriangleCount;
        for (int iTri = 1; iTri <= nbtriangles; iTri++)
        {
            Triangle(iTri, out int p1, out int p2, out int p3);
            boite.SetVoid();
            Point3D P1 = Point(p1);
            Point3D P2 = Point(p2);
            Point3D P3 = Point(p3);
            if (P1.SquareDistanceTo(P2) > LONGUEUR_MINI_EDGE_TRIANGLE)
            {
                if (P1.SquareDistanceTo(P3) > LONGUEUR_MINI_EDGE_TRIANGLE)
                {
                    if (P2.SquareDistanceTo(P3) > LONGUEUR_MINI_EDGE_TRIANGLE)
                    {
                        boite.Add(P1);
                        boite.Add(P2);
                        boite.Add(P3);
                    }
                }
            }
            boite.Enlarge(_theDeflection);
            _theComponentsBnd.SetValue(iTri, boite);
        }
    }

    /// <summary>The boxes of the triangles (the polyhedron's own array: don't modify).</summary>
    public NCollection_HArray1<Box> ComponentsBounding() => _theComponentsBnd!;

    /// <summary>The number of triangles (2 × nbdU × nbdV).</summary>
    public int TriangleCount => _nbdeltaU * _nbdeltaV * 2;

    /// <summary>The number of points ((nbdU + 1) × (nbdV + 1)).</summary>
    public int PointCount => (_nbdeltaU + 1) * (_nbdeltaV + 1);

    /// <summary>
    /// The triangle connected to <paramref name="triang"/> through the edge from <paramref name="pivot"/> to
    /// <paramref name="pedge"/>, and its third point. On a free edge <paramref name="triCon"/> is 0 and
    /// <paramref name="otherP"/> is the point on the other side of the pivot along the free edge (to turn around a
    /// vertex).
    /// </summary>
    public int TriConnex(int triang, int pivot, int pedge, out int triCon, out int otherP)
    {
        int pivotm1 = pivot - 1;
        int nbdeltaVp1 = _nbdeltaV + 1;
        int nbdeltaVm2 = _nbdeltaV + _nbdeltaV;

        // Pivot position in the MaTriangle :
        int ligP = pivotm1 / nbdeltaVp1;
        int colP = pivotm1 - ligP * nbdeltaVp1;

        // Point sur Edge position in the MaTriangle and edge typ :
        int ligE = 0, colE = 0, typE = 0;
        if (pedge != 0)
        {
            ligE = (pedge - 1) / nbdeltaVp1;
            colE = (pedge - 1) - (ligE * nbdeltaVp1);
            // Horizontal
            if (ligP == ligE)
            {
                typE = 1;
                // Vertical
            }
            else if (colP == colE)
            {
                typE = 2;
                // Oblique
            }
            else
            {
                typE = 3;
            }
        }
        else
        {
            typE = 0;
        }

        // Triangle position General case :
        int linT = 0, colT = 0;
        int linO = 0, colO = 0;
        int t, tt;
        if (triang != 0)
        {
            t = (triang - 1) / (nbdeltaVm2);
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
                case 1: // Horizontal
                    if (linT == ligP)
                    {
                        linT++;
                        linO = ligP + 1;
                        colO = (colP > colE) ? colP : colE; //--colO=Max(colP, colE);
                    }
                    else
                    {
                        linT--;
                        linO = ligP - 1;
                        colO = (colP < colE) ? colP : colE; //--colO=Min(colP, colE);
                    }
                    break;
                case 2: // Vertical
                    if (colT == (colP + colP))
                    {
                        colT++;
                        linO = (ligP > ligE) ? ligP : ligE; //--linO=Max(ligP, ligE);
                        colO = colP + 1;
                    }
                    else
                    {
                        colT--;
                        linO = (ligP < ligE) ? ligP : ligE; //--linO=Min(ligP, ligE);
                        colO = colP - 1;
                    }
                    break;
                case 3: // Oblique
                    if ((colT & 1) == 0)
                    {
                        colT--;
                        linO = (ligP > ligE) ? ligP : ligE; //--linO=Max(ligP, ligE);
                        colO = (colP < colE) ? colP : colE; //--colO=Min(colP, colE);
                    }
                    else
                    {
                        colT++;
                        linO = (ligP < ligE) ? ligP : ligE; //--linO=Min(ligP, ligE);
                        colO = (colP > colE) ? colP : colE; //--colO=Max(colP, colE);
                    }
                    break;
            }
        }
        else
        {
            // Unknown Triangle position :
            if (pedge == 0)
            {
                // Unknown edge :
                linT = (1 > ligP) ? 1 : ligP;                   //--linT=Max(1, ligP);
                colT = (1 > (colP + colP)) ? 1 : (colP + colP); //--colT=Max(1, colP+colP);
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
                // Known edge We take the left or down connectivity :
                switch (typE)
                {
                    case 1: // Horizontal
                        linT = ligP + 1;
                        colT = (colP > colE) ? colP : colE; //--colT=Max(colP,colE);
                        colT += colT;
                        linO = ligP + 1;
                        colO = (colP > colE) ? colP : colE; //--colO=Max(colP,colE);
                        break;
                    case 2: // Vertical
                        linT = (ligP > ligE) ? ligP : ligE; //--linT=Max(ligP, ligE);
                        colT = colP + colP;
                        linO = (ligP < ligE) ? ligP : ligE; //--linO=Min(ligP, ligE);
                        colO = colP - 1;
                        break;
                    case 3: // Oblique
                        linT = (ligP > ligE) ? ligP : ligE; //--linT=Max(ligP, ligE);
                        colT = colP + colE;
                        linO = (ligP > ligE) ? ligP : ligE; //--linO=Max(ligP, ligE);
                        colO = (colP < colE) ? colP : colE; //--colO=Min(colP, colE);
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
            else if (colO > _nbdeltaV)
            {
                colO = _nbdeltaV;
                linO = 1;
            }
            triCon = 0;
        }
        else if (linT > _nbdeltaU)
        {
            linO = _nbdeltaU;
            colO = colP + colP - colE;
            if (colO < 0)
            {
                colO = 0;
                linO = _nbdeltaU - 1;
            }
            else if (colO > _nbdeltaV)
            {
                colO = _nbdeltaV;
                linO = _nbdeltaU - 1;
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
            else if (linO > _nbdeltaU)
            {
                linO = _nbdeltaU;
                colO = 1;
            }
            triCon = 0;
        }
        else if (colT > _nbdeltaV)
        {
            colO = _nbdeltaV;
            linO = ligP + ligP - ligE;
            if (linO < 0)
            {
                linO = 0;
                colO = _nbdeltaV - 1;
            }
            else if (linO > _nbdeltaU)
            {
                linO = _nbdeltaU;
                colO = _nbdeltaV - 1;
            }
            triCon = 0;
        }

        otherP = linO * nbdeltaVp1 + colO + 1;

        //----------------------------------------------------
        //-- Detection des cas ou le triangle retourne est
        //-- invalide. Dans ce cas, on retourne le triangle
        //-- suivant par un nouvel appel a TriConnex.
        //--
        //-- Si En entree : Point(Pivot)==Point(Pedge)
        //-- Alors on retourne OtherP a 0
        //-- et Tricon = Triangle
        //--
        if (pedge != 0 && Point(pivot).SquareDistanceTo(Point(pedge)) <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            otherP = 0;
            triCon = triang;
            return (triCon);
        }
        if (pedge != 0 && Point(otherP).SquareDistanceTo(Point(pedge)) <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            // OCCT: "BUG NON CORRIGE": 0 where the next triangle should be looked up (occt-issues.md, 5D)
            return (0); //-- BUG NON CORRIGE ( a revoir le role de nbdeltaU et nbdeltaV)
        }
        return triCon;
    }

    /// <summary>
    /// The plane equation of the triangle <paramref name="triang"/>: its unit normal and the distance of the plane
    /// from the origin. On a triangle with an edge shorter than 1e-7 the normal is (1, 0, 0) and
    /// <paramref name="polarDistance"/> is left unchanged, as in OCCT.
    /// </summary>
    public void PlaneEquation(int triang, out Xyz normalVector, ref double polarDistance)
    {
        Triangle(triang, out int i1, out int i2, out int i3);

        Xyz pointi1 = Point(i1).XYZ();
        Xyz pointi2 = Point(i2).XYZ();
        Xyz pointi3 = Point(i3).XYZ();

        Xyz v1 = pointi2 - pointi1;
        Xyz v2 = pointi3 - pointi2;
        Xyz v3 = pointi1 - pointi3;

        // OCCT: on a degenerate triangle PolarDistance is left as it was (occt-issues.md, 5D)
        normalVector = new Xyz();
        if (v1.SquareModulus() <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            normalVector.SetCoord(1.0, 0.0, 0.0);
            return;
        }
        if (v2.SquareModulus() <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            normalVector.SetCoord(1.0, 0.0, 0.0);
            return;
        }
        if (v3.SquareModulus() <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            normalVector.SetCoord(1.0, 0.0, 0.0);
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
            polarDistance = normalVector * Point(i1).XYZ();
        }
    }

    /// <summary>True if the triangle <paramref name="triang"/> contains <paramref name="thePnt"/>.</summary>
    public bool Contain(int triang, in Point3D thePnt)
    {
        Triangle(triang, out int i1, out int i2, out int i3);
        Xyz pointi1 = Point(i1).XYZ();
        Xyz pointi2 = Point(i2).XYZ();
        Xyz pointi3 = Point(i3).XYZ();

        Xyz v1 = (pointi2 - pointi1) ^ (thePnt.XYZ() - pointi1);
        Xyz v2 = (pointi3 - pointi2) ^ (thePnt.XYZ() - pointi2);
        Xyz v3 = (pointi1 - pointi3) ^ (thePnt.XYZ() - pointi3);
        return v1 * v2 >= 0.0 && v2 * v3 >= 0.0 && v3 * v1 >= 0.0;
    }

    /// <summary>Does nothing (as in OCCT).</summary>
    public void Dump()
    {
    }

    /// <summary>The numbers of cells in U and in V.</summary>
    public void Size(out int nbdu, out int nbdv)
    {
        nbdu = _nbdeltaU;
        nbdv = _nbdeltaV;
    }

    /// <summary>The three vertex indices of the triangle <paramref name="index"/>.</summary>
    public void Triangle(int index, out int p1, out int p2, out int p3)
    {
        int line = 1 + ((index - 1) / (_nbdeltaV * 2));
        int colon = 1 + ((index - 1) % (_nbdeltaV * 2));
        int colpnt = (colon + 1) / 2;

        // General formula = (line-1)*(nbdeltaV+1)+colpnt

        //  Position of P1 = MesXYZ(line,colpnt);
        p1 = (line - 1) * (_nbdeltaV + 1) + colpnt;

        //  Position of P2= MesXYZ(line+1,colpnt+((colon-1)%2));
        p2 = line * (_nbdeltaV + 1) + colpnt + ((colon - 1) % 2);

        //  Position of P3= MesXYZ(line+(colon%2),colpnt+1);
        p3 = (line - 1 + (colon % 2)) * (_nbdeltaV + 1) + colpnt + 1;
    }

    /// <summary>The point of index <paramref name="index"/> and its parameters.</summary>
    public Point3D Point(int index, out double u, out double v)
    {
        u = C_MyU![index];
        v = C_MyV![index];
        return C_MyPnts![index];
    }

    /// <summary>The point of index <paramref name="index"/>.</summary>
    public Point3D Point(int index) => C_MyPnts![index];

    /// <summary>Does nothing (as in OCCT: "must not be called").</summary>
    public void Point(in Point3D pnt, int lig, int col, double u, double v)
    {
        _ = pnt;
        _ = lig;
        _ = col;
        _ = u;
        _ = v;
    }

    /// <summary>The point of index <paramref name="index"/>.</summary>
    public void Point(int index, out Point3D p) => p = C_MyPnts![index];
}
