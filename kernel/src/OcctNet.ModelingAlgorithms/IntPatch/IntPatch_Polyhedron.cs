// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polyhedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polyhedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;

namespace Occt;

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
public sealed class IntPatch_Polyhedron : IIntCurveSurface_Polyhedron
{
    private const double LONGUEUR_MINI_EDGE_TRIANGLE = 1e-14;
    private const double DEFLECTION_COEFF = 1.1;
    private const int NBMAXUV = 30;

    private Box TheBnd = new();
    private NCollection_HArray1<Box>? TheComponentsBnd;
    private double TheDeflection;
    private readonly int nbdeltaU;
    private readonly int nbdeltaV;
    private Point3D[]? C_MyPnts;
    private double[]? C_MyU;
    private double[]? C_MyV;
#pragma warning disable CS0414 // OCCT sets them; only its undefined Has... getters would read them (see the remarks)
    private readonly bool UMinSingular;
    private readonly bool UMaxSingular;
    private readonly bool VMinSingular;
    private readonly bool VMaxSingular;
#pragma warning restore CS0414

    //================================================================================
    private static int NbPOnU(SurfaceAdaptorBase S)
    {
        double u0 = S.FirstUParameter();
        double u1 = S.LastUParameter();
        int nbpu = IntPatch_HInterTool.NbSamplesU(S, u0, u1);
        return (nbpu > NBMAXUV ? NBMAXUV : nbpu);
    }

    //================================================================================
    private static int NbPOnV(SurfaceAdaptorBase S)
    {
        double v0 = S.FirstVParameter();
        double v1 = S.LastVParameter();
        int nbpv = IntPatch_HInterTool.NbSamplesV(S, v0, v1);
        return (nbpv > NBMAXUV ? NBMAXUV : nbpv);
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
    public IntPatch_Polyhedron(SurfaceAdaptorBase Surface)
    {
        TheDeflection = Epsilon(100.0);
        nbdeltaU = NbPOnU(Surface);
        nbdeltaV = NbPOnV(Surface);
        UMinSingular = IntPatch_HInterTool.SingularOnUMin(Surface);
        UMaxSingular = IntPatch_HInterTool.SingularOnUMax(Surface);
        VMinSingular = IntPatch_HInterTool.SingularOnVMin(Surface);
        VMaxSingular = IntPatch_HInterTool.SingularOnVMax(Surface);
        Init(Surface);
    }

    /// <summary>A polyhedron of the surface with <paramref name="nbu"/> by <paramref name="nbv"/> cells (at least 1 by 1).</summary>
    public IntPatch_Polyhedron(SurfaceAdaptorBase Surface, int nbu, int nbv)
    {
        TheDeflection = Epsilon(100.0);
        nbdeltaU = (nbu < 1 ? 1 : nbu);
        nbdeltaV = (nbv < 1 ? 1 : nbv);
        UMinSingular = IntPatch_HInterTool.SingularOnUMin(Surface);
        UMaxSingular = IntPatch_HInterTool.SingularOnUMax(Surface);
        VMinSingular = IntPatch_HInterTool.SingularOnVMin(Surface);
        VMaxSingular = IntPatch_HInterTool.SingularOnVMax(Surface);
        Init(Surface);
    }

    // The body both of OCCT's constructors repeat verbatim after their member initialisers.
    private void Init(SurfaceAdaptorBase Surface)
    {
        int t = (nbdeltaU + 1) * (nbdeltaV + 1) + 1;
        Point3D[] CMyPnts = new Point3D[t];
        double[] CMyU = new double[t];
        double[] CMyV = new double[t];
        C_MyPnts = CMyPnts;
        C_MyU = CMyU;
        C_MyV = CMyV;

        double u0 = Surface.FirstUParameter();
        double u1 = Surface.LastUParameter();
        double v0 = Surface.FirstVParameter();
        double v1 = Surface.LastVParameter();

        // Build UV parameter arrays
        NCollection_Array1<double> aUParams = new(0, nbdeltaU);
        NCollection_Array1<double> aVParams = new(0, nbdeltaV);
        double U1mU0sNbdeltaU = (u1 - u0) / (double)nbdeltaU;
        double V1mV0sNbdeltaV = (v1 - v0) / (double)nbdeltaV;

        for (int i = 0; i <= nbdeltaU; ++i)
        {
            aUParams.SetValue(i, u0 + i * U1mU0sNbdeltaU);
        }
        for (int j = 0; j <= nbdeltaV; ++j)
        {
            aVParams.SetValue(j, v0 + j * V1mV0sNbdeltaV);
        }

        // Use grid evaluator for batch point evaluation
        SurfaceGrid anEval = new(Surface);
        NCollection_Array2<Point3D> aGridPnts = anEval.EvaluateGrid(aUParams, aVParams);

        // Copy to internal arrays and build bounding box
        int Index = 1;
        for (int i1 = 0; i1 <= nbdeltaU; ++i1)
        {
            for (int i2 = 0; i2 <= nbdeltaV; ++i2)
            {
                CMyPnts[Index] = aGridPnts.Value(i1 + 1, i2 + 1);
                CMyU[Index] = aUParams.Value(i1);
                CMyV[Index] = aVParams.Value(i2);
                TheBnd.Add(CMyPnts[Index]);
                Index++;
            }
        }

        // Compute max deflection using surface adaptor
        double tol = IntCurveSurface_PolyhedronUtils.ComputeMaxDeflection(Surface, this, NbTriangles());
        tol *= DEFLECTION_COEFF;

        DeflectionOverEstimation(tol);
        FillBounding();
    }

    /// <summary>The parameters of the point of index <paramref name="Index"/>.</summary>
    public void Parameters(int Index, out double U, out double V)
    {
        U = C_MyU![Index];
        V = C_MyV![Index];
    }

    /// <summary>Sets the deflection (at least 1e-4) and enlarges the box by it.</summary>
    public void DeflectionOverEstimation(double flec)
    {
        if (flec < 0.0001)
        {
            TheDeflection = 0.0001;
            TheBnd.Enlarge(0.0001);
        }
        else
        {
            TheDeflection = flec;
            TheBnd.Enlarge(flec);
        }
    }

    /// <summary>The deflection.</summary>
    public double DeflectionOverEstimation() => TheDeflection;

    /// <summary>The box of the polyhedron.</summary>
    public Box Bounding() => TheBnd;

    /// <summary>
    /// Computes the boxes of the triangles, enlarged by the deflection; a triangle with an edge shorter than 1e-7 has
    /// a void box.
    /// </summary>
    public void FillBounding()
    {
        TheComponentsBnd = new NCollection_HArray1<Box>(1, NbTriangles());
        Box Boite = new();
        int nbtriangles = NbTriangles();
        for (int iTri = 1; iTri <= nbtriangles; iTri++)
        {
            Triangle(iTri, out int p1, out int p2, out int p3);
            Boite.SetVoid();
            Point3D P1 = Point(p1);
            Point3D P2 = Point(p2);
            Point3D P3 = Point(p3);
            if (P1.SquareDistance(P2) > LONGUEUR_MINI_EDGE_TRIANGLE)
            {
                if (P1.SquareDistance(P3) > LONGUEUR_MINI_EDGE_TRIANGLE)
                {
                    if (P2.SquareDistance(P3) > LONGUEUR_MINI_EDGE_TRIANGLE)
                    {
                        Boite.Add(P1);
                        Boite.Add(P2);
                        Boite.Add(P3);
                    }
                }
            }
            Boite.Enlarge(TheDeflection);
            TheComponentsBnd.SetValue(iTri, Boite);
        }
    }

    /// <summary>The boxes of the triangles (the polyhedron's own array: don't modify).</summary>
    public NCollection_HArray1<Box> ComponentsBounding() => TheComponentsBnd!;

    /// <summary>The number of triangles (2 × nbdU × nbdV).</summary>
    public int NbTriangles() => nbdeltaU * nbdeltaV * 2;

    /// <summary>The number of points ((nbdU + 1) × (nbdV + 1)).</summary>
    public int NbPoints() => (nbdeltaU + 1) * (nbdeltaV + 1);

    /// <summary>
    /// The triangle connected to <paramref name="Triang"/> through the edge from <paramref name="Pivot"/> to
    /// <paramref name="Pedge"/>, and its third point. On a free edge <paramref name="TriCon"/> is 0 and
    /// <paramref name="OtherP"/> is the point on the other side of the pivot along the free edge (to turn around a
    /// vertex).
    /// </summary>
    public int TriConnex(int Triang, int Pivot, int Pedge, out int TriCon, out int OtherP)
    {
        int Pivotm1 = Pivot - 1;
        int nbdeltaVp1 = nbdeltaV + 1;
        int nbdeltaVm2 = nbdeltaV + nbdeltaV;

        // Pivot position in the MaTriangle :
        int ligP = Pivotm1 / nbdeltaVp1;
        int colP = Pivotm1 - ligP * nbdeltaVp1;

        // Point sur Edge position in the MaTriangle and edge typ :
        int ligE = 0, colE = 0, typE = 0;
        if (Pedge != 0)
        {
            ligE = (Pedge - 1) / nbdeltaVp1;
            colE = (Pedge - 1) - (ligE * nbdeltaVp1);
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
        if (Triang != 0)
        {
            t = (Triang - 1) / (nbdeltaVm2);
            tt = (Triang - 1) - t * nbdeltaVm2;
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
            if (Pedge == 0)
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

        TriCon = (linT - 1) * nbdeltaVm2 + colT;

        if (linT < 1)
        {
            linO = 0;
            colO = colP + colP - colE;
            if (colO < 0)
            {
                colO = 0;
                linO = 1;
            }
            else if (colO > nbdeltaV)
            {
                colO = nbdeltaV;
                linO = 1;
            }
            TriCon = 0;
        }
        else if (linT > nbdeltaU)
        {
            linO = nbdeltaU;
            colO = colP + colP - colE;
            if (colO < 0)
            {
                colO = 0;
                linO = nbdeltaU - 1;
            }
            else if (colO > nbdeltaV)
            {
                colO = nbdeltaV;
                linO = nbdeltaU - 1;
            }
            TriCon = 0;
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
            else if (linO > nbdeltaU)
            {
                linO = nbdeltaU;
                colO = 1;
            }
            TriCon = 0;
        }
        else if (colT > nbdeltaV)
        {
            colO = nbdeltaV;
            linO = ligP + ligP - ligE;
            if (linO < 0)
            {
                linO = 0;
                colO = nbdeltaV - 1;
            }
            else if (linO > nbdeltaU)
            {
                linO = nbdeltaU;
                colO = nbdeltaV - 1;
            }
            TriCon = 0;
        }

        OtherP = linO * nbdeltaVp1 + colO + 1;

        //----------------------------------------------------
        //-- Detection des cas ou le triangle retourne est
        //-- invalide. Dans ce cas, on retourne le triangle
        //-- suivant par un nouvel appel a TriConnex.
        //--
        //-- Si En entree : Point(Pivot)==Point(Pedge)
        //-- Alors on retourne OtherP a 0
        //-- et Tricon = Triangle
        //--
        if (Pedge != 0 && Point(Pivot).SquareDistance(Point(Pedge)) <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            OtherP = 0;
            TriCon = Triang;
            return (TriCon);
        }
        if (Pedge != 0 && Point(OtherP).SquareDistance(Point(Pedge)) <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            // OCCT: "BUG NON CORRIGE": 0 where the next triangle should be looked up (occt-issues.md, 5D)
            return (0); //-- BUG NON CORRIGE ( a revoir le role de nbdeltaU et nbdeltaV)
        }
        return TriCon;
    }

    /// <summary>
    /// The plane equation of the triangle <paramref name="Triang"/>: its unit normal and the distance of the plane
    /// from the origin. On a triangle with an edge shorter than 1e-7 the normal is (1, 0, 0) and
    /// <paramref name="PolarDistance"/> is left unchanged, as in OCCT.
    /// </summary>
    public void PlaneEquation(int Triang, out Xyz NormalVector, ref double PolarDistance)
    {
        Triangle(Triang, out int i1, out int i2, out int i3);

        Xyz Pointi1 = Point(i1).XYZ();
        Xyz Pointi2 = Point(i2).XYZ();
        Xyz Pointi3 = Point(i3).XYZ();

        Xyz v1 = Pointi2 - Pointi1;
        Xyz v2 = Pointi3 - Pointi2;
        Xyz v3 = Pointi1 - Pointi3;

        // OCCT: on a degenerate triangle PolarDistance is left as it was (occt-issues.md, 5D)
        NormalVector = new Xyz();
        if (v1.SquareModulus() <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            NormalVector.SetCoord(1.0, 0.0, 0.0);
            return;
        }
        if (v2.SquareModulus() <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            NormalVector.SetCoord(1.0, 0.0, 0.0);
            return;
        }
        if (v3.SquareModulus() <= LONGUEUR_MINI_EDGE_TRIANGLE)
        {
            NormalVector.SetCoord(1.0, 0.0, 0.0);
            return;
        }

        NormalVector = (v1 ^ v2) + (v2 ^ v3) + (v3 ^ v1);
        double aNormLen = NormalVector.Modulus();
        if (aNormLen < Constants.Resolution())
        {
            PolarDistance = 0.0;
        }
        else
        {
            NormalVector.Divide(aNormLen);
            PolarDistance = NormalVector * Point(i1).XYZ();
        }
    }

    /// <summary>True if the triangle <paramref name="Triang"/> contains <paramref name="ThePnt"/>.</summary>
    public bool Contain(int Triang, in Point3D ThePnt)
    {
        Triangle(Triang, out int i1, out int i2, out int i3);
        Xyz Pointi1 = Point(i1).XYZ();
        Xyz Pointi2 = Point(i2).XYZ();
        Xyz Pointi3 = Point(i3).XYZ();

        Xyz v1 = (Pointi2 - Pointi1) ^ (ThePnt.XYZ() - Pointi1);
        Xyz v2 = (Pointi3 - Pointi2) ^ (ThePnt.XYZ() - Pointi2);
        Xyz v3 = (Pointi1 - Pointi3) ^ (ThePnt.XYZ() - Pointi3);
        return v1 * v2 >= 0.0 && v2 * v3 >= 0.0 && v3 * v1 >= 0.0;
    }

    /// <summary>Does nothing (as in OCCT).</summary>
    public void Dump()
    {
    }

    /// <summary>The numbers of cells in U and in V.</summary>
    public void Size(out int nbdu, out int nbdv)
    {
        nbdu = nbdeltaU;
        nbdv = nbdeltaV;
    }

    /// <summary>The three vertex indices of the triangle <paramref name="Index"/>.</summary>
    public void Triangle(int Index, out int P1, out int P2, out int P3)
    {
        int line = 1 + ((Index - 1) / (nbdeltaV * 2));
        int colon = 1 + ((Index - 1) % (nbdeltaV * 2));
        int colpnt = (colon + 1) / 2;

        // General formula = (line-1)*(nbdeltaV+1)+colpnt

        //  Position of P1 = MesXYZ(line,colpnt);
        P1 = (line - 1) * (nbdeltaV + 1) + colpnt;

        //  Position of P2= MesXYZ(line+1,colpnt+((colon-1)%2));
        P2 = line * (nbdeltaV + 1) + colpnt + ((colon - 1) % 2);

        //  Position of P3= MesXYZ(line+(colon%2),colpnt+1);
        P3 = (line - 1 + (colon % 2)) * (nbdeltaV + 1) + colpnt + 1;
    }

    /// <summary>The point of index <paramref name="Index"/> and its parameters.</summary>
    public Point3D Point(int Index, out double U, out double V)
    {
        U = C_MyU![Index];
        V = C_MyV![Index];
        return C_MyPnts![Index];
    }

    /// <summary>The point of index <paramref name="Index"/>.</summary>
    public Point3D Point(int Index) => C_MyPnts![Index];

    /// <summary>Does nothing (as in OCCT: "must not be called").</summary>
    public void Point(in Point3D thePnt, int lig, int col, double U, double V)
    {
        _ = thePnt;
        _ = lig;
        _ = col;
        _ = U;
        _ = V;
    }

    /// <summary>The point of index <paramref name="Index"/>.</summary>
    public void Point(int Index, out Point3D P) => P = C_MyPnts![Index];
}
