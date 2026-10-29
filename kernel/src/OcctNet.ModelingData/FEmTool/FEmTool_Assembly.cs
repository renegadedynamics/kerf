// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_Assembly.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_Assembly.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Assembles and solves the system of a (one-dimensional) finite element problem: minimise
/// <c>1/2 tX H X - tB X</c> under the linear constraints <c>G X = C</c>.
/// </summary>
/// <remarks>
/// <para>
/// <c>Table(dimension, element)</c> maps the local variables of an element in a dimension to global variable
/// indices; <c>Dependence(dim1, dim2)</c> is non-zero where two dimensions are coupled. A working object
/// (decision 21) with OCCT's public mutators.
/// </para>
/// <para>
/// <see cref="AssemblyTable"/> returns the table given to the constructor, which the assembly keeps (shared, as
/// OCCT keeps the handle).
/// </para>
/// </remarks>
public class FEmTool_Assembly
{
    private readonly NCollection_Array2<int> myDepTable;
    private readonly NCollection_HArray2<NCollection_HArray1<int>> myRefTable;
    private bool IsSolved;
    private readonly FEmTool_ProfileMatrix H;
    private readonly math_Vector B;
    private FEmTool_ProfileMatrix? GHGt;
    private readonly NCollection_Sequence<NCollection_List<NCollection_HArray1<double>>> G = new();
    private readonly NCollection_Sequence<double> C = new();

    private static int MinIndex(NCollection_HArray2<NCollection_HArray1<int>> Table)
    {
        int dim, el, nvar, Imin;
        int diml = Table.LowerRow(), dimu = Table.UpperRow(), ell = Table.LowerCol(), elu = Table.UpperCol(), nvarl,
            nvaru;

        NCollection_HArray1<int> T = Table.Value(diml, ell);

        nvarl = T.Lower();

        Imin = T.Value(nvarl);

        for (dim = diml; dim <= dimu; dim++)
        {
            for (el = ell; el <= elu; el++)
            {
                T = Table.Value(dim, el);
                nvarl = T.Lower();
                nvaru = T.Upper();
                for (nvar = nvarl; nvar <= nvaru; nvar++)
                {
                    Imin = Min(Imin, T.Value(nvar));
                }
            }
        }
        return Imin;
    }

    private static int MaxIndex(NCollection_HArray2<NCollection_HArray1<int>> Table)
    {
        int dim, el, nvar, Imax;
        int diml = Table.LowerRow(), dimu = Table.UpperRow(), ell = Table.LowerCol(), elu = Table.UpperCol(), nvarl,
            nvaru;

        NCollection_HArray1<int> T = Table.Value(diml, ell);

        nvarl = T.Lower();

        Imax = T.Value(nvarl);

        for (dim = diml; dim <= dimu; dim++)
        {
            for (el = ell; el <= elu; el++)
            {
                T = Table.Value(dim, el);
                nvarl = T.Lower();
                nvaru = T.Upper();
                for (nvar = nvarl; nvar <= nvaru; nvar++)
                {
                    Imax = Max(Imax, T.Value(nvar));
                }
            }
        }
        return Imax;
    }

    /// <summary>
    /// Prepares the assembly for the dependence table <paramref name="Dependence"/> (copied) and the assembly
    /// table <paramref name="Table"/> (kept).
    /// </summary>
    public FEmTool_Assembly(NCollection_Array2<int> Dependence, NCollection_HArray2<NCollection_HArray1<int>> Table)
    {
        myDepTable = new NCollection_Array2<int>(1, Dependence.ColLength(), 1, Dependence.RowLength());
        B = new math_Vector(MinIndex(Table), MaxIndex(Table));
        IsSolved = false;
        myDepTable.Assign(Dependence);
        myRefTable = Table;

        NCollection_Array1<int> FirstIndexes = new(1, B.Length());
        FirstIndexes.Init(B.Length());

        int dim, el, nvar, Imin, I0 = 1 - B.Lower(), i;

        int diml = Table.LowerRow(), dimu = Table.UpperRow(), ell = Table.LowerCol(), elu = Table.UpperCol(), nvarl,
            nvaru;

        NCollection_HArray1<int> T;
        for (dim = diml; dim <= dimu; dim++)
        {
            for (el = ell; el <= elu; el++)
            {
                T = Table.Value(dim, el);
                nvarl = T.Lower();
                nvaru = T.Upper();
                Imin = T.Value(nvarl) + I0;

                for (nvar = nvarl; nvar <= nvaru; nvar++)
                {
                    Imin = Min(Imin, T.Value(nvar) + I0);
                }

                for (nvar = nvarl; nvar <= nvaru; nvar++)
                {
                    i = T.Value(nvar) + I0;
                    FirstIndexes[i] = Min(FirstIndexes[i], Imin);
                }
            }
        }

        H = new FEmTool_ProfileMatrix(FirstIndexes);

        NullifyMatrix();
        NullifyVector();
    }

    /// <summary>Sets every coefficient of the matrix to zero.</summary>
    public void NullifyMatrix()
    {
        H.Init(0.0);
        IsSolved = false;
    }

    /// <summary>
    /// Adds an elementary matrix to the assembly matrix. Throws Standard_DomainError if dimensions
    /// <paramref name="Dimension1"/> and <paramref name="Dimension2"/> are independent.
    /// </summary>
    public void AddMatrix(int Element, int Dimension1, int Dimension2, math_Matrix Mat)
    {
        if (myDepTable[Dimension1, Dimension2] == 0)
        {
            throw new Standard_DomainError("FEmTool_Assembly::AddMatrix");
        }

        NCollection_Array1<int> T1 = myRefTable.Value(Dimension1, Element).Array1();
        NCollection_Array1<int> T2 = myRefTable.Value(Dimension2, Element).Array1();

        int nvarl = T1.Lower(), nvaru = Min(T1.Upper(), nvarl + Mat.RowNumber() - 1);

        int I, J, I0 = 1 - B.Lower(), i, ii, j,

            i0 = Mat.LowerRow() - nvarl, j0 = Mat.LowerCol() - nvarl;

        for (i = nvarl; i <= nvaru; i++)
        {
            I = T1[i] + I0;
            ii = i0 + i;
            for (j = nvarl; j <= i; j++)
            {
                J = T2[j] + I0;
                H.ChangeValue(I, J) += Mat[ii, j0 + j];
            }
        }

        IsSolved = false;
    }

    /// <summary>Sets every coordinate of the second member to zero.</summary>
    public void NullifyVector()
    {
        B.Init(0.0);
    }

    /// <summary>Adds an elementary vector to the second member.</summary>
    public void AddVector(int Element, int Dimension, math_Vector Vec)
    {
        NCollection_Array1<int> T = myRefTable.Value(Dimension, Element).Array1();
        int nvarl = T.Lower(), nvaru = Min(T.Upper(), nvarl + Vec.Length() - 1), i0 = Vec.Lower() - nvarl;

        int i;

        for (i = nvarl; i <= nvaru; i++)
        {
            B[T[i]] += Vec[i0 + i];
        }
    }

    /// <summary>Solves the assembled system; false if it fails.</summary>
    public bool Solve()
    {
        IsSolved = H.Decompose();

        if (!G.IsEmpty() && IsSolved)
        {
            // calculating H-1 Gt
            math_Vector gi = new(B.Lower(), B.Upper()), qi = new(B.Lower(), B.Upper());
            // OCCT declares i, j and k in both blocks below; C# needs one declaration
            int i, j, k;

            if (GHGt is null || GHGt.RowNumber() != G.Length())
            {
                NCollection_Array1<int> FirstIndexes = new(1, G.Length());
                //-----------------------------------------------------------------
                NCollection_Array2<int> H1 = new(1, NbGlobVar(), 1, NbGlobVar());
                H1.Init(1);
                int l, BlockBeg = 1, BlockEnd;
                bool Block, Zero;
                for (i = 2; i <= NbGlobVar(); i++)
                {
                    BlockEnd = i - 1;
                    if (!H.IsInProfile(i, BlockEnd))
                    {
                        // Maybe, begin of block
                        Block = true;
                        for (j = i + 1; j <= NbGlobVar(); j++)
                        {
                            if (H.IsInProfile(j, BlockEnd))
                            {
                                Block = false;
                                break;
                            }
                        }
                        if (Block)
                        {
                            for (j = i; j <= NbGlobVar(); j++)
                            {
                                for (k = BlockBeg; k <= BlockEnd; k++)
                                {
                                    H1[j, k] = 0;
                                    H1[k, j] = 0;
                                }
                            }
                            BlockBeg = BlockEnd + 1;
                        }
                        else
                        {
                            i = j;
                        }
                    }
                }

                NCollection_List<NCollection_HArray1<double>>.Iterator Iter1 = new();
                NCollection_List<NCollection_HArray1<double>>.Iterator Iter2 = new();
                for (i = 1; i <= G.Length(); i++)
                {
                    NCollection_List<NCollection_HArray1<double>> Gi = G.Value(i);
                    for (j = 1; j <= i; j++)
                    {
                        NCollection_List<NCollection_HArray1<double>> Gj = G.Value(j);
                        Zero = true;
                        for (Iter1.Initialize(Gi); Iter1.More(); Iter1.Next())
                        {
                            NCollection_HArray1<double> a = Iter1.Value();
                            for (k = a.Lower(); k <= a.Upper(); k++)
                            {
                                for (Iter2.Initialize(Gj); Iter2.More(); Iter2.Next())
                                {
                                    NCollection_HArray1<double> b = Iter2.Value();
                                    for (l = b.Lower(); l <= b.Upper(); l++)
                                    {
                                        if (H1[k, l] != 0)
                                        {
                                            Zero = false;
                                            break;
                                        }
                                    }
                                    if (!Zero)
                                    {
                                        break;
                                    }
                                }
                                if (!Zero)
                                {
                                    break;
                                }
                            }
                            if (!Zero)
                            {
                                break;
                            }
                        }
                        if (!Zero)
                        {
                            FirstIndexes[i] = j;
                            break;
                        }
                    }
                }
                //-----------------------------------------------------------------------
                GHGt = new FEmTool_ProfileMatrix(FirstIndexes);
            }

            GHGt.Init(0.0);
            NCollection_List<NCollection_HArray1<double>>.Iterator Iter = new();

            for (i = 1; i <= G.Length(); i++)
            {
                NCollection_List<NCollection_HArray1<double>> L = G.Value(i);
                gi.Init(0.0);
                // preparing i-th line of G (or column of Gt)
                for (Iter.Initialize(L); Iter.More(); Iter.Next())
                {
                    NCollection_HArray1<double> a = Iter.Value();
                    for (j = a.Lower(); j <= a.Upper(); j++)
                    {
                        gi[j] = a.Value(j); // gi - full line of G
                    }
                }
                //                                     -1 t
                H.Solve(gi, qi); // solving H*qi = gi, qi is column of H  G

                //                               -1 t
                // Calculation of product M = G H G
                // for each i all elements of i-th column of M are calculated for k >= i
                for (k = i; k <= G.Length(); k++)
                {
                    if (GHGt.IsInProfile(k, i))
                    {
                        double m = 0.0; // m = M(k,i)
                        NCollection_List<NCollection_HArray1<double>> aL = G.Value(k);
                        for (Iter.Initialize(aL); Iter.More(); Iter.Next())
                        {
                            NCollection_HArray1<double> a = Iter.Value();
                            for (j = a.Lower(); j <= a.Upper(); j++)
                            {
                                m += qi[j] * a.Value(j); // scalar product of
                            }
                            // k-th line of G and i-th column of H-1 Gt
                        }

                        GHGt.ChangeValue(k, i) = m;
                    }
                }
            }

            IsSolved = GHGt.Decompose();
        }

        return IsSolved;
    }

    /// <summary>The solution. Throws StdFail_NotDone if <see cref="Solve"/> did not succeed.</summary>
    public void Solution(math_Vector Solution)
    {
        if (!IsSolved)
        {
            throw new StdFail_NotDone("FEmTool_Assembly::Solution");
        }

        if (G.IsEmpty())
        {
            H.Solve(B, Solution);
        }
        else
        {
            math_Vector v1 = new(B.Lower(), B.Upper());
            H.Solve(B, v1);

            math_Vector l = new(1, G.Length()), v2 = new(1, G.Length());
            int i, j;
            NCollection_List<NCollection_HArray1<double>>.Iterator Iter = new();

            for (i = 1; i <= G.Length(); i++)
            {
                NCollection_List<NCollection_HArray1<double>> L = G.Value(i);
                double m = 0.0;
                for (Iter.Initialize(L); Iter.More(); Iter.Next())
                {
                    NCollection_HArray1<double> a = Iter.Value();
                    for (j = a.Lower(); j <= a.Upper(); j++)
                    {
                        m += v1[j] * a.Value(j); // scalar product
                    }
                    // G v1
                }
                v2[i] = m - C.Value(i);
            }

            GHGt!.Solve(v2, l); // Solving M*l = v2

            v1.Initialized(B);
            // Calculation v1 = B-Gt*l
            // v1(j) = B(j) - Gt(j,i)*l(i) = B(j) - G(i,j)*l(i)
            //
            for (i = 1; i <= G.Length(); i++)
            {
                NCollection_List<NCollection_HArray1<double>> L = G.Value(i);
                for (Iter.Initialize(L); Iter.More(); Iter.Next())
                {
                    NCollection_HArray1<double> a = Iter.Value();
                    for (j = a.Lower(); j <= a.Upper(); j++)
                    {
                        v1[j] -= l[i] * a.Value(j);
                    }
                }
            }
            H.Solve(v1, Solution);
        }
    }

    /// <summary>The number of global variables.</summary>
    public int NbGlobVar() => B.Length();

    /// <summary>The assembly table mapping element-local indices to global indices (shared: don't modify).</summary>
    public NCollection_HArray2<NCollection_HArray1<int>> AssemblyTable() => myRefTable;

    /// <summary>The assembly table, through an output parameter (deprecated in OCCT: use <see cref="AssemblyTable"/>).</summary>
    public void GetAssemblyTable(out NCollection_HArray2<NCollection_HArray1<int>> AssTable)
    {
        AssTable = AssemblyTable();
    }

    /// <summary>Deletes all constraints.</summary>
    public void ResetConstraint()
    {
        G.Clear();
        C.Clear();
    }

    /// <summary>Sets every constraint coefficient and value to zero.</summary>
    public void NullifyConstraint()
    {
        NCollection_List<NCollection_HArray1<double>>.Iterator Iter = new();
        int i;

        for (i = 1; i <= G.Length(); i++)
        {
            C.SetValue(i, 0.0);
            for (Iter.Initialize(G.Value(i)); Iter.More(); Iter.Next())
            {
                Iter.Value().Init(0.0);
            }
        }
    }

    /// <summary>
    /// Adds the linear form <paramref name="LinearForm"/> of element <paramref name="Element"/> in dimension
    /// <paramref name="Dimension"/> to constraint <paramref name="IndexofConstraint"/>, and
    /// <paramref name="Value"/> to its value.
    /// </summary>
    public void AddConstraint(int IndexofConstraint, int Element, int Dimension, math_Vector LinearForm, double Value)
    {
        while (G.Length() < IndexofConstraint)
        {
            // Add new lines in G
            NCollection_List<NCollection_HArray1<double>> L1 = new();
            G.Append(L1);
            C.Append(0.0);
        }

        NCollection_List<NCollection_HArray1<double>> L = G.ChangeValue(IndexofConstraint);

        NCollection_HArray1<int> Indexes = myRefTable.Value(Dimension, Element);
        int i, Imax = 0, Imin = NbGlobVar();

        for (i = Indexes.Lower(); i <= Indexes.Upper(); i++)
        {
            Imin = Min(Imin, Indexes.Value(i));
            Imax = Max(Imax, Indexes.Value(i));
        }

        // OCCT dereferences a null Coeff (crash) when the list is not empty and the new range is wider than
        // every segment on both sides; the port throws a NullReferenceException there
        NCollection_HArray1<double>? Coeff = null;

        if (L.IsEmpty())
        {
            Coeff = new NCollection_HArray1<double>(Imin, Imax);
            Coeff.Init(0.0);
            L.Append(Coeff);
        }
        else
        {
            NCollection_List<NCollection_HArray1<double>>.Iterator Iter = new(L);
            // OCCT declares the segment ranks s1 and s2 as doubles
            double s1 = 0, s2 = 0;
            NCollection_HArray1<double>? Aux1 = null, Aux2 = null;
            for (i = 1; Iter.More(); Iter.Next(), i++)
            {
                if (Imin >= Iter.Value().Lower())
                {
                    s1 = i;
                    Aux1 = Iter.Value();
                    if (Imax <= Iter.Value().Upper())
                    {
                        s2 = s1;
                        Coeff = Iter.Value();
                        break;
                    }
                }

                if (Imax <= Iter.Value().Upper())
                {
                    s2 = i;
                    Aux2 = Iter.Value();
                }
            }

            if (s1 != s2)
            {
                if (s1 == 0)
                {
                    if (Imax < Aux2!.Lower())
                    {
                        // inserting before first segment
                        Coeff = new NCollection_HArray1<double>(Imin, Imax);
                        Coeff.Init(0.0);
                        L.Prepend(Coeff);
                    }
                    else
                    {
                        // merge new and first segment
                        Coeff = new NCollection_HArray1<double>(Imin, Aux2.Upper());
                        for (i = Imin; i <= Aux2.Lower() - 1; i++)
                        {
                            Coeff.SetValue(i, 0.0);
                        }
                        for (i = Aux2.Lower(); i <= Aux2.Upper(); i++)
                        {
                            Coeff.SetValue(i, Aux2.Value(i));
                        }
                        L.ChangeFirst() = Coeff;
                    }
                }
                else if (s2 == 0)
                {
                    if (Imin > Aux1!.Upper())
                    {
                        // append new
                        Coeff = new NCollection_HArray1<double>(Imin, Imax);
                        Coeff.Init(0.0);
                        L.Append(Coeff);
                    }
                    else
                    {
                        // merge new and last segment
                        Coeff = new NCollection_HArray1<double>(Aux1.Lower(), Imax);
                        for (i = Aux1.Lower(); i <= Aux1.Upper(); i++)
                        {
                            Coeff.SetValue(i, Aux1.Value(i));
                        }
                        for (i = Aux1.Upper() + 1; i <= Imax; i++)
                        {
                            Coeff.SetValue(i, 0.0);
                        }
                        L.ChangeLast() = Coeff;
                    }
                }
                else if (Imin <= Aux1!.Upper() && Imax < Aux2!.Lower())
                {
                    // merge s1 and new
                    Coeff = new NCollection_HArray1<double>(Aux1.Lower(), Imax);
                    for (i = Aux1.Lower(); i <= Aux1.Upper(); i++)
                    {
                        Coeff.SetValue(i, Aux1.Value(i));
                    }
                    for (i = Aux1.Upper() + 1; i <= Imax; i++)
                    {
                        Coeff.SetValue(i, 0.0);
                    }
                    Iter.Initialize(L);
                    for (i = 1; i < s1; Iter.Next(), i++)
                    {
                        ;
                    }
                    Iter.ChangeValue() = Coeff;
                }
                else if (Imin > Aux1.Upper() && Imax >= Aux2!.Lower())
                {
                    // merge new and first segment
                    Coeff = new NCollection_HArray1<double>(Imin, Aux2.Upper());
                    for (i = Imin; i <= Aux2.Lower() - 1; i++)
                    {
                        Coeff.SetValue(i, 0.0);
                    }
                    for (i = Aux2.Lower(); i <= Aux2.Upper(); i++)
                    {
                        Coeff.SetValue(i, Aux2.Value(i));
                    }
                    Iter.Initialize(L);
                    for (i = 1; i < s2; Iter.Next(), i++)
                    {
                        ;
                    }
                    Iter.ChangeValue() = Coeff;
                }
                else if (Imin > Aux1.Upper() && Imax < Aux2!.Lower())
                {
                    // inserting new between s1 and s2
                    Coeff = new NCollection_HArray1<double>(Imin, Imax);
                    Coeff.Init(0.0);
                    Iter.Initialize(L);
                    for (i = 1; i < s1; Iter.Next(), i++)
                    {
                        ;
                    }
                    L.InsertAfter(Coeff, Iter);
                }
                else
                {
                    // merge s1, new, s2 and remove s2
                    Coeff = new NCollection_HArray1<double>(Aux1.Lower(), Aux2!.Upper());
                    for (i = Aux1.Lower(); i <= Aux1.Upper(); i++)
                    {
                        Coeff.SetValue(i, Aux1.Value(i));
                    }
                    for (i = Aux1.Upper() + 1; i <= Aux2.Lower() - 1; i++)
                    {
                        Coeff.SetValue(i, 0.0);
                    }
                    for (i = Aux2.Lower(); i <= Aux2.Upper(); i++)
                    {
                        Coeff.SetValue(i, Aux2.Value(i));
                    }
                    Iter.Initialize(L);
                    for (i = 1; i < s1; Iter.Next(), i++)
                    {
                        ;
                    }
                    Iter.ChangeValue() = Coeff;
                    Iter.Next();
                    L.Remove(Iter);
                }
            }
        }

        // adding
        int j = LinearForm.Lower();
        for (i = Indexes.Lower(); i <= Indexes.Upper(); i++, j++)
        {
            Coeff!.ChangeValue(Indexes.Value(i)) += LinearForm[j];
        }

        C.ChangeValue(IndexofConstraint) += Value;
    }
}
