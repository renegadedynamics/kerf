// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_Assembly.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_Assembly.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

namespace Kerf.Engine.Approximation.FiniteElements;

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
internal class Assembly
{
    private readonly NCollection_Array2<int> _depTable;
    private readonly NCollection_HArray2<NCollection_HArray1<int>> _refTable;
    private bool _isSolved;
    private readonly FemProfileMatrix _h;
    private readonly math_Vector _b;
    private FemProfileMatrix? _ghGt;
    private readonly NCollection_Sequence<NCollection_List<NCollection_HArray1<double>>> _g = new();
    private readonly NCollection_Sequence<double> _c = new();

    private static int MinIndex(NCollection_HArray2<NCollection_HArray1<int>> table)
    {
        int dim, el, nvar, imin;
        int diml = table.LowerRow(), dimu = table.UpperRow(), ell = table.LowerCol(), elu = table.UpperCol(), nvarl,
            nvaru;

        NCollection_HArray1<int> t = table.Value(diml, ell);

        nvarl = t.Lower();

        imin = t.Value(nvarl);

        for (dim = diml; dim <= dimu; dim++)
        {
            for (el = ell; el <= elu; el++)
            {
                t = table.Value(dim, el);
                nvarl = t.Lower();
                nvaru = t.Upper();
                for (nvar = nvarl; nvar <= nvaru; nvar++)
                {
                    imin = Min(imin, t.Value(nvar));
                }
            }
        }
        return imin;
    }

    private static int MaxIndex(NCollection_HArray2<NCollection_HArray1<int>> table)
    {
        int dim, el, nvar, imax;
        int diml = table.LowerRow(), dimu = table.UpperRow(), ell = table.LowerCol(), elu = table.UpperCol(), nvarl,
            nvaru;

        NCollection_HArray1<int> t = table.Value(diml, ell);

        nvarl = t.Lower();

        imax = t.Value(nvarl);

        for (dim = diml; dim <= dimu; dim++)
        {
            for (el = ell; el <= elu; el++)
            {
                t = table.Value(dim, el);
                nvarl = t.Lower();
                nvaru = t.Upper();
                for (nvar = nvarl; nvar <= nvaru; nvar++)
                {
                    imax = Max(imax, t.Value(nvar));
                }
            }
        }
        return imax;
    }

    /// <summary>
    /// Prepares the assembly for the dependence table <paramref name="dependence"/> (copied) and the assembly
    /// table <paramref name="table"/> (kept).
    /// </summary>
    public Assembly(NCollection_Array2<int> dependence, NCollection_HArray2<NCollection_HArray1<int>> table)
    {
        _depTable = new NCollection_Array2<int>(1, dependence.ColLength(), 1, dependence.RowLength());
        _b = new math_Vector(MinIndex(table), MaxIndex(table));
        _isSolved = false;
        _depTable.Assign(dependence);
        _refTable = table;

        NCollection_Array1<int> firstIndexes = new(1, _b.Length());
        firstIndexes.Init(_b.Length());

        int dim, el, nvar, imin, i0 = 1 - _b.Lower(), i;

        int diml = table.LowerRow(), dimu = table.UpperRow(), ell = table.LowerCol(), elu = table.UpperCol(), nvarl,
            nvaru;

        NCollection_HArray1<int> t;
        for (dim = diml; dim <= dimu; dim++)
        {
            for (el = ell; el <= elu; el++)
            {
                t = table.Value(dim, el);
                nvarl = t.Lower();
                nvaru = t.Upper();
                imin = t.Value(nvarl) + i0;

                for (nvar = nvarl; nvar <= nvaru; nvar++)
                {
                    imin = Min(imin, t.Value(nvar) + i0);
                }

                for (nvar = nvarl; nvar <= nvaru; nvar++)
                {
                    i = t.Value(nvar) + i0;
                    firstIndexes[i] = Min(firstIndexes[i], imin);
                }
            }
        }

        _h = new FemProfileMatrix(firstIndexes);

        NullifyMatrix();
        NullifyVector();
    }

    /// <summary>Sets every coefficient of the matrix to zero.</summary>
    public void NullifyMatrix()
    {
        _h.Init(0.0);
        _isSolved = false;
    }

    /// <summary>
    /// Adds an elementary matrix to the assembly matrix. Throws Standard_DomainError if dimensions
    /// <paramref name="dimension1"/> and <paramref name="dimension2"/> are independent.
    /// </summary>
    public void AddMatrix(int element, int dimension1, int dimension2, Matrix mat)
    {
        if (_depTable[dimension1, dimension2] == 0)
        {
            throw new DomainErrorException("FEmTool_Assembly::AddMatrix");
        }

        NCollection_Array1<int> t1 = _refTable.Value(dimension1, element).Array1();
        NCollection_Array1<int> t2 = _refTable.Value(dimension2, element).Array1();

        int nvarl = t1.Lower(), nvaru = Min(t1.Upper(), nvarl + mat.RowNumber() - 1);

        int I, J, I0 = 1 - _b.Lower(), i, ii, j,

            i0 = mat.LowerRow() - nvarl, j0 = mat.LowerCol() - nvarl;

        for (i = nvarl; i <= nvaru; i++)
        {
            I = t1[i] + I0;
            ii = i0 + i;
            for (j = nvarl; j <= i; j++)
            {
                J = t2[j] + I0;
                _h.ChangeValue(I, J) += mat[ii, j0 + j];
            }
        }

        _isSolved = false;
    }

    /// <summary>Sets every coordinate of the second member to zero.</summary>
    public void NullifyVector()
    {
        _b.Init(0.0);
    }

    /// <summary>Adds an elementary vector to the second member.</summary>
    public void AddVector(int element, int dimension, math_Vector vec)
    {
        NCollection_Array1<int> t = _refTable.Value(dimension, element).Array1();
        int nvarl = t.Lower(), nvaru = Min(t.Upper(), nvarl + vec.Length() - 1), i0 = vec.Lower() - nvarl;

        int i;

        for (i = nvarl; i <= nvaru; i++)
        {
            _b[t[i]] += vec[i0 + i];
        }
    }

    /// <summary>Solves the assembled system; false if it fails.</summary>
    public bool Solve()
    {
        _isSolved = _h.Decompose();

        if (!_g.IsEmpty() && _isSolved)
        {
            // calculating H-1 Gt
            math_Vector gi = new(_b.Lower(), _b.Upper()), qi = new(_b.Lower(), _b.Upper());
            // OCCT declares i, j and k in both blocks below; C# needs one declaration
            int i, j, k;

            if (_ghGt is null || _ghGt.RowNumber() != _g.Length())
            {
                NCollection_Array1<int> firstIndexes = new(1, _g.Length());
                //-----------------------------------------------------------------
                NCollection_Array2<int> h1 = new(1, GlobVarCount, 1, GlobVarCount);
                h1.Init(1);
                int l, blockBeg = 1, blockEnd;
                bool block, zero;
                for (i = 2; i <= GlobVarCount; i++)
                {
                    blockEnd = i - 1;
                    if (!_h.IsInProfile(i, blockEnd))
                    {
                        // Maybe, begin of block
                        block = true;
                        for (j = i + 1; j <= GlobVarCount; j++)
                        {
                            if (_h.IsInProfile(j, blockEnd))
                            {
                                block = false;
                                break;
                            }
                        }
                        if (block)
                        {
                            for (j = i; j <= GlobVarCount; j++)
                            {
                                for (k = blockBeg; k <= blockEnd; k++)
                                {
                                    h1[j, k] = 0;
                                    h1[k, j] = 0;
                                }
                            }
                            blockBeg = blockEnd + 1;
                        }
                        else
                        {
                            i = j;
                        }
                    }
                }

                NCollection_List<NCollection_HArray1<double>>.Iterator iter1 = new();
                NCollection_List<NCollection_HArray1<double>>.Iterator iter2 = new();
                for (i = 1; i <= _g.Length(); i++)
                {
                    NCollection_List<NCollection_HArray1<double>> Gi = _g.Value(i);
                    for (j = 1; j <= i; j++)
                    {
                        NCollection_List<NCollection_HArray1<double>> gj = _g.Value(j);
                        zero = true;
                        for (iter1.Initialize(Gi); iter1.More(); iter1.Next())
                        {
                            NCollection_HArray1<double> a = iter1.Value();
                            for (k = a.Lower(); k <= a.Upper(); k++)
                            {
                                for (iter2.Initialize(gj); iter2.More(); iter2.Next())
                                {
                                    NCollection_HArray1<double> b = iter2.Value();
                                    for (l = b.Lower(); l <= b.Upper(); l++)
                                    {
                                        if (h1[k, l] != 0)
                                        {
                                            zero = false;
                                            break;
                                        }
                                    }
                                    if (!zero)
                                    {
                                        break;
                                    }
                                }
                                if (!zero)
                                {
                                    break;
                                }
                            }
                            if (!zero)
                            {
                                break;
                            }
                        }
                        if (!zero)
                        {
                            firstIndexes[i] = j;
                            break;
                        }
                    }
                }
                //-----------------------------------------------------------------------
                _ghGt = new FemProfileMatrix(firstIndexes);
            }

            _ghGt.Init(0.0);
            NCollection_List<NCollection_HArray1<double>>.Iterator iter = new();

            for (i = 1; i <= _g.Length(); i++)
            {
                NCollection_List<NCollection_HArray1<double>> L = _g.Value(i);
                gi.Init(0.0);
                // preparing i-th line of G (or column of Gt)
                for (iter.Initialize(L); iter.More(); iter.Next())
                {
                    NCollection_HArray1<double> a = iter.Value();
                    for (j = a.Lower(); j <= a.Upper(); j++)
                    {
                        gi[j] = a.Value(j); // gi - full line of G
                    }
                }
                //                                     -1 t
                _h.Solve(gi, qi); // solving H*qi = gi, qi is column of H  G

                //                               -1 t
                // Calculation of product M = G H G
                // for each i all elements of i-th column of M are calculated for k >= i
                for (k = i; k <= _g.Length(); k++)
                {
                    if (_ghGt.IsInProfile(k, i))
                    {
                        double m = 0.0; // m = M(k,i)
                        NCollection_List<NCollection_HArray1<double>> aL = _g.Value(k);
                        for (iter.Initialize(aL); iter.More(); iter.Next())
                        {
                            NCollection_HArray1<double> a = iter.Value();
                            for (j = a.Lower(); j <= a.Upper(); j++)
                            {
                                m += qi[j] * a.Value(j); // scalar product of
                            }
                            // k-th line of G and i-th column of H-1 Gt
                        }

                        _ghGt.ChangeValue(k, i) = m;
                    }
                }
            }

            _isSolved = _ghGt.Decompose();
        }

        return _isSolved;
    }

    /// <summary>The solution. Throws StdFail_NotDone if <see cref="Solve"/> did not succeed.</summary>
    public void Solution(math_Vector solution)
    {
        if (!_isSolved)
        {
            throw new NotDoneException("FEmTool_Assembly::Solution");
        }

        if (_g.IsEmpty())
        {
            _h.Solve(_b, solution);
        }
        else
        {
            math_Vector v1 = new(_b.Lower(), _b.Upper());
            _h.Solve(_b, v1);

            math_Vector l = new(1, _g.Length()), v2 = new(1, _g.Length());
            int i, j;
            NCollection_List<NCollection_HArray1<double>>.Iterator iter = new();

            for (i = 1; i <= _g.Length(); i++)
            {
                NCollection_List<NCollection_HArray1<double>> L = _g.Value(i);
                double m = 0.0;
                for (iter.Initialize(L); iter.More(); iter.Next())
                {
                    NCollection_HArray1<double> a = iter.Value();
                    for (j = a.Lower(); j <= a.Upper(); j++)
                    {
                        m += v1[j] * a.Value(j); // scalar product
                    }
                    // G v1
                }
                v2[i] = m - _c.Value(i);
            }

            _ghGt!.Solve(v2, l); // Solving M*l = v2

            v1.Initialized(_b);
            // Calculation v1 = B-Gt*l
            // v1(j) = B(j) - Gt(j,i)*l(i) = B(j) - G(i,j)*l(i)
            //
            for (i = 1; i <= _g.Length(); i++)
            {
                NCollection_List<NCollection_HArray1<double>> L = _g.Value(i);
                for (iter.Initialize(L); iter.More(); iter.Next())
                {
                    NCollection_HArray1<double> a = iter.Value();
                    for (j = a.Lower(); j <= a.Upper(); j++)
                    {
                        v1[j] -= l[i] * a.Value(j);
                    }
                }
            }
            _h.Solve(v1, solution);
        }
    }

    /// <summary>The number of global variables.</summary>
    public int GlobVarCount => _b.Length();

    /// <summary>The assembly table mapping element-local indices to global indices (shared: don't modify).</summary>
    public NCollection_HArray2<NCollection_HArray1<int>> AssemblyTable() => _refTable;

    /// <summary>The assembly table, through an output parameter (deprecated in OCCT: use <see cref="AssemblyTable"/>).</summary>
    public void GetAssemblyTable(out NCollection_HArray2<NCollection_HArray1<int>> assTable)
    {
        assTable = AssemblyTable();
    }

    /// <summary>Deletes all constraints.</summary>
    public void ResetConstraint()
    {
        _g.Clear();
        _c.Clear();
    }

    /// <summary>Sets every constraint coefficient and value to zero.</summary>
    public void NullifyConstraint()
    {
        NCollection_List<NCollection_HArray1<double>>.Iterator iter = new();
        int i;

        for (i = 1; i <= _g.Length(); i++)
        {
            _c.SetValue(i, 0.0);
            for (iter.Initialize(_g.Value(i)); iter.More(); iter.Next())
            {
                iter.Value().Init(0.0);
            }
        }
    }

    /// <summary>
    /// Adds the linear form <paramref name="linearForm"/> of element <paramref name="element"/> in dimension
    /// <paramref name="dimension"/> to constraint <paramref name="indexofConstraint"/>, and
    /// <paramref name="value"/> to its value.
    /// </summary>
    public void AddConstraint(int indexofConstraint, int element, int dimension, math_Vector linearForm, double value)
    {
        while (_g.Length() < indexofConstraint)
        {
            // Add new lines in G
            NCollection_List<NCollection_HArray1<double>> l1 = new();
            _g.Append(l1);
            _c.Append(0.0);
        }

        NCollection_List<NCollection_HArray1<double>> l = _g.ChangeValue(indexofConstraint);

        NCollection_HArray1<int> indexes = _refTable.Value(dimension, element);
        int i, imax = 0, imin = GlobVarCount;

        for (i = indexes.Lower(); i <= indexes.Upper(); i++)
        {
            imin = Min(imin, indexes.Value(i));
            imax = Max(imax, indexes.Value(i));
        }

        // OCCT dereferences a null Coeff (crash) when the list is not empty and the new range is wider than
        // every segment on both sides; the port throws a NullReferenceException there
        NCollection_HArray1<double>? coeff = null;

        if (l.IsEmpty())
        {
            coeff = new NCollection_HArray1<double>(imin, imax);
            coeff.Init(0.0);
            l.Append(coeff);
        }
        else
        {
            NCollection_List<NCollection_HArray1<double>>.Iterator iter = new(l);
            // OCCT declares the segment ranks s1 and s2 as doubles
            double s1 = 0, s2 = 0;
            NCollection_HArray1<double>? aux1 = null, aux2 = null;
            for (i = 1; iter.More(); iter.Next(), i++)
            {
                if (imin >= iter.Value().Lower())
                {
                    s1 = i;
                    aux1 = iter.Value();
                    if (imax <= iter.Value().Upper())
                    {
                        s2 = s1;
                        coeff = iter.Value();
                        break;
                    }
                }

                if (imax <= iter.Value().Upper())
                {
                    s2 = i;
                    aux2 = iter.Value();
                }
            }

            if (s1 != s2)
            {
                if (s1 == 0)
                {
                    if (imax < aux2!.Lower())
                    {
                        // inserting before first segment
                        coeff = new NCollection_HArray1<double>(imin, imax);
                        coeff.Init(0.0);
                        l.Prepend(coeff);
                    }
                    else
                    {
                        // merge new and first segment
                        coeff = new NCollection_HArray1<double>(imin, aux2.Upper());
                        for (i = imin; i <= aux2.Lower() - 1; i++)
                        {
                            coeff.SetValue(i, 0.0);
                        }
                        for (i = aux2.Lower(); i <= aux2.Upper(); i++)
                        {
                            coeff.SetValue(i, aux2.Value(i));
                        }
                        l.ChangeFirst() = coeff;
                    }
                }
                else if (s2 == 0)
                {
                    if (imin > aux1!.Upper())
                    {
                        // append new
                        coeff = new NCollection_HArray1<double>(imin, imax);
                        coeff.Init(0.0);
                        l.Append(coeff);
                    }
                    else
                    {
                        // merge new and last segment
                        coeff = new NCollection_HArray1<double>(aux1.Lower(), imax);
                        for (i = aux1.Lower(); i <= aux1.Upper(); i++)
                        {
                            coeff.SetValue(i, aux1.Value(i));
                        }
                        for (i = aux1.Upper() + 1; i <= imax; i++)
                        {
                            coeff.SetValue(i, 0.0);
                        }
                        l.ChangeLast() = coeff;
                    }
                }
                else if (imin <= aux1!.Upper() && imax < aux2!.Lower())
                {
                    // merge s1 and new
                    coeff = new NCollection_HArray1<double>(aux1.Lower(), imax);
                    for (i = aux1.Lower(); i <= aux1.Upper(); i++)
                    {
                        coeff.SetValue(i, aux1.Value(i));
                    }
                    for (i = aux1.Upper() + 1; i <= imax; i++)
                    {
                        coeff.SetValue(i, 0.0);
                    }
                    iter.Initialize(l);
                    for (i = 1; i < s1; iter.Next(), i++)
                    {
                        ;
                    }
                    iter.ChangeValue() = coeff;
                }
                else if (imin > aux1.Upper() && imax >= aux2!.Lower())
                {
                    // merge new and first segment
                    coeff = new NCollection_HArray1<double>(imin, aux2.Upper());
                    for (i = imin; i <= aux2.Lower() - 1; i++)
                    {
                        coeff.SetValue(i, 0.0);
                    }
                    for (i = aux2.Lower(); i <= aux2.Upper(); i++)
                    {
                        coeff.SetValue(i, aux2.Value(i));
                    }
                    iter.Initialize(l);
                    for (i = 1; i < s2; iter.Next(), i++)
                    {
                        ;
                    }
                    iter.ChangeValue() = coeff;
                }
                else if (imin > aux1.Upper() && imax < aux2!.Lower())
                {
                    // inserting new between s1 and s2
                    coeff = new NCollection_HArray1<double>(imin, imax);
                    coeff.Init(0.0);
                    iter.Initialize(l);
                    for (i = 1; i < s1; iter.Next(), i++)
                    {
                        ;
                    }
                    l.InsertAfter(coeff, iter);
                }
                else
                {
                    // merge s1, new, s2 and remove s2
                    coeff = new NCollection_HArray1<double>(aux1.Lower(), aux2!.Upper());
                    for (i = aux1.Lower(); i <= aux1.Upper(); i++)
                    {
                        coeff.SetValue(i, aux1.Value(i));
                    }
                    for (i = aux1.Upper() + 1; i <= aux2.Lower() - 1; i++)
                    {
                        coeff.SetValue(i, 0.0);
                    }
                    for (i = aux2.Lower(); i <= aux2.Upper(); i++)
                    {
                        coeff.SetValue(i, aux2.Value(i));
                    }
                    iter.Initialize(l);
                    for (i = 1; i < s1; iter.Next(), i++)
                    {
                        ;
                    }
                    iter.ChangeValue() = coeff;
                    iter.Next();
                    l.Remove(iter);
                }
            }
        }

        // adding
        int j = linearForm.Lower();
        for (i = indexes.Lower(); i <= indexes.Upper(); i++, j++)
        {
            coeff!.ChangeValue(indexes.Value(i)) += linearForm[j];
        }

        _c.ChangeValue(indexofConstraint) += value;
    }
}
