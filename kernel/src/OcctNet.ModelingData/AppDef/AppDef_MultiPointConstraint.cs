// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MultiPointConstraint.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MultiPointConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A MultiPoint of a MultiLine with optional tangency and curvature constraints: several 2d or 3d points (the 3d
/// points first) that share one constraint and one parameter, approximated in parallel. A tangency point is also a
/// passing point.
/// </summary>
/// <remarks>
/// A class, as its base (decisions 141 and 201): a C++ copy shares the point, tangent and curvature arrays, and so
/// does a C# reference. The setters stay public: callers build MultiLines from these.
/// </remarks>
public class AppDef_MultiPointConstraint : AppParCurves_MultiPoint
{
    private NCollection_HArray1<gp_Vec>? tabTang;
    private NCollection_HArray1<gp_Vec>? tabCurv;
    private NCollection_HArray1<gp_Vec2d>? tabTang2d;
    private NCollection_HArray1<gp_Vec2d>? tabCurv2d;

    /// <summary>An undefined MultiPointConstraint.</summary>
    public AppDef_MultiPointConstraint() { }

    /// <summary>
    /// <paramref name="NbPoles"/> 3d points and <paramref name="NbPoles2d"/> 2d points, to be set with
    /// <see cref="AppParCurves_MultiPoint.SetPoint"/> and <see cref="AppParCurves_MultiPoint.SetPoint2d"/>.
    /// </summary>
    public AppDef_MultiPointConstraint(int NbPoles, int NbPoles2d) : base(NbPoles, NbPoles2d) { }

    /// <summary>A MultiPoint of 3d points only.</summary>
    public AppDef_MultiPointConstraint(NCollection_Array1<gp_Pnt> tabP) : base(tabP) { }

    /// <summary>A MultiPoint of 2d points only.</summary>
    public AppDef_MultiPointConstraint(NCollection_Array1<gp_Pnt2d> tabP2d) : base(tabP2d) { }

    /// <summary>A MultiPoint of 3d and 2d points.</summary>
    public AppDef_MultiPointConstraint(NCollection_Array1<gp_Pnt> tabP, NCollection_Array1<gp_Pnt2d> tabP2d)
        : base(tabP, tabP2d) { }

    /// <summary>
    /// A MultiPoint with tangency and curvature constraints. Throws Standard_ConstructionError unless there are as
    /// many tangents and curvatures as points, in each dimension.
    /// </summary>
    public AppDef_MultiPointConstraint(NCollection_Array1<gp_Pnt> tabP, NCollection_Array1<gp_Pnt2d> tabP2d,
                                       NCollection_Array1<gp_Vec> tabVec, NCollection_Array1<gp_Vec2d> tabVec2d,
                                       NCollection_Array1<gp_Vec> tabCur, NCollection_Array1<gp_Vec2d> tabCur2d)
        : base(tabP, tabP2d)
    {
        if ((tabP.Length() != tabVec.Length()) || (tabP2d.Length() != tabVec2d.Length())
            || (tabCur.Length() != tabP.Length()) || (tabCur2d.Length() != tabP2d.Length()))
        {
            throw new Standard_ConstructionError();
        }

        tabTang = new NCollection_HArray1<gp_Vec>(1, tabVec.Length());
        tabTang2d = new NCollection_HArray1<gp_Vec2d>(1, tabVec2d.Length());

        int i, Lower = tabVec.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            tabTang.SetValue(i, tabVec.Value(Lower + i - 1));
        }
        Lower = tabVec2d.Lower();
        for (i = 1; i <= tabVec2d.Length(); i++)
        {
            tabTang2d.SetValue(i, tabVec2d.Value(Lower + i - 1));
        }

        tabCurv = new NCollection_HArray1<gp_Vec>(1, tabCur.Length());
        tabCurv2d = new NCollection_HArray1<gp_Vec2d>(1, tabCur2d.Length());

        Lower = tabCur.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            tabCurv.SetValue(i, tabCur.Value(Lower + i - 1));
        }
        Lower = tabCur2d.Lower();
        for (i = 1; i <= tabCur2d.Length(); i++)
        {
            tabCurv2d.SetValue(i, tabCur2d.Value(Lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint with tangency constraints. Throws Standard_ConstructionError unless there are as many
    /// tangents as points, in each dimension.
    /// </summary>
    public AppDef_MultiPointConstraint(NCollection_Array1<gp_Pnt> tabP, NCollection_Array1<gp_Pnt2d> tabP2d,
                                       NCollection_Array1<gp_Vec> tabVec, NCollection_Array1<gp_Vec2d> tabVec2d)
        : base(tabP, tabP2d)
    {
        if ((tabP.Length() != tabVec.Length()) || (tabP2d.Length() != tabVec2d.Length()))
        {
            throw new Standard_ConstructionError();
        }

        tabTang = new NCollection_HArray1<gp_Vec>(1, tabVec.Length());
        tabTang2d = new NCollection_HArray1<gp_Vec2d>(1, tabVec2d.Length());

        int i, Lower = tabVec.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            tabTang.SetValue(i, tabVec.Value(Lower + i - 1));
        }
        Lower = tabVec2d.Lower();
        for (i = 1; i <= tabVec2d.Length(); i++)
        {
            tabTang2d.SetValue(i, tabVec2d.Value(Lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint of 3d points with tangency constraints. Throws Standard_ConstructionError unless there are as
    /// many tangents as points.
    /// </summary>
    public AppDef_MultiPointConstraint(NCollection_Array1<gp_Pnt> tabP, NCollection_Array1<gp_Vec> tabVec)
        : base(tabP)
    {
        if (tabP.Length() != tabVec.Length())
        {
            throw new Standard_ConstructionError();
        }

        tabTang = new NCollection_HArray1<gp_Vec>(1, tabVec.Length());

        int i, Lower = tabVec.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            tabTang.SetValue(i, tabVec.Value(Lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint of 3d points with tangency and curvature constraints. Throws Standard_ConstructionError unless
    /// there are as many tangents and curvatures as points.
    /// </summary>
    public AppDef_MultiPointConstraint(NCollection_Array1<gp_Pnt> tabP, NCollection_Array1<gp_Vec> tabVec,
                                       NCollection_Array1<gp_Vec> tabCur)
        : base(tabP)
    {
        if ((tabP.Length() != tabVec.Length()) || (tabP.Length() != tabCur.Length()))
        {
            throw new Standard_ConstructionError();
        }

        tabTang = new NCollection_HArray1<gp_Vec>(1, tabVec.Length());
        int i, Lower = tabVec.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            tabTang.SetValue(i, tabVec.Value(Lower + i - 1));
        }

        tabCurv = new NCollection_HArray1<gp_Vec>(1, tabCur.Length());
        Lower = tabCur.Lower();
        for (i = 1; i <= tabCur.Length(); i++)
        {
            tabCurv.SetValue(i, tabCur.Value(Lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint of 2d points with tangency constraints. Throws Standard_ConstructionError unless there are as
    /// many tangents as points.
    /// </summary>
    public AppDef_MultiPointConstraint(NCollection_Array1<gp_Pnt2d> tabP2d, NCollection_Array1<gp_Vec2d> tabVec2d)
        : base(tabP2d)
    {
        if (tabP2d.Length() != tabVec2d.Length())
        {
            throw new Standard_ConstructionError();
        }

        tabTang2d = new NCollection_HArray1<gp_Vec2d>(1, tabVec2d.Length());
        int i, Lower = tabVec2d.Lower();
        for (i = 1; i <= tabVec2d.Length(); i++)
        {
            tabTang2d.SetValue(i, tabVec2d.Value(Lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint of 2d points with tangency and curvature constraints. Throws Standard_ConstructionError unless
    /// there are as many tangents and curvatures as points.
    /// </summary>
    public AppDef_MultiPointConstraint(NCollection_Array1<gp_Pnt2d> tabP2d, NCollection_Array1<gp_Vec2d> tabVec2d,
                                       NCollection_Array1<gp_Vec2d> tabCur2d)
        : base(tabP2d)
    {
        if ((tabP2d.Length() != tabVec2d.Length()) || (tabCur2d.Length() != tabP2d.Length()))
        {
            throw new Standard_ConstructionError();
        }

        tabTang2d = new NCollection_HArray1<gp_Vec2d>(1, tabVec2d.Length());
        int i, Lower = tabVec2d.Lower();
        for (i = 1; i <= tabVec2d.Length(); i++)
        {
            tabTang2d.SetValue(i, tabVec2d.Value(Lower + i - 1));
        }

        tabCurv2d = new NCollection_HArray1<gp_Vec2d>(1, tabCur2d.Length());
        Lower = tabCur2d.Lower();
        for (i = 1; i <= tabCur2d.Length(); i++)
        {
            tabCurv2d.SetValue(i, tabCur2d.Value(Lower + i - 1));
        }
    }

    /// <summary>Sets the tangent of the 3d point of range <paramref name="Index"/> (1 to NbPoints).</summary>
    public void SetTang(int Index, in gp_Vec Tang)
    {
        tabTang ??= new NCollection_HArray1<gp_Vec>(1, nbP);
        if ((Index <= 0) || (Index > nbP))
        {
            throw new Standard_OutOfRange();
        }
        tabTang.SetValue(Index, Tang);
    }

    /// <summary>The tangent of the 3d point of range <paramref name="Index"/> (1 to NbPoints).</summary>
    public gp_Vec Tang(int Index)
    {
        if ((Index <= 0) || (Index > nbP))
        {
            throw new Standard_OutOfRange();
        }
        return tabTang!.Value(Index);
    }

    /// <summary>Sets the tangent of the 2d point of range <paramref name="Index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public void SetTang2d(int Index, in gp_Vec2d Tang2d)
    {
        tabTang2d ??= new NCollection_HArray1<gp_Vec2d>(1, nbP2d);

        if ((Index <= nbP) || (Index > nbP + nbP2d))
        {
            throw new Standard_OutOfRange();
        }
        tabTang2d.SetValue(Index - nbP, Tang2d);
    }

    /// <summary>The tangent of the 2d point of range <paramref name="Index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public gp_Vec2d Tang2d(int Index)
    {
        if ((Index <= nbP) || (Index > nbP + nbP2d))
        {
            throw new Standard_OutOfRange();
        }
        return tabTang2d!.Value(Index - nbP);
    }

    /// <summary>
    /// Sets the normal vector of the 3d point of range <paramref name="Index"/> (1 to NbPoints), whose norm is the
    /// normal curvature.
    /// </summary>
    public void SetCurv(int Index, in gp_Vec Curv)
    {
        tabCurv ??= new NCollection_HArray1<gp_Vec>(1, nbP);
        if ((Index <= 0) || (Index > nbP))
        {
            throw new Standard_OutOfRange();
        }
        tabCurv.SetValue(Index, Curv);
    }

    /// <summary>The normal vector of the 3d point of range <paramref name="Index"/> (1 to NbPoints).</summary>
    public gp_Vec Curv(int Index)
    {
        if ((Index <= 0) || (Index > nbP))
        {
            throw new Standard_OutOfRange();
        }
        return tabCurv!.Value(Index);
    }

    /// <summary>
    /// Sets the normal vector of the 2d point of range <paramref name="Index"/> (NbPoints + 1 to NbPoints +
    /// NbPoints2d), whose norm is the normal curvature.
    /// </summary>
    public void SetCurv2d(int Index, in gp_Vec2d Curv2d)
    {
        tabCurv2d ??= new NCollection_HArray1<gp_Vec2d>(1, nbP2d);
        if ((Index <= nbP) || (Index > nbP + nbP2d))
        {
            throw new Standard_OutOfRange();
        }
        tabCurv2d.SetValue(Index - nbP, Curv2d);
    }

    /// <summary>The normal vector of the 2d point of range <paramref name="Index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public gp_Vec2d Curv2d(int Index)
    {
        if ((Index <= nbP) || (Index > nbP + nbP2d))
        {
            throw new Standard_OutOfRange();
        }
        return tabCurv2d!.Value(Index - nbP);
    }

    /// <summary>True if the MultiPoint has tangents.</summary>
    public bool IsTangencyPoint() => !(tabTang is null && tabTang2d is null);

    /// <summary>True if the MultiPoint has curvatures.</summary>
    public bool IsCurvaturePoint() => !(tabCurv is null && tabCurv2d is null);

    /// <summary>Writes a one-line description.</summary>
    public override void Dump(TextWriter o)
    {
        o.Write("AppDef_MultiPointConstraint dump:" + '\n');
    }
}
