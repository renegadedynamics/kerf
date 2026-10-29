// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiCurve.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A set of n Bezier curves of the same degree approximating a MultiLine, stored as its poles: NbPoles
/// MultiPoints, each holding the pole of every curve (3d curves first, then 2d curves).
/// </summary>
/// <remarks>
/// A class, as <see cref="AppParCurves_MultiPoint"/> (decision 141): OCCT's copies share the array of
/// MultiPoints, which is a handle. <see cref="AppParCurves_MultiCurve(AppParCurves_MultiCurve)"/> is the C++ copy
/// constructor (and the slicing copy of a MultiBSpCurve), sharing the poles. <see cref="Value(int)"/> returns the
/// stored MultiPoint (OCCT returns a const reference; shared: don't modify).
/// </remarks>
public class AppParCurves_MultiCurve
{
    protected NCollection_HArray1<AppParCurves_MultiPoint>? tabPoint;

    /// <summary>An indefinite MultiCurve.</summary>
    public AppParCurves_MultiCurve() { }

    /// <summary>A MultiCurve of <paramref name="NbPol"/> MultiPoints, to be set with <see cref="SetValue"/>.</summary>
    public AppParCurves_MultiCurve(int NbPol)
    {
        tabPoint = NewPoints(NbPol);
    }

    /// <summary>A MultiCurve of the MultiPoints <paramref name="tabMU"/>, which must all have NbCurves points.</summary>
    public AppParCurves_MultiCurve(NCollection_Array1<AppParCurves_MultiPoint> tabMU)
    {
        tabPoint = new NCollection_HArray1<AppParCurves_MultiPoint>(1, tabMU.Length());
        int i, Lower = tabMU.Lower();
        for (i = 1; i <= tabMU.Length(); i++)
        {
            tabPoint.SetValue(i, tabMU.Value(Lower + i - 1));
        }
    }

    /// <summary>OCCT's copy constructor: the copy shares the poles of <paramref name="theOther"/>.</summary>
    public AppParCurves_MultiCurve(AppParCurves_MultiCurve theOther)
    {
        tabPoint = theOther.tabPoint;
    }

    // C++ default-constructs the elements of the array: empty MultiPoints, not nulls
    private static NCollection_HArray1<AppParCurves_MultiPoint> NewPoints(int theNbPoles)
    {
        NCollection_HArray1<AppParCurves_MultiPoint> aTab = new(1, theNbPoles);
        for (int i = 1; i <= theNbPoles; i++)
        {
            aTab[i] = new AppParCurves_MultiPoint();
        }
        return aTab;
    }

    /// <summary>Gives the MultiCurve <paramref name="nbPoles"/> new, empty MultiPoints.</summary>
    public void SetNbPoles(int nbPoles)
    {
        tabPoint = NewPoints(nbPoles);
    }

    /// <summary>Sets the MultiPoint of range <paramref name="Index"/> (stored, not copied, as C++ stores a sharing copy).</summary>
    public void SetValue(int Index, AppParCurves_MultiPoint MPoint)
    {
        if ((Index <= 0) || (Index > tabPoint!.Length()))
        {
            throw new Standard_OutOfRange();
        }
        tabPoint.SetValue(Index, MPoint);
    }

    /// <summary>The number of curves.</summary>
    public int NbCurves()
    {
        if (tabPoint is null)
        {
            return 0;
        }
        AppParCurves_MultiPoint MP = tabPoint.Value(1);
        return MP.NbPoints() + MP.NbPoints2d();
    }

    /// <summary>The number of poles of each curve.</summary>
    public virtual int NbPoles()
    {
        if (tabPoint is null)
        {
            return 0;
        }
        return tabPoint.Length();
    }

    /// <summary>The degree of the curves.</summary>
    public virtual int Degree() => tabPoint!.Length() - 1;

    /// <summary>The dimension (3 or 2) of the curve of range <paramref name="CuIndex"/>.</summary>
    public int Dimension(int CuIndex)
    {
        int Lo = tabPoint!.Lower();
        int nb = tabPoint.Value(Lo).NbPoints() + tabPoint.Value(Lo).NbPoints2d();
        if ((CuIndex <= 0) || (CuIndex > nb))
        {
            throw new Standard_OutOfRange();
        }
        return tabPoint.Value(Lo).Dimension(CuIndex);
    }

    /// <summary>Fills <paramref name="TabPnt"/> (from 1) with the poles of the 3d curve <paramref name="CuIndex"/>.</summary>
    public void Curve(int CuIndex, NCollection_Array1<gp_Pnt> TabPnt)
    {
        if ((CuIndex <= 0))
        {
            throw new Standard_OutOfRange();
        }
        for (int i = 1; i <= tabPoint!.Length(); i++)
        {
            TabPnt[i] = tabPoint.Value(i).Point(CuIndex);
        }
    }

    /// <summary>Fills <paramref name="TabPnt2d"/> (from 1) with the poles of the 2d curve <paramref name="CuIndex"/>.</summary>
    public void Curve(int CuIndex, NCollection_Array1<gp_Pnt2d> TabPnt2d)
    {
        if ((CuIndex <= 0))
        {
            throw new Standard_OutOfRange();
        }
        for (int i = 1; i <= tabPoint!.Length(); i++)
        {
            TabPnt2d[i] = tabPoint.Value(i).Point2d(CuIndex);
        }
    }

    /// <summary>The MultiPoint of range <paramref name="Index"/> (shared: don't modify).</summary>
    public AppParCurves_MultiPoint Value(int Index)
    {
        if ((Index <= 0) || (Index > tabPoint!.Length()))
        {
            throw new Standard_OutOfRange();
        }
        return tabPoint.Value(Index);
    }

    /// <summary>The pole <paramref name="Nieme"/> of the 3d curve <paramref name="CuIndex"/>.</summary>
    public gp_Pnt Pole(int CuIndex, int Nieme)
    {
        // OCCT: && where || is meant, so the check only fires when both indexes are non-positive
        if ((CuIndex <= 0) && Nieme <= 0)
        {
            throw new Standard_OutOfRange();
        }
        return tabPoint!.Value(Nieme).Point(CuIndex);
    }

    /// <summary>The pole <paramref name="Nieme"/> of the 2d curve <paramref name="CuIndex"/>.</summary>
    public gp_Pnt2d Pole2d(int CuIndex, int Nieme)
    {
        // OCCT: && where || is meant (see Pole)
        if ((CuIndex <= 0) && Nieme <= 0)
        {
            throw new Standard_OutOfRange();
        }
        return tabPoint!.Value(Nieme).Point2d(CuIndex);
    }

    /// <summary>
    /// Applies newx = x + dx * oldx, newy = y + dy * oldy, newz = z + dz * oldz to the poles of the 3d curve
    /// <paramref name="CuIndex"/>.
    /// </summary>
    public void Transform(int CuIndex, double x, double dx, double y, double dy, double z, double dz)
    {
        if (Dimension(CuIndex) != 3)
        {
            throw new Standard_OutOfRange();
        }
        for (int i = 1; i <= tabPoint!.Length(); i++)
        {
            tabPoint[i].Transform(CuIndex, x, dx, y, dy, z, dz);
        }
    }

    /// <summary>
    /// Applies newx = x + dx * oldx, newy = y + dy * oldy to the poles of the 2d curve <paramref name="CuIndex"/>.
    /// </summary>
    public void Transform2d(int CuIndex, double x, double dx, double y, double dy)
    {
        if (Dimension(CuIndex) != 2)
        {
            throw new Standard_OutOfRange();
        }
        for (int i = 1; i <= tabPoint!.Length(); i++)
        {
            tabPoint[i].Transform2d(CuIndex, x, dx, y, dy);
        }
    }

    /// <summary>The point at <paramref name="U"/> of the 3d Bezier curve <paramref name="CuIndex"/>.</summary>
    public virtual void Value(int CuIndex, double U, out gp_Pnt Pt)
    {
        if (Dimension(CuIndex) != 3)
        {
            throw new Standard_OutOfRange();
        }
        NCollection_Array1<gp_Pnt> TabPoles = new(1, tabPoint!.Length());
        for (int i = 1; i <= tabPoint.Length(); i++)
        {
            TabPoles[i] = tabPoint.Value(i).Point(CuIndex);
        }
        BSplCLib.D0(U, TabPoles, BSplCLib.NoWeights(), out Pt);
    }

    /// <summary>The point at <paramref name="U"/> of the 2d Bezier curve <paramref name="CuIndex"/>.</summary>
    public virtual void Value(int CuIndex, double U, out gp_Pnt2d Pt)
    {
        if (Dimension(CuIndex) != 2)
        {
            throw new Standard_OutOfRange();
        }
        NCollection_Array1<gp_Pnt2d> TabPole = new(1, tabPoint!.Length());
        for (int i = 1; i <= tabPoint.Length(); i++)
        {
            TabPole[i] = tabPoint.Value(i).Point2d(CuIndex);
        }
        BSplCLib.D0(U, TabPole, BSplCLib.NoWeights(), out Pt);
    }

    /// <summary>The point and first derivative at <paramref name="U"/> of the 3d Bezier curve <paramref name="CuIndex"/>.</summary>
    public virtual void D1(int CuIndex, double U, out gp_Pnt Pt, out gp_Vec V1)
    {
        if (Dimension(CuIndex) != 3)
        {
            throw new Standard_OutOfRange();
        }
        NCollection_Array1<gp_Pnt> TabPole = new(1, tabPoint!.Length());
        for (int i = 1; i <= tabPoint.Length(); i++)
        {
            TabPole[i] = tabPoint.Value(i).Point(CuIndex);
        }
        BSplCLib.D1(U, TabPole, BSplCLib.NoWeights(), out Pt, out V1);
    }

    /// <summary>The point and two derivatives at <paramref name="U"/> of the 3d Bezier curve <paramref name="CuIndex"/>.</summary>
    public virtual void D2(int CuIndex, double U, out gp_Pnt Pt, out gp_Vec V1, out gp_Vec V2)
    {
        if (Dimension(CuIndex) != 3)
        {
            throw new Standard_OutOfRange();
        }
        NCollection_Array1<gp_Pnt> TabPole = new(1, tabPoint!.Length());
        for (int i = 1; i <= tabPoint.Length(); i++)
        {
            TabPole[i] = tabPoint.Value(i).Point(CuIndex);
        }
        BSplCLib.D2(U, TabPole, BSplCLib.NoWeights(), out Pt, out V1, out V2);
    }

    /// <summary>The point and first derivative at <paramref name="U"/> of the 2d Bezier curve <paramref name="CuIndex"/>.</summary>
    public virtual void D1(int CuIndex, double U, out gp_Pnt2d Pt, out gp_Vec2d V1)
    {
        if (Dimension(CuIndex) != 2)
        {
            throw new Standard_OutOfRange();
        }
        NCollection_Array1<gp_Pnt2d> TabPole = new(1, tabPoint!.Length());
        for (int i = 1; i <= tabPoint.Length(); i++)
        {
            TabPole[i] = tabPoint.Value(i).Point2d(CuIndex);
        }
        BSplCLib.D1(U, TabPole, BSplCLib.NoWeights(), out Pt, out V1);
    }

    /// <summary>The point and two derivatives at <paramref name="U"/> of the 2d Bezier curve <paramref name="CuIndex"/>.</summary>
    public virtual void D2(int CuIndex, double U, out gp_Pnt2d Pt, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        if (Dimension(CuIndex) != 2)
        {
            throw new Standard_OutOfRange();
        }
        NCollection_Array1<gp_Pnt2d> TabPole = new(1, tabPoint!.Length());
        for (int i = 1; i <= tabPoint.Length(); i++)
        {
            TabPole[i] = tabPoint.Value(i).Point2d(CuIndex);
        }
        BSplCLib.D2(U, TabPole, BSplCLib.NoWeights(), out Pt, out V1, out V2);
    }

    /// <summary>Writes the number of curves and their degree (OCCT's pole listing is commented out).</summary>
    public virtual void Dump(TextWriter o)
    {
        o.Write("AppParCurves_MultiCurve dump:" + '\n');
        o.Write(" It contains " + AppParCurves_MultiPoint.Int(NbCurves()) + " Bezier curves of degree "
                + AppParCurves_MultiPoint.Int(tabPoint!.Length() - 1) + '\n');
        o.Write(" The poles are: " + '\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
