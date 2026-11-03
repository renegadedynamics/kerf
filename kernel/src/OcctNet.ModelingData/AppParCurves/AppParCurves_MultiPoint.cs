// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiPoint.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiPoint.lxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using System.Globalization;

namespace Occt;

/// <summary>
/// The points of a MultiPoint: first the 3d points, then the 2d points. They are the poles of the curves of a
/// MultiCurve, or the data points of a MultiLine.
/// </summary>
/// <remarks>
/// A class (decision 141): OCCT's copies share the point arrays, which are handles, so a C# reference behaves as
/// a C++ copy does. The point counts never change after construction. <see cref="AppParCurves_MultiPoint(AppParCurves_MultiPoint)"/>
/// is the C++ copy constructor, sharing the arrays. The setters stay public, as in OCCT: MultiPoints are the
/// input data callers build (AppDef_MultiPointConstraint derives from this class).
/// </remarks>
public class AppParCurves_MultiPoint
{
    protected NCollection_HArray1<Point3D>? ttabPoint;
    protected NCollection_HArray1<Point2D>? ttabPoint2d;
    protected int nbP;
    protected int nbP2d;

    /// <summary>An indefinite MultiPoint.</summary>
    public AppParCurves_MultiPoint()
    {
        nbP = 0;
        nbP2d = 0;
    }

    /// <summary>
    /// <paramref name="NbPoles"/> 3d points and <paramref name="NbPoles2d"/> 2d points, to be set with
    /// <see cref="SetPoint"/> and <see cref="SetPoint2d"/>.
    /// </summary>
    public AppParCurves_MultiPoint(int NbPoles, int NbPoles2d)
    {
        nbP = NbPoles;
        nbP2d = NbPoles2d;
        if (nbP != 0)
        {
            NCollection_HArray1<Point3D> tab3d = new(1, NbPoles);
            ttabPoint = tab3d;
        }
        if (nbP2d != 0)
        {
            NCollection_HArray1<Point2D> tab2d = new(1, NbPoles2d);
            ttabPoint2d = tab2d;
        }
    }

    /// <summary>A MultiPoint of 3d points only (copied).</summary>
    public AppParCurves_MultiPoint(NCollection_Array1<Point3D> tabP)
    {
        nbP2d = 0;
        nbP = tabP.Length();
        NCollection_HArray1<Point3D> tab3d = new(1, nbP);
        ttabPoint = tab3d;
        int Lower = tabP.Lower();
        NCollection_Array1<Point3D> P3d = tabPoint.ChangeArray1();
        for (int i = 1; i <= tabP.Length(); i++)
        {
            P3d.SetValue(i, tabP.Value(Lower + i - 1));
        }
    }

    /// <summary>A MultiPoint of 2d points only (copied).</summary>
    public AppParCurves_MultiPoint(NCollection_Array1<Point2D> tabP2d)
    {
        nbP = 0;
        nbP2d = tabP2d.Length();
        NCollection_HArray1<Point2D> tab2d = new(1, nbP2d);
        ttabPoint2d = tab2d;
        int Lower = tabP2d.Lower();
        NCollection_Array1<Point2D> P2d = tabPoint2d.ChangeArray1();
        for (int i = 1; i <= nbP2d; i++)
        {
            P2d.SetValue(i, tabP2d.Value(Lower + i - 1));
        }
    }

    /// <summary>A MultiPoint of the 3d points <paramref name="tabP"/> and the 2d points <paramref name="tabP2d"/> (copied).</summary>
    public AppParCurves_MultiPoint(NCollection_Array1<Point3D> tabP, NCollection_Array1<Point2D> tabP2d)
    {
        nbP = tabP.Length();
        nbP2d = tabP2d.Length();
        NCollection_HArray1<Point3D> t3d = new(1, nbP);
        ttabPoint = t3d;
        NCollection_HArray1<Point2D> t2d = new(1, nbP2d);
        ttabPoint2d = t2d;
        NCollection_Array1<Point3D> P3d = tabPoint.ChangeArray1();
        int i, Lower = tabP.Lower();
        for (i = 1; i <= nbP; i++)
        {
            P3d.SetValue(i, tabP.Value(Lower + i - 1));
        }
        Lower = tabP2d.Lower();
        NCollection_Array1<Point2D> P2d = tabPoint2d.ChangeArray1();
        for (i = 1; i <= nbP2d; i++)
        {
            P2d.SetValue(i, tabP2d.Value(Lower + i - 1));
        }
    }

    /// <summary>OCCT's copy constructor: the copy shares the point arrays of <paramref name="theOther"/>.</summary>
    public AppParCurves_MultiPoint(AppParCurves_MultiPoint theOther)
    {
        ttabPoint = theOther.ttabPoint;
        ttabPoint2d = theOther.ttabPoint2d;
        nbP = theOther.nbP;
        nbP2d = theOther.nbP2d;
    }

    // OCCT's tabPoint and tabPoint2d macros (down-casts of the handles)
    private NCollection_HArray1<Point3D> tabPoint => ttabPoint!;

    private NCollection_HArray1<Point2D> tabPoint2d => ttabPoint2d!;

    /// <summary>The number of 3d points.</summary>
    public int NbPoints() => nbP;

    /// <summary>The number of 2d points.</summary>
    public int NbPoints2d() => nbP2d;

    /// <summary>The dimension (3 or 2) of the point of range <paramref name="Index"/>.</summary>
    public int Dimension(int Index)
    {
        if (Index < 0 || Index > (nbP + nbP2d))
        {
            throw new OutOfRangeException();
        }
        if (Index <= nbP)
        {
            return 3;
        }
        else
        {
            return 2;
        }
    }

    /// <summary>
    /// Applies newx = x + dx * oldx, newy = y + dy * oldy, newz = z + dz * oldz to the 3d point of range
    /// <paramref name="CuIndex"/>.
    /// </summary>
    public void Transform(int CuIndex, double x, double dx, double y, double dy, double z, double dz)
    {
        if (Dimension(CuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        Point3D P, newP = new();
        P = Point(CuIndex);
        newP.SetCoord(x + P.X() * dx, y + P.Y() * dy, z + P.Z() * dz);
        tabPoint.SetValue(CuIndex, newP);
    }

    /// <summary>
    /// Applies newx = x + dx * oldx, newy = y + dy * oldy to the 2d point of range <paramref name="CuIndex"/>.
    /// </summary>
    public void Transform2d(int CuIndex, double x, double dx, double y, double dy)
    {
        if (Dimension(CuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        Point2D P, newP = new();
        P = Point2d(CuIndex);
        newP.SetCoord(x + P.X() * dx, y + P.Y() * dy);
        SetPoint2d(CuIndex, newP);
    }

    /// <summary>Sets the 3d point of range <paramref name="Index"/> (1 to NbPoints).</summary>
    public void SetPoint(int Index, in Point3D Point)
    {
        Standard_OutOfRange_Raise_if((Index <= 0) || (Index > nbP), "AppParCurves_MultiPoint::SetPoint() - wrong index");
        tabPoint.SetValue(Index, Point);
    }

    /// <summary>The 3d point of range <paramref name="Index"/> (1 to NbPoints).</summary>
    public Point3D Point(int Index)
    {
        Standard_OutOfRange_Raise_if((Index <= 0) || (Index > nbP), "AppParCurves_MultiPoint::Point() - wrong index");
        return tabPoint.Value(Index);
    }

    /// <summary>Sets the 2d point of range <paramref name="Index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public void SetPoint2d(int Index, in Point2D Point)
    {
        Standard_OutOfRange_Raise_if((Index <= nbP) || (Index > nbP + nbP2d),
                                     "AppParCurves_MultiPoint::SetPoint2d() - wrong index");
        tabPoint2d.SetValue(Index - nbP, Point);
    }

    /// <summary>The 2d point of range <paramref name="Index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public Point2D Point2d(int Index)
    {
        Standard_OutOfRange_Raise_if((Index <= nbP) || (Index > nbP + nbP2d),
                                     "AppParCurves_MultiPoint::Point2d() - wrong index");
        return tabPoint2d.Value(Index - nbP);
    }

    /// <summary>Writes the points.</summary>
    public virtual void Dump(TextWriter o)
    {
        o.Write("AppParCurves_MultiPoint dump:" + '\n');
        int aNbPnts3D = NbPoints(), aNbPnts2D = NbPoints2d();
        o.Write("It contains " + Int(aNbPnts3D) + " 3d points and " + Int(aNbPnts2D) + " 2d points." + '\n');
        if (aNbPnts3D > 0)
        {
            for (int i = tabPoint.Lower(); i <= tabPoint.Upper(); i++)
            {
                o.Write("3D-Point #" + Int(i) + '\n');
                o.Write(" Pole x = " + Real(tabPoint.Value(i).X()) + '\n');
                o.Write(" Pole y = " + Real(tabPoint.Value(i).Y()) + '\n');
                o.Write(" Pole z = " + Real(tabPoint.Value(i).Z()) + '\n');
            }
        }
        if (aNbPnts2D > 0)
        {
            for (int i = tabPoint2d.Lower(); i <= tabPoint2d.Upper(); i++)
            {
                o.Write("2D-Point #" + Int(i) + '\n');
                o.Write(" Pole x = " + Real(tabPoint2d.Value(i).X()) + '\n');
                o.Write(" Pole y = " + Real(tabPoint2d.Value(i).Y()) + '\n');
            }
        }
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }

    // C++ stream formatting: OS << int, OS << double (six significant digits, as printf's %g)
    internal static string Int(int theValue) => theValue.ToString(CultureInfo.InvariantCulture);

    internal static string Real(double theValue) => global::Kerf.Engine.Meshing.Poly.Real(theValue);
}
