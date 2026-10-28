// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Box.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Box.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// A bounding box in 3D space, parallel to the axes of the coordinate system. It may be infinite (open) in one or
/// more directions, whole (infinite in all six), or void (empty). A gap is included on both sides in every direction
/// when the finite bounds are consulted.
/// </summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_Box()</c>) is a void box with a null gap. <c>default(Bnd_Box)</c> is not: it is a
/// non-void box reduced to the origin. Ported code must write <c>new Bnd_Box()</c>.
/// </remarks>
public partial struct Bnd_Box : IEquatable<Bnd_Box>
{
    /// <summary>The box limits (Xmin, Xmax, Ymin, Ymax, Zmin, Zmax), gap and open directions included.</summary>
    public readonly record struct Limits(double Xmin, double Xmax, double Ymin, double Ymax, double Zmin, double Zmax);

    // Precision constant for infinite bounds
    private const double THE_BND_PRECISION_INFINITE = 1e+100;

    // Bit flags (OCCT's protected enum MaskFlags)
    private const int VoidMask = 0x01;
    private const int XminMask = 0x02;
    private const int XmaxMask = 0x04;
    private const int YminMask = 0x08;
    private const int YmaxMask = 0x10;
    private const int ZminMask = 0x20;
    private const int ZmaxMask = 0x40;
    private const int WholeMask = 0x7e;

    private double Xmin;
    private double Xmax;
    private double Ymin;
    private double Ymax;
    private double Zmin;
    private double Zmax;
    private double Gap;
    private int Flags;

    /// <summary>Creates an empty (void) box with a null gap.</summary>
    public Bnd_Box()
    {
        Xmin = RealLast();
        Xmax = -RealLast();
        Ymin = RealLast();
        Ymax = -RealLast();
        Zmin = RealLast();
        Zmax = -RealLast();
        Gap = 0.0;
        Flags = VoidMask;
    }

    /// <summary>Creates a (non-void) box from its minimum and maximum corners, with a null gap.</summary>
    public Bnd_Box(in gp_Pnt theMin, in gp_Pnt theMax)
    {
        Xmin = theMin.X();
        Xmax = theMax.X();
        Ymin = theMin.Y();
        Ymax = theMax.Y();
        Zmin = theMin.Z();
        Zmax = theMax.Z();
        Gap = 0.0;
        Flags = 0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetWhole() => Flags = WholeMask;

    internal void SetVoid()
    {
        Xmin = RealLast();
        Xmax = -RealLast();
        Ymin = RealLast();
        Ymax = -RealLast();
        Zmin = RealLast();
        Zmax = -RealLast();
        Gap = 0.0;
        Flags = VoidMask;
    }

    internal void Set(in gp_Pnt P)
    {
        SetVoid();
        Add(P);
    }

    internal void Set(in gp_Pnt P, in gp_Dir D)
    {
        SetVoid();
        Add(P, D);
    }

    internal void Update(double x, double y, double z, double X, double Y, double Z)
    {
        if (IsVoid())
        {
            Xmin = x;
            Ymin = y;
            Zmin = z;
            Xmax = X;
            Ymax = Y;
            Zmax = Z;
            Flags &= ~VoidMask;
        }
        else
        {
            Xmin = Min(Xmin, x);
            Xmax = Max(Xmax, X);
            Ymin = Min(Ymin, y);
            Ymax = Max(Ymax, Y);
            Zmin = Min(Zmin, z);
            Zmax = Max(Zmax, Z);
        }
    }

    internal void Update(double X, double Y, double Z)
    {
        if (IsVoid())
        {
            Xmin = X;
            Ymin = Y;
            Zmin = Z;
            Xmax = X;
            Ymax = Y;
            Zmax = Z;
            Flags &= ~VoidMask;
        }
        else
        {
            Xmin = Min(Xmin, X);
            Xmax = Max(Xmax, X);
            Ymin = Min(Ymin, Y);
            Ymax = Max(Ymax, Y);
            Zmin = Min(Zmin, Z);
            Zmax = Max(Zmax, Z);
        }
    }

    internal void SetGap(double Tol) => Gap = Math.Abs(Tol);

    internal void Enlarge(double Tol) => Gap = Max(Gap, Math.Abs(Tol));

    internal void OpenXmin() => Flags |= XminMask;

    internal void OpenXmax() => Flags |= XmaxMask;

    internal void OpenYmin() => Flags |= YminMask;

    internal void OpenYmax() => Flags |= YmaxMask;

    internal void OpenZmin() => Flags |= ZminMask;

    internal void OpenZmax() => Flags |= ZmaxMask;

    internal void Add(in Bnd_Box Other)
    {
        if (Other.IsVoid())
        {
            return;
        }
        else if (IsVoid())
        {
            this = Other;
            return;
        }

        Gap = Max(Gap, Other.Gap);

        if (IsWhole())
        {
            return;
        }
        else if (Other.IsWhole())
        {
            SetWhole();
            return;
        }

        if (!IsOpenXmin())
        {
            if (Other.IsOpenXmin())
            {
                OpenXmin();
            }
            else if (Xmin > Other.Xmin)
            {
                Xmin = Other.Xmin;
            }
        }

        if (!IsOpenXmax())
        {
            if (Other.IsOpenXmax())
            {
                OpenXmax();
            }
            else if (Xmax < Other.Xmax)
            {
                Xmax = Other.Xmax;
            }
        }

        if (!IsOpenYmin())
        {
            if (Other.IsOpenYmin())
            {
                OpenYmin();
            }
            else if (Ymin > Other.Ymin)
            {
                Ymin = Other.Ymin;
            }
        }

        if (!IsOpenYmax())
        {
            if (Other.IsOpenYmax())
            {
                OpenYmax();
            }
            else if (Ymax < Other.Ymax)
            {
                Ymax = Other.Ymax;
            }
        }

        if (!IsOpenZmin())
        {
            if (Other.IsOpenZmin())
            {
                OpenZmin();
            }
            else if (Zmin > Other.Zmin)
            {
                Zmin = Other.Zmin;
            }
        }

        if (!IsOpenZmax())
        {
            if (Other.IsOpenZmax())
            {
                OpenZmax();
            }
            else if (Zmax < Other.Zmax)
            {
                Zmax = Other.Zmax;
            }
        }
    }

    internal void Add(in gp_Pnt P)
    {
        P.Coord(out double X, out double Y, out double Z);
        Update(X, Y, Z);
    }

    internal void Add(in gp_Pnt P, in gp_Dir D)
    {
        Add(P);
        Add(D);
    }

    internal void Add(in gp_Dir D)
    {
        D.Coord(out double DX, out double DY, out double DZ);

        if (DX < -RealEpsilon())
        {
            OpenXmin();
        }
        else if (DX > RealEpsilon())
        {
            OpenXmax();
        }

        if (DY < -RealEpsilon())
        {
            OpenYmin();
        }
        else if (DY > RealEpsilon())
        {
            OpenYmax();
        }

        if (DZ < -RealEpsilon())
        {
            OpenZmin();
        }
        else if (DZ > RealEpsilon())
        {
            OpenZmax();
        }
    }

    // Copies with a mutator applied (the public way to "set")

    /// <summary>A void box: the same as <c>new Bnd_Box()</c>.</summary>
    public static Bnd_Box Void() => new();

    /// <summary>A box bounding the single point <paramref name="P"/> (OCCT's <c>Set(P)</c>).</summary>
    public static Bnd_Box FromPoint(in gp_Pnt P)
    {
        var aBox = new Bnd_Box();
        aBox.Set(P);
        return aBox;
    }

    /// <summary>A box bounding the half-line from <paramref name="P"/> along <paramref name="D"/> (OCCT's <c>Set(P, D)</c>).</summary>
    public static Bnd_Box FromPoint(in gp_Pnt P, in gp_Dir D)
    {
        var aBox = new Bnd_Box();
        aBox.Set(P, D);
        return aBox;
    }

    /// <summary>This box made infinite in all directions (its finite bounds are kept, as in OCCT).</summary>
    public readonly Bnd_Box WithWhole()
    {
        Bnd_Box aCopy = this;
        aCopy.SetWhole();
        return aCopy;
    }

    public readonly Bnd_Box WithVoid() => new();

    public readonly Bnd_Box WithGap(double Tol)
    {
        Bnd_Box aCopy = this;
        aCopy.SetGap(Tol);
        return aCopy;
    }

    /// <summary>This box enlarged to contain the given intervals.</summary>
    public readonly Bnd_Box Updated(double x, double y, double z, double X, double Y, double Z)
    {
        Bnd_Box aCopy = this;
        aCopy.Update(x, y, z, X, Y, Z);
        return aCopy;
    }

    /// <summary>This box enlarged to contain the point (X, Y, Z).</summary>
    public readonly Bnd_Box Updated(double X, double Y, double Z)
    {
        Bnd_Box aCopy = this;
        aCopy.Update(X, Y, Z);
        return aCopy;
    }

    /// <summary>This box with its gap raised to at least |Tol|.</summary>
    public readonly Bnd_Box Enlarged(double Tol)
    {
        Bnd_Box aCopy = this;
        aCopy.Enlarge(Tol);
        return aCopy;
    }

    public readonly Bnd_Box OpenedXmin()
    {
        Bnd_Box aCopy = this;
        aCopy.OpenXmin();
        return aCopy;
    }

    public readonly Bnd_Box OpenedXmax()
    {
        Bnd_Box aCopy = this;
        aCopy.OpenXmax();
        return aCopy;
    }

    public readonly Bnd_Box OpenedYmin()
    {
        Bnd_Box aCopy = this;
        aCopy.OpenYmin();
        return aCopy;
    }

    public readonly Bnd_Box OpenedYmax()
    {
        Bnd_Box aCopy = this;
        aCopy.OpenYmax();
        return aCopy;
    }

    public readonly Bnd_Box OpenedZmin()
    {
        Bnd_Box aCopy = this;
        aCopy.OpenZmin();
        return aCopy;
    }

    public readonly Bnd_Box OpenedZmax()
    {
        Bnd_Box aCopy = this;
        aCopy.OpenZmax();
        return aCopy;
    }

    /// <summary>This box enlarged to contain <paramref name="Other"/>.</summary>
    public readonly Bnd_Box Added(in Bnd_Box Other)
    {
        Bnd_Box aCopy = this;
        aCopy.Add(Other);
        return aCopy;
    }

    /// <summary>This box enlarged to contain <paramref name="P"/>.</summary>
    public readonly Bnd_Box Added(in gp_Pnt P)
    {
        Bnd_Box aCopy = this;
        aCopy.Add(P);
        return aCopy;
    }

    /// <summary>This box extended from <paramref name="P"/> in the direction <paramref name="D"/>.</summary>
    public readonly Bnd_Box Added(in gp_Pnt P, in gp_Dir D)
    {
        Bnd_Box aCopy = this;
        aCopy.Add(P, D);
        return aCopy;
    }

    /// <summary>This box extended in the direction <paramref name="D"/> (made open on that side).</summary>
    public readonly Bnd_Box Added(in gp_Dir D)
    {
        Bnd_Box aCopy = this;
        aCopy.Add(D);
        return aCopy;
    }

    // Queries

    public readonly double GetGap() => Gap;

    /// <summary>The bounds, gap included; open directions give ±1e100. Throws Standard_ConstructionError if void.</summary>
    public readonly void Get(out double theXmin, out double theYmin, out double theZmin,
                             out double theXmax, out double theYmax, out double theZmax)
    {
        if (IsVoid())
        {
            throw new Standard_ConstructionError("Bnd_Box is void");
        }

        theXmin = GetXMin();
        theXmax = GetXMax();
        theYmin = GetYMin();
        theYmax = GetYMax();
        theZmin = GetZMin();
        theZmax = GetZMax();
    }

    /// <summary>The bounds as a <see cref="Limits"/>, gap included; for a void box, the raw internal values.</summary>
    public readonly Limits Get() => new(GetXMin(), GetXMax(), GetYMin(), GetYMax(), GetZMin(), GetZMax());

    public readonly double GetXMin() => IsOpenXmin() ? -THE_BND_PRECISION_INFINITE : Xmin - Gap;

    public readonly double GetXMax() => IsOpenXmax() ? THE_BND_PRECISION_INFINITE : Xmax + Gap;

    public readonly double GetYMin() => IsOpenYmin() ? -THE_BND_PRECISION_INFINITE : Ymin - Gap;

    public readonly double GetYMax() => IsOpenYmax() ? THE_BND_PRECISION_INFINITE : Ymax + Gap;

    public readonly double GetZMin() => IsOpenZmin() ? -THE_BND_PRECISION_INFINITE : Zmin - Gap;

    public readonly double GetZMax() => IsOpenZmax() ? THE_BND_PRECISION_INFINITE : Zmax + Gap;

    /// <summary>The lower corner, gap included. Throws Standard_ConstructionError if void.</summary>
    public readonly gp_Pnt CornerMin()
    {
        if (IsVoid())
        {
            throw new Standard_ConstructionError("Bnd_Box is void");
        }

        return new gp_Pnt(IsOpenXmin() ? -THE_BND_PRECISION_INFINITE : Xmin - Gap,
                          IsOpenYmin() ? -THE_BND_PRECISION_INFINITE : Ymin - Gap,
                          IsOpenZmin() ? -THE_BND_PRECISION_INFINITE : Zmin - Gap);
    }

    /// <summary>The upper corner, gap included. Throws Standard_ConstructionError if void.</summary>
    public readonly gp_Pnt CornerMax()
    {
        if (IsVoid())
        {
            throw new Standard_ConstructionError("Bnd_Box is void");
        }

        return new gp_Pnt(IsOpenXmax() ? THE_BND_PRECISION_INFINITE : Xmax + Gap,
                          IsOpenYmax() ? THE_BND_PRECISION_INFINITE : Ymax + Gap,
                          IsOpenZmax() ? THE_BND_PRECISION_INFINITE : Zmax + Gap);
    }

    /// <summary>The center, gap included, or null if the box is void.</summary>
    public readonly gp_Pnt? Center()
    {
        if (IsVoid())
        {
            return null;
        }

        return new gp_Pnt(0.5 * (GetXMin() + GetXMax()),
                          0.5 * (GetYMin() + GetYMax()),
                          0.5 * (GetZMin() + GetZMax()));
    }

    /// <summary>True if the box has at least one open direction.</summary>
    public readonly bool IsOpen() => (Flags & WholeMask) != 0;

    public readonly bool IsOpenXmin() => (Flags & XminMask) != 0;

    public readonly bool IsOpenXmax() => (Flags & XmaxMask) != 0;

    public readonly bool IsOpenYmin() => (Flags & YminMask) != 0;

    public readonly bool IsOpenYmax() => (Flags & YmaxMask) != 0;

    public readonly bool IsOpenZmin() => (Flags & ZminMask) != 0;

    public readonly bool IsOpenZmax() => (Flags & ZmaxMask) != 0;

    /// <summary>True if the box is infinite in all six directions.</summary>
    public readonly bool IsWhole() => (Flags & WholeMask) == WholeMask;

    /// <summary>True if the box is empty.</summary>
    public readonly bool IsVoid() => (Flags & VoidMask) != 0;

    /// <summary>True if xmax - xmin &lt; tol.</summary>
    public readonly bool IsXThin(double tol)
    {
        if (IsWhole())
        {
            return false;
        }

        if (IsVoid())
        {
            return true;
        }

        if (IsOpenXmin())
        {
            return false;
        }

        if (IsOpenXmax())
        {
            return false;
        }

        if (Xmax - Xmin < tol)
        {
            return true;
        }

        return false;
    }

    /// <summary>True if ymax - ymin &lt; tol.</summary>
    public readonly bool IsYThin(double tol)
    {
        if (IsWhole())
        {
            return false;
        }

        if (IsVoid())
        {
            return true;
        }

        if (IsOpenYmin())
        {
            return false;
        }

        if (IsOpenYmax())
        {
            return false;
        }

        if (Ymax - Ymin < tol)
        {
            return true;
        }

        return false;
    }

    /// <summary>True if zmax - zmin &lt; tol.</summary>
    public readonly bool IsZThin(double tol)
    {
        if (IsWhole())
        {
            return false;
        }

        if (IsVoid())
        {
            return true;
        }

        if (IsOpenZmin())
        {
            return false;
        }

        if (IsOpenZmax())
        {
            return false;
        }

        if (Zmax - Zmin < tol)
        {
            return true;
        }

        return false;
    }

    /// <summary>True if the box is thin in all three dimensions.</summary>
    public readonly bool IsThin(double tol)
    {
        if (IsWhole())
        {
            return false;
        }

        if (IsVoid())
        {
            return true;
        }

        if (IsOpenXmin() || IsOpenXmax() || Xmax - Xmin >= tol)
        {
            return false;
        }

        if (IsOpenYmin() || IsOpenYmax() || Ymax - Ymin >= tol)
        {
            return false;
        }

        if (IsOpenZmin() || IsOpenZmax() || Zmax - Zmin >= tol)
        {
            return false;
        }

        return true;
    }

    /// <summary>The box bounding this box transformed by <paramref name="T"/> (generally larger than needed).</summary>
    public readonly Bnd_Box Transformed(in gp_Trsf T)
    {
        if (IsVoid())
        {
            return new Bnd_Box();
        }
        else if (T.Form() == gp_Identity)
        {
            return this;
        }
        else if (T.Form() == gp_Translation)
        {
            if (!HasFinitePart())
            {
                return this;
            }

            gp_XYZ aDelta = T.TranslationPart();
            Bnd_Box aNewBox = this;
            aNewBox.Xmin += aDelta.X();
            aNewBox.Xmax += aDelta.X();
            aNewBox.Ymin += aDelta.Y();
            aNewBox.Ymax += aDelta.Y();
            aNewBox.Zmin += aDelta.Z();
            aNewBox.Zmax += aDelta.Z();
            return aNewBox;
        }

        Bnd_Box aNewBox2 = new();
        if (HasFinitePart())
        {
            Span<gp_Pnt> aCorners =
            [
                new gp_Pnt(Xmin, Ymin, Zmin),
                new gp_Pnt(Xmax, Ymin, Zmin),
                new gp_Pnt(Xmin, Ymax, Zmin),
                new gp_Pnt(Xmax, Ymax, Zmin),
                new gp_Pnt(Xmin, Ymin, Zmax),
                new gp_Pnt(Xmax, Ymin, Zmax),
                new gp_Pnt(Xmin, Ymax, Zmax),
                new gp_Pnt(Xmax, Ymax, Zmax),
            ];
            for (int aCornerIter = 0; aCornerIter < 8; ++aCornerIter)
            {
                aCorners[aCornerIter].Transform(T);
                aNewBox2.Add(aCorners[aCornerIter]);
            }
        }

        aNewBox2.Gap = Gap;
        if (!IsOpen())
        {
            return aNewBox2;
        }

        Span<gp_Dir> aDirs = stackalloc gp_Dir[6];
        int aNbDirs = 0;
        if (IsOpenXmin())
        {
            aDirs[aNbDirs++] = new gp_Dir(gp_Dir.D.NX);
        }

        if (IsOpenXmax())
        {
            aDirs[aNbDirs++] = new gp_Dir(gp_Dir.D.X);
        }

        if (IsOpenYmin())
        {
            aDirs[aNbDirs++] = new gp_Dir(gp_Dir.D.NY);
        }

        if (IsOpenYmax())
        {
            aDirs[aNbDirs++] = new gp_Dir(gp_Dir.D.Y);
        }

        if (IsOpenZmin())
        {
            aDirs[aNbDirs++] = new gp_Dir(gp_Dir.D.NZ);
        }

        if (IsOpenZmax())
        {
            aDirs[aNbDirs++] = new gp_Dir(gp_Dir.D.Z);
        }

        for (int aDirIter = 0; aDirIter < aNbDirs; ++aDirIter)
        {
            aDirs[aDirIter].Transform(T);
            aNewBox2.Add(aDirs[aDirIter]);
        }

        return aNewBox2;
    }

    /// <summary>True if the point is outside the box.</summary>
    public readonly bool IsOut(in gp_Pnt P)
    {
        if (IsWhole())
        {
            return false;
        }
        else if (IsVoid())
        {
            return true;
        }
        else
        {
            P.Coord(out double X, out double Y, out double Z);
            if (!IsOpenXmin() && (X < (Xmin - Gap)))
            {
                return true;
            }
            else if (!IsOpenXmax() && (X > (Xmax + Gap)))
            {
                return true;
            }
            else if (!IsOpenYmin() && (Y < (Ymin - Gap)))
            {
                return true;
            }
            else if (!IsOpenYmax() && (Y > (Ymax + Gap)))
            {
                return true;
            }
            else if (!IsOpenZmin() && (Z < (Zmin - Gap)))
            {
                return true;
            }
            else if (!IsOpenZmax() && (Z > (Zmax + Gap)))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    /// <summary>False if the plane intersects the box.</summary>
    public readonly bool IsOut(in gp_Pln P)
    {
        if (IsWhole())
        {
            return false;
        }
        else if (IsVoid())
        {
            return true;
        }
        else
        {
            P.Coefficients(out double A, out double B, out double C, out double D);
            double aXmin = GetXMin();
            double aXmax = GetXMax();
            double aYmin = GetYMin();
            double aYmax = GetYMax();
            double aZmin = GetZMin();
            double aZmax = GetZMax();
            double d = A * aXmin + B * aYmin + C * aZmin + D;
            bool plus = d > 0;
            if (plus != ((A * aXmin + B * aYmin + C * aZmax + D) > 0))
            {
                return false;
            }

            if (plus != ((A * aXmin + B * aYmax + C * aZmin + D) > 0))
            {
                return false;
            }

            if (plus != ((A * aXmin + B * aYmax + C * aZmax + D) > 0))
            {
                return false;
            }

            if (plus != ((A * aXmax + B * aYmin + C * aZmin + D) > 0))
            {
                return false;
            }

            if (plus != ((A * aXmax + B * aYmin + C * aZmax + D) > 0))
            {
                return false;
            }

            if (plus != ((A * aXmax + B * aYmax + C * aZmin + D) > 0))
            {
                return false;
            }

            return plus == ((A * aXmax + B * aYmax + C * aZmax + D) > 0);
        }
    }

    /// <summary>False if the line intersects the box.</summary>
    public readonly bool IsOut(in gp_Lin L)
    {
        if (IsWhole())
        {
            return false;
        }
        else if (IsVoid())
        {
            return true;
        }
        else
        {
            double xmin = 0, xmax = 0, ymin = 0, ymax = 0, zmin = 0, zmax = 0;
            double parmin, parmax, par1, par2;
            bool xToSet, yToSet;
            Get(out double myXmin, out double myYmin, out double myZmin, out double myXmax, out double myYmax, out double myZmax);

            gp_XYZ aLoc = L.Location().XYZ();
            gp_XYZ aDir = L.Direction().XYZ();
            double aDirX = Math.Abs(aDir.X());
            double aDirY = Math.Abs(aDir.Y());
            double aDirZ = Math.Abs(aDir.Z());

            if (aDirX > 0.0)
            {
                par1 = (myXmin - aLoc.X()) / aDir.X();
                par2 = (myXmax - aLoc.X()) / aDir.X();
                parmin = Min(par1, par2);
                parmax = Max(par1, par2);
                xToSet = true;
            }
            else
            {
                if (aLoc.X() < myXmin || myXmax < aLoc.X())
                {
                    return true;
                }

                xmin = aLoc.X();
                xmax = aLoc.X();
                parmin = -THE_BND_PRECISION_INFINITE;
                parmax = THE_BND_PRECISION_INFINITE;
                xToSet = false;
            }

            if (aDirY > 0.0)
            {
                par1 = (myYmin - aLoc.Y()) / aDir.Y();
                par2 = (myYmax - aLoc.Y()) / aDir.Y();
                // Check if parameter ranges from this axis are disjoint (early exit)
                if (parmax < Min(par1, par2) || parmin > Max(par1, par2))
                {
                    return true;
                }

                parmin = Max(parmin, Min(par1, par2));
                parmax = Min(parmax, Max(par1, par2));
                yToSet = true;
            }
            else
            {
                if (aLoc.Y() < myYmin || myYmax < aLoc.Y())
                {
                    return true;
                }

                ymin = aLoc.Y();
                ymax = aLoc.Y();
                yToSet = false;
            }

            if (aDirZ > 0.0)
            {
                par1 = (myZmin - aLoc.Z()) / aDir.Z();
                par2 = (myZmax - aLoc.Z()) / aDir.Z();
                // Check if parameter ranges from this axis are disjoint (early exit)
                if (parmax < Min(par1, par2) || parmin > Max(par1, par2))
                {
                    return true;
                }

                parmin = Max(parmin, Min(par1, par2));
                parmax = Min(parmax, Max(par1, par2));
                par1 = aLoc.Z() + parmin * aDir.Z();
                par2 = aLoc.Z() + parmax * aDir.Z();
                zmin = Min(par1, par2);
                zmax = Max(par1, par2);
            }
            else
            {
                if (aLoc.Z() < myZmin || myZmax < aLoc.Z())
                {
                    return true;
                }

                zmin = aLoc.Z();
                zmax = aLoc.Z();
            }

            if (zmax < myZmin || myZmax < zmin)
            {
                return true;
            }

            if (xToSet)
            {
                par1 = aLoc.X() + parmin * aDir.X();
                par2 = aLoc.X() + parmax * aDir.X();
                xmin = Min(par1, par2);
                xmax = Max(par1, par2);
            }

            if (xmax < myXmin || myXmax < xmin)
            {
                return true;
            }

            if (yToSet)
            {
                par1 = aLoc.Y() + parmin * aDir.Y();
                par2 = aLoc.Y() + parmax * aDir.Y();
                ymin = Min(par1, par2);
                ymax = Max(par1, par2);
            }

            if (ymax < myYmin || myYmax < ymin)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>False if <paramref name="Other"/> intersects or is inside this box.</summary>
    public readonly bool IsOut(in Bnd_Box Other)
    {
        // Fast path for non-open boxes with early exit
        if (Flags == 0 && Other.Flags == 0)
        {
            double aDelta0 = Other.Gap + Gap;
            // Early exit on first separating axis found
            if (Xmin - Other.Xmax > aDelta0)
            {
                return true;
            }

            if (Other.Xmin - Xmax > aDelta0)
            {
                return true;
            }

            if (Ymin - Other.Ymax > aDelta0)
            {
                return true;
            }

            if (Other.Ymin - Ymax > aDelta0)
            {
                return true;
            }

            if (Zmin - Other.Zmax > aDelta0)
            {
                return true;
            }

            if (Other.Zmin - Zmax > aDelta0)
            {
                return true;
            }

            return false;
        }

        // Handle special cases
        if (IsVoid() || Other.IsVoid())
        {
            return true;
        }

        if (IsWhole() || Other.IsWhole())
        {
            return false;
        }

        double aDelta = Other.Gap + Gap;

        // Check each axis with early exit
        if (!IsOpenXmin() && !Other.IsOpenXmax() && Xmin - Other.Xmax > aDelta)
        {
            return true;
        }

        if (!IsOpenXmax() && !Other.IsOpenXmin() && Other.Xmin - Xmax > aDelta)
        {
            return true;
        }

        if (!IsOpenYmin() && !Other.IsOpenYmax() && Ymin - Other.Ymax > aDelta)
        {
            return true;
        }

        if (!IsOpenYmax() && !Other.IsOpenYmin() && Other.Ymin - Ymax > aDelta)
        {
            return true;
        }

        if (!IsOpenZmin() && !Other.IsOpenZmax() && Zmin - Other.Zmax > aDelta)
        {
            return true;
        }

        if (!IsOpenZmax() && !Other.IsOpenZmin() && Other.Zmin - Zmax > aDelta)
        {
            return true;
        }

        return false;
    }

    /// <summary>False if <paramref name="Other"/> transformed by <paramref name="T"/> intersects or is inside this box.</summary>
    public readonly bool IsOut(in Bnd_Box Other, in gp_Trsf T) => IsOut(Other.Transformed(T));

    /// <summary>False if <paramref name="Other"/> transformed by <paramref name="T2"/> intersects or is inside this box transformed by <paramref name="T1"/>.</summary>
    public readonly bool IsOut(in gp_Trsf T1, in Bnd_Box Other, in gp_Trsf T2) => Transformed(T1).IsOut(Other.Transformed(T2));

    /// <summary>False if the flat band between two parallel lines through <paramref name="P1"/> and <paramref name="P2"/> with direction <paramref name="D"/> intersects the box.</summary>
    public readonly bool IsOut(in gp_Pnt P1, in gp_Pnt P2, in gp_Dir D)
    {
        if (IsWhole())
        {
            return false;
        }
        else if (IsVoid())
        {
            return true;
        }

        double eps = RealSmall();
        Get(out double myXmin, out double myYmin, out double myZmin, out double myXmax, out double myYmax, out double myZmax);

        if (Math.Abs(D.X()) < eps && Math.Abs(D.Y()) < eps)
        {
            return IsSegmentOut(myXmin, myYmin, myXmax, myYmax, P1.X(), P1.Y(), P2.X(), P2.Y());
        }

        if (Math.Abs(D.X()) < eps && Math.Abs(D.Z()) < eps)
        {
            return IsSegmentOut(myXmin, myZmin, myXmax, myZmax, P1.X(), P1.Z(), P2.X(), P2.Z());
        }

        if (Math.Abs(D.Y()) < eps && Math.Abs(D.Z()) < eps)
        {
            return IsSegmentOut(myYmin, myZmin, myYmax, myZmax, P1.Y(), P1.Z(), P2.Y(), P2.Z());
        }

        if (Math.Abs(D.X()) < eps)
        {
            if (!IsSegmentOut(myXmin, myZmin, myXmax, myZmax,
                              P1.X(), (myYmin - P1.Y()) * D.Z() / D.Y() + P1.Z(),
                              P2.X(), (myYmin - P2.Y()) * D.Z() / D.Y() + P2.Z()))
            {
                return false;
            }

            if (!IsSegmentOut(myXmin, myZmin, myXmax, myZmax,
                              P1.X(), (myYmax - P1.Y()) * D.Z() / D.Y() + P1.Z(),
                              P2.X(), (myYmax - P2.Y()) * D.Z() / D.Y() + P2.Z()))
            {
                return false;
            }

            if (!IsSegmentOut(myXmin, myYmin, myXmax, myYmax,
                              P1.X(), (myZmin - P1.Z()) * D.Y() / D.Z() + P1.Y(),
                              P2.X(), (myZmin - P2.Z()) * D.Y() / D.Z() + P2.Y()))
            {
                return false;
            }

            if (!IsSegmentOut(myXmin, myYmin, myXmax, myYmax,
                              P1.X(), (myZmax - P1.Z()) * D.Y() / D.Z() + P1.Y(),
                              P2.X(), (myZmax - P2.Z()) * D.Y() / D.Z() + P2.Y()))
            {
                return false;
            }

            return true;
        } // if(D.X() == 0)

        if (Math.Abs(D.Y()) < eps)
        {
            if (!IsSegmentOut(myYmin, myZmin, myYmax, myZmax,
                              P1.Y(), (myXmin - P1.X()) * D.Z() / D.X() + P1.Z(),
                              P2.Y(), (myXmin - P2.X()) * D.Z() / D.X() + P2.Z()))
            {
                return false;
            }

            if (!IsSegmentOut(myYmin, myZmin, myYmax, myZmax,
                              P1.Y(), (myXmax - P1.X()) * D.Z() / D.X() + P1.Z(),
                              P2.Y(), (myXmax - P2.X()) * D.Z() / D.X() + P2.Z()))
            {
                return false;
            }

            if (!IsSegmentOut(myYmin, myXmin, myYmax, myXmax,
                              P1.Y(), (myZmin - P1.Z()) * D.X() / D.Z() + P1.X(),
                              P2.Y(), (myZmin - P2.Z()) * D.X() / D.Z() + P2.X()))
            {
                return false;
            }

            if (!IsSegmentOut(myYmin, myXmin, myYmax, myXmax,
                              P1.Y(), (myZmax - P1.Z()) * D.X() / D.Z() + P1.X(),
                              P2.Y(), (myZmax - P2.Z()) * D.X() / D.Z() + P2.X()))
            {
                return false;
            }

            return true;
        } // if(D.Y() == 0)

        if (Math.Abs(D.Z()) < eps)
        {
            if (!IsSegmentOut(myZmin, myXmin, myZmax, myXmax,
                              P1.Z(), (myYmax - P1.Y()) * D.X() / D.Y() + P1.X(),
                              P2.Z(), (myYmax - P2.Y()) * D.X() / D.Y() + P2.X()))
            {
                return false;
            }

            if (!IsSegmentOut(myZmin, myXmin, myZmax, myXmax,
                              P1.Z(), (myYmin - P1.Y()) * D.X() / D.Y() + P1.X(),
                              P2.Z(), (myYmin - P2.Y()) * D.X() / D.Y() + P2.X()))
            {
                return false;
            }

            if (!IsSegmentOut(myZmin, myYmin, myZmax, myYmax,
                              P1.Z(), (myXmax - P1.X()) * D.Y() / D.X() + P1.Y(),
                              P2.Z(), (myXmax - P2.X()) * D.Y() / D.X() + P2.Y()))
            {
                return false;
            }

            if (!IsSegmentOut(myZmin, myYmin, myZmax, myYmax,
                              P1.Z(), (myXmin - P1.X()) * D.Y() / D.X() + P1.Y(),
                              P2.Z(), (myXmin - P2.X()) * D.Y() / D.X() + P2.Y()))
            {
                return false;
            }

            return true;
        } // if(D.Z() == 0)

        if (!IsSegmentOut(myXmin, myZmin, myXmax, myZmax,
                          (myYmin - P1.Y()) / D.Y() * D.X() + P1.X(),
                          (myYmin - P1.Y()) / D.Y() * D.Z() + P1.Z(),
                          (myYmin - P2.Y()) / D.Y() * D.X() + P2.X(),
                          (myYmin - P2.Y()) / D.Y() * D.Z() + P2.Z()))
        {
            return false;
        }

        if (!IsSegmentOut(myXmin, myZmin, myXmax, myZmax,
                          (myYmax - P1.Y()) / D.Y() * D.X() + P1.X(),
                          (myYmax - P1.Y()) / D.Y() * D.Z() + P1.Z(),
                          (myYmax - P2.Y()) / D.Y() * D.X() + P2.X(),
                          (myYmax - P2.Y()) / D.Y() * D.Z() + P2.Z()))
        {
            return false;
        }

        if (!IsSegmentOut(myXmin, myYmin, myXmax, myYmax,
                          (myZmin - P1.Z()) / D.Z() * D.X() + P1.X(),
                          (myZmin - P1.Z()) / D.Z() * D.Y() + P1.Y(),
                          (myZmin - P2.Z()) / D.Z() * D.X() + P2.X(),
                          (myZmin - P2.Z()) / D.Z() * D.Y() + P2.Y()))
        {
            return false;
        }

        if (!IsSegmentOut(myXmin, myYmin, myXmax, myYmax,
                          (myZmax - P1.Z()) / D.Z() * D.X() + P1.X(),
                          (myZmax - P1.Z()) / D.Z() * D.Y() + P1.Y(),
                          (myZmax - P2.Z()) / D.Z() * D.X() + P2.X(),
                          (myZmax - P2.Z()) / D.Z() * D.Y() + P2.Y()))
        {
            return false;
        }

        if (!IsSegmentOut(myZmin, myYmin, myZmax, myYmax,
                          (myXmin - P1.X()) / D.X() * D.Z() + P1.Z(),
                          (myXmin - P1.X()) / D.X() * D.Y() + P1.Y(),
                          (myXmin - P2.X()) / D.X() * D.Z() + P2.Z(),
                          (myXmin - P2.X()) / D.X() * D.Y() + P2.Y()))
        {
            return false;
        }

        if (!IsSegmentOut(myZmin, myYmin, myZmax, myYmax,
                          (myXmax - P1.X()) / D.X() * D.Z() + P1.Z(),
                          (myXmax - P1.X()) / D.X() * D.Y() + P1.Y(),
                          (myXmax - P2.X()) / D.X() * D.Z() + P2.Z(),
                          (myXmax - P2.X()) / D.X() * D.Y() + P2.Y()))
        {
            return false;
        }

        return true;
    }

    /// <summary>True if the point is inside or on the boundary of this box.</summary>
    public readonly bool Contains(in gp_Pnt theP) => !IsOut(theP);

    /// <summary>True if the other box intersects or is inside this box.</summary>
    public readonly bool Intersects(in Bnd_Box theOther) => !IsOut(theOther);

    /// <summary>The minimum distance between two boxes (0 if either is void).</summary>
    public readonly double Distance(in Bnd_Box Other)
    {
        if (IsVoid() || Other.IsVoid())
        {
            return 0.0;
        }

        Get(out double aXMinB1, out double aYMinB1, out double aZMinB1, out double aXMaxB1, out double aYMaxB1, out double aZMaxB1);
        Other.Get(out double aXMinB2, out double aYMinB2, out double aZMinB2, out double aXMaxB2, out double aYMaxB2, out double aZMaxB2);

        double aDistX = DistanceInDimension(aXMinB1, aXMaxB1, aXMinB2, aXMaxB2);
        double aDistY = DistanceInDimension(aYMinB1, aYMaxB1, aYMinB2, aYMaxB2);
        double aDistZ = DistanceInDimension(aZMinB1, aZMaxB1, aZMinB2, aZMaxB2);

        return Math.Sqrt(aDistX + aDistY + aDistZ);
    }

    /// <summary>The squared diagonal (0 if void).</summary>
    public readonly double SquareExtent()
    {
        if (IsVoid())
        {
            return 0.0;
        }

        double aDx = Xmax - Xmin + Gap + Gap;
        double aDy = Ymax - Ymin + Gap + Gap;
        double aDz = Zmax - Zmin + Gap + Gap;
        return aDx * aDx + aDy * aDy + aDz * aDz;
    }

    /// <summary>
    /// The finite part of an infinite box (this box if already finite). Can be void if the box has open sides and
    /// no finite points. Relies on the open flags: infinite points added with Add are returned as is.
    /// </summary>
    public readonly Bnd_Box FinitePart()
    {
        if (!HasFinitePart())
        {
            return new Bnd_Box();
        }

        Bnd_Box aBox = new();
        aBox.Update(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
        aBox.SetGap(Gap);
        return aBox;
    }

    /// <summary>True if this box has a finite part.</summary>
    public readonly bool HasFinitePart() => !IsVoid() && Xmax >= Xmin;

    // Dump, DumpJson and InitFromJson are left out: ToString replaces them.

    // Anonymous-namespace helpers of Bnd_Box.cxx

    // Computes minimum squared distance between two 1D intervals
    private static double DistMini2Box(double theR1Min, double theR1Max, double theR2Min, double theR2Max)
    {
        double aR1 = Square(theR1Min - theR2Max);
        double aR2 = Square(theR1Max - theR2Min);
        return Min(aR1, aR2);
    }

    // Computes squared distance in one dimension, returns 0 if intervals overlap
    private static double DistanceInDimension(double theMin1, double theMax1, double theMin2, double theMax2)
    {
        // Check if intervals overlap
        if ((theMin1 <= theMin2 && theMin2 <= theMax1) || (theMin2 <= theMin1 && theMin1 <= theMax2))
        {
            return 0.0;
        }

        return DistMini2Box(theMin1, theMax1, theMin2, theMax2);
    }

    // Tests if a 2D segment is outside a 2D box
    private static bool IsSegmentOut(double theX1, double theY1, double theX2, double theY2,
                                     double theXs1, double theYs1, double theXs2, double theYs2)
    {
        double anEps = RealSmall();
        double aXsMin = Min(theXs1, theXs2);
        double aXsMax = Max(theXs1, theXs2);
        double aYsMin = Min(theYs1, theYs2);
        double aYsMax = Max(theYs1, theYs2);

        if (aYsMax - aYsMin < anEps && (theY1 - theYs1 < anEps && theYs1 - theY2 < anEps)
            && ((aXsMin - theX1 < anEps && theX1 - aXsMax < anEps)
                || (aXsMin - theX2 < anEps && theX2 - aXsMax < anEps)
                || (theX1 - theXs1 < anEps && theXs1 - theX2 < anEps)))
        {
            return false;
        }

        if (aXsMax - aXsMin < anEps && (theX1 - theXs1 < anEps && theXs1 - theX2 < anEps)
            && ((aYsMin - theY1 < anEps && theY1 - aYsMax < anEps)
                || (aYsMin - theY2 < anEps && theY2 - aYsMax < anEps)
                || (theY1 - theYs1 < anEps && theYs1 - theY2 < anEps)))
        {
            return false;
        }

        if ((theXs1 < theX1 && theXs2 < theX1) || (theXs1 > theX2 && theXs2 > theX2)
            || (theYs1 < theY1 && theYs2 < theY1) || (theYs1 > theY2 && theYs2 > theY2))
        {
            return true;
        }

        if (Math.Abs(theXs2 - theXs1) > anEps)
        {
            double aYa = (Min(theX1, theX2) - theXs1) * (theYs2 - theYs1) / (theXs2 - theXs1) + theYs1;
            double aYb = (Max(theX1, theX2) - theXs1) * (theYs2 - theYs1) / (theXs2 - theXs1) + theYs1;
            if ((aYa < theY1 && aYb < theY1) || (aYa > theY2 && aYb > theY2))
            {
                return true;
            }
        }
        else if (Math.Abs(theYs2 - theYs1) > anEps)
        {
            double aXa = (Min(theY1, theY2) - theYs1) * (theXs2 - theXs1) / (theYs2 - theYs1) + theXs1;
            double aXb = (Max(theY1, theY2) - theYs1) * (theXs2 - theXs1) / (theYs2 - theYs1) + theXs1;
            if ((aXa < theX1 && aXb < theX1) || (aXa > theX2 && aXb > theX2))
            {
                return true;
            }
        }
        else
        {
            return true;
        }

        return false;
    }

    // Exact equality, for use as a value

    public readonly bool Equals(Bnd_Box other)
        => Xmin.Equals(other.Xmin) && Xmax.Equals(other.Xmax) && Ymin.Equals(other.Ymin) && Ymax.Equals(other.Ymax)
           && Zmin.Equals(other.Zmin) && Zmax.Equals(other.Zmax) && Gap.Equals(other.Gap) && Flags == other.Flags;

    public override readonly bool Equals(object? obj) => obj is Bnd_Box other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Xmin, Xmax, Ymin, Ymax, Zmin, Zmax, Gap, Flags);

    public static bool operator ==(in Bnd_Box theLeft, in Bnd_Box theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Bnd_Box theLeft, in Bnd_Box theRight) => !theLeft.Equals(theRight);

    /// <summary>The raw fields: Xmin, Xmax, Ymin, Ymax, Zmin, Zmax, Gap and the flag bits.</summary>
    public override readonly string ToString() => gp.Format("Bnd_Box", Xmin, Xmax, Ymin, Ymax, Zmin, Zmax, Gap, Flags);
}
