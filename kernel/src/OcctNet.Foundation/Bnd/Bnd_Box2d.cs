// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Box2d.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Box2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// A bounding box in 2D space, parallel to the axes. It may be open (infinite) in some directions, whole or void.
/// A gap is included on both sides in every direction when the finite bounds are consulted.
/// </summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_Box2d()</c>) is a void box with a null gap. <c>default(Bnd_Box2d)</c> is not: it is
/// a non-void box reduced to the origin. Ported code must write <c>new Bnd_Box2d()</c>.
/// </remarks>
public partial struct Bnd_Box2d : IEquatable<Bnd_Box2d>
{
    /// <summary>The box limits (Xmin, Xmax, Ymin, Ymax), gap and open directions included.</summary>
    public readonly record struct Limits(double Xmin, double Xmax, double Ymin, double Ymax);

    // Precision constant for infinite bounds
    private const double THE_BND_PRECISION_INFINITE = 1e+100;

    // Bit flags (OCCT's protected enum MaskFlags)
    private const int VoidMask = 0x01;
    private const int XminMask = 0x02;
    private const int XmaxMask = 0x04;
    private const int YminMask = 0x08;
    private const int YmaxMask = 0x10;
    private const int WholeMask = 0x1e;

    private double Xmin;
    private double Xmax;
    private double Ymin;
    private double Ymax;
    private double Gap;
    private int Flags;

    /// <summary>Creates an empty (void) box with a null gap.</summary>
    public Bnd_Box2d()
    {
        Xmin = RealLast();
        Xmax = -RealLast();
        Ymin = RealLast();
        Ymax = -RealLast();
        Gap = 0.0;
        Flags = VoidMask;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetWhole() => Flags = WholeMask;

    internal void SetVoid()
    {
        Xmin = RealLast();
        Xmax = -RealLast();
        Ymin = RealLast();
        Ymax = -RealLast();
        Gap = 0.0;
        Flags = VoidMask;
    }

    internal void Set(in gp_Pnt2d thePnt)
    {
        Flags = VoidMask;
        Gap = 0.0;
        Add(thePnt);
    }

    internal void Set(in gp_Pnt2d thePnt, in gp_Dir2d theDir)
    {
        Flags = VoidMask;
        Gap = 0.0;
        Add(thePnt, theDir);
    }

    internal void Update(double x, double y, double X, double Y)
    {
        if ((Flags & VoidMask) != 0)
        {
            Xmin = x;
            Ymin = y;
            Xmax = X;
            Ymax = Y;
            Flags &= ~VoidMask;
        }
        else
        {
            if ((Flags & XminMask) == 0)
            {
                Xmin = Min(Xmin, x);
            }

            if ((Flags & XmaxMask) == 0)
            {
                Xmax = Max(Xmax, X);
            }

            if ((Flags & YminMask) == 0)
            {
                Ymin = Min(Ymin, y);
            }

            if ((Flags & YmaxMask) == 0)
            {
                Ymax = Max(Ymax, Y);
            }
        }
    }

    internal void Update(double X, double Y)
    {
        if ((Flags & VoidMask) != 0)
        {
            Xmin = X;
            Ymin = Y;
            Xmax = X;
            Ymax = Y;
            Flags &= ~VoidMask;
        }
        else
        {
            if ((Flags & XminMask) == 0)
            {
                Xmin = Min(Xmin, X);
            }

            if ((Flags & XmaxMask) == 0)
            {
                Xmax = Max(Xmax, X);
            }

            if ((Flags & YminMask) == 0)
            {
                Ymin = Min(Ymin, Y);
            }

            if ((Flags & YmaxMask) == 0)
            {
                Ymax = Max(Ymax, Y);
            }
        }
    }

    internal void SetGap(double Tol) => Gap = Math.Abs(Tol);

    internal void Enlarge(double theTol) => Gap = Max(Gap, Math.Abs(theTol));

    internal void OpenXmin() => Flags |= XminMask;

    internal void OpenXmax() => Flags |= XmaxMask;

    internal void OpenYmin() => Flags |= YminMask;

    internal void OpenYmax() => Flags |= YmaxMask;

    internal void Add(in Bnd_Box2d Other)
    {
        if (IsWhole())
        {
            return;
        }
        else if (Other.IsVoid())
        {
            return;
        }
        else if (Other.IsWhole())
        {
            SetWhole();
        }
        else if (IsVoid())
        {
            this = Other;
        }
        else
        {
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

            Gap = Max(Gap, Other.Gap);
        }
    }

    internal void Add(in gp_Pnt2d thePnt) => Update(thePnt.X(), thePnt.Y());

    internal void Add(in gp_Pnt2d thePnt, in gp_Dir2d theDir)
    {
        Add(thePnt);
        Add(theDir);
    }

    internal void Add(in gp_Dir2d D)
    {
        double DX = D.X();
        double DY = D.Y();

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
    }

    // Copies with a mutator applied (the public way to "set")

    /// <summary>A void box: the same as <c>new Bnd_Box2d()</c>.</summary>
    public static Bnd_Box2d Void() => new();

    /// <summary>A box bounding the single point <paramref name="thePnt"/> (OCCT's <c>Set(P)</c>).</summary>
    public static Bnd_Box2d FromPoint(in gp_Pnt2d thePnt)
    {
        var aBox = new Bnd_Box2d();
        aBox.Set(thePnt);
        return aBox;
    }

    /// <summary>A box bounding the half-line from <paramref name="thePnt"/> along <paramref name="theDir"/> (OCCT's <c>Set(P, D)</c>).</summary>
    public static Bnd_Box2d FromPoint(in gp_Pnt2d thePnt, in gp_Dir2d theDir)
    {
        var aBox = new Bnd_Box2d();
        aBox.Set(thePnt, theDir);
        return aBox;
    }

    /// <summary>This box made infinite in all directions (its finite bounds are kept, as in OCCT).</summary>
    public readonly Bnd_Box2d WithWhole()
    {
        Bnd_Box2d aCopy = this;
        aCopy.SetWhole();
        return aCopy;
    }

    public readonly Bnd_Box2d WithVoid() => new();

    public readonly Bnd_Box2d WithGap(double Tol)
    {
        Bnd_Box2d aCopy = this;
        aCopy.SetGap(Tol);
        return aCopy;
    }

    public readonly Bnd_Box2d Updated(double x, double y, double X, double Y)
    {
        Bnd_Box2d aCopy = this;
        aCopy.Update(x, y, X, Y);
        return aCopy;
    }

    public readonly Bnd_Box2d Updated(double X, double Y)
    {
        Bnd_Box2d aCopy = this;
        aCopy.Update(X, Y);
        return aCopy;
    }

    public readonly Bnd_Box2d Enlarged(double theTol)
    {
        Bnd_Box2d aCopy = this;
        aCopy.Enlarge(theTol);
        return aCopy;
    }

    public readonly Bnd_Box2d OpenedXmin()
    {
        Bnd_Box2d aCopy = this;
        aCopy.OpenXmin();
        return aCopy;
    }

    public readonly Bnd_Box2d OpenedXmax()
    {
        Bnd_Box2d aCopy = this;
        aCopy.OpenXmax();
        return aCopy;
    }

    public readonly Bnd_Box2d OpenedYmin()
    {
        Bnd_Box2d aCopy = this;
        aCopy.OpenYmin();
        return aCopy;
    }

    public readonly Bnd_Box2d OpenedYmax()
    {
        Bnd_Box2d aCopy = this;
        aCopy.OpenYmax();
        return aCopy;
    }

    public readonly Bnd_Box2d Added(in Bnd_Box2d Other)
    {
        Bnd_Box2d aCopy = this;
        aCopy.Add(Other);
        return aCopy;
    }

    public readonly Bnd_Box2d Added(in gp_Pnt2d thePnt)
    {
        Bnd_Box2d aCopy = this;
        aCopy.Add(thePnt);
        return aCopy;
    }

    public readonly Bnd_Box2d Added(in gp_Pnt2d thePnt, in gp_Dir2d theDir)
    {
        Bnd_Box2d aCopy = this;
        aCopy.Add(thePnt, theDir);
        return aCopy;
    }

    public readonly Bnd_Box2d Added(in gp_Dir2d D)
    {
        Bnd_Box2d aCopy = this;
        aCopy.Add(D);
        return aCopy;
    }

    // Queries

    public readonly double GetGap() => Gap;

    /// <summary>The bounds, gap included; open directions give ±1e100. Throws Standard_ConstructionError if void.</summary>
    public readonly void Get(out double x, out double y, out double Xm, out double Ym)
    {
        if ((Flags & VoidMask) != 0)
        {
            throw new Standard_ConstructionError("Bnd_Box is void");
        }

        x = GetXMin();
        Xm = GetXMax();
        y = GetYMin();
        Ym = GetYMax();
    }

    /// <summary>The bounds as a <see cref="Limits"/>, gap included; for a void box, the raw internal values.</summary>
    public readonly Limits Get() => new(GetXMin(), GetXMax(), GetYMin(), GetYMax());

    public readonly double GetXMin() => (Flags & XminMask) != 0 ? -THE_BND_PRECISION_INFINITE : Xmin - Gap;

    public readonly double GetXMax() => (Flags & XmaxMask) != 0 ? THE_BND_PRECISION_INFINITE : Xmax + Gap;

    public readonly double GetYMin() => (Flags & YminMask) != 0 ? -THE_BND_PRECISION_INFINITE : Ymin - Gap;

    public readonly double GetYMax() => (Flags & YmaxMask) != 0 ? THE_BND_PRECISION_INFINITE : Ymax + Gap;

    /// <summary>The center, gap included, or null if the box is void.</summary>
    public readonly gp_Pnt2d? Center()
    {
        if (IsVoid())
        {
            return null;
        }

        return new gp_Pnt2d(0.5 * (GetXMin() + GetXMax()), 0.5 * (GetYMin() + GetYMax()));
    }

    public readonly bool IsOpenXmin() => (Flags & XminMask) != 0;

    public readonly bool IsOpenXmax() => (Flags & XmaxMask) != 0;

    public readonly bool IsOpenYmin() => (Flags & YminMask) != 0;

    public readonly bool IsOpenYmax() => (Flags & YmaxMask) != 0;

    public readonly bool IsWhole() => (Flags & WholeMask) == WholeMask;

    public readonly bool IsVoid() => (Flags & VoidMask) != 0;

    /// <summary>The box bounding this box transformed by <paramref name="T"/>.</summary>
    public readonly Bnd_Box2d Transformed(in gp_Trsf2d T)
    {
        gp_TrsfForm aF = T.Form();
        Bnd_Box2d aNewBox = this;
        if (IsVoid())
        {
            return aNewBox;
        }

        if (aF == gp_Identity)
        {
        }
        else if (aF == gp_Translation)
        {
            T.TranslationPart().Coord(out double aDX, out double aDY);
            if ((Flags & XminMask) == 0)
            {
                aNewBox.Xmin += aDX;
            }

            if ((Flags & XmaxMask) == 0)
            {
                aNewBox.Xmax += aDX;
            }

            if ((Flags & YminMask) == 0)
            {
                aNewBox.Ymin += aDY;
            }

            if ((Flags & YmaxMask) == 0)
            {
                aNewBox.Ymax += aDY;
            }
        }
        else
        {
            Span<gp_Pnt2d> aP = stackalloc gp_Pnt2d[4];
            Span<bool> aVertex = stackalloc bool[4];
            aVertex[0] = true;
            aVertex[1] = true;
            aVertex[2] = true;
            aVertex[3] = true;
            Span<gp_Dir2d> aD = stackalloc gp_Dir2d[6];
            int aNbDirs = 0;

            if ((Flags & XminMask) != 0)
            {
                aD[aNbDirs++] = new gp_Dir2d(gp_Dir2d.D.NX);
                aVertex[0] = aVertex[2] = false;
            }

            if ((Flags & XmaxMask) != 0)
            {
                aD[aNbDirs++] = new gp_Dir2d(gp_Dir2d.D.X);
                aVertex[1] = aVertex[3] = false;
            }

            if ((Flags & YminMask) != 0)
            {
                aD[aNbDirs++] = new gp_Dir2d(gp_Dir2d.D.NY);
                aVertex[0] = aVertex[1] = false;
            }

            if ((Flags & YmaxMask) != 0)
            {
                aD[aNbDirs++] = new gp_Dir2d(gp_Dir2d.D.Y);
                aVertex[2] = aVertex[3] = false;
            }

            aNewBox.SetVoid();

            for (int i = 0; i < aNbDirs; i++)
            {
                aD[i].Transform(T);
                aNewBox.Add(aD[i]);
            }

            aP[0].SetCoord(Xmin, Ymin);
            aP[1].SetCoord(Xmax, Ymin);
            aP[2].SetCoord(Xmin, Ymax);
            aP[3].SetCoord(Xmax, Ymax);
            if (aVertex[0])
            {
                aP[0].Transform(T);
                aNewBox.Add(aP[0]);
            }

            if (aVertex[1])
            {
                aP[1].Transform(T);
                aNewBox.Add(aP[1]);
            }

            if (aVertex[2])
            {
                aP[2].Transform(T);
                aNewBox.Add(aP[2]);
            }

            if (aVertex[3])
            {
                aP[3].Transform(T);
                aNewBox.Add(aP[3]);
            }

            aNewBox.Gap = Gap;
        }

        return aNewBox;
    }

    /// <summary>True if the point is outside the box.</summary>
    public readonly bool IsOut(in gp_Pnt2d P)
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
            double X = P.X();
            double Y = P.Y();
            if ((Flags & XminMask) == 0 && (X < (Xmin - Gap)))
            {
                return true;
            }
            else if ((Flags & XmaxMask) == 0 && (X > (Xmax + Gap)))
            {
                return true;
            }
            else if ((Flags & YminMask) == 0 && (Y < (Ymin - Gap)))
            {
                return true;
            }
            else if ((Flags & YmaxMask) == 0 && (Y > (Ymax + Gap)))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    /// <summary>True if the line does not intersect the box.</summary>
    public readonly bool IsOut(in gp_Lin2d theL)
    {
        if (IsWhole())
        {
            return false;
        }

        if (IsVoid())
        {
            return true;
        }

        Get(out double aXMin, out double aYMin, out double aXMax, out double aYMax);

        gp_XY aCenter = new((aXMin + aXMax) / 2, (aYMin + aYMax) / 2);
        gp_XY aHeigh = new(Math.Abs(aXMax - aCenter.X()), Math.Abs(aYMax - aCenter.Y()));

        Span<double> aProd =
        [
            theL.Direction().XY() ^ (aCenter - theL.Location().XY()),
            theL.Direction().X() * aHeigh.Y(),
            theL.Direction().Y() * aHeigh.X(),
        ];
        bool aStatus = (Math.Abs(aProd[0]) > (Math.Abs(aProd[1]) + Math.Abs(aProd[2])));
        return aStatus;
    }

    /// <summary>True if the segment [theP0, theP1] does not intersect the box.</summary>
    public readonly bool IsOut(in gp_Pnt2d theP0, in gp_Pnt2d theP1)
    {
        if (IsWhole())
        {
            return false;
        }

        if (IsVoid())
        {
            return true;
        }

        bool aStatus = true;
        Get(out double aLocXMin, out double aLocYMin, out double aLocXMax, out double aLocYMax);

        //// Intersect the line containing the segment.
        gp_XY aSegDelta = theP1.XY() - theP0.XY();

        gp_XY aCenter = new((aLocXMin + aLocXMax) / 2, (aLocYMin + aLocYMax) / 2);
        gp_XY aHeigh = new(Math.Abs(aLocXMax - aCenter.X()), Math.Abs(aLocYMax - aCenter.Y()));

        Span<double> aProd =
        [
            aSegDelta ^ (aCenter - theP0.XY()),
            aSegDelta.X() * aHeigh.Y(),
            aSegDelta.Y() * aHeigh.X(),
        ];

        if ((Math.Abs(aProd[0]) <= (Math.Abs(aProd[1]) + Math.Abs(aProd[2]))))
        {
            // Intersection with line detected; check the segment as bounding box
            gp_XY aHSeg = new(0.5 * aSegDelta.X(), 0.5 * aSegDelta.Y());
            gp_XY aHSegAbs = new(Math.Abs(aHSeg.X()), Math.Abs(aHSeg.Y()));
            aStatus = ((Math.Abs((theP0.XY() + aHSeg - aCenter).X()) > (aHeigh + aHSegAbs).X())
                       || (Math.Abs((theP0.XY() + aHSeg - aCenter).Y()) > (aHeigh + aHSegAbs).Y()));
        }

        return aStatus;
    }

    /// <summary>True if <paramref name="Other"/> does not intersect this box.</summary>
    public readonly bool IsOut(in Bnd_Box2d Other)
    {
        // Fast path for non-open, non-void, non-whole boxes (most common case)
        if (Flags == 0 && Other.Flags == 0)
        {
            double aDelta = Other.Gap + Gap;
            if (Xmin - Other.Xmax > aDelta)
            {
                return true;
            }

            if (Other.Xmin - Xmax > aDelta)
            {
                return true;
            }

            if (Ymin - Other.Ymax > aDelta)
            {
                return true;
            }

            if (Other.Ymin - Ymax > aDelta)
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

        Other.Get(out double OXmin, out double OYmin, out double OXmax, out double OYmax);
        if ((Flags & XminMask) == 0 && (OXmax < (Xmin - Gap)))
        {
            return true;
        }

        if ((Flags & XmaxMask) == 0 && (OXmin > (Xmax + Gap)))
        {
            return true;
        }

        if ((Flags & YminMask) == 0 && (OYmax < (Ymin - Gap)))
        {
            return true;
        }

        if ((Flags & YmaxMask) == 0 && (OYmin > (Ymax + Gap)))
        {
            return true;
        }

        return false;
    }

    public readonly bool Contains(in gp_Pnt2d theP) => !IsOut(theP);

    public readonly bool Intersects(in Bnd_Box2d theOther) => !IsOut(theOther);

    /// <summary>The minimum distance between two boxes (0 if either is void).</summary>
    public readonly double Distance(in Bnd_Box2d theOther)
    {
        if (IsVoid() || theOther.IsVoid())
        {
            return 0.0;
        }

        Get(out double aXMin1, out double aYMin1, out double aXMax1, out double aYMax1);
        theOther.Get(out double aXMin2, out double aYMin2, out double aXMax2, out double aYMax2);

        double aDx = DistAxis(aXMin1, aXMax1, aXMin2, aXMax2);
        double aDy = DistAxis(aYMin1, aYMax1, aYMin2, aYMax2);

        return Math.Sqrt(aDx + aDy);
    }

    // Compute squared distance per axis (a lambda in OCCT)
    private static double DistAxis(double theMin1, double theMax1, double theMin2, double theMax2)
    {
        if (theMin1 > theMax2)
        {
            double aD = theMin1 - theMax2;
            return aD * aD;
        }

        if (theMin2 > theMax1)
        {
            double aD = theMin2 - theMax1;
            return aD * aD;
        }

        return 0.0;
    }

    public readonly bool IsOut(in Bnd_Box2d theOther, in gp_Trsf2d theTrsf) => IsOut(theOther.Transformed(theTrsf));

    public readonly bool IsOut(in gp_Trsf2d T1, in Bnd_Box2d Other, in gp_Trsf2d T2) => Transformed(T1).IsOut(Other.Transformed(T2));

    /// <summary>The squared diagonal (0 if void).</summary>
    public readonly double SquareExtent()
    {
        if (IsVoid())
        {
            return 0.0;
        }

        double aDx = Xmax - Xmin + Gap + Gap;
        double aDy = Ymax - Ymin + Gap + Gap;
        return aDx * aDx + aDy * aDy;
    }

    // Dump is left out: ToString replaces it.

    // Exact equality, for use as a value

    public readonly bool Equals(Bnd_Box2d other)
        => Xmin.Equals(other.Xmin) && Xmax.Equals(other.Xmax) && Ymin.Equals(other.Ymin) && Ymax.Equals(other.Ymax)
           && Gap.Equals(other.Gap) && Flags == other.Flags;

    public override readonly bool Equals(object? obj) => obj is Bnd_Box2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Xmin, Xmax, Ymin, Ymax, Gap, Flags);

    public static bool operator ==(in Bnd_Box2d theLeft, in Bnd_Box2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Bnd_Box2d theLeft, in Bnd_Box2d theRight) => !theLeft.Equals(theRight);

    /// <summary>The raw fields: Xmin, Xmax, Ymin, Ymax, Gap and the flag bits.</summary>
    public override readonly string ToString() => gp.Format("Bnd_Box2d", Xmin, Xmax, Ymin, Ymax, Gap, Flags);
}
