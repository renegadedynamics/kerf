// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyAlgo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using static Occt.HLRAlgo_PolyMask;

namespace Occt;

/// <summary>
/// The polygonal hidden-line removal on triangulations: indexes the shells' segments and hiding triangles in a
/// coarse grid of the projected box, then hides each segment by the triangles in front of it.
/// </summary>
/// <remarks>
/// A working object (a transient), as OCCT's. <see cref="Hide"/> and <see cref="Show"/> return a reference to the
/// current segment's points, as OCCT's.
/// </remarks>
public class HLRAlgo_PolyAlgo : Transient
{
    private NCollection_Array1<HLRAlgo_PolyShellData?> myHShell = new();
    private HLRAlgo_PolyData.Triangle myTriangle;
    private readonly NCollection_List<HLRAlgo_BiPoint>.Iterator mySegListIt = new();
    private int myNbrShell;
    private int myCurShell;
    private bool myFound;

    public HLRAlgo_PolyAlgo()
    {
        myNbrShell = 0;
        myCurShell = 0;
        myFound = false;
        myTriangle.TolParam = 0.00000001;
        myTriangle.TolAng = 0.0001;
    }

    /// <summary>Sizes the shells array to <paramref name="theNbShells"/>.</summary>
    public void Init(int theNbShells)
    {
        myHShell.Resize(1, theNbShells, false);
    }

    public NCollection_Array1<HLRAlgo_PolyShellData?> PolyShell() => myHShell;

    public NCollection_Array1<HLRAlgo_PolyShellData?> ChangePolyShell() => myHShell;

    public void Clear()
    {
        myHShell = new NCollection_Array1<HLRAlgo_PolyShellData?>();
        myNbrShell = 0;
    }

    /// <summary>Prepares all the data: the grid indices of the segments, triangles, faces and shells.</summary>
    public void Update()
    {
        int j;
        int nxMin, nyMin, nzMin, nxMax, nyMax, nzMax;
        double xShellMin, yShellMin, zShellMin;
        double xShellMax, yShellMax, zShellMax;
        double xPolyTMin, yPolyTMin, zPolyTMin;
        double xPolyTMax, yPolyTMax, zPolyTMax;
        double xTrianMin, yTrianMin, zTrianMin;
        double xTrianMax, yTrianMax, zTrianMax;
        double xSegmnMin, ySegmnMin, zSegmnMin;
        double xSegmnMax, ySegmnMax, zSegmnMax;
        double Big = global::Kerf.Engine.Foundation.Precision.Infinite();
        Box aBox = new();

        myNbrShell = myHShell.Length();
        for (int aShellIter = myHShell.Lower(); aShellIter <= myHShell.Upper(); ++aShellIter)
        {
            HLRAlgo_PolyShellData aPsd = myHShell[aShellIter]!;
            aPsd.UpdateGlobalMinMax(ref aBox);
        }

        (double aXMin, double aXMax, double aYMin, double aYMax, double aZMin, double aZMax) = aBox.Get();
        double dx = aXMax - aXMin;
        double dy = aYMax - aYMin;
        double dz = aZMax - aZMin;
        double precad = dx;
        if (precad < dy)
        {
            precad = dy;
        }
        if (precad < dz)
        {
            precad = dz;
        }
        myTriangle.Tolerance = precad * myTriangle.TolParam;
        precad = precad * 0.01;
        double SurDX = 1020 / (dx + precad);
        double SurDY = 1020 / (dy + precad);
        double SurDZ = 508 / (dz + precad);
        precad = precad * 0.5;
        double DecaX = -aXMin + precad;
        double DecaY = -aYMin + precad;
        double DecaZ = -aZMin + precad;

        for (int aShellIter = myHShell.Lower(); aShellIter <= myHShell.Upper(); ++aShellIter)
        {
            HLRAlgo_PolyShellData aPsd = myHShell[aShellIter]!;
            ref HLRAlgo_PolyShellData.ShellIndices aShellIndices = ref aPsd.Indices();
            xShellMin = Big;
            yShellMin = Big;
            zShellMin = Big;
            xShellMax = -Big;
            yShellMax = -Big;
            zShellMax = -Big;

            for (mySegListIt.Initialize(aPsd.Edges()); mySegListIt.More(); mySegListIt.Next())
            {
                ref HLRAlgo_BiPoint BP = ref mySegListIt.ChangeValue();
                ref HLRAlgo_BiPoint.PointsT aPoints = ref BP.Points();
                ref HLRAlgo_BiPoint.IndicesT theIndices = ref BP.Indices();
                if (aPoints.PntP1.X() < aPoints.PntP2.X())
                {
                    xSegmnMin = aPoints.PntP1.X();
                    xSegmnMax = aPoints.PntP2.X();
                }
                else
                {
                    xSegmnMin = aPoints.PntP2.X();
                    xSegmnMax = aPoints.PntP1.X();
                }
                if (aPoints.PntP1.Y() < aPoints.PntP2.Y())
                {
                    ySegmnMin = aPoints.PntP1.Y();
                    ySegmnMax = aPoints.PntP2.Y();
                }
                else
                {
                    ySegmnMin = aPoints.PntP2.Y();
                    ySegmnMax = aPoints.PntP1.Y();
                }
                if (aPoints.PntP1.Z() < aPoints.PntP2.Z())
                {
                    zSegmnMin = aPoints.PntP1.Z();
                    zSegmnMax = aPoints.PntP2.Z();
                }
                else
                {
                    zSegmnMin = aPoints.PntP2.Z();
                    zSegmnMax = aPoints.PntP1.Z();
                }
                nxMin = (int)((DecaX + xSegmnMin) * SurDX);
                nyMin = (int)((DecaY + ySegmnMin) * SurDY);
                nzMin = (int)((DecaZ + zSegmnMin) * SurDZ);
                nxMax = (int)((DecaX + xSegmnMax) * SurDX);
                nyMax = (int)((DecaY + ySegmnMax) * SurDY);
                nzMax = (int)((DecaZ + zSegmnMax) * SurDZ);
                theIndices.MinSeg = nyMin + (nxMin << 11);
                theIndices.MinSeg <<= 10;
                theIndices.MinSeg += nzMin;
                theIndices.MaxSeg = nyMax + (nxMax << 11);
                theIndices.MaxSeg <<= 10;
                theIndices.MaxSeg += nzMax + 0x00000200;
                if (xShellMin > xSegmnMin)
                {
                    xShellMin = xSegmnMin;
                }
                if (xShellMax < xSegmnMax)
                {
                    xShellMax = xSegmnMax;
                }
                if (yShellMin > ySegmnMin)
                {
                    yShellMin = ySegmnMin;
                }
                if (yShellMax < ySegmnMax)
                {
                    yShellMax = ySegmnMax;
                }
                if (zShellMin > zSegmnMin)
                {
                    zShellMin = zSegmnMin;
                }
                if (zShellMax < zSegmnMax)
                {
                    zShellMax = zSegmnMax;
                }
            }
            NCollection_Array1<HLRAlgo_PolyData?> aPolyg = aPsd.PolyData();
            int nbFace = aPolyg.Upper();
            int nbFaHi = 0;
            for (j = 1; j <= nbFace; j++)
            {
                HLRAlgo_PolyData aPd = aPolyg[j]!;
                if (aPd.Hiding())
                {
                    nbFaHi++;
                    xPolyTMin = Big;
                    yPolyTMin = Big;
                    zPolyTMin = Big;
                    xPolyTMax = -Big;
                    yPolyTMax = -Big;
                    zPolyTMax = -Big;
                    int otheri, nbHide = 0; // min,max;
                    double X1, X2, X3, Y1, Y2, Y3, Z1, Z2, Z3;
                    double dn, dnx, dny, dnz, dx1, dy1, dz1, dx2, dy2, dz2, dx3, dy3;
                    double adx1, ady1, adx2, ady2, adx3, ady3;
                    double a = 0.0, b = 0.0, c = 0.0, d = 0.0;
                    ref HLRAlgo_PolyData.FaceIndices PolyTIndices = ref aPd.Indices();
                    NCollection_Array1<Xyz> Nodes = aPd.Nodes();
                    NCollection_Array1<HLRAlgo_TriangleData> TData = aPd.TData();
                    NCollection_Array1<HLRAlgo_PolyHidingData> PHDat = aPd.PHDat();
                    int nbT = TData.Upper();

                    for (otheri = 1; otheri <= nbT; otheri++)
                    {
                        ref HLRAlgo_TriangleData aTD = ref TData[otheri];
                        if ((aTD.Flags & HLRAlgo_PolyMask_FMskHiding) != 0)
                        {
                            Xyz P1 = Nodes.Value(aTD.Node1);
                            Xyz P2 = Nodes.Value(aTD.Node2);
                            Xyz P3 = Nodes.Value(aTD.Node3);
                            X1 = P1.X();
                            Y1 = P1.Y();
                            Z1 = P1.Z();
                            X2 = P2.X();
                            Y2 = P2.Y();
                            Z2 = P2.Z();
                            X3 = P3.X();
                            Y3 = P3.Y();
                            Z3 = P3.Z();
                            xTrianMax = xTrianMin = X1;
                            yTrianMax = yTrianMin = Y1;
                            zTrianMax = zTrianMin = Z1;
                            if (xTrianMin > X2)
                            {
                                xTrianMin = X2;
                            }
                            else if (xTrianMax < X2)
                            {
                                xTrianMax = X2;
                            }
                            if (yTrianMin > Y2)
                            {
                                yTrianMin = Y2;
                            }
                            else if (yTrianMax < Y2)
                            {
                                yTrianMax = Y2;
                            }
                            if (zTrianMin > Z2)
                            {
                                zTrianMin = Z2;
                            }
                            else if (zTrianMax < Z2)
                            {
                                zTrianMax = Z2;
                            }
                            if (xTrianMin > X3)
                            {
                                xTrianMin = X3;
                            }
                            else if (xTrianMax < X3)
                            {
                                xTrianMax = X3;
                            }
                            if (yTrianMin > Y3)
                            {
                                yTrianMin = Y3;
                            }
                            else if (yTrianMax < Y3)
                            {
                                yTrianMax = Y3;
                            }
                            if (zTrianMin > Z3)
                            {
                                zTrianMin = Z3;
                            }
                            else if (zTrianMax < Z3)
                            {
                                zTrianMax = Z3;
                            }
                            nxMin = (int)((DecaX + xTrianMin) * SurDX);
                            nyMin = (int)((DecaY + yTrianMin) * SurDY);
                            nzMin = (int)((DecaZ + zTrianMin) * SurDZ);
                            nxMax = (int)((DecaX + xTrianMax) * SurDX);
                            nyMax = (int)((DecaY + yTrianMax) * SurDY);
                            nzMax = (int)((DecaZ + zTrianMax) * SurDZ);
                            int MinTrian, MaxTrian;
                            MinTrian = nyMin + (nxMin << 11);
                            MinTrian <<= 10;
                            MinTrian += nzMin - 0x00000200;
                            MaxTrian = nyMax + (nxMax << 11);
                            MaxTrian <<= 10;
                            MaxTrian += nzMax;
                            dx1 = X2 - X1;
                            dy1 = Y2 - Y1;
                            dz1 = Z2 - Z1;
                            dx2 = X3 - X2;
                            dy2 = Y3 - Y2;
                            dz2 = Z3 - Z2;
                            dx3 = X1 - X3;
                            dy3 = Y1 - Y3;
                            dnx = dy1 * dz2 - dy2 * dz1;
                            dny = dz1 * dx2 - dz2 * dx1;
                            dnz = dx1 * dy2 - dx2 * dy1;
                            dn = Math.Sqrt(dnx * dnx + dny * dny + dnz * dnz);
                            if (dn > 0)
                            {
                                a = dnx / dn;
                                b = dny / dn;
                                c = dnz / dn;
                            }
                            d = a * X1 + b * Y1 + c * Z1;
                            nbHide++;
                            PHDat[nbHide].Set(otheri, MinTrian, MaxTrian, a, b, c, d);
                            adx1 = dx1;
                            ady1 = dy1;
                            if (dx1 < 0)
                            {
                                adx1 = -dx1;
                            }
                            if (dy1 < 0)
                            {
                                ady1 = -dy1;
                            }
                            adx2 = dx2;
                            ady2 = dy2;
                            if (dx2 < 0)
                            {
                                adx2 = -dx2;
                            }
                            if (dy2 < 0)
                            {
                                ady2 = -dy2;
                            }
                            adx3 = dx3;
                            ady3 = dy3;
                            if (dx3 < 0)
                            {
                                adx3 = -dx3;
                            }
                            if (dy3 < 0)
                            {
                                ady3 = -dy3;
                            }
                            if (adx1 > ady1)
                            {
                                aTD.Flags |= HLRAlgo_PolyMask_EMskGrALin1;
                            }
                            else
                            {
                                aTD.Flags &= ~HLRAlgo_PolyMask_EMskGrALin1;
                            }
                            if (adx2 > ady2)
                            {
                                aTD.Flags |= HLRAlgo_PolyMask_EMskGrALin2;
                            }
                            else
                            {
                                aTD.Flags &= ~HLRAlgo_PolyMask_EMskGrALin2;
                            }
                            if (adx3 > ady3)
                            {
                                aTD.Flags |= HLRAlgo_PolyMask_EMskGrALin3;
                            }
                            else
                            {
                                aTD.Flags &= ~HLRAlgo_PolyMask_EMskGrALin3;
                            }
                            if (xPolyTMin > xTrianMin)
                            {
                                xPolyTMin = xTrianMin;
                            }
                            if (xPolyTMax < xTrianMax)
                            {
                                xPolyTMax = xTrianMax;
                            }
                            if (yPolyTMin > yTrianMin)
                            {
                                yPolyTMin = yTrianMin;
                            }
                            if (yPolyTMax < yTrianMax)
                            {
                                yPolyTMax = yTrianMax;
                            }
                            if (zPolyTMin > zTrianMin)
                            {
                                zPolyTMin = zTrianMin;
                            }
                            if (zPolyTMax < zTrianMax)
                            {
                                zPolyTMax = zTrianMax;
                            }
                        }
                    }
                    nxMin = (int)((DecaX + xPolyTMin) * SurDX);
                    nyMin = (int)((DecaY + yPolyTMin) * SurDY);
                    nzMin = (int)((DecaZ + zPolyTMin) * SurDZ);
                    nxMax = (int)((DecaX + xPolyTMax) * SurDX);
                    nyMax = (int)((DecaY + yPolyTMax) * SurDY);
                    nzMax = (int)((DecaZ + zPolyTMax) * SurDZ);
                    PolyTIndices.Min = nyMin + (nxMin << 11);
                    PolyTIndices.Min <<= 10;
                    PolyTIndices.Min += nzMin - 0x00000200;
                    PolyTIndices.Max = nyMax + (nxMax << 11);
                    PolyTIndices.Max <<= 10;
                    PolyTIndices.Max += nzMax;
                    if (xShellMin > xPolyTMin)
                    {
                        xShellMin = xPolyTMin;
                    }
                    if (xShellMax < xPolyTMax)
                    {
                        xShellMax = xPolyTMax;
                    }
                    if (yShellMin > yPolyTMin)
                    {
                        yShellMin = yPolyTMin;
                    }
                    if (yShellMax < yPolyTMax)
                    {
                        yShellMax = yPolyTMax;
                    }
                    if (zShellMin > zPolyTMin)
                    {
                        zShellMin = zPolyTMin;
                    }
                    if (zShellMax < zPolyTMax)
                    {
                        zShellMax = zPolyTMax;
                    }
                }
            }
            if (nbFaHi > 0)
            {
                nxMin = (int)((DecaX + xShellMin) * SurDX);
                nyMin = (int)((DecaY + yShellMin) * SurDY);
                nzMin = (int)((DecaZ + zShellMin) * SurDZ);
                nxMax = (int)((DecaX + xShellMax) * SurDX);
                nyMax = (int)((DecaY + yShellMax) * SurDY);
                nzMax = (int)((DecaZ + zShellMax) * SurDZ);
                aShellIndices.Min = nyMin + (nxMin << 11);
                aShellIndices.Min <<= 10;
                aShellIndices.Min += nzMin - 0x00000200;
                aShellIndices.Max = nyMax + (nxMax << 11);
                aShellIndices.Max <<= 10;
                aShellIndices.Max += nzMax;
                aPsd.UpdateHiding(nbFaHi);
                int aHiddenIndex = 1;
                for (j = 1; j <= nbFace; j++)
                {
                    HLRAlgo_PolyData aPd = aPolyg[j]!;
                    if (aPd.Hiding())
                    {
                        aPsd.HidingPolyData().SetValue(aHiddenIndex++, aPd);
                    }
                }
            }
            else
            {
                aPsd.UpdateHiding(0);
                aShellIndices.Min = 0;
                aShellIndices.Max = 0;
            }
        }
    }

    public void InitHide()
    {
        myCurShell = 0;
        NextHide();
    }

    public bool MoreHide() => myFound;

    public void NextHide()
    {
        myFound = false;
        if (myCurShell != 0)
        {
            mySegListIt.Next();
            if (mySegListIt.More())
            {
                myFound = true;
            }
        }

        if (!myFound)
        {
            myCurShell++;

            while (myCurShell <= myNbrShell && !myFound)
            {
                HLRAlgo_PolyShellData aData = myHShell[myCurShell]!;
                mySegListIt.Initialize(aData.Edges());
                if (mySegListIt.More())
                {
                    myFound = true;
                }
                else
                {
                    myCurShell++;
                }
            }
        }
    }

    /// <summary>
    /// Hides the current segment by the hiding shells: sets <paramref name="theStatus"/> (OCCT assigns a new status)
    /// and the segment's index and flags; returns a reference to its points.
    /// </summary>
    public ref HLRAlgo_BiPoint.PointsT Hide(HLRAlgo_EdgeStatus theStatus, out int theIndex, out bool theReg1,
                                            out bool theRegn, out bool theOutl, out bool theIntl)
    {
        ref HLRAlgo_BiPoint aBP = ref mySegListIt.ChangeValue();
        ref HLRAlgo_BiPoint.PointsT aPoints = ref aBP.Points();
        ref HLRAlgo_BiPoint.IndicesT anIndices = ref aBP.Indices();
        theStatus.Assign(new HLRAlgo_EdgeStatus(0.0, (float)myTriangle.TolParam, 1.0, (float)myTriangle.TolParam));
        theIndex = anIndices.ShapeIndex;
        theReg1 = aBP.Rg1Line();
        theRegn = aBP.RgNLine();
        theOutl = aBP.OutLine();
        theIntl = aBP.IntLine();
        if (aBP.Hidden())
        {
            theStatus.HideAll();
            return ref aPoints;
        }

        for (int s = 1; s <= myNbrShell; s++)
        {
            HLRAlgo_PolyShellData aPsd = myHShell[s]!;
            if (!aPsd.Hiding())
            {
                continue;
            }

            ref HLRAlgo_PolyShellData.ShellIndices aShellIndices = ref aPsd.Indices();
            if (((aShellIndices.Max - anIndices.MinSeg) & 0x80100200) == 0
                && ((anIndices.MaxSeg - aShellIndices.Min) & 0x80100000) == 0)
            {
                bool isHidingShell = (s == myCurShell);
                NCollection_Array1<HLRAlgo_PolyData?> aFace = aPsd.HidingPolyData();
                int nbFace = aFace.Upper();
                for (int f = 1; f <= nbFace; f++)
                {
                    HLRAlgo_PolyData aPd = aFace[f]!;
                    aPd.HideByPolyData(aPoints, ref myTriangle, ref anIndices, isHidingShell, theStatus);
                }
            }
        }
        return ref aPoints;
    }

    public void InitShow()
    {
        myCurShell = 0;
        NextShow();
    }

    public bool MoreShow() => myFound;

    public void NextShow()
    {
        myFound = false;
        if (myCurShell != 0)
        {
            mySegListIt.Next();
            if (mySegListIt.More())
            {
                myFound = true;
            }
        }
        if (!myFound)
        {
            myCurShell++;

            while (myCurShell <= myNbrShell && !myFound)
            {
                mySegListIt.Initialize(myHShell[myCurShell]!.Edges());
                if (mySegListIt.More())
                {
                    myFound = true;
                }
                else
                {
                    myCurShell++;
                }
            }
        }
    }

    /// <summary>The current segment's index and flags; returns a reference to its points.</summary>
    public ref HLRAlgo_BiPoint.PointsT Show(out int Index, out bool reg1, out bool regn, out bool outl, out bool intl)
    {
        ref HLRAlgo_BiPoint BP = ref mySegListIt.ChangeValue();
        ref HLRAlgo_BiPoint.IndicesT theIndices = ref BP.Indices();
        ref HLRAlgo_BiPoint.PointsT aPoints = ref BP.Points();
        Index = theIndices.ShapeIndex;
        reg1 = BP.Rg1Line();
        regn = BP.RgNLine();
        outl = BP.OutLine();
        intl = BP.IntLine();
        return ref aPoints;
    }
}
