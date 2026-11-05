// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyAlgo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Occt;
using static Kerf.Engine.HiddenLines.Algorithm.PolyMask;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// The polygonal hidden-line removal on triangulations: indexes the shells' segments and hiding triangles in a
/// coarse grid of the projected box, then hides each segment by the triangles in front of it.
/// </summary>
/// <remarks>
/// A working object (a transient), as OCCT's. <see cref="Hide"/> and <see cref="Show"/> return a reference to the
/// current segment's points, as OCCT's.
/// </remarks>
internal class AlgoPolyAlgo : Transient
{
    private NCollection_Array1<PolyShellData?> _hShell = new();
    private PolyData.Triangle _triangle;
    private readonly NCollection_List<AlgoBiPoint>.Iterator _segListIt = new();
    private int _nbrShell;
    private int _curShell;
    private bool _found;

    public AlgoPolyAlgo()
    {
        _nbrShell = 0;
        _curShell = 0;
        _found = false;
        _triangle.TolParam = 0.00000001;
        _triangle.TolAng = 0.0001;
    }

    /// <summary>Sizes the shells array to <paramref name="nbShells"/>.</summary>
    public void Init(int nbShells)
    {
        _hShell.Resize(1, nbShells, false);
    }

    public NCollection_Array1<PolyShellData?> PolyShell() => _hShell;

    public NCollection_Array1<PolyShellData?> ChangePolyShell() => _hShell;

    public void Clear()
    {
        _hShell = new NCollection_Array1<PolyShellData?>();
        _nbrShell = 0;
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
        double big = global::Kerf.Engine.Foundation.Precision.Infinite();
        Box box = new();

        _nbrShell = _hShell.Length();
        for (int shellIter = _hShell.Lower(); shellIter <= _hShell.Upper(); ++shellIter)
        {
            PolyShellData psd = _hShell[shellIter]!;
            psd.UpdateGlobalMinMax(ref box);
        }

        (double xMin, double xMax, double yMin, double yMax, double zMin, double zMax) = box.Get();
        double dx = xMax - xMin;
        double dy = yMax - yMin;
        double dz = zMax - zMin;
        double precad = dx;
        if (precad < dy)
        {
            precad = dy;
        }
        if (precad < dz)
        {
            precad = dz;
        }
        _triangle.Tolerance = precad * _triangle.TolParam;
        precad = precad * 0.01;
        double surDX = 1020 / (dx + precad);
        double surDY = 1020 / (dy + precad);
        double surDZ = 508 / (dz + precad);
        precad = precad * 0.5;
        double decaX = -xMin + precad;
        double decaY = -yMin + precad;
        double decaZ = -zMin + precad;

        for (int shellIter = _hShell.Lower(); shellIter <= _hShell.Upper(); ++shellIter)
        {
            PolyShellData psd = _hShell[shellIter]!;
            ref PolyShellData.ShellIndices shellIndices = ref psd.Indices();
            xShellMin = big;
            yShellMin = big;
            zShellMin = big;
            xShellMax = -big;
            yShellMax = -big;
            zShellMax = -big;

            for (_segListIt.Initialize(psd.Edges()); _segListIt.More(); _segListIt.Next())
            {
                ref AlgoBiPoint bp = ref _segListIt.ChangeValue();
                ref AlgoBiPoint.PointsT points = ref bp.Points();
                ref AlgoBiPoint.IndicesT theIndices = ref bp.Indices();
                if (points.PntP1.X() < points.PntP2.X())
                {
                    xSegmnMin = points.PntP1.X();
                    xSegmnMax = points.PntP2.X();
                }
                else
                {
                    xSegmnMin = points.PntP2.X();
                    xSegmnMax = points.PntP1.X();
                }
                if (points.PntP1.Y() < points.PntP2.Y())
                {
                    ySegmnMin = points.PntP1.Y();
                    ySegmnMax = points.PntP2.Y();
                }
                else
                {
                    ySegmnMin = points.PntP2.Y();
                    ySegmnMax = points.PntP1.Y();
                }
                if (points.PntP1.Z() < points.PntP2.Z())
                {
                    zSegmnMin = points.PntP1.Z();
                    zSegmnMax = points.PntP2.Z();
                }
                else
                {
                    zSegmnMin = points.PntP2.Z();
                    zSegmnMax = points.PntP1.Z();
                }
                nxMin = (int)((decaX + xSegmnMin) * surDX);
                nyMin = (int)((decaY + ySegmnMin) * surDY);
                nzMin = (int)((decaZ + zSegmnMin) * surDZ);
                nxMax = (int)((decaX + xSegmnMax) * surDX);
                nyMax = (int)((decaY + ySegmnMax) * surDY);
                nzMax = (int)((decaZ + zSegmnMax) * surDZ);
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
            NCollection_Array1<PolyData?> polyg = psd.PolyData();
            int nbFace = polyg.Upper();
            int nbFaHi = 0;
            for (j = 1; j <= nbFace; j++)
            {
                PolyData pd = polyg[j]!;
                if (pd.Hiding())
                {
                    nbFaHi++;
                    xPolyTMin = big;
                    yPolyTMin = big;
                    zPolyTMin = big;
                    xPolyTMax = -big;
                    yPolyTMax = -big;
                    zPolyTMax = -big;
                    int otheri, nbHide = 0; // min,max;
                    double x1, x2, x3, y1, y2, y3, z1, z2, z3;
                    double dn, dnx, dny, dnz, dx1, dy1, dz1, dx2, dy2, dz2, dx3, dy3;
                    double adx1, ady1, adx2, ady2, adx3, ady3;
                    double a = 0.0, b = 0.0, c = 0.0, d = 0.0;
                    ref PolyData.FaceIndices polyTIndices = ref pd.Indices();
                    NCollection_Array1<Xyz> nodes = pd.Nodes();
                    NCollection_Array1<TriangleData> tData = pd.TData();
                    NCollection_Array1<PolyHidingData> phDat = pd.PHDat();
                    int nbT = tData.Upper();

                    for (otheri = 1; otheri <= nbT; otheri++)
                    {
                        ref TriangleData td = ref tData[otheri];
                        if ((td.Flags & HLRAlgo_PolyMask_FMskHiding) != 0)
                        {
                            Xyz p1 = nodes.Value(td.Node1);
                            Xyz p2 = nodes.Value(td.Node2);
                            Xyz p3 = nodes.Value(td.Node3);
                            x1 = p1.X();
                            y1 = p1.Y();
                            z1 = p1.Z();
                            x2 = p2.X();
                            y2 = p2.Y();
                            z2 = p2.Z();
                            x3 = p3.X();
                            y3 = p3.Y();
                            z3 = p3.Z();
                            xTrianMax = xTrianMin = x1;
                            yTrianMax = yTrianMin = y1;
                            zTrianMax = zTrianMin = z1;
                            if (xTrianMin > x2)
                            {
                                xTrianMin = x2;
                            }
                            else if (xTrianMax < x2)
                            {
                                xTrianMax = x2;
                            }
                            if (yTrianMin > y2)
                            {
                                yTrianMin = y2;
                            }
                            else if (yTrianMax < y2)
                            {
                                yTrianMax = y2;
                            }
                            if (zTrianMin > z2)
                            {
                                zTrianMin = z2;
                            }
                            else if (zTrianMax < z2)
                            {
                                zTrianMax = z2;
                            }
                            if (xTrianMin > x3)
                            {
                                xTrianMin = x3;
                            }
                            else if (xTrianMax < x3)
                            {
                                xTrianMax = x3;
                            }
                            if (yTrianMin > y3)
                            {
                                yTrianMin = y3;
                            }
                            else if (yTrianMax < y3)
                            {
                                yTrianMax = y3;
                            }
                            if (zTrianMin > z3)
                            {
                                zTrianMin = z3;
                            }
                            else if (zTrianMax < z3)
                            {
                                zTrianMax = z3;
                            }
                            nxMin = (int)((decaX + xTrianMin) * surDX);
                            nyMin = (int)((decaY + yTrianMin) * surDY);
                            nzMin = (int)((decaZ + zTrianMin) * surDZ);
                            nxMax = (int)((decaX + xTrianMax) * surDX);
                            nyMax = (int)((decaY + yTrianMax) * surDY);
                            nzMax = (int)((decaZ + zTrianMax) * surDZ);
                            int minTrian, maxTrian;
                            minTrian = nyMin + (nxMin << 11);
                            minTrian <<= 10;
                            minTrian += nzMin - 0x00000200;
                            maxTrian = nyMax + (nxMax << 11);
                            maxTrian <<= 10;
                            maxTrian += nzMax;
                            dx1 = x2 - x1;
                            dy1 = y2 - y1;
                            dz1 = z2 - z1;
                            dx2 = x3 - x2;
                            dy2 = y3 - y2;
                            dz2 = z3 - z2;
                            dx3 = x1 - x3;
                            dy3 = y1 - y3;
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
                            d = a * x1 + b * y1 + c * z1;
                            nbHide++;
                            phDat[nbHide].Set(otheri, minTrian, maxTrian, a, b, c, d);
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
                                td.Flags |= HLRAlgo_PolyMask_EMskGrALin1;
                            }
                            else
                            {
                                td.Flags &= ~HLRAlgo_PolyMask_EMskGrALin1;
                            }
                            if (adx2 > ady2)
                            {
                                td.Flags |= HLRAlgo_PolyMask_EMskGrALin2;
                            }
                            else
                            {
                                td.Flags &= ~HLRAlgo_PolyMask_EMskGrALin2;
                            }
                            if (adx3 > ady3)
                            {
                                td.Flags |= HLRAlgo_PolyMask_EMskGrALin3;
                            }
                            else
                            {
                                td.Flags &= ~HLRAlgo_PolyMask_EMskGrALin3;
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
                    nxMin = (int)((decaX + xPolyTMin) * surDX);
                    nyMin = (int)((decaY + yPolyTMin) * surDY);
                    nzMin = (int)((decaZ + zPolyTMin) * surDZ);
                    nxMax = (int)((decaX + xPolyTMax) * surDX);
                    nyMax = (int)((decaY + yPolyTMax) * surDY);
                    nzMax = (int)((decaZ + zPolyTMax) * surDZ);
                    polyTIndices.Min = nyMin + (nxMin << 11);
                    polyTIndices.Min <<= 10;
                    polyTIndices.Min += nzMin - 0x00000200;
                    polyTIndices.Max = nyMax + (nxMax << 11);
                    polyTIndices.Max <<= 10;
                    polyTIndices.Max += nzMax;
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
                nxMin = (int)((decaX + xShellMin) * surDX);
                nyMin = (int)((decaY + yShellMin) * surDY);
                nzMin = (int)((decaZ + zShellMin) * surDZ);
                nxMax = (int)((decaX + xShellMax) * surDX);
                nyMax = (int)((decaY + yShellMax) * surDY);
                nzMax = (int)((decaZ + zShellMax) * surDZ);
                shellIndices.Min = nyMin + (nxMin << 11);
                shellIndices.Min <<= 10;
                shellIndices.Min += nzMin - 0x00000200;
                shellIndices.Max = nyMax + (nxMax << 11);
                shellIndices.Max <<= 10;
                shellIndices.Max += nzMax;
                psd.UpdateHiding(nbFaHi);
                int hiddenIndex = 1;
                for (j = 1; j <= nbFace; j++)
                {
                    PolyData pd = polyg[j]!;
                    if (pd.Hiding())
                    {
                        psd.HidingPolyData().SetValue(hiddenIndex++, pd);
                    }
                }
            }
            else
            {
                psd.UpdateHiding(0);
                shellIndices.Min = 0;
                shellIndices.Max = 0;
            }
        }
    }

    public void InitHide()
    {
        _curShell = 0;
        NextHide();
    }

    public bool MoreHide() => _found;

    public void NextHide()
    {
        _found = false;
        if (_curShell != 0)
        {
            _segListIt.Next();
            if (_segListIt.More())
            {
                _found = true;
            }
        }

        if (!_found)
        {
            _curShell++;

            while (_curShell <= _nbrShell && !_found)
            {
                PolyShellData data = _hShell[_curShell]!;
                _segListIt.Initialize(data.Edges());
                if (_segListIt.More())
                {
                    _found = true;
                }
                else
                {
                    _curShell++;
                }
            }
        }
    }

    /// <summary>
    /// Hides the current segment by the hiding shells: sets <paramref name="status"/> (OCCT assigns a new status)
    /// and the segment's index and flags; returns a reference to its points.
    /// </summary>
    public ref AlgoBiPoint.PointsT Hide(EdgeStatus status, out int index, out bool reg1,
                                            out bool regn, out bool outl, out bool intl)
    {
        ref AlgoBiPoint bp = ref _segListIt.ChangeValue();
        ref AlgoBiPoint.PointsT points = ref bp.Points();
        ref AlgoBiPoint.IndicesT indices = ref bp.Indices();
        status.Assign(new EdgeStatus(0.0, (float)_triangle.TolParam, 1.0, (float)_triangle.TolParam));
        index = indices.ShapeIndex;
        reg1 = bp.Rg1Line();
        regn = bp.RgNLine();
        outl = bp.OutLine();
        intl = bp.IntLine();
        if (bp.Hidden())
        {
            status.HideAll();
            return ref points;
        }

        for (int s = 1; s <= _nbrShell; s++)
        {
            PolyShellData psd = _hShell[s]!;
            if (!psd.Hiding())
            {
                continue;
            }

            ref PolyShellData.ShellIndices shellIndices = ref psd.Indices();
            if (((shellIndices.Max - indices.MinSeg) & 0x80100200) == 0
                && ((indices.MaxSeg - shellIndices.Min) & 0x80100000) == 0)
            {
                bool isHidingShell = (s == _curShell);
                NCollection_Array1<PolyData?> face = psd.HidingPolyData();
                int nbFace = face.Upper();
                for (int f = 1; f <= nbFace; f++)
                {
                    PolyData pd = face[f]!;
                    pd.HideByPolyData(points, ref _triangle, ref indices, isHidingShell, status);
                }
            }
        }
        return ref points;
    }

    public void InitShow()
    {
        _curShell = 0;
        NextShow();
    }

    public bool MoreShow() => _found;

    public void NextShow()
    {
        _found = false;
        if (_curShell != 0)
        {
            _segListIt.Next();
            if (_segListIt.More())
            {
                _found = true;
            }
        }
        if (!_found)
        {
            _curShell++;

            while (_curShell <= _nbrShell && !_found)
            {
                _segListIt.Initialize(_hShell[_curShell]!.Edges());
                if (_segListIt.More())
                {
                    _found = true;
                }
                else
                {
                    _curShell++;
                }
            }
        }
    }

    /// <summary>The current segment's index and flags; returns a reference to its points.</summary>
    public ref AlgoBiPoint.PointsT Show(out int index, out bool reg1, out bool regn, out bool outl, out bool intl)
    {
        ref AlgoBiPoint bp = ref _segListIt.ChangeValue();
        ref AlgoBiPoint.IndicesT theIndices = ref bp.Indices();
        ref AlgoBiPoint.PointsT points = ref bp.Points();
        index = theIndices.ShapeIndex;
        reg1 = bp.Rg1Line();
        regn = bp.RgNLine();
        outl = bp.OutLine();
        intl = bp.IntLine();
        return ref points;
    }
}
