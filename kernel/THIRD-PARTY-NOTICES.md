# Third-party notices

OCCT.NET is licensed under the LGPL 2.1 with the Open CASCADE exception (see [LICENSE](LICENSE) and [NOTICE](NOTICE)).
It also contains ports of third-party code that OCCT itself includes under other licences. Each keeps its licence,
and its notice is reproduced here and at the top of the files that port it.

## Delabella

- **What:** DELABELLA, a Delaunay triangulation library, included in OCCT as
  `src/ModelingAlgorithms/TKMesh/BRepMesh/delabella.cpp` and `delabella.pxx` (used by
  `BRepMesh_DelabellaBaseMeshAlgo`).
- **Ported in:** `src/OcctNet.ModelingAlgorithms/BRepMesh/delabella.cs`.
- **Licence:** MIT.

```
MIT License

DELABELLA - Delaunay triangulation library
Copyright (C) 2018 GUMIX - Marcin Sokalski

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
